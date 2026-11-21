//! `agcad-view` — the dumb renderer.
//!
//! Philosophy (Lite XL style, scaled up to CAD): this process owns **no
//! document state**. It pulls a scene snapshot from the server over the
//! command bus, uploads it to the GPU, draws it. A second connection listens
//! for `scene_changed` and triggers a re-fetch, so live preview is automatic;
//! `R` forces a manual refresh.
//!
//! Controls: drag = orbit, wheel = zoom, `R` = manual refresh. A corner
//! triad shows the world axes and follows the camera.

mod cam;
mod net;



use std::io::{BufRead, BufReader};
use std::net::TcpStream;
use std::sync::Arc;
use std::sync::mpsc;

use agcad_protocol::{EntitySnapshot, Mat4};
use winit::application::ApplicationHandler;
use winit::event::{ElementState, MouseScrollDelta, WindowEvent};
use winit::event_loop::{ActiveEventLoop, EventLoop};
use winit::keyboard::{KeyCode, PhysicalKey};
use winit::window::{Window, WindowId};

/// Corner axis-gizmo size and margin, in physical pixels.
const GIZMO_SIZE: f32 = 110.0;
const GIZMO_MARGIN: f32 = 16.0;

const WGSL: &str = r#"
struct Uniforms {
    view_proj: mat4x4<f32>,
};
@group(0) @binding(0) var<uniform> u: Uniforms;

struct Instance {
    model: mat4x4<f32>,
    color: vec4<f32>,
};
@group(1) @binding(0) var<storage, read> instances: array<Instance>;

struct VSOut {
    @builtin(position) pos: vec4<f32>,
    @location(0) normal: vec3<f32>,
    @location(1) color: vec4<f32>,
};

@vertex
fn vs(@location(0) pos: vec3<f32>, @location(1) normal: vec3<f32>, @builtin(instance_index) ii: u32) -> VSOut {
    let inst = instances[ii];
    var out: VSOut;
    out.pos = u.view_proj * inst.model * vec4<f32>(pos, 1.0);
    out.normal = (inst.model * vec4<f32>(normal, 0.0)).xyz;
    out.color = inst.color;
    return out;
}

@fragment
fn fs(in: VSOut) -> @location(0) vec4<f32> {
    let n = normalize(in.normal);
    // Two key lights plus ambient, so every face reads distinctly.
    let l1 = normalize(vec3<f32>(0.5, -0.7, 0.6));
    let l2 = normalize(vec3<f32>(-0.6, 0.5, 0.4));
    let diff = max(dot(n, l1), 0.0) * 0.55 + max(dot(n, l2), 0.0) * 0.25 + 0.28;
    return vec4<f32>(in.color.rgb * diff, in.color.a);
}
"#;

/// One GPU draw call description. Entities are indexed triangles, the grid
/// and the axis gizmo are non-indexed line lists; all live in the same
/// buffers. Overlay draws ignore depth so they sit on top of the scene.
struct DrawRange {
    line: bool,
    overlay: bool,
    /// First index for triangles, first vertex for lines.
    first_index: u32,
    count: u32,
    base_instance: u32,
}

/// CPU-side scene buffers, ready for upload.
struct SceneData {
    verts: Vec<f32>,
    idx: Vec<u32>,
    inst: Vec<f32>,
    draws: Vec<DrawRange>,
}

struct SceneGpu {
    vertex_buf: wgpu::Buffer,
    index_buf: wgpu::Buffer,
    draws: Vec<DrawRange>,
}

struct State {
    device: wgpu::Device,
    queue: wgpu::Queue,
    surface: wgpu::Surface<'static>,
    surface_format: wgpu::TextureFormat,
    pipeline_tri: wgpu::RenderPipeline,
    pipeline_line: wgpu::RenderPipeline,
    /// Line pipeline with depth test disabled, for the corner axis gizmo.
    pipeline_line_overlay: wgpu::RenderPipeline,
    bind_group_0: wgpu::BindGroup,
    bind_group_1: wgpu::BindGroup,
    uniform_buf: wgpu::Buffer,
    gizmo_uniform_buf: wgpu::Buffer,
    gizmo_bind_group: wgpu::BindGroup,
    depth_tex: wgpu::Texture,
    depth_view: wgpu::TextureView,
    size: (u32, u32),
    scene: Option<SceneGpu>,
    camera: OrbitCamera,
}

impl State {
    fn new(window: &Arc<Window>) -> Result<Self, Box<dyn std::error::Error>> {
        let instance = wgpu::Instance::new(wgpu::InstanceDescriptor::new_without_display_handle());
        let surface = instance.create_surface(window.clone())?;
        let adapter = pollster::block_on(instance.request_adapter(&wgpu::RequestAdapterOptions {
            power_preference: wgpu::PowerPreference::HighPerformance,
            compatible_surface: Some(&surface),
            force_fallback_adapter: false,
            apply_limit_buckets: false,
        }))
        .map_err(|e| format!("no suitable GPU adapter: {e}"))?;
        let (device, queue) =
            pollster::block_on(adapter.request_device(&wgpu::DeviceDescriptor {
                label: Some("agcad-viewer"),
                required_features: wgpu::Features::empty(),
                required_limits: wgpu::Limits::default(),
                experimental_features: wgpu::ExperimentalFeatures::default(),
                memory_hints: wgpu::MemoryHints::default(),
                trace: wgpu::Trace::Off,
            }))?;

        let caps = surface.get_capabilities(&adapter);
        let surface_format = caps
            .formats
            .iter()
            .find(|f| matches!(f, wgpu::TextureFormat::Rgba8Unorm | wgpu::TextureFormat::Bgra8Unorm))
            .copied()
            .unwrap_or(caps.formats[0]);
        let alpha_mode = if caps.alpha_modes.contains(&wgpu::CompositeAlphaMode::Opaque) {
            wgpu::CompositeAlphaMode::Opaque
        } else {
            caps.alpha_modes[0]
        };

        let size = window.inner_size();
        let width = size.width.max(1);
        let height = size.height.max(1);
        surface.configure(
            &device,
            &wgpu::SurfaceConfiguration {
                usage: wgpu::TextureUsages::RENDER_ATTACHMENT,
                format: surface_format,
                color_space: wgpu::SurfaceColorSpace::Auto,
                width,
                height,
                present_mode: wgpu::PresentMode::Fifo,
                alpha_mode,
                view_formats: vec![],
                desired_maximum_frame_latency: 2,
            },
        );

        let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("agcad shader"),
            source: wgpu::ShaderSource::Wgsl(WGSL.into()),
        });

        let uniform_buf = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("view_proj"),
            size: 64,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let gizmo_uniform_buf = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("gizmo_view_proj"),
            size: 64,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });

        let bgl0 = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("uniforms"),
            entries: &[wgpu::BindGroupLayoutEntry {
                binding: 0,
                visibility: wgpu::ShaderStages::VERTEX,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Uniform,
                    has_dynamic_offset: false,
                    min_binding_size: wgpu::BufferSize::new(64),
                },
                count: None,
            }],
        });
        let bgl1 = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("instances"),
            entries: &[wgpu::BindGroupLayoutEntry {
                binding: 0,
                visibility: wgpu::ShaderStages::VERTEX,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Storage { read_only: true },
                    has_dynamic_offset: false,
                    min_binding_size: wgpu::BufferSize::new(80),
                },
                count: None,
            }],
        });

        let bind_group_0 = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("bg uniforms"),
            layout: &bgl0,
            entries: &[wgpu::BindGroupEntry {
                binding: 0,
                resource: uniform_buf.as_entire_binding(),
            }],
        });
        let gizmo_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("bg gizmo uniforms"),
            layout: &bgl0,
            entries: &[wgpu::BindGroupEntry {
                binding: 0,
                resource: gizmo_uniform_buf.as_entire_binding(),
            }],
        });
        // Placeholder storage binding; replaced on every scene upload.
        let placeholder = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("instance placeholder"),
            size: 80,
            usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let bind_group_1 = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("bg instances"),
            layout: &bgl1,
            entries: &[wgpu::BindGroupEntry {
                binding: 0,
                resource: placeholder.as_entire_binding(),
            }],
        });

        let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("agcad layout"),
            bind_group_layouts: &[Some(&bgl0), Some(&bgl1)],
            immediate_size: 0,
        });

        let vertex_buffers = [Some(wgpu::VertexBufferLayout {
            array_stride: 24,
            step_mode: wgpu::VertexStepMode::Vertex,
            attributes: &[
                wgpu::VertexAttribute {
                    format: wgpu::VertexFormat::Float32x3,
                    offset: 0,
                    shader_location: 0,
                },
                wgpu::VertexAttribute {
                    format: wgpu::VertexFormat::Float32x3,
                    offset: 12,
                    shader_location: 1,
                },
            ],
        })];

        let make_pipeline = |topology: wgpu::PrimitiveTopology, overlay: bool| {
            let depth_stencil = wgpu::DepthStencilState {
                format: wgpu::TextureFormat::Depth24Plus,
                depth_write_enabled: Some(!overlay),
                depth_compare: Some(if overlay {
                    wgpu::CompareFunction::Always
                } else {
                    wgpu::CompareFunction::Less
                }),
                stencil: wgpu::StencilState::default(),
                bias: wgpu::DepthBiasState::default(),
            };
            device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
                label: Some(if overlay {
                    "agcad overlay lines"
                } else if topology == wgpu::PrimitiveTopology::TriangleList {
                    "agcad tri"
                } else {
                    "agcad lines"
                }),
                layout: Some(&pipeline_layout),
                vertex: wgpu::VertexState {
                    module: &shader,
                    entry_point: Some("vs"),
                    buffers: &vertex_buffers,
                    compilation_options: Default::default(),
                },
                fragment: Some(wgpu::FragmentState {
                    module: &shader,
                    entry_point: Some("fs"),
                    targets: &[Some(wgpu::ColorTargetState {
                        format: surface_format,
                        blend: Some(wgpu::BlendState::ALPHA_BLENDING),
                        write_mask: wgpu::ColorWrites::ALL,
                    })],
                    compilation_options: Default::default(),
                }),
                primitive: wgpu::PrimitiveState {
                    topology,
                    cull_mode: None, // winding is not guaranteed in v0 tessellation
                    ..Default::default()
                },
                depth_stencil: Some(depth_stencil),
                multisample: wgpu::MultisampleState::default(),
                multiview_mask: None,
                cache: None,
            })
        };

        let pipeline_tri = make_pipeline(wgpu::PrimitiveTopology::TriangleList, false);
        let pipeline_line = make_pipeline(wgpu::PrimitiveTopology::LineList, false);
        let pipeline_line_overlay = make_pipeline(wgpu::PrimitiveTopology::LineList, true);

        let depth_tex = device.create_texture(&wgpu::TextureDescriptor {
            label: Some("depth"),
            size: wgpu::Extent3d { width, height, depth_or_array_layers: 1 },
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: wgpu::TextureFormat::Depth24Plus,
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT,
            view_formats: &[],
        });
        let depth_view = depth_tex.create_view(&wgpu::TextureViewDescriptor::default());

        Ok(Self {
            device,
            queue,
            surface,
            surface_format,
            pipeline_tri,
            pipeline_line,
            pipeline_line_overlay,
            bind_group_0,
            bind_group_1,
            uniform_buf,
            gizmo_uniform_buf,
            gizmo_bind_group,
            depth_tex,
            depth_view,
            size: (width, height),
            scene: None,
            camera: OrbitCamera::new(),
        })
    }

    fn resize(&mut self, width: u32, height: u32) {
        let width = width.max(1);
        let height = height.max(1);
        self.size = (width, height);
        self.surface.configure(
            &self.device,
            &wgpu::SurfaceConfiguration {
                usage: wgpu::TextureUsages::RENDER_ATTACHMENT,
                format: self.surface_format,
                color_space: wgpu::SurfaceColorSpace::Auto,
                width,
                height,
                present_mode: wgpu::PresentMode::Fifo,
                alpha_mode: wgpu::CompositeAlphaMode::Opaque,
                view_formats: vec![],
                desired_maximum_frame_latency: 2,
            },
        );
        self.depth_tex = self.device.create_texture(&wgpu::TextureDescriptor {
            label: Some("depth"),
            size: wgpu::Extent3d { width, height, depth_or_array_layers: 1 },
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: wgpu::TextureFormat::Depth24Plus,
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT,
            view_formats: &[],
        });
        self.depth_view = self.depth_tex.create_view(&wgpu::TextureViewDescriptor::default());
    }

    /// Upload a fresh snapshot: entities first, then the reference grid.
    fn set_scene(&mut self, entities: Vec<EntitySnapshot>) {
        let SceneData { verts, idx, inst, draws } = build_scene(&entities);

        let vertex_buf = self.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("vertices"),
            size: ((verts.len() * 4) as u64).max(4),
            usage: wgpu::BufferUsages::VERTEX | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        self.queue.write_buffer(&vertex_buf, 0, bytemuck::cast_slice(&verts));

        let index_buf = self.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("indices"),
            size: ((idx.len() * 4) as u64).max(4),
            usage: wgpu::BufferUsages::INDEX | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        self.queue.write_buffer(&index_buf, 0, bytemuck::cast_slice(&idx));

        let instance_buf = self.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("instances"),
            size: ((inst.len() * 4) as u64).max(4),
            usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        self.queue.write_buffer(&instance_buf, 0, bytemuck::cast_slice(&inst));

        // Rebind the storage buffer to the new instance data.
        self.bind_group_1 = self.device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("bg instances"),
            layout: &self.pipeline_tri.get_bind_group_layout(1),
            entries: &[wgpu::BindGroupEntry {
                binding: 0,
                resource: instance_buf.as_entire_binding(),
            }],
        });

        self.scene = Some(SceneGpu { vertex_buf, index_buf, draws });
    }

    fn render(&mut self) {
        let vp = self.camera.view_proj(self.size);
        self.queue.write_buffer(&self.uniform_buf, 0, bytemuck::cast_slice(&vp));
        let gizmo_vp = self.camera.gizmo_view_proj();
        self.queue
            .write_buffer(&self.gizmo_uniform_buf, 0, bytemuck::cast_slice(&gizmo_vp));

        let frame = match self.surface.get_current_texture() {
            wgpu::CurrentSurfaceTexture::Success(frame)
            | wgpu::CurrentSurfaceTexture::Suboptimal(frame) => frame,
            wgpu::CurrentSurfaceTexture::Lost | wgpu::CurrentSurfaceTexture::Outdated => {
                self.resize(self.size.0, self.size.1);
                return;
            },
            _ => return,
        };
        let view = frame.texture.create_view(&wgpu::TextureViewDescriptor::default());
        let mut encoder = self
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor { label: Some("frame") });

        {
            let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("main pass"),
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view: &view,
                    depth_slice: None,
                    resolve_target: None,
                    ops: wgpu::Operations {
                        load: wgpu::LoadOp::Clear(wgpu::Color {
                            r: 0.10,
                            g: 0.11,
                            b: 0.13,
                            a: 1.0,
                        }),
                        store: wgpu::StoreOp::Store,
                    },
                })],
                depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                    view: &self.depth_view,
                    depth_ops: Some(wgpu::Operations {
                        load: wgpu::LoadOp::Clear(1.0),
                        store: wgpu::StoreOp::Store,
                    }),
                    stencil_ops: None,
                }),
                timestamp_writes: None,
                occlusion_query_set: None,
                multiview_mask: None,
            });

            if let Some(scene) = &self.scene {
                pass.set_bind_group(0, &self.bind_group_0, &[]);
                pass.set_bind_group(1, &self.bind_group_1, &[]);
                // Draw the grid first, entities on top.
                if let Some(grid) = scene.draws.iter().find(|d| d.line && !d.overlay) {
                    pass.set_pipeline(&self.pipeline_line);
                    pass.set_vertex_buffer(0, scene.vertex_buf.slice(..));
                    let end = grid.base_instance + 1;
                    pass.draw(
                        grid.first_index..grid.first_index + grid.count,
                        grid.base_instance..end,
                    );
                }
                pass.set_vertex_buffer(0, scene.vertex_buf.slice(..));
                pass.set_index_buffer(scene.index_buf.slice(..), wgpu::IndexFormat::Uint32);
                pass.set_pipeline(&self.pipeline_tri);
                for d in scene.draws.iter().filter(|d| !d.line) {
                    let first = d.base_instance;
                    pass.draw_indexed(
                        d.first_index..d.first_index + d.count,
                        0,
                        first..first + 1,
                    );
                }

                // Axis gizmo: a small triad in the bottom-left corner, drawn
                // with its own projection and depth-always so the scene never
                // hides it.
                let (win_w, win_h) = (self.size.0 as f32, self.size.1 as f32);
                let gizmo_size = GIZMO_SIZE.min(win_w.min(win_h) * 0.5);
                if gizmo_size > 0.0 {
                    let margin = GIZMO_MARGIN.min(win_w.min(win_h) * 0.05);
                    pass.set_viewport(
                        margin,
                        win_h - gizmo_size - margin,
                        gizmo_size,
                        gizmo_size,
                        0.0,
                        1.0,
                    );
                    pass.set_bind_group(0, &self.gizmo_bind_group, &[]);
                    pass.set_pipeline(&self.pipeline_line_overlay);
                    pass.set_vertex_buffer(0, scene.vertex_buf.slice(..));
                    for d in scene.draws.iter().filter(|d| d.overlay) {
                        let end = d.base_instance + 1;
                        pass.draw(
                            d.first_index..d.first_index + d.count,
                            d.base_instance..end,
                        );
                    }
                    pass.set_viewport(0.0, 0.0, win_w, win_h, 0.0, 1.0);
                }
            }
        }

        self.queue.submit([encoder.finish()]);
        self.queue.present(frame);
    }
}

/// Assemble the GPU buffers and draw ranges for a scene snapshot. Pure so the
/// buffer layout can be unit-tested without a GPU.
fn build_scene(entities: &[EntitySnapshot]) -> SceneData {
    let mut verts: Vec<f32> = Vec::new();
    let mut idx: Vec<u32> = Vec::new();
    let mut inst: Vec<f32> = Vec::new();
    let mut draws: Vec<DrawRange> = Vec::new();

    for e in entities {
        let vert_base = (verts.len() / 6) as u32;
        let index_base = idx.len() as u32;
        let instance_base = (inst.len() / 20) as u32;
        for (p, n) in e.mesh.positions.iter().zip(&e.mesh.normals) {
            verts.extend_from_slice(&[p[0], p[1], p[2], n[0], n[1], n[2]]);
        }
        for &i in &e.mesh.indices {
            idx.push(vert_base + i);
        }
        inst.extend_from_slice(&flatten_mat(&e.transform));
        inst.extend_from_slice(&e.color);
        draws.push(DrawRange {
            line: false,
            overlay: false,
            first_index: index_base,
            count: e.mesh.indices.len() as u32,
            base_instance: instance_base,
        });
    }

    // Reference grid: XY plane at z=0 (Z-up ground), 10 mm cells, 2 m span.
    // Appended after the entity vertices, so the draw range must start at
    // `grid_start` rather than zero.
    let grid_base_instance = (inst.len() / 20) as u32;
    let grid_start = verts.len() / 6;
    for k in -10..=10 {
        let p = (k * 10) as f32;
        for line in [[p, -100.0, 0.0], [p, 100.0, 0.0], [-100.0, p, 0.0], [100.0, p, 0.0]] {
            verts.extend_from_slice(&[line[0], line[1], line[2], 0.0, 0.0, 1.0]);
        }
    }
    let grid_vertex_count = (verts.len() / 6 - grid_start) as u32;
    let identity = cam::identity();
    inst.extend_from_slice(&flatten_mat(&identity));
    inst.extend_from_slice(&[0.30, 0.31, 0.34, 1.0]);
    draws.push(DrawRange {
        line: true,
        overlay: false,
        first_index: grid_start as u32,
        count: grid_vertex_count,
        base_instance: grid_base_instance,
    });

    // Corner axis gizmo: three unit-length lines (X red, Y green, Z blue).
    // One instance per axis so each line gets its own colour. The fragment
    // shader still applies its two key lights, so point the line normals at
    // the primary light to keep the overlay bright (line normals are unused
    // otherwise).
    let overlay_normal = cam::normalize([0.5, -0.7, 0.6]);
    for (axis, color) in [
        ([1.0, 0.0, 0.0], [1.00, 0.32, 0.30, 1.0]),
        ([0.0, 1.0, 0.0], [0.35, 1.00, 0.35, 1.0]),
        ([0.0, 0.0, 1.0], [0.40, 0.62, 1.00, 1.0]),
    ] {
        let axis_start = (verts.len() / 6) as u32;
        verts.extend_from_slice(&[0.0, 0.0, 0.0]);
        verts.extend(overlay_normal);
        verts.extend(axis);
        verts.extend(overlay_normal);
        let axis_instance = (inst.len() / 20) as u32;
        inst.extend_from_slice(&flatten_mat(&identity));
        inst.extend_from_slice(&color);
        draws.push(DrawRange {
            line: true,
            overlay: true,
            first_index: axis_start,
            count: 2,
            base_instance: axis_instance,
        });
    }

    SceneData { verts, idx, inst, draws }
}

fn flatten_mat(m: &Mat4) -> [f32; 16] {
    let mut out = [0.0f32; 16];
    for (col, c) in m.iter().enumerate() {
        out[col * 4..col * 4 + 4].copy_from_slice(c);
    }
    out
}

// --- camera -----------------------------------------------------------------

#[derive(Debug, Clone)]
struct OrbitCamera {
    yaw: f32,
    pitch: f32,
    dist: f32,
    target: [f32; 3],
}

impl OrbitCamera {
    fn new() -> Self {
        Self { yaw: -0.7, pitch: 0.5, dist: 250.0, target: [0.0, 0.0, 20.0] }
    }

    fn eye(&self) -> [f32; 3] {
        let (cp, sp) = (self.pitch.cos(), self.pitch.sin());
        let (cy, sy) = (self.yaw.cos(), self.yaw.sin());
        [
            self.target[0] + self.dist * cp * cy,
            self.target[1] + self.dist * cp * sy,
            self.target[2] + self.dist * sp,
        ]
    }

    fn view_proj(&self, size: (u32, u32)) -> [f32; 16] {
        let view = cam::look_at_zup(self.eye(), self.target, [0.0, 0.0, 1.0]);
        let aspect = size.0 as f32 / size.1 as f32;
        let proj = cam::perspective_opengl(50.0, aspect, 1.0, 20_000.0);
        let vp = cam::mul(&cam::to_wgpu_depth(&proj), &view);
        let mut out = [0.0f32; 16];
        for (col, c) in vp.iter().enumerate() {
            out[col * 4..col * 4 + 4].copy_from_slice(c);
        }
        out
    }

    /// View-projection for the corner triad: the main camera's orientation,
    /// but looking at the origin from a fixed distance with an orthographic
    /// projection, so the axes stay a constant size on screen.
    fn gizmo_view_proj(&self) -> [f32; 16] {
        let dir = cam::normalize([
            self.eye()[0] - self.target[0],
            self.eye()[1] - self.target[1],
            self.eye()[2] - self.target[2],
        ]);
        let eye = [dir[0] * 4.0, dir[1] * 4.0, dir[2] * 4.0];
        let view = cam::look_at_zup(eye, [0.0, 0.0, 0.0], [0.0, 0.0, 1.0]);
        let proj = cam::orthographic(1.3, 1.0, 0.1, 10.0);
        flatten_mat(&cam::mul(&cam::to_wgpu_depth(&proj), &view))
    }

    fn orbit(&mut self, dx: f64, dy: f64) {
        self.yaw -= dx as f32 * 0.008;
        self.pitch = (self.pitch + dy as f32 * 0.008).clamp(-1.5, 1.5);
    }

    fn zoom(&mut self, factor: f32) {
        self.dist = (self.dist * factor).clamp(10.0, 10_000.0);
    }
}

// --- app ---------------------------------------------------------------------

struct App {
    host: String,
    port: u16,
    dragging: bool,
    last_cursor: Option<(f64, f64)>,
    window: Option<Arc<Window>>,
    state: Option<State>,
    /// Receives a tick whenever the server broadcasts `scene_changed`.
    reload_rx: Option<mpsc::Receiver<()>>,
}

impl App {
    fn new(host: String, port: u16) -> Self {
        Self {
            host,
            port,
            dragging: false,
            last_cursor: None,
            window: None,
            state: None,
            reload_rx: None,
        }
    }

    /// Persistent connection that only listens for `scene_changed`
    /// notifications; every tick nudges the render loop to re-fetch.
    fn start_live_listener(&mut self) {
        let (tx, rx) = mpsc::channel::<()>();
        self.reload_rx = Some(rx);
        let host = self.host.clone();
        let port = self.port;
        std::thread::spawn(move || {
            let stream = match TcpStream::connect((host.as_str(), port)) {
                Ok(s) => s,
                Err(err) => {
                    eprintln!("live preview off: cannot connect to {host}:{port}: {err} (press R to refresh)");
                    return;
                },
            };
            eprintln!("live preview: watching {host}:{port} for scene_changed");
            for line in BufReader::new(stream).lines() {
                match line {
                    Ok(l) if l.contains("scene_changed") => {
                        let _ = tx.send(());
                    },
                    Ok(_) => {},
                    Err(_) => break,
                }
            }
            eprintln!("live preview: notification connection closed (press R to refresh)");
        });
    }

    fn reload_scene(&mut self) {
        let Some(state) = self.state.as_mut() else { return };
        match net::fetch_scene(&self.host, self.port) {
            Ok((doc, entities)) => {
                eprintln!(
                    "scene: doc {doc}, {} entities (live; R = manual refresh)",
                    entities.len()
                );
                state.set_scene(entities);
            },
            Err(err) => {
                eprintln!("scene fetch failed: {err}");
                eprintln!("tip: start a server (`agcad --tcp {0}`) and create a document via the Python client, then press R", self.port);
                state.set_scene(Vec::new());
            },
        }
    }
}

impl ApplicationHandler for App {
    fn resumed(&mut self, event_loop: &ActiveEventLoop) {
        if self.state.is_some() {
            return;
        }
        let attrs = Window::default_attributes().with_title("agcad viewer");
        let window = Arc::new(event_loop.create_window(attrs).expect("cannot create window"));
        match State::new(&window) {
            Ok(state) => {
                self.window = Some(window);
                self.state = Some(state);
                self.reload_scene();
                self.start_live_listener();
            },
            Err(err) => {
                eprintln!("viewer init failed: {err}");
                event_loop.exit();
            },
        }
    }

    fn window_event(
        &mut self,
        event_loop: &ActiveEventLoop,
        _window_id: WindowId,
        event: WindowEvent,
    ) {
        let Some(_window) = self.window.clone() else { return };
        match event {
            WindowEvent::CloseRequested => event_loop.exit(),
            WindowEvent::Resized(size) => {
                if let Some(state) = self.state.as_mut() {
                    state.resize(size.width, size.height);
                }
            },
            WindowEvent::RedrawRequested => {
                if let Some(state) = self.state.as_mut() {
                    state.render();
                }
            },
            WindowEvent::MouseInput { state: mouse, button, .. } => {
                if button == winit::event::MouseButton::Left {
                    self.dragging = mouse == ElementState::Pressed;
                }
            },
            WindowEvent::CursorMoved { position, .. } => {
                if self.dragging {
                    if let Some(last) = self.last_cursor {
                        if let Some(state) = self.state.as_mut() {
                            state.camera.orbit(position.x - last.0, position.y - last.1);
                        }
                    }
                }
                self.last_cursor = Some((position.x, position.y));
            },
            WindowEvent::MouseWheel { delta, .. } => {
                let y = match delta {
                    MouseScrollDelta::LineDelta(_, y) => y,
                    MouseScrollDelta::PixelDelta(p) => (p.y / 60.0) as f32,
                };
                if let Some(state) = self.state.as_mut() {
                    state.camera.zoom((-y * 0.12).exp());
                }
            },
            WindowEvent::KeyboardInput { event, .. } => {
                if event.state == ElementState::Pressed
                    && event.physical_key == PhysicalKey::Code(KeyCode::KeyR)
                {
                    self.reload_scene();
                }
            },
            _ => {},
        }
    }

    fn about_to_wait(&mut self, _event_loop: &ActiveEventLoop) {
        // Drain any pending scene_changed ticks and reload once.
        let mut reload = false;
        if let Some(rx) = &self.reload_rx {
            while rx.try_recv().is_ok() {
                reload = true;
            }
        }
        if reload {
            self.reload_scene();
        }
        if let Some(window) = &self.window {
            window.request_redraw();
        }
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut host = "127.0.0.1".to_string();
    let mut port: u16 = 7860;
    let mut args = std::env::args().skip(1);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--host" => host = args.next().unwrap_or(host),
            "--port" => port = args.next().and_then(|v| v.parse().ok()).unwrap_or(port),
            other => {
                eprintln!("usage: agcad-view [--host HOST] [--port PORT]");
                if other != "--help" && other != "-h" {
                    return Err(format!("unknown argument: {other}").into());
                }
                return Ok(());
            },
        }
    }

    eprintln!("agcad-view | scene source: tcp://{host}:{port}");
    let event_loop = EventLoop::new()?;
    let mut app = App::new(host, port);
    event_loop.run_app(&mut app)?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use agcad_protocol::{Bbox, Mesh};

    fn one_triangle() -> EntitySnapshot {
        EntitySnapshot {
            id: 1,
            name: "triangle".to_string(),
            translation: [0.0; 3],
            rotation_deg: [0.0; 3],
            scale: [1.0; 3],
            transform: cam::identity(),
            world_bbox: Bbox { min: [0.0; 3], max: [1.0; 3] },
            color: [1.0, 0.0, 0.0, 1.0],
            mesh: Mesh {
                positions: vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]],
                normals: vec![[0.0, 0.0, 1.0]; 3],
                indices: vec![0, 1, 2],
            },
            sketch: None,
        }
    }

    #[test]
    fn grid_draw_starts_after_entity_vertices() {
        let data = build_scene(&[one_triangle()]);
        let grid = data
            .draws
            .iter()
            .find(|d| d.line && !d.overlay)
            .expect("grid draw");

        // Entity vertices come first, so the grid must not start at vertex 0.
        assert_eq!(grid.first_index, 3);
        assert_eq!(grid.count, 84); // 21 lines in each direction, two vertices each.
    }

    #[test]
    fn gizmo_axes_are_overlay_lines_after_the_grid() {
        let data = build_scene(&[one_triangle()]);
        let grid = data
            .draws
            .iter()
            .find(|d| d.line && !d.overlay)
            .expect("grid draw");
        let gizmo: Vec<_> = data.draws.iter().filter(|d| d.overlay).collect();

        // Three axes, one line each, one instance each (distinct colours).
        assert_eq!(gizmo.len(), 3);
        assert!(gizmo.iter().all(|d| d.line && d.count == 2));
        let first = gizmo[0];
        assert_eq!(first.first_index, grid.first_index + grid.count);
        let total: u32 = gizmo.iter().map(|d| d.count).sum();
        assert_eq!(first.first_index + total, (data.verts.len() / 6) as u32);

        // Every draw in this scene uses exactly one instance (20 matrix
        // floats + 4 colour floats).
        assert_eq!(data.inst.len() / 20, data.draws.len());
    }

    #[test]
    fn gizmo_axes_fit_inside_the_ortho_view() {
        // Flattened column-major: m[col * 4 + row].
        let project = |m: &[f32; 16], p: [f32; 3]| {
            let at = |col: usize, row: usize| m[col * 4 + row];
            let x = at(0, 0) * p[0] + at(1, 0) * p[1] + at(2, 0) * p[2] + at(3, 0);
            let y = at(0, 1) * p[0] + at(1, 1) * p[1] + at(2, 1) * p[2] + at(3, 1);
            let w = at(0, 3) * p[0] + at(1, 3) * p[1] + at(2, 3) * p[2] + at(3, 3);
            (x / w, y / w, w)
        };

        let mut cams = vec![OrbitCamera::new()];
        for (yaw, pitch) in [(-0.7, 1.5), (-0.7, -1.5), (2.0, 0.9), (-2.4, -0.4)] {
            cams.push(OrbitCamera {
                yaw,
                pitch,
                dist: 250.0,
                target: [0.0, 0.0, 20.0],
            });
        }
        for cam in &cams {
            let m = cam.gizmo_view_proj();
            for p in [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]] {
                let (x, y, w) = project(&m, p);
                assert!(w > 0.0, "point behind the gizmo camera: {p:?}");
                assert!(x.abs() <= 1.0, "x outside clip: {p:?}");
                assert!(y.abs() <= 1.0, "y outside clip: {p:?}");
            }
        }

        // At the default view the camera sits at +X/-Y/+Z, so +X points
        // down-right, +Y up-right and +Z straight up.
        let m = OrbitCamera::new().gizmo_view_proj();
        let (x, y, _) = project(&m, [1.0, 0.0, 0.0]);
        assert!(x > 0.2 && y < 0.0, "X should point down-right, got {x},{y}");
        let (x, y, _) = project(&m, [0.0, 1.0, 0.0]);
        assert!(x > 0.2 && y > 0.0, "Y should point up-right, got {x},{y}");
        let (x, y, _) = project(&m, [0.0, 0.0, 1.0]);
        assert!(x.abs() < 1e-6 && y > 0.2, "Z should point up, got {x},{y}");
    }
}
