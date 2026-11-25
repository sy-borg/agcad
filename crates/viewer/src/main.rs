//! `wright-view` — the dumb renderer.
//!
//! Philosophy (Lite XL style, scaled up to CAD): this process owns **no
//! document state**. It pulls a scene snapshot from the server over the
//! command bus, uploads it to the GPU, draws it. A second connection listens
//! for `scene_changed` and triggers a re-fetch, so live preview is automatic;
//! `R` forces a manual refresh.
//!
//! Controls: left/MMB drag = orbit, Shift+MMB drag = pan, wheel = zoom,
//! `R` = manual refresh, `L` = toggle
//! entity labels, `Tab`/`[`/`]` = switch document, `1`-`9` = select a
//! document. A click casts a ray (`pick`) and shows the geometric selector
//! in a HUD; `C`/`F`/`E` copy the edge-or-face/face/edge selector. A corner
//! triad shows the world axes and follows the camera.
//!
//! Document selection is view state, not document state: the viewer holds
//! the choice locally and passes it to `get_scene`, so several viewers can
//! watch different documents on the same shared engine.

mod cam;
mod net;



use std::io::{BufRead, BufReader, Write};
use std::net::TcpStream;
use std::process::{Command as Process, Stdio};
use std::sync::Arc;
use std::sync::mpsc;
use std::time::Duration;

use wright_protocol::{DocId, DocInfo, EntitySnapshot, Mat4, PickHit, Ray};
use font8x8::UnicodeFonts;
use winit::application::ApplicationHandler;
use winit::event::{ElementState, MouseButton, MouseScrollDelta, WindowEvent};
use winit::event_loop::{ActiveEventLoop, EventLoop};
use winit::keyboard::{KeyCode, ModifiersState, PhysicalKey};
use winit::window::{Window, WindowId};

/// Corner axis-gizmo size and margin, in physical pixels.
const GIZMO_SIZE: f32 = 110.0;
const GIZMO_MARGIN: f32 = 16.0;
/// Click radius for edge snapping, in physical pixels. Converted to world
/// units at the orbit target before the ray goes to the engine.
const PICK_RADIUS_PX: f32 = 6.0;
/// Vertical field of view. Shared by projection, picking and panning so a
/// screen-space drag covers the same world distance in every path.
const FOV_Y_DEG: f32 = 50.0;
/// Bitmap text: one 8x8 cell per printable ASCII character plus one solid
/// white cell for panels and dots.
const FONT_COLS: u32 = 16;
const FONT_ROWS: u32 = 7;
const ATLAS_W: u32 = FONT_COLS * 8;
const ATLAS_H: u32 = FONT_ROWS * 8;
const WHITE_CELL: u32 = 96;

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

/// Screen-space bitmap text. Positions arrive in physical pixels with the
/// origin top-left; the shader maps them to clip space. Sampling an R8
/// glyph atlas with nearest filtering keeps labels crisp at any scale.
const TEXT_WGSL: &str = r#"
struct Screen {
    size: vec2<f32>,
    _pad: vec2<f32>,
};
@group(0) @binding(0) var<uniform> screen: Screen;
@group(1) @binding(0) var atlas: texture_2d<f32>;
@group(1) @binding(1) var samp: sampler;

struct VSOut {
    @builtin(position) pos: vec4<f32>,
    @location(0) uv: vec2<f32>,
    @location(1) color: vec4<f32>,
};

@vertex
fn vs(@location(0) pos: vec2<f32>, @location(1) uv: vec2<f32>, @location(2) color: vec4<f32>) -> VSOut {
    var out: VSOut;
    out.pos = vec4<f32>(
        pos.x / screen.size.x * 2.0 - 1.0,
        1.0 - pos.y / screen.size.y * 2.0,
        0.0,
        1.0,
    );
    out.uv = uv;
    out.color = color;
    return out;
}

@fragment
fn fs(in: VSOut) -> @location(0) vec4<f32> {
    let a = textureSample(atlas, samp, in.uv).r;
    return vec4<f32>(in.color.rgb, in.color.a * a);
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
    /// Screen-space bitmap text + panels (labels and the pick HUD).
    pipeline_text: wgpu::RenderPipeline,
    bind_group_0: wgpu::BindGroup,
    bind_group_1: wgpu::BindGroup,
    text_screen_buf: wgpu::Buffer,
    text_screen_bg: wgpu::BindGroup,
    text_atlas_bg: wgpu::BindGroup,
    text_gpu: TextGpu,
    uniform_buf: wgpu::Buffer,
    gizmo_uniform_buf: wgpu::Buffer,
    gizmo_bind_group: wgpu::BindGroup,
    depth_tex: wgpu::Texture,
    depth_view: wgpu::TextureView,
    size: (u32, u32),
    scene: Option<SceneGpu>,
    camera: OrbitCamera,
    /// Last fetched scene, kept CPU-side so labels can follow the camera.
    entities: Vec<EntitySnapshot>,
    /// Documents the last refresh saw, for the HUD picker.
    docs: Vec<DocInfo>,
    /// The document currently on screen (`None` when the engine is empty).
    doc: Option<DocId>,
    labels_on: bool,
    pick: PickState,
}

/// What the last click found. The HUD keeps showing a hit even after a
/// live rebuild; the marker is only drawn while the entity still exists.
enum PickState {
    None,
    Hit(Box<PickHit>),
    Miss,
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
                label: Some("wright-viewer"),
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
            label: Some("wright shader"),
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
            label: Some("wright layout"),
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
                    "wright overlay lines"
                } else if topology == wgpu::PrimitiveTopology::TriangleList {
                    "wright tri"
                } else {
                    "wright lines"
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

        // --- screen-space text -------------------------------------------------
        let text_shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("wright text shader"),
            source: wgpu::ShaderSource::Wgsl(TEXT_WGSL.into()),
        });
        let text_bgl0 = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("text screen"),
            entries: &[wgpu::BindGroupLayoutEntry {
                binding: 0,
                visibility: wgpu::ShaderStages::VERTEX,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Uniform,
                    has_dynamic_offset: false,
                    min_binding_size: wgpu::BufferSize::new(16),
                },
                count: None,
            }],
        });
        let text_bgl1 = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("text atlas"),
            entries: &[
                wgpu::BindGroupLayoutEntry {
                    binding: 0,
                    visibility: wgpu::ShaderStages::FRAGMENT,
                    ty: wgpu::BindingType::Texture {
                        sample_type: wgpu::TextureSampleType::Float { filterable: false },
                        view_dimension: wgpu::TextureViewDimension::D2,
                        multisampled: false,
                    },
                    count: None,
                },
                wgpu::BindGroupLayoutEntry {
                    binding: 1,
                    visibility: wgpu::ShaderStages::FRAGMENT,
                    ty: wgpu::BindingType::Sampler(wgpu::SamplerBindingType::NonFiltering),
                    count: None,
                },
            ],
        });
        let text_screen_buf = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("text screen size"),
            size: 16,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let text_screen_bg = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("bg text screen"),
            layout: &text_bgl0,
            entries: &[wgpu::BindGroupEntry {
                binding: 0,
                resource: text_screen_buf.as_entire_binding(),
            }],
        });
        let atlas_tex = device.create_texture(&wgpu::TextureDescriptor {
            label: Some("font atlas"),
            size: wgpu::Extent3d { width: ATLAS_W, height: ATLAS_H, depth_or_array_layers: 1 },
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: wgpu::TextureFormat::R8Unorm,
            usage: wgpu::TextureUsages::TEXTURE_BINDING | wgpu::TextureUsages::COPY_DST,
            view_formats: &[],
        });
        let atlas_pixels = build_font_atlas();
        queue.write_texture(
            wgpu::TexelCopyTextureInfo {
                texture: &atlas_tex,
                mip_level: 0,
                origin: wgpu::Origin3d::ZERO,
                aspect: wgpu::TextureAspect::All,
            },
            &atlas_pixels,
            wgpu::TexelCopyBufferLayout {
                offset: 0,
                bytes_per_row: Some(ATLAS_W),
                rows_per_image: Some(ATLAS_H),
            },
            wgpu::Extent3d { width: ATLAS_W, height: ATLAS_H, depth_or_array_layers: 1 },
        );
        let atlas_view = atlas_tex.create_view(&wgpu::TextureViewDescriptor::default());
        let atlas_sampler = device.create_sampler(&wgpu::SamplerDescriptor {
            label: Some("font sampler"),
            mag_filter: wgpu::FilterMode::Nearest,
            min_filter: wgpu::FilterMode::Nearest,
            address_mode_u: wgpu::AddressMode::ClampToEdge,
            address_mode_v: wgpu::AddressMode::ClampToEdge,
            ..Default::default()
        });
        let text_atlas_bg = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("bg text atlas"),
            layout: &text_bgl1,
            entries: &[
                wgpu::BindGroupEntry { binding: 0, resource: wgpu::BindingResource::TextureView(&atlas_view) },
                wgpu::BindGroupEntry { binding: 1, resource: wgpu::BindingResource::Sampler(&atlas_sampler) },
            ],
        });
        let text_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("text layout"),
            bind_group_layouts: &[Some(&text_bgl0), Some(&text_bgl1)],
            immediate_size: 0,
        });
        let pipeline_text = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("wright text"),
            layout: Some(&text_layout),
            vertex: wgpu::VertexState {
                module: &text_shader,
                entry_point: Some("vs"),
                buffers: &[Some(wgpu::VertexBufferLayout {
                    array_stride: 32,
                    step_mode: wgpu::VertexStepMode::Vertex,
                    attributes: &[
                        wgpu::VertexAttribute {
                            format: wgpu::VertexFormat::Float32x2,
                            offset: 0,
                            shader_location: 0,
                        },
                        wgpu::VertexAttribute {
                            format: wgpu::VertexFormat::Float32x2,
                            offset: 8,
                            shader_location: 1,
                        },
                        wgpu::VertexAttribute {
                            format: wgpu::VertexFormat::Float32x4,
                            offset: 16,
                            shader_location: 2,
                        },
                    ],
                })],
                compilation_options: Default::default(),
            },
            fragment: Some(wgpu::FragmentState {
                module: &text_shader,
                entry_point: Some("fs"),
                targets: &[Some(wgpu::ColorTargetState {
                    format: surface_format,
                    blend: Some(wgpu::BlendState::ALPHA_BLENDING),
                    write_mask: wgpu::ColorWrites::ALL,
                })],
                compilation_options: Default::default(),
            }),
            primitive: wgpu::PrimitiveState {
                topology: wgpu::PrimitiveTopology::TriangleList,
                cull_mode: None,
                ..Default::default()
            },
            depth_stencil: Some(wgpu::DepthStencilState {
                format: wgpu::TextureFormat::Depth24Plus,
                depth_write_enabled: Some(false),
                depth_compare: Some(wgpu::CompareFunction::Always),
                stencil: wgpu::StencilState::default(),
                bias: wgpu::DepthBiasState::default(),
            }),
            multisample: wgpu::MultisampleState::default(),
            multiview_mask: None,
            cache: None,
        });

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
        let text_gpu = TextGpu::new(&device);

        Ok(Self {
            device,
            queue,
            surface,
            surface_format,
            pipeline_tri,
            pipeline_line,
            pipeline_line_overlay,
            pipeline_text,
            bind_group_0,
            bind_group_1,
            text_screen_buf,
            text_screen_bg,
            text_atlas_bg,
            text_gpu,
            uniform_buf,
            gizmo_uniform_buf,
            gizmo_bind_group,
            depth_tex,
            depth_view,
            size: (width, height),
            scene: None,
            camera: OrbitCamera::new(),
            entities: Vec::new(),
            docs: Vec::new(),
            doc: None,
            labels_on: true,
            pick: PickState::None,
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
        self.entities = entities;

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

    /// Record the document list and resolved selection for the HUD. The
    /// scene itself arrives separately through `set_scene`.
    fn set_documents(&mut self, docs: Vec<DocInfo>, doc: Option<DocId>) {
        self.docs = docs;
        self.doc = doc;
    }

    fn render(&mut self) {
        let vp = self.camera.view_proj(self.size);
        self.queue.write_buffer(&self.uniform_buf, 0, bytemuck::cast_slice(&vp));
        let gizmo_vp = self.camera.gizmo_view_proj();
        self.queue
            .write_buffer(&self.gizmo_uniform_buf, 0, bytemuck::cast_slice(&gizmo_vp));
        let overlay = build_overlay(
            &self.entities,
            &self.camera,
            self.size,
            self.labels_on,
            &self.pick,
            &self.docs,
            self.doc,
        );
        self.queue.write_buffer(
            &self.text_screen_buf,
            0,
            bytemuck::cast_slice(&[self.size.0 as f32, self.size.1 as f32, 0.0, 0.0]),
        );
        if !overlay.is_empty() {
            self.text_gpu.upload(&self.device, &self.queue, &overlay);
        }

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

            if !overlay.is_empty() {
                pass.set_pipeline(&self.pipeline_text);
                pass.set_bind_group(0, &self.text_screen_bg, &[]);
                pass.set_bind_group(1, &self.text_atlas_bg, &[]);
                pass.set_vertex_buffer(0, self.text_gpu.vertex.slice(..));
                pass.set_index_buffer(self.text_gpu.index.slice(..), wgpu::IndexFormat::Uint32);
                pass.draw_indexed(0..overlay.idx.len() as u32, 0, 0..1);
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

// --- screen-space text -------------------------------------------------------

/// One glyph cell's UV window in the atlas.
#[derive(Clone, Copy)]
struct CellUv {
    u0: f32,
    v0: f32,
    u1: f32,
    v1: f32,
}

fn atlas_uv(cell: u32) -> CellUv {
    let col = cell % FONT_COLS;
    let row = cell / FONT_COLS;
    let inset = 0.001; // half-texel-ish guard against bleed at nearest edges
    CellUv {
        u0: (col as f32 * 8.0) / ATLAS_W as f32 + inset,
        v0: (row as f32 * 8.0) / ATLAS_H as f32 + inset,
        u1: ((col as f32 + 1.0) * 8.0) / ATLAS_W as f32 - inset,
        v1: ((row as f32 + 1.0) * 8.0) / ATLAS_H as f32 - inset,
    }
}

/// Printable ASCII maps to cells 0..95; the last cell is solid white.
fn glyph_cell(ch: char) -> u32 {
    let code = ch as u32;
    if (0x20..=0x7e).contains(&code) { code - 0x20 } else { ('?' as u32) - 0x20 }
}

/// Build the R8 glyph atlas: one 8x8 cell per character, row-major, with
/// font8x8's BASIC_FONTS. The final cell is filled white so panels and
/// leader dots can reuse the same pipeline.
fn build_font_atlas() -> Vec<u8> {
    let mut pixels = vec![0u8; (ATLAS_W * ATLAS_H) as usize];
    for code in 0x20u8..=0x7e {
        let Some(glyph) = font8x8::BASIC_FONTS.get(code as char) else { continue };
        let cell = (code as u32) - 0x20;
        let (cx, cy) = ((cell % FONT_COLS) * 8, (cell / FONT_COLS) * 8);
        for (row, bits) in glyph.iter().enumerate() {
            for col in 0..8 {
                if (bits >> col) & 1 != 0 {
                    pixels[((cy + row as u32) * ATLAS_W + cx + col) as usize] = 255;
                }
            }
        }
    }
    let (cx, cy) = ((WHITE_CELL % FONT_COLS) * 8, (WHITE_CELL / FONT_COLS) * 8);
    for y in cy..cy + 8 {
        for x in cx..cx + 8 {
            pixels[(y * ATLAS_W + x) as usize] = 255;
        }
    }
    pixels
}

/// CPU-side triangle list for the text pass. Positions are physical pixels
/// (origin top-left); every glyph and panel is two triangles. GPU vertices
/// are pos2 + uv2 + color4.
#[derive(Default)]
struct TextBatch {
    verts: Vec<[f32; 8]>,
    idx: Vec<u32>,
}

impl TextBatch {
    fn quad(&mut self, x: f32, y: f32, w: f32, h: f32, color: [f32; 4], cell: CellUv) {
        let base = self.verts.len() as u32;
        for (px, py, u, v) in [
            (x, y, cell.u0, cell.v0),
            (x + w, y, cell.u1, cell.v0),
            (x + w, y + h, cell.u1, cell.v1),
            (x, y + h, cell.u0, cell.v1),
        ] {
            self.verts.push([px, py, u, v, color[0], color[1], color[2], color[3]]);
        }
        self.idx.extend_from_slice(&[base, base + 1, base + 2, base, base + 2, base + 3]);
    }

    fn panel(&mut self, x: f32, y: f32, w: f32, h: f32, color: [f32; 4]) {
        let white = atlas_uv(WHITE_CELL);
        // Sample the middle of the white cell for every corner.
        let cell = CellUv {
            u0: (white.u0 + white.u1) * 0.5,
            v0: (white.v0 + white.v1) * 0.5,
            u1: (white.u0 + white.u1) * 0.5,
            v1: (white.v0 + white.v1) * 0.5,
        };
        self.quad(x, y, w, h, color, cell);
    }

    fn text(&mut self, text: &str, x: f32, y: f32, scale: f32, color: [f32; 4]) {
        let mut cursor = x;
        for ch in text.chars() {
            self.quad(cursor, y, 8.0 * scale, 8.0 * scale, color, atlas_uv(glyph_cell(ch)));
            cursor += 8.0 * scale;
        }
    }

    fn is_empty(&self) -> bool {
        self.idx.is_empty()
    }

    fn width(text: &str, scale: f32) -> f32 {
        text.chars().count() as f32 * 8.0 * scale
    }
}

/// Persistent GPU buffers for the text pass. Labels move every frame while
/// the camera orbits, so the contents are rewritten but the allocations are
/// reused until a longer label needs more room.
struct TextGpu {
    vertex: wgpu::Buffer,
    vertex_cap: u64,
    index: wgpu::Buffer,
    index_cap: u64,
}

impl TextGpu {
    fn new(device: &wgpu::Device) -> Self {
        let vertex_cap = 4096;
        let index_cap = 4096;
        Self {
            vertex: device.create_buffer(&wgpu::BufferDescriptor {
                label: Some("text vertices"),
                size: vertex_cap,
                usage: wgpu::BufferUsages::VERTEX | wgpu::BufferUsages::COPY_DST,
                mapped_at_creation: false,
            }),
            vertex_cap,
            index: device.create_buffer(&wgpu::BufferDescriptor {
                label: Some("text indices"),
                size: index_cap,
                usage: wgpu::BufferUsages::INDEX | wgpu::BufferUsages::COPY_DST,
                mapped_at_creation: false,
            }),
            index_cap,
        }
    }

    /// Upload one batch, growing a buffer only when it no longer fits.
    fn upload(&mut self, device: &wgpu::Device, queue: &wgpu::Queue, batch: &TextBatch) {
        let vbytes = (batch.verts.len() * 32) as u64;
        if vbytes > self.vertex_cap {
            self.vertex_cap = vbytes.next_power_of_two();
            self.vertex = device.create_buffer(&wgpu::BufferDescriptor {
                label: Some("text vertices"),
                size: self.vertex_cap,
                usage: wgpu::BufferUsages::VERTEX | wgpu::BufferUsages::COPY_DST,
                mapped_at_creation: false,
            });
        }
        let ibytes = (batch.idx.len() * 4) as u64;
        if ibytes > self.index_cap {
            self.index_cap = ibytes.next_power_of_two();
            self.index = device.create_buffer(&wgpu::BufferDescriptor {
                label: Some("text indices"),
                size: self.index_cap,
                usage: wgpu::BufferUsages::INDEX | wgpu::BufferUsages::COPY_DST,
                mapped_at_creation: false,
            });
        }
        if vbytes > 0 {
            queue.write_buffer(&self.vertex, 0, bytemuck::cast_slice(&batch.verts));
        }
        if ibytes > 0 {
            queue.write_buffer(&self.index, 0, bytemuck::cast_slice(&batch.idx));
        }
    }
}

/// Project a world point through a column-major view-projection matrix.
/// Returns physical-pixel coordinates, or `None` when the point is behind
/// the camera or outside the frame.
fn project(vp: &[f32; 16], p: [f32; 3], size: (u32, u32)) -> Option<(f32, f32)> {
    let at = |col: usize, row: usize| vp[col * 4 + row];
    let x = at(0, 0) * p[0] + at(1, 0) * p[1] + at(2, 0) * p[2] + at(3, 0);
    let y = at(0, 1) * p[0] + at(1, 1) * p[1] + at(2, 1) * p[2] + at(3, 1);
    let w = at(0, 3) * p[0] + at(1, 3) * p[1] + at(2, 3) * p[2] + at(3, 3);
    if w <= 1e-4 {
        return None;
    }
    let (nx, ny) = (x / w, y / w);
    if !(-1.0..=1.0).contains(&nx) || !(-1.0..=1.0).contains(&ny) {
        return None;
    }
    Some((
        (nx * 0.5 + 0.5) * size.0 as f32,
        (1.0 - (ny * 0.5 + 0.5)) * size.1 as f32,
    ))
}

/// Break a long HUD line at a fixed character budget. JSON has no spaces,
/// so a hard split is the honest choice; the copied text is intact.
fn wrap_line(line: &str, max: usize) -> Vec<String> {
    let chars: Vec<char> = line.chars().collect();
    if chars.len() <= max {
        return vec![line.to_string()];
    }
    chars.chunks(max).map(|c| c.iter().collect()).collect()
}

fn selector_json<T: serde::Serialize>(selector: &T) -> String {
    serde_json::to_string(selector).unwrap_or_default()
}

/// The selector a plain `C` copies: an edge when the click snapped to one,
/// otherwise the face. `F`/`E` choose explicitly.
fn primary_selector(pick: &PickState) -> Option<String> {
    match pick {
        PickState::Hit(hit) => Some(match &hit.edge {
            Some(edge) => selector_json(edge),
            None => selector_json(&hit.face),
        }),
        _ => None,
    }
}

fn face_selector(pick: &PickState) -> Option<String> {
    match pick {
        PickState::Hit(hit) => Some(selector_json(&hit.face)),
        _ => None,
    }
}

fn edge_selector(pick: &PickState) -> Option<String> {
    match pick {
        PickState::Hit(hit) => hit.edge.as_ref().map(selector_json),
        _ => None,
    }
}

/// The HUD's document line: which document is on screen, out of how many,
/// and the key hint when there is something to switch to.
fn document_line(docs: &[DocInfo], doc: Option<DocId>) -> String {
    let Some(doc) = doc else {
        return "no open documents - waiting (R to retry)".into();
    };
    let Some(index) = docs.iter().position(|d| d.doc == doc) else {
        return format!("doc #{doc}");
    };
    let info = &docs[index];
    if docs.len() > 1 {
        format!(
            "doc {}/{}: \"{}\"  #{}   Tab/[ ] switch",
            index + 1,
            docs.len(),
            info.name,
            doc
        )
    } else {
        format!("doc 1/1: \"{}\"  #{}", info.name, doc)
    }
}

/// Build the screen-space overlay: entity labels (toggleable), the hit
/// marker and the pick HUD. Pure, so its geometry is unit-testable without
/// a GPU.
fn build_overlay(
    entities: &[EntitySnapshot],
    camera: &OrbitCamera,
    size: (u32, u32),
    labels: bool,
    pick: &PickState,
    docs: &[DocInfo],
    doc: Option<DocId>,
) -> TextBatch {
    let mut batch = TextBatch::default();
    let vp = camera.view_proj(size);

    if labels {
        for (i, entity) in entities.iter().enumerate() {
            let b = entity.world_bbox;
            let center = [
                ((b.min[0] + b.max[0]) * 0.5) as f32,
                ((b.min[1] + b.max[1]) * 0.5) as f32,
                ((b.min[2] + b.max[2]) * 0.5) as f32,
            ];
            let Some((sx, sy)) = project(&vp, center, size) else { continue };
            let selected = matches!(pick, PickState::Hit(h) if h.entity == entity.id);
            let color = if selected {
                [1.0, 0.72, 0.25, 1.0]
            } else {
                [0.94, 0.95, 0.98, 0.95]
            };
            // The leader dot sits on the projected bbox centre; the label
            // is staggered a little so parallel parts do not collide.
            batch.panel(sx - 2.5, sy - 2.5, 5.0, 5.0, color);
            let text = format!("{} #{}", entity.name, entity.id);
            let x = sx + 9.0;
            let y = sy - 28.0 - (i % 3) as f32 * 18.0;
            batch.text(&text, x + 1.0, y + 1.0, 2.0, [0.0, 0.0, 0.0, 0.8]);
            batch.text(&text, x, y, 2.0, color);
        }
    }

    // Hit marker: an orange cross at the picked point, while that entity
    // still exists (a rebuild can delete it).
    if let PickState::Hit(hit) = pick
        && entities.iter().any(|e| e.id == hit.entity)
        && let Some((sx, sy)) = project(
            &vp,
            [hit.point[0] as f32, hit.point[1] as f32, hit.point[2] as f32],
            size,
        )
    {
        let color = [1.0, 0.72, 0.25, 1.0];
        batch.panel(sx - 7.0, sy - 1.5, 14.0, 3.0, color);
        batch.panel(sx - 1.5, sy - 7.0, 3.0, 14.0, color);
    }

    // HUD: the selectors as exact JSON, ready to copy into the chat.
    let scale = 1.5;
    let line_height = 8.0 * scale + 6.0;
    let mut lines: Vec<String> = vec![document_line(docs, doc)];
    match pick {
        PickState::None => {
            lines.push("click a face or edge to pick  ·  L labels  ·  R reload".into());
        },
        PickState::Miss => {
            lines.push("no hit — nothing solid under the cursor".into());
            lines.push("click a face or edge to pick  ·  L labels  ·  R reload".into());
        },
        PickState::Hit(hit) => {
            lines.push(format!(
                "picked {} #{}  @ [{:.2}, {:.2}, {:.2}]",
                hit.name, hit.entity, hit.point[0], hit.point[1], hit.point[2]
            ));
            lines.push(format!("face: {}", selector_json(&hit.face)));
            if let Some(edge) = &hit.edge {
                lines.push(format!("edge: {}", selector_json(edge)));
                lines.push("C copy edge  ·  F face  ·  E edge  ·  L labels  ·  Esc clear".into());
            } else {
                lines.push("C/F copy face  ·  L labels  ·  Esc clear".into());
            }
        },
    }
    let max_chars = (((size.0 as f32 - 32.0) / (8.0 * scale)).floor().max(20.0)) as usize;
    let wrapped: Vec<String> = lines.iter().flat_map(|l| wrap_line(l, max_chars)).collect();
    let longest = wrapped.iter().map(|l| l.chars().count()).max().unwrap_or(0);
    let panel_w = TextBatch::width(&"x".repeat(longest), scale) + 20.0;
    let panel_h = wrapped.len() as f32 * line_height + 10.0;
    batch.panel(8.0, 8.0, panel_w, panel_h, [0.03, 0.04, 0.06, 0.78]);
    for (i, line) in wrapped.iter().enumerate() {
        batch.text(line, 18.0, 14.0 + i as f32 * line_height, scale, [0.92, 0.94, 0.98, 1.0]);
    }

    batch
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
        let proj = cam::perspective_opengl(FOV_Y_DEG, aspect, 1.0, 20_000.0);
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

    /// Slide the camera target in the screen plane, Blender-style: the world
    /// follows the cursor, so a right/down drag moves the target left/up.
    /// `dx`/`dy` are physical pixels, `size` is the viewport in the same units.
    fn pan(&mut self, dx: f64, dy: f64, size: (u32, u32)) {
        let eye = self.eye();
        let forward = cam::normalize([
            self.target[0] - eye[0],
            self.target[1] - eye[1],
            self.target[2] - eye[2],
        ]);
        let right = cam::normalize(cam::cross(forward, [0.0, 0.0, 1.0]));
        let up = cam::cross(right, forward);
        let world_per_px =
            2.0 * self.dist * (FOV_Y_DEG.to_radians() * 0.5).tan() / size.1.max(1) as f32;
        let k = world_per_px as f64;
        for (axis, offset) in [(&right, -dx), (&up, dy)] {
            self.target[0] += (axis[0] as f64 * offset * k) as f32;
            self.target[1] += (axis[1] as f64 * offset * k) as f32;
            self.target[2] += (axis[2] as f64 * offset * k) as f32;
        }
    }

    /// World-space ray through a physical-pixel position, plus a pick
    /// tolerance in mm for the same click (a few pixels at the orbit
    /// distance, so edge snapping feels the same at every zoom).
    fn ray(&self, px: f64, py: f64, size: (u32, u32)) -> (Ray, f64) {
        let eye = self.eye();
        let forward = cam::normalize([
            self.target[0] - eye[0],
            self.target[1] - eye[1],
            self.target[2] - eye[2],
        ]);
        let right = cam::normalize(cam::cross(forward, [0.0, 0.0, 1.0]));
        let up = cam::cross(right, forward);
        let tan_half = (FOV_Y_DEG.to_radians() * 0.5).tan();
        let aspect = size.0 as f32 / size.1.max(1) as f32;
        let nx = 2.0 * px as f32 / size.0.max(1) as f32 - 1.0;
        let ny = 1.0 - 2.0 * py as f32 / size.1.max(1) as f32;
        let dir = cam::normalize([
            forward[0] + right[0] * nx * aspect * tan_half + up[0] * ny * tan_half,
            forward[1] + right[1] * nx * aspect * tan_half + up[1] * ny * tan_half,
            forward[2] + right[2] * nx * aspect * tan_half + up[2] * ny * tan_half,
        ]);
        let world_per_px = 2.0 * self.dist * tan_half / size.1.max(1) as f32;
        (
            Ray {
                origin: [eye[0] as f64, eye[1] as f64, eye[2] as f64],
                direction: [dir[0] as f64, dir[1] as f64, dir[2] as f64],
            },
            (world_per_px * PICK_RADIUS_PX) as f64,
        )
    }
}

// --- app ---------------------------------------------------------------------

/// One message from the live watcher to the render loop.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum SceneEvent {
    /// Reconnect or a document-agnostic change: re-list and re-fetch.
    Refresh,
    /// A specific document changed; reload only if it is on screen.
    Changed(Option<DocId>),
}

/// Parse a `scene_changed` notification line. Returns `None` for responses
/// and unrelated messages; a missing `params.doc` becomes `Changed(None)`.
fn parse_scene_event(line: &str) -> Option<SceneEvent> {
    let value: serde_json::Value = serde_json::from_str(line).ok()?;
    if value.get("method")?.as_str()? != "scene_changed" {
        return None;
    }
    let doc = value
        .get("params")
        .and_then(|p| p.get("doc"))
        .and_then(serde_json::Value::as_u64);
    Some(SceneEvent::Changed(doc))
}

/// Zero-based document slot for the number keys `1`-`9`.
fn digit_slot(code: KeyCode) -> Option<usize> {
    Some(match code {
        KeyCode::Digit1 => 0,
        KeyCode::Digit2 => 1,
        KeyCode::Digit3 => 2,
        KeyCode::Digit4 => 3,
        KeyCode::Digit5 => 4,
        KeyCode::Digit6 => 5,
        KeyCode::Digit7 => 6,
        KeyCode::Digit8 => 7,
        KeyCode::Digit9 => 8,
        _ => return None,
    })
}

/// Middle-button drag mode: Blender-style MMB orbits, Shift+MMB pans.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum MiddleDrag {
    Orbit,
    Pan,
}

struct App {
    host: String,
    port: u16,
    /// The document the viewer should show. `None` follows the first open
    /// document; cycling or `1`-`9` pins an explicit choice.
    selected_doc: Option<DocId>,
    /// The resolved document currently on screen.
    doc: Option<DocId>,
    dragging: bool,
    /// Active middle-button drag, if any.
    mmb: Option<MiddleDrag>,
    /// Keyboard modifiers, for Shift+MMB panning.
    modifiers: ModifiersState,
    /// Where the left button went down, and whether the pointer moved far
    /// enough to make it an orbit rather than a click.
    press_at: Option<(f64, f64)>,
    moved: bool,
    last_cursor: Option<(f64, f64)>,
    window: Option<Arc<Window>>,
    state: Option<State>,
    /// Receives an event whenever the server broadcasts `scene_changed`.
    reload_rx: Option<mpsc::Receiver<SceneEvent>>,
}

impl App {
    fn new(host: String, port: u16, doc: Option<DocId>) -> Self {
        Self {
            host,
            port,
            selected_doc: doc,
            doc: None,
            dragging: false,
            mmb: None,
            modifiers: ModifiersState::default(),
            press_at: None,
            moved: false,
            last_cursor: None,
            window: None,
            state: None,
            reload_rx: None,
        }
    }

    /// Persistent connection that only listens for `scene_changed`
    /// notifications; every tick nudges the render loop to re-fetch.
    ///
    /// The listener reconnects by itself: a backend restart closes the
    /// stream, and the viewer must not go permanently deaf. Every (re)connect
    /// also sends a tick so the scene is re-fetched immediately; `R` remains
    /// a manual fallback.
    fn start_live_listener(&mut self) {
        let (tx, rx) = mpsc::channel::<SceneEvent>();
        self.reload_rx = Some(rx);
        let host = self.host.clone();
        let port = self.port;
        std::thread::spawn(move || {
            let mut backoff = Duration::from_millis(250);
            let mut warned = false;
            loop {
                let stream = match TcpStream::connect((host.as_str(), port)) {
                    Ok(s) => {
                        if warned {
                            eprintln!("live preview: reconnected to {host}:{port}");
                        } else {
                            eprintln!("live preview: watching {host}:{port} for scene_changed");
                        }
                        warned = false;
                        backoff = Duration::from_millis(250);
                        // The scene may have changed while we were away (or
                        // the server may have restarted): refresh once now,
                        // then keep following notifications.
                        if tx.send(SceneEvent::Refresh).is_err() {
                            return; // viewer window is gone
                        }
                        s
                    },
                    Err(_) => {
                        if !warned {
                            eprintln!(
                                "live preview off: cannot connect to {host}:{port} \
                                 (retrying; it reconnects by itself)"
                            );
                            warned = true;
                        }
                        std::thread::sleep(backoff);
                        backoff = (backoff * 2).min(Duration::from_secs(2));
                        continue;
                    },
                };
                for line in BufReader::new(stream).lines() {
                    match line {
                        Ok(l) => {
                            if let Some(event) = parse_scene_event(&l)
                                && tx.send(event).is_err()
                            {
                                return;
                            }
                        },
                        Err(_) => break,
                    }
                }
                // EOF: the server went away (restart) — reconnect shortly.
                std::thread::sleep(backoff);
                backoff = (backoff * 2).min(Duration::from_secs(2));
            }
        });
    }

    fn reload_scene(&mut self) {
        if self.state.is_none() {
            return;
        }
        match net::fetch_scene(&self.host, self.port, self.selected_doc) {
            Ok(fetched) => {
                // A selected document that has been closed falls back to
                // "follow the first document" instead of staying stale.
                if self.selected_doc.is_some() && fetched.doc != self.selected_doc {
                    self.selected_doc = None;
                }
                self.doc = fetched.doc;
                let label = match fetched
                    .doc
                    .and_then(|id| fetched.docs.iter().find(|d| d.doc == id))
                {
                    Some(info) => format!("doc {} \"{}\"", info.doc, info.name),
                    None => "no open documents".into(),
                };
                eprintln!(
                    "scene: {label}, {} entities (live; Tab/[ ] switches, R = refresh)",
                    fetched.entities.len()
                );
                self.set_window_title(fetched.doc, &fetched.docs);
                let Some(state) = self.state.as_mut() else { return };
                // A rebuild can delete the picked entity: drop the stale
                // marker, but keep the HUD text — the selector may still be
                // what the human is about to hand to the agent.
                if let PickState::Hit(hit) = &state.pick
                    && !fetched.entities.iter().any(|e| e.id == hit.entity)
                {
                    state.pick = PickState::None;
                }
                state.set_documents(fetched.docs, fetched.doc);
                state.set_scene(fetched.entities);
            },
            Err(err) => {
                eprintln!("scene fetch failed: {err}");
                eprintln!("tip: start a server (`wright --tcp {0}`) and create a document via the Python client, then press R", self.port);
                self.doc = None;
                self.set_window_title(None, &[]);
                if let Some(state) = self.state.as_mut() {
                    state.set_documents(Vec::new(), None);
                    state.set_scene(Vec::new());
                }
            },
        }
    }

    fn set_window_title(&self, doc: Option<DocId>, docs: &[DocInfo]) {
        let title = match doc.and_then(|id| docs.iter().find(|d| d.doc == id)) {
            Some(info) => format!("Wright CAD viewer - {} (#{})", info.name, info.doc),
            None => "Wright CAD viewer - no open documents".into(),
        };
        if let Some(window) = &self.window {
            window.set_title(&title);
        }
    }

    /// Pin the viewer to one document and refresh. Selection is local view
    /// state, so it never travels over the bus.
    fn choose_document(&mut self, doc: DocId) {
        if self.doc == Some(doc) {
            return;
        }
        self.selected_doc = Some(doc);
        // Entity ids are per document, so a pick from the old document must
        // not leak into the new one.
        if let Some(state) = self.state.as_mut() {
            state.pick = PickState::None;
        }
        self.reload_scene();
    }

    /// Step through the open documents (Tab / bracket keys).
    fn cycle_document(&mut self, delta: isize) {
        let Some(state) = self.state.as_ref() else { return };
        if state.docs.is_empty() {
            return;
        }
        let count = state.docs.len() as isize;
        let current = self
            .doc
            .and_then(|id| state.docs.iter().position(|d| d.doc == id));
        let index = match current {
            Some(index) => (index as isize + delta).rem_euclid(count),
            None if delta < 0 => count - 1,
            None => 0,
        };
        let doc = state.docs[index as usize].doc;
        self.choose_document(doc);
    }

    /// Select a document by its `1`-`9` slot in `list_documents` order.
    fn select_document_index(&mut self, slot: usize) {
        let doc = self
            .state
            .as_ref()
            .and_then(|state| state.docs.get(slot))
            .map(|info| info.doc);
        if let Some(doc) = doc {
            self.choose_document(doc);
        }
    }

    /// Cast a ray through a click. A miss is a normal answer, not an error.
    fn do_pick(&mut self, x: f64, y: f64) {
        let Some(doc) = self.doc else { return };
        let Some(state) = self.state.as_ref() else { return };
        let (ray, tolerance) = state.camera.ray(x, y, state.size);
        match net::pick(&self.host, self.port, doc, ray, tolerance) {
            Ok(Some(hit)) => {
                // The terminal is part of the handoff too: the full hit is
                // one copy away if the clipboard helper is unavailable.
                if let Ok(json) = serde_json::to_string(&hit) {
                    eprintln!("pick: {json}");
                }
                if let Some(state) = self.state.as_mut() {
                    state.pick = PickState::Hit(Box::new(hit));
                }
            },
            Ok(None) => {
                eprintln!("pick: no hit");
                if let Some(state) = self.state.as_mut() {
                    state.pick = PickState::Miss;
                }
            },
            Err(err) => eprintln!("pick failed: {err}"),
        }
    }

    /// Copy one selector to the system clipboard (wl-copy → xclip → xsel).
    /// Falls back to the terminal print above when no helper is installed.
    fn copy_selector(&mut self, which: &str) {
        let Some(state) = self.state.as_ref() else { return };
        let text = match which {
            "face" => face_selector(&state.pick),
            "edge" => edge_selector(&state.pick),
            _ => primary_selector(&state.pick),
        };
        let Some(text) = text else {
            eprintln!("copy: nothing picked (or no edge under the click)");
            return;
        };
        if copy_to_clipboard(&text) {
            eprintln!("copied {which} selector: {text}");
        } else {
            eprintln!("clipboard helper unavailable; {which} selector is: {text}");
        }
    }
}

/// Best-effort system clipboard, dependency-free. Returns whether a helper
/// accepted the text.
fn copy_to_clipboard(text: &str) -> bool {
    let helpers: [(&str, &[&str]); 3] = [
        ("wl-copy", &[]),
        ("xclip", &["-selection", "clipboard"]),
        ("xsel", &["--clipboard", "--input"]),
    ];
    for (cmd, args) in helpers {
        let Ok(mut child) = Process::new(cmd)
            .args(args)
            .stdin(Stdio::piped())
            .stdout(Stdio::null())
            .stderr(Stdio::null())
            .spawn()
        else {
            continue;
        };
        if let Some(mut stdin) = child.stdin.take() {
            let _ = stdin.write_all(text.as_bytes());
        }
        if child.wait().map(|s| s.success()).unwrap_or(false) {
            return true;
        }
    }
    false
}

impl ApplicationHandler for App {
    fn resumed(&mut self, event_loop: &ActiveEventLoop) {
        if self.state.is_some() {
            return;
        }
        let attrs = Window::default_attributes().with_title("Wright CAD viewer");
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
            WindowEvent::ModifiersChanged(modifiers) => {
                self.modifiers = modifiers.state();
            },
            WindowEvent::MouseInput { state: mouse, button, .. } => match (button, mouse) {
                (MouseButton::Left, ElementState::Pressed) => {
                    self.dragging = true;
                    self.press_at = self.last_cursor;
                    self.moved = false;
                },
                (MouseButton::Left, ElementState::Released) => {
                    self.dragging = false;
                    // Only a real press-release pair is a click; a
                    // stray release (focus change) must not pick.
                    let pressed_at = self.press_at.take();
                    let click = !self.moved && pressed_at.is_some();
                    self.moved = false;
                    if click
                        && let Some((x, y)) = pressed_at.or(self.last_cursor)
                    {
                        self.do_pick(x, y);
                    }
                },
                (MouseButton::Middle, ElementState::Pressed) => {
                    self.mmb = Some(if self.modifiers.shift_key() {
                        MiddleDrag::Pan
                    } else {
                        MiddleDrag::Orbit
                    });
                },
                (MouseButton::Middle, ElementState::Released) => self.mmb = None,
                _ => {},
            },
            WindowEvent::CursorMoved { position, .. } => {
                if let Some(last) = self.last_cursor {
                    let (dx, dy) = (position.x - last.0, position.y - last.1);
                    if self.dragging && dx.abs() + dy.abs() > 2.0 {
                        self.moved = true;
                    }
                    if let Some(state) = self.state.as_mut()
                        && (self.dragging || self.mmb.is_some())
                    {
                        match self.mmb {
                            Some(MiddleDrag::Pan) => state.camera.pan(dx, dy, state.size),
                            _ => state.camera.orbit(dx, dy),
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
                if event.state != ElementState::Pressed {
                    return;
                }
                match event.physical_key {
                    PhysicalKey::Code(code) => {
                        if let Some(slot) = digit_slot(code) {
                            self.select_document_index(slot);
                            return;
                        }
                        match code {
                            KeyCode::KeyR => self.reload_scene(),
                            KeyCode::Tab | KeyCode::BracketRight => self.cycle_document(1),
                            KeyCode::BracketLeft => self.cycle_document(-1),
                            KeyCode::KeyL => {
                                if let Some(state) = self.state.as_mut() {
                                    state.labels_on = !state.labels_on;
                                    eprintln!(
                                        "labels: {}",
                                        if state.labels_on { "on" } else { "off" }
                                    );
                                }
                            },
                            KeyCode::KeyC => self.copy_selector("primary"),
                            KeyCode::KeyF => self.copy_selector("face"),
                            KeyCode::KeyE => self.copy_selector("edge"),
                            KeyCode::Escape => {
                                if let Some(state) = self.state.as_mut() {
                                    state.pick = PickState::None;
                                }
                            },
                            _ => {},
                        }
                    },
                    _ => {},
                }
            },
            _ => {},
        }
    }

    fn about_to_wait(&mut self, _event_loop: &ActiveEventLoop) {
        // Drain pending scene_changed events and reload once. Only changes
        // to the document on screen (or the empty-list case) matter, so a
        // busy neighbour document does not make this viewer re-fetch.
        let mut reload = false;
        if let Some(rx) = &self.reload_rx {
            while let Ok(event) = rx.try_recv() {
                reload |= match event {
                    SceneEvent::Refresh | SceneEvent::Changed(None) => true,
                    SceneEvent::Changed(Some(doc)) => {
                        self.doc.is_none() || self.doc == Some(doc)
                    },
                };
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
    let mut doc: Option<DocId> = None;
    let mut args = std::env::args().skip(1);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--host" => host = args.next().unwrap_or(host),
            "--port" => port = args.next().and_then(|v| v.parse().ok()).unwrap_or(port),
            "--doc" => match args.next().and_then(|v| v.parse::<DocId>().ok()) {
                Some(id) => doc = Some(id),
                None => {
                    eprintln!("--doc needs a numeric document id");
                    return Err("invalid --doc".into());
                },
            },
            other => {
                eprintln!("usage: wright-view [--host HOST] [--port PORT] [--doc ID]");
                if other != "--help" && other != "-h" {
                    return Err(format!("unknown argument: {other}").into());
                }
                return Ok(());
            },
        }
    }

    eprintln!("wright-view | scene source: tcp://{host}:{port}");
    let event_loop = EventLoop::new()?;
    let mut app = App::new(host, port, doc);
    event_loop.run_app(&mut app)?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use wright_protocol::{Bbox, EdgeSelector, FaceSelector, Mesh};

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
    fn gizmo_axes_fit_inside_the_ortho_view() {        // Flattened column-major: m[col * 4 + row].
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

    fn box_snapshot(id: u64, name: &str) -> EntitySnapshot {
        EntitySnapshot {
            id,
            name: name.to_string(),
            translation: [0.0; 3],
            rotation_deg: [0.0; 3],
            scale: [1.0; 3],
            transform: cam::identity(),
            world_bbox: Bbox { min: [-5.0, -5.0, 0.0], max: [5.0, 5.0, 10.0] },
            color: [0.6, 0.7, 0.8, 1.0],
            mesh: Mesh {
                positions: vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]],
                normals: vec![[0.0, 0.0, 1.0]; 3],
                indices: vec![0, 1, 2],
            },
            sketch: None,
        }
    }

    fn hit() -> PickHit {
        PickHit {
            entity: 2,
            name: "boss".into(),
            distance: 90.0,
            point: [0.0, 0.0, 10.0],
            normal: [0.0, 0.0, 1.0],
            face: FaceSelector { entity: 2, normal: [0.0, 0.0, 1.0], near: [0.0, 0.0, 10.0] },
            edge: Some(EdgeSelector {
                direction: [0.0, 1.0, 0.0],
                near: [5.0, 0.0, 10.0],
                near_radius: 5.5,
            }),
        }
    }

    #[test]
    fn labels_are_toggleable_and_include_the_entity_id() {
        let cam = OrbitCamera::new();
        let size = (800, 600);
        let entities = [box_snapshot(2, "boss")];
        let docs = [DocInfo { doc: 2, name: "bracket".into(), entity_count: 1 }];
        let on = build_overlay(&entities, &cam, size, true, &PickState::None, &docs, Some(2));
        let off = build_overlay(&entities, &cam, size, false, &PickState::None, &docs, Some(2));
        assert!(!on.is_empty());
        // Labels draw a dot plus 8 glyphs ("boss #2" = 7 chars, actually
        // seven), so hiding them must shrink the batch. The HUD hint stays.
        assert!(on.idx.len() > off.idx.len(), "labels should add geometry");
    }

    #[test]
    fn a_pick_hud_carries_copy_ready_json() {
        let mut pick = PickState::Hit(Box::new(hit()));
        let primary = primary_selector(&pick).unwrap();
        assert!(primary.contains("\"direction\"") && primary.contains("\"near_radius\""));
        assert!(face_selector(&pick).unwrap().contains("\"entity\":2"));
        assert!(edge_selector(&pick).unwrap().contains("\"direction\""));
        match &mut pick {
            PickState::Hit(h) => h.edge = None,
            _ => unreachable!(),
        }
        // Without an edge the primary selector falls back to the face.
        assert!(primary_selector(&pick).unwrap().contains("\"entity\":2"));
        assert!(edge_selector(&pick).is_none());
    }

    #[test]
    fn the_hud_wraps_long_selectors() {
        let parts = wrap_line(&"x".repeat(100), 30);
        assert_eq!(parts.len(), 4);
        assert!(parts.iter().all(|p| p.chars().count() <= 30));
        assert_eq!(wrap_line("short", 30), vec!["short".to_string()]);
    }

    #[test]
    fn the_atlas_has_glyphs_and_a_white_cell() {
        let atlas = build_font_atlas();
        assert_eq!(atlas.len(), (ATLAS_W * ATLAS_H) as usize);
        let at = |cell: u32| {
            let (cx, cy) = ((cell % FONT_COLS) * 8, (cell / FONT_COLS) * 8);
            (0..8).flat_map(move |y| (0..8).map(move |x| (cx + x, cy + y)))
                .filter(|&(x, y)| atlas[(y * ATLAS_W + x) as usize] > 0)
                .count()
        };
        assert!(at('A' as u32 - 0x20) > 0, "A must have pixels");
        assert_eq!(at(WHITE_CELL), 64);
        // Unknown characters fall back to '?'.
        assert_eq!(glyph_cell('\u{2603}'), glyph_cell('?'));
    }

    #[test]
    fn the_click_ray_passes_through_the_screen_centre() {
        let cam = OrbitCamera::new();
        let (ray, tol) = cam.ray(400.0, 300.0, (800, 600));
        let eye = cam.eye();
        let want = cam::normalize([
            cam.target[0] - eye[0],
            cam.target[1] - eye[1],
            cam.target[2] - eye[2],
        ]);
        for i in 0..3 {
            assert!((ray.direction[i] as f32 - want[i]).abs() < 1e-5, "{:?}", ray.direction);
        }
        assert!(tol > 0.0);
        // Off-centre picks stay inside the frustum and differ from forward.
        let (edge, _) = cam.ray(0.0, 0.0, (800, 600));
        assert!((edge.direction[0] - ray.direction[0]).abs() > 1e-3);
    }

    #[test]
    fn panning_slides_the_target_in_the_screen_plane() {
        let mut cam = OrbitCamera::new();
        let before = cam.target;
        let eye = cam.eye();
        let forward = cam::normalize([
            before[0] - eye[0],
            before[1] - eye[1],
            before[2] - eye[2],
        ]);
        let right = cam::normalize(cam::cross(forward, [0.0, 0.0, 1.0]));
        let up = cam::cross(right, forward);
        let size = (800, 400);
        let (dx, dy) = (10.0, -6.0);
        cam.pan(dx, dy, size);

        let delta =
            [cam.target[0] - before[0], cam.target[1] - before[1], cam.target[2] - before[2]];
        let world_per_px = 2.0 * cam.dist * (FOV_Y_DEG.to_radians() * 0.5).tan() / size.1 as f32;
        // The world follows the cursor: drag right/down moves the target
        // left/up by exactly the on-screen distance at the target plane.
        assert!((cam::dot(delta, right) + dx as f32 * world_per_px).abs() < 1e-3);
        assert!((cam::dot(delta, up) - dy as f32 * world_per_px).abs() < 1e-3);
        assert!(cam::dot(delta, forward).abs() < 1e-3);
        // Panning neither zooms nor re-aims the camera.
        assert_eq!(cam.dist, 250.0);
        assert_eq!(cam.yaw, OrbitCamera::new().yaw);
        assert_eq!(cam.pitch, OrbitCamera::new().pitch);
    }

    fn doc_info(doc: DocId, name: &str) -> DocInfo {
        DocInfo { doc, name: name.to_string(), entity_count: 0 }
    }

    #[test]
    fn selection_prefers_a_live_choice_and_falls_back_to_the_first() {
        let docs = [doc_info(1, "bracket"), doc_info(2, "jig")];
        assert_eq!(net::select_document(&docs, Some(2)), Some(2));
        assert_eq!(net::select_document(&docs, Some(9)), Some(1));
        assert_eq!(net::select_document(&docs, None), Some(1));
        assert_eq!(net::select_document(&[], Some(2)), None);
        assert_eq!(net::select_document(&[], None), None);
    }

    #[test]
    fn scene_notifications_carry_the_document_id() {
        let with_doc = r#"{"jsonrpc":"2.0","method":"scene_changed","params":{"doc":2}}"#;
        assert_eq!(parse_scene_event(with_doc), Some(SceneEvent::Changed(Some(2))));
        // Document-agnostic notifications still trigger a re-list.
        let no_doc = r#"{"jsonrpc":"2.0","method":"scene_changed","params":{}}"#;
        assert_eq!(parse_scene_event(no_doc), Some(SceneEvent::Changed(None)));
        // Responses and junk are not events.
        assert_eq!(parse_scene_event(r#"{"jsonrpc":"2.0","id":1,"result":{}}"#), None);
        assert_eq!(parse_scene_event("not json"), None);
    }

    #[test]
    fn the_hud_names_the_selected_document_and_counts_documents() {
        let docs = [doc_info(1, "bracket"), doc_info(2, "jig")];
        let line = document_line(&docs, Some(2));
        assert!(line.contains("doc 2/2"), "{line}");
        assert!(line.contains("jig"), "{line}");
        assert!(line.contains("Tab"), "{line}");
        // One document needs no picker hint.
        let single = document_line(&docs[..1], Some(1));
        assert!(single.contains("doc 1/1"), "{single}");
        assert!(!single.contains("Tab"), "{single}");
        assert!(document_line(&[], None).contains("no open documents"));
    }

    #[test]
    fn number_keys_map_to_document_slots() {
        assert_eq!(digit_slot(KeyCode::Digit1), Some(0));
        assert_eq!(digit_slot(KeyCode::Digit9), Some(8));
        assert_eq!(digit_slot(KeyCode::KeyR), None);
    }

    /// A backend restart closes the notification stream. The watcher must
    /// reconnect on its own (and nudge a re-fetch), not go deaf until the
    /// viewer is restarted.
    #[test]
    fn live_listener_reconnects_after_the_server_restarts() {
        let listener = std::net::TcpListener::bind("127.0.0.1:0").expect("bind fake backend");
        let port = listener.local_addr().expect("local addr").port();

        let (accepted_tx, accepted_rx) = mpsc::channel();
        std::thread::spawn(move || {
            // Accept, then drop the stream (a restart), then accept again.
            for i in 0..2 {
                let (stream, _) = listener.accept().expect("accept");
                let _ = accepted_tx.send(i);
                std::thread::sleep(Duration::from_millis(50));
                drop(stream);
            }
        });

        let mut app = App::new("127.0.0.1".to_string(), port, None);
        app.start_live_listener();
        let rx = app.reload_rx.as_ref().expect("reload channel");
        // One tick when the watcher first connects, one after reconnecting.
        rx.recv_timeout(Duration::from_secs(2)).expect("tick on first connect");
        rx.recv_timeout(Duration::from_secs(5)).expect("tick after reconnect");
        accepted_rx.recv_timeout(Duration::from_secs(2)).expect("both connections accepted");
    }
}
