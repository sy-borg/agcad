//! Deterministic software rasterizer for headless PNG previews.
//!
//! The agent path never touches a GPU: same scene + same camera →
//! bit-identical PNG, whether it runs on a desktop, in SSH, or in CI.
//! Flat-shaded with a fixed key light plus a small camera headlight so
//! silhouettes read clearly at preview resolutions. A second pass draws the
//! feature edges where faces meet, which is what makes an orthographic front
//! view read like a CAD drawing instead of a flat silhouette.
//!
//! Pipeline per triangle: world transform → backface test (CCW outward
//! winding) → near-plane clip in clip space → perspective-correct depth
//! (z/w is linear in screen space) → z-buffered barycentric fill. Edges are
//! extracted from welded topology by dihedral angle, clipped against the
//! near plane, and drawn as anti-aliased one-pixel lines with a depth test so
//! hidden edges stay hidden.

use std::collections::BTreeMap;

use wright_protocol::{Mat4, Mesh, Rgba};

use crate::mat4;
use crate::Error;

/// One drawable: a local mesh, its world transform, and its color.
pub struct RenderItem<'a> {
    pub mesh: &'a Mesh,
    pub model: Mat4,
    pub color: Rgba,
}

/// Perspective look-at camera.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Camera {
    pub eye: [f64; 3],
    pub target: [f64; 3],
    pub up: [f64; 3],
    pub fov_y_deg: f64,
    /// Parallel projection. `fov_y_deg` then controls the framed height at
    /// the target distance, so switching a perspective camera to ortho keeps
    /// the same framing.
    pub ortho: bool,
}

/// Fixed view direction for auto-framing: a pleasant 3/4 CAD view
/// (front-right, slightly above), Z-up.
const AUTO_DIR: [f64; 3] = [0.9, -1.0, 0.55];

impl Camera {
    /// Frame the whole scene: fit its bounding sphere in the vertical
    /// frustum with a little margin.
    pub fn auto(items: &[RenderItem<'_>], width: u32, height: u32) -> Camera {
        let fov: f64 = 40.0;
        let (center, radius) = scene_bounds(items);
        let dir = normalize(AUTO_DIR);
        // Distance that fits the sphere: r / sin(fov/2), with margin.
        let dist = (radius / (fov * 0.5).to_radians().sin()) * 1.08;
        let eye = [
            center[0] + dir[0] * dist,
            center[1] + dir[1] * dist,
            center[2] + dir[2] * dist,
        ];
        let _ = (width, height); // aspect handled by the projection
        Camera { eye, target: center, up: [0.0, 0.0, 1.0], fov_y_deg: fov, ortho: false }
    }
}

/// Bounding sphere `(center, radius)` of all world-space geometry.
/// Empty scenes get a sensible default frame.
pub fn scene_bounds(items: &[RenderItem<'_>]) -> ([f64; 3], f64) {
    let mut min = [f64::MAX; 3];
    let mut max = [f64::MIN; 3];
    let mut any = false;
    for item in items {
        for p in &item.mesh.positions {
            let w = mat4::transform_point(&item.model, *p);
            for i in 0..3 {
                let v = w[i] as f64;
                min[i] = min[i].min(v);
                max[i] = max[i].max(v);
            }
            any = true;
        }
    }
    if !any {
        return ([0.0; 3], 10.0);
    }
    let center = [(min[0] + max[0]) * 0.5, (min[1] + max[1]) * 0.5, (min[2] + max[2]) * 0.5];
    let radius = (0..3)
        .map(|i| (max[i] - min[i]) * 0.5)
        .map(|h| h * h)
        .sum::<f64>()
        .sqrt()
        .max(1.0);
    (center, radius)
}

/// Render the items and encode a PNG. Returns `(png_bytes, width, height)`.
pub fn render_png(
    items: &[RenderItem<'_>],
    camera: Camera,
    width: u32,
    height: u32,
    background: Rgba,
) -> Result<(Vec<u8>, u32, u32), Error> {
    let rgb = rasterize(items, camera, width, height, background);
    let png = encode_png(&rgb, width, height)?;
    Ok((png, width, height))
}

/// Base64 (standard alphabet, padded) — keeps image payloads on the bus
/// without pulling in a dependency.
pub fn base64_encode(data: &[u8]) -> String {
    const B64: &[u8; 64] = b"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    let mut out = String::with_capacity(data.len().div_ceil(3) * 4);
    for chunk in data.chunks(3) {
        let b1 = chunk[0] as u32;
        let b2 = *chunk.get(1).unwrap_or(&0) as u32;
        let b3 = *chunk.get(2).unwrap_or(&0) as u32;
        let n = (b1 << 16) | (b2 << 8) | b3;
        out.push(B64[(n >> 18) as usize & 63] as char);
        out.push(B64[(n >> 12) as usize & 63] as char);
        out.push(if chunk.len() > 1 { B64[(n >> 6) as usize & 63] as char } else { '=' });
        out.push(if chunk.len() > 2 { B64[n as usize & 63] as char } else { '=' });
    }
    out
}

// --- rasterizer ---------------------------------------------------------------

const NEAR_EPS: f64 = 1e-6;

/// Key light direction (world space, pointing *from* the surface *to* the
/// light). Fixed so renders are comparable across calls.
const KEY_LIGHT: [f64; 3] = [0.35, -0.45, 0.83];

/// Faces must turn by at least this much for their shared edge to count as a
/// feature edge. Below it, the edge is a tessellation seam (a 32-segment
/// cylinder turns 11.25° per facet); at 90° it is a box corner. Matches the
/// engine's default fillet/pick threshold.
const EDGE_MIN_ANGLE_DEG: f64 = 30.0;

/// Screen-space depth slack for an edge on the surface it outlines. The
/// interpolation is exact for a shared edge, so this only absorbs rounding.
const EDGE_DEPTH_SLACK: f64 = 1e-6;

/// Color for the edge pass: dark ink on a light canvas, light ink on a dark
/// one, so the same renderer works for drawing-style and inspection-style
/// backgrounds.
fn edge_ink(background: Rgba) -> [u8; 3] {
    let luma = 0.2126 * background[0] as f64
        + 0.7152 * background[1] as f64
        + 0.0722 * background[2] as f64;
    if luma > 0.5 {
        [26, 29, 34]
    } else {
        [210, 216, 224]
    }
}

/// The shared projection used by both the shaded render and the silhouette
/// mask, so comparison samples the same pixels the picture shows.
fn view_projection(items: &[RenderItem<'_>], camera: Camera, width: u32, height: u32) -> Mat4 {
    let aspect = width as f64 / height.max(1) as f64;
    let (center, radius) = scene_bounds(items);
    let eye_dist = norm([camera.eye[0] - center[0], camera.eye[1] - center[1], camera.eye[2] - center[2]]);
    let near = ((eye_dist - radius) * 0.5).max(radius * 1e-3);
    let far = (eye_dist + radius * 2.0) * 2.0;

    let view = mat4::look_at(camera.eye, camera.target, camera.up);
    let proj = if camera.ortho {
        // Match the perspective framing at the target plane, then project
        // with parallel rays: same image, no size change with depth.
        let half_h = eye_dist * (camera.fov_y_deg.to_radians() * 0.5).tan();
        mat4::orthographic(half_h * aspect, half_h, near, far)
    } else {
        mat4::perspective(camera.fov_y_deg, aspect, near, far)
    };
    mat4::mul(&proj, &view)
}

/// Physical size of one screen pixel for an orthographic camera, using the
/// same framing math as [`view_projection`]. `None` for a perspective camera,
/// where the scale varies with depth.
pub fn ortho_mm_per_px(camera: Camera, items: &[RenderItem<'_>], height: u32) -> Option<f64> {
    if !camera.ortho {
        return None;
    }
    let (center, _) = scene_bounds(items);
    let eye_dist = norm([camera.eye[0] - center[0], camera.eye[1] - center[1], camera.eye[2] - center[2]]);
    let half_h = eye_dist * (camera.fov_y_deg.to_radians() * 0.5).tan();
    Some((2.0 * half_h / height.max(1) as f64).abs())
}

/// Rasterize a binary silhouette with the shaded renderer's camera and depth
/// buffer. Used by `compare_render`: an exact mask is more robust than
/// thresholding the shaded PNG, whose colors the document controls.
pub fn render_mask(
    items: &[RenderItem<'_>],
    camera: Camera,
    width: u32,
    height: u32,
) -> Vec<bool> {
    let (w, h) = (width as usize, height as usize);
    let vp = view_projection(items, camera, width, height);
    let mut mask = vec![false; w * h];
    let mut depth = vec![f64::INFINITY; w * h];

    for item in items {
        let world: Vec<[f64; 3]> = item
            .mesh
            .positions
            .iter()
            .map(|&p| mat4::transform_point(&item.model, p).map(|v| v as f64))
            .collect();

        for tri in item.mesh.indices.chunks_exact(3) {
            let (i0, i1, i2) = (tri[0] as usize, tri[1] as usize, tri[2] as usize);
            if i0 >= world.len() || i1 >= world.len() || i2 >= world.len() {
                continue;
            }
            let (p0, p1, p2) = (world[i0], world[i1], world[i2]);
            let n = normalize(cross3(sub(p1, p0), sub(p2, p0)));
            let view_dir = normalize(sub(camera.eye, p0));
            if n[0] * n[0] + n[1] * n[1] + n[2] * n[2] > 0.5 && dot(n, view_dir) <= 0.0 {
                continue;
            }
            let clip: Vec<[f64; 4]> = [p0, p1, p2].iter().map(|&p| mat4::transform_h(&vp, p)).collect();
            let clipped = clip_near(&clip);
            if clipped.len() < 3 {
                continue;
            }
            let screen: Vec<([f64; 2], f64)> = clipped
                .iter()
                .map(|c| {
                    let inv_w = 1.0 / c[3];
                    (
                        [
                            (c[0] * inv_w * 0.5 + 0.5) * width as f64,
                            (0.5 - c[1] * inv_w * 0.5) * height as f64,
                        ],
                        c[2] * inv_w,
                    )
                })
                .collect();
            for k in 1..screen.len().saturating_sub(1) {
                raster_tri_mask(
                    &screen[0], &screen[k], &screen[k + 1], &mut mask, &mut depth, w, h,
                );
            }
        }
    }
    mask
}

fn rasterize(items: &[RenderItem<'_>], camera: Camera, width: u32, height: u32, background: Rgba) -> Vec<u8> {
    let (w, h) = (width as usize, height as usize);
    let vp = view_projection(items, camera, width, height);

    let mut color = vec![0u8; w * h * 3];
    let bg: [u8; 3] = [0, 1, 2].map(|i| (background[i].clamp(0.0, 1.0) * 255.0) as u8);
    for px in color.chunks_exact_mut(3) {
        px.copy_from_slice(&bg);
    }
    let mut depth = vec![f64::INFINITY; w * h];

    let key = normalize(KEY_LIGHT);
    // Headlight direction is constant per frame. Using the vector to each
    // triangle's first vertex instead makes large flat faces shade in visible
    // facets (each triangle gets a slightly different headlight term).
    let head_dir = normalize(sub(camera.eye, camera.target));

    for item in items {
        // World-space positions (f32 math, matching the viewer convention).
        let world: Vec<[f64; 3]> = item
            .mesh
            .positions
            .iter()
            .map(|&p| mat4::transform_point(&item.model, p).map(|v| v as f64))
            .collect();

        for tri in item.mesh.indices.chunks_exact(3) {
            let (i0, i1, i2) = (tri[0] as usize, tri[1] as usize, tri[2] as usize);
            let (p0, p1, p2) = (world[i0], world[i1], world[i2]);

            // Face normal + backface cull (CCW-from-outside winding).
            let n = normalize(cross3(sub(p1, p0), sub(p2, p0)));
            let view_dir = normalize(sub(camera.eye, p0));
            if n[0] * n[0] + n[1] * n[1] + n[2] * n[2] > 0.5 && dot(n, view_dir) <= 0.0 {
                continue;
            }

            // Flat shading: ambient + key light + camera headlight.
            let lam = dot(n, key).max(0.0);
            let head = dot(n, head_dir).max(0.0);
            let intensity = (0.30 + 0.62 * lam + 0.18 * head).min(1.0);
            let tri_color: [u8; 3] =
                [0, 1, 2].map(|i| ((item.color[i] as f64 * intensity).clamp(0.0, 1.0) * 255.0) as u8);

            // Clip space.
            let clip: Vec<[f64; 4]> = [p0, p1, p2].iter().map(|&p| mat4::transform_h(&vp, p)).collect();

            let clipped = clip_near(&clip);
            if clipped.len() < 3 {
                continue;
            }

            // Project: NDC → screen (y down); z/w is the depth varying.
            let screen: Vec<([f64; 2], f64)> = clipped
                .iter()
                .map(|c| {
                    let inv_w = 1.0 / c[3];
                    (
                        [
                            (c[0] * inv_w * 0.5 + 0.5) * width as f64,
                            (0.5 - c[1] * inv_w * 0.5) * height as f64,
                        ],
                        c[2] * inv_w,
                    )
                })
                .collect();

            // Fan-triangulate the clipped polygon.
            for k in 1..screen.len().saturating_sub(1) {
                raster_tri(&screen[0], &screen[k], &screen[k + 1], tri_color, &mut color, &mut depth, w, h);
            }
        }
    }

    // Shaded-with-edges: after every face has filled the depth buffer, ink
    // the feature edges. The depth test keeps edges on the far side hidden,
    // and the one-pixel anti-aliased line reads as a crisp CAD outline.
    let ink = edge_ink(background);
    for item in items {
        let world: Vec<[f64; 3]> = item
            .mesh
            .positions
            .iter()
            .map(|&p| mat4::transform_point(&item.model, p).map(|v| v as f64))
            .collect();
        for (a, b) in display_edges(item.mesh, &world) {
            let ca = mat4::transform_h(&vp, a);
            let cb = mat4::transform_h(&vp, b);
            let Some((ca, cb)) = clip_segment_near(ca, cb) else { continue };
            let pa = screen_point(ca, width, height);
            let pb = screen_point(cb, width, height);
            raster_line(&pa, &pb, ink, &mut color, &mut depth, w, h);
        }
    }
    color
}

/// Feature edges of one mesh in mesh/world order: shared by exactly two
/// non-degenerate faces whose normals differ by at least
/// [`EDGE_MIN_ANGLE_DEG`]. Boundary and non-manifold edges are skipped, so
/// open display ribbons (sketches) contribute nothing.
fn display_edges(mesh: &Mesh, world: &[[f64; 3]]) -> Vec<([f64; 3], [f64; 3])> {
    // Weld by exact f32 bits, the same rule as the engine's sharp-edge
    // extraction: crease normals duplicate positions, but the bits match.
    let mut ids: BTreeMap<[u32; 3], u32> = BTreeMap::new();
    let mut rep: Vec<u32> = Vec::new();
    let mut remap: Vec<u32> = Vec::with_capacity(mesh.positions.len());
    for (i, p) in mesh.positions.iter().enumerate() {
        let key = [p[0].to_bits(), p[1].to_bits(), p[2].to_bits()];
        let id = match ids.get(&key) {
            Some(&id) => id,
            None => {
                let id = rep.len() as u32;
                ids.insert(key, id);
                rep.push(i as u32);
                id
            },
        };
        remap.push(id);
    }

    struct EdgeRec {
        count: u32,
        normals: [[f64; 3]; 2],
        n: u8,
    }
    let mut edges: BTreeMap<(u32, u32), EdgeRec> = BTreeMap::new();
    for tri in mesh.indices.chunks_exact(3) {
        let (Some(&i0), Some(&i1), Some(&i2)) = (
            remap.get(tri[0] as usize),
            remap.get(tri[1] as usize),
            remap.get(tri[2] as usize),
        ) else {
            continue;
        };
        let p0 = world[rep[i0 as usize] as usize];
        let p1 = world[rep[i1 as usize] as usize];
        let p2 = world[rep[i2 as usize] as usize];
        let n = normalize(cross3(sub(p1, p0), sub(p2, p0)));
        let degenerate = dot(n, n) < 0.5;
        for (a, b) in [(i0, i1), (i1, i2), (i2, i0)] {
            let key = if a < b { (a, b) } else { (b, a) };
            let rec = edges.entry(key).or_insert(EdgeRec {
                count: 0,
                normals: [[0.0; 3]; 2],
                n: 0,
            });
            rec.count += 1;
            if !degenerate && rec.n < 2 {
                rec.normals[rec.n as usize] = n;
                rec.n += 1;
            }
        }
    }

    let min_cos = EDGE_MIN_ANGLE_DEG.to_radians().cos();
    let mut out = Vec::new();
    for ((a, b), rec) in edges {
        if rec.count == 2
            && rec.n == 2
            && dot(rec.normals[0], rec.normals[1]) < min_cos
        {
            out.push((
                world[rep[a as usize] as usize],
                world[rep[b as usize] as usize],
            ));
        }
    }
    out
}

/// Clip a line segment against the near plane in clip space.
fn clip_segment_near(a: [f64; 4], b: [f64; 4]) -> Option<([f64; 4], [f64; 4])> {
    let (da, db) = (a[3] - NEAR_EPS, b[3] - NEAR_EPS);
    if da < 0.0 && db < 0.0 {
        return None;
    }
    if da >= 0.0 && db >= 0.0 {
        return Some((a, b));
    }
    let t = da / (da - db);
    let cut = [
        a[0] + (b[0] - a[0]) * t,
        a[1] + (b[1] - a[1]) * t,
        a[2] + (b[2] - a[2]) * t,
        a[3] + (b[3] - a[3]) * t,
    ];
    Some(if da >= 0.0 { (a, cut) } else { (cut, b) })
}

fn screen_point(c: [f64; 4], width: u32, height: u32) -> ([f64; 2], f64) {
    let inv_w = 1.0 / c[3];
    (
        [
            (c[0] * inv_w * 0.5 + 0.5) * width as f64,
            (0.5 - c[1] * inv_w * 0.5) * height as f64,
        ],
        c[2] * inv_w,
    )
}

/// Anti-aliased one-pixel line (Wu-style coverage along the major axis) with
/// the same depth buffer as the faces. Depth varies linearly in screen space,
/// so the line's depth can be interpolated straight from the endpoints.
fn raster_line(
    a: &([f64; 2], f64),
    b: &([f64; 2], f64),
    color: [u8; 3],
    buf: &mut [u8],
    depth: &mut [f64],
    w: usize,
    h: usize,
) {
    let (mut x0, mut y0, mut z0) = (a.0[0], a.0[1], a.1);
    let (mut x1, mut y1, mut z1) = (b.0[0], b.0[1], b.1);
    let dx = x1 - x0;
    let dy = y1 - y0;
    if dx.abs() < 1e-12 && dy.abs() < 1e-12 {
        return;
    }

    // Walk the major axis in pixel-column/row steps so each step lightens
    // the two neighbouring pixels. Clamping the walk to the viewport keeps a
    // segment that leaves the screen cheap.
    let x_major = dx.abs() >= dy.abs();
    if x_major {
        if x0 > x1 {
            std::mem::swap(&mut x0, &mut x1);
            std::mem::swap(&mut y0, &mut y1);
            std::mem::swap(&mut z0, &mut z1);
        }
        let span = x1 - x0;
        let start = ((x0 - 0.5).round() as i64).max(-1);
        let end = ((x1 - 0.5).round() as i64).min(w as i64);
        for ix in start..=end {
            let px = ix as f64 + 0.5;
            let t = ((px - x0) / span).clamp(0.0, 1.0);
            let py = y0 + (y1 - y0) * t;
            let pz = z0 + (z1 - z0) * t;
            let cy = py - 0.5;
            let iy = cy.floor();
            let fy = cy - iy;
            plot_edge(ix, iy as i64, 1.0 - fy, pz, color, buf, depth, w, h);
            plot_edge(ix, iy as i64 + 1, fy, pz, color, buf, depth, w, h);
        }
    } else {
        if y0 > y1 {
            std::mem::swap(&mut x0, &mut x1);
            std::mem::swap(&mut y0, &mut y1);
            std::mem::swap(&mut z0, &mut z1);
        }
        let span = y1 - y0;
        let start = ((y0 - 0.5).round() as i64).max(-1);
        let end = ((y1 - 0.5).round() as i64).min(h as i64);
        for iy in start..=end {
            let py = iy as f64 + 0.5;
            let t = ((py - y0) / span).clamp(0.0, 1.0);
            let px = x0 + (x1 - x0) * t;
            let pz = z0 + (z1 - z0) * t;
            let cx = px - 0.5;
            let ix = cx.floor();
            let fx = cx - ix;
            plot_edge(ix as i64, iy, 1.0 - fx, pz, color, buf, depth, w, h);
            plot_edge(ix as i64 + 1, iy, fx, pz, color, buf, depth, w, h);
        }
    }
}

#[allow(clippy::too_many_arguments)]
fn plot_edge(
    x: i64,
    y: i64,
    coverage: f64,
    z: f64,
    color: [u8; 3],
    buf: &mut [u8],
    depth: &mut [f64],
    w: usize,
    h: usize,
) {
    if !(0..w as i64).contains(&x) || !(0..h as i64).contains(&y) || coverage <= 0.002 {
        return;
    }
    let di = y as usize * w + x as usize;
    if z - depth[di] > EDGE_DEPTH_SLACK {
        return;
    }
    let a = coverage.clamp(0.0, 1.0);
    for (c, &ink) in color.iter().enumerate() {
        let dst = buf[di * 3 + c] as f64;
        buf[di * 3 + c] = (dst + (ink as f64 - dst) * a).round().clamp(0.0, 255.0) as u8;
    }
}

/// Sutherland–Hodgman clip of a polygon against `w > eps` (the near plane
/// in clip space, before the divide).
fn clip_near(poly: &[[f64; 4]]) -> Vec<[f64; 4]> {
    let mut out = Vec::with_capacity(poly.len() + 1);
    for i in 0..poly.len() {
        let a = poly[i];
        let b = poly[(i + 1) % poly.len()];
        let da = a[3] - NEAR_EPS;
        let db = b[3] - NEAR_EPS;
        if da >= 0.0 {
            out.push(a);
        }
        if (da >= 0.0) != (db >= 0.0) {
            let t = da / (da - db);
            out.push([
                a[0] + (b[0] - a[0]) * t,
                a[1] + (b[1] - a[1]) * t,
                a[2] + (b[2] - a[2]) * t,
                a[3] + (b[3] - a[3]) * t,
            ]);
        }
    }
    out
}

#[allow(clippy::too_many_arguments)]
fn raster_tri(
    v0: &([f64; 2], f64),
    v1: &([f64; 2], f64),
    v2: &([f64; 2], f64),
    color: [u8; 3],
    buf: &mut [u8],
    depth: &mut [f64],
    w: usize,
    h: usize,
) {
    let (p0, z0) = *v0;
    let (p1, z1) = *v1;
    let (p2, z2) = *v2;
    let area2 = (p1[0] - p0[0]) * (p2[1] - p0[1]) - (p1[1] - p0[1]) * (p2[0] - p0[0]);
    if area2.abs() < 1e-12 {
        return;
    }
    let inv = 1.0 / area2;

    let min_x = p0[0].min(p1[0]).min(p2[0]).floor().max(0.0) as usize;
    let max_x = (p0[0].max(p1[0]).max(p2[0]).ceil().min(w as f64 - 1.0)).max(0.0) as usize;
    let min_y = p0[1].min(p1[1]).min(p2[1]).floor().max(0.0) as usize;
    let max_y = (p0[1].max(p1[1]).max(p2[1]).ceil().min(h as f64 - 1.0)).max(0.0) as usize;

    let edge = |a: [f64; 2], b: [f64; 2], p: [f64; 2]| {
        (b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0])
    };

    for y in min_y..=max_y {
        let py = y as f64 + 0.5;
        for x in min_x..=max_x {
            let p = [x as f64 + 0.5, py];
            // Barycentric via edge functions (sign-normalized by `inv`).
            let b0 = edge(p1, p2, p) * inv;
            let b1 = edge(p2, p0, p) * inv;
            let b2 = edge(p0, p1, p) * inv;
            // Tiny negative slack so shared edges don't leave cracks; the
            // depth test absorbs the (harmless) double-draw.
            if b0 < -1e-6 || b1 < -1e-6 || b2 < -1e-6 {
                continue;
            }
            // z/w is linear in screen space: perspective-correct depth.
            let z = b0 * z0 + b1 * z1 + b2 * z2;
            let di = y * w + x;
            if z < depth[di] {
                depth[di] = z;
                buf[di * 3..di * 3 + 3].copy_from_slice(&color);
            }
        }
    }
}

#[allow(clippy::too_many_arguments)]
fn raster_tri_mask(
    v0: &([f64; 2], f64),
    v1: &([f64; 2], f64),
    v2: &([f64; 2], f64),
    mask: &mut [bool],
    depth: &mut [f64],
    w: usize,
    h: usize,
) {
    let ((p0, z0), (p1, z1), (p2, z2)) = (*v0, *v1, *v2);
    let area2 = (p1[0] - p0[0]) * (p2[1] - p0[1]) - (p1[1] - p0[1]) * (p2[0] - p0[0]);
    if area2.abs() < 1e-12 {
        return;
    }
    let inv = 1.0 / area2;
    let min_x = p0[0].min(p1[0]).min(p2[0]).floor().max(0.0) as usize;
    let max_x = (p0[0].max(p1[0]).max(p2[0]).ceil().min(w as f64 - 1.0)).max(0.0) as usize;
    let min_y = p0[1].min(p1[1]).min(p2[1]).floor().max(0.0) as usize;
    let max_y = (p0[1].max(p1[1]).max(p2[1]).ceil().min(h as f64 - 1.0)).max(0.0) as usize;

    let edge = |a: [f64; 2], b: [f64; 2], p: [f64; 2]| {
        (b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0])
    };
    for y in min_y..=max_y {
        let py = y as f64 + 0.5;
        for x in min_x..=max_x {
            let p = [x as f64 + 0.5, py];
            let b0 = edge(p1, p2, p) * inv;
            let b1 = edge(p2, p0, p) * inv;
            let b2 = edge(p0, p1, p) * inv;
            if b0 < -1e-6 || b1 < -1e-6 || b2 < -1e-6 {
                continue;
            }
            let z = b0 * z0 + b1 * z1 + b2 * z2;
            let di = y * w + x;
            if z < depth[di] {
                depth[di] = z;
                mask[di] = true;
            }
        }
    }
}

fn encode_png(rgb: &[u8], width: u32, height: u32) -> Result<Vec<u8>, Error> {    let mut out = Vec::new();
    {
        let mut enc = png::Encoder::new(&mut out, width, height);
        enc.set_color(png::ColorType::Rgb);
        enc.set_depth(png::BitDepth::Eight);
        let mut writer = enc
            .write_header()
            .map_err(|e| Error::Invalid(format!("png header failed: {e}")))?;
        writer
            .write_image_data(rgb)
            .map_err(|e| Error::Invalid(format!("png encode failed: {e}")))?;
    }
    Ok(out)
}

// --- tiny vector helpers ------------------------------------------------------

fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn cross3(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn norm(a: [f64; 3]) -> f64 {
    (a[0] * a[0] + a[1] * a[1] + a[2] * a[2]).sqrt()
}

fn normalize(v: [f64; 3]) -> [f64; 3] {
    let len = norm(v);
    if len < 1e-12 { [0.0; 3] } else { [v[0] / len, v[1] / len, v[2] / len] }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A minimal two-triangle quad facing +Z (deterministic, hand-built).
    fn quad_mesh() -> Mesh {
        let positions = vec![[-1.0, -1.0, 0.0], [1.0, -1.0, 0.0], [1.0, 1.0, 0.0], [-1.0, 1.0, 0.0]];
        let normals = vec![[0.0, 0.0, 1.0]; 4];
        let indices = vec![0, 1, 2, 0, 2, 3];
        Mesh { positions, normals, indices }
    }

    #[test]
    fn base64_round_trip_shape() {
        assert_eq!(base64_encode(b""), "");
        assert_eq!(base64_encode(b"f"), "Zg==");
        assert_eq!(base64_encode(b"fo"), "Zm8=");
        assert_eq!(base64_encode(b"foo"), "Zm9v");
        assert_eq!(base64_encode(b"foob"), "Zm9vYg==");
        assert_eq!(base64_encode(b"fooba"), "Zm9vYmE=");
        assert_eq!(base64_encode(b"foobar"), "Zm9vYmFy");
    }

    #[test]
    fn quad_rasterizes_to_expected_pixels() {
        let mesh = quad_mesh();
        let items = [RenderItem {
            mesh: &mesh,
            model: mat4::identity(),
            color: [1.0, 0.0, 0.0, 1.0],
        }];
        let camera = Camera {
            eye: [0.0, 0.0, 10.0],
            target: [0.0, 0.0, 0.0],
            up: [0.0, 1.0, 0.0],
            fov_y_deg: 40.0,
            ortho: false,
        };
        let rgb = rasterize(&items, camera, 33, 33, [0.0, 0.0, 0.0, 1.0]);
        // Center pixel must be red-ish, corner must be background.
        let c = (16 * 33 + 16) * 3;
        assert!(rgb[c] > 200 && rgb[c + 1] < 60 && rgb[c + 2] < 60, "center not red: {:?}", &rgb[c..c + 3]);
        let corner = (0 * 33 + 0) * 3;
        assert_eq!(&rgb[corner..corner + 3], &[0, 0, 0], "corner not background");
    }

    #[test]
    fn near_clipping_produces_valid_polygon() {
        let poly = vec![
            [0.0, 0.0, 0.0, 5.0],
            [0.0, 0.0, 0.0, -1.0], // behind the near plane
            [1.0, 1.0, 0.0, 5.0],
        ];
        let clipped = clip_near(&poly);
        assert_eq!(clipped.len(), 4);
        for c in &clipped {
            assert!(c[3] > 0.0);
        }
    }

    #[test]
    fn a_box_has_twelve_display_edges() {
        let mesh = crate::geo::box_mesh([10.0, 10.0, 10.0]);
        let world: Vec<[f64; 3]> = mesh.positions.iter().map(|p| p.map(|v| v as f64)).collect();
        let edges = display_edges(&mesh, &world);
        assert_eq!(edges.len(), 12, "six faces → twelve feature edges");
    }

    #[test]
    fn coplanar_and_open_meshes_have_no_display_edges() {
        // A quad is open: all four edges are boundaries, not face joints.
        let quad = quad_mesh();
        let world: Vec<[f64; 3]> = quad.positions.iter().map(|p| p.map(|v| v as f64)).collect();
        assert!(display_edges(&quad, &world).is_empty());

        // Two coplanar triangles sharing an edge: a tessellation seam.
        let mut flat = quad_mesh();
        flat.normals.clear();
        assert!(display_edges(&flat, &world).is_empty());
    }

    #[test]
    fn cylindrical_seams_are_not_display_edges() {
        // 32 facets turn 11.25° per seam, below the 30° feature threshold, so
        // only the two cap rims survive.
        let mesh = crate::geo::cylinder_mesh(5.0, 10.0, 32);
        let world: Vec<[f64; 3]> = mesh.positions.iter().map(|p| p.map(|v| v as f64)).collect();
        let edges = display_edges(&mesh, &world);
        assert_eq!(edges.len(), 64, "two rims × 32 segments");
        assert!(edges.iter().all(|(a, b)| (a[2] - b[2]).abs() < 1e-6));
    }

    #[test]
    fn silhouette_mask_fills_the_box_and_reports_scale() {
        let mesh = crate::geo::box_mesh([10.0, 10.0, 10.0]);
        let items = [RenderItem {
            mesh: &mesh,
            model: mat4::identity(),
            color: [0.2, 0.2, 0.2, 1.0],
        }];
        let camera = Camera {
            eye: [0.0, -40.0, 0.0],
            target: [0.0, 0.0, 0.0],
            up: [0.0, 0.0, 1.0],
            fov_y_deg: 40.0,
            ortho: true,
        };
        let mask = render_mask(&items, camera, 64, 64);
        assert!(mask[32 * 64 + 32], "center should be covered");
        assert!(!mask[0], "corner should be background");
        let scale = ortho_mm_per_px(camera, &items, 64).unwrap();
        assert!(scale > 0.0);
    }

    #[test]
    fn edge_ink_contrasts_with_the_background() {
        assert!(edge_ink([1.0, 1.0, 1.0, 1.0])[0] < 64);
        assert!(edge_ink([0.0, 0.0, 0.0, 1.0])[0] > 192);
    }

    #[test]
    fn a_box_render_inks_its_feature_edges() {
        let mesh = crate::geo::box_mesh([10.0, 10.0, 10.0]);
        let items = [RenderItem {
            mesh: &mesh,
            model: mat4::identity(),
            color: [0.85, 0.90, 1.0, 1.0],
        }];
        let camera = Camera {
            eye: [30.0, -40.0, 35.0],
            target: [0.0, 0.0, 0.0],
            up: [0.0, 0.0, 1.0],
            fov_y_deg: 40.0,
            ortho: true,
        };
        let rgb = rasterize(&items, camera, 120, 120, [1.0, 1.0, 1.0, 1.0]);
        let dark = rgb
            .chunks_exact(3)
            .any(|p| p[0] < 50 && p[1] < 50 && p[2] < 50);
        assert!(dark, "the box should have inked feature edges");
    }
}
