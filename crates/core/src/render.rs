//! Deterministic software rasterizer for headless PNG previews.
//!
//! The agent path never touches a GPU: same scene + same camera →
//! bit-identical PNG, whether it runs on a desktop, in SSH, or in CI.
//! Flat-shaded with a fixed key light plus a small camera headlight so
//! silhouettes read clearly at preview resolutions.
//!
//! Pipeline per triangle: world transform → backface test (CCW outward
//! winding) → near-plane clip in clip space → perspective-correct depth
//! (z/w is linear in screen space) → z-buffered barycentric fill.

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

fn rasterize(items: &[RenderItem<'_>], camera: Camera, width: u32, height: u32, background: Rgba) -> Vec<u8> {
    let (w, h) = (width as usize, height as usize);
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
    let vp = mat4::mul(&proj, &view);

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
    color
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

fn encode_png(rgb: &[u8], width: u32, height: u32) -> Result<Vec<u8>, Error> {
    let mut out = Vec::new();
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
}
