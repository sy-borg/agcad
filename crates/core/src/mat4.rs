//! Minimal column-major 4x4 matrix math. CAD convention: Z is up.
//!
//! Storage is `m[col][row]` (OpenGL/WGSL style); points transform as
//! `p' = M · p`.

use agcad_protocol::Mat4;

pub fn identity() -> Mat4 {
    [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]
}

/// `a * b` — applies `b` first.
pub fn mul(a: &Mat4, b: &Mat4) -> Mat4 {
    let mut m = [[0.0f32; 4]; 4];
    for (j, col) in m.iter_mut().enumerate() {
        for (i, v) in col.iter_mut().enumerate() {
            *v = (0..4).map(|k| a[k][i] * b[j][k]).sum();
        }
    }
    m
}

pub fn translation(t: [f64; 3]) -> Mat4 {
    let mut m = identity();
    m[3][0] = t[0] as f32;
    m[3][1] = t[1] as f32;
    m[3][2] = t[2] as f32;
    m
}

fn rot_x(deg: f64) -> Mat4 {
    let r = deg.to_radians();
    let (s, c) = (r.sin() as f32, r.cos() as f32);
    [[1.0, 0.0, 0.0, 0.0], [0.0, c, s, 0.0], [0.0, -s, c, 0.0], [0.0, 0.0, 0.0, 1.0]]
}

fn rot_y(deg: f64) -> Mat4 {
    let r = deg.to_radians();
    let (s, c) = (r.sin() as f32, r.cos() as f32);
    [[c, 0.0, -s, 0.0], [0.0, 1.0, 0.0, 0.0], [s, 0.0, c, 0.0], [0.0, 0.0, 0.0, 1.0]]
}

fn rot_z(deg: f64) -> Mat4 {
    let r = deg.to_radians();
    let (s, c) = (r.sin() as f32, r.cos() as f32);
    [[c, s, 0.0, 0.0], [-s, c, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]]
}

/// Extrinsic XYZ: rotate around X first, then Y, then Z.
/// `R = Rz · Ry · Rx`.
pub fn rotation_xyz_deg(deg: [f64; 3]) -> Mat4 {
    mul(&rot_z(deg[2]), &mul(&rot_y(deg[1]), &rot_x(deg[0])))
}

pub fn scale(s: [f64; 3]) -> Mat4 {
    let mut m = identity();
    m[0][0] = s[0] as f32;
    m[1][1] = s[1] as f32;
    m[2][2] = s[2] as f32;
    m
}

/// `T * Rz * Ry * Rx * S`.
pub fn compose(t: [f64; 3], rotation_deg: [f64; 3], s: [f64; 3]) -> Mat4 {
    mul(&translation(t), &mul(&rotation_xyz_deg(rotation_deg), &scale(s)))
}

/// Transform a point by a model matrix (w = 1, no perspective).
pub fn transform_point(m: &Mat4, p: [f32; 3]) -> [f32; 3] {
    [
        m[0][0] * p[0] + m[1][0] * p[1] + m[2][0] * p[2] + m[3][0],
        m[0][1] * p[0] + m[1][1] * p[1] + m[2][1] * p[2] + m[3][1],
        m[0][2] * p[0] + m[1][2] * p[1] + m[2][2] * p[2] + m[3][2],
    ]
}

/// Transform a homogeneous point (w = 1) by a matrix, returning the 4-vector.
pub fn transform_h(m: &Mat4, p: [f64; 3]) -> [f64; 4] {
    let (x, y, z) = (p[0] as f32, p[1] as f32, p[2] as f32);
    [
        (m[0][0] * x + m[1][0] * y + m[2][0] * z + m[3][0]) as f64,
        (m[0][1] * x + m[1][1] * y + m[2][1] * z + m[3][1]) as f64,
        (m[0][2] * x + m[1][2] * y + m[2][2] * z + m[3][2]) as f64,
        (m[0][3] * x + m[1][3] * y + m[2][3] * z + m[3][3]) as f64,
    ]
}

fn normalize(v: [f64; 3]) -> [f64; 3] {
    let len = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
    if len < 1e-12 { [0.0; 3] } else { [v[0] / len, v[1] / len, v[2] / len] }
}

fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

/// Right-handed look-at view matrix (OpenGL convention: camera looks down
/// -Z in view space). Falls back to alternate up vectors when `up` is
/// (anti)parallel to the view direction.
pub fn look_at(eye: [f64; 3], target: [f64; 3], up: [f64; 3]) -> Mat4 {
    let f = normalize([target[0] - eye[0], target[1] - eye[1], target[2] - eye[2]]);
    let mut up = up;
    // If `up` is (anti)parallel to the view direction, pick a fallback.
    for candidate in [up, [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]] {
        let s = normalize(cross(f, candidate));
        if s[0] * s[0] + s[1] * s[1] + s[2] * s[2] > 0.5 {
            up = candidate;
            break;
        }
    }
    let s = normalize(cross(f, up));
    let u = cross(s, f);

    let dot = |a: [f64; 3], b: [f64; 3]| a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    // Column-major; rows are (s, -s·eye), (u, -u·eye), (-f, f·eye), (0,0,0,1).
    [
        [s[0] as f32, u[0] as f32, -f[0] as f32, 0.0],
        [s[1] as f32, u[1] as f32, -f[1] as f32, 0.0],
        [s[2] as f32, u[2] as f32, -f[2] as f32, 0.0],
        [-dot(s, eye) as f32, -dot(u, eye) as f32, dot(f, eye) as f32, 1.0],
    ]
}

/// Parallel projection: the box `[-half_w, half_w] x [-half_h, half_h]`
/// (in view space) maps to the full frame, and depth maps to the same
/// `-1..1` range as [`perspective`] so the rasterizer's depth test is
/// unchanged. `w` stays 1, so no divide happens.
pub fn orthographic(half_w: f64, half_h: f64, near: f64, far: f64) -> Mat4 {
    let mut m = identity();
    m[0][0] = (1.0 / half_w.max(1e-9)) as f32;
    m[1][1] = (1.0 / half_h.max(1e-9)) as f32;
    // z: [-near, -far] -> [-1, 1]
    m[2][2] = (-2.0 / (far - near)) as f32;
    m[3][2] = (-(far + near) / (far - near)) as f32;
    m
}

/// Perspective projection, OpenGL convention (maps the -Z view frustum to
/// NDC z ∈ [-1, 1]).
pub fn perspective(fov_y_deg: f64, aspect: f64, near: f64, far: f64) -> Mat4 {
    let f = (1.0 / (fov_y_deg.to_radians() * 0.5).tan()) as f32;
    let (near, far) = (near as f32, far as f32);
    let mut m = identity();
    m[0][0] = f / (aspect.max(1e-6) as f32);
    m[1][1] = f;
    m[2][2] = (far + near) / (near - far);
    m[2][3] = -1.0;
    m[3][2] = 2.0 * far * near / (near - far);
    m
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn orthographic_keeps_size_independent_of_depth() {
        let m = orthographic(50.0, 40.0, 1.0, 100.0);
        let near = transform_h(&m, [10.0, 5.0, -1.0]);
        let far = transform_h(&m, [10.0, 5.0, -100.0]);
        // Parallel rays: the same x/y projects to the same NDC x/y at any depth.
        assert_eq!(near[3], 1.0, "ortho w must stay 1");
        assert_eq!(far[3], 1.0);
        assert!((near[0] - far[0]).abs() < 1e-6, "{near:?} {far:?}");
        assert!((near[0] - 0.2).abs() < 1e-6, "{}", near[0]);
        assert!((near[1] - far[1]).abs() < 1e-6 && (near[1] - 0.125).abs() < 1e-6);
        // Depth still orders front to back within -1..1.
        assert!((near[2] + 1.0).abs() < 1e-6, "{}", near[2]);
        assert!((far[2] - 1.0).abs() < 1e-6, "{}", far[2]);
    }

    #[test]
    fn perspective_changes_size_with_depth() {
        let m = perspective(40.0, 1.0, 1.0, 100.0);
        let near = transform_h(&m, [10.0, 5.0, -10.0]);
        let far = transform_h(&m, [10.0, 5.0, -50.0]);
        let near_x = near[0] / near[3];
        let far_x = far[0] / far[3];
        assert!(near_x.abs() > far_x.abs() * 3.0, "{near_x} {far_x}");
    }
}
