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
