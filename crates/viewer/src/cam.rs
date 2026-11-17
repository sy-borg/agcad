//! Tiny column-major mat4 helpers for the viewer's camera.
//! (The core engine has its own copy in `agcad-core::mat4`; sharing a
//! crate would couple the viewer to the engine — not worth it for ~60
//! lines. If this grows, promote it to `agcad-math`.)

pub type Mat4 = [[f32; 4]; 4]; // m[col][row]

pub fn identity() -> Mat4 {
    [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]
}

pub fn mul(a: &Mat4, b: &Mat4) -> Mat4 {
    let mut m = [[0.0f32; 4]; 4];
    for (j, col) in m.iter_mut().enumerate() {
        for (i, v) in col.iter_mut().enumerate() {
            *v = (0..4).map(|k| a[k][i] * b[j][k]).sum();
        }
    }
    m
}

pub fn dot(a: [f32; 3], b: [f32; 3]) -> f32 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

pub fn cross(a: [f32; 3], b: [f32; 3]) -> [f32; 3] {
    [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}

pub fn normalize(v: [f32; 3]) -> [f32; 3] {
    let len = dot(v, v).sqrt();
    if len < 1e-6 {
        [0.0, 0.0, 0.0]
    } else {
        [v[0] / len, v[1] / len, v[2] / len]
    }
}

/// Right-handed `look_at`, Z-up. Returns the view matrix.
pub fn look_at_zup(eye: [f32; 3], target: [f32; 3], up: [f32; 3]) -> Mat4 {
    let f = normalize([target[0] - eye[0], target[1] - eye[1], target[2] - eye[2]]);
    let s = normalize(cross(f, up));
    let u = cross(s, f);
    [
        [s[0], u[0], -f[0], 0.0],
        [s[1], u[1], -f[1], 0.0],
        [s[2], u[2], -f[2], 0.0],
        [-dot(s, eye), -dot(u, eye), dot(f, eye), 1.0],
    ]
}

/// gluPerspective-style projection (clip space -1..1 in z).
pub fn perspective_opengl(fov_y_deg: f32, aspect: f32, near: f32, far: f32) -> Mat4 {
    let f = (fov_y_deg.to_radians() * 0.5).tan().recip();
    let dz = near - far;
    [
        [f / aspect, 0.0, 0.0, 0.0],
        [0.0, f, 0.0, 0.0],
        [0.0, 0.0, (far + near) / dz, -1.0],
        [0.0, 0.0, (2.0 * far * near) / dz, 0.0],
    ]
}

/// Remap OpenGL clip z (-1..1) to wgpu depth (0..1).
pub fn to_wgpu_depth(m: &Mat4) -> Mat4 {
    let mut out = *m;
    // row 2: z' = 0.5*z + 0.5*w
    for col in 0..4 {
        out[col][2] = 0.5 * m[col][2] + 0.5 * m[col][3];
    }
    out
}
