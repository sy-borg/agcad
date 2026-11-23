//! Repeating and mirroring solids: the geometry half of the pattern commands.
//!
//! Linear copies only move the source; circular copies move and rotate it, so
//! both reuse the source's local mesh and stay cheap for a large count. A
//! mirror is a reflection, which no `T · R · S` placement can express without
//! a negative scale (and inside-out winding), so it bakes the reflected world
//! mesh and re-centers it like a boolean result.
//!
//! Everything here is deterministic: no iteration-order dependence, exact
//! `f64` trig, and the copy placement is a pure function of the source
//! placement plus the pattern arguments.

use wright_protocol::{Mesh, Vec3};

use crate::{boolean, plane, transform_mesh, Entity, Error};

/// One copy's placement rule. The index is 1-based and excludes the original.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum CopyKind {
    Linear { direction: Vec3, spacing: f64, index: u32 },
    /// `step_deg` is the angle from one copy to the next (angle / count).
    Circular { center: Vec3, axis: Vec3, step_deg: f64, index: u32 },
    Mirror { origin: Vec3, normal: Vec3 },
}

impl CopyKind {
    /// Stable name for reports.
    pub fn label(&self) -> &'static str {
        match self {
            Self::Linear { .. } => "pattern",
            Self::Circular { .. } => "pattern",
            Self::Mirror { .. } => "mirror",
        }
    }
}

/// The placement and local mesh of one copy.
pub struct CopyGeometry {
    pub mesh: Mesh,
    pub translation: Vec3,
    pub rotation_deg: Vec3,
    pub scale: Vec3,
}

fn sub(a: Vec3, b: Vec3) -> Vec3 {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn add(a: Vec3, b: Vec3) -> Vec3 {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

fn mul(v: Vec3, s: f64) -> Vec3 {
    [v[0] * s, v[1] * s, v[2] * s]
}

fn dot(a: Vec3, b: Vec3) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn normalize(v: Vec3) -> Vec3 {
    let len = dot(v, v).sqrt();
    if len < 1e-12 { [0.0; 3] } else { mul(v, 1.0 / len) }
}

type Mat3 = [[f64; 3]; 3];

fn mat3_mul(a: Mat3, b: Mat3) -> Mat3 {
    let mut out = [[0.0; 3]; 3];
    for (r, row) in out.iter_mut().enumerate() {
        for (c, v) in row.iter_mut().enumerate() {
            *v = (0..3).map(|k| a[r][k] * b[k][c]).sum();
        }
    }
    out
}

fn mat3_apply(m: Mat3, v: Vec3) -> Vec3 {
    [
        m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2],
        m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
        m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2],
    ]
}

/// Extrinsic XYZ rotation matrix (`Rz · Ry · Rx`), matching `mat4::compose`.
fn euler_matrix(deg: Vec3) -> Mat3 {
    let (sx, cx) = deg[0].to_radians().sin_cos();
    let (sy, cy) = deg[1].to_radians().sin_cos();
    let (sz, cz) = deg[2].to_radians().sin_cos();
    let rx = [[1.0, 0.0, 0.0], [0.0, cx, -sx], [0.0, sx, cx]];
    let ry = [[cy, 0.0, sy], [0.0, 1.0, 0.0], [-sy, 0.0, cy]];
    let rz = [[cz, -sz, 0.0], [sz, cz, 0.0], [0.0, 0.0, 1.0]];
    mat3_mul(rz, mat3_mul(ry, rx))
}

/// Rodrigues rotation about a unit axis.
fn axis_rotation(axis: Vec3, deg: f64) -> Mat3 {
    let (s, c) = deg.to_radians().sin_cos();
    let t = 1.0 - c;
    let [x, y, z] = axis;
    [
        [t * x * x + c, t * x * y - s * z, t * x * z + s * y],
        [t * x * y + s * z, t * y * y + c, t * y * z - s * x],
        [t * x * z - s * y, t * y * z + s * x, t * z * z + c],
    ]
}

fn columns(m: Mat3) -> (Vec3, Vec3, Vec3) {
    (
        [m[0][0], m[1][0], m[2][0]],
        [m[0][1], m[1][1], m[2][1]],
        [m[0][2], m[1][2], m[2][2]],
    )
}

/// Build one copy from a source solid.
pub fn build(source: &Entity, kind: &CopyKind) -> Result<CopyGeometry, Error> {
    match *kind {
        CopyKind::Linear { direction, spacing, index } => {
            let offset = mul(normalize(direction), spacing * index as f64);
            Ok(CopyGeometry {
                mesh: source.mesh.clone(),
                translation: add(source.translation, offset),
                rotation_deg: source.rotation_deg,
                scale: source.scale,
            })
        },
        CopyKind::Circular { center, axis, step_deg, index } => {
            let axis = normalize(axis);
            let r = axis_rotation(axis, step_deg * index as f64);
            let translation = add(center, mat3_apply(r, sub(source.translation, center)));
            let rotation = mat3_mul(r, euler_matrix(source.rotation_deg));
            let (x, y, z) = columns(rotation);
            Ok(CopyGeometry {
                mesh: source.mesh.clone(),
                translation,
                rotation_deg: plane::euler_xyz_deg(x, y, z),
                scale: source.scale,
            })
        },
        CopyKind::Mirror { origin, normal } => {
            let n = normalize(normal);
            if dot(n, n) < 0.5 {
                return Err(Error::Invalid(
                    "mirror plane normal must be a non-zero vector (e.g. [1,0,0])".into(),
                ));
            }
            let world = transform_mesh(&source.mesh, &source.transform());
            let d = dot(n, origin);
            let positions: Vec<[f32; 3]> = world
                .positions
                .iter()
                .map(|p| {
                    let q = [p[0] as f64, p[1] as f64, p[2] as f64];
                    let off = dot(n, q) - d;
                    let m = sub(q, mul(n, 2.0 * off));
                    [m[0] as f32, m[1] as f32, m[2] as f32]
                })
                .collect();
            // A reflection reverses handedness: flip the winding so the copy
            // still has outward-facing triangles, then rebuild crease normals.
            let mut indices = world.indices;
            for tri in indices.chunks_exact_mut(3) {
                tri.swap(1, 2);
            }
            let reflected = boolean::crease_mesh(&positions, &indices);
            let (mesh, center) = crate::recenter(&reflected);
            Ok(CopyGeometry {
                mesh,
                translation: center,
                rotation_deg: [0.0; 3],
                scale: [1.0; 3],
            })
        },
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{mat4, Shape};

    fn box_entity() -> Entity {
        Entity {
            id: 1,
            name: "box".into(),
            shape: Shape::Box { size: [10.0, 10.0, 10.0] },
            mesh: crate::geo::box_mesh([10.0, 10.0, 10.0]),
            translation: [0.0, 0.0, 0.0],
            rotation_deg: [0.0; 3],
            scale: [1.0; 3],
            color: [1.0; 4],
            geom_key: Default::default(),
        }
    }

    fn world_center(copy: &CopyGeometry) -> Vec3 {
        let m = mat4::compose(copy.translation, copy.rotation_deg, copy.scale);
        let p = mat4::transform_point(&m, [0.0, 0.0, 0.0]);
        [p[0] as f64, p[1] as f64, p[2] as f64]
    }

    #[test]
    fn linear_and_circular_copies_land_where_expected() {
        let src = box_entity();
        let line = build(
            &src,
            &CopyKind::Linear { direction: [1.0, 0.0, 0.0], spacing: 20.0, index: 3 },
        )
        .unwrap();
        assert!((world_center(&line)[0] - 60.0).abs() < 1e-9);
        assert_eq!(line.mesh.positions, src.mesh.positions);

        let circ = build(
            &src,
            &CopyKind::Circular {
                center: [0.0; 3],
                axis: [0.0, 0.0, 1.0],
                step_deg: 90.0,
                index: 1,
            },
        )
        .unwrap();
        let c = world_center(&circ);
        assert!(c[0].abs() < 1e-9 && (c[1] - 0.0).abs() < 1e-9);
        // A point 10 mm out on +X must land 10 mm out on +Y.
        let m = mat4::compose(circ.translation, circ.rotation_deg, circ.scale);
        let p = mat4::transform_point(&m, [10.0, 0.0, 0.0]);
        assert!(p[0].abs() < 1e-4 && (p[1] - 10.0).abs() < 1e-4, "{p:?}");
    }

    #[test]
    fn mirror_makes_an_outward_copy() {
        let mut src = box_entity();
        src.translation = [10.0, 0.0, 0.0];
        let copy = build(&src, &CopyKind::Mirror { origin: [0.0; 3], normal: [1.0, 0.0, 0.0] }).unwrap();
        assert!((copy.translation[0] + 10.0).abs() < 1e-6, "{:?}", copy.translation);
        let volume = crate::fillet::signed_volume(&copy.mesh);
        assert!(volume > 0.0, "mirrored winding must stay outward: {volume}");
    }
}
