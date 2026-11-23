//! Sketch planes: standard datum planes, explicit planes, and planar faces.
//!
//! A sketch lives in its own 2D frame. This module turns a requested plane
//! into that frame — origin, outward normal (local +Z) and in-plane X — and
//! then into the entity's `(translation, rotation_deg)` placement.
//!
//! Face selection is geometric, like [`crate::edges::EdgeFilter`]: the mesh
//! is welded by exact f32 bits, triangles are grouped into maximal coplanar
//! patches, and the caller picks a patch by outward normal and/or a
//! world-space point. A curved surface is never merged into one face (its
//! facets differ by more than the coplanarity threshold), so a sketch can
//! only land on a genuinely flat patch. Selection is deterministic: fixed
//! traversal order, `BTreeMap` adjacency, no hashing.

use std::collections::BTreeMap;

use wright_protocol::{DatumPlane, EntityId, Mesh, Vec3};

use crate::{edges, mat4, Error};

/// Cos of the angle below which two adjacent triangles count as coplanar.
/// ~0.26°: far above f32 rounding, far below the facet turn of any curved
/// primitive or revolve (which is ≥ ~0.7° even at 512 segments).
const COPLANAR_COS: f64 = 0.99999;

/// A resolved sketch frame in world space. Local +Z is `normal`; local +X is
/// `x_axis`; local +Y is `normal × x_axis`.
#[derive(Debug, Clone, Copy)]
pub struct Frame {
    pub origin: Vec3,
    pub normal: Vec3,
    pub x_axis: Vec3,
}

impl Frame {
    pub fn y_axis(&self) -> Vec3 {
        cross(self.normal, self.x_axis)
    }
}

fn normalize(v: Vec3) -> Vec3 {
    let len = dot(v, v).sqrt();
    if len < 1e-12 { [0.0; 3] } else { [v[0] / len, v[1] / len, v[2] / len] }
}

fn dot(a: Vec3, b: Vec3) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn cross(a: Vec3, b: Vec3) -> Vec3 {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

fn sub(a: Vec3, b: Vec3) -> Vec3 {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn mul(v: Vec3, s: f64) -> Vec3 {
    [v[0] * s, v[1] * s, v[2] * s]
}

fn finite(v: Vec3) -> bool {
    v.iter().all(|x| x.is_finite())
}

/// A standard datum plane, offset along its normal.
pub fn datum_frame(plane: DatumPlane, offset: f64) -> Result<Frame, Error> {
    if !offset.is_finite() {
        return Err(Error::Invalid("sketch plane offset must be finite".into()));
    }
    let (x_axis, normal) = match plane {
        DatumPlane::Xy => ([1.0, 0.0, 0.0], [0.0, 0.0, 1.0]),
        // Right-handed: X × Z = -Y, so the front view (X right, Z up)
        // looks along +Y and extrudes toward -Y.
        DatumPlane::Xz => ([1.0, 0.0, 0.0], [0.0, -1.0, 0.0]),
        DatumPlane::Yz => ([0.0, 1.0, 0.0], [1.0, 0.0, 0.0]),
    };
    Ok(Frame { origin: mul(normal, offset), normal, x_axis })
}

/// A plane given by an origin and a normal, with an optional in-plane X.
pub fn custom_frame(origin: Vec3, normal: Vec3, x_axis: Option<Vec3>) -> Result<Frame, Error> {
    if !finite(origin) {
        return Err(Error::Invalid("sketch plane origin must be finite".into()));
    }
    if !finite(normal) || dot(normal, normal) < 1e-12 {
        return Err(Error::Invalid(
            "sketch plane normal must be a non-zero vector (e.g. [0,0,1] for an upward-facing plane)"
                .into(),
        ));
    }
    let normal = normalize(normal);
    let x_axis = match x_axis {
        Some(v) => {
            if !finite(v) || dot(v, v) < 1e-12 {
                return Err(Error::Invalid(
                    "sketch plane x_axis must be a non-zero vector when given".into(),
                ));
            }
            let projected = sub(v, mul(normal, dot(v, normal)));
            if dot(projected, projected).sqrt() < 1e-9 {
                return Err(Error::Invalid(
                    "sketch plane x_axis must not be parallel to the plane normal".into(),
                ));
            }
            normalize(projected)
        },
        None => in_plane_x(normal),
    };
    Ok(Frame { origin, normal, x_axis })
}

/// Pick a deterministic in-plane X for a normal: world +X projected onto the
/// plane, or world +Y when the plane is nearly perpendicular to X. The
/// result is always well-defined because one of those projections is long.
fn in_plane_x(normal: Vec3) -> Vec3 {
    let mut helper = [1.0, 0.0, 0.0];
    if dot(normal, helper).abs() > 0.9 {
        helper = [0.0, 1.0, 0.0];
    }
    normalize(sub(helper, mul(normal, dot(helper, normal))))
}

/// A maximal coplanar patch of the mesh.
///
/// `triangles` indexes the **input mesh's** triangle order (`indices` in
/// triples), which is the same order the welded mesh keeps; a picking
/// consumer can use it to map a hit triangle back to its face. Fields are
/// world-space when the mesh handed to [`planar_faces`] is.
#[derive(Debug, Clone)]
pub struct PlanarFace {
    /// Area-weighted outward normal.
    pub normal: Vec3,
    /// Area-weighted centroid (a point on the plane).
    pub centroid: Vec3,
    /// `dot(normal, centroid)`: the plane offset.
    pub offset: f64,
    pub area: f64,
    /// Triangle indices belonging to this patch, ascending.
    pub triangles: Vec<usize>,
}

/// Choose a planar face and build its sketch frame. `normal_filter` is an
/// optional outward normal (world space); `near` ranks parallel faces by
/// distance from the point to their plane.
pub fn face_frame(
    mesh: &Mesh,
    near: Option<Vec3>,
    normal_filter: Option<Vec3>,
    angle_tol_deg: Option<f64>,
    entity: EntityId,
) -> Result<Frame, Error> {
    if near.is_some_and(|v| !finite(v)) {
        return Err(Error::Invalid("sketch plane near point must be finite".into()));
    }
    let want = match normal_filter {
        Some(v) => {
            if !finite(v) || dot(v, v) < 1e-12 {
                return Err(Error::Invalid(
                    "sketch plane face.normal must be a non-zero vector (e.g. [0,0,1])".into(),
                ));
            }
            Some(normalize(v))
        },
        None => None,
    };
    let tol = angle_tol_deg.unwrap_or(1.0);
    if !(tol > 0.0 && tol <= 90.0) {
        return Err(Error::Invalid("sketch plane angle_tol_deg must be in (0, 90]".into()));
    }
    let min_cos = tol.to_radians().cos();

    let faces = planar_faces(mesh)?;
    if faces.is_empty() {
        return Err(Error::Invalid(format!(
            "entity {entity} has no planar face to sketch on (hint: a sketch needs a flat face; \
             use add_sketch with a datum or custom plane instead)"
        )));
    }

    let candidates: Vec<&PlanarFace> = faces
        .iter()
        .filter(|f| match want {
            Some(w) => dot(f.normal, w) >= min_cos,
            None => true,
        })
        .collect();
    if candidates.is_empty() {
        let mut normals: Vec<String> = faces
            .iter()
            .map(|f| {
                format!(
                    "[{:.2}, {:.2}, {:.2}]",
                    f.normal[0], f.normal[1], f.normal[2]
                )
            })
            .collect();
        normals.sort();
        normals.dedup();
        let wanted = want.unwrap();
        return Err(Error::Invalid(format!(
            "entity {entity} has no face with outward normal [{:.2}, {:.2}, {:.2}] \
             (hint: available face normals: {}; a curved surface has no single planar face)",
            wanted[0],
            wanted[1],
            wanted[2],
            normals.join(", ")
        )));
    }

    // Deterministic pick: nearest plane when a point is given, otherwise the
    // largest face (the obvious one an agent means). `planar_faces` visits
    // triangles in mesh order, so ties keep a stable candidate order.
    let chosen = match near {
        Some(p) => candidates
            .iter()
            .copied()
            .min_by(|a, b| {
                let da = (dot(a.normal, p) - a.offset).abs();
                let db = (dot(b.normal, p) - b.offset).abs();
                da.total_cmp(&db).then_with(|| b.area.total_cmp(&a.area))
            })
            .expect("candidates is non-empty"),
        None => candidates
            .iter()
            .copied()
            .max_by(|a, b| a.area.total_cmp(&b.area))
            .expect("candidates is non-empty"),
    };

    let origin = match near {
        Some(p) => sub(p, mul(chosen.normal, dot(chosen.normal, p) - chosen.offset)),
        None => chosen.centroid,
    };
    Ok(Frame { origin, normal: chosen.normal, x_axis: in_plane_x(chosen.normal) })
}

/// Group a welded triangle mesh into maximal coplanar patches.
pub fn planar_faces(mesh: &Mesh) -> Result<Vec<PlanarFace>, Error> {
    let (positions, tris) = edges::weld(mesh)?;
    let count = tris.len() / 3;
    if count == 0 {
        return Ok(Vec::new());
    }

    // Geometric triangle normals and areas (world positions, so a scaled
    // entity still yields correct outward normals).
    let mut normals: Vec<Vec3> = Vec::with_capacity(count);
    let mut areas: Vec<f64> = Vec::with_capacity(count);
    let mut degenerate = vec![false; count];
    for t in 0..count {
        let i = &tris[t * 3..t * 3 + 3];
        let p0 = positions[i[0] as usize];
        let p1 = positions[i[1] as usize];
        let p2 = positions[i[2] as usize];
        let c = cross(sub(p1, p0), sub(p2, p0));
        let area2 = dot(c, c).sqrt();
        degenerate[t] = area2 < 1e-12;
        areas.push(area2 * 0.5);
        normals.push(if degenerate[t] { [0.0; 3] } else { mul(c, 1.0 / area2) });
    }

    // Undirected edge → the triangles that use it.
    let mut adj: BTreeMap<(u32, u32), Vec<usize>> = BTreeMap::new();
    for t in 0..count {
        let i = &tris[t * 3..t * 3 + 3];
        for (a, b) in [(i[0], i[1]), (i[1], i[2]), (i[2], i[0])] {
            let key = if a < b { (a, b) } else { (b, a) };
            adj.entry(key).or_default().push(t);
        }
    }

    let mut seen = vec![false; count];
    let mut faces = Vec::new();
    for start in 0..count {
        if seen[start] || degenerate[start] {
            continue;
        }
        seen[start] = true;
        let mut stack = vec![start];
        let mut group = Vec::new();
        while let Some(t) = stack.pop() {
            group.push(t);
            let i = &tris[t * 3..t * 3 + 3];
            for (a, b) in [(i[0], i[1]), (i[1], i[2]), (i[2], i[0])] {
                let key = if a < b { (a, b) } else { (b, a) };
                if let Some(neighbors) = adj.get(&key) {
                    for &u in neighbors {
                        if u == t || seen[u] || degenerate[u] {
                            continue;
                        }
                        if dot(normals[t], normals[u]) >= COPLANAR_COS {
                            seen[u] = true;
                            stack.push(u);
                        }
                    }
                }
            }
        }
        group.sort_unstable();

        let mut normal_sum = [0.0; 3];
        let mut centroid_sum = [0.0; 3];
        let mut area = 0.0;
        for &t in &group {
            let i = &tris[t * 3..t * 3 + 3];
            let (p0, p1, p2) = (
                positions[i[0] as usize],
                positions[i[1] as usize],
                positions[i[2] as usize],
            );
            let a = areas[t];
            let c = [
                (p0[0] + p1[0] + p2[0]) / 3.0,
                (p0[1] + p1[1] + p2[1]) / 3.0,
                (p0[2] + p1[2] + p2[2]) / 3.0,
            ];
            for k in 0..3 {
                normal_sum[k] += normals[t][k] * a;
                centroid_sum[k] += c[k] * a;
            }
            area += a;
        }
        if area <= 0.0 {
            continue;
        }
        let normal = normalize(normal_sum);
        if dot(normal, normal) < 0.5 {
            continue;
        }
        let centroid = [
            centroid_sum[0] / area,
            centroid_sum[1] / area,
            centroid_sum[2] / area,
        ];
        let offset = dot(normal, centroid);
        faces.push(PlanarFace { normal, centroid, offset, area, triangles: group });
    }
    Ok(faces)
}

/// Euler XYZ degrees (`Rz·Ry·Rx`) for a right-handed basis whose columns are
/// local X, local Y and local Z in world space. At the gimbal lock of
/// `|normal.z| = 1` the Y rotation is ±90° and Z is set to zero.
pub fn euler_xyz_deg(x_axis: Vec3, y_axis: Vec3, normal: Vec3) -> Vec3 {
    let (r00, r10, r20) = (x_axis[0], x_axis[1], x_axis[2]);
    let (r11, r21) = (y_axis[1], y_axis[2]);
    let (r12, r22) = (normal[1], normal[2]);
    let cos_b = (r00 * r00 + r10 * r10).sqrt();
    if cos_b > 1e-9 {
        let beta = (-r20).clamp(-1.0, 1.0).asin();
        let alpha = r21.atan2(r22);
        let gamma = r10.atan2(r00);
        [alpha.to_degrees(), beta.to_degrees(), gamma.to_degrees()]
    } else {
        let beta = if r20 < 0.0 { std::f64::consts::FRAC_PI_2 } else { -std::f64::consts::FRAC_PI_2 };
        let alpha = (-r12).atan2(r11);
        [alpha.to_degrees(), beta.to_degrees(), 0.0]
    }
}

/// Entity placement `(translation, rotation_deg)` that puts local XY on the
/// frame.
pub fn placement(frame: &Frame) -> (Vec3, Vec3) {
    (
        frame.origin,
        euler_xyz_deg(frame.x_axis, frame.y_axis(), frame.normal),
    )
}

/// World-space plane frame of an entity transform, if it is a sketch.
pub fn frame_from_transform(transform: &wright_protocol::Mat4) -> wright_protocol::SketchPlaneFrame {
    let origin = mat4::transform_point(transform, [0.0, 0.0, 0.0]);
    let x = normalize(mat4::transform_direction(transform, [1.0, 0.0, 0.0]));
    let n = normalize(mat4::transform_direction(transform, [0.0, 0.0, 1.0]));
    wright_protocol::SketchPlaneFrame {
        origin: [origin[0] as f64, origin[1] as f64, origin[2] as f64],
        normal: n,
        x_axis: x,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geo;

    fn transformed(frame: &Frame) -> wright_protocol::Mat4 {
        let (t, r) = placement(frame);
        mat4::compose(t, r, [1.0; 3])
    }

    #[test]
    fn datum_frames_round_trip_through_euler() {
        for plane in [DatumPlane::Xy, DatumPlane::Xz, DatumPlane::Yz] {
            let f = datum_frame(plane, 3.0).unwrap();
            let m = transformed(&f);
            let x = mat4::transform_direction(&m, [1.0, 0.0, 0.0]);
            let y = mat4::transform_direction(&m, [0.0, 1.0, 0.0]);
            let n = mat4::transform_direction(&m, [0.0, 0.0, 1.0]);
            for (got, want) in [(x, f.x_axis), (y, f.y_axis()), (n, f.normal)] {
                for i in 0..3 {
                    assert!((got[i] - want[i]).abs() < 1e-5, "{plane:?}: {got:?} vs {want:?}");
                }
            }
            let o = mat4::transform_point(&m, [0.0, 0.0, 0.0]);
            for i in 0..3 {
                assert!((o[i] as f64 - f.origin[i]).abs() < 1e-5, "{plane:?}");
            }
        }
    }

    #[test]
    fn custom_frame_projects_the_x_axis() {
        let f = custom_frame([1.0, 2.0, 3.0], [0.0, 0.0, 2.0], Some([1.0, 4.0, 9.0])).unwrap();
        assert!((f.normal[2] - 1.0).abs() < 1e-9);
        assert!((f.x_axis[2]).abs() < 1e-9, "x must lie in the plane: {:?}", f.x_axis);
        assert!((f.x_axis[0] - 1.0 / 17.0f64.sqrt()).abs() < 1e-6);
        assert!(custom_frame([0.0; 3], [0.0; 3], None).is_err());
        assert!(custom_frame([0.0; 3], [0.0, 0.0, 1.0], Some([0.0, 0.0, 2.0])).is_err());
    }

    #[test]
    fn finds_the_top_face_of_a_box() {
        let mesh = geo::box_mesh([40.0, 30.0, 10.0]);
        let f = face_frame(&mesh, None, Some([0.0, 0.0, 1.0]), None, 1).unwrap();
        assert!((f.normal[2] - 1.0).abs() < 1e-9);
        assert!((f.origin[2] - 5.0).abs() < 1e-9, "{:?}", f.origin);
        assert!((f.x_axis[0] - 1.0).abs() < 1e-9);
        assert!((f.origin[0]).abs() < 1e-9 && (f.origin[1]).abs() < 1e-9);

        let f = face_frame(&mesh, Some([3.0, -2.0, 5.0]), Some([0.0, 0.0, 1.0]), None, 1).unwrap();
        assert!((f.origin[0] - 3.0).abs() < 1e-9 && (f.origin[1] + 2.0).abs() < 1e-9);
    }

    #[test]
    fn missing_normal_lists_the_available_faces() {
        let mesh = geo::box_mesh([10.0; 3]);
        let err = face_frame(&mesh, None, Some([1.0, 1.0, 0.0]), None, 7).unwrap_err();
        let msg = err.to_string();
        assert!(msg.contains("entity 7"), "{msg}");
        assert!(msg.contains("available face normals"), "{msg}");
        assert!(msg.contains("[0.00, 0.00, 1.00]"), "{msg}");
    }
}
