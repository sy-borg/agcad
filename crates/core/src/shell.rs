//! Shell: hollow a solid by offsetting its surface inward.
//!
//! The wall is built as `body − inner`, where `inner` is the body's surface
//! with every vertex moved along the inward normal by `thickness`. A vertex
//! belongs to several faces, so its displacement is the least-squares solution
//! of `n · d = -thickness` over its adjacent face normals. On a box corner
//! that reproduces the exact offset corner; on a tessellated rim it moves the
//! vertex diagonally, which is exactly the offset surface the boolean wants.
//!
//! Open faces (a cup without its lid) get `n · d = +overcut` instead, so the
//! inner surface pokes out through that face and the difference leaves an
//! opening. The overcut is the body's bbox diagonal, large enough to clear any
//! wall the caller can sensibly ask for.
//!
//! The inner surface is imported through Manifold's strict mesher. A wall too
//! thick for a feature makes the offset self-intersect, which is reported as a
//! clear error instead of producing a broken solid.

use wright_protocol::{Mesh, Vec3};

use crate::{boolean, edges, Error};

fn sub(a: Vec3, b: Vec3) -> Vec3 {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
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

fn normalize(v: Vec3) -> Vec3 {
    let len = dot(v, v).sqrt();
    if len < 1e-12 { [0.0; 3] } else { [v[0] / len, v[1] / len, v[2] / len] }
}

fn bbox_diagonal(mesh: &Mesh) -> f64 {
    if mesh.positions.is_empty() {
        return 0.0;
    }
    let mut lo = [f64::MAX; 3];
    let mut hi = [f64::MIN; 3];
    for p in &mesh.positions {
        for i in 0..3 {
            lo[i] = lo[i].min(p[i] as f64);
            hi[i] = hi[i].max(p[i] as f64);
        }
    }
    ((hi[0] - lo[0]).powi(2) + (hi[1] - lo[1]).powi(2) + (hi[2] - lo[2]).powi(2)).sqrt()
}

/// Invert a symmetric 3x3 matrix with a small Tikhonov term, or `None` when it
/// carries no usable direction (an isolated vertex).
fn solve_symmetric(a: [[f64; 3]; 3], b: Vec3) -> Option<Vec3> {
    let det = a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[1][2])
        - a[0][1] * (a[0][1] * a[2][2] - a[1][2] * a[0][2])
        + a[0][2] * (a[0][1] * a[1][2] - a[1][1] * a[0][2]);
    if !det.is_finite() || det.abs() < 1e-16 {
        return None;
    }
    let inv = 1.0 / det;
    let i00 = (a[1][1] * a[2][2] - a[1][2] * a[1][2]) * inv;
    let i01 = (a[0][2] * a[1][2] - a[0][1] * a[2][2]) * inv;
    let i02 = (a[0][1] * a[1][2] - a[0][2] * a[1][1]) * inv;
    let i11 = (a[0][0] * a[2][2] - a[0][2] * a[0][2]) * inv;
    let i12 = (a[0][2] * a[0][1] - a[0][0] * a[1][2]) * inv;
    let i22 = (a[0][0] * a[1][1] - a[0][1] * a[0][1]) * inv;
    Some([
        i00 * b[0] + i01 * b[1] + i02 * b[2],
        i01 * b[0] + i11 * b[1] + i12 * b[2],
        i02 * b[0] + i12 * b[1] + i22 * b[2],
    ])
}

/// Hollow a **world-space** closed solid. `open` holds outward face normals
/// to leave open; an empty list makes a closed hollow body.
pub fn shell(
    mesh: &Mesh,
    thickness: f64,
    open: &[Vec3],
    angle_tol_deg: f64,
) -> Result<Mesh, Error> {
    if !(thickness.is_finite() && thickness > 0.0) {
        return Err(Error::Invalid(format!(
            "shell thickness must be a finite value > 0 (got {thickness})"
        )));
    }
    if !(angle_tol_deg > 0.0 && angle_tol_deg <= 90.0) {
        return Err(Error::Invalid("shell angle_tol_deg must be in (0, 90]".into()));
    }
    let opens: Vec<Vec3> = open.iter().map(|n| normalize(*n)).collect();
    if opens.iter().any(|n| dot(*n, *n) < 0.5) {
        return Err(Error::Invalid(
            "shell open normals must be non-zero vectors (e.g. [[0,0,1]] for the top)".into(),
        ));
    }

    let (positions, tris) = edges::weld(mesh)
        .map_err(|e| Error::Invalid(format!("shell needs a closed solid: {e}")))?;
    if tris.len() < 12 {
        return Err(Error::Invalid("shell needs a solid with triangles".into()));
    }
    let count = tris.len() / 3;
    let mut tri_normal: Vec<Option<Vec3>> = Vec::with_capacity(count);
    for t in 0..count {
        let i = &tris[t * 3..t * 3 + 3];
        let p0 = positions[i[0] as usize];
        let p1 = positions[i[1] as usize];
        let p2 = positions[i[2] as usize];
        let n = normalize(cross(sub(p1, p0), sub(p2, p0)));
        tri_normal.push(if dot(n, n) < 0.5 { None } else { Some(n) });
    }

    let diag = bbox_diagonal(mesh).max(1.0);
    let overcut = diag.max(1.0) * 1.5;
    let min_cos = angle_tol_deg.to_radians().cos();
    let open_tris: Vec<bool> = tri_normal
        .iter()
        .map(|n| match n {
            Some(n) => opens.iter().any(|want| dot(*n, *want) >= min_cos),
            None => false,
        })
        .collect();
    if !opens.is_empty() && !open_tris.iter().any(|&open| open) {
        return Err(Error::Invalid(
            "shell found no face matching the requested open normals (hint: use an outward \
             face normal such as [0,0,1] for the top)"
                .into(),
        ));
    }

    // Displacement per welded vertex: least squares over the planes it touches.
    let mut a = vec![[[0.0f64; 3]; 3]; positions.len()];
    let mut b = vec![[0.0f64; 3]; positions.len()];
    for t in 0..count {
        let Some(n) = tri_normal[t] else { continue };
        let c = if open_tris[t] { overcut } else { -thickness };
        let i = &tris[t * 3..t * 3 + 3];
        for &v in i {
            let v = v as usize;
            for r in 0..3 {
                for cc in 0..3 {
                    a[v][r][cc] += n[r] * n[cc];
                }
                b[v][r] += n[r] * c;
            }
        }
    }

    let mut inner_positions: Vec<[f32; 3]> = Vec::with_capacity(positions.len());
    for v in 0..positions.len() {
        // A tiny Tikhonov term makes a flat-face vertex (one normal) solvable;
        // the determinant check above is scaled for that rank-1 case.
        for r in 0..3 {
            a[v][r][r] += 1e-7;
        }
        let d = solve_symmetric(a[v], b[v]).unwrap_or([0.0; 3]);
        inner_positions.push([
            (positions[v][0] + d[0]) as f32,
            (positions[v][1] + d[1]) as f32,
            (positions[v][2] + d[2]) as f32,
        ]);
    }
    let inner = boolean::crease_mesh(&inner_positions, &tris);
    let inner_volume = crate::fillet::signed_volume(&inner);
    if !(inner_volume.is_finite() && inner_volume > 0.0) {
        return Err(Error::Invalid(format!(
            "shell wall {thickness} mm is too thick for this shape: the inward offset turned \
             itself inside out (hint: use a wall thinner than half the thinnest feature)"
        )));
    }
    let body_volume = crate::fillet::signed_volume(mesh);
    if opens.is_empty() && inner_volume >= body_volume {
        return Err(Error::Invalid(format!(
            "shell wall {thickness} mm leaves no cavity (inner volume {inner_volume:.3} >= body \
             volume {body_volume:.3}; hint: use a thinner wall)"
        )));
    }

    boolean::boolean(wright_protocol::BooleanOp::Difference, &[mesh.clone(), inner]).map_err(|e| {
        Error::Invalid(format!(
            "shell failed: {e} (hint: {thickness} mm may be too thick for this shape; try a \
             thinner wall and check validate)"
        ))
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::fillet::signed_volume;
    use crate::geo;

    fn world_box(size: [f64; 3], center: [f64; 3]) -> Mesh {
        let mut m = geo::box_mesh(size);
        for p in &mut m.positions {
            for i in 0..3 {
                p[i] += center[i] as f32;
            }
        }
        m
    }

    #[test]
    fn closed_shell_removes_the_expected_volume() {
        let body = world_box([40.0, 30.0, 20.0], [0.0; 3]);
        let shelled = shell(&body, 2.0, &[], 1.0).unwrap();
        let outer = 40.0 * 30.0 * 20.0;
        let inner = 36.0 * 26.0 * 16.0;
        let v = signed_volume(&shelled);
        assert!((v - (outer - inner)).abs() / (outer - inner) < 0.01, "volume {v}");
        let check = crate::validate::check_solid(&shelled);
        assert!(check.watertight && check.non_manifold_edges == 0, "{check:?}");
    }

    #[test]
    fn open_shell_has_no_lid() {
        let body = world_box([40.0, 30.0, 20.0], [0.0; 3]);
        let shelled = shell(&body, 2.0, &[[0.0, 0.0, 1.0]], 1.0).unwrap();
        let outer = 40.0 * 30.0 * 20.0;
        // Cavity 36 x 26 x 18, open at the top: volume = outer - cavity.
        let v = signed_volume(&shelled);
        assert!((v - (outer - 36.0 * 26.0 * 18.0)).abs() / outer < 0.02, "volume {v}");
        let check = crate::validate::check_solid(&shelled);
        assert!(check.watertight && check.non_manifold_edges == 0, "{check:?}");
    }

    #[test]
    fn an_impossible_wall_is_a_clear_error() {
        let body = world_box([10.0, 10.0, 10.0], [0.0; 3]);
        let err = shell(&body, 8.0, &[], 1.0).unwrap_err();
        assert!(err.to_string().contains("too thick"), "{err}");
    }

    #[test]
    fn a_shell_follows_a_boolean_and_keeps_flat_face_vertices_sane() {
        let plate = world_box([40.0, 40.0, 12.0], [0.0; 3]);
        let drilled = crate::hole::drill(
            &plate,
            &crate::hole::HoleSpec {
                center: [8.0, 8.0, 6.0],
                normal: [0.0, 0.0, 1.0],
                diameter: 6.0,
                depth: None,
                counterbore_diameter: None,
                counterbore_depth: None,
                countersink_angle_deg: None,
                segments: Some(48),
            },
        )
        .unwrap();
        let shelled = shell(&drilled, 2.0, &[[0.0, 0.0, 1.0]], 1.0).unwrap();
        let check = crate::validate::check_solid(&shelled);
        assert!(check.watertight && check.non_manifold_edges == 0, "{check:?}");
        assert!(check.volume > 0.0);
    }
}
