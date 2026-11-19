//! Turning 2D sketch profiles into printable solids: extrude and revolve.
//!
//! Both go through [manifold-rust]'s constructors, which guarantee
//! watertight, manifold output (what a slicer needs) and are deterministic —
//! the same profile and parameters produce the same mesh bits every replay.
//!
//! Profiles are normalized with Clipper2's even-odd union first: nested
//! loops alternate solid/hole regardless of the order the user drew them,
//! crossing loops resolve, and the result is a clean set of contours
//! (outers CCW, holes CW) that the Manifold constructors expect.
//!
//! Local coordinates: extrusion runs along +Z from the sketch plane; a
//! revolve turns around the sketch's X or Y axis. The engine places the
//! solid with the sketch's transform, so the base always sits on the sketch
//! plane and the extrusion direction is the sketch normal.

use clipper2_rust::{union_subjects_d, FillRule, PathD, PathsD, Point};
use manifold_rust::linalg::Vec2;
use manifold_rust::manifold::Manifold;
use manifold_rust::types::{Error as KernelError, Polygons};

use agcad_protocol::{Mesh, RevolveAxis};

use crate::boolean::from_manifold;
use crate::Error;

/// Decimal places Clipper2 keeps (matches Manifold's CrossSection). Sketch
/// coordinates are millimeters, so 1e-8 mm snapping is far below print
/// resolution and keeps the operation deterministic.
const PRECISION: i32 = 8;

/// Merge raw loops into non-overlapping contours using the even-odd rule.
fn normalize(loops: &[Vec<[f64; 2]>]) -> Result<Polygons, Error> {
    let paths: PathsD = loops
        .iter()
        .map(|lp| lp.iter().map(|p| Point::new(p[0], p[1])).collect::<PathD>())
        .collect();
    let merged = union_subjects_d(&paths, FillRule::EvenOdd, PRECISION);
    let polys: Polygons = merged
        .iter()
        .map(|path| path.iter().map(|p| Vec2::new(p.x, p.y)).collect())
        .collect();
    if polys.is_empty() {
        return Err(Error::Invalid(
            "profile encloses no area (hint: loops must be closed and non-degenerate; \
             check for zero-length lines or a sketch that failed to solve)"
                .into(),
        ));
    }
    Ok(polys)
}

fn kernel_error(what: &str, status: KernelError) -> Error {
    Error::Invalid(format!(
        "{what} failed in the kernel: {status:?} (hint: the profile must be a set of \
         closed, non-self-intersecting loops)"
    ))
}

fn checked(solid: &Manifold, what: &str) -> Result<(), Error> {
    let status = solid.status();
    if status != KernelError::NoError {
        return Err(kernel_error(what, status));
    }
    if solid.num_tri() == 0 {
        return Err(Error::Invalid(format!(
            "{what} produced no geometry (hint: check that the sketch profile encloses area)"
        )));
    }
    Ok(())
}

/// Extrude a profile along +Z. A negative `distance` builds the same solid
/// on the other side of the sketch plane (mesh z runs -|distance|..0).
pub fn extrude(loops: &[Vec<[f64; 2]>], distance: f64) -> Result<(Mesh, f64), Error> {
    let polys = normalize(loops)?;
    let height = distance.abs();
    let solid = Manifold::extrude(&polys, height, 0, 0.0, Vec2::new(1.0, 1.0));
    checked(&solid, "extrude")?;

    let mut mesh = from_manifold(&solid);
    if distance < 0.0 {
        // Mirror across the sketch plane, reversing winding so the solid
        // stays outward-facing (backface culling and slicers both care).
        for p in &mut mesh.positions {
            p[2] = -p[2];
        }
        for n in &mut mesh.normals {
            n[2] = -n[2];
        }
        for tri in mesh.indices.chunks_exact_mut(3) {
            tri.swap(1, 2);
        }
    }
    Ok((mesh, solid.volume()))
}

/// Revolve a profile around an in-plane axis. The profile must stay on one
/// side of the axis (touching it is allowed); a 360° sweep needs no caps,
/// partial sweeps get flat end caps from the kernel.
pub fn revolve(
    loops: &[Vec<[f64; 2]>],
    angle_deg: f64,
    axis: RevolveAxis,
    segments: Option<u32>,
) -> Result<(Mesh, f64), Error> {
    let polys = normalize(loops)?;

    // The kernel revolves around its Y axis with X as radius, mapping
    // (r, a) → (r·cosφ, r·sinφ, a). Pick the sketch coordinate that becomes
    // the radius and the one that runs along the axis.
    let (radius_of, along_of) = match axis {
        RevolveAxis::X => (1usize, 0usize), // radius = y, axis = x
        RevolveAxis::Y => (0usize, 1usize), // radius = x, axis = y
    };

    let (mut min_r, mut max_r) = (f64::MAX, f64::MIN);
    let (mut min_a, mut max_a) = (f64::MAX, f64::MIN);
    for poly in &polys {
        for p in poly {
            min_r = min_r.min(p[radius_of]);
            max_r = max_r.max(p[radius_of]);
            min_a = min_a.min(p[along_of]);
            max_a = max_a.max(p[along_of]);
        }
    }
    let scale = (max_a - min_a).max(max_r - min_r).max(1.0);
    let eps = 1e-9 * scale;
    if min_r < -eps && max_r > eps {
        return Err(Error::Invalid(format!(
            "profile crosses the revolve axis ({} spans {min_r:.4}..{max_r:.4} across it; \
             hint: keep the whole profile on one side of the {} axis)",
            if axis == RevolveAxis::X { "y" } else { "x" },
            axis.as_str(),
        )));
    }

    // Profile entirely on the negative side: mirror it to positive radius.
    // Mirroring reverses every contour's winding, so reverse them back.
    let mirror = max_r <= eps;
    let mapped: Polygons = polys
        .iter()
        .map(|poly| {
            let mut out: Vec<Vec2> = poly
                .iter()
                .map(|p| {
                    let r = if mirror { -p[radius_of] } else { p[radius_of] };
                    Vec2::new(r, p[along_of])
                })
                .collect();
            if mirror {
                out.reverse();
            }
            out
        })
        .collect();

    let angle = angle_deg.clamp(0.1, 360.0);
    let segs = match segments {
        Some(n) => n.clamp(3, 512) as i32,
        None => {
            // Aim for a sub-0.1 mm chord error at the largest radius.
            let r = max_r.abs().max(1.0);
            ((r * 50.0).sqrt().ceil() as i32).clamp(24, 256)
        },
    };
    let solid = Manifold::revolve(&mapped, segs, angle);
    checked(&solid, "revolve")?;

    // The kernel builds (r·cosφ, r·sinφ, along): its revolution axis is the
    // local Z axis. Our axes are in the sketch plane, so rotate the frame:
    //   around X: (u, v, w) → (w, u, v)      (proper rotation)
    //   around Y: (u, v, w) → (u, w, v)      (reflection → reverse winding)
    let mut mesh = from_manifold(&solid);
    match axis {
        RevolveAxis::X => {
            for p in &mut mesh.positions {
                *p = [p[2], p[0], p[1]];
            }
            for n in &mut mesh.normals {
                *n = [n[2], n[0], n[1]];
            }
        },
        RevolveAxis::Y => {
            for p in &mut mesh.positions {
                *p = [p[0], p[2], p[1]];
            }
            for n in &mut mesh.normals {
                *n = [n[0], n[2], n[1]];
            }
            for tri in mesh.indices.chunks_exact_mut(3) {
                tri.swap(1, 2);
            }
        },
    }
    Ok((mesh, solid.volume()))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn rect(x0: f64, y0: f64, x1: f64, y1: f64) -> Vec<[f64; 2]> {
        vec![[x0, y0], [x1, y0], [x1, y1], [x0, y1]]
    }

    fn bbox(m: &Mesh) -> ([f32; 3], [f32; 3]) {
        let mut lo = [f32::MAX; 3];
        let mut hi = [f32::MIN; 3];
        for p in &m.positions {
            for i in 0..3 {
                lo[i] = lo[i].min(p[i]);
                hi[i] = hi[i].max(p[i]);
            }
        }
        (lo, hi)
    }

    #[test]
    fn extrude_rectangle_has_expected_volume_and_extent() {
        let (mesh, volume) = extrude(&[rect(0.0, 0.0, 40.0, 30.0)], 5.0).unwrap();
        assert!((volume - 40.0 * 30.0 * 5.0).abs() < 1e-6, "volume {volume}");
        let (lo, hi) = bbox(&mesh);
        assert!((lo[2] - 0.0).abs() < 1e-6 && (hi[2] - 5.0).abs() < 1e-6, "z {lo:?}..{hi:?}");
        assert!(!mesh.indices.is_empty());
    }

    #[test]
    fn negative_extrude_mirrors_across_the_plane() {
        let (mesh, volume) = extrude(&[rect(0.0, 0.0, 10.0, 10.0)], -3.0).unwrap();
        assert!((volume - 300.0).abs() < 1e-6);
        let (lo, hi) = bbox(&mesh);
        assert!((lo[2] + 3.0).abs() < 1e-6 && hi[2].abs() < 1e-6, "z {lo:?}..{hi:?}");
    }

    #[test]
    fn extrude_handles_nested_holes_either_orientation() {
        // Outer square and an inner square drawn clockwise (opposite order):
        // even-odd still turns the inner loop into a hole.
        let outer = rect(0.0, 0.0, 40.0, 40.0);
        let inner = rect(10.0, 10.0, 30.0, 30.0).into_iter().rev().collect::<Vec<_>>();
        let (_, volume) = extrude(&[outer, inner], 2.0).unwrap();
        let expected = (40.0 * 40.0 - 20.0 * 20.0) * 2.0;
        assert!((volume - expected).abs() < 1e-6, "volume {volume} vs {expected}");
    }

    #[test]
    fn revolve_rectangle_off_axis_is_a_tube() {
        // y in 5..10 → radii 5..10, x in 0..20 → length 20.
        let (_, volume) = revolve(&[rect(0.0, 5.0, 20.0, 10.0)], 360.0, RevolveAxis::X, Some(256)).unwrap();
        let expected = std::f64::consts::PI * (10.0f64.powi(2) - 5.0f64.powi(2)) * 20.0;
        assert!((volume - expected).abs() / expected < 0.005, "volume {volume} vs {expected}");
    }

    #[test]
    fn partial_revolve_is_a_fraction_of_the_full_one() {
        let (_, full) = revolve(&[rect(0.0, 5.0, 20.0, 10.0)], 360.0, RevolveAxis::X, Some(128)).unwrap();
        let (_, quarter) = revolve(&[rect(0.0, 5.0, 20.0, 10.0)], 90.0, RevolveAxis::X, Some(128)).unwrap();
        assert!((quarter - full / 4.0).abs() / full < 0.01, "quarter {quarter}, full {full}");
    }

    #[test]
    fn revolve_axis_lands_in_the_sketch_plane() {
        // Around X: the tube runs along X (y/z are the radius directions).
        let (m, _) = revolve(&[rect(0.0, 5.0, 20.0, 10.0)], 360.0, RevolveAxis::X, Some(64)).unwrap();
        let (lo, hi) = bbox(&m);
        assert!((lo[0] - 0.0).abs() < 1e-4 && (hi[0] - 20.0).abs() < 1e-4, "x {lo:?}..{hi:?}");
        assert!((lo[1] + 10.0).abs() < 1e-3 && (hi[1] - 10.0).abs() < 1e-3, "y {lo:?}..{hi:?}");
        assert!((lo[2] + 10.0).abs() < 1e-3 && (hi[2] - 10.0).abs() < 1e-3, "z {lo:?}..{hi:?}");

        // Around Y: the tube runs along Y instead.
        let (m, _) = revolve(&[rect(5.0, 0.0, 10.0, 20.0)], 360.0, RevolveAxis::Y, Some(64)).unwrap();
        let (lo, hi) = bbox(&m);
        assert!((lo[1] - 0.0).abs() < 1e-4 && (hi[1] - 20.0).abs() < 1e-4, "y {lo:?}..{hi:?}");
        assert!((lo[0] + 10.0).abs() < 1e-3 && (hi[0] - 10.0).abs() < 1e-3, "x {lo:?}..{hi:?}");
        assert!((lo[2] + 10.0).abs() < 1e-3 && (hi[2] - 10.0).abs() < 1e-3, "z {lo:?}..{hi:?}");
    }

    #[test]
    fn revolve_profile_crossing_the_axis_is_rejected() {
        let err = revolve(&[rect(0.0, -5.0, 20.0, 10.0)], 360.0, RevolveAxis::X, None).unwrap_err();
        assert!(err.to_string().contains("crosses the revolve axis"), "{err}");
    }

    #[test]
    fn revolve_profile_on_the_negative_side_is_mirrored() {
        let (_, volume) = revolve(&[rect(0.0, -10.0, 20.0, -5.0)], 360.0, RevolveAxis::X, Some(256)).unwrap();
        let expected = std::f64::consts::PI * (10.0f64.powi(2) - 5.0f64.powi(2)) * 20.0;
        assert!((volume - expected).abs() / expected < 0.005, "volume {volume}");
    }
}
