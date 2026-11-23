//! Holes: cylindrical through/blind holes with counterbore and countersink
//! seats, cut into a **world-space** solid.
//!
//! The cutters are built with Manifold's cylinder/cone constructor, oriented
//! from the surface normal, and subtracted in one boolean. Everything starts
//! `overcut` outside the entry point so the cutter always breaks through the
//! surface instead of leaving a hairline skin; a through-hole overcuts the far
//! side too. The result is the same kind of in-place reshape as a fillet.

use manifold_rust::linalg::{Mat3x4, Vec3 as MVec3};
use manifold_rust::manifold::Manifold;

use wright_protocol::{BooleanOp, Mesh, Vec3};

use crate::boolean;
use crate::Error;

/// The resolved arguments of one hole.
#[derive(Debug, Clone, Copy)]
pub struct HoleSpec {
    /// A point on the entry surface (world space).
    pub center: Vec3,
    /// Outward surface normal at the entry point.
    pub normal: Vec3,
    pub diameter: f64,
    /// `None` or a non-positive value drills through the whole body.
    pub depth: Option<f64>,
    pub counterbore_diameter: Option<f64>,
    pub counterbore_depth: Option<f64>,
    /// Included countersink angle in degrees.
    pub countersink_angle_deg: Option<f64>,
    pub segments: Option<u32>,
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

/// A right-handed frame whose Z is `axis`, plus its X (deterministic).
fn frame(axis: Vec3) -> (Vec3, Vec3, Vec3) {
    let z = normalize(axis);
    let mut helper = [1.0, 0.0, 0.0];
    if dot(z, helper).abs() > 0.9 {
        helper = [0.0, 1.0, 0.0];
    }
    let x = normalize([
        helper[0] - z[0] * dot(helper, z),
        helper[1] - z[1] * dot(helper, z),
        helper[2] - z[2] * dot(helper, z),
    ]);
    (x, cross(z, x), z)
}

fn v3(v: Vec3) -> MVec3 {
    MVec3::new(v[0], v[1], v[2])
}

/// A cone/cylinder of `length` starting at `start`, extruded along `dir`
/// (which is a unit vector), with the given end radii.
fn piece(
    start: Vec3,
    dir: Vec3,
    length: f64,
    r_low: f64,
    r_high: f64,
    segments: i32,
) -> Manifold {
    let (x, y, z) = frame(dir);
    let transform = Mat3x4::from_cols(v3(x), v3(y), v3(z), v3(start));
    Manifold::cylinder(length, r_low, r_high, segments).transform(&transform)
}

fn segments_for(r: f64, requested: Option<u32>) -> i32 {
    match requested {
        Some(n) => n.clamp(8, 512) as i32,
        None => ((r * 80.0).sqrt().ceil() as i32).clamp(16, 128),
    }
}

/// Cut the hole into a world-space body.
pub fn drill(body: &Mesh, spec: &HoleSpec) -> Result<Mesh, Error> {
    let normal = normalize(spec.normal);
    if dot(normal, normal) < 0.5 {
        return Err(Error::Invalid(
            "hole normal must be a non-zero vector (e.g. [0,0,1] for the top face)".into(),
        ));
    }
    if !(spec.diameter.is_finite() && spec.diameter > 0.0) {
        return Err(Error::Invalid(format!(
            "hole diameter must be a finite value > 0 (got {})",
            spec.diameter
        )));
    }
    let r = spec.diameter * 0.5;
    let diag = bbox_diagonal(body).max(1.0);
    let overcut = (diag * 0.02).max(0.5);
    let start = [
        spec.center[0] + normal[0] * overcut,
        spec.center[1] + normal[1] * overcut,
        spec.center[2] + normal[2] * overcut,
    ];
    let dir = [-normal[0], -normal[1], -normal[2]];
    let depth = match spec.depth {
        Some(d) if d.is_finite() && d > 0.0 => d,
        Some(d) if !d.is_finite() => {
            return Err(Error::Invalid("hole depth must be finite".into()));
        },
        _ => diag * 2.0,
    };
    let length = depth + overcut;
    let segments = segments_for(r, spec.segments);

    if spec.counterbore_diameter.is_none()
        && (spec.counterbore_depth.is_some() || spec.countersink_angle_deg.is_some())
    {
        return Err(Error::Invalid(
            "a counterbore/countersink needs counterbore_diameter (the larger seat diameter)"
                .into(),
        ));
    }

    let mut cutters: Vec<Mesh> = Vec::new();
    cutters.push(boolean::from_manifold(&piece(start, dir, length, r, r, segments)));

    match (spec.counterbore_diameter, spec.counterbore_depth, spec.countersink_angle_deg) {
        (Some(cb_d), _, Some(angle)) => {
            if !(cb_d.is_finite() && cb_d > spec.diameter) {
                return Err(Error::Invalid(format!(
                    "countersink diameter must be larger than the hole diameter ({})",
                    spec.diameter
                )));
            }
            if !(angle.is_finite() && angle > 0.0 && angle < 180.0) {
                return Err(Error::Invalid(
                    "countersink_angle_deg must be the included angle in (0, 180) degrees".into(),
                ));
            }
            let tan_half = (angle.to_radians() * 0.5).tan();
            let cb_r = cb_d * 0.5;
            let cone_h = (cb_r - r) / tan_half;
            if cone_h <= 1e-9 {
                return Err(Error::Invalid(
                    "countersink has no depth: increase counterbore_diameter".into(),
                ));
            }
            // Start the cone outside the surface so the cut breaks through
            // cleanly; at the surface its radius is exactly cb_r.
            let r_start = cb_r + overcut * tan_half;
            let cone_len = cone_h + overcut;
            let cone = piece(start, dir, cone_len, r_start, r, segments);
            cutters.push(boolean::from_manifold(&cone));
        },
        (Some(cb_d), Some(cb_depth), None) => {
            if !(cb_d.is_finite() && cb_d > spec.diameter) {
                return Err(Error::Invalid(format!(
                    "counterbore diameter must be larger than the hole diameter ({})",
                    spec.diameter
                )));
            }
            if !(cb_depth.is_finite() && cb_depth > 0.0) {
                return Err(Error::Invalid(
                    "counterbore_depth must be a finite value > 0".into(),
                ));
            }
            let cb_len = cb_depth + overcut;
            cutters.push(boolean::from_manifold(&piece(
                start,
                dir,
                cb_len,
                cb_d * 0.5,
                cb_d * 0.5,
                segments,
            )));
        },
        (Some(_), None, None) => {
            return Err(Error::Invalid(
                "counterbore_diameter needs counterbore_depth (cylindrical seat) or \
                 countersink_angle_deg (conical seat)"
                    .into(),
            ));
        },
        _ => {},
    }

    let mut operands = Vec::with_capacity(cutters.len() + 1);
    operands.push(body.clone());
    operands.extend(cutters);
    boolean::boolean(BooleanOp::Difference, &operands).map_err(|e| {
        Error::Invalid(format!(
            "hole failed: {e} (hint: the hole must intersect the solid; check center and normal \
             against validate/get_scene)"
        ))
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::fillet::signed_volume;
    use crate::geo;

    fn block() -> Mesh {
        let mut m = geo::box_mesh([40.0, 40.0, 10.0]);
        for p in &mut m.positions {
            p[1] += 0.0;
        }
        m
    }

    #[test]
    fn through_hole_removes_a_cylinder() {
        let body = block();
        let out = drill(
            &body,
            &HoleSpec {
                center: [0.0, 0.0, 5.0],
                normal: [0.0, 0.0, 1.0],
                diameter: 8.0,
                depth: None,
                counterbore_diameter: None,
                counterbore_depth: None,
                countersink_angle_deg: None,
                segments: Some(64),
            },
        )
        .unwrap();
        let removed = 40.0 * 40.0 * 10.0 - signed_volume(&out);
        let expected = std::f64::consts::PI * 16.0 * 10.0;
        assert!((removed - expected).abs() / expected < 0.02, "removed {removed} vs {expected}");
        let check = crate::validate::check_solid(&out);
        assert!(check.watertight && check.non_manifold_edges == 0, "{check:?}");
    }

    #[test]
    fn counterbore_removes_more_than_a_simple_hole() {        let body = block();
        let simple = drill(
            &body,
            &HoleSpec {
                center: [0.0, 0.0, 5.0],
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
        let cb = drill(
            &body,
            &HoleSpec {
                center: [0.0, 0.0, 5.0],
                normal: [0.0, 0.0, 1.0],
                diameter: 6.0,
                depth: None,
                counterbore_diameter: Some(11.0),
                counterbore_depth: Some(3.0),
                countersink_angle_deg: None,
                segments: Some(48),
            },
        )
        .unwrap();
        assert!(signed_volume(&cb) < signed_volume(&simple) - 100.0);
    }

    #[test]
    fn countersink_is_a_cone_and_stays_printable() {
        let body = block();
        let spec = |countersink_angle_deg| HoleSpec {
            center: [0.0, 0.0, 5.0],
            normal: [0.0, 0.0, 1.0],
            diameter: 6.0,
            depth: None,
            counterbore_diameter: Some(12.0),
            counterbore_depth: None,
            countersink_angle_deg,
            segments: Some(64),
        };
        let simple = drill(
            &body,
            &HoleSpec {
                center: [0.0, 0.0, 5.0],
                normal: [0.0, 0.0, 1.0],
                diameter: 6.0,
                depth: None,
                counterbore_diameter: None,
                counterbore_depth: None,
                countersink_angle_deg: None,
                segments: Some(64),
            },
        )
        .unwrap();
        let csk = drill(&body, &spec(Some(90.0))).unwrap();
        assert!(signed_volume(&csk) < signed_volume(&simple) - 10.0);
        let check = crate::validate::check_solid(&csk);
        assert!(check.watertight && check.non_manifold_edges == 0, "{check:?}");
        assert!(drill(&body, &spec(None)).is_err(), "a seat needs depth or an angle");
    }

    #[test]
    fn blind_holes_stop_inside_the_material() {        let body = block();
        let blind = drill(
            &body,
            &HoleSpec {
                center: [0.0, 0.0, 5.0],
                normal: [0.0, 0.0, 1.0],
                diameter: 6.0,
                depth: Some(4.0),
                counterbore_diameter: None,
                counterbore_depth: None,
                countersink_angle_deg: None,
                segments: Some(48),
            },
        )
        .unwrap();
        let removed = 40.0 * 40.0 * 10.0 - signed_volume(&blind);
        let expected = std::f64::consts::PI * 9.0 * 4.0;
        assert!((removed - expected).abs() / expected < 0.03, "removed {removed} vs {expected}");
    }

    #[test]
    fn a_side_hole_follows_the_outward_normal() {
        let body = block();
        let out = drill(
            &body,
            &HoleSpec {
                center: [20.0, 0.0, 0.0],
                normal: [1.0, 0.0, 0.0],
                diameter: 8.0,
                depth: None,
                counterbore_diameter: None,
                counterbore_depth: None,
                countersink_angle_deg: None,
                segments: Some(64),
            },
        )
        .unwrap();
        let removed = 40.0 * 40.0 * 10.0 - signed_volume(&out);
        let expected = std::f64::consts::PI * 16.0 * 40.0;
        assert!((removed - expected).abs() / expected < 0.03, "removed {removed} vs {expected}");
        let check = crate::validate::check_solid(&out);
        assert!(check.watertight && check.non_manifold_edges == 0, "{check:?}");
    }
}
