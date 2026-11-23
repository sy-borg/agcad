//! Sweeps: a closed sketch profile carried along a 3D path.
//!
//! The solid is the union of one prism per path segment. Each prism is the
//! normalized profile extruded along its segment, oriented so the profile
//! plane is perpendicular to the path. Overlapping prisms union cleanly, so a
//! bend does not need a miter and a helix needs no special-casing: the helix is
//! just a path generator.
//!
//! This is deliberately the same machinery for a polyline and a helix, which
//! is what makes a spring, a worm and the ridge of a screw thread one shape.
//! The `thread` feature goes further and sweeps a V-profile with a pitch
//! advance around the axis directly; a generic sweep is the flexible tool.

use manifold_rust::linalg::{Mat3x4, Vec2, Vec3 as MVec3};
use manifold_rust::manifold::Manifold;

use agcad_protocol::{Hand, Mesh, Scalar, SweepPath, Vec3};

use crate::boolean::from_manifold;
use crate::{solid, Error};

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

fn sub(a: Vec3, b: Vec3) -> Vec3 {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn mul(v: Vec3, s: f64) -> Vec3 {
    [v[0] * s, v[1] * s, v[2] * s]
}

fn add(a: Vec3, b: Vec3) -> Vec3 {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

/// A deterministic in-plane X for a normal (world +X, or +Y when parallel).
pub fn in_plane_x(normal: Vec3) -> Vec3 {
    let mut helper = [1.0, 0.0, 0.0];
    if dot(normal, helper).abs() > 0.9 {
        helper = [0.0, 1.0, 0.0];
    }
    normalize(sub(helper, mul(normal, dot(helper, normal))))
}

/// Resolve a sweep path into world-space stations.
///
/// `resolve` turns a `Scalar` into a number (the document supplies it).
pub fn path_points(
    path: &SweepPath,
    segments: Option<u32>,
    resolve: impl Fn(&Scalar) -> Result<f64, Error>,
) -> Result<Vec<Vec3>, Error> {
    match path {
        SweepPath::Polyline { points, closed } => {
            if points.len() < 2 {
                return Err(Error::Invalid(
                    "sweep polyline needs at least 2 points (hint: the profile is placed at each \
                     station, perpendicular to the path)"
                        .into(),
                ));
            }
            let mut out = Vec::with_capacity(points.len() + usize::from(*closed));
            for p in points {
                out.push([resolve(&p[0])?, resolve(&p[1])?, resolve(&p[2])?]);
            }
            if *closed {
                out.push(out[0]);
            }
            Ok(out)
        },
        SweepPath::Helix {
            origin,
            axis,
            radius,
            pitch,
            turns,
            hand,
            segments_per_turn,
        } => {
            let origin = match origin {
                Some(o) => [resolve(&o[0])?, resolve(&o[1])?, resolve(&o[2])?],
                None => [0.0; 3],
            };
            let axis = axis.map(|a| normalize(a)).unwrap_or([0.0, 0.0, 1.0]);
            if dot(axis, axis) < 0.5 {
                return Err(Error::Invalid(
                    "helix axis must be a non-zero vector (e.g. [0,0,1])".into(),
                ));
            }
            let radius = resolve(radius)?;
            let pitch = resolve(pitch)?;
            let turns = resolve(turns)?;
            if !(radius.is_finite() && radius > 0.0) {
                return Err(Error::Invalid(format!("helix radius must be > 0 (got {radius})")));
            }
            if !(pitch.is_finite() && pitch > 0.0) {
                return Err(Error::Invalid(format!("helix pitch must be > 0 (got {pitch})")));
            }
            if !(turns.is_finite() && turns > 0.0 && turns <= 1000.0) {
                return Err(Error::Invalid(format!(
                    "helix turns must be in (0, 1000] (got {turns})"
                )));
            }
            let per_turn = segments
                .or(*segments_per_turn)
                .map(|n| n.clamp(6, 360))
                .unwrap_or(32);
            let total = (turns * per_turn as f64).round().max(2.0) as u32;
            if total > 2048 {
                return Err(Error::Invalid(format!(
                    "helix sweep would use {total} stations (limit 2048; hint: reduce turns or \
                     segments_per_turn)"
                )));
            }
            let x = in_plane_x(axis);
            let y = cross(axis, x);
            let sign = hand.sign();
            let mut out = Vec::with_capacity(total as usize + 1);
            for i in 0..=total {
                let frac = i as f64 / per_turn as f64; // turns completed
                let theta = sign * frac * std::f64::consts::TAU;
                let (s, c) = theta.sin_cos();
                let radial = add(mul(x, c * radius), mul(y, s * radius));
                out.push(add(add(origin, radial), mul(axis, pitch * frac)));
            }
            Ok(out)
        },
    }
}

/// Sweep `loops` (2D, sketch-local) along `path` (world), orienting the
/// profile's local X by `x_ref` (a world direction from the sketch).
pub fn sweep(
    loops: &[Vec<[f64; 2]>],
    x_ref: Vec3,
    path: &[Vec3],
) -> Result<Mesh, Error> {
    if path.len() < 2 {
        return Err(Error::Invalid("sweep path needs at least 2 points".into()));
    }
    let polys = solid::normalize(loops)?;
    let x_ref = normalize(x_ref);
    if dot(x_ref, x_ref) < 0.5 {
        return Err(Error::Invalid(
            "sweep profile has no usable in-plane X direction".into(),
        ));
    }

    let mut acc: Option<Manifold> = None;
    for pair in path.windows(2) {
        let dir = normalize(sub(pair[1], pair[0]));
        let len = dot(sub(pair[1], pair[0]), sub(pair[1], pair[0])).sqrt();
        if len < 1e-9 {
            continue;
        }
        if dot(dir, dir) < 0.5 {
            continue;
        }
        // Project the sketch's world X onto the plane normal to this segment.
        let mut x = sub(x_ref, mul(dir, dot(x_ref, dir)));
        if dot(x, x).sqrt() < 1e-9 {
            x = in_plane_x(dir);
        }
        let x = normalize(x);
        let y = cross(dir, x);
        let prism = Manifold::extrude(&polys, len, 0, 0.0, Vec2::new(1.0, 1.0));
        if prism.num_tri() == 0 {
            return Err(Error::Invalid(
                "sweep produced no geometry (hint: the profile must enclose area)".into(),
            ));
        }
        let transform = Mat3x4::from_cols(
            MVec3::new(x[0], x[1], x[2]),
            MVec3::new(y[0], y[1], y[2]),
            MVec3::new(dir[0], dir[1], dir[2]),
            MVec3::new(pair[0][0], pair[0][1], pair[0][2]),
        );
        let placed = prism.transform(&transform);
        acc = Some(match acc {
            Some(acc) => acc.union(&placed),
            None => placed,
        });
    }
    let Some(result) = acc else {
        return Err(Error::Invalid(
            "sweep path has no non-zero segment".into(),
        ));
    };
    if result.num_tri() == 0 {
        return Err(Error::Invalid("sweep produced no geometry".into()));
    }
    Ok(from_manifold(&result))
}

/// A helix path used by the thread builder: stations along the helix with the
/// radius and axial advance. Kept here so the sweep and the thread agree on
/// the frame and hand.
pub fn helix_stations(
    origin: Vec3,
    axis: Vec3,
    radius: f64,
    pitch: f64,
    turns: f64,
    hand: Hand,
    per_turn: u32,
) -> Vec<Vec3> {
    let axis = normalize(axis);
    let x = in_plane_x(axis);
    let y = cross(axis, x);
    let sign = hand.sign();
    let total = (turns * per_turn as f64).round().max(2.0) as u32;
    (0..=total)
        .map(|i| {
            let frac = i as f64 / per_turn as f64;
            let theta = sign * frac * std::f64::consts::TAU;
            let (s, c) = theta.sin_cos();
            add(
                add(origin, add(mul(x, c * radius), mul(y, s * radius))),
                mul(axis, pitch * frac),
            )
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::fillet::signed_volume;
    use crate::geo;

    fn square(half: f64) -> Vec<[f64; 2]> {
        vec![[-half, -half], [half, -half], [half, half], [-half, half]]
    }

    #[test]
    fn a_straight_sweep_is_an_extrusion() {
        let path = vec![[0.0, 0.0, 0.0], [0.0, 0.0, 20.0]];
        let m = sweep(&[square(5.0)], [1.0, 0.0, 0.0], &path).unwrap();
        let v = signed_volume(&m);
        assert!((v - 100.0 * 20.0).abs() / (100.0 * 20.0) < 1e-4, "volume {v}");
        let check = crate::validate::check_solid(&m);
        assert!(check.watertight && check.non_manifold_edges == 0, "{check:?}");
    }

    #[test]
    fn a_helix_has_the_requested_radius_and_rise() {
        let pts = helix_stations([0.0; 3], [0.0, 0.0, 1.0], 10.0, 5.0, 2.0, Hand::Right, 24);
        let last = *pts.last().unwrap();
        assert!((last[2] - 10.0).abs() < 1e-9, "{last:?}");
        let radius = (last[0] * last[0] + last[1] * last[1]).sqrt();
        assert!((radius - 10.0).abs() < 1e-9, "{last:?}");
        assert!(pts.len() > 40);
    }

    #[test]
    fn a_helical_sweep_is_closed() {
        let path = helix_stations([0.0; 3], [0.0, 0.0, 1.0], 12.0, 8.0, 1.0, Hand::Right, 16);
        let m = sweep(&[square(2.0)], [1.0, 0.0, 0.0], &path).unwrap();
        let check = crate::validate::check_solid(&m);
        assert!(check.watertight && check.non_manifold_edges == 0, "{check:?}");
        assert!(check.volume > 0.0);
    }

    #[test]
    fn a_circle_profile_makes_a_spring_like_tube() {
        let mut circle = Vec::new();
        for i in 0..24 {
            let t = i as f64 * std::f64::consts::TAU / 24.0;
            circle.push([1.0 * t.cos(), 1.0 * t.sin()]);
        }
        let path = helix_stations([0.0; 3], [0.0, 0.0, 1.0], 10.0, 6.0, 1.0, Hand::Right, 16);
        let m = sweep(&[circle], [1.0, 0.0, 0.0], &path).unwrap();
        let check = crate::validate::check_solid(&m);
        assert!(check.watertight && check.non_manifold_edges == 0, "{check:?}");
        let _ = geo::box_mesh([1.0; 3]);
    }
}
