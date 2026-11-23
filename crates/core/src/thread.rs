//! Threads: a core cylinder with a real helical V-ridge.
//!
//! This is the helical sweep specialized to the shape a screw needs. In the
//! (radial, axial) plane the tooth is a triangle from the minor radius to the
//! major radius; sweeping it around the axis with one `pitch` of axial advance
//! per turn produces the thread. The result is unioned with the minor-radius
//! core and clipped to the requested length, so both ends are flat.
//!
//! An external thread is the solid a screw occupies. Subtracting it from a
//! body produces the matching threaded hole — the same shape, used as a
//! cutter. That is why `thread` has no separate internal mode: `boolean`
//! difference is the internal mode.
//!
//! `hand` flips the direction of the helix; `flank_angle_deg` is the included
//! angle between the two flanks (60° for ISO metric threads).

use manifold_rust::manifold::Manifold;

use agcad_protocol::{Hand, Mesh, Vec3};

use crate::boolean::{from_manifold, to_manifold};
use crate::fillet::signed_volume;
use crate::sweep::in_plane_x;
use crate::Error;

/// Resolved thread arguments.
#[derive(Debug, Clone, Copy)]
pub struct ThreadSpec {
    pub major_diameter: f64,
    pub minor_diameter: f64,
    pub pitch: f64,
    pub length: f64,
    pub hand: Hand,
    pub flank_angle_deg: f64,
    pub segments_per_turn: u32,
    pub origin: Vec3,
}

fn cross(a: Vec3, b: Vec3) -> Vec3 {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

fn mul(v: Vec3, s: f64) -> Vec3 {
    [v[0] * s, v[1] * s, v[2] * s]
}

fn add(a: Vec3, b: Vec3) -> Vec3 {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

/// The namesake shape: a threaded solid, axis +Z, from `origin` up by `length`.
pub fn thread(spec: &ThreadSpec) -> Result<Mesh, Error> {
    let p = spec.pitch;
    let length = spec.length;
    let major = spec.major_diameter;
    let minor = spec.minor_diameter;
    if !(major.is_finite() && major > 0.0) {
        return Err(Error::Invalid(format!("thread major_diameter must be > 0 (got {major})")));
    }
    if !(minor.is_finite() && minor > 0.0 && minor < major) {
        return Err(Error::Invalid(format!(
            "thread minor_diameter must be > 0 and smaller than the major diameter (got \
             {minor} for major {major})"
        )));
    }
    if !(p.is_finite() && p > 0.0) {
        return Err(Error::Invalid(format!("thread pitch must be > 0 (got {p})")));
    }
    if !(length.is_finite() && length > 0.0) {
        return Err(Error::Invalid(format!("thread length must be > 0 (got {length})")));
    }
    if !(spec.flank_angle_deg.is_finite()
        && spec.flank_angle_deg > 0.0
        && spec.flank_angle_deg < 180.0)
    {
        return Err(Error::Invalid(
            "thread flank_angle_deg must be the included angle in (0, 180) degrees".into(),
        ));
    }

    let r_major = major * 0.5;
    let r_minor = minor * 0.5;
    let depth = r_major - r_minor;
    let half_flank = spec.flank_angle_deg.to_radians() * 0.5;
    let half_width = (depth * half_flank.tan()).min(p * 0.49);
    let per_turn = spec.segments_per_turn.clamp(12, 360);
    let turns = length / p;
    let total = ((turns * per_turn as f64).ceil() as u32).max(2);
    if total > 4096 {
        return Err(Error::Invalid(format!(
            "thread would use {total} stations (limit 4096; hint: increase the pitch or reduce \
             segments_per_turn)"
        )));
    }

    let axis = [0.0, 0.0, 1.0];
    let x_axis = in_plane_x(axis);
    let y_axis = cross(axis, x_axis);
    let sign = spec.hand.sign();

    // (radius, axial offset from the station center), in CCW order. The base
    // sinks slightly into the core so the union is a proper overlap instead of
    // two surfaces that only touch — a coincident-face boolean is degenerate.
    let r_base = (r_minor - (p * 0.1).min(depth * 0.5).max(1e-3)).max(0.05);
    let profile = [(r_base, -half_width), (r_major, 0.0), (r_base, half_width)];

    let mut positions: Vec<[f32; 3]> = Vec::with_capacity((total as usize + 1) * 3);
    for i in 0..=total {
        let frac = i as f64 / per_turn as f64;
        let theta = sign * frac * std::f64::consts::TAU;
        let (s, c) = theta.sin_cos();
        let radial = add(mul(x_axis, c), mul(y_axis, s));
        let center = add(spec.origin, mul(axis, frac * p));
        for (r, z) in profile {
            let pnt = add(add(center, mul(radial, r)), mul(axis, z));
            positions.push([pnt[0] as f32, pnt[1] as f32, pnt[2] as f32]);
        }
    }

    let mut indices: Vec<u32> = Vec::new();
    let ring = |i: u32| i * 3;
    for i in 0..total {
        for j in 0..3u32 {
            let a = ring(i) + j;
            let b = ring(i) + (j + 1) % 3;
            let c = ring(i + 1) + (j + 1) % 3;
            let d = ring(i + 1) + j;
            indices.extend_from_slice(&[a, b, c, a, c, d]);
        }
    }
    let last = total * 3;
    // The profile is CCW viewed along the sweep tangent, so the start cap is
    // the "bottom" (reverse order) and the end cap is the "top".
    indices.extend_from_slice(&[0, 2, 1]);
    indices.extend_from_slice(&[last, last + 1, last + 2]);

    let mut ridge = Mesh { positions, normals: Vec::new(), indices };
    if signed_volume(&ridge) < 0.0 {
        for tri in ridge.indices.chunks_exact_mut(3) {
            tri.swap(1, 2);
        }
    }
    let ridge = to_manifold(&ridge).map_err(|e| {
        Error::Invalid(format!(
            "thread ridge is not a valid solid: {e} (hint: increase the pitch or decrease the \
             thread depth; the flanks must not cross)"
        ))
    })?;

    let side = per_turn.max(24) as i32;
    let core = Manifold::cylinder(length, r_minor, -1.0, side);
    let clip = Manifold::cylinder(length, r_major + p, -1.0, side)
        .translate(manifold_rust::linalg::Vec3::new(spec.origin[0], spec.origin[1], spec.origin[2]));
    let core = core.translate(manifold_rust::linalg::Vec3::new(
        spec.origin[0],
        spec.origin[1],
        spec.origin[2],
    ));
    let solid = ridge.union(&core).intersection(&clip);
    if solid.num_tri() == 0 {
        return Err(Error::Invalid("thread produced no geometry".into()));
    }
    let status = solid.status();
    if status != manifold_rust::types::Error::NoError {
        return Err(Error::Invalid(format!("thread failed in the kernel: {status:?}")));
    }
    Ok(from_manifold(&solid))
}

/// ISO metric default: the minor diameter is `major − 1.0825 · pitch`.
pub fn default_minor(major: f64, pitch: f64) -> f64 {
    major - 1.0825 * pitch
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn an_m8_thread_is_watertight_and_bigger_than_its_core() {
        let spec = ThreadSpec {
            major_diameter: 8.0,
            minor_diameter: default_minor(8.0, 1.25),
            pitch: 1.25,
            length: 10.0,
            hand: Hand::Right,
            flank_angle_deg: 60.0,
            segments_per_turn: 48,
            origin: [0.0; 3],
        };
        let m = thread(&spec).unwrap();
        let check = crate::validate::check_solid(&m);
        assert!(check.watertight && check.non_manifold_edges == 0, "{check:?}");
        let core = std::f64::consts::PI * (spec.minor_diameter * 0.5).powi(2) * spec.length;
        assert!(check.volume > core * 0.95, "volume {} vs core {core}", check.volume);
        let bbox = check.bbox;
        assert!((bbox.max[2] - bbox.min[2] - 10.0).abs() < 1e-3, "{bbox:?}");
    }

    #[test]
    fn left_hand_mirrors_the_ridge_not_the_shape() {
        let mut spec = ThreadSpec {
            major_diameter: 10.0,
            minor_diameter: default_minor(10.0, 1.5),
            pitch: 1.5,
            length: 6.0,
            hand: Hand::Right,
            flank_angle_deg: 60.0,
            segments_per_turn: 24,
            origin: [0.0; 3],
        };
        let right = thread(&spec).unwrap();
        spec.hand = Hand::Left;
        let left = thread(&spec).unwrap();
        assert_eq!(right.indices.len(), left.indices.len());
        assert!(
            (signed_volume(&right) - signed_volume(&left)).abs() < 1e-6,
            "hand must not change the volume"
        );
    }

    #[test]
    fn subtracting_a_thread_makes_a_threaded_hole() {
        let mut block = crate::geo::box_mesh([20.0, 20.0, 10.0]);
        for p in &mut block.positions {
            p[2] += 5.0;
        }
        let spec = ThreadSpec {
            major_diameter: 8.0,
            minor_diameter: default_minor(8.0, 1.25),
            pitch: 1.25,
            length: 14.0,
            hand: Hand::Right,
            flank_angle_deg: 60.0,
            segments_per_turn: 32,
            origin: [0.0, 0.0, -2.0],
        };
        let bolt = thread(&spec).unwrap();
        let cut = crate::boolean::boolean(
            agcad_protocol::BooleanOp::Difference,
            &[block.clone(), bolt],
        )
        .unwrap();
        let check = crate::validate::check_solid(&cut);
        assert!(check.watertight && check.non_manifold_edges == 0, "{check:?}");
        assert!(check.volume < signed_volume(&block) - 200.0);
    }
}
