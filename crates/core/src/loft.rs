//! Lofts: a solid whose cross-section morphs between closed profiles.
//!
//! Each profile is resampled by arc length to a common vertex count and rotated
//! to the best cyclic correspondence with the previous profile, so a square
//! lofts into a circle without a twisted seam. The side is a quad strip; the
//! ends are ear-clipped caps, so non-convex profiles (a star, an L) still cap
//! correctly. Holes are not lofted — one closed outer loop per sketch.
//!
//! The result is imported through Manifold's strict mesher, which is what
//! turns a malformed connection (a self-crossing morph) into an actionable
//! error instead of a broken solid.

use agcad_protocol::{Mesh, Vec3};

use crate::{boolean, fillet, Error};

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

fn normalize(v: Vec3) -> Vec3 {
    let len = dot(v, v).sqrt();
    if len < 1e-12 { [0.0; 3] } else { mul(v, 1.0 / len) }
}

fn centroid(pts: &[Vec3]) -> Vec3 {
    let mut c = [0.0; 3];
    for p in pts {
        for i in 0..3 {
            c[i] += p[i];
        }
    }
    mul(c, 1.0 / pts.len().max(1) as f64)
}

fn newell(pts: &[Vec3]) -> Vec3 {
    let mut n = [0.0; 3];
    for i in 0..pts.len() {
        let a = pts[i];
        let b = pts[(i + 1) % pts.len()];
        n[0] += (a[1] - b[1]) * (a[2] + b[2]);
        n[1] += (a[2] - b[2]) * (a[0] + b[0]);
        n[2] += (a[0] - b[0]) * (a[1] + b[1]);
    }
    n
}

/// Resample a closed polyline to `n` equally spaced points by arc length.
fn resample(pts: &[Vec3], n: usize) -> Result<Vec<Vec3>, Error> {
    if pts.len() < 3 {
        return Err(Error::Invalid(
            "loft profiles need at least 3 points (hint: use a closed sketch loop)".into(),
        ));
    }
    let mut lengths = Vec::with_capacity(pts.len());
    let mut total = 0.0;
    for i in 0..pts.len() {
        let d = sub(pts[(i + 1) % pts.len()], pts[i]);
        let l = dot(d, d).sqrt();
        lengths.push(l);
        total += l;
    }
    if total < 1e-9 {
        return Err(Error::Invalid("loft profile has zero perimeter".into()));
    }
    let mut out = Vec::with_capacity(n);
    let mut seg = 0usize;
    let mut acc = 0.0;
    for i in 0..n {
        let target = total * i as f64 / n as f64;
        while seg + 1 < lengths.len() && acc + lengths[seg] < target {
            acc += lengths[seg];
            seg += 1;
        }
        let t = if lengths[seg] > 1e-12 {
            ((target - acc) / lengths[seg]).clamp(0.0, 1.0)
        } else {
            0.0
        };
        let a = pts[seg];
        let b = pts[(seg + 1) % pts.len()];
        out.push([a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t, a[2] + (b[2] - a[2]) * t]);
    }
    Ok(out)
}

/// Cyclically shift `pts` so its correspondence to `previous` is closest.
fn align(previous: &[Vec3], pts: &[Vec3]) -> Vec<Vec3> {
    let n = pts.len();
    let mut best = 0usize;
    let mut best_cost = f64::MAX;
    for shift in 0..n {
        let mut cost = 0.0;
        for i in 0..n {
            let a = previous[i];
            let b = pts[(i + shift) % n];
            cost += (a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2) + (a[2] - b[2]).powi(2);
        }
        if cost < best_cost {
            best_cost = cost;
            best = shift;
        }
    }
    (0..n).map(|i| pts[(i + best) % n]).collect()
}

/// Ear clipping for a simple CCW polygon. Falls back to a fan when the polygon
/// is self-intersecting or numerically degenerate; Manifold then rejects the
/// result with a clear message, which is better than silently capping wrong.
fn ear_clip(points: &[[f64; 2]]) -> Vec<[usize; 3]> {
    let n = points.len();
    if n < 3 {
        return Vec::new();
    }
    let mut idx: Vec<usize> = (0..n).collect();
    let area2 = |a: [f64; 2], b: [f64; 2], c: [f64; 2]| {
        (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
    };
    let inside = |p: [f64; 2], a: [f64; 2], b: [f64; 2], c: [f64; 2]| {
        let d1 = area2(a, b, p);
        let d2 = area2(b, c, p);
        let d3 = area2(c, a, p);
        d1 >= -1e-12 && d2 >= -1e-12 && d3 >= -1e-12
    };
    let mut tris = Vec::with_capacity(n.saturating_sub(2));
    let mut guard = 0usize;
    while idx.len() > 3 && guard < n * n {
        guard += 1;
        let m = idx.len();
        let mut clipped = false;
        for k in 0..m {
            let (ia, ib, ic) = (idx[(k + m - 1) % m], idx[k], idx[(k + 1) % m]);
            let (a, b, c) = (points[ia], points[ib], points[ic]);
            if area2(a, b, c) <= 1e-12 {
                continue; // reflex or collinear: not an ear
            }
            if idx
                .iter()
                .any(|&j| j != ia && j != ib && j != ic && inside(points[j], a, b, c))
            {
                continue;
            }
            tris.push([ia, ib, ic]);
            idx.remove(k);
            clipped = true;
            break;
        }
        if !clipped {
            break;
        }
    }
    if idx.len() == 3 {
        tris.push([idx[0], idx[1], idx[2]]);
    } else {
        // Fallback fan; the kernel will validate it.
        for k in 1..n.saturating_sub(1) {
            tris.push([0, k, k + 1]);
        }
    }
    tris
}

/// Build a lofted solid from world-space rings (one closed loop per profile).
pub fn loft(rings: &[Vec<Vec3>], segments: u32) -> Result<Mesh, Error> {
    if rings.len() < 2 {
        return Err(Error::Invalid(
            "loft needs at least 2 sketch profiles (hint: sketches=[a, b])".into(),
        ));
    }
    let common = rings.windows(2).all(|w| w[0].len() == w[1].len());
    let n = if common {
        rings[0].len().max(3)
    } else {
        segments.clamp(8, 512) as usize
    };
    let first = centroid(&rings[0]);
    let last = centroid(rings.last().unwrap());
    // The travel direction is "up": it fixes which side of the solid is the
    // bottom cap and keeps every ring's winding consistent with it. For
    // coincident profiles fall back to the first loop's own normal.
    let mut up = normalize(sub(last, first));
    if dot(up, up) < 0.5 {
        up = normalize(newell(&rings[0]));
    }
    if dot(up, up) < 0.5 {
        return Err(Error::Invalid(
            "loft profiles are coincident and have no usable direction (hint: place the sketches \
             on different planes or at different heights)"
                .into(),
        ));
    }

    let mut resampled: Vec<Vec<Vec3>> = Vec::with_capacity(rings.len());
    for (i, ring) in rings.iter().enumerate() {
        let mut pts = if common { ring.clone() } else { resample(ring, n)? };
        if dot(newell(&pts), up) < 0.0 {
            pts.reverse();
        }
        let pts = if i == 0 { pts } else { align(&resampled[i - 1], &pts) };
        if i > 0 {
            let mut span = 0.0;
            for (a, b) in resampled[i - 1].iter().zip(&pts) {
                let d = sub(*b, *a);
                span += dot(d, d).sqrt();
            }
            if span < 1e-9 {
                return Err(Error::Invalid(
                    "two loft profiles coincide; move one along the loft direction".into(),
                ));
            }
        }
        resampled.push(pts);
    }

    // Projected 2D coordinates for the caps, using a right-handed basis in the
    // loft plane. The ring order is CCW viewed from +up, so a CCW ear triangle
    // has normal +up.
    let x_axis = crate::sweep::in_plane_x(up);
    let y_axis = cross(up, x_axis);
    let project = |p: Vec3| [dot(p, x_axis), dot(p, y_axis)];

    let mut mesh = Mesh::default();
    for ring in &resampled {
        for p in ring {
            mesh.positions.push([p[0] as f32, p[1] as f32, p[2] as f32]);
            mesh.normals.push([0.0, 0.0, 0.0]);
        }
    }
    let stride = n as u32;
    let ring_index = |k: usize, i: usize| (k as u32 * stride + (i as u32 % stride)) as u32;

    for k in 0..resampled.len() - 1 {
        for i in 0..n {
            let a = ring_index(k, i);
            let b = ring_index(k, i + 1);
            let c = ring_index(k + 1, i + 1);
            let d = ring_index(k + 1, i);
            mesh.indices.extend_from_slice(&[a, b, c, a, c, d]);
        }
    }

    let bottom2: Vec<[f64; 2]> = resampled[0].iter().map(|p| project(*p)).collect();
    let top2: Vec<[f64; 2]> = resampled.last().unwrap().iter().map(|p| project(*p)).collect();
    let bottom = ear_clip(&bottom2);
    let top = ear_clip(&top2);
    let base = (resampled.len() - 1) as u32 * stride;
    for t in bottom {
        // Bottom faces -up.
        mesh.indices.extend_from_slice(&[t[0] as u32, t[2] as u32, t[1] as u32]);
    }
    for t in top {
        mesh.indices
            .extend_from_slice(&[base + t[0] as u32, base + t[1] as u32, base + t[2] as u32]);
    }

    if fillet::signed_volume(&mesh) < 0.0 {
        for tri in mesh.indices.chunks_exact_mut(3) {
            tri.swap(1, 2);
        }
    }
    boolean::validate_mesh(&mesh).map_err(|e| {
        Error::Invalid(format!(
            "loft failed: {e} (hint: the profiles must morph without self-intersecting; check \
             profile order and that both are simple closed loops)"
        ))
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ring(cx: f64, cy: f64, cz: f64, r: f64, n: usize) -> Vec<Vec3> {
        (0..n)
            .map(|i| {
                let t = i as f64 * std::f64::consts::TAU / n as f64;
                [cx + r * t.cos(), cy + r * t.sin(), cz]
            })
            .collect()
    }

    #[test]
    fn cylinder_loft_has_the_frustum_volume() {
        let lower = ring(0.0, 0.0, 0.0, 10.0, 32);
        let upper = ring(0.0, 0.0, 20.0, 10.0, 32);
        let m = loft(&[lower, upper], 32).unwrap();
        let v = fillet::signed_volume(&m);
        let expected = std::f64::consts::PI * 100.0 * 20.0;
        assert!((v - expected).abs() / expected < 0.02, "volume {v} vs {expected}");
        let check = crate::validate::check_solid(&m);
        assert!(check.watertight && check.non_manifold_edges == 0, "{check:?}");
    }

    #[test]
    fn square_to_circle_needs_no_matching_vertex_order() {
        let square: Vec<Vec3> = vec![
            [-10.0, -10.0, 0.0],
            [10.0, -10.0, 0.0],
            [10.0, 10.0, 0.0],
            [-10.0, 10.0, 0.0],
        ];
        let circle = ring(0.0, 0.0, 20.0, 7.0, 17);
        let m = loft(&[square, circle], 48).unwrap();
        let check = crate::validate::check_solid(&m);
        assert!(check.watertight && check.non_manifold_edges == 0, "{check:?}");
        assert!(check.volume > 0.0);
    }

    #[test]
    fn a_non_convex_profile_caps_correctly() {
        // An L shape, drawn CCW.
        let l: Vec<Vec3> = vec![
            [0.0, 0.0, 0.0],
            [10.0, 0.0, 0.0],
            [10.0, 4.0, 0.0],
            [4.0, 4.0, 0.0],
            [4.0, 10.0, 0.0],
            [0.0, 10.0, 0.0],
        ];
        let top: Vec<Vec3> = l.iter().map(|p| [p[0], p[1], 5.0]).collect();
        let m = loft(&[l, top], 16).unwrap();
        let v = fillet::signed_volume(&m);
        assert!((v - 64.0 * 5.0).abs() < 1e-3, "volume {v}");
    }
}
