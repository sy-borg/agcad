//! Cutting and projecting a solid into sketch geometry (M22).
//!
//! Both operations consume a **world-space triangle mesh** and a resolved
//! plane frame, and produce 2D curves in that frame:
//!
//! * [`section_curves`] intersects every triangle with the plane, welds the
//!   resulting segment endpoints and chains them into polylines. Adjacent
//!   triangles share an edge, so a watertight solid produces closed loops
//!   (holes included) that can be extruded directly.
//! * [`project_curves`] projects boundary, sharp and silhouette edges onto
//!   the plane. Front-facing sharp edges are kept and back-facing ones are
//!   dropped, so a top view of a box shows its outline instead of every
//!   hidden edge.
//!
//! Everything is deterministic: welded vertex ids come from a `BTreeMap`,
//! loops are traced from the smallest vertex id, and no hash iteration order
//! leaks into the output.

use std::collections::{BTreeMap, BTreeSet};

use wright_protocol::Mesh;

use crate::edges::{self, dot, normalize, sub};
use crate::import2d::Curve2d;
use crate::plane::Frame;
use crate::Error;

/// Section intersections closer than this (in sketch units) are the same
/// point. The value is a fraction of the model diagonal, with an absolute
/// floor for tiny drawings.
const WELD_FRACTION: f64 = 1e-6;
const WELD_FLOOR: f64 = 1e-6;

/// The plane cut of `mesh`, in `frame` coordinates.
pub fn section_curves(mesh: &Mesh, frame: &Frame) -> Result<Vec<Curve2d>, Error> {
    let (positions, tris) = edges::weld(mesh)?;
    let (lo, hi) = bounds(&positions);
    let diag = (0..3)
        .map(|i| (hi[i] - lo[i]) * (hi[i] - lo[i]))
        .sum::<f64>()
        .sqrt();
    let weld = (diag * WELD_FRACTION).max(WELD_FLOOR);
    let on_plane = weld.max(1e-9);

    let to2d = |p: [f64; 3]| -> [f64; 2] {
        let d = sub(p, frame.origin);
        [dot(d, frame.x_axis), dot(d, frame.y_axis())]
    };

    let mut segments: Vec<([f64; 2], [f64; 2])> = Vec::new();
    for tri in tris.chunks_exact(3) {
        let p = [
            positions[tri[0] as usize],
            positions[tri[1] as usize],
            positions[tri[2] as usize],
        ];
        let d = p.map(|q| dot(sub(q, frame.origin), frame.normal));
        let all_positive = d.iter().all(|v| *v > on_plane);
        let all_negative = d.iter().all(|v| *v < -on_plane);
        if all_positive || all_negative {
            continue;
        }
        let all_on = d.iter().all(|v| v.abs() <= on_plane);
        if all_on {
            // Coplanar triangle: it contributes area, not a cut curve.
            continue;
        }

        let mut points: Vec<[f64; 2]> = Vec::with_capacity(3);
        for k in 0..3 {
            if d[k].abs() <= on_plane {
                push_unique(&mut points, to2d(p[k]), weld);
            }
            let j = (k + 1) % 3;
            if (d[k] > on_plane && d[j] < -on_plane) || (d[k] < -on_plane && d[j] > on_plane) {
                let t = d[k] / (d[k] - d[j]);
                let q = [
                    p[k][0] + t * (p[j][0] - p[k][0]),
                    p[k][1] + t * (p[j][1] - p[k][1]),
                    p[k][2] + t * (p[j][2] - p[k][2]),
                ];
                push_unique(&mut points, to2d(q), weld);
            }
        }
        if points.len() == 2 {
            if distance(points[0], points[1]) > weld {
                segments.push((points[0], points[1]));
            }
        }
    }

    if segments.is_empty() {
        return Ok(Vec::new());
    }

    // Quantize endpoints so adjacent triangles meet exactly.
    let mut ids: BTreeMap<[i64; 2], usize> = BTreeMap::new();
    let mut points: Vec<[f64; 2]> = Vec::new();
    let key = |p: [f64; 2]| -> [i64; 2] {
        [
            (p[0] / weld).round() as i64,
            (p[1] / weld).round() as i64,
        ]
    };
    let mut edges_set: BTreeSet<(usize, usize)> = BTreeSet::new();
    for (a, b) in &segments {
        let ka = key(*a);
        let kb = key(*b);
        if ka == kb {
            continue;
        }
        let ia = *ids.entry(ka).or_insert_with(|| {
            points.push(*a);
            points.len() - 1
        });
        let ib = *ids.entry(kb).or_insert_with(|| {
            points.push(*b);
            points.len() - 1
        });
        let pair = if ia < ib { (ia, ib) } else { (ib, ia) };
        edges_set.insert(pair);
    }

    let mut adjacency: BTreeMap<usize, BTreeSet<usize>> = BTreeMap::new();
    for (a, b) in &edges_set {
        adjacency.entry(*a).or_default().insert(*b);
        adjacency.entry(*b).or_default().insert(*a);
    }

    let mut used: BTreeSet<(usize, usize)> = BTreeSet::new();
    let mut curves = Vec::new();
    for &start in adjacency.keys() {
        while let Some(next) = adjacency
            .get(&start)
            .and_then(|n| n.iter().find(|&&n| !used.contains(&edge_key(start, n))))
            .copied()
        {
            let mut chain = vec![start];
            let mut prev = start;
            let mut cur = next;
            used.insert(edge_key(prev, cur));
            chain.push(cur);
            loop {
                let Some(neighbors) = adjacency.get(&cur) else { break };
                let mut stepped = false;
                for &n in neighbors {
                    if n != prev && !used.contains(&edge_key(cur, n)) {
                        used.insert(edge_key(cur, n));
                        prev = cur;
                        cur = n;
                        chain.push(cur);
                        stepped = true;
                        break;
                    }
                }
                if !stepped {
                    break;
                }
                if cur == start {
                    break;
                }
            }
            let closed = chain.len() > 2 && chain.first() == chain.last();
            let simplified = simplify_chain(&chain, &points, closed, weld);
            if simplified.len() < 2 {
                continue;
            }
            for pair in simplified.windows(2) {
                curves.push(Curve2d::Line { a: pair[0], b: pair[1] });
            }
            if closed {
                let first = *simplified.first().unwrap();
                let last = *simplified.last().unwrap();
                if distance(first, last) > weld {
                    curves.push(Curve2d::Line { a: last, b: first });
                }
            }
        }
    }
    Ok(curves)
}

/// Project boundary, sharp and silhouette edges of `mesh` onto `frame`.
pub fn project_curves(
    mesh: &Mesh,
    frame: &Frame,
    min_angle_deg: f64,
) -> Result<Vec<Curve2d>, Error> {
    let (positions, tris) = edges::weld(mesh)?;
    let (lo, hi) = bounds(&positions);
    let diag = (0..3)
        .map(|i| (hi[i] - lo[i]) * (hi[i] - lo[i]))
        .sum::<f64>()
        .sqrt();
    let weld = (diag * WELD_FRACTION).max(WELD_FLOOR);
    let min_cos = (min_angle_deg.clamp(0.0, 179.0)).to_radians().cos();

    let mut triangles: Vec<[u32; 3]> = tris
        .chunks_exact(3)
        .map(|t| [t[0], t[1], t[2]])
        .collect();
    triangles.retain(|t| t[0] != t[1] && t[1] != t[2] && t[0] != t[2]);

    let mut normals: Vec<[f64; 3]> = Vec::with_capacity(triangles.len());
    for t in &triangles {
        let a = positions[t[0] as usize];
        let b = positions[t[1] as usize];
        let c = positions[t[2] as usize];
        normals.push(normalize(crate::edges::cross(sub(b, a), sub(c, a))));
    }

    let mut edge_faces: BTreeMap<(u32, u32), Vec<usize>> = BTreeMap::new();
    for (index, t) in triangles.iter().enumerate() {
        for (a, b) in [(t[0], t[1]), (t[1], t[2]), (t[2], t[0])] {
            let key = if a < b { (a, b) } else { (b, a) };
            edge_faces.entry(key).or_default().push(index);
        }
    }

    let to2d = |p: [f64; 3]| -> [f64; 2] {
        let d = sub(p, frame.origin);
        [dot(d, frame.x_axis), dot(d, frame.y_axis())]
    };

    let mut seen: BTreeSet<([i64; 2], [i64; 2])> = BTreeSet::new();
    let key = |p: [f64; 2]| -> [i64; 2] {
        [
            (p[0] / weld).round() as i64,
            (p[1] / weld).round() as i64,
        ]
    };
    let mut curves = Vec::new();
    for ((a, b), faces) in &edge_faces {
        let include = match faces.as_slice() {
            [] => false,
            [_] => true,
            [f0, f1, ..] => {
                let n0 = normals[*f0];
                let n1 = normals[*f1];
                let sharp = dot(n0, n1) < min_cos;
                let f0_dot = dot(n0, frame.normal);
                let f1_dot = dot(n1, frame.normal);
                let silhouette = f0_dot * f1_dot <= 1e-12;
                let front = f0_dot > 1e-9 || f1_dot > 1e-9;
                silhouette || (sharp && front)
            },
        };
        if !include {
            continue;
        }
        let pa = to2d(positions[*a as usize]);
        let pb = to2d(positions[*b as usize]);
        if distance(pa, pb) <= weld {
            continue;
        }
        let (ka, kb) = (key(pa), key(pb));
        let pair = if ka < kb { (ka, kb) } else { (kb, ka) };
        if seen.insert(pair) {
            curves.push(Curve2d::Line { a: pa, b: pb });
        }
    }
    Ok(curves)
}

fn edge_key(a: usize, b: usize) -> (usize, usize) {
    if a < b { (a, b) } else { (b, a) }
}

fn distance(a: [f64; 2], b: [f64; 2]) -> f64 {
    let dx = a[0] - b[0];
    let dy = a[1] - b[1];
    (dx * dx + dy * dy).sqrt()
}

fn push_unique(points: &mut Vec<[f64; 2]>, p: [f64; 2], tol: f64) {
    if !points.iter().any(|q| distance(*q, p) <= tol) {
        points.push(p);
    }
}

/// Drop interior points of a straight run. Keeps the geometry exactly and
/// makes a box cut four lines per loop instead of one per triangle edge.
fn simplify_chain(
    chain: &[usize],
    points: &[[f64; 2]],
    closed: bool,
    tol: f64,
) -> Vec<[f64; 2]> {
    let pts: Vec<[f64; 2]> = chain.iter().map(|&i| points[i]).collect();
    if pts.len() <= 2 {
        return pts;
    }
    let mut out = Vec::with_capacity(pts.len());
    out.push(pts[0]);
    for i in 1..pts.len() - 1 {
        let a = out[out.len() - 1];
        let b = pts[i];
        let c = pts[i + 1];
        let ab = [b[0] - a[0], b[1] - a[1]];
        let bc = [c[0] - b[0], c[1] - b[1]];
        let cross = ab[0] * bc[1] - ab[1] * bc[0];
        let len = (ab[0] * ab[0] + ab[1] * ab[1]).sqrt()
            + (bc[0] * bc[0] + bc[1] * bc[1]).sqrt();
        if cross.abs() <= tol * len.max(1.0) {
            continue;
        }
        out.push(b);
    }
    out.push(*pts.last().unwrap());
    if closed && out.len() >= 3 {
        // The wrap-around joint can also be collinear; leave the closing
        // duplicate point in place because the caller adds the closing line.
        let a = out[out.len() - 2];
        let b = out[out.len() - 1];
        let c = out[1];
        let ab = [b[0] - a[0], b[1] - a[1]];
        let bc = [c[0] - b[0], c[1] - b[1]];
        let cross = ab[0] * bc[1] - ab[1] * bc[0];
        let len = (ab[0] * ab[0] + ab[1] * ab[1]).sqrt()
            + (bc[0] * bc[0] + bc[1] * bc[1]).sqrt();
        if cross.abs() <= tol * len.max(1.0) && out.len() > 3 {
            out.pop();
        }
    }
    out
}

fn bounds(points: &[[f64; 3]]) -> ([f64; 3], [f64; 3]) {
    let mut lo = [f64::MAX; 3];
    let mut hi = [f64::MIN; 3];
    for p in points {
        for i in 0..3 {
            lo[i] = lo[i].min(p[i]);
            hi[i] = hi[i].max(p[i]);
        }
    }
    if points.is_empty() {
        ([0.0; 3], [0.0; 3])
    } else {
        (lo, hi)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geo;
    use crate::plane;

    fn frame(origin: [f64; 3], normal: [f64; 3], x_axis: Option<[f64; 3]>) -> Frame {
        plane::custom_frame(origin, normal, x_axis).unwrap()
    }

    #[test]
    fn a_box_cut_is_one_closed_loop_with_four_lines() {
        let mesh = geo::box_mesh([20.0, 10.0, 6.0]);
        // box_mesh is centered on the origin, so z = 0 cuts the middle.
        let f = frame([0.0, 0.0, 0.0], [0.0, 0.0, 1.0], Some([1.0, 0.0, 0.0]));
        let curves = section_curves(&mesh, &f).unwrap();
        assert_eq!(curves.len(), 4, "{curves:?}");
        let mut pts: BTreeSet<[i64; 2]> = BTreeSet::new();
        for c in &curves {
            if let Curve2d::Line { a, b } = c {
                for p in [a, b] {
                    pts.insert([(p[0] * 1e6).round() as i64, (p[1] * 1e6).round() as i64]);
                }
            }
        }
        assert_eq!(pts.len(), 4);
    }

    #[test]
    fn a_cylinder_cut_is_a_closed_ring() {
        let mesh = geo::cylinder_mesh(5.0, 20.0, 32);
        let f = frame([0.0, 0.0, 0.0], [0.0, 0.0, 1.0], None);
        let curves = section_curves(&mesh, &f).unwrap();
        assert!(curves.len() >= 24, "{}", curves.len());
    }

    #[test]
    fn projecting_a_box_from_above_keeps_the_top_outline() {
        let mesh = geo::box_mesh([20.0, 10.0, 6.0]);
        // Looking down -Z from above: the top face is front-facing.
        let f = frame([0.0, 0.0, 100.0], [0.0, 0.0, 1.0], Some([1.0, 0.0, 0.0]));
        let curves = project_curves(&mesh, &f, 30.0).unwrap();
        assert_eq!(curves.len(), 4, "{curves:?}");
    }
}
