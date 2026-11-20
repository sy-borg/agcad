//! Fillets and chamfers: reshape a solid's sharp edges.
//!
//! Both operations are *local*: at a sharp edge, the material near the corner
//! is replaced by a tool shape. In a frame where `u` and `v` point away from
//! the edge into the corner and `d` runs along the edge, the corner sliver is
//!
//! * fillet — the `r × r` square minus a quarter cylinder of radius `r`
//!   tangent to both faces (so the edge becomes a rounded surface), and
//! * chamfer — the right triangle with legs `r` (so the edge becomes one flat
//!   bevel).
//!
//! The sliver is *subtracted* at a convex edge (material outside the rounded
//! corner is cut away) and *added* at a concave edge (the inside corner is
//! filled). That is the same tool for both, only the boolean flips — which
//! is also why a chamfer on an inside corner adds material, as it should.
//!
//! Each selected edge is one boolean against the running result. Edges are
//! selected from the *original* mesh, so overlapping tools near a shared
//! vertex are all applied; the corner where three fillets meet is the
//! intersection of the three cylinders (a small, printable curved triangle)
//! rather than a single spherical patch.
//!
//! Everything here is deterministic: edges come out of a `BTreeMap` in key
//! order and the kernel is bit-reproducible, so a replay re-cuts the same
//! bytes.

use agcad_protocol::{BooleanOp, EdgeFilter, Mesh};

use crate::edges::{self, SharpEdge};
use crate::{boolean, solid, Error};

#[derive(Debug, Clone, Copy)]
pub enum Kind {
    Fillet(f64),
    Chamfer(f64),
}

impl Kind {
    fn size(self) -> f64 {
        match self {
            Self::Fillet(v) | Self::Chamfer(v) => v,
        }
    }

    pub fn name(self) -> &'static str {
        match self {
            Self::Fillet(_) => "fillet",
            Self::Chamfer(_) => "chamfer",
        }
    }
}

/// Apply a fillet/chamfer to every sharp edge matching `filter`.
///
/// `mesh` is in the space the filter is expressed in (the engine passes
/// world space). Returns the modified mesh and the number of edges that were
/// touched.
pub fn apply(
    mesh: &Mesh,
    kind: Kind,
    filter: &EdgeFilter,
    min_angle_deg: f64,
) -> Result<(Mesh, u32), Error> {
    let size = kind.size();
    if !(size > 0.0) || !size.is_finite() {
        return Err(Error::Invalid(format!(
            "{} size must be a positive number of millimeters (got {size})",
            kind.name()
        )));
    }
    edges::validate_filter(filter)?;

    let all = edges::sharp_edges(mesh, min_angle_deg)?;
    let selected: Vec<&SharpEdge> =
        all.iter().filter(|e| edges::matches(filter, e)).collect();
    if selected.is_empty() {
        return Err(Error::Invalid(format!(
            "no sharp edges matched (the solid has {} sharp edge{}; hint: omit `edges` to \
             take all of them, or widen the direction/near filter)",
            all.len(),
            if all.len() == 1 { "" } else { "s" }
        )));
    }

    let mut current = mesh.clone();
    for edge in &selected {
        let tool = tool(edge, kind, 0.0)?;
        // Convex: cut the corner sliver away. Concave: fill it in.
        let op = if edge.convex { BooleanOp::Difference } else { BooleanOp::Union };
        let m = edge.midpoint();
        current = boolean::boolean(op, &[current, tool]).map_err(|e| {
            Error::Invalid(format!(
                "{} failed at the edge through ({:.2}, {:.2}, {:.2}): {e} (hint: the {} size \
                 may be too large for the faces around this edge)",
                kind.name(),
                m[0],
                m[1],
                m[2],
                kind.name()
            ))
        })?;
        if current.indices.is_empty() {
            return Err(Error::Invalid(format!(
                "{} removed the whole solid at the edge through ({:.2}, {:.2}, {:.2}) \
                 (hint: the {} size is larger than the faces around this edge)",
                kind.name(), m[0], m[1], m[2], kind.name()
            )));
        }
    }

    // Every finite tool ends at the edge's endpoints, which leaves a small
    // end-cap where two selected edges meet. At a convex vertex that cap is
    // already outside the material, but at a reflex vertex (two rim edges
    // meeting at an inside corner) it protrudes as a spike. Miter those
    // corners by also cutting the intersection of the two *extended* tools:
    // that intersection is exactly the material above both bevels/arcs.
    //
    // Only edges that share a vertex can form a corner, so group by endpoint
    // instead of testing every pair — a rounded rim can have hundreds of
    // short edges, and all-pairs would be quadratic.
    use std::collections::{BTreeMap, BTreeSet};

    let key = |p: [f64; 3]| {
        [
            (p[0] * 1000.0).round() as i64,
            (p[1] * 1000.0).round() as i64,
            (p[2] * 1000.0).round() as i64,
        ]
    };
    let mut at: BTreeMap<[i64; 3], Vec<usize>> = BTreeMap::new();
    for (i, e) in selected.iter().enumerate() {
        at.entry(key(e.a)).or_default().push(i);
        at.entry(key(e.b)).or_default().push(i);
    }

    let overshoot = size * 2.0;
    let mut extended: BTreeMap<usize, Mesh> = BTreeMap::new();
    let mut convex_corners: Vec<Mesh> = Vec::new();
    let mut concave_corners: Vec<Mesh> = Vec::new();
    let mut seen: BTreeSet<(usize, usize)> = BTreeSet::new();
    for idxs in at.values() {
        for x in 0..idxs.len() {
            for y in (x + 1)..idxs.len() {
                let (i, j) = (idxs[x].min(idxs[y]), idxs[x].max(idxs[y]));
                if !seen.insert((i, j)) {
                    continue;
                }
                let (ei, ej) = (selected[i], selected[j]);
                if ei.convex != ej.convex {
                    continue;
                }
                // Collinear neighbours have no corner to miter.
                if edges::dot(ei.direction(), ej.direction()).abs() > 0.999 {
                    continue;
                }
                let a = match extended.get(&i) {
                    Some(m) => m.clone(),
                    None => {
                        let m = tool(ei, kind, overshoot)?;
                        extended.insert(i, m.clone());
                        m
                    },
                };
                let b = match extended.get(&j) {
                    Some(m) => m.clone(),
                    None => {
                        let m = tool(ej, kind, overshoot)?;
                        extended.insert(j, m.clone());
                        m
                    },
                };
                let corner = boolean::boolean(BooleanOp::Intersection, &[a, b])?;
                if corner.indices.is_empty() {
                    continue;
                }
                if ei.convex {
                    convex_corners.push(corner);
                } else {
                    concave_corners.push(corner);
                }
            }
        }
    }
    if !convex_corners.is_empty() {
        let merged = boolean::boolean(BooleanOp::Union, &convex_corners)?;
        current = boolean::boolean(BooleanOp::Difference, &[current, merged])?;
    }
    if !concave_corners.is_empty() {
        let merged = boolean::boolean(BooleanOp::Union, &concave_corners)?;
        current = boolean::boolean(BooleanOp::Union, &[current, merged])?;
    }
    Ok((current, selected.len() as u32))
}

/// Build the corner tool for one edge: a prism of the cross-section, from
/// `a - overshoot` to `b + overshoot`, oriented so `u`/`v` point into the
/// corner being reshaped. A non-zero `overshoot` is only used to intersect
/// neighbouring tools at a shared vertex.
fn tool(edge: &SharpEdge, kind: Kind, overshoot: f64) -> Result<Mesh, Error> {
    let size = kind.size();
    let length = edge.length();
    if length < 1e-9 {
        return Err(Error::Invalid("fillet/chamfer: degenerate (zero-length) edge".into()));
    }
    // Into the material at a convex edge; into the empty corner at a
    // concave one. In both cases the tool sits on the side that gets
    // reshaped.
    let sign = if edge.convex { -1.0 } else { 1.0 };
    let u = edges::normalize(edges::scale(edge.n1, sign));
    let v = edges::normalize(edges::scale(edge.n2, sign));
    let w = edge.direction();

    let profile = match kind {
        Kind::Chamfer(_) => vec![[0.0, 0.0], [size, 0.0], [0.0, size]],
        Kind::Fillet(_) => fillet_profile(size, edge),
    };

    // Reuse the tested 2D→3D extrusion (it normalizes the loop with Clipper
    // and produces a watertight prism), then move it into the edge frame.
    let (mut mesh, _) = solid::extrude(&[profile], length + 2.0 * overshoot)?;
    for p in &mut mesh.positions {
        let (x, y, z) = (p[0] as f64, p[1] as f64, p[2] as f64 - overshoot);
        *p = [
            (edge.a[0] + x * u[0] + y * v[0] + z * w[0]) as f32,
            (edge.a[1] + x * u[1] + y * v[1] + z * w[1]) as f32,
            (edge.a[2] + x * u[2] + y * v[2] + z * w[2]) as f32,
        ];
    }
    // A concave edge's (u, v, w) frame is left-handed, which inverts the
    // prism's winding. Fix it so the tool is a valid outward-facing solid.
    if signed_volume(&mesh) < 0.0 {
        for tri in mesh.indices.chunks_exact_mut(3) {
            tri.swap(1, 2);
        }
        for n in &mut mesh.normals {
            *n = [-n[0], -n[1], -n[2]];
        }
    }
    Ok(mesh)
}

/// The rounded corner: square `[0, r]²` minus the quarter disc centered at
/// `(r, r)`, i.e. the corner, both face setbacks, and the tangent arc.
fn fillet_profile(r: f64, edge: &SharpEdge) -> Vec<[f64; 2]> {
    // The arc spans the angle between the two faces: 90° on a box corner.
    let arc_deg = edges::dot(edge.n1, edge.n2).clamp(-1.0, 1.0).acos().to_degrees();
    // 16 facets per 90°: fine enough that flat shading reads as a round
    // surface at print scale, cheap enough to boolean quickly.
    let steps = ((arc_deg / 90.0) * 16.0).ceil().max(6.0) as usize;
    let mut pts = vec![[0.0, 0.0], [r, 0.0]];
    // Center at (r, r); the arc runs from angle -90° (tangent to face 2) to
    // -180° (tangent to face 1), bulging toward the corner.
    for i in 1..steps {
        let t = i as f64 / steps as f64;
        let theta = (-0.5 - 0.5 * t) * std::f64::consts::PI;
        pts.push([r + r * theta.cos(), r + r * theta.sin()]);
    }
    pts.push([0.0, r]);
    pts
}

/// Signed volume via the divergence theorem; positive for outward winding.
pub fn signed_volume(mesh: &Mesh) -> f64 {
    let mut v = 0.0;
    for tri in mesh.indices.chunks_exact(3) {
        let p = |i: u32| {
            let q = mesh.positions[i as usize];
            [q[0] as f64, q[1] as f64, q[2] as f64]
        };
        let (a, b, c) = (p(tri[0]), p(tri[1]), p(tri[2]));
        v += edges::dot(a, edges::cross(b, c)) / 6.0;
    }
    v
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geo;

    fn all_edges() -> EdgeFilter {
        EdgeFilter::default()
    }

    fn assert_closed(mesh: &Mesh) {
        // Every undirected edge used by exactly two triangles. Vertices are
        // duplicated per corner (for crease normals), so weld by position
        // bits first — the same rule the kernel uses.
        use std::collections::BTreeMap;
        let mut ids: BTreeMap<[u32; 3], u32> = BTreeMap::new();
        let mut count: BTreeMap<(u32, u32), u32> = BTreeMap::new();
        for tri in mesh.indices.chunks_exact(3) {
            let w: Vec<u32> = tri
                .iter()
                .map(|&i| {
                    let p = mesh.positions[i as usize];
                    let key = [p[0].to_bits(), p[1].to_bits(), p[2].to_bits()];
                    let next = ids.len() as u32;
                    *ids.entry(key).or_insert(next)
                })
                .collect();
            for (a, b) in [(w[0], w[1]), (w[1], w[2]), (w[2], w[0])] {
                let key = if a < b { (a, b) } else { (b, a) };
                *count.entry(key).or_default() += 1;
            }
        }
        assert!(count.values().all(|&c| c == 2), "mesh is not watertight");
        assert!(signed_volume(mesh) > 0.0, "winding must stay outward");
    }

    #[test]
    fn fillet_rounds_every_box_edge() {
        let cube = geo::box_mesh([10.0, 10.0, 10.0]);
        let (m, n) = apply(&cube, Kind::Fillet(2.0), &all_edges(), 30.0).unwrap();
        assert_eq!(n, 12);
        assert_closed(&m);
        let v = signed_volume(&m);
        // 1000 minus 12 rounded slivers (each ~ r²(1 - π/4) × length).
        let sliver = 2.0 * 2.0 * (1.0 - std::f64::consts::FRAC_PI_4) * 10.0;
        let expected = 1000.0 - 12.0 * sliver;
        assert!((v - expected).abs() / expected < 0.06, "{v} vs {expected}");
        // The bounding box is unchanged: the fillet is tangent to the faces.
        for p in &m.positions {
            assert!(p[0].abs() <= 5.001 && p[1].abs() <= 5.001 && p[2].abs() <= 5.001);
        }
    }

    #[test]
    fn chamfer_bevels_every_box_edge() {
        let cube = geo::box_mesh([10.0, 10.0, 10.0]);
        let (m, n) = apply(&cube, Kind::Chamfer(2.0), &all_edges(), 30.0).unwrap();
        assert_eq!(n, 12);
        assert_closed(&m);
        let v = signed_volume(&m);
        // Each bevel removes a triangle prism (area r²/2 × length 10). At
        // each corner the three prisms overlap in a region of 0.75 r³, so
        // the union removes 3·(r³/2) − 0.75 r³ per corner instead of 3·r³/2.
        let expected = 1000.0 - 12.0 * (0.5 * 2.0 * 2.0 * 10.0) + 8.0 * 0.75 * 2.0f64.powi(3);
        assert!((v - expected).abs() / expected < 0.02, "{v} vs {expected}");
    }

    #[test]
    fn direction_selects_only_vertical_edges() {
        let cube = geo::box_mesh([10.0, 10.0, 10.0]);
        let filter = EdgeFilter { direction: Some([0.0, 0.0, 1.0]), ..Default::default() };
        let (m, n) = apply(&cube, Kind::Fillet(2.0), &filter, 30.0).unwrap();
        assert_eq!(n, 4);
        assert_closed(&m);
        // Vertical corners are gone, top and bottom rims stay sharp.
        let left = edges::sharp_edges(&m, 30.0).unwrap();
        assert!(!left.is_empty());
        assert!(
            left.iter().all(|e| edges::dot(e.direction(), [0.0, 0.0, 1.0]).abs() < 1e-6),
            "only the top and bottom rims stay sharp"
        );
        // Volume loss is four slivers, not twelve.
        let sliver = 2.0 * 2.0 * (1.0 - std::f64::consts::FRAC_PI_4) * 10.0;
        assert!((signed_volume(&m) - (1000.0 - 4.0 * sliver)).abs() / 900.0 < 0.02);
    }

    #[test]
    fn concave_edges_are_filled() {
        use agcad_protocol::BooleanOp;
        let outer = geo::box_mesh([10.0, 10.0, 10.0]);
        let mut inner = geo::box_mesh([4.0, 4.0, 12.0]);
        for p in &mut inner.positions {
            p[0] += 3.0;
            p[1] += 3.0;
        }
        let cut = crate::boolean::boolean(BooleanOp::Difference, &[outer, inner]).unwrap();
        let before = signed_volume(&cut);

        let filter = EdgeFilter { convex: Some(false), ..Default::default() };
        let (m, n) = apply(&cut, Kind::Fillet(1.5), &filter, 30.0).unwrap();
        assert_eq!(n, 1);
        assert_closed(&m);
        // Filling an inside corner adds material.
        assert!(signed_volume(&m) > before, "{} vs {before}", signed_volume(&m));

        // The same corners chamfered also gain (flat) material, a little less.
        let (c, n) = apply(&cut, Kind::Chamfer(1.5), &EdgeFilter { convex: Some(false), ..Default::default() }, 30.0).unwrap();
        assert_eq!(n, 1);
        assert_closed(&c);
        assert!(signed_volume(&c) > before);
        // A flat bevel cuts the corner diagonally, so it adds more material
        // than the arc that stays tangent to both faces.
        assert!(signed_volume(&c) > signed_volume(&m), "bevel fills past the round");
    }

    #[test]
    fn a_reflex_rim_corner_is_mitered_not_left_as_a_spike() {
        use agcad_protocol::BooleanOp;
        // An L-shaped plan: a square with one corner cut away, leaving one
        // reflex vertical edge and one reflex corner where two top rim edges
        // meet. Chamfering the rim without mitering leaves a 1.5 mm spike
        // (a short convex vertical edge) above that corner.
        let outer = geo::box_mesh([20.0, 20.0, 10.0]);
        let mut cutter = geo::box_mesh([10.0, 10.0, 20.0]);
        for p in &mut cutter.positions {
            p[0] += 10.0;
            p[1] += 10.0;
        }
        let l = crate::boolean::boolean(BooleanOp::Difference, &[outer, cutter]).unwrap();
        let vertical = |m: &Mesh| -> Vec<f64> {
            edges::sharp_edges(m, 30.0)
                .unwrap()
                .iter()
                .filter(|e| edges::dot(e.direction(), [0.0, 0.0, 1.0]).abs() > 0.99)
                .map(|e| e.length())
                .collect()
        };
        assert_eq!(vertical(&l).len(), 6, "an L outline has six vertical edges");

        let rim = EdgeFilter { normals: Some(vec![[0.0, 0.0, 1.0]]), ..Default::default() };
        for kind in [Kind::Chamfer(1.5), Kind::Fillet(1.5)] {
            let (m, n) = apply(&l, kind, &rim, 30.0).unwrap();
            assert_eq!(n, 6, "every top rim edge");
            assert_closed(&m);
            let lens = vertical(&m);
            // A fillet's mitered corner leaves a hairline seam (< 0.2 mm)
            // where the two quarter-round patches meet; that is fine. What
            // must not remain is the 1.5 mm end-cap spike, so every
            // *substantial* vertical edge has to span the remaining height.
            let long: Vec<f64> = lens.iter().copied().filter(|&len| len > 1.0).collect();
            assert_eq!(long.len(), 6, "six full-height vertical edges: {lens:?}");
            assert!(long.iter().all(|&len| len > 5.0), "leftover spike: {long:?}");
        }
    }

    #[test]
    fn a_sphere_has_nothing_to_fillet() {
        let sphere = geo::sphere_mesh(10.0, 24);
        let err = apply(&sphere, Kind::Fillet(1.0), &all_edges(), 30.0).unwrap_err();
        assert!(err.to_string().contains("no sharp edges matched"), "{err}");
    }

    #[test]
    fn an_oversized_radius_is_reported() {
        let cube = geo::box_mesh([10.0, 10.0, 10.0]);
        let err = apply(&cube, Kind::Fillet(40.0), &all_edges(), 30.0).unwrap_err();
        // Whatever the kernel does (empty result or failure), the message
        // must point at the size.
        assert!(err.to_string().contains("fillet"), "{err}");
    }
}
