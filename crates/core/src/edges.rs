//! Sharp-edge extraction and selection.
//!
//! Fillets and chamfers need to know where a solid's faces meet. A triangle
//! mesh has no notion of an edge, so we rebuild one: weld coincident
//! vertices by exact bits (the same rule the boolean kernel uses), pair
//! triangles by undirected vertex pair, and keep only the pairs whose face
//! normals differ by at least `min_angle_deg`.
//!
//! That angle threshold is what separates a real edge from a tessellation
//! seam: a 32-segment cylinder turns 11.25° per facet, a box corner turns
//! 90°. The default (30°) keeps curved primitives smooth while still
//! catching every ordinary CAD edge.
//!
//! Edges are reported in the space of the mesh handed in. The engine calls
//! this with **world-space** positions, so [`EdgeFilter`] conditions
//! (`direction`, `near`, `normals`) are world-space too, and a rotated or
//! scaled entity is still selectable in the coordinates the user sees.

use std::collections::BTreeMap;

use agcad_protocol::{EdgeFilter, Mesh};

use crate::Error;

/// A straight edge segment where two faces meet, with the outward normals of
/// those faces. Convex means the material is inside the corner (dihedral
/// angle below 180°) — a box's 12 edges are convex, the inside of a notch is
/// concave.
#[derive(Debug, Clone)]
pub struct SharpEdge {
    pub a: [f64; 3],
    pub b: [f64; 3],
    pub n1: [f64; 3],
    pub n2: [f64; 3],
    pub convex: bool,
}

impl SharpEdge {
    pub fn midpoint(&self) -> [f64; 3] {
        [
            (self.a[0] + self.b[0]) * 0.5,
            (self.a[1] + self.b[1]) * 0.5,
            (self.a[2] + self.b[2]) * 0.5,
        ]
    }

    pub fn direction(&self) -> [f64; 3] {
        normalize(sub(self.b, self.a))
    }

    pub fn length(&self) -> f64 {
        let d = sub(self.b, self.a);
        dot(d, d).sqrt()
    }

    /// Interior dihedral angle in degrees (90 for a box corner, 270 for a
    /// square inside corner).
    pub fn dihedral_deg(&self) -> f64 {
        let between = dot(self.n1, self.n2).clamp(-1.0, 1.0).acos().to_degrees();
        if self.convex { 180.0 - between } else { 180.0 + between }
    }
}

pub fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

pub fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

pub fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

pub fn scale(v: [f64; 3], s: f64) -> [f64; 3] {
    [v[0] * s, v[1] * s, v[2] * s]
}

pub fn normalize(v: [f64; 3]) -> [f64; 3] {
    let len = dot(v, v).sqrt();
    if len < 1e-12 { [0.0; 3] } else { [v[0] / len, v[1] / len, v[2] / len] }
}

struct EdgeRecord {
    /// Directed edge as first seen on a non-degenerate triangle — fixes the
    /// orientation used for the convexity test.
    a: u32,
    b: u32,
    /// The first and second face normals. `None` until a non-degenerate
    /// triangle uses the edge: a zero-area triangle has no normal, but it
    /// still shares the edge, and ignoring it would make a closed solid look
    /// open (a boolean can leave a zero-area flap on a tangent seam).
    n1: Option<[f64; 3]>,
    n2: Option<[f64; 3]>,
    count: u32,
}

/// Every sharp edge of a closed triangle mesh, in deterministic order.
///
/// Errors when the mesh is not closed (a boundary edge means it is not a
/// printable solid) or non-manifold (an edge shared by more than two
/// triangles).
pub fn sharp_edges(mesh: &Mesh, min_angle_deg: f64) -> Result<Vec<SharpEdge>, Error> {
    let (positions, tris) = weld(mesh)?;
    let min_cos = (min_angle_deg.clamp(0.0, 179.0)).to_radians().cos();

    let mut map: BTreeMap<(u32, u32), EdgeRecord> = BTreeMap::new();
    for tri in tris.chunks_exact(3) {
        let (i0, i1, i2) = (tri[0], tri[1], tri[2]);
        let (p0, p1, p2) = (positions[i0 as usize], positions[i1 as usize], positions[i2 as usize]);
        let n = normalize(cross(sub(p1, p0), sub(p2, p0)));
        let degenerate = dot(n, n) < 0.5;
        for (a, b) in [(i0, i1), (i1, i2), (i2, i0)] {
            let key = if a < b { (a, b) } else { (b, a) };
            match map.get_mut(&key) {
                Some(rec) => {
                    rec.count += 1;
                    if !degenerate {
                        if rec.n1.is_none() {
                            rec.n1 = Some(n);
                            // The flap's winding is meaningless; take the
                            // orientation from the first real triangle.
                            rec.a = a;
                            rec.b = b;
                        } else if rec.n2.is_none() {
                            rec.n2 = Some(n);
                        }
                    }
                },
                None => {
                    map.insert(
                        key,
                        EdgeRecord {
                            a,
                            b,
                            n1: if degenerate { None } else { Some(n) },
                            n2: None,
                            count: 1,
                        },
                    );
                },
            }
        }
    }

    let mut out = Vec::new();
    for ((ka, _kb), rec) in map {
        if rec.count == 1 {
            let p = positions[ka as usize];
            return Err(Error::Invalid(format!(
                "mesh is not a closed solid: edge at ({:.3}, {:.3}, {:.3}) belongs to one \
                 triangle (hint: fillet/chamfer need a watertight solid)",
                p[0], p[1], p[2]
            )));
        }
        if rec.count > 2 {
            let p = positions[ka as usize];
            return Err(Error::Invalid(format!(
                "mesh is not manifold: edge at ({:.3}, {:.3}, {:.3}) belongs to {} triangles",
                p[0], p[1], p[2], rec.count
            )));
        }
        let Some(n1) = rec.n1 else { continue }; // only zero-area flaps
        let n2 = rec.n2.unwrap_or(n1);
        if dot(n1, n2) >= min_cos {
            continue; // coplanar or a tessellation seam, not a real edge
        }
        // Keep the direction the convexity test used: `rec.n1` is the normal
        // of the triangle that first traversed `rec.a → rec.b`. Storing the
        // sorted pair instead would flip `direction()` for roughly half the
        // edges, and the tool frame would then disagree with `convex`.
        let a = positions[rec.a as usize];
        let b = positions[rec.b as usize];
        let d = sub(b, a);
        let convex = dot(cross(n1, n2), d) > 0.0;
        out.push(SharpEdge { a, b, n1, n2, convex });
    }
    Ok(out)
}

/// Weld vertices by exact f32 bits. Primitives emit face-local duplicates;
/// booleans emit already-welded meshes. Exact bits are enough because the
/// generators are deterministic.
pub(crate) fn weld(mesh: &Mesh) -> Result<(Vec<[f64; 3]>, Vec<u32>), Error> {
    let mut ids: BTreeMap<[u32; 3], u32> = BTreeMap::new();
    let mut positions: Vec<[f64; 3]> = Vec::new();
    let mut remap: Vec<u32> = Vec::with_capacity(mesh.positions.len());
    for p in &mesh.positions {
        let key = [p[0].to_bits(), p[1].to_bits(), p[2].to_bits()];
        let id = match ids.get(&key) {
            Some(&id) => id,
            None => {
                let id = positions.len() as u32;
                ids.insert(key, id);
                positions.push([p[0] as f64, p[1] as f64, p[2] as f64]);
                id
            },
        };
        remap.push(id);
    }
    if mesh.indices.len() % 3 != 0 {
        return Err(Error::Invalid("mesh indices are not a multiple of 3".into()));
    }
    let mut tris = Vec::with_capacity(mesh.indices.len());
    for &i in &mesh.indices {
        let Some(&w) = remap.get(i as usize) else {
            return Err(Error::Invalid(format!(
                "mesh index {i} is out of range ({} vertices)",
                mesh.positions.len()
            )));
        };
        tris.push(w);
    }
    Ok((positions, tris))
}

/// Check a filter once, before touching geometry, so a bad selector is a
/// clear parameter error rather than a silent "no edges matched".
pub fn validate_filter(filter: &EdgeFilter) -> Result<(), Error> {
    if let Some(d) = filter.direction {
        if dot(d, d) < 1e-12 {
            return Err(Error::Invalid(
                "edges.direction must be a non-zero vector (e.g. [0,0,1] for vertical edges)".into(),
            ));
        }
    }
    if filter.near.is_some() != filter.near_radius.is_some() {
        return Err(Error::Invalid(
            "edges.near and edges.near_radius must be given together".into(),
        ));
    }
    if let Some(r) = filter.near_radius {
        if !(r > 0.0) {
            return Err(Error::Invalid("edges.near_radius must be > 0".into()));
        }
    }
    if let Some(pair) = &filter.normals {
        if pair.is_empty() || pair.len() > 2 {
            return Err(Error::Invalid(
                "edges.normals takes one face normal (all its edges) or two (the edge between \
                 them)"
                    .into(),
            ));
        }
        if pair.iter().any(|n| dot(*n, *n) < 1e-12) {
            return Err(Error::Invalid(
                "edges.normals must contain non-zero vectors (outward face normals)".into(),
            ));
        }
    }
    if let Some(tol) = filter.angle_tol_deg {
        if !(tol > 0.0 && tol <= 90.0) {
            return Err(Error::Invalid("edges.angle_tol_deg must be in (0, 90]".into()));
        }
    }
    Ok(())
}

/// True when every condition that is set on `filter` holds for `edge`.
pub fn matches(filter: &EdgeFilter, edge: &SharpEdge) -> bool {
    let tol = filter.angle_tol_deg.unwrap_or(1.0).to_radians();
    let min_cos = tol.cos();

    if let Some(want) = filter.direction {
        let d = edge.direction();
        if dot(d, normalize(want)).abs() < min_cos {
            return false;
        }
    }
    if let (Some(p), Some(r)) = (filter.near, filter.near_radius) {
        let m = edge.midpoint();
        let delta = sub(m, p);
        if dot(delta, delta).sqrt() > r {
            return false;
        }
    }
    if let Some(pair) = &filter.normals {
        let touches = |n: [f64; 3]| pair.iter().any(|w| dot(normalize(*w), n) >= min_cos);
        match pair.len() {
            1 => {
                if !touches(edge.n1) && !touches(edge.n2) {
                    return false;
                }
            },
            2 => {
                let (f0, f1) = (normalize(pair[0]), normalize(pair[1]));
                let forward = dot(edge.n1, f0) >= min_cos && dot(edge.n2, f1) >= min_cos;
                let backward = dot(edge.n1, f1) >= min_cos && dot(edge.n2, f0) >= min_cos;
                if !(forward || backward) {
                    return false;
                }
            },
            _ => return false,
        }
    }
    if let Some(want) = filter.convex {
        if edge.convex != want {
            return false;
        }
    }
    true
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geo;

    fn box_edges(size: f64) -> Vec<SharpEdge> {
        sharp_edges(&geo::box_mesh([size, size, size]), 30.0).unwrap()
    }

    #[test]
    fn a_box_has_twelve_convex_edges() {
        let edges = box_edges(10.0);
        assert_eq!(edges.len(), 12, "6 faces → 12 edges");
        assert!(edges.iter().all(|e| e.convex), "every box edge is convex");
        assert!(edges.iter().all(|e| (e.dihedral_deg() - 90.0).abs() < 1e-6));

        // Four edges run along each axis.
        for axis in 0..3 {
            let mut dir = [0.0; 3];
            dir[axis] = 1.0;
            let n = edges.iter().filter(|e| dot(e.direction(), dir).abs() > 0.999).count();
            assert_eq!(n, 4, "axis {axis}");
        }
        // Face-local duplicates were welded: every edge endpoint is a cube
        // corner, so there are only 8 distinct vertices.
        let mut corners: Vec<[i64; 3]> = edges
            .iter()
            .flat_map(|e| [e.a, e.b])
            .map(|p| [p[0].round() as i64, p[1].round() as i64, p[2].round() as i64])
            .collect();
        corners.sort_unstable();
        corners.dedup();
        assert_eq!(corners.len(), 8);
    }

    #[test]
    fn tessellation_seams_are_not_edges() {
        // A 32-segment cylinder turns 11.25° per facet — below the 30°
        // threshold — so only the two cap rims count.
        let mesh = geo::cylinder_mesh(5.0, 10.0, 32);
        let edges = sharp_edges(&mesh, 30.0).unwrap();
        assert_eq!(edges.len(), 64, "two rims × 32 segments, no side seams");
        assert!(edges.iter().all(|e| e.convex));
        assert!(edges.iter().all(|e| dot(e.direction(), [0.0, 0.0, 1.0]).abs() < 1e-9));
    }

    #[test]
    fn concave_edges_are_detected() {
        use agcad_protocol::BooleanOp;
        let outer = geo::box_mesh([10.0, 10.0, 10.0]);
        let mut inner = geo::box_mesh([4.0, 4.0, 12.0]);
        // Shift the cutter to a corner so the difference leaves an L-shaped
        // plan with two concave vertical edges.
        for p in &mut inner.positions {
            p[0] += 3.0;
            p[1] += 3.0;
        }
        let cut = crate::boolean::boolean(BooleanOp::Difference, &[outer, inner]).unwrap();
        let edges = sharp_edges(&cut, 30.0).unwrap();
        // A square notch has one re-entrant (concave) vertical corner.
        let concave: Vec<_> = edges.iter().filter(|e| !e.convex).collect();
        assert_eq!(concave.len(), 1, "one inside corner, got {}", concave.len());
        for e in concave {
            assert!((e.dihedral_deg() - 270.0).abs() < 1e-6, "{}", e.dihedral_deg());
        }
    }

    #[test]
    fn filters_are_anded_and_world_oriented() {
        let edges = box_edges(10.0);
        let vertical = EdgeFilter { direction: Some([0.0, 0.0, 1.0]), ..Default::default() };
        assert_eq!(edges.iter().filter(|e| matches(&vertical, e)).count(), 4);

        let top_rim = EdgeFilter {
            normals: Some(vec![[0.0, 0.0, 1.0], [1.0, 0.0, 0.0]]),
            ..Default::default()
        };
        let picked: Vec<_> = edges.iter().filter(|e| matches(&top_rim, e)).collect();
        assert_eq!(picked.len(), 1, "one edge between the top and +X faces");

        // The pair matches in either order.
        let swapped = EdgeFilter {
            normals: Some(vec![[1.0, 0.0, 0.0], [0.0, 0.0, 1.0]]),
            ..Default::default()
        };
        assert!(matches(&swapped, picked[0]));

        // One normal means "every edge of that face".
        let top_face = EdgeFilter { normals: Some(vec![[0.0, 0.0, 1.0]]), ..Default::default() };
        assert_eq!(edges.iter().filter(|e| matches(&top_face, e)).count(), 4);

        // Direction AND position: only the corner at (5, 5, ...).
        let corner = EdgeFilter {
            direction: Some([0.0, 0.0, 1.0]),
            near: Some([5.0, 5.0, 0.0]),
            near_radius: Some(1.0),
            ..Default::default()
        };
        assert_eq!(edges.iter().filter(|e| matches(&corner, e)).count(), 1);

        let concave_only = EdgeFilter { convex: Some(false), ..Default::default() };
        assert_eq!(edges.iter().filter(|e| matches(&concave_only, e)).count(), 0);
    }

    #[test]
    fn bad_filters_are_rejected() {
        let zero = EdgeFilter { direction: Some([0.0; 3]), ..Default::default() };
        assert!(validate_filter(&zero).is_err());
        let half = EdgeFilter { near: Some([0.0; 3]), ..Default::default() };
        assert!(validate_filter(&half).is_err());
        let ok = EdgeFilter {
            near: Some([0.0; 3]),
            near_radius: Some(5.0),
            angle_tol_deg: Some(2.0),
            ..Default::default()
        };
        assert!(validate_filter(&ok).is_ok());
        let three = EdgeFilter {
            normals: Some(vec![[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]),
            ..Default::default()
        };
        assert!(validate_filter(&three).is_err());
    }
}
