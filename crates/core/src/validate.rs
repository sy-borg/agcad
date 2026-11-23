//! Trust checks: is a triangle mesh a printable solid, how thin is it, and
//! did a rebuild really change anything?
//!
//! Everything here is deterministic and cheap enough to run in an agent loop:
//!
//! * **Topology** — weld vertices by exact f32 bits (the same rule as the
//!   boolean kernel and `edges::sharp_edges`) and count the triangles on
//!   every undirected edge. One triangle means a hole, more than two means a
//!   non-manifold edge.
//! * **Self-intersection** — a sweep over triangle AABBs sorted by min X,
//!   with a triangle/triangle test that skips topological neighbours. The
//!   check is intentionally approximate: it proves "none found", not
//!   "mathematically none exists".
//! * **Minimum wall** — one inward ray per sampled triangle; the nearest
//!   non-adjacent surface hit is the local thickness. Sampling is capped so
//!   the cost stays linear-ish in triangle count.
//! * **Signatures** — FNV-1a over entity name, placement, color and geometry.
//!   Same bits in, same hex string out, so a scene diff can compare two
//!   revisions without shipping meshes.
//!
//! No randomness, clocks, hash iteration or locale-dependent formatting.

use std::collections::BTreeMap;

use crate::{Entity, Shape, edges};
use wright_protocol::{Bbox, EntityId, Mesh, SketchElement, Vec3};

/// Cap on the number of reported self-intersection locations.
const MAX_INTERSECTION_POINTS: usize = 8;
/// Cap on the number of triangles sampled for the wall estimate.
const MAX_WALL_SAMPLES: usize = 256;

/// The geometry part of a solid validation report.
#[derive(Debug, Clone)]
pub struct MeshCheck {
    pub watertight: bool,
    pub open_edges: usize,
    pub non_manifold_edges: usize,
    pub degenerate_tris: usize,
    pub self_intersections: usize,
    pub self_intersection_points: Vec<Vec3>,
    pub volume: f64,
    pub bbox: Bbox,
    pub tris: usize,
    pub min_wall: Option<f64>,
    pub min_wall_at: Option<Vec3>,
}

/// Check a **world-space** mesh. `min_wall` is measured as-is (the caller
/// decides whether to skip it for open meshes); it is always `None` when the
/// mesh is not watertight, because an inward ray would escape through the
/// hole and the number would be meaningless.
pub fn check_solid(mesh: &Mesh) -> MeshCheck {
    let tris = mesh.indices.len() / 3;
    let (positions, welded) = match edges::weld(mesh) {
        Ok(pair) => pair,
        Err(_) => {
            // Invalid indices are a malformed entity, not a document error:
            // report it as an open surface instead of failing the command.
            return MeshCheck {
                watertight: false,
                open_edges: 1,
                non_manifold_edges: 0,
                degenerate_tris: 0,
                self_intersections: 0,
                self_intersection_points: Vec::new(),
                volume: crate::fillet::signed_volume(mesh),
                bbox: mesh_bbox(mesh),
                tris,
                min_wall: None,
                min_wall_at: None,
            };
        }
    };

    let mut edge_count: BTreeMap<(u32, u32), u32> = BTreeMap::new();
    let mut degenerate_tris = 0usize;
    for tri in welded.chunks_exact(3) {
        let (i0, i1, i2) = (tri[0], tri[1], tri[2]);
        let (p0, p1, p2) = (
            positions[i0 as usize],
            positions[i1 as usize],
            positions[i2 as usize],
        );
        let n = edges::cross(edges::sub(p1, p0), edges::sub(p2, p0));
        if edges::dot(n, n) < 1e-24 {
            degenerate_tris += 1;
        }
        for (a, b) in [(i0, i1), (i1, i2), (i2, i0)] {
            let key = if a < b { (a, b) } else { (b, a) };
            *edge_count.entry(key).or_insert(0) += 1;
        }
    }
    let open_edges = edge_count.values().filter(|&&n| n == 1).count();
    let non_manifold_edges = edge_count.values().filter(|&&n| n > 2).count();
    let watertight = tris > 0 && open_edges == 0 && non_manifold_edges == 0;

    let triangles = build_triangles(&positions, &welded);
    let (self_intersections, self_intersection_points) = self_intersections(&triangles, &positions);
    let (min_wall, min_wall_at) = if watertight {
        estimate_min_wall(&triangles, &positions)
    } else {
        (None, None)
    };

    MeshCheck {
        watertight,
        open_edges,
        non_manifold_edges,
        degenerate_tris,
        self_intersections,
        self_intersection_points,
        volume: crate::fillet::signed_volume(mesh),
        bbox: bbox_from_positions(&positions),
        tris,
        min_wall,
        min_wall_at,
    }
}

/// A stable content fingerprint of one entity. It covers everything a scene
/// diff should notice: name, color, placement and the local mesh (solids) or
/// the solved drawing and constraints (sketches). Derived normals are left
/// out; they are a pure function of positions and indices.
pub fn entity_signature(entity: &Entity) -> String {
    let mut h = Hasher::new();
    h.str("wright/entity/1");
    h.str(&entity.name);
    h.vec3(entity.translation);
    h.vec3(entity.rotation_deg);
    h.vec3(entity.scale);
    for c in entity.color {
        h.f32(c);
    }
    h.str(entity.shape.kind());

    match &entity.shape {
        Shape::Sketch {
            elements,
            constraints,
            solved,
            residual,
            dof,
            ..
        } => {
            h.str("sketch");
            h.u64(elements.len() as u64);
            for element in elements {
                match element {
                    SketchElement::Line { id, p1, p2, construction } => {
                        h.str("line");
                        h.u64(*id);
                        h.u64(u64::from(*construction));
                        h.xy(*p1);
                        h.xy(*p2);
                    }
                    SketchElement::Circle { id, center, radius, construction } => {
                        h.str("circle");
                        h.u64(*id);
                        h.u64(u64::from(*construction));
                        h.xy(*center);
                        h.f64(*radius);
                    }
                    SketchElement::Arc { id, center, radius, start_deg, end_deg, construction } => {
                        h.str("arc");
                        h.u64(*id);
                        h.u64(u64::from(*construction));
                        h.xy(*center);
                        h.f64(*radius);
                        h.f64(*start_deg);
                        h.f64(*end_deg);
                    }
                    SketchElement::Spline { id, points, closed, construction } => {
                        h.str("spline");
                        h.u64(*id);
                        h.u64(u64::from(*construction));
                        h.u64(u64::from(*closed));
                        h.u64(points.len() as u64);
                        for p in points {
                            h.xy(*p);
                        }
                    }
                }
            }
            h.u64(constraints.len() as u64);
            for constraint in constraints {
                h.u64(constraint.id);
                h.str(constraint.kind.as_str());
                h.u64(constraint.refs.len() as u64);
                for r in &constraint.refs {
                    h.u64(r.element);
                    h.u64(r.point.map_or(u64::MAX, u64::from));
                }
                match constraint.value {
                    Some(v) => {
                        h.u64(1);
                        h.f64(v);
                    }
                    None => h.u64(0),
                }
            }
            h.u64(u64::from(*solved));
            h.f64(*residual);
            h.u64(*dof as u64);
        }
        _ => {
            h.str("mesh");
            h.u64(entity.mesh.positions.len() as u64);
            for p in &entity.mesh.positions {
                h.vec3f(*p);
            }
            h.u64(entity.mesh.indices.len() as u64);
            for i in &entity.mesh.indices {
                h.u32(*i);
            }
        }
    }
    h.finish()
}

/// A fingerprint over every entity id + signature, in id order.
pub fn scene_signature(entities: &BTreeMap<EntityId, Entity>) -> String {
    let mut h = Hasher::new();
    h.str("wright/scene/1");
    h.u64(entities.len() as u64);
    for (id, entity) in entities {
        h.u64(*id);
        h.str(&entity_signature(entity));
    }
    h.finish()
}

// --- triangles -----------------------------------------------------------------

struct Tri {
    /// Welded vertex ids, in winding order.
    v: [u32; 3],
    p: [[f64; 3]; 3],
    lo: [f64; 3],
    hi: [f64; 3],
    /// Unit normal; zero for a degenerate triangle.
    n: [f64; 3],
    /// Triangle area (mm²).
    area: f64,
}

fn build_triangles(positions: &[[f64; 3]], welded: &[u32]) -> Vec<Tri> {
    let mut out = Vec::with_capacity(welded.len() / 3);
    for tri in welded.chunks_exact(3) {
        let v = [tri[0], tri[1], tri[2]];
        let p = [
            positions[tri[0] as usize],
            positions[tri[1] as usize],
            positions[tri[2] as usize],
        ];
        let cross = edges::cross(edges::sub(p[1], p[0]), edges::sub(p[2], p[0]));
        let n = edges::normalize(cross);
        let area = 0.5 * edges::dot(cross, cross).sqrt();
        let mut lo = [f64::MAX; 3];
        let mut hi = [f64::MIN; 3];
        for q in p {
            for (i, v) in q.iter().enumerate() {
                lo[i] = lo[i].min(*v);
                hi[i] = hi[i].max(*v);
            }
        }
        out.push(Tri {
            v,
            p,
            lo,
            hi,
            n,
            area,
        });
    }
    out
}

/// `(count, sample points)` of intersecting non-adjacent triangle pairs.
fn self_intersections(tris: &[Tri], positions: &[[f64; 3]]) -> (usize, Vec<Vec3>) {
    let diag = bbox_from_positions(positions);
    let scale = diagonal(&diag);
    // f32 mesh coordinates carry ~1e-5 relative error; the kernel's own
    // import tolerance is of the same order. Treat sub-tolerance plane
    // separations as contact, not crossing.
    let tol = (scale * 1e-5).max(1e-6);
    let min_area = negligible_area(scale);
    let mut order: Vec<usize> = (0..tris.len())
        .filter(|&i| !is_degenerate(&tris[i]) && tris[i].area >= min_area)
        .collect();
    // Sweep by min X. The tie-break on the triangle index keeps the visit
    // order deterministic even for exactly coincident extents.
    order.sort_by(|&a, &b| {
        tris[a].lo[0]
            .total_cmp(&tris[b].lo[0])
            .then_with(|| a.cmp(&b))
    });
    let mut count = 0usize;
    let mut points = Vec::new();
    for (oi, &i) in order.iter().enumerate() {
        let a = &tris[i];
        for &j in &order[oi + 1..] {
            let b = &tris[j];
            if b.lo[0] > a.hi[0] + tol {
                break; // sorted by min X: no later triangle can overlap
            }
            if !aabb_overlap(a, b, tol) || shares_vertex(a, b) {
                continue;
            }
            if tri_tri_intersect(a, b, tol) {
                count += 1;
                if points.len() < MAX_INTERSECTION_POINTS {
                    points.push(midpoint(centroid(a), centroid(b)));
                }
            }
        }
    }
    (count, points)
}

fn is_degenerate(t: &Tri) -> bool {
    edges::dot(t.n, t.n) < 0.5
}

/// Smaller than this and a triangle is numerical debris (a sub-print-
/// resolution sliver), not a surface that should drive a trust decision.
fn negligible_area(scale: f64) -> f64 {
    (scale * scale * 1e-12).max(1e-3)
}

fn aabb_overlap(a: &Tri, b: &Tri, tol: f64) -> bool {
    (0..3).all(|i| a.lo[i] <= b.hi[i] + tol && b.lo[i] <= a.hi[i] + tol)
}

fn shares_vertex(a: &Tri, b: &Tri) -> bool {
    a.v.iter().any(|x| b.v.contains(x))
}

/// Non-coplanar intersection: both triangles must straddle the other's plane
/// by more than the tolerance, otherwise they can only touch at a vertex/edge
/// (which is not a defect). Coplanar triangles are handled separately by
/// clipping the overlap polygon.
fn tri_tri_intersect(a: &Tri, b: &Tri, tol: f64) -> bool {
    if is_degenerate(a) || is_degenerate(b) {
        return false;
    }
    let da: Vec<f64> =
        b.p.iter()
            .map(|p| edges::dot(a.n, edges::sub(*p, a.p[0])))
            .collect();
    let db: Vec<f64> =
        a.p.iter()
            .map(|p| edges::dot(b.n, edges::sub(*p, b.p[0])))
            .collect();
    if da.iter().all(|d| d.abs() <= tol) && db.iter().all(|d| d.abs() <= tol) {
        return coplanar_overlap(a, b, tol);
    }
    // A positive-length intersection requires each triangle to have vertices
    // strictly on both sides of the other's plane. This is what keeps tangent
    // seams and numerical slivers from being reported as self-intersections.
    let a_crosses = da.iter().any(|d| *d > tol) && da.iter().any(|d| *d < -tol);
    let b_crosses = db.iter().any(|d| *d > tol) && db.iter().any(|d| *d < -tol);
    if !a_crosses || !b_crosses {
        return false;
    }

    for edge in edges_of(&a.p) {
        if segment_triangle(edge.0, edge.1, &b.p) {
            return true;
        }
    }
    for edge in edges_of(&b.p) {
        if segment_triangle(edge.0, edge.1, &a.p) {
            return true;
        }
    }
    false
}

fn coplanar_overlap(a: &Tri, b: &Tri, tol: f64) -> bool {
    // Project along the dominant component of the shared plane normal.
    let axis = (0..3)
        .max_by(|&i, &j| a.n[i].abs().total_cmp(&a.n[j].abs()))
        .unwrap_or(0);
    let (u, v) = match axis {
        0 => (1, 2),
        1 => (0, 2),
        _ => (0, 1),
    };
    let pa: Vec<[f64; 2]> = a.p.iter().map(|p| [p[u], p[v]]).collect();
    let pb: Vec<[f64; 2]> = b.p.iter().map(|p| [p[u], p[v]]).collect();

    // The intersection of two coplanar triangles is a polygon. Compute its
    // area instead of treating a shared boundary as a defect: tangent seams
    // touch along edges all the time, and a sub-resolution sliver is noise.
    let clipped = clip_polygon(&pa, &pb);
    let area = polygon_area(&clipped).abs();
    let area_eps = (tol * tol * 100.0).max(1e-12);
    area > area_eps
}

/// Sutherland-Hodgman clip of `subject` by the CCW polygon `clip`.
fn clip_polygon(subject: &[[f64; 2]], clip: &[[f64; 2]]) -> Vec<[f64; 2]> {
    let ccw = polygon_area(clip) >= 0.0;
    let mut out = subject.to_vec();
    for i in 0..clip.len() {
        if out.is_empty() {
            break;
        }
        let a = clip[i];
        let b = clip[(i + 1) % clip.len()];
        let side = |p: [f64; 2]| cross2([b[0] - a[0], b[1] - a[1]], [p[0] - a[0], p[1] - a[1]]);
        let inside = |d: f64| if ccw { d >= 0.0 } else { d <= 0.0 };
        let input = out;
        out = Vec::new();
        for j in 0..input.len() {
            let p = input[j];
            let q = input[(j + 1) % input.len()];
            let dp = side(p);
            let dq = side(q);
            if inside(dp) {
                out.push(p);
            }
            if inside(dp) != inside(dq) {
                let denom = dp - dq;
                if denom.abs() > 1e-18 {
                    let t = dp / denom;
                    out.push([p[0] + (q[0] - p[0]) * t, p[1] + (q[1] - p[1]) * t]);
                }
            }
        }
    }
    out
}

fn polygon_area(poly: &[[f64; 2]]) -> f64 {
    if poly.len() < 3 {
        return 0.0;
    }
    let mut sum = 0.0;
    for i in 0..poly.len() {
        let p = poly[i];
        let q = poly[(i + 1) % poly.len()];
        sum += p[0] * q[1] - q[0] * p[1];
    }
    sum * 0.5
}

fn cross2(a: [f64; 2], b: [f64; 2]) -> f64 {
    a[0] * b[1] - a[1] * b[0]
}

fn edges_of(p: &[[f64; 3]; 3]) -> [([f64; 3], [f64; 3]); 3] {
    [(p[0], p[1]), (p[1], p[2]), (p[2], p[0])]
}

/// Moller-Trumbore segment/triangle test. `p0`/`p1` define the segment, so
/// `t` is clamped to [0, 1].
fn segment_triangle(p0: [f64; 3], p1: [f64; 3], tri: &[[f64; 3]; 3]) -> bool {
    let (a, b, c) = (tri[0], tri[1], tri[2]);
    let d = edges::sub(p1, p0);
    let e1 = edges::sub(b, a);
    let e2 = edges::sub(c, a);
    let h = edges::cross(d, e2);
    let det = edges::dot(e1, h);
    if det.abs() < 1e-12 {
        return false;
    }
    let inv = 1.0 / det;
    let s = edges::sub(p0, a);
    let u = edges::dot(s, h) * inv;
    if !(-1e-9..=1.0 + 1e-9).contains(&u) {
        return false;
    }
    let q = edges::cross(s, e1);
    let v = edges::dot(d, q) * inv;
    if v < -1e-9 || u + v > 1.0 + 1e-9 {
        return false;
    }
    let t = edges::dot(e2, q) * inv;
    (-1e-9..=1.0 + 1e-9).contains(&t)
}

// --- minimum wall ---------------------------------------------------------------

/// Cast one ray inward from every sampled triangle and return the nearest
/// non-edge-adjacent hit. The estimate is deliberately conservative: it
/// reports the local material thickness at the sampled surface point, not a
/// global medial-axis thickness.
fn estimate_min_wall(tris: &[Tri], positions: &[[f64; 3]]) -> (Option<f64>, Option<Vec3>) {
    if tris.is_empty() {
        return (None, None);
    }
    let diag = bbox_from_positions(positions);
    let scale = diagonal(&diag);
    let offset = (scale * 1e-6).max(1e-4);
    // Contact thinner than print resolution is numerical noise (tangent
    // seams leave sub-micron slivers), not a wall we should fail. 0.05 mm is
    // an order of magnitude below a typical nozzle, so nothing printable is
    // hidden by this cutoff.
    let min_measure = (scale * 1e-5).max(0.05);
    let min_area = negligible_area(scale);
    let stride = tris.len().div_ceil(MAX_WALL_SAMPLES).max(1);

    let mut best = f64::INFINITY;
    let mut best_at = None;
    for i in (0..tris.len()).step_by(stride) {
        let t = &tris[i];
        if is_degenerate(t) || t.area < min_area {
            continue;
        }
        let center = centroid(t);
        let origin = edges::sub(center, edges::scale(t.n, offset));
        let dir = edges::scale(t.n, -1.0);
        let mut local = f64::INFINITY;
        for (j, u) in tris.iter().enumerate() {
            if j == i || shares_edge(t, u) || is_degenerate(u) || u.area < min_area {
                continue;
            }
            if let Some(distance) = ray_triangle(origin, dir, &u.p) {
                // The ray starts `offset` inside the surface, so the wall
                // thickness is the hit distance plus that offset.
                let thickness = distance + offset;
                if thickness >= min_measure && thickness < local {
                    local = thickness;
                }
            }
        }
        if local < best {
            best = local;
            best_at = Some(center);
        }
    }
    if best.is_finite() {
        (Some(best), best_at)
    } else {
        (None, None)
    }
}

/// Edge-adjacent (or identical) triangles are the surface the ray starts on;
/// hitting one at grazing incidence is not a wall measurement.
fn shares_edge(a: &Tri, b: &Tri) -> bool {
    let shared = a.v.iter().filter(|x| b.v.contains(x)).count();
    shared >= 2
}

/// Ray/triangle intersection, returning the positive distance along a unit
/// `dir`. Two-sided: the inward ray fires from inside the solid and reaches
/// the opposite surface regardless of its winding.
fn ray_triangle(origin: [f64; 3], dir: [f64; 3], tri: &[[f64; 3]; 3]) -> Option<f64> {
    let (a, b, c) = (tri[0], tri[1], tri[2]);
    let e1 = edges::sub(b, a);
    let e2 = edges::sub(c, a);
    let h = edges::cross(dir, e2);
    let det = edges::dot(e1, h);
    if det.abs() < 1e-12 {
        return None;
    }
    let inv = 1.0 / det;
    let s = edges::sub(origin, a);
    let u = edges::dot(s, h) * inv;
    if !(-1e-9..=1.0 + 1e-9).contains(&u) {
        return None;
    }
    let q = edges::cross(s, e1);
    let v = edges::dot(dir, q) * inv;
    if v < -1e-9 || u + v > 1.0 + 1e-9 {
        return None;
    }
    let t = edges::dot(e2, q) * inv;
    (t > 1e-9).then_some(t)
}

fn centroid(t: &Tri) -> [f64; 3] {
    [
        (t.p[0][0] + t.p[1][0] + t.p[2][0]) / 3.0,
        (t.p[0][1] + t.p[1][1] + t.p[2][1]) / 3.0,
        (t.p[0][2] + t.p[1][2] + t.p[2][2]) / 3.0,
    ]
}

fn midpoint(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        (a[0] + b[0]) * 0.5,
        (a[1] + b[1]) * 0.5,
        (a[2] + b[2]) * 0.5,
    ]
}

fn mesh_bbox(mesh: &Mesh) -> Bbox {
    let mut lo = [f64::MAX; 3];
    let mut hi = [f64::MIN; 3];
    for p in &mesh.positions {
        for (i, v) in p.iter().enumerate() {
            lo[i] = lo[i].min(*v as f64);
            hi[i] = hi[i].max(*v as f64);
        }
    }
    if mesh.positions.is_empty() {
        Bbox {
            min: [0.0; 3],
            max: [0.0; 3],
        }
    } else {
        Bbox { min: lo, max: hi }
    }
}

fn bbox_from_positions(positions: &[[f64; 3]]) -> Bbox {
    let mut lo = [f64::MAX; 3];
    let mut hi = [f64::MIN; 3];
    for p in positions {
        for (i, v) in p.iter().enumerate() {
            lo[i] = lo[i].min(*v);
            hi[i] = hi[i].max(*v);
        }
    }
    if positions.is_empty() {
        Bbox {
            min: [0.0; 3],
            max: [0.0; 3],
        }
    } else {
        Bbox { min: lo, max: hi }
    }
}

fn diagonal(b: &Bbox) -> f64 {
    edges::sub(b.max, b.min)
        .iter()
        .map(|v| v * v)
        .sum::<f64>()
        .sqrt()
}

// --- signatures ----------------------------------------------------------------

/// FNV-1a, chosen because it is tiny, deterministic and dependency-free.
struct Hasher(u64);

impl Hasher {
    fn new() -> Self {
        Self(0xcbf2_9ce4_8422_2325)
    }
    fn bytes(&mut self, bytes: &[u8]) {
        for &b in bytes {
            self.0 ^= u64::from(b);
            self.0 = self.0.wrapping_mul(0x0000_0100_0000_01b3);
        }
    }
    fn str(&mut self, s: &str) {
        self.bytes(s.as_bytes());
        self.bytes(&[0]);
    }
    fn u64(&mut self, v: u64) {
        self.bytes(&v.to_le_bytes());
    }
    fn u32(&mut self, v: u32) {
        self.bytes(&v.to_le_bytes());
    }
    fn f32(&mut self, v: f32) {
        self.u32(v.to_bits());
    }
    fn f64(&mut self, v: f64) {
        self.u64(v.to_bits());
    }
    fn vec3(&mut self, v: [f64; 3]) {
        for x in v {
            self.f64(x);
        }
    }
    fn vec3f(&mut self, v: [f32; 3]) {
        for x in v {
            self.f32(x);
        }
    }
    fn xy(&mut self, v: [f64; 2]) {
        self.f64(v[0]);
        self.f64(v[1]);
    }
    fn finish(self) -> String {
        format!("{:016x}", self.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geo;

    fn concat(a: &Mesh, b: &Mesh) -> Mesh {
        let mut out = Mesh::default();
        out.positions.extend_from_slice(&a.positions);
        out.normals.extend_from_slice(&a.normals);
        out.indices.extend_from_slice(&a.indices);
        let base = a.positions.len() as u32;
        out.indices.extend(b.indices.iter().map(|i| i + base));
        out.positions.extend_from_slice(&b.positions);
        out.normals.extend_from_slice(&b.normals);
        out
    }

    fn shifted(mesh: &Mesh, by: [f32; 3]) -> Mesh {
        let mut out = mesh.clone();
        for p in &mut out.positions {
            for i in 0..3 {
                p[i] += by[i];
            }
        }
        out
    }

    #[test]
    fn a_box_is_watertight_with_the_expected_volume_and_wall() {
        let check = check_solid(&geo::box_mesh([10.0, 20.0, 30.0]));
        assert!(check.watertight);
        assert_eq!(check.open_edges, 0);
        assert_eq!(check.non_manifold_edges, 0);
        assert_eq!(check.self_intersections, 0);
        assert!(
            (check.volume - 6000.0).abs() < 1e-6,
            "volume {}",
            check.volume
        );
        assert_eq!(check.tris, 12);
        let wall = check.min_wall.expect("closed box has a wall estimate");
        assert!((wall - 10.0).abs() < 1e-3, "min wall {wall}, expected 10");
        let bb = check.bbox;
        assert!(
            (bb.min[0] + 5.0).abs() < 1e-5 && (bb.max[2] - 15.0).abs() < 1e-5,
            "{bb:?}"
        );
    }

    #[test]
    fn an_open_mesh_reports_the_hole_and_skips_the_wall() {
        let mut mesh = geo::box_mesh([10.0; 3]);
        mesh.indices.truncate(mesh.indices.len() - 3);
        let check = check_solid(&mesh);
        assert!(!check.watertight);
        assert_eq!(
            check.open_edges, 3,
            "removing a triangle opens its three edges"
        );
        assert_eq!(check.min_wall, None);
    }

    #[test]
    fn interpenetrating_components_are_self_intersections() {
        // Two closed boxes merged into one entity without a boolean: every
        // edge still has two triangles, but the surfaces cross.
        let a = geo::box_mesh([10.0; 3]);
        let b = shifted(&a, [4.0, 0.0, 0.0]);
        let check = check_solid(&concat(&a, &b));
        assert!(
            check.watertight,
            "two closed components are still watertight"
        );
        assert!(check.self_intersections > 0, "overlap must be detected");
        assert!(!check.self_intersection_points.is_empty());
    }

    #[test]
    fn a_thin_plate_has_its_thickness_as_the_minimum_wall() {
        let check = check_solid(&geo::box_mesh([20.0, 20.0, 0.8]));
        let wall = check.min_wall.expect("closed plate has a wall estimate");
        assert!((wall - 0.8).abs() < 1e-3, "min wall {wall}, expected 0.8");
    }
}
