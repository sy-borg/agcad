//! Stable references: enumerate the faces and edges a selector can address,
//! and report what a selector actually bound to.
//!
//! A triangle mesh has no faces or edges, only welded triangle pairs. This
//! module rebuilds the topology an agent talks about:
//!
//! * **Faces** are maximal coplanar patches (the same grouping
//!   [`crate::plane::planar_faces`] uses for sketch planes). Each face gets a
//!   deterministic 1-based id, a readable descriptor and a fingerprint.
//! * **Edges** are logical runs: sharp segments that share an endpoint and
//!   continue in (nearly) the same direction are one edge. A 32-segment
//!   cylinder rim is therefore one edge, not 32 candidates, while a box
//!   corner still splits into three edges at 90°.
//!
//! Two hashes travel with every group:
//!
//! * `fingerprint` is an **identity**: kind + convexity + direction + adjacent
//!   face normals + a rank among identical groups. It ignores absolute
//!   position, so a knob that resizes a part keeps the same reference.
//! * `geometry` hashes the exact welded vertices. It changes when the same
//!   feature moves, which is reported as `moved`, not as drift.
//!
//! Selectors that carry a [`wright_protocol::RefExpect`] are compared against
//! the live binding. Drift means the selector now selects a different logical
//! feature; that is the failure mode this milestone exists to make loud.
//!
//! Everything here is deterministic: welded ids come from a `BTreeMap`,
//! groups keep mesh order, and hashes are FNV-1a over exact bits.

use std::collections::{BTreeMap, BTreeSet};

use wright_protocol::{
    Bbox, EdgeFilter, EdgeGroupInfo, FaceGroupInfo, FaceSelector, Mesh, RefExpect,
    ReferenceDrift, ReferenceReport, Vec3,
};

use crate::edges::{self, SharpEdge};
use crate::plane;
use crate::Error;

/// Two sharp segments continue the same logical edge when their directions
/// differ by no more than this. 32-segment rims turn 11.25° per facet;
/// 90° box corners are far above it. Deliberately independent of
/// `min_angle_deg`: that threshold decides what is sharp, this one decides
/// what is one run.
const RUN_TANGENT_DEG: f64 = 20.0;

/// A resolved planar face.
#[derive(Debug, Clone)]
pub struct FaceGroup {
    pub id: u32,
    pub normal: Vec3,
    pub center: Vec3,
    pub area: f64,
    pub bbox: Bbox,
    pub tris: usize,
    pub edge_ids: Vec<u32>,
    pub fingerprint: String,
    pub geometry: String,
    pub descriptor: String,
    pub selector: FaceSelector,
}

/// A resolved logical edge.
#[derive(Debug, Clone)]
pub struct EdgeGroup {
    pub id: u32,
    pub kind: &'static str,
    pub a: Option<Vec3>,
    pub b: Option<Vec3>,
    pub midpoint: Vec3,
    pub direction: Option<Vec3>,
    pub length: f64,
    pub convex: Option<bool>,
    pub dihedral_deg: Option<f64>,
    pub segments: usize,
    pub faces: Vec<u32>,
    pub fingerprint: String,
    pub geometry: String,
    pub descriptor: String,
    pub selector: EdgeFilter,
}

/// The complete face/edge listing of one solid, in world space.
#[derive(Debug, Clone)]
pub struct Topology {
    pub faces: Vec<FaceGroup>,
    pub edges: Vec<EdgeGroup>,
    /// The underlying sharp segments, in `edges::sharp_edges` order.
    pub sharp: Vec<SharpEdge>,
    pub bbox_diagonal: f64,
    /// Segment index → logical edge id (1-based).
    segment_to_group: Vec<u32>,
}

/// Build the face and logical-edge listing of a **world-space** mesh.
pub fn topology(mesh: &Mesh, entity: u64, min_angle_deg: f64) -> Result<Topology, Error> {
    let (positions, welded_tris) = edges::weld(mesh)?;
    let (lo, hi) = positions_bounds(&positions);
    let diag = (0..3)
        .map(|i| (hi[i] - lo[i]) * (hi[i] - lo[i]))
        .sum::<f64>()
        .sqrt();

    let patches = plane::planar_faces(mesh)?;
    let mut faces: Vec<FaceGroup> = Vec::with_capacity(patches.len());
    let mut ranks: BTreeMap<String, u32> = BTreeMap::new();
    for (index, patch) in patches.iter().enumerate() {
        let (nx, ny, nz) = vec_key(patch.normal, 1e4, false);
        let base = format!("face:{nx}:{ny}:{nz}");
        let ordinal = ranks.get(&base).copied().unwrap_or(0);
        ranks.insert(base.clone(), ordinal + 1);
        let fingerprint = hash_str(&format!("wright/face/1|{base}#{ordinal}"));

        let mut bbox = Bbox { min: [f64::MAX; 3], max: [f64::MIN; 3] };
        let mut geometry = Fnv::new();
        geometry.str("wright/face-geom/1");
        let mut verts: BTreeSet<[u64; 3]> = BTreeSet::new();
        for &t in &patch.triangles {
            for &i in &welded_tris[t * 3..t * 3 + 3] {
                let p = positions[i as usize];
                include(&mut bbox, p);
                verts.insert([p[0].to_bits(), p[1].to_bits(), p[2].to_bits()]);
            }
        }
        for v in &verts {
            for x in v {
                geometry.u64(*x);
            }
        }

        let id = (index + 1) as u32;
        let descriptor = format!(
            "planar face #{id}: normal {}, area {:.2} mm², center {}",
            axis_name(patch.normal),
            patch.area,
            point(patch.centroid)
        );
        faces.push(FaceGroup {
            id,
            normal: patch.normal,
            center: patch.centroid,
            area: patch.area,
            bbox,
            tris: patch.triangles.len(),
            edge_ids: Vec::new(),
            fingerprint,
            geometry: geometry.finish(),
            descriptor,
            selector: FaceSelector { entity, normal: patch.normal, near: patch.centroid },
        });
    }

    let sharp = edges::sharp_edges(mesh, min_angle_deg)?;
    let groups = group_segments(&sharp);
    let mut segment_to_group = vec![0u32; sharp.len()];
    let mut edge_ranks: BTreeMap<String, u32> = BTreeMap::new();
    let mut out_edges: Vec<EdgeGroup> = Vec::with_capacity(groups.len());
    for (index, members) in groups.iter().enumerate() {
        let id = (index + 1) as u32;
        for &m in members {
            segment_to_group[m] = id;
        }
        let mut built = describe_edge(id, members, &sharp, &faces, diag);
        let ordinal = edge_ranks.get(&built.base).copied().unwrap_or(0);
        edge_ranks.insert(built.base.clone(), ordinal + 1);
        built.info.fingerprint =
            hash_str(&format!("wright/edge/1|{}#{ordinal}", built.base));
        built.info.geometry = built.geometry;
        built.info.selector = built.selector;
        built.info.descriptor = describe_text(&built.info);
        out_edges.push(built.info);
    }

    // Face ↔ edge adjacency, now that both sides have ids.
    for edge in &out_edges {
        for &fid in &edge.faces {
            if let Some(face) = faces.iter_mut().find(|f| f.id == fid) {
                face.edge_ids.push(edge.id);
            }
        }
    }
    for face in &mut faces {
        face.edge_ids.sort_unstable();
        face.edge_ids.dedup();
    }

    Ok(Topology { faces, edges: out_edges, sharp, bbox_diagonal: diag, segment_to_group })
}

/// Public listing for `faces{entity}`.
pub fn face_groups(topology: &Topology) -> Vec<FaceGroupInfo> {
    topology
        .faces
        .iter()
        .map(|f| FaceGroupInfo {
            id: f.id,
            kind: "planar".into(),
            normal: f.normal,
            center: f.center,
            area: f.area,
            bbox: f.bbox,
            tris: f.tris,
            edges: f.edge_ids.clone(),
            descriptor: f.descriptor.clone(),
            fingerprint: f.fingerprint.clone(),
            geometry: f.geometry.clone(),
            selector: f.selector,
        })
        .collect()
}

/// Public listing for `edges{entity}`.
pub fn edge_groups(topology: &Topology) -> Vec<EdgeGroupInfo> {
    topology
        .edges
        .iter()
        .map(|e| EdgeGroupInfo {
            id: e.id,
            kind: e.kind.into(),
            a: e.a,
            b: e.b,
            midpoint: e.midpoint,
            direction: e.direction,
            length: e.length,
            convex: e.convex,
            dihedral_deg: e.dihedral_deg,
            segments: e.segments,
            faces: e.faces.clone(),
            descriptor: e.descriptor.clone(),
            fingerprint: e.fingerprint.clone(),
            geometry: e.geometry.clone(),
            selector: e.selector.clone(),
        })
        .collect()
}

/// Resolve an [`EdgeFilter`] to logical edge groups, reporting ambiguity and
/// drift against the filter's own stored expectation.
pub fn match_edges(topology: &Topology, filter: &EdgeFilter) -> ReferenceReport {
    let mut per_group: BTreeMap<u32, usize> = BTreeMap::new();
    for (index, segment) in topology.sharp.iter().enumerate() {
        if edges::matches(filter, segment) {
            let gid = topology.segment_to_group[index];
            *per_group.entry(gid).or_insert(0) += 1;
        }
    }

    let groups: Vec<&EdgeGroup> = topology
        .edges
        .iter()
        .filter(|g| per_group.contains_key(&g.id))
        .collect();
    let ids: Vec<u32> = groups.iter().map(|g| g.id).collect();
    let elements = per_group.values().sum();
    let mut report = ReferenceReport {
        matched: ids.len(),
        groups: ids,
        ambiguous: groups.len() > 1 && !filter_is_empty(filter),
        chosen: None,
        elements,
        fingerprint: one_or_set(&groups, true),
        geometry: one_or_set(&groups, false),
        descriptors: descriptors_of(groups.iter().map(|g| g.descriptor.as_str())),
        drift: None,
        moved: false,
    };
    apply_expect(filter.expect.as_ref(), &mut report);
    report
}

/// Resolve a face-plane reference (normal filter + optional near point) to
/// planar faces. `chosen` follows the same rule as `add_sketch`: the nearest
/// plane when `near` is given, otherwise the largest face.
pub fn match_faces(
    topology: &Topology,
    near: Option<Vec3>,
    normal: Option<Vec3>,
    angle_tol_deg: Option<f64>,
    expect: Option<&RefExpect>,
) -> ReferenceReport {
    let tol = angle_tol_deg.unwrap_or(1.0).clamp(1e-6, 90.0);
    let min_cos = tol.to_radians().cos();
    let want = normal.map(edges::normalize);

    let candidates: Vec<&FaceGroup> = topology
        .faces
        .iter()
        .filter(|f| match want {
            Some(w) => edges::dot(f.normal, w) >= min_cos,
            None => true,
        })
        .collect();

    let chosen = match near {
        Some(p) => candidates.iter().copied().min_by(|a, b| {
            let da = (edges::dot(a.normal, p) - edges::dot(a.normal, a.center)).abs();
            let db = (edges::dot(b.normal, p) - edges::dot(b.normal, b.center)).abs();
            da.total_cmp(&db)
                .then_with(|| {
                    distance(a.center, p).total_cmp(&distance(b.center, p))
                })
                .then_with(|| b.area.total_cmp(&a.area))
                .then_with(|| a.id.cmp(&b.id))
        }),
        None => candidates
            .iter()
            .copied()
            .max_by(|a, b| a.area.total_cmp(&b.area).then_with(|| b.id.cmp(&a.id))),
    };

    let ids: Vec<u32> = candidates.iter().map(|f| f.id).collect();
    let chosen_id = chosen.map(|f| f.id);
    let mut ordered: Vec<&FaceGroup> = Vec::new();
    if let Some(c) = chosen {
        ordered.push(c);
    }
    for f in &candidates {
        if chosen_id != Some(f.id) {
            ordered.push(f);
        }
    }

    let mut report = ReferenceReport {
        matched: ids.len(),
        groups: ids,
        ambiguous: candidates.len() > 1,
        chosen: chosen_id,
        elements: chosen.map_or(0, |f| f.tris),
        fingerprint: chosen.map(|f| f.fingerprint.clone()),
        geometry: chosen.map(|f| f.geometry.clone()),
        descriptors: descriptors_of(ordered.into_iter().map(|f| f.descriptor.as_str())),
        drift: None,
        moved: false,
    };
    apply_expect(expect, &mut report);
    report
}

/// Resolve the entry face of a hole from its center and outward normal.
/// Unlike a sketch plane there is no "largest face" fallback: the point is
/// on the surface, so every face whose plane contains it and whose normal
/// agrees is a candidate.
pub fn match_entry_face(
    topology: &Topology,
    center: Vec3,
    normal: Vec3,
    angle_tol_deg: Option<f64>,
) -> ReferenceReport {
    let tol = angle_tol_deg.unwrap_or(5.0).clamp(1e-6, 90.0);
    let min_cos = tol.to_radians().cos();
    let want = edges::normalize(normal);
    let plane_tol = (topology.bbox_diagonal * 1e-5).max(1e-4);
    let candidates: Vec<&FaceGroup> = topology
        .faces
        .iter()
        .filter(|f| {
            edges::dot(f.normal, want) >= min_cos
                && (edges::dot(f.normal, center) - edges::dot(f.normal, f.center)).abs()
                    <= plane_tol
        })
        .collect();
    let chosen = candidates
        .iter()
        .copied()
        .min_by(|a, b| a.area.total_cmp(&b.area).then_with(|| a.id.cmp(&b.id)));
    let ids: Vec<u32> = candidates.iter().map(|f| f.id).collect();
    ReferenceReport {
        matched: ids.len(),
        groups: ids,
        ambiguous: candidates.len() > 1,
        chosen: chosen.map(|f| f.id),
        elements: chosen.map_or(0, |f| f.tris),
        fingerprint: chosen.map(|f| f.fingerprint.clone()),
        geometry: chosen.map(|f| f.geometry.clone()),
        descriptors: descriptors_of(candidates.iter().map(|f| f.descriptor.as_str())),
        drift: None,
        moved: false,
    }
}

/// Check a selector's stored expectation against the live report.
fn apply_expect(expect: Option<&RefExpect>, report: &mut ReferenceReport) {
    let Some(expect) = expect else { return };
    match (&report.fingerprint, &report.geometry) {
        (Some(actual), Some(geometry)) => {
            if actual != &expect.fingerprint {
                report.drift = Some(ReferenceDrift {
                    expected: expect.fingerprint.clone(),
                    actual: actual.clone(),
                    expected_descriptor: expect.descriptor.clone(),
                    actual_descriptor: report
                        .descriptors
                        .first()
                        .cloned()
                        .unwrap_or_else(|| "no matching geometry".into()),
                });
            } else {
                report.moved = geometry != &expect.geometry;
            }
        },
        _ => {
            report.drift = Some(ReferenceDrift {
                expected: expect.fingerprint.clone(),
                actual: String::new(),
                expected_descriptor: expect.descriptor.clone(),
                actual_descriptor: "no matching geometry".into(),
            });
        },
    }
}

/// The expectation to store for a successfully resolved report.
pub fn expect_of(report: &ReferenceReport) -> Option<RefExpect> {
    let fingerprint = report.fingerprint.clone()?;
    let geometry = report.geometry.clone().unwrap_or_default();
    let descriptor = match (report.descriptors.first(), report.matched) {
        (None, _) => "no matching geometry".to_string(),
        (Some(first), 1) => first.clone(),
        (Some(first), n) => format!("{n} matched: {first}; …"),
    };
    Some(RefExpect { fingerprint, geometry, matched: report.matched, descriptor })
}

/// A report for a non-geometric reference (a mirror plane, a pair of named
/// connectors) with a deterministic fingerprint.
pub fn fixed_report(domain: &str, values: &[f64], descriptor: String) -> ReferenceReport {
    let mut h = Fnv::new();
    h.str(domain);
    for v in values {
        h.u64(v.to_bits());
    }
    ReferenceReport::fixed(h.finish(), descriptor)
}

/// The report for a mirror plane: one logical match, no ambiguity.
pub fn mirror_report(origin: Vec3, normal: Vec3) -> ReferenceReport {
    let n = edges::normalize(normal);
    fixed_report(
        "wright/mirror-plane/1",
        &[origin[0], origin[1], origin[2], n[0], n[1], n[2]],
        format!("mirror plane through {}, normal {}", point(origin), axis_name(n)),
    )
}

/// True when the filter has no conditions (select every sharp edge).
pub fn filter_is_empty(filter: &EdgeFilter) -> bool {
    filter.direction.is_none()
        && filter.angle_tol_deg.is_none()
        && filter.near.is_none()
        && filter.near_radius.is_none()
        && filter.normals.is_none()
        && filter.convex.is_none()
}

// --- internals ---------------------------------------------------------------

struct Described {
    /// Identity token (without the ordinal).
    base: String,
    selector: EdgeFilter,
    geometry: String,
    info: EdgeGroup,
}

/// Split sharp segments into runs that share an endpoint and continue in the
/// same direction.
fn group_segments(sharp: &[SharpEdge]) -> Vec<Vec<usize>> {
    let n = sharp.len();
    let mut parent: Vec<usize> = (0..n).collect();
    fn find(parent: &mut [usize], mut x: usize) -> usize {
        while parent[x] != x {
            parent[x] = parent[parent[x]];
            x = parent[x];
        }
        x
    }
    fn union(parent: &mut [usize], a: usize, b: usize) {
        let (ra, rb) = (find(parent, a), find(parent, b));
        if ra != rb {
            let (lo, hi) = (ra.min(rb), ra.max(rb));
            parent[hi] = lo;
        }
    }

    let mut at: BTreeMap<[i64; 3], Vec<usize>> = BTreeMap::new();
    for (i, e) in sharp.iter().enumerate() {
        at.entry(vertex_key(e.a)).or_default().push(i);
        at.entry(vertex_key(e.b)).or_default().push(i);
    }
    let tangent_cos = RUN_TANGENT_DEG.to_radians().cos();
    for members in at.values() {
        for i in 0..members.len() {
            for j in (i + 1)..members.len() {
                let (a, b) = (members[i], members[j]);
                if edges::dot(sharp[a].direction(), sharp[b].direction()).abs() >= tangent_cos {
                    union(&mut parent, a, b);
                }
            }
        }
    }

    let mut groups: BTreeMap<usize, Vec<usize>> = BTreeMap::new();
    for i in 0..n {
        let root = find(&mut parent, i);
        groups.entry(root).or_default().push(i);
    }
    let mut out: Vec<Vec<usize>> = groups.into_values().collect();
    for g in &mut out {
        g.sort_unstable();
    }
    out.sort_by_key(|g| g[0]);
    out
}

fn describe_edge(
    id: u32,
    members: &[usize],
    sharp: &[SharpEdge],
    faces: &[FaceGroup],
    diag: f64,
) -> Described {
    let segs: Vec<&SharpEdge> = members.iter().map(|&i| &sharp[i]).collect();
    let dirs: Vec<Vec3> = segs.iter().map(|e| e.direction()).collect();
    let first_dir = dirs[0];
    let straight = dirs
        .iter()
        .all(|d| edges::dot(*d, first_dir).abs() >= 1.0 - 1e-9);
    let length: f64 = segs.iter().map(|e| e.length()).sum();
    let midpoint = length_weighted_midpoint(&segs, length);

    let (kind, direction, a, b) = if straight {
        let dir = canonical_dir(first_dir);
        let mut low = f64::MAX;
        let mut high = f64::MIN;
        let mut pa = midpoint;
        let mut pb = midpoint;
        for e in &segs {
            for p in [e.a, e.b] {
                let t = edges::dot(p, dir);
                if t < low {
                    low = t;
                    pa = p;
                }
                if t > high {
                    high = t;
                    pb = p;
                }
            }
        }
        ("line", Some(dir), Some(pa), Some(pb))
    } else {
        ("curve", None, None, None)
    };

    let convex = uniform_bool(segs.iter().map(|e| e.convex));
    let dihedral = uniform_f64(segs.iter().map(|e| e.dihedral_deg()));

    let mut face_ids: BTreeSet<u32> = BTreeSet::new();
    let plane_tol = (diag * 1e-5).max(1e-4);
    for e in &segs {
        for n in [e.n1, e.n2] {
            if let Some(f) = face_for(faces, e.midpoint(), n, plane_tol) {
                face_ids.insert(f);
            }
        }
    }
    let faces_vec: Vec<u32> = face_ids.into_iter().collect();

    // Identity: kind, convexity, direction (for a line), and the set of
    // adjacent face normals. Excludes absolute positions so resizes keep the
    // same fingerprint.
    let mut normals: BTreeSet<String> = BTreeSet::new();
    for e in &segs {
        normals.insert(key_string(e.n1, 1e4, false));
        normals.insert(key_string(e.n2, 1e4, false));
    }
    let dir_token = direction
        .map(|d| key_string(d, 1e4, true))
        .unwrap_or_else(|| "curve".into());
    let convex_token = match convex {
        Some(true) => "convex",
        Some(false) => "concave",
        None => "mixed",
    };
    let base = format!(
        "edge:{}:{}:{}:{}",
        kind,
        convex_token,
        dir_token,
        normals.into_iter().collect::<Vec<_>>().join(",")
    );

    // Exact geometry: welded endpoints and face normals, in segment order.
    let mut geometry = Fnv::new();
    geometry.str("wright/edge-geom/1");
    for e in &segs {
        for v in [e.a, e.b, e.n1, e.n2] {
            for x in v {
                geometry.u64(x.to_bits());
            }
        }
    }

    let mut selector = EdgeFilter { convex, ..Default::default() };
    if let Some(dir) = direction {
        selector.direction = Some(dir);
        selector.near = Some(midpoint);
        selector.near_radius = Some((length * 0.25).max(1e-3) + 1e-6);
    } else {
        // A rim or freeform run has no single direction. `near` is its
        // centroid; the radius reaches every segment midpoint.
        let reach = segs
            .iter()
            .map(|e| distance(e.midpoint(), midpoint))
            .fold(0.0f64, f64::max)
            + segs.iter().map(|e| e.length() * 0.5).fold(0.0f64, f64::max)
            + 1e-3;
        selector.near = Some(midpoint);
        selector.near_radius = Some(reach.max(1e-3));
        // If every segment shares one face normal (the cap of a rim), use it
        // to narrow the selector beyond the radius.
        if let Some(common) = common_normal(&segs) {
            selector.normals = Some(vec![common]);
        }
    }

    let info = EdgeGroup {
        id,
        kind,
        a,
        b,
        midpoint,
        direction,
        length,
        convex,
        dihedral_deg: dihedral,
        segments: segs.len(),
        faces: faces_vec,
        fingerprint: String::new(),
        geometry: String::new(),
        descriptor: String::new(),
        selector: selector.clone(),
    };
    Described { base, selector, geometry: geometry.finish(), info }
}

fn describe_text(info: &EdgeGroup) -> String {
    let face_text = match info.faces.len() {
        0 => "no adjacent face".to_string(),
        1 => format!("face #{}", info.faces[0]),
        2 => format!("faces #{} and #{}", info.faces[0], info.faces[1]),
        3 => {
            format!("faces #{}, #{} and #{}", info.faces[0], info.faces[1], info.faces[2])
        },
        n => format!("{n} adjacent faces"),
    };
    let convex = match info.convex {
        Some(true) => "convex",
        Some(false) => "concave",
        None => "mixed",
    };
    let dir = info
        .direction
        .map(|d| format!(", direction {}", axis_name(d)))
        .unwrap_or_default();
    format!(
        "{} edge #{}: {:.2} mm, {} segment(s){dir}, {convex}, between {face_text}, midpoint {}",
        info.kind,
        info.id,
        info.length,
        info.segments,
        point(info.midpoint)
    )
}

// --- small helpers -----------------------------------------------------------

fn vertex_key(p: Vec3) -> [i64; 3] {
    [
        (p[0] * 1000.0).round() as i64,
        (p[1] * 1000.0).round() as i64,
        (p[2] * 1000.0).round() as i64,
    ]
}

fn positions_bounds(positions: &[Vec3]) -> (Vec3, Vec3) {
    let mut lo = [f64::MAX; 3];
    let mut hi = [f64::MIN; 3];
    for p in positions {
        for i in 0..3 {
            lo[i] = lo[i].min(p[i]);
            hi[i] = hi[i].max(p[i]);
        }
    }
    if positions.is_empty() {
        ([0.0; 3], [0.0; 3])
    } else {
        (lo, hi)
    }
}

fn include(b: &mut Bbox, p: Vec3) {
    for i in 0..3 {
        b.min[i] = b.min[i].min(p[i]);
        b.max[i] = b.max[i].max(p[i]);
    }
}

fn length_weighted_midpoint(segs: &[&SharpEdge], total: f64) -> Vec3 {
    if total <= 0.0 {
        return segs.first().map_or([0.0; 3], |e| e.midpoint());
    }
    let mut sum = [0.0; 3];
    for e in segs {
        let w = e.length();
        let m = e.midpoint();
        for i in 0..3 {
            sum[i] += m[i] * w;
        }
    }
    [sum[0] / total, sum[1] / total, sum[2] / total]
}

/// Make an undirected edge direction point toward the positive side of its
/// first non-zero axis, so descriptors read "+Z" regardless of mesh winding.
fn canonical_dir(d: Vec3) -> Vec3 {
    let n = edges::normalize(d);
    let first = if n[0].abs() > 1e-9 {
        n[0]
    } else if n[1].abs() > 1e-9 {
        n[1]
    } else {
        n[2]
    };
    if first < 0.0 {
        [-n[0], -n[1], -n[2]]
    } else {
        n
    }
}

fn uniform_bool(mut values: impl Iterator<Item = bool>) -> Option<bool> {    let first = values.next()?;
    if values.all(|v| v == first) {
        Some(first)
    } else {
        None
    }
}

fn uniform_f64(mut values: impl Iterator<Item = f64>) -> Option<f64> {
    let first = values.next()?;
    if values.all(|v| (v - first).abs() < 1e-6) {
        Some(first)
    } else {
        None
    }
}

fn face_for(faces: &[FaceGroup], point: Vec3, normal: Vec3, tol: f64) -> Option<u32> {
    let want = edges::normalize(normal);
    let min_cos = 1e-4f64.to_radians().cos();
    faces
        .iter()
        .filter(|f| edges::dot(f.normal, want) >= min_cos)
        .filter(|f| {
            (edges::dot(f.normal, point) - edges::dot(f.normal, f.center)).abs() <= tol
        })
        .min_by(|a, b| a.area.total_cmp(&b.area).then_with(|| a.id.cmp(&b.id)))
        .map(|f| f.id)
}

fn common_normal(segs: &[&SharpEdge]) -> Option<Vec3> {
    let first = segs.first()?.n1;
    if segs.iter().all(|e| {
        edges::dot(e.n1, first).abs() >= 1.0 - 1e-9
            || edges::dot(e.n2, first).abs() >= 1.0 - 1e-9
    }) {
        Some(first)
    } else {
        None
    }
}

fn distance(a: Vec3, b: Vec3) -> f64 {
    let d = [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
    edges::dot(d, d).sqrt()
}

fn descriptors_of<'a>(items: impl Iterator<Item = &'a str>) -> Vec<String> {
    const CAP: usize = 8;
    let mut out = Vec::new();
    let mut more = 0;
    for item in items {
        if out.len() < CAP {
            out.push(item.to_string());
        } else {
            more += 1;
        }
    }
    if more > 0 {
        out.push(format!("… and {more} more"));
    }
    out
}

/// The group's own fingerprint when exactly one matched, otherwise a hash of
/// the matched set. Keeps a single-edge reference readable and comparable
/// with the `edges` listing.
fn one_or_set(groups: &[&EdgeGroup], identity: bool) -> Option<String> {
    match groups {
        [] => None,
        [only] => Some(if identity {
            only.fingerprint.clone()
        } else {
            only.geometry.clone()
        }),
        _ => set_hash(groups.iter().map(|g| {
            if identity {
                g.fingerprint.as_str()
            } else {
                g.geometry.as_str()
            }
        })),
    }
}

fn set_hash<'a>(items: impl Iterator<Item = &'a str>) -> Option<String> {    let mut sorted: Vec<&str> = items.collect();
    if sorted.is_empty() {
        return None;
    }
    sorted.sort_unstable();
    let mut h = Fnv::new();
    h.str("wright/ref-set/1");
    for s in sorted {
        h.str(s);
    }
    Some(h.finish())
}

fn hash_str(s: &str) -> String {
    let mut h = Fnv::new();
    h.str(s);
    h.finish()
}

fn point(p: Vec3) -> String {
    format!("({:.2}, {:.2}, {:.2})", p[0], p[1], p[2])
}

fn axis_name(v: Vec3) -> String {
    let n = edges::normalize(v);
    let axes = [
        ([1.0, 0.0, 0.0], "+X"),
        ([-1.0, 0.0, 0.0], "-X"),
        ([0.0, 1.0, 0.0], "+Y"),
        ([0.0, -1.0, 0.0], "-Y"),
        ([0.0, 0.0, 1.0], "+Z"),
        ([0.0, 0.0, -1.0], "-Z"),
    ];
    for (axis, name) in axes {
        if edges::dot(n, axis) >= 1.0 - 1e-6 {
            return name.to_string();
        }
    }
    format!("({:.3}, {:.3}, {:.3})", n[0], n[1], n[2])
}

fn vec_key(v: Vec3, scale: f64, canonical: bool) -> (i64, i64, i64) {
    let mut q = [
        (v[0] * scale).round() as i64,
        (v[1] * scale).round() as i64,
        (v[2] * scale).round() as i64,
    ];
    if canonical && (q[0] < 0 || (q[0] == 0 && (q[1] < 0 || (q[1] == 0 && q[2] < 0)))) {
        q = [-q[0], -q[1], -q[2]];
    }
    (q[0], q[1], q[2])
}

fn key_string(v: Vec3, scale: f64, canonical: bool) -> String {
    let (x, y, z) = vec_key(v, scale, canonical);
    format!("{x},{y},{z}")
}

/// FNV-1a, the same parameters the entity signatures use.
struct Fnv(u64);

impl Fnv {
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
    fn finish(self) -> String {
        format!("{:016x}", self.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geo;

    fn box_topology(size: f64) -> Topology {
        topology(&geo::box_mesh([size; 3]), 1, 30.0).unwrap()
    }

    fn set(items: impl IntoIterator<Item = String>) -> BTreeSet<String> {
        items.into_iter().collect()
    }

    #[test]
    fn a_box_has_six_faces_and_twelve_logical_edges() {
        let t = box_topology(10.0);
        assert_eq!(t.faces.len(), 6);
        assert_eq!(t.edges.len(), 12);
        for face in &t.faces {
            assert_eq!(face.area, 100.0, "{}", face.descriptor);
            assert_eq!(face.edge_ids.len(), 4, "{}", face.descriptor);
            assert_eq!(face.tris, 2);
        }
        for edge in &t.edges {
            assert_eq!(edge.kind, "line");
            assert_eq!(edge.segments, 1);
            assert_eq!(edge.faces.len(), 2, "{}", edge.descriptor);
            assert!((edge.length - 10.0).abs() < 1e-6);
            assert_eq!(edge.convex, Some(true));
            assert_eq!(edge.dihedral_deg, Some(90.0));
        }
    }

    #[test]
    fn every_listed_group_selector_re_selects_exactly_that_group() {
        let t = box_topology(10.0);
        for edge in &t.edges {
            let report = match_edges(&t, &edge.selector);
            assert_eq!(report.groups, vec![edge.id], "{}", edge.descriptor);
            assert_eq!(report.matched, 1);
            assert!(!report.ambiguous, "group selectors are precise");
            assert_eq!(
                report.fingerprint.as_deref(),
                Some(edge.fingerprint.as_str())
            );
        }
        for face in &t.faces {
            let report = match_faces(
                &t,
                Some(face.selector.near),
                Some(face.selector.normal),
                None,
                None,
            );
            assert_eq!(report.chosen, Some(face.id), "{}", face.descriptor);
            assert_eq!(
                report.fingerprint.as_deref(),
                Some(face.fingerprint.as_str())
            );
        }
    }

    #[test]
    fn a_tessellated_rim_is_one_curve_edge() {
        let mesh = geo::cylinder_mesh(5.0, 10.0, 32);
        let t = topology(&mesh, 1, 30.0).unwrap();
        let curves: Vec<&EdgeGroup> = t.edges.iter().filter(|e| e.kind == "curve").collect();
        assert_eq!(curves.len(), 2, "one rim per cap");
        for curve in curves {
            assert_eq!(curve.segments, 32, "{}", curve.descriptor);
            assert!(curve.faces.len() > 30, "cap + every side facet");
            let report = match_edges(&t, &curve.selector);
            assert_eq!(report.groups, vec![curve.id], "{}", curve.descriptor);
        }
    }

    #[test]
    fn a_direction_filter_reports_every_matching_group() {
        let t = box_topology(10.0);
        let vertical = EdgeFilter { direction: Some([0.0, 0.0, 1.0]), ..Default::default() };
        let report = match_edges(&t, &vertical);
        assert_eq!(report.matched, 4);
        assert!(report.ambiguous);
        let ids: Vec<u32> = t
            .edges
            .iter()
            .filter(|e| edges::matches(&vertical, &t.sharp[e.id as usize - 1]))
            .map(|e| e.id)
            .collect();
        assert_eq!(report.groups, ids);
    }

    #[test]
    fn fingerprints_survive_a_resize_while_geometry_is_reported_as_moved() {
        let small = box_topology(10.0);
        let large = box_topology(11.0);

        assert_eq!(
            set(small.faces.iter().map(|f| f.fingerprint.clone())),
            set(large.faces.iter().map(|f| f.fingerprint.clone()))
        );
        assert_eq!(
            set(small.edges.iter().map(|e| e.fingerprint.clone())),
            set(large.edges.iter().map(|e| e.fingerprint.clone()))
        );
        assert_ne!(
            set(small.edges.iter().map(|e| e.geometry.clone())),
            set(large.edges.iter().map(|e| e.geometry.clone()))
        );

        let mut filter = small.edges[0].selector.clone();
        filter.expect = expect_of(&match_edges(&small, &filter));
        let report = match_edges(&large, &filter);
        assert!(report.drift.is_none(), "same logical edge");
        assert!(report.moved, "the edge moved/resized");
    }

    #[test]
    fn an_expectation_that_names_another_feature_is_drift() {
        let t = box_topology(10.0);
        let mut filter = t.edges[0].selector.clone();
        filter.expect = Some(RefExpect {
            fingerprint: "0000000000000000".into(),
            geometry: "0000000000000000".into(),
            matched: 1,
            descriptor: "the edge that used to be here".into(),
        });
        let report = match_edges(&t, &filter);
        let drift = report.drift.expect("mismatched expectation is drift");
        assert_eq!(drift.expected, "0000000000000000");
        assert_eq!(drift.actual, t.edges[0].fingerprint);
    }

    #[test]
    fn a_missing_face_reports_no_binding_and_drift_against_an_expectation() {
        let t = box_topology(10.0);
        let expect = RefExpect {
            fingerprint: "abc".into(),
            geometry: "def".into(),
            matched: 1,
            descriptor: "gone".into(),
        };
        let report = match_faces(&t, None, Some([1.0, 1.0, 1.0]), None, Some(&expect));
        assert_eq!(report.matched, 0);
        assert!(report.drift.is_some());
    }
}
