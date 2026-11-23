//! Mesh booleans via [manifold-rust](https://docs.rs/manifold-rust) — the
//! pure-Rust port of the Manifold geometry kernel (v3.5.4 algorithms).
//!
//! Why this kernel: watertight, guaranteed-manifold output (what a slicer
//! needs), and **bit-identical determinism** — which the parametric replay
//! story depends on (same program → same geometry, every time).
//!
//! All conversions are `f32` (wright) ↔ `f64` (Manifold). Precision loss on
//! the way in is irrelevant for millimeter-scale print geometry; the
//! kernel itself works in f64 internally.

use manifold_rust::manifold::Manifold;
use manifold_rust::types::MeshGL64;

use wright_protocol::{BooleanOp, Mesh};

use crate::Error;

/// Run a boolean over **world-space** meshes (already transformed). Returns
/// the fused mesh with smooth per-vertex normals. Errors carry hints the
/// agent can act on.
pub fn boolean(op: BooleanOp, meshes: &[Mesh]) -> Result<Mesh, Error> {
    let solids: Vec<Manifold> = meshes.iter().map(|m| to_manifold(m)).collect::<Result<_, _>>()?;

    let mut iter = solids.into_iter();
    let first = iter.next().ok_or_else(|| {
        Error::Invalid("boolean needs at least 2 entities (hint: union/difference/intersection all take a list)".into())
    })?;

    let result = match op {
        BooleanOp::Union => iter.fold(first, |acc, m| acc.union(&m)),
        BooleanOp::Difference => iter.fold(first, |acc, m| acc.difference(&m)),
        BooleanOp::Intersection => iter.fold(first, |acc, m| acc.intersection(&m)),
    };

    // A valid solid can legitimately be empty (difference ate everything).
    // Anything else is a kernel-level failure worth reporting loudly.
    let status = result.status();
    if status != manifold_rust::types::Error::NoError && !is_empty(&result) {
        return Err(Error::Invalid(format!(
            "boolean {op} failed in the kernel: {status:?} (hint: operands must be closed solids; check that every source entity is a valid primitive or boolean result)"
        )));
    }
    Ok(from_manifold(&result))
}

fn is_empty(m: &Manifold) -> bool {
    m.num_tri() == 0
}

/// wright `Mesh` (f32, indexed) → Manifold solid, via strict import.
///
/// Strict import requires every edge to be shared by exactly two
/// triangles, so duplicated vertices are welded by exact position first
/// (our primitives emit face-local vertices; the generators are
/// deterministic, so equal positions have equal bits).
pub(crate) fn to_manifold(mesh: &Mesh) -> Result<Manifold, Error> {
    use std::collections::BTreeMap;

    let mut map: BTreeMap<[u32; 3], u64> = BTreeMap::new();
    let mut verts: Vec<[f64; 3]> = Vec::new();
    let remap: Vec<u64> = mesh
        .positions
        .iter()
        .map(|p| {
            let key = [p[0].to_bits(), p[1].to_bits(), p[2].to_bits()];
            let len = verts.len() as u64;
            *map.entry(key).or_insert_with(|| {
                verts.push([p[0] as f64, p[1] as f64, p[2] as f64]);
                len
            })
        })
        .collect();
    let tris: Vec<u64> = mesh.indices.iter().map(|&i| remap[i as usize]).collect();

    // The import tolerance becomes the kernel's epsilon. Our meshes are f32,
    // so a boolean can emit vertices a few f32 ulps apart; on a filleted
    // tessellated rim that leaves zero-length edges, and welding by exact bits
    // afterwards turns them into non-manifold seams. A handful of f32 ulps
    // (floored at 10 nm, far below any printable feature) lets the kernel
    // treat them as one vertex.
    let mut lo = [f64::MAX; 3];
    let mut hi = [f64::MIN; 3];
    for p in &verts {
        for k in 0..3 {
            lo[k] = lo[k].min(p[k]);
            hi[k] = hi[k].max(p[k]);
        }
    }
    let diag = ((hi[0] - lo[0]).powi(2) + (hi[1] - lo[1]).powi(2) + (hi[2] - lo[2]).powi(2)).sqrt();
    let tolerance = (5e-7 * diag).max(3e-4);
    let m = MeshGL64 {
        num_prop: 3,
        vert_properties: verts.iter().flat_map(|p| p.to_vec()).collect(),
        tri_verts: tris,
        tolerance,
        ..Default::default()
    };
    let solid = Manifold::from_mesh_gl64(&m);
    let status = solid.status();
    if status != manifold_rust::types::Error::NoError {
        return Err(Error::Invalid(format!(
            "mesh is not a valid solid ({status:?}; {} verts, {} tris; hint: operands must be closed solids — check the source entities)",
            m.num_vert(),
            m.num_tri(),
        )));
    }
    Ok(solid)
}

/// Manifold solid → wright `Mesh` (f32, indexed).
///
/// Normals are crease-aware: faces that meet at more than [`CREASE_DEG`] keep
/// a hard edge, while gentle facets (a fillet's arc, a tessellated cylinder)
/// share a smooth normal. That needs one vertex per triangle corner, so the
/// result has 3× the triangle count in vertices — harmless, because every
/// consumer that needs topology (`boolean::to_manifold`, `edges::weld`)
/// welds by exact position bits anyway.
pub(crate) fn from_manifold(solid: &Manifold) -> Mesh {
    let m = solid.get_mesh_gl64(0);
    let positions: Vec<[f32; 3]> = m
        .vert_properties
        .chunks_exact(3)
        .map(|c| [c[0] as f32, c[1] as f32, c[2] as f32])
        .collect();
    let indices: Vec<u32> = m.tri_verts.iter().map(|&i| i as u32).collect();
    crease_mesh(&positions, &indices)
}

/// Faces meeting at less than this angle are shaded as one smooth surface.
pub(crate) const CREASE_DEG: f64 = 35.0;

/// Import a triangulated mesh through Manifold's strict mesher and return it
/// with crease-aware normals. Use this to validate hand-built geometry (a
/// loft, a thread ridge): a non-manifold or open result is a clear error
/// instead of a broken scene entity.
pub(crate) fn validate_mesh(mesh: &Mesh) -> Result<Mesh, Error> {
    let solid = to_manifold(mesh)?;
    Ok(from_manifold(&solid))
}

/// Rebuild a mesh with per-corner normals that only average faces within
/// [`CREASE_DEG`] of the corner's own triangle. Used after a world-space
/// operation is mapped back into an entity's local frame, where cached
/// normals would be wrong.
pub(crate) fn crease_mesh(positions: &[[f32; 3]], indices: &[u32]) -> Mesh {
    // Weld by exact bits first: adjacency is by *position*, so duplicated
    // vertices from an earlier crease pass do not split the smoothing
    // groups.
    use std::collections::BTreeMap;
    let mut ids: BTreeMap<[u32; 3], u32> = BTreeMap::new();
    let mut welded: Vec<[f32; 3]> = Vec::new();
    let mut remap: Vec<u32> = Vec::with_capacity(positions.len());
    for p in positions {
        let key = [p[0].to_bits(), p[1].to_bits(), p[2].to_bits()];
        let id = *ids.entry(key).or_insert_with(|| {
            welded.push(*p);
            (welded.len() - 1) as u32
        });
        remap.push(id);
    }
    let tris: Vec<[u32; 3]> = indices
        .chunks_exact(3)
        .map(|t| [remap[t[0] as usize], remap[t[1] as usize], remap[t[2] as usize]])
        .collect();

    // Face normals and the faces meeting at each welded vertex.
    let face_normal = |t: &[u32; 3]| -> [f32; 3] {
        let p = |i: u32| welded[i as usize];
        let (p0, p1, p2) = (p(t[0]), p(t[1]), p(t[2]));
        let e1 = [p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]];
        let e2 = [p2[0] - p0[0], p2[1] - p0[1], p2[2] - p0[2]];
        [
            e1[1] * e2[2] - e1[2] * e2[1],
            e1[2] * e2[0] - e1[0] * e2[2],
            e1[0] * e2[1] - e1[1] * e2[0],
        ]
    };
    let normals: Vec<[f32; 3]> = tris.iter().map(face_normal).collect();
    let mut around: Vec<Vec<usize>> = vec![Vec::new(); welded.len()];
    for (ti, t) in tris.iter().enumerate() {
        for &v in t {
            around[v as usize].push(ti);
        }
    }

    let min_cos = CREASE_DEG.to_radians().cos() as f32;
    let mut out_positions = Vec::with_capacity(tris.len() * 3);
    let mut out_normals = Vec::with_capacity(tris.len() * 3);
    let mut out_indices = Vec::with_capacity(tris.len() * 3);
    for (ti, t) in tris.iter().enumerate() {
        let own = normals[ti];
        for &v in t {
            let mut acc = [0.0f32; 3];
            for &other in &around[v as usize] {
                let n = normals[other];
                let len = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
                if len < 1e-12 {
                    continue;
                }
                let unit = [n[0] / len, n[1] / len, n[2] / len];
                let own_len = (own[0] * own[0] + own[1] * own[1] + own[2] * own[2]).sqrt();
                let own_unit =
                    [own[0] / own_len, own[1] / own_len, own[2] / own_len];
                let d = unit[0] * own_unit[0] + unit[1] * own_unit[1] + unit[2] * own_unit[2];
                if d >= min_cos {
                    acc[0] += unit[0];
                    acc[1] += unit[1];
                    acc[2] += unit[2];
                }
            }
            let len = (acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2]).sqrt();
            let n = if len > 1e-12 {
                [acc[0] / len, acc[1] / len, acc[2] / len]
            } else {
                own
            };
            out_positions.push(welded[v as usize]);
            out_normals.push(n);
            out_indices.push((out_positions.len() - 1) as u32);
        }
    }
    Mesh { positions: out_positions, normals: out_normals, indices: out_indices }
}
