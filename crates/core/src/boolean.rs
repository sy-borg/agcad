//! Mesh booleans via [manifold-rust](https://docs.rs/manifold-rust) — the
//! pure-Rust port of the Manifold geometry kernel (v3.5.4 algorithms).
//!
//! Why this kernel: watertight, guaranteed-manifold output (what a slicer
//! needs), and **bit-identical determinism** — which the parametric replay
//! story depends on (same program → same geometry, every time).
//!
//! All conversions are `f32` (agcad) ↔ `f64` (Manifold). Precision loss on
//! the way in is irrelevant for millimeter-scale print geometry; the
//! kernel itself works in f64 internally.

use manifold_rust::manifold::Manifold;
use manifold_rust::types::MeshGL64;

use agcad_protocol::{BooleanOp, Mesh};

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

/// agcad `Mesh` (f32, indexed) → Manifold solid, via strict import.
///
/// Strict import requires every edge to be shared by exactly two
/// triangles, so duplicated vertices are welded by exact position first
/// (our primitives emit face-local vertices; the generators are
/// deterministic, so equal positions have equal bits).
fn to_manifold(mesh: &Mesh) -> Result<Manifold, Error> {
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

    let m = MeshGL64 {
        num_prop: 3,
        vert_properties: verts.iter().flat_map(|p| p.to_vec()).collect(),
        tri_verts: tris,
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

/// Manifold solid → agcad `Mesh` (f32, indexed), with per-vertex normals
/// accumulated from adjacent faces (smooth shading for curved results).
fn from_manifold(solid: &Manifold) -> Mesh {
    let m = solid.get_mesh_gl64(0);
    let positions: Vec<[f32; 3]> = m
        .vert_properties
        .chunks_exact(3)
        .map(|c| [c[0] as f32, c[1] as f32, c[2] as f32])
        .collect();

    // Accumulate face normals onto shared vertices, then normalize.
    let mut normals = vec![[0.0f32; 3]; positions.len()];
    for tri in m.tri_verts.chunks_exact(3) {
        let (a, b, c) = (tri[0] as usize, tri[1] as usize, tri[2] as usize);
        if a >= positions.len() || b >= positions.len() || c >= positions.len() {
            continue;
        }
        let (p0, p1, p2) = (positions[a], positions[b], positions[c]);
        let e1 = [p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]];
        let e2 = [p2[0] - p0[0], p2[1] - p0[1], p2[2] - p0[2]];
        let n = [
            e1[1] * e2[2] - e1[2] * e2[1],
            e1[2] * e2[0] - e1[0] * e2[2],
            e1[0] * e2[1] - e1[1] * e2[0],
        ];
        for &v in tri {
            let slot = &mut normals[v as usize];
            slot[0] += n[0];
            slot[1] += n[1];
            slot[2] += n[2];
        }
    }
    for n in normals.iter_mut() {
        let len = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
        if len > 1e-12 {
            *n = [n[0] / len, n[1] / len, n[2] / len];
        }
    }

    Mesh { positions, normals, indices: m.tri_verts.iter().map(|&i| i as u32).collect() }
}
