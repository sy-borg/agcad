//! Mesh repair for non-watertight imports (M22).
//!
//! Real interchange files are messy: one flipped triangle, a crack where two
//! shells meet, a fin left over from a boolean, a missing lid. `validate`
//! reports those honestly; this module is the explicit "make it printable"
//! step. It is deliberately a sequence of local, deterministic repairs:
//!
//! 1. weld by exact `f32` bits and drop repeated-corner / duplicate triangles;
//! 2. drop triangles that make an edge belong to more than two triangles;
//! 3. orient every connected shell consistently, then outward (negative
//!    volume is flipped);
//! 4. fill closed boundary loops — a triangle with one triangle, larger loops
//!    with a centroid fan.
//!
//! Nothing here guesses at self-intersections or coordinate drift; those stay
//! visible so the caller can decide whether the part is really the one they
//! meant. The output normals use the same crease-aware shading as import.

use std::collections::{BTreeMap, BTreeSet};

use wright_protocol::{Mesh, RepairReport};

use crate::Error;

/// Repair switches. Default is the full repair.
#[derive(Debug, Clone, Copy)]
pub struct Options {
    pub fill_holes: bool,
    pub fix_winding: bool,
    pub remove_non_manifold: bool,
    pub smooth_angle_deg: f64,
}

impl Default for Options {
    fn default() -> Self {
        Self {
            fill_holes: true,
            fix_winding: true,
            remove_non_manifold: true,
            smooth_angle_deg: crate::mesh_import::DEFAULT_SMOOTH_ANGLE_DEG,
        }
    }
}

/// Repair `mesh` and report what changed.
pub fn repair(mesh: &Mesh, opts: Options) -> Result<(Mesh, RepairReport), Error> {
    if !opts.smooth_angle_deg.is_finite() {
        return Err(Error::Invalid(
            "smooth_angle_deg must be a finite angle in degrees".into(),
        ));
    }
    let vertices_before = mesh.positions.len();
    let triangles_before = mesh.indices.len() / 3;
    if !mesh.indices.len().is_multiple_of(3) {
        return Err(Error::Invalid(format!(
            "mesh indices must be a multiple of 3 (got {}; hint: each triangle is three vertex \
             indices)",
            mesh.indices.len()
        )));
    }

    let (mut positions, mut triangles) = weld(mesh)?;
    drop_degenerate(&mut triangles);

    if opts.remove_non_manifold {
        prune_non_manifold(&mut triangles);
    }
    let flipped = if opts.fix_winding {
        orient_components(&positions, &mut triangles)
    } else {
        0
    };

    let mut holes_filled = 0usize;
    let mut triangles_added = 0usize;
    if opts.fill_holes {
        fill_boundary_loops(
            &mut positions,
            &mut triangles,
            &mut holes_filled,
            &mut triangles_added,
        );
    }

    let indices: Vec<u32> = triangles.iter().flatten().copied().collect();
    let positions_f32: Vec<[f32; 3]> = positions
        .iter()
        .map(|p| [p[0] as f32, p[1] as f32, p[2] as f32])
        .collect();
    let (out, cleaned) =
        crate::mesh_import::build(&positions_f32, &indices, opts.smooth_angle_deg)?;

    let (open_edges, non_manifold_edges) = edge_stats(&cleaned.triangles);
    let triangles_after = cleaned.triangles.len();
    let vertices_after = cleaned.positions.len();
    let triangles_removed = (triangles_before + triangles_added).saturating_sub(triangles_after);

    Ok((
        out,
        RepairReport {
            vertices_before,
            vertices_after,
            triangles_before,
            triangles_after,
            holes_filled,
            triangles_added,
            triangles_removed,
            flipped,
            open_edges,
            non_manifold_edges,
            watertight: open_edges == 0 && non_manifold_edges == 0,
        },
    ))
}

// --- weld + clean ------------------------------------------------------------

fn weld(mesh: &Mesh) -> Result<(Vec<[f64; 3]>, Vec<[u32; 3]>), Error> {
    let mut ids: BTreeMap<[u32; 3], u32> = BTreeMap::new();
    let mut positions: Vec<[f64; 3]> = Vec::new();
    let mut remap: Vec<u32> = Vec::with_capacity(mesh.positions.len());
    for p in &mesh.positions {
        let q = p.map(|v| if v == 0.0 { 0.0 } else { v });
        if q.iter().any(|v| !v.is_finite()) {
            return Err(Error::Invalid(
                "mesh contains a non-finite vertex (hint: check for NaN/inf coordinates)".into(),
            ));
        }
        let key = q.map(|v| v.to_bits());
        let id = match ids.get(&key) {
            Some(&id) => id,
            None => {
                let id = positions.len() as u32;
                ids.insert(key, id);
                positions.push([q[0] as f64, q[1] as f64, q[2] as f64]);
                id
            },
        };
        remap.push(id);
    }
    let mut triangles = Vec::with_capacity(mesh.indices.len() / 3);
    for t in mesh.indices.chunks_exact(3) {
        let tri = [
            *remap.get(t[0] as usize).ok_or_else(|| bad_index(t[0], remap.len()))?,
            *remap.get(t[1] as usize).ok_or_else(|| bad_index(t[1], remap.len()))?,
            *remap.get(t[2] as usize).ok_or_else(|| bad_index(t[2], remap.len()))?,
        ];
        triangles.push(tri);
    }
    Ok((positions, triangles))
}

fn bad_index(index: u32, len: usize) -> Error {
    Error::Invalid(format!(
        "mesh index {index} is out of range for {len} vertices (hint: indices are 0-based)"
    ))
}

fn drop_degenerate(triangles: &mut Vec<[u32; 3]>) {
    let mut seen: BTreeSet<[u32; 3]> = BTreeSet::new();
    triangles.retain(|t| {
        if t[0] == t[1] || t[1] == t[2] || t[0] == t[2] {
            return false;
        }
        let mut key = *t;
        key.sort_unstable();
        seen.insert(key)
    });
}

// --- non-manifold pruning ----------------------------------------------------

fn edge_key(a: u32, b: u32) -> (u32, u32) {
    if a < b { (a, b) } else { (b, a) }
}

fn edge_incidence(triangles: &[[u32; 3]]) -> BTreeMap<(u32, u32), Vec<(usize, bool)>> {
    let mut out: BTreeMap<(u32, u32), Vec<(usize, bool)>> = BTreeMap::new();
    for (index, t) in triangles.iter().enumerate() {
        for (a, b) in [(t[0], t[1]), (t[1], t[2]), (t[2], t[0])] {
            out.entry(edge_key(a, b)).or_default().push((index, a < b));
        }
    }
    out
}

fn prune_non_manifold(triangles: &mut Vec<[u32; 3]>) {
    loop {
        let incidence = edge_incidence(triangles);
        let mut drop: BTreeSet<usize> = BTreeSet::new();
        for entries in incidence.values() {
            if entries.len() <= 2 {
                continue;
            }
            let first = entries[0];
            let second = entries
                .iter()
                .skip(1)
                .find(|(_, forward)| *forward != first.1)
                .copied()
                .unwrap_or(entries[1]);
            for &(tri, _) in entries {
                if tri != first.0 && tri != second.0 {
                    drop.insert(tri);
                }
            }
        }
        if drop.is_empty() {
            break;
        }
        let mut index = 0usize;
        triangles.retain(|_| {
            let keep = !drop.contains(&index);
            index += 1;
            keep
        });
    }
}

// --- winding -----------------------------------------------------------------

fn directed_flag(tri: [u32; 3], key: (u32, u32)) -> bool {
    for (a, b) in [(tri[0], tri[1]), (tri[1], tri[2]), (tri[2], tri[0])] {
        if edge_key(a, b) == key {
            return a < b;
        }
    }
    false
}

/// Orient every connected shell consistently and outward. Returns the number
/// of triangle windings that changed.
fn orient_components(positions: &[[f64; 3]], triangles: &mut [[u32; 3]]) -> usize {
    let mut adjacency: BTreeMap<usize, Vec<(usize, (u32, u32))>> = BTreeMap::new();
    for (key, entries) in edge_incidence(triangles) {
        if entries.len() != 2 {
            continue;
        }
        let (a, _) = entries[0];
        let (b, _) = entries[1];
        adjacency.entry(a).or_default().push((b, key));
        adjacency.entry(b).or_default().push((a, key));
    }
    for list in adjacency.values_mut() {
        list.sort_unstable();
        list.dedup();
    }

    let mut visited = vec![false; triangles.len()];
    let mut components: Vec<Vec<usize>> = Vec::new();
    for start in 0..triangles.len() {
        if visited[start] {
            continue;
        }
        let mut component = Vec::new();
        let mut queue = vec![start];
        visited[start] = true;
        while let Some(cur) = queue.pop() {
            component.push(cur);
            if let Some(neighbors) = adjacency.get(&cur) {
                for &(n, _) in neighbors {
                    if !visited[n] {
                        visited[n] = true;
                        queue.push(n);
                    }
                }
            }
        }
        component.sort_unstable();
        components.push(component);
    }

    let mut changed = 0usize;
    for component in &components {
        let mut flip = vec![false; triangles.len()];
        let mut seen = vec![false; triangles.len()];
        let mut queue = vec![component[0]];
        seen[component[0]] = true;
        while let Some(cur) = queue.pop() {
            let Some(neighbors) = adjacency.get(&cur) else { continue };
            for &(n, key) in neighbors {
                if seen[n] {
                    continue;
                }
                seen[n] = true;
                let same = directed_flag(triangles[cur], key) == directed_flag(triangles[n], key);
                flip[n] = flip[cur] ^ same;
                queue.push(n);
            }
        }
        for &i in component {
            if flip[i] {
                triangles[i].swap(1, 2);
                changed += 1;
            }
        }

        let volume = component
            .iter()
            .map(|&i| triangle_volume(positions, triangles[i]))
            .sum::<f64>();
        if volume < 0.0 {
            for &i in component {
                triangles[i].swap(1, 2);
                changed += 1;
            }
        }
    }
    changed
}

fn triangle_volume(positions: &[[f64; 3]], t: [u32; 3]) -> f64 {
    let a = positions[t[0] as usize];
    let b = positions[t[1] as usize];
    let c = positions[t[2] as usize];
    (a[0] * (b[1] * c[2] - b[2] * c[1])
        - a[1] * (b[0] * c[2] - b[2] * c[0])
        + a[2] * (b[0] * c[1] - b[1] * c[0]))
        / 6.0
}

// --- hole filling ------------------------------------------------------------

fn fill_boundary_loops(
    positions: &mut Vec<[f64; 3]>,
    triangles: &mut Vec<[u32; 3]>,
    holes_filled: &mut usize,
    triangles_added: &mut usize,
) {
    let mut counts: BTreeMap<(u32, u32), u32> = BTreeMap::new();
    for t in triangles.iter() {
        for (a, b) in [(t[0], t[1]), (t[1], t[2]), (t[2], t[0])] {
            *counts.entry(edge_key(a, b)).or_insert(0) += 1;
        }
    }
    let mut outgoing: BTreeMap<u32, Vec<u32>> = BTreeMap::new();
    let mut directed: BTreeSet<(u32, u32)> = BTreeSet::new();
    for t in triangles.iter() {
        for (a, b) in [(t[0], t[1]), (t[1], t[2]), (t[2], t[0])] {
            if counts.get(&edge_key(a, b)).copied().unwrap_or(0) == 1 {
                outgoing.entry(a).or_default().push(b);
                directed.insert((a, b));
            }
        }
    }
    for list in outgoing.values_mut() {
        list.sort_unstable();
        list.dedup();
    }

    let starts: Vec<(u32, u32)> = directed.iter().copied().collect();
    let mut used: BTreeSet<(u32, u32)> = BTreeSet::new();
    for (start_a, start_b) in starts {
        if used.contains(&(start_a, start_b)) {
            continue;
        }
        let mut loop_edges: Vec<(u32, u32)> = Vec::new();
        let mut a = start_a;
        let mut b = start_b;
        let mut closed = false;
        loop {
            if !used.insert((a, b)) {
                break;
            }
            loop_edges.push((a, b));
            if b == start_a {
                closed = true;
                break;
            }
            let Some(nexts) = outgoing.get(&b) else { break };
            let Some(&next) = nexts.iter().find(|&&n| !used.contains(&(b, n))) else { break };
            a = b;
            b = next;
            if loop_edges.len() > 100_000 {
                break;
            }
        }
        if !closed || loop_edges.len() < 3 {
            continue;
        }
        *holes_filled += 1;
        if loop_edges.len() == 3 {
            let (a0, b0) = loop_edges[0];
            let (_, b1) = loop_edges[1];
            triangles.push([b0, a0, b1]);
            *triangles_added += 1;
            continue;
        }
        let mut center = [0.0f64; 3];
        for &(a, _) in &loop_edges {
            let p = positions[a as usize];
            center[0] += p[0];
            center[1] += p[1];
            center[2] += p[2];
        }
        let n = loop_edges.len() as f64;
        let center = [center[0] / n, center[1] / n, center[2] / n];
        let cid = positions.len() as u32;
        positions.push(center);
        for &(a, b) in &loop_edges {
            triangles.push([b, a, cid]);
            *triangles_added += 1;
        }
    }
}

// --- final stats -------------------------------------------------------------

fn edge_stats(triangles: &[[u32; 3]]) -> (usize, usize) {
    let mut counts: BTreeMap<(u32, u32), u32> = BTreeMap::new();
    for t in triangles {
        for (a, b) in [(t[0], t[1]), (t[1], t[2]), (t[2], t[0])] {
            *counts.entry(edge_key(a, b)).or_insert(0) += 1;
        }
    }
    let open = counts.values().filter(|&&c| c == 1).count();
    let non_manifold = counts.values().filter(|&&c| c > 2).count();
    (open, non_manifold)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geo;
    use wright_protocol::Mesh;

    fn remove_triangle(mesh: &Mesh, index: usize) -> Mesh {
        let mut out = mesh.clone();
        out.indices.drain(index * 3..index * 3 + 3);
        out
    }

    #[test]
    fn a_missing_triangle_is_filled_and_stays_closed() {
        let mesh = geo::box_mesh([10.0, 8.0, 6.0]);
        let broken = remove_triangle(&mesh, 0);
        let (fixed, report) = repair(&broken, Options::default()).unwrap();
        assert!(report.watertight, "{report:?}");
        assert_eq!(report.open_edges, 0);
        assert_eq!(report.non_manifold_edges, 0);
        assert!(report.holes_filled >= 1, "{report:?}");
        assert!(!fixed.indices.is_empty());
        assert!(crate::fillet::signed_volume(&fixed) > 0.0);
    }

    #[test]
    fn an_inverted_shell_is_turned_outward() {
        let mut mesh = geo::box_mesh([10.0, 8.0, 6.0]);
        for chunk in mesh.indices.chunks_exact_mut(3) {
            chunk.swap(1, 2);
        }
        assert!(crate::fillet::signed_volume(&mesh) < 0.0);
        let (fixed, report) = repair(&mesh, Options::default()).unwrap();
        assert!(crate::fillet::signed_volume(&fixed) > 0.0);
        assert!(report.flipped > 0, "{report:?}");
        assert!(report.watertight);
    }

    #[test]
    fn a_non_manifold_fin_is_removed() {
        let mut mesh = geo::box_mesh([10.0, 8.0, 6.0]);
        // Share the face diagonal, which already has two incident triangles:
        // the fin makes it non-manifold, so the repair must drop it.
        let a = mesh.indices[0];
        let b = mesh.indices[2];
        let base = mesh.positions.len() as u32;
        mesh.positions.push([20.0, 20.0, 20.0]);
        mesh.indices.extend_from_slice(&[a, b, base]);
        let (fixed, report) = repair(&mesh, Options::default()).unwrap();
        assert_eq!(report.non_manifold_edges, 0, "{report:?}");
        assert!(report.watertight, "{report:?}");
        assert!(report.triangles_removed >= 1, "{report:?}");
        assert!(crate::fillet::signed_volume(&fixed) > 0.0);
    }
}
