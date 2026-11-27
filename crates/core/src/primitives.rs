//! Analytic primitive discovery for solids and imported meshes (M22).
//!
//! A triangle mesh has no cylinders, only a lot of small facets. This module
//! recovers the shapes an agent needs to *re-author* an imported part:
//!
//! * planar faces (delegated to the M20 face enumerator when the mesh is
//!   closed, with a tolerant fallback for open imports);
//! * cylindrical surfaces, by clustering triangles across smooth edges and
//!   fitting an axis/radius to the cluster's vertices;
//! * hole candidates, i.e. inner cylinders, with a through/blind guess from
//!   the axial extent.
//!
//! Fitting is deterministic: welded vertices come from a `BTreeMap`, clusters
//! are visited in triangle order, and the 3×3 eigen-decomposition uses a
//! fixed Jacobi sweep. A parameter change that resizes a cylinder keeps its
//! identity fingerprint; the radius is reported and may move.

use std::collections::{BTreeMap, BTreeSet};

use wright_protocol::{Bbox, CylinderInfo, FaceGroupInfo, FaceSelector, HoleCandidate, Mesh, Vec3};

use crate::edges::{self, cross, dot, normalize, sub};
use crate::plane;
use crate::refs;
use crate::Error;

/// Facets turning less than this are treated as one smooth surface.
const SMOOTH_DEG: f64 = 30.0;
/// A fit whose radial error exceeds this fraction of the radius is rejected.
const FIT_TOLERANCE: f64 = 0.03;
/// Cylinders need at least this many triangles; a couple of facets is noise.
const MIN_TRIS: usize = 4;

/// One `primitives` answer.
#[derive(Debug, Clone)]
pub struct Report {
    pub planar_faces: Vec<FaceGroupInfo>,
    pub cylinders: Vec<CylinderInfo>,
    pub holes: Vec<HoleCandidate>,
}

/// Find the analytic primitives of a **world-space** mesh.
pub fn primitives(world: &Mesh, entity: u64) -> Result<Report, Error> {
    let planar_faces = match refs::topology(world, entity, SMOOTH_DEG) {
        Ok(topology) => refs::face_groups(&topology),
        Err(_) => simple_faces(world, entity)?,
    };
    let cylinders = cylinders(world)?;
    let holes = holes(&cylinders, world);
    Ok(Report { planar_faces, cylinders, holes })
}

// --- planar fallback ---------------------------------------------------------

fn simple_faces(world: &Mesh, entity: u64) -> Result<Vec<FaceGroupInfo>, Error> {
    let patches = plane::planar_faces(world)?;
    let mut ranks: BTreeMap<String, u32> = BTreeMap::new();
    let mut out = Vec::with_capacity(patches.len());
    for (index, patch) in patches.iter().enumerate() {
        let (nx, ny, nz) = vec_key(patch.normal, 1e4);
        let base = format!("face:{nx}:{ny}:{nz}");
        let ordinal = ranks.get(&base).copied().unwrap_or(0);
        ranks.insert(base.clone(), ordinal + 1);
        let fingerprint = hash_str(&format!("wright/face/1|{base}#{ordinal}"));
        let mut bbox = Bbox { min: [f64::MAX; 3], max: [f64::MIN; 3] };
        let mut verts: BTreeSet<[u32; 3]> = BTreeSet::new();
        for &t in &patch.triangles {
            for k in 0..3 {
                let Some(&idx) = world.indices.get(t * 3 + k) else { continue };
                let Some(p) = world.positions.get(idx as usize) else { continue };
                include(&mut bbox, [p[0] as f64, p[1] as f64, p[2] as f64]);
                verts.insert([p[0].to_bits(), p[1].to_bits(), p[2].to_bits()]);
            }
        }
        let id = (index + 1) as u32;
        out.push(FaceGroupInfo {
            id,
            kind: "planar".into(),
            normal: patch.normal,
            center: patch.centroid,
            area: patch.area,
            bbox,
            tris: patch.triangles.len(),
            edges: Vec::new(),
            descriptor: format!(
                "planar face #{id}: normal {}, area {:.2} mm², center {}",
                axis_name(patch.normal),
                patch.area,
                point(patch.centroid)
            ),
            fingerprint,
            geometry: hash_str(&format!(
                "wright/face-geom/1|{}",
                verts
                    .iter()
                    .flat_map(|v| v.iter().map(|x| x.to_string()))
                    .collect::<Vec<_>>()
                    .join(",")
            )),
            selector: FaceSelector { entity, normal: patch.normal, near: patch.centroid },
        });
    }
    Ok(out)
}

// --- cylinder fitting --------------------------------------------------------

struct Cluster {
    tris: Vec<usize>,
    area: f64,
    centroid: Vec3,
}

fn cylinders(world: &Mesh) -> Result<Vec<CylinderInfo>, Error> {
    let (positions, tris) = edges::weld(world)?;
    if tris.len() < MIN_TRIS * 3 {
        return Ok(Vec::new());
    }
    let mut triangles: Vec<[u32; 3]> = tris
        .chunks_exact(3)
        .map(|t| [t[0], t[1], t[2]])
        .collect();
    triangles.retain(|t| t[0] != t[1] && t[1] != t[2] && t[0] != t[2]);
    if triangles.len() < MIN_TRIS {
        return Ok(Vec::new());
    }

    let mut faces: Vec<(Vec3, f64, Vec3)> = Vec::with_capacity(triangles.len());
    for t in &triangles {
        let a = positions[t[0] as usize];
        let b = positions[t[1] as usize];
        let c = positions[t[2] as usize];
        let n = normalize(cross(sub(b, a), sub(c, a)));
        let area = 0.5 * dot(cross(sub(b, a), sub(c, a)), n).abs();
        faces.push((n, area, [
            (a[0] + b[0] + c[0]) / 3.0,
            (a[1] + b[1] + c[1]) / 3.0,
            (a[2] + b[2] + c[2]) / 3.0,
        ]));
    }

    let min_cos = SMOOTH_DEG.to_radians().cos();
    let mut edge_faces: BTreeMap<(u32, u32), Vec<usize>> = BTreeMap::new();
    for (index, t) in triangles.iter().enumerate() {
        for (a, b) in [(t[0], t[1]), (t[1], t[2]), (t[2], t[0])] {
            let key = if a < b { (a, b) } else { (b, a) };
            edge_faces.entry(key).or_default().push(index);
        }
    }
    let mut adjacency: BTreeMap<usize, Vec<usize>> = BTreeMap::new();
    for list in edge_faces.values() {
        if list.len() != 2 {
            continue;
        }
        let (a, b) = (list[0], list[1]);
        if dot(faces[a].0, faces[b].0) >= min_cos {
            adjacency.entry(a).or_default().push(b);
            adjacency.entry(b).or_default().push(a);
        }
    }
    for list in adjacency.values_mut() {
        list.sort_unstable();
        list.dedup();
    }

    let mut visited = vec![false; triangles.len()];
    let mut clusters: Vec<Cluster> = Vec::new();
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
                for &n in neighbors {
                    if !visited[n] {
                        visited[n] = true;
                        queue.push(n);
                    }
                }
            }
        }
        component.sort_unstable();
        if component.len() < MIN_TRIS {
            continue;
        }
        let mut normal = [0.0; 3];
        let mut area = 0.0;
        let mut centroid = [0.0; 3];
        for &i in &component {
            let (n, a, c) = faces[i];
            normal[0] += n[0] * a;
            normal[1] += n[1] * a;
            normal[2] += n[2] * a;
            area += a;
            centroid[0] += c[0] * a;
            centroid[1] += c[1] * a;
            centroid[2] += c[2] * a;
        }
        if area <= 0.0 {
            continue;
        }
        let normal = normalize(normal);
        let _ = normal;
        clusters.push(Cluster {
            tris: component,
            area,
            centroid: [centroid[0] / area, centroid[1] / area, centroid[2] / area],
        });
    }

    let mut out = Vec::new();
    let mut ranks: BTreeMap<String, u32> = BTreeMap::new();
    for cluster in &clusters {
        let Some((axis, center, radius, height, inner, bbox)) =
            fit_cylinder(&positions, &triangles, &faces, cluster)
        else {
            continue;
        };
        let (ax, ay, az) = vec_key(axis, 1e4);
        let base = format!("cylinder:{ax}:{ay}:{az}:{}", if inner { "inner" } else { "outer" });
        let ordinal = ranks.get(&base).copied().unwrap_or(0);
        ranks.insert(base.clone(), ordinal + 1);
        let fingerprint = hash_str(&format!("wright/cylinder/1|{base}#{ordinal}"));
        let id = (out.len() + 1) as u32;
        out.push(CylinderInfo {
            id,
            axis,
            center,
            radius,
            height,
            inner,
            area: cluster.area,
            bbox,
            tris: cluster.tris.len(),
            descriptor: format!(
                "cylinder #{id}: axis {}, radius {:.3} mm, height {:.3} mm, {}",
                axis_name(axis),
                radius,
                height,
                if inner { "inner (hole)" } else { "outer (boss/shaft)" }
            ),
            fingerprint,
        });
    }
    Ok(out)
}

#[allow(clippy::type_complexity)]
fn fit_cylinder(
    positions: &[[f64; 3]],
    triangles: &[[u32; 3]],
    faces: &[(Vec3, f64, Vec3)],
    cluster: &Cluster,
) -> Option<(Vec3, Vec3, f64, f64, bool, Bbox)> {
    let mut covariance = [[0.0f64; 3]; 3];
    for &i in &cluster.tris {
        let (n, a, _) = faces[i];
        for r in 0..3 {
            for c in 0..3 {
                covariance[r][c] += a * n[r] * n[c];
            }
        }
    }
    let (values, vectors) = eig_sym(covariance);
    let max = values[2];
    if max <= 1e-12 || values[0] > 0.05 * max || values[1] < 0.05 * max {
        return None;
    }
    let mut axis = vectors[0];
    canonicalize_axis(&mut axis);
    if dot(axis, axis) < 0.5 {
        return None;
    }

    // A deterministic orthonormal basis perpendicular to the axis.
    let helper = if axis[0].abs() < 0.9 { [1.0, 0.0, 0.0] } else { [0.0, 1.0, 0.0] };
    let u = normalize(cross(helper, axis));
    let v = cross(axis, u);
    if dot(u, u) < 0.5 || dot(v, v) < 0.5 {
        return None;
    }

    let mut vertices: BTreeSet<u32> = BTreeSet::new();
    for &i in &cluster.tris {
        let t = triangles[i];
        vertices.insert(t[0]);
        vertices.insert(t[1]);
        vertices.insert(t[2]);
    }
    let mut sx = 0.0;
    let mut sy = 0.0;
    let mut sxx = 0.0;
    let mut syy = 0.0;
    let mut sxy = 0.0;
    let mut sz = 0.0;
    let mut sxz = 0.0;
    let mut syz = 0.0;
    let mut count = 0.0;
    let mut amin = f64::MAX;
    let mut amax = f64::MIN;
    let mut bbox = Bbox { min: [f64::MAX; 3], max: [f64::MIN; 3] };
    for &id in &vertices {
        let p = positions[id as usize];
        include(&mut bbox, p);
        let d = sub(p, cluster.centroid);
        let x = dot(d, u);
        let y = dot(d, v);
        let z = x * x + y * y;
        sx += x;
        sy += y;
        sxx += x * x;
        syy += y * y;
        sxy += x * y;
        sz += z;
        sxz += x * z;
        syz += y * z;
        amin = amin.min(dot(p, axis));
        amax = amax.max(dot(p, axis));
        count += 1.0;
    }
    if count < 3.0 {
        return None;
    }
    // Kasa circle fit: x²+y² + D x + E y + F = 0, via its normal equations.
    let mut m = [[sxx, sxy, sx], [sxy, syy, sy], [sx, sy, count]];
    let mut r = [-sxz, -syz, -sz];
    let solved = solve3(&mut m, &mut r)?;
    let (d, e, f) = (solved[0], solved[1], solved[2]);
    let cx = -d / 2.0;
    let cy = -e / 2.0;
    let radius_sq = cx * cx + cy * cy - f;
    if !radius_sq.is_finite() || radius_sq <= 0.0 {
        return None;
    }
    let radius = radius_sq.sqrt();
    let mut max_err = 0.0f64;
    for &id in &vertices {
        let p = positions[id as usize];
        let d = sub(p, cluster.centroid);
        let x = dot(d, u) - cx;
        let y = dot(d, v) - cy;
        max_err = max_err.max(((x * x + y * y).sqrt() - radius).abs());
    }
    if max_err > FIT_TOLERANCE * radius {
        return None;
    }

    // Inner vs outer: the surface normal points toward the axis for a hole.
    let mut radial_dot = 0.0;
    for &i in &cluster.tris {
        let (n, a, c) = faces[i];
        let d = sub(c, cluster.centroid);
        let rx = dot(d, u) - cx;
        let ry = dot(d, v) - cy;
        let rl = (rx * rx + ry * ry).sqrt();
        if rl < 1e-12 {
            continue;
        }
        let nu = dot(n, u);
        let nv = dot(n, v);
        radial_dot += a * (nu * rx + nv * ry) / rl;
    }
    let inner = radial_dot < 0.0;
    let mid = (amin + amax) / 2.0;
    let center = [
        u[0] * cx + v[0] * cy + axis[0] * mid,
        u[1] * cx + v[1] * cy + axis[1] * mid,
        u[2] * cx + v[2] * cy + axis[2] * mid,
    ];
    let height = (amax - amin).max(0.0);
    Some((axis, center, radius, height, inner, bbox))
}

fn canonicalize_axis(axis: &mut Vec3) {
    // Pick the sign with the largest-magnitude component positive.
    let mut best = 0usize;
    for i in 1..3 {
        if axis[i].abs() > axis[best].abs() {
            best = i;
        }
    }
    if axis[best] < 0.0 {
        for v in axis.iter_mut() {
            *v = -*v;
        }
    }
    for v in axis.iter_mut() {
        if v.abs() < 1e-12 {
            *v = 0.0;
        }
    }
    *axis = normalize(*axis);
}

fn holes(cylinders: &[CylinderInfo], world: &Mesh) -> Vec<HoleCandidate> {
    let mut out = Vec::new();
    for cylinder in cylinders.iter().filter(|c| c.inner) {
        // Axial extent of the whole solid: a cylinder spanning it is likely a
        // through-hole rather than a blind pocket.
        let mut lo = f64::MAX;
        let mut hi = f64::MIN;
        for p in &world.positions {
            let d = p[0] as f64 * cylinder.axis[0]
                + p[1] as f64 * cylinder.axis[1]
                + p[2] as f64 * cylinder.axis[2];
            lo = lo.min(d);
            hi = hi.max(d);
        }
        let extent = (hi - lo).max(0.0);
        let through = extent > 0.0 && cylinder.height >= 0.95 * extent;
        out.push(HoleCandidate {
            id: (out.len() + 1) as u32,
            cylinder: cylinder.id,
            center: cylinder.center,
            axis: cylinder.axis,
            radius: cylinder.radius,
            depth: cylinder.height,
            through,
            descriptor: format!(
                "hole #{}: ⌀{:.3} mm, depth {:.3} mm{}, axis {}",
                out.len() + 1,
                cylinder.radius * 2.0,
                cylinder.height,
                if through { " (through)" } else { " (blind)" },
                axis_name(cylinder.axis)
            ),
        });
    }
    out
}

// --- linear algebra ----------------------------------------------------------

fn solve3(a: &mut [[f64; 3]; 3], b: &mut [f64; 3]) -> Option<[f64; 3]> {
    for col in 0..3 {
        let (pivot, _) = (col..3).fold((col, a[col][col].abs()), |best, row| {
            let v = a[row][col].abs();
            if v > best.1 { (row, v) } else { best }
        });
        if a[pivot][col].abs() < 1e-14 {
            return None;
        }
        a.swap(pivot, col);
        b.swap(pivot, col);
        let inv = 1.0 / a[col][col];
        for row in (col + 1)..3 {
            let f = a[row][col] * inv;
            for k in col..3 {
                a[row][k] -= f * a[col][k];
            }
            b[row] -= f * b[col];
        }
    }
    let mut out = [0.0; 3];
    for row in (0..3).rev() {
        let s: f64 = (row + 1..3).map(|k| a[row][k] * out[k]).sum();
        out[row] = (b[row] - s) / a[row][row];
    }
    Some(out)
}

/// Jacobi eigen-decomposition of a symmetric 3×3 matrix. Returns
/// `(eigenvalues ascending, eigenvectors)` with the vector for each value.
fn eig_sym(mut a: [[f64; 3]; 3]) -> ([f64; 3], [Vec3; 3]) {
    let mut v = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    for _ in 0..32 {
        let (p, q) = {
            let mut best = (0usize, 1usize, a[0][1].abs());
            for (r, c) in [(0usize, 2usize), (1, 2)] {
                if a[r][c].abs() > best.2 {
                    best = (r, c, a[r][c].abs());
                }
            }
            (best.0, best.1)
        };
        if a[p][q].abs() < 1e-15 {
            break;
        }
        let theta = (a[q][q] - a[p][p]) / (2.0 * a[p][q]);
        let t = theta.signum() / (theta.abs() + (theta * theta + 1.0).sqrt());
        let c = 1.0 / (t * t + 1.0).sqrt();
        let s = t * c;
        let app = a[p][p];
        let aqq = a[q][q];
        let apq = a[p][q];
        a[p][p] = app - t * apq;
        a[q][q] = aqq + t * apq;
        a[p][q] = 0.0;
        a[q][p] = 0.0;
        for r in 0..3 {
            if r != p && r != q {
                let arp = a[r][p];
                let arq = a[r][q];
                a[r][p] = c * arp - s * arq;
                a[p][r] = a[r][p];
                a[r][q] = s * arp + c * arq;
                a[q][r] = a[r][q];
            }
        }
        for r in 0..3 {
            let vrp = v[r][p];
            let vrq = v[r][q];
            v[r][p] = c * vrp - s * vrq;
            v[r][q] = s * vrp + c * vrq;
        }
    }
    let mut values = [a[0][0], a[1][1], a[2][2]];
    let mut vectors = [
        [v[0][0], v[1][0], v[2][0]],
        [v[0][1], v[1][1], v[2][1]],
        [v[0][2], v[1][2], v[2][2]],
    ];
    // Sort ascending by value.
    for i in 0..2 {
        for j in 0..2 - i {
            if values[j] > values[j + 1] {
                values.swap(j, j + 1);
                vectors.swap(j, j + 1);
            }
        }
    }
    (values, vectors)
}

// --- formatting helpers ------------------------------------------------------

fn include(bbox: &mut Bbox, p: Vec3) {
    for i in 0..3 {
        bbox.min[i] = bbox.min[i].min(p[i]);
        bbox.max[i] = bbox.max[i].max(p[i]);
    }
}

fn vec_key(v: Vec3, scale: f64) -> (i64, i64, i64) {
    (
        (v[0] * scale).round() as i64,
        (v[1] * scale).round() as i64,
        (v[2] * scale).round() as i64,
    )
}

fn axis_name(n: Vec3) -> String {
    let axes = ["+X", "-X", "+Y", "-Y", "+Z", "-Z"];
    let best = (0..3)
        .flat_map(|i| [(i, 1.0), (i, -1.0)])
        .max_by(|a, b| {
            (n[a.0] * a.1).abs().partial_cmp(&(n[b.0] * b.1).abs()).unwrap()
        });
    if let Some((axis, sign)) = best {
        if n[axis] * sign > 0.999 {
            return axes[axis * 2 + if sign > 0.0 { 0 } else { 1 }].to_string();
        }
    }
    format!("({:.3}, {:.3}, {:.3})", n[0], n[1], n[2])
}

fn point(p: Vec3) -> String {
    format!("({:.2}, {:.2}, {:.2})", p[0], p[1], p[2])
}

fn hash_str(text: &str) -> String {
    let mut hash = 0xcbf29ce484222325u64;
    for b in text.as_bytes() {
        hash ^= *b as u64;
        hash = hash.wrapping_mul(0x100000001b3);
    }
    format!("{hash:016x}")
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geo;

    #[test]
    fn a_cylinder_boss_has_an_outer_cylinder() {
        let mesh = geo::cylinder_mesh(5.0, 20.0, 32);
        let report = primitives(&mesh, 1).unwrap();
        assert_eq!(report.cylinders.len(), 1, "{report:?}");
        let c = &report.cylinders[0];
        assert!(!c.inner);
        assert!((c.radius - 5.0).abs() < 1e-3, "{c:?}");
        assert!((c.height - 20.0).abs() < 1e-3, "{c:?}");
        assert!(c.axis[2].abs() > 0.99, "{c:?}");
        assert!(report.holes.is_empty());
    }

    #[test]
    fn a_hole_in_a_block_is_an_inner_cylinder() {
        let mut block = geo::box_mesh([40.0, 40.0, 10.0]);
        let hole = geo::cylinder_mesh(5.0, 10.0, 32);
        // Shrink the block so the hole is visible; we only fit surfaces, so
        // simply unioning the meshes is enough for the cylinder detector.
        let base = block.positions.len() as u32;
        for p in &hole.positions {
            block.positions.push([p[0] + 10.0, p[1], p[2]]);
        }
        // Reverse the appended cylinder's winding so its normals point toward
        // the axis, exactly like the wall of a drilled hole.
        for tri in hole.indices.chunks_exact(3) {
            block.indices.extend_from_slice(&[base + tri[0], base + tri[2], base + tri[1]]);
        }
        let report = primitives(&block, 1).unwrap();
        assert!(
            report
                .cylinders
                .iter()
                .any(|c| c.inner && c.radius > 4.9 && c.radius < 5.1),
            "{report:?}"
        );
    }

    #[test]
    fn inverted_cylinder_normals_are_inner_and_holes() {
        let mut mesh = geo::cylinder_mesh(3.0, 8.0, 24);
        for chunk in mesh.indices.chunks_exact_mut(3) {
            chunk.swap(1, 2);
        }
        let report = primitives(&mesh, 1).unwrap();
        assert!(report.cylinders.iter().any(|c| c.inner), "{report:?}");
        assert!(!report.holes.is_empty(), "{report:?}");
    }
}
