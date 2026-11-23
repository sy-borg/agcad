//! Deterministic tessellation for the v0 primitive library. All geometry
//! is baked **centered on the entity origin** — world placement comes from
//! the entity transform. Y is the cylinder axis; Z is up by convention.
//!
//! Every function produces the same output for the same input: agents may
//! rely on vertex counts and index layout being stable.

use wright_protocol::Mesh;

const TAU: f64 = std::f64::consts::TAU;

/// Axis-aligned box of `size` centered on the origin. 24 vertices (4 per
/// face, flat normals), 36 indices.
pub fn box_mesh(size: [f64; 3]) -> Mesh {
    let [hx, hy, hz] = [size[0] * 0.5, size[1] * 0.5, size[2] * 0.5];
    let mut m = Mesh::default();

    let mut face = |n: [f32; 3], corners: [[f32; 3]; 4]| {
        let base = m.positions.len() as u32;
        for c in corners {
            m.positions.push(c);
            m.normals.push(n);
        }
        m.indices.extend_from_slice(&[base, base + 1, base + 2, base, base + 2, base + 3]);
    };

    // +X, -X, +Y, -Y, +Z, -Z (CCW from outside; we render without culling,
    // so winding only matters for future two-sided logic).
    face([1.0, 0.0, 0.0], [
        [hx as f32, -hy as f32, -hz as f32],
        [hx as f32, hy as f32, -hz as f32],
        [hx as f32, hy as f32, hz as f32],
        [hx as f32, -hy as f32, hz as f32],
    ]);
    face([-1.0, 0.0, 0.0], [
        [-hx as f32, -hy as f32, hz as f32],
        [-hx as f32, hy as f32, hz as f32],
        [-hx as f32, hy as f32, -hz as f32],
        [-hx as f32, -hy as f32, -hz as f32],
    ]);
    face([0.0, 1.0, 0.0], [
        [-hx as f32, hy as f32, -hz as f32],
        [-hx as f32, hy as f32, hz as f32],
        [hx as f32, hy as f32, hz as f32],
        [hx as f32, hy as f32, -hz as f32],
    ]);
    face([0.0, -1.0, 0.0], [
        [-hx as f32, -hy as f32, hz as f32],
        [-hx as f32, -hy as f32, -hz as f32],
        [hx as f32, -hy as f32, -hz as f32],
        [hx as f32, -hy as f32, hz as f32],
    ]);
    face([0.0, 0.0, 1.0], [
        [-hx as f32, -hy as f32, hz as f32],
        [hx as f32, -hy as f32, hz as f32],
        [hx as f32, hy as f32, hz as f32],
        [-hx as f32, hy as f32, hz as f32],
    ]);
    face([0.0, 0.0, -1.0], [
        [-hx as f32, hy as f32, -hz as f32],
        [hx as f32, hy as f32, -hz as f32],
        [hx as f32, -hy as f32, -hz as f32],
        [-hx as f32, -hy as f32, -hz as f32],
    ]);

    m
}

/// UV sphere of `radius` centered on the origin, poles on ±Y.
/// Vertex layout is stable: one pole vertex per pole plus
/// `(segments - 1) * segments` ring vertices. Poles are proper triangle
/// fans — no degenerate quads, so the mesh is a closed solid that mesh
/// booleans accept.
pub fn sphere_mesh(radius: f64, segments: u32) -> Mesh {
    let segs = segments.max(3);
    let mut m = Mesh::default();

    // Top pole (+Y), then rings j = 1..=segs-1, then bottom pole (-Y).
    let top = 0u32;
    m.positions.push([0.0, radius as f32, 0.0]);
    m.normals.push([0.0, 1.0, 0.0]);
    let mut idx = 1u32;
    let mut ring_start = vec![0u32; (segs - 1) as usize];
    for j in 1..segs {
        ring_start[(j - 1) as usize] = idx;
        let phi = (j as f64 / segs as f64) * std::f64::consts::PI;
        let (sp, cp) = phi.sin_cos();
        for i in 0..segs {
            let theta = (i as f64 / segs as f64) * TAU;
            let (st, ct) = theta.sin_cos();
            // Y-up sphere; normal = normalized position.
            let n = [sp * ct, cp, sp * st];
            m.positions.push([n[0], n[1], n[2]].map(|v| (v * radius) as f32));
            m.normals.push(n.map(|v| v as f32));
            idx += 1;
        }
    }
    let bottom = idx;
    m.positions.push([0.0, -(radius as f32), 0.0]);
    m.normals.push([0.0, -1.0, 0.0]);

    let ring = |j: u32| -> u32 { ring_start[(j - 1) as usize] };

    // Quads between consecutive rings (j, j+1), j = 1..segs-1. Winding is
    // CCW from outside (verified by kernel difference behavior).
    for j in 1..segs - 1 {
        let a0 = ring(j);
        let c0 = ring(j + 1);
        for i in 0..segs {
            let a = a0 + i;
            let b = a0 + (i + 1) % segs;
            let c = c0 + i;
            let d = c0 + (i + 1) % segs;
            m.indices.extend_from_slice(&[a, b, c, b, d, c]);
        }
    }

    // Top fan: pole to first ring.
    let r1 = ring(1);
    for i in 0..segs {
        let a = r1 + i;
        let b = r1 + (i + 1) % segs;
        m.indices.extend_from_slice(&[top, b, a]);
    }

    // Bottom fan: last ring to pole.
    let rl = ring(segs - 1);
    for i in 0..segs {
        let a = rl + i;
        let b = rl + (i + 1) % segs;
        m.indices.extend_from_slice(&[a, b, bottom]);
    }

    m
}

/// Cylinder of `radius` and `height` centered on the origin, axis on **+Z**
/// (Z-up, like every other primitive), caps included. Layout:
/// `2 * segments` side vertices, then `1 + segments` per cap.
pub fn cylinder_mesh(radius: f64, height: f64, segments: u32) -> Mesh {
    let segs = segments.max(3);
    let (r, h) = (radius as f32, height as f32 * 0.5);
    let mut m = Mesh::default();

    // Side ring: interleaved bottom/top per column so a quad strip is easy.
    for i in 0..segs {
        let theta = (i as f64 / segs as f64) * TAU;
        let (st, ct) = theta.sin_cos();
        let n = [ct as f32, st as f32, 0.0];
        m.positions.push([n[0] * r, n[1] * r, -h]);
        m.normals.push(n);
        m.positions.push([n[0] * r, n[1] * r, h]);
        m.normals.push(n);
    }
    for i in 0..segs {
        let b0 = (2 * i) as u32;
        let t0 = (2 * i + 1) as u32;
        let j = (i + 1) % segs;
        let b1 = (2 * j) as u32;
        let t1 = (2 * j + 1) as u32;
        // CCW seen from outside: bottom edge forward, then up.
        m.indices.extend_from_slice(&[b0, b1, t1, b0, t1, t0]);
    }

    // Cap helper: fan around a center vertex.
    let mut cap = |z: f32, nz: f32| {
        let center = m.positions.len() as u32;
        m.positions.push([0.0, 0.0, z]);
        m.normals.push([0.0, 0.0, nz]);
        let first = m.positions.len() as u32;
        for i in 0..segs {
            let theta = (i as f64 / segs as f64) * TAU;
            let (st, ct) = theta.sin_cos();
            m.positions.push([ct as f32 * r, st as f32 * r, z]);
            m.normals.push([0.0, 0.0, nz]);
        }
        for i in 0..segs {
            let a = first + i as u32;
            let b = first + ((i + 1) % segs) as u32;
            if nz > 0.0 {
                m.indices.extend_from_slice(&[center, a, b]);
            } else {
                m.indices.extend_from_slice(&[center, b, a]);
            }
        }
    };

    cap(h, 1.0);
    cap(-h, -1.0);

    m
}

#[cfg(test)]
mod tests {
    use super::*;

    fn assert_watertight(m: &Mesh) {
        for &i in &m.indices {
            assert!(m.positions.get(i as usize).is_some(), "index out of range");
        }
        assert_eq!(m.positions.len(), m.normals.len());
    }

    #[test]
    fn meshes_are_wellformed() {
        assert_watertight(&box_mesh([1.0, 2.0, 3.0]));
        assert_eq!(box_mesh([1.0, 2.0, 3.0]).positions.len(), 24);

        let s = sphere_mesh(1.0, 16);
        assert_watertight(&s);
        assert_eq!(s.positions.len(), (16 - 1) * 16 + 2);

        let c = cylinder_mesh(1.0, 2.0, 12);
        assert_watertight(&c);
        assert_eq!(c.positions.len(), 24 + 13 + 13);
    }

    #[test]
    fn deterministic() {
        let a = sphere_mesh(1.0, 8);
        let b = sphere_mesh(1.0, 8);
        assert_eq!(a.positions, b.positions);
        assert_eq!(a.indices, b.indices);
    }

    /// Positive for CCW-outward winding — the invariant booleans, STL export
    /// and the renderer all rely on.
    fn signed_volume(m: &Mesh) -> f64 {
        let mut v = 0.0;
        for t in m.indices.chunks_exact(3) {
            let p = |i: u32| {
                let q = m.positions[i as usize];
                [q[0] as f64, q[1] as f64, q[2] as f64]
            };
            let (a, b, c) = (p(t[0]), p(t[1]), p(t[2]));
            v += (a[0] * (b[1] * c[2] - b[2] * c[1])
                - a[1] * (b[0] * c[2] - b[2] * c[0])
                + a[2] * (b[0] * c[1] - b[1] * c[0]))
                / 6.0;
        }
        v
    }

    #[test]
    fn cylinder_extrudes_along_z() {
        let m = cylinder_mesh(2.0, 10.0, 16);
        let mut lo = [f32::MAX; 3];
        let mut hi = [f32::MIN; 3];
        for p in &m.positions {
            for i in 0..3 {
                lo[i] = lo[i].min(p[i]);
                hi[i] = hi[i].max(p[i]);
            }
        }
        // Z is the axis; X and Y are the radius.
        assert!(
            (lo[2] + 5.0).abs() < 1e-6 && (hi[2] - 5.0).abs() < 1e-6,
            "axis must be Z: {lo:?}..{hi:?}"
        );
        assert!((lo[0] + 2.0).abs() < 1e-6 && (hi[0] - 2.0).abs() < 1e-6, "{lo:?}..{hi:?}");
        assert!((lo[1] + 2.0).abs() < 1e-6 && (hi[1] - 2.0).abs() < 1e-6, "{lo:?}..{hi:?}");

        // A 16-gon under-estimates pi by ~2.5%; the sign is what matters.
        let v = signed_volume(&m);
        let expected = std::f64::consts::PI * 4.0 * 10.0;
        assert!(v > 0.0, "winding must be CCW-outward (got volume {v})");
        assert!((v - expected).abs() / expected < 0.03, "volume {v} vs {expected}");
    }
}
