//! Deterministic tessellation for the v0 primitive library. All geometry
//! is baked **centered on the entity origin** — world placement comes from
//! the entity transform. Y is the cylinder axis; Z is up by convention.
//!
//! Every function produces the same output for the same input: agents may
//! rely on vertex counts and index layout being stable.

use agcad_protocol::Mesh;

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
/// Vertex layout is stable: `(segments + 1) * segments` vertices.
pub fn sphere_mesh(radius: f64, segments: u32) -> Mesh {
    let segs = segments.max(3);
    let mut m = Mesh::default();

    for j in 0..=segs {
        let phi = (j as f64 / segs as f64) * std::f64::consts::PI;
        let (sp, cp) = phi.sin_cos();
        for i in 0..segs {
            let theta = (i as f64 / segs as f64) * TAU;
            let (st, ct) = theta.sin_cos();
            // Y-up sphere.
            let n = [sp * ct, cp, sp * st];
            m.positions.push([n[0], n[1], n[2]].map(|v| (v * radius) as f32));
            m.normals.push(n.map(|v| v as f32));
        }
    }

    let cols = segs;
    for j in 0..segs {
        for i in 0..segs {
            let a = j * cols + i;
            let b = j * cols + (i + 1) % cols;
            let c = (j + 1) * cols + i;
            let d = (j + 1) * cols + (i + 1) % cols;
            let (a, b, c, d) = (a as u32, b as u32, c as u32, d as u32);
            m.indices.extend_from_slice(&[a, c, b, b, c, d]);
        }
    }

    m
}

/// Cylinder of `radius` and `height` centered on the origin, axis on Y,
/// caps included. Layout: `2 * segments` side vertices, then
/// `1 + segments` per cap.
pub fn cylinder_mesh(radius: f64, height: f64, segments: u32) -> Mesh {
    let segs = segments.max(3);
    let (r, h) = (radius as f32, height as f32 * 0.5);
    let mut m = Mesh::default();

    // Side ring: interleaved bottom/top per column so a quad strip is easy.
    for i in 0..segs {
        let theta = (i as f64 / segs as f64) * TAU;
        let (st, ct) = theta.sin_cos();
        let n = [ct as f32, 0.0, st as f32];
        m.positions.push([n[0] * r, -h, n[2] * r]);
        m.normals.push(n);
        m.positions.push([n[0] * r, h, n[2] * r]);
        m.normals.push(n);
    }
    for i in 0..segs {
        let b0 = (2 * i) as u32;
        let t0 = (2 * i + 1) as u32;
        let j = (i + 1) % segs;
        let b1 = (2 * j) as u32;
        let t1 = (2 * j + 1) as u32;
        m.indices.extend_from_slice(&[b0, b1, t0, t0, b1, t1]);
    }

    // Cap helper: fan around a center vertex.
    let mut cap = |y: f32, ny: f32| {
        let center = m.positions.len() as u32;
        m.positions.push([0.0, y, 0.0]);
        m.normals.push([0.0, ny, 0.0]);
        let first = m.positions.len() as u32;
        for i in 0..segs {
            let theta = (i as f64 / segs as f64) * TAU;
            let (st, ct) = theta.sin_cos();
            m.positions.push([ct as f32 * r, y, st as f32 * r]);
            m.normals.push([0.0, ny, 0.0]);
        }
        for i in 0..segs {
            let a = first + i as u32;
            let b = first + ((i + 1) % segs) as u32;
            if ny > 0.0 {
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
        assert_eq!(s.positions.len(), 17 * 16);

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
}
