//! Binary STL export: world-space triangles of the selected entities into
//! one file, ready for any slicer.
//!
//! Format: 80-byte header (must *not* start with "solid" — binary STL is
//! detected that way), `u32` triangle count, then per triangle a face
//! normal, three vertices, and a `u16` attribute — all little-endian `f32`.

use wright_protocol::{Mat4, Mesh};

use crate::mat4;
use crate::Error;

/// Serialize the given world-space meshes into binary STL bytes.
///
/// Each entity's local mesh is transformed by its model matrix first, so
/// the file matches exactly what the viewer shows. Face normals are
/// computed here; degenerate triangles get a zero normal (parsers ignore
/// STL normals anyway).
pub fn stl_bytes(entries: &[(Mesh, Mat4)]) -> (Vec<u8>, usize) {
    // Pre-compute world-space triangles so the count is exact.
    let mut tris: Vec<([f32; 3], [f32; 3], [f32; 3])> = Vec::new();
    for (mesh, m) in entries {
        for tri in mesh.indices.chunks_exact(3) {
            let world: [[f32; 3]; 3] = std::array::from_fn(|i| {
                let p = mesh.positions[tri[i] as usize];
                mat4::transform_point(m, p)
            });
            tris.push((world[0], world[1], world[2]));
        }
    }

    let mut out = Vec::with_capacity(84 + tris.len() * 50);
    let mut header = [0u8; 80];
    let label = b"wright binary STL - parametric solids, units mm";
    header[..label.len()].copy_from_slice(label);
    out.extend_from_slice(&header);
    out.extend_from_slice(&(tris.len() as u32).to_le_bytes());

    for &(a, b, c) in &tris {
        out.extend_from_slice(&face_normal(a, b, c));
        for p in [a, b, c] {
            for &v in &p {
                out.extend_from_slice(&v.to_le_bytes());
            }
        }
        out.extend_from_slice(&0u16.to_le_bytes()); // attribute byte count
    }
    (out, tris.len())
}

fn face_normal(a: [f32; 3], b: [f32; 3], c: [f32; 3]) -> [u8; 12] {
    let e1 = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
    let e2 = [c[0] - a[0], c[1] - a[1], c[2] - a[2]];
    let n = [
        e1[1] * e2[2] - e1[2] * e2[1],
        e1[2] * e2[0] - e1[0] * e2[2],
        e1[0] * e2[1] - e1[1] * e2[0],
    ];
    let len = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
    let unit = if len > 1e-12 { [n[0] / len, n[1] / len, n[2] / len] } else { [0.0; 3] };
    let mut bytes = [0u8; 12];
    for (i, v) in unit.iter().enumerate() {
        bytes[i * 4..i * 4 + 4].copy_from_slice(&v.to_le_bytes());
    }
    bytes
}

/// Write an STL file, returning `(path, triangles)`.
pub fn write_stl(path: &str, entries: &[(Mesh, Mat4)]) -> Result<(String, usize), Error> {
    let (bytes, tris) = stl_bytes(entries);
    std::fs::write(path, &bytes)
        .map_err(|e| Error::Invalid(format!("cannot write STL to '{path}': {e} (hint: use an absolute path the server can write)")))?;
    let resolved = std::fs::canonicalize(path)
        .map(|p| p.to_string_lossy().into_owned())
        .unwrap_or_else(|_| path.to_string());
    Ok((resolved, tris))
}
