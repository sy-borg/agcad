//! Mesh import: turn STL (binary or ASCII), OBJ and 3MF files into the
//! triangle-mesh solid representation the rest of the engine uses.
//!
//! Import is deliberately data, not a live file reference. The parsed
//! triangles become an [`crate::Command::AddMesh`] program step, so a
//! `.wcad` save replays without the original file. Everything here is
//! deterministic: vertices are welded by exact `f32` position bits, triangles
//! are visited in file order, and degenerate/duplicate triangles are dropped
//! before shading normals are derived.
//!
//! Two repairs are applied, because real interchange files are messy:
//!
//! * a globally inward winding (negative signed volume) is flipped outward;
//! * vertices are welded and duplicate triangles removed.
//!
//! Anything beyond that — cracks from coordinate drift, self-intersections,
//! non-manifold contacts — is left alone so `validate` can report it honestly
//! instead of the importer silently changing the part.

use std::collections::{BTreeMap, BTreeSet};
use std::io::Read;
use std::path::Path;

use wright_protocol::{Mesh, MeshFormat};

use crate::{Error, Result};

/// Default crease angle for shading normals, matching the edge extractor's
/// definition of a real edge.
pub const DEFAULT_SMOOTH_ANGLE_DEG: f64 = 30.0;

/// A parsed file, before welding and clean-up. Coordinates are in the file's
/// own units and `unit_scale` converts them to millimeters.
struct Parsed {
    positions: Vec<[f64; 3]>,
    triangles: Vec<[u32; 3]>,
    name: Option<String>,
    unit_scale: f64,
}

/// Cleaned import data: welded vertices and the triangles that survived.
#[derive(Debug)]
pub struct CleanMesh {
    pub positions: Vec<[f32; 3]>,
    pub triangles: Vec<[u32; 3]>,
    /// Degenerate or duplicate input triangles that were discarded.
    pub dropped: usize,
}

/// The result of reading a mesh file, ready to hand to `add_mesh`.
#[derive(Debug)]
pub struct Imported {
    pub format: MeshFormat,
    pub name: String,
    pub positions: Vec<[f32; 3]>,
    /// Flat triangle indices (three per triangle).
    pub indices: Vec<u32>,
    pub vertices: usize,
    pub triangles: usize,
    pub dropped: usize,
    pub bytes: usize,
}

/// Read `path` and parse it as a mesh. `format` overrides detection; `scale`
/// is a uniform multiplier applied after the file's own unit conversion.
pub fn import_file(path: &str, format: Option<MeshFormat>, scale: f64) -> Result<Imported> {
    if !scale.is_finite() || scale <= 0.0 {
        return Err(Error::Invalid(format!(
            "scale must be a finite value > 0 (got {scale}; hint: use 25.4 for inches when the \
             file has no unit metadata)"
        )));
    }
    let bytes = std::fs::read(path).map_err(|e| {
        Error::Document(format!(
            "cannot read mesh '{path}': {e} (hint: use an absolute path the server can read)"
        ))
    })?;
    let format = match format {
        Some(f) => f,
        None => detect_format(&bytes, path)?,
    };

    let parsed = match format {
        MeshFormat::Stl => parse_stl(&bytes),
        MeshFormat::Obj => parse_obj(&bytes),
        MeshFormat::ThreeMf => parse_3mf(&bytes),
    }
    .map_err(|e| Error::Document(format!("cannot import '{path}' as {format}: {e}")))?;

    let total_scale = parsed.unit_scale * scale;
    let mut raw = Vec::with_capacity(parsed.positions.len());
    for p in &parsed.positions {
        let q = [
            (p[0] * total_scale) as f32,
            (p[1] * total_scale) as f32,
            (p[2] * total_scale) as f32,
        ];
        if q.iter().any(|v| !v.is_finite()) {
            return Err(Error::Document(format!(
                "mesh '{path}' contains a non-finite vertex (hint: check the file for NaN/inf \
                 coordinates)"
            )));
        }
        raw.push(q);
    }

    let cleaned = clean(&raw, &parsed.triangles)?;
    let name = parsed
        .name
        .filter(|n| !n.trim().is_empty())
        .unwrap_or_else(|| file_stem(path));
    Ok(Imported {
        format,
        name,
        vertices: cleaned.positions.len(),
        triangles: cleaned.triangles.len(),
        indices: cleaned.triangles.iter().flatten().copied().collect(),
        positions: cleaned.positions,
        dropped: cleaned.dropped,
        bytes: bytes.len(),
    })
}

/// Build the renderable/indexed mesh for `add_mesh`: clean the raw triangles,
/// then derive crease-aware normals. Returns the cleaned topology so callers
/// can report counts.
pub fn build(positions: &[[f32; 3]], indices: &[u32], smooth_angle_deg: f64) -> Result<(Mesh, CleanMesh)> {
    if !smooth_angle_deg.is_finite() {
        return Err(Error::Invalid(format!(
            "smooth_angle_deg must be a finite angle in degrees (got {smooth_angle_deg})"
        )));
    }
    if !indices.len().is_multiple_of(3) {
        return Err(Error::Invalid(format!(
            "mesh indices must be a multiple of 3 (got {}; hint: each triangle is three vertex \
             indices)",
            indices.len()
        )));
    }
    if indices.is_empty() {
        return Err(Error::Invalid(
            "mesh has no triangles (hint: pass at least three indices)".into(),
        ));
    }
    let triangles: Vec<[u32; 3]> = indices
        .chunks_exact(3)
        .map(|c| [c[0], c[1], c[2]])
        .collect();
    let cleaned = clean(positions, &triangles)?;
    let mesh = with_normals(&cleaned.positions, &cleaned.triangles, smooth_angle_deg);
    Ok((mesh, cleaned))
}

// --- clean-up ---------------------------------------------------------------

/// Weld vertices by exact `f32` bits, drop triangles that cannot contribute
/// (repeated corners, exact duplicates), and flip the winding when the signed
/// volume is inward. Zero-area triangles with distinct corners are kept: they
/// can be structural seam triangles.
pub fn clean(positions: &[[f32; 3]], triangles: &[[u32; 3]]) -> Result<CleanMesh> {
    let mut unique: Vec<[f32; 3]> = Vec::with_capacity(positions.len());
    let mut ids: BTreeMap<[u32; 3], u32> = BTreeMap::new();
    let mut remap: Vec<u32> = Vec::with_capacity(positions.len());
    for p in positions {
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
                let id = unique.len() as u32;
                ids.insert(key, id);
                unique.push(q);
                id
            },
        };
        remap.push(id);
    }

    let mut seen: BTreeSet<[u32; 3]> = BTreeSet::new();
    let mut kept: Vec<[u32; 3]> = Vec::with_capacity(triangles.len());
    let mut dropped = 0usize;
    for tri in triangles {
        let t = [
            *remap.get(tri[0] as usize).ok_or_else(|| bad_index(tri[0], positions.len()))?,
            *remap.get(tri[1] as usize).ok_or_else(|| bad_index(tri[1], positions.len()))?,
            *remap.get(tri[2] as usize).ok_or_else(|| bad_index(tri[2], positions.len()))?,
        ];
        if t[0] == t[1] || t[1] == t[2] || t[0] == t[2] {
            dropped += 1;
            continue;
        }
        // Zero-area triangles with distinct corners are deliberately kept:
        // the manifold kernel emits them on tangent seams and using them is
        // what keeps the surrounding edge pairing closed.
        let mut key = t;
        key.sort_unstable();
        if !seen.insert(key) {
            dropped += 1;
            continue;
        }
        kept.push(t);
    }
    if kept.is_empty() {
        return Err(Error::Invalid(
            "mesh has no usable triangles (hint: every triangle was degenerate or duplicate; \
             check the file)"
                .into(),
        ));
    }

    // A single global winding flip is the best we can do without connectivity
    // analysis; it catches the common "exported inside out" case.
    if signed_volume(&unique, &kept) < 0.0 {
        for t in &mut kept {
            t.swap(1, 2);
        }
    }

    // Drop vertices no surviving triangle references and renumber compactly.
    let mut used = vec![false; unique.len()];
    for t in &kept {
        for &i in t {
            used[i as usize] = true;
        }
    }
    let mut new_id = vec![u32::MAX; unique.len()];
    let mut out_positions = Vec::with_capacity(unique.len());
    for (i, keep) in used.iter().enumerate() {
        if *keep {
            new_id[i] = out_positions.len() as u32;
            out_positions.push(unique[i]);
        }
    }
    for t in &mut kept {
        for i in t {
            *i = new_id[*i as usize];
        }
    }

    Ok(CleanMesh { positions: out_positions, triangles: kept, dropped })
}

fn bad_index(index: u32, len: usize) -> Error {
    Error::Invalid(format!(
        "mesh index {index} is out of range for {len} vertices (hint: indices are 0-based)"
    ))
}

fn signed_volume(positions: &[[f32; 3]], triangles: &[[u32; 3]]) -> f64 {
    let p = |i: u32| -> [f64; 3] {
        let v = positions[i as usize];
        [v[0] as f64, v[1] as f64, v[2] as f64]
    };
    let mut volume = 0.0;
    for t in triangles {
        let a = p(t[0]);
        let b = p(t[1]);
        let c = p(t[2]);
        volume += dot(a, cross(b, c)) / 6.0;
    }
    volume
}

fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

// --- normals ----------------------------------------------------------------

/// Crease-aware normals: at each corner, average the incident face normals
/// whose directions are within `smooth_angle_deg` of the corner's own face.
/// Faces are area-weighted, so a dense fan does not out-vote a large face.
fn with_normals(positions: &[[f32; 3]], triangles: &[[u32; 3]], smooth_angle_deg: f64) -> Mesh {
    let angle = smooth_angle_deg.clamp(0.0, 180.0);
    let threshold = angle.to_radians().cos();

    let faces: Vec<([f64; 3], f64)> = triangles
        .iter()
        .map(|t| {
            let a = to_f64(positions[t[0] as usize]);
            let b = to_f64(positions[t[1] as usize]);
            let c = to_f64(positions[t[2] as usize]);
            let n = cross([b[0] - a[0], b[1] - a[1], b[2] - a[2]], [
                c[0] - a[0],
                c[1] - a[1],
                c[2] - a[2],
            ]);
            let len = dot(n, n).sqrt();
            if len > 0.0 {
                ([n[0] / len, n[1] / len, n[2] / len], len * 0.5)
            } else {
                ([0.0, 0.0, 1.0], 0.0)
            }
        })
        .collect();

    let mut incident: Vec<Vec<u32>> = vec![Vec::new(); positions.len()];
    for (fi, t) in triangles.iter().enumerate() {
        for &v in t {
            incident[v as usize].push(fi as u32);
        }
    }

    let mut mesh = Mesh::default();
    let mut lookup: BTreeMap<([u32; 3], [u32; 3]), u32> = BTreeMap::new();
    for (fi, t) in triangles.iter().enumerate() {
        let (face_normal, _) = faces[fi];
        let mut ids = [0u32; 3];
        for (corner, &v) in t.iter().enumerate() {
            let mut sum = [0.0; 3];
            for &gi in &incident[v as usize] {
                let (other, area) = faces[gi as usize];
                if dot(face_normal, other) >= threshold - 1e-12 {
                    sum[0] += other[0] * area;
                    sum[1] += other[1] * area;
                    sum[2] += other[2] * area;
                }
            }
            let normal = normalize(sum).unwrap_or_else(|| {
                normalize(face_normal).unwrap_or([0.0, 0.0, 1.0])
            });
            let p = positions[v as usize];
            let key = (
                p.map(|x| (if x == 0.0 { 0.0 } else { x }).to_bits()),
                normal.map(f32::to_bits),
            );
            let id = *lookup.entry(key).or_insert_with(|| {
                let id = mesh.positions.len() as u32;
                mesh.positions.push(p);
                mesh.normals.push(normal);
                id
            });
            ids[corner] = id;
        }
        mesh.indices.extend_from_slice(&ids);
    }
    mesh
}

fn normalize(v: [f64; 3]) -> Option<[f32; 3]> {
    let len = dot(v, v).sqrt();
    if !len.is_finite() || len <= 0.0 {
        return None;
    }
    Some([
        clean_zero((v[0] / len) as f32),
        clean_zero((v[1] / len) as f32),
        clean_zero((v[2] / len) as f32),
    ])
}

fn clean_zero(v: f32) -> f32 {
    if v == 0.0 { 0.0 } else { v }
}

fn to_f64(p: [f32; 3]) -> [f64; 3] {
    [p[0] as f64, p[1] as f64, p[2] as f64]
}

// --- format detection -------------------------------------------------------

fn detect_format(bytes: &[u8], path: &str) -> Result<MeshFormat> {
    if let Some(format) = Path::new(path)
        .extension()
        .and_then(|e| e.to_str())
        .and_then(MeshFormat::from_extension)
    {
        return Ok(format);
    }
    if bytes.starts_with(b"PK\x03\x04") {
        return Ok(MeshFormat::ThreeMf);
    }
    if binary_stl_count(bytes).is_some() {
        return Ok(MeshFormat::Stl);
    }
    let head = &bytes[..bytes.len().min(4096)];
    if let Ok(text) = std::str::from_utf8(head) {
        let lower = text.to_ascii_lowercase();
        if lower.contains("facet") || lower.contains("vertex") {
            return Ok(MeshFormat::Stl);
        }
        let mut has_v = false;
        let mut has_f = false;
        for line in text.lines() {
            let line = line.trim_start();
            has_v |= line.starts_with("v ");
            has_f |= line.starts_with("f ");
        }
        if has_v && has_f {
            return Ok(MeshFormat::Obj);
        }
    }
    Err(Error::Document(format!(
        "cannot detect the mesh format of '{path}' (hint: supported formats are STL, OBJ and 3MF; \
         name the file .stl/.obj/.3mf or pass `format`)"
    )))
}

fn file_stem(path: &str) -> String {
    Path::new(path)
        .file_stem()
        .and_then(|s| s.to_str())
        .filter(|s| !s.is_empty())
        .unwrap_or("mesh")
        .to_string()
}

// --- STL --------------------------------------------------------------------

fn binary_stl_count(bytes: &[u8]) -> Option<usize> {
    if bytes.len() < 84 {
        return None;
    }
    let count = u32::from_le_bytes([bytes[80], bytes[81], bytes[82], bytes[83]]) as usize;
    let expected = 84usize.checked_add(count.checked_mul(50)?)?;
    (expected == bytes.len()).then_some(count)
}

fn looks_ascii_stl(bytes: &[u8]) -> bool {
    let head = &bytes[..bytes.len().min(4096)];
    let Ok(text) = std::str::from_utf8(head) else {
        return false;
    };
    let trimmed = text.trim_start();
    if !trimmed.to_ascii_lowercase().starts_with("solid") {
        return false;
    }
    let lower = text.to_ascii_lowercase();
    lower.contains("facet") || lower.contains("vertex")
}

fn parse_stl(bytes: &[u8]) -> Result<Parsed> {
    if let Some(count) = binary_stl_count(bytes) {
        return parse_binary_stl(bytes, count);
    }
    if looks_ascii_stl(bytes) {
        return parse_ascii_stl(bytes);
    }
    if bytes.len() >= 84 {
        // Tolerate trailing bytes after the declared triangles.
        let count = u32::from_le_bytes([bytes[80], bytes[81], bytes[82], bytes[83]]) as usize;
        let fits = count
            .checked_mul(50)
            .and_then(|n| 84usize.checked_add(n))
            .is_some_and(|n| n <= bytes.len());
        if count > 0 && fits {
            return parse_binary_stl(bytes, count);
        }
    }
    Err(Error::Document(
        "STL: not a binary or ASCII STL (hint: binary STL is 84 + 50*triangles bytes; ASCII STL \
         starts with `solid` and uses `vertex` lines)"
            .into(),
    ))
}

fn parse_binary_stl(bytes: &[u8], count: usize) -> Result<Parsed> {
    let mut positions = Vec::with_capacity(count * 3);
    let mut triangles = Vec::with_capacity(count);
    for i in 0..count {
        let base = 84 + i * 50;
        // Skip the facet normal (12 bytes); three vertices follow.
        for corner in 0..3 {
            let at = base + 12 + corner * 12;
            let x = read_f32(bytes, at)? as f64;
            let y = read_f32(bytes, at + 4)? as f64;
            let z = read_f32(bytes, at + 8)? as f64;
            positions.push([x, y, z]);
        }
        let base_id = (i * 3) as u32;
        triangles.push([base_id, base_id + 1, base_id + 2]);
    }
    if positions.is_empty() {
        return Err(Error::Document(
            "STL: file contains no triangles (hint: the triangle count is zero)".into(),
        ));
    }
    Ok(Parsed {
        positions,
        triangles,
        name: None,
        unit_scale: 1.0,
    })
}

fn parse_ascii_stl(bytes: &[u8]) -> Result<Parsed> {
    let text = String::from_utf8_lossy(bytes);
    let mut name = None;
    if let Some(first) = text.lines().next() {
        let first = first.trim();
        if first.get(..5).is_some_and(|head| head.eq_ignore_ascii_case("solid")) {
            let rest = first[5..].trim();
            if !rest.is_empty() {
                name = Some(rest.to_string());
            }
        }
    }

    let tokens: Vec<&str> = text.split_ascii_whitespace().collect();
    let mut positions = Vec::new();
    let mut i = 0;
    while i < tokens.len() {
        if tokens[i].eq_ignore_ascii_case("vertex") {
            if i + 3 >= tokens.len() {
                return Err(Error::Document(
                    "STL: a `vertex` line is missing coordinates".into(),
                ));
            }
            positions.push([
                tokens[i + 1].parse::<f64>().map_err(|_| bad_stl_number(tokens[i + 1]))?,
                tokens[i + 2].parse::<f64>().map_err(|_| bad_stl_number(tokens[i + 2]))?,
                tokens[i + 3].parse::<f64>().map_err(|_| bad_stl_number(tokens[i + 3]))?,
            ]);
            i += 4;
        } else {
            i += 1;
        }
    }
    if positions.is_empty() {
        return Err(Error::Document(
            "STL: no `vertex` lines found (hint: is this really an ASCII STL?)".into(),
        ));
    }
    if positions.len() % 3 != 0 {
        return Err(Error::Document(format!(
            "STL: found {} vertices, not a multiple of 3 (hint: every triangle has three)",
            positions.len()
        )));
    }
    let triangles = (0..positions.len() / 3)
        .map(|i| {
            let b = (i * 3) as u32;
            [b, b + 1, b + 2]
        })
        .collect();
    Ok(Parsed {
        positions,
        triangles,
        name,
        unit_scale: 1.0,
    })
}

fn bad_stl_number(raw: &str) -> Error {
    Error::Document(format!("STL: `{raw}` is not a number"))
}

fn read_f32(bytes: &[u8], at: usize) -> Result<f32> {
    let b = bytes.get(at..at + 4).ok_or_else(|| {
        Error::Document("STL: file is truncated (hint: binary STL needs 50 bytes per triangle)".into())
    })?;
    Ok(f32::from_le_bytes([b[0], b[1], b[2], b[3]]))
}

// --- OBJ --------------------------------------------------------------------

fn parse_obj(bytes: &[u8]) -> Result<Parsed> {
    let text = String::from_utf8_lossy(bytes);
    let mut positions: Vec<[f64; 3]> = Vec::new();
    let mut triangles: Vec<[u32; 3]> = Vec::new();
    let mut name = None;
    for (line_no, raw) in text.lines().enumerate() {
        let line = raw.trim();
        if line.is_empty() || line.starts_with('#') {
            continue;
        }
        let mut parts = line.split_ascii_whitespace();
        match parts.next() {
            Some("v") => {
                let x = obj_number(parts.next(), line_no)?;
                let y = obj_number(parts.next(), line_no)?;
                let z = obj_number(parts.next(), line_no)?;
                positions.push([x, y, z]);
            },
            Some("o") => {
                if name.is_none() {
                    let rest = parts.collect::<Vec<_>>().join(" ");
                    if !rest.trim().is_empty() {
                        name = Some(rest);
                    }
                }
            },
            Some("f") => {
                let mut corners = Vec::new();
                for token in parts {
                    let raw = token.split('/').next().unwrap_or("");
                    let index: i64 = raw.parse().map_err(|_| {
                        Error::Document(format!(
                            "OBJ line {}: `{token}` is not a vertex reference",
                            line_no + 1
                        ))
                    })?;
                    let resolved = if index > 0 {
                        index - 1
                    } else if index < 0 {
                        positions.len() as i64 + index
                    } else {
                        return Err(Error::Document(format!(
                            "OBJ line {}: vertex index 0 is not valid (hint: OBJ indices are \
                             1-based; negative indices count from the end)",
                            line_no + 1
                        )));
                    };
                    if resolved < 0 || resolved >= positions.len() as i64 {
                        return Err(Error::Document(format!(
                            "OBJ line {}: vertex index {index} is out of range ({} vertices so far)",
                            line_no + 1,
                            positions.len()
                        )));
                    }
                    corners.push(resolved as u32);
                }
                if corners.len() < 3 {
                    return Err(Error::Document(format!(
                        "OBJ line {}: a face needs at least 3 vertices",
                        line_no + 1
                    )));
                }
                // Fan-triangulate polygons (OBJ has no guaranteed convexity,
                // but fanning is the format's conventional interpretation).
                for i in 1..corners.len() - 1 {
                    triangles.push([corners[0], corners[i], corners[i + 1]]);
                }
            },
            _ => {},
        }
    }
    if positions.is_empty() {
        return Err(Error::Document("OBJ: no `v` vertex lines found".into()));
    }
    if triangles.is_empty() {
        return Err(Error::Document(
            "OBJ: no `f` faces found (hint: points and lines are not a solid)".into(),
        ));
    }
    Ok(Parsed {
        positions,
        triangles,
        name,
        unit_scale: 1.0,
    })
}

fn obj_number(raw: Option<&str>, line_no: usize) -> Result<f64> {
    let raw = raw.ok_or_else(|| {
        Error::Document(format!("OBJ line {}: `v` needs three coordinates", line_no + 1))
    })?;
    raw.parse::<f64>().map_err(|_| {
        Error::Document(format!("OBJ line {}: `{raw}` is not a number", line_no + 1))
    })
}

// --- 3MF --------------------------------------------------------------------

fn parse_3mf(bytes: &[u8]) -> Result<Parsed> {
    let entries = zip_entries(bytes)?;
    let part = model_part(&entries)?;
    let xml = std::str::from_utf8(entries.get(&part).unwrap()).map_err(|_| {
        Error::Document(format!("3MF: model part `{part}` is not valid UTF-8"))
    })?;

    let mut unit_scale = 1.0;
    let mut objects: BTreeMap<u32, Object> = BTreeMap::new();
    let mut current: Option<Object> = None;
    let mut in_vertices = false;
    let mut in_triangles = false;
    let mut in_build = false;
    let mut items: Vec<BuildItem> = Vec::new();

    for_each_tag(xml, |tag, closing, self_closing, attrs| {
        let name = local_name(tag);
        match name {
            "model" if !closing => {
                unit_scale = unit_scale_of(attr(attrs, "unit"))?;
            },
            "object" => {
                if closing {
                    let obj = current.take().ok_or_else(|| {
                        Error::Document("3MF: stray </object> (hint: malformed model XML)".into())
                    })?;
                    let id = obj.id;
                    if objects.insert(id, obj).is_some() {
                        return Err(Error::Document(format!("3MF: duplicate object id {id}")));
                    }
                } else {
                    if current.is_some() {
                        return Err(Error::Document("3MF: nested <object> elements".into()));
                    }
                    let id = attr_u32(attrs, "id", "object")?;
                    current = Some(Object {
                        id,
                        name: attr(attrs, "name").map(str::to_string),
                        positions: Vec::new(),
                        triangles: Vec::new(),
                        components: Vec::new(),
                    });
                }
            },
            "vertices" => in_vertices = !closing && !self_closing,
            "triangles" => in_triangles = !closing && !self_closing,
            "build" => in_build = !closing && !self_closing,
            "vertex" if in_vertices && !closing => {
                let obj = current.as_mut().ok_or_else(|| {
                    Error::Document("3MF: <vertex> outside an <object>".into())
                })?;
                obj.positions.push([
                    attr_f64(attrs, "x", "vertex")?,
                    attr_f64(attrs, "y", "vertex")?,
                    attr_f64(attrs, "z", "vertex")?,
                ]);
            },
            "triangle" if in_triangles && !closing => {
                let obj = current.as_mut().ok_or_else(|| {
                    Error::Document("3MF: <triangle> outside an <object>".into())
                })?;
                obj.triangles.push([
                    attr_u32(attrs, "v1", "triangle")?,
                    attr_u32(attrs, "v2", "triangle")?,
                    attr_u32(attrs, "v3", "triangle")?,
                ]);
            },
            "component" if !closing => {
                let obj = current.as_mut().ok_or_else(|| {
                    Error::Document("3MF: <component> outside an <object>".into())
                })?;
                obj.components.push((
                    attr_u32(attrs, "objectid", "component")?,
                    parse_transform(attr(attrs, "transform"))?,
                ));
            },
            "item" if in_build && !closing => {
                items.push(BuildItem {
                    object: attr_u32(attrs, "objectid", "item")?,
                    transform: parse_transform(attr(attrs, "transform"))?,
                });
            },
            _ => {},
        }
        Ok(())
    })?;

    if objects.is_empty() {
        return Err(Error::Document(
            "3MF: model has no objects (hint: an empty 3MF has nothing to import)".into(),
        ));
    }

    // Validate every object's local indices before resolving.
    for obj in objects.values() {
        for tri in &obj.triangles {
            for &i in tri {
                if i as usize >= obj.positions.len() {
                    return Err(Error::Document(format!(
                        "3MF object {}: triangle index {i} is out of range for {} vertices",
                        obj.id,
                        obj.positions.len()
                    )));
                }
            }
        }
    }

    // The build section is authoritative. If it is missing (or empty) fall
    // back to every object that has geometry and is not used as a component.
    if items.is_empty() {
        let referenced: BTreeSet<u32> = objects
            .values()
            .flat_map(|o| o.components.iter().map(|(id, _)| *id))
            .collect();
        items = objects
            .values()
            .filter(|o| {
                (!o.triangles.is_empty() || !o.components.is_empty())
                    && !referenced.contains(&o.id)
            })
            .map(|o| BuildItem { object: o.id, transform: None })
            .collect();
    }
    if items.is_empty() {
        return Err(Error::Document(
            "3MF: the build is empty (hint: no <item> in <build> and no standalone mesh objects)"
                .into(),
        ));
    }

    let name = if items.len() == 1 {
        objects.get(&items[0].object).and_then(|o| o.name.clone())
    } else {
        None
    };

    let mut positions = Vec::new();
    let mut triangles = Vec::new();
    let mut stack = Vec::new();
    for item in &items {
        resolve_object(
            item.object,
            item.transform,
            &objects,
            &mut positions,
            &mut triangles,
            &mut stack,
        )?;
    }
    if triangles.is_empty() {
        return Err(Error::Document(
            "3MF: the build resolves to no triangles (hint: objects may only contain components \
             that do not exist)"
                .into(),
        ));
    }
    Ok(Parsed {
        positions,
        triangles,
        name,
        unit_scale,
    })
}

struct Object {
    id: u32,
    name: Option<String>,
    positions: Vec<[f64; 3]>,
    triangles: Vec<[u32; 3]>,
    components: Vec<(u32, Option<[f64; 12]>)>,
}

struct BuildItem {
    object: u32,
    transform: Option<[f64; 12]>,
}

fn resolve_object(
    id: u32,
    parent: Option<[f64; 12]>,
    objects: &BTreeMap<u32, Object>,
    positions: &mut Vec<[f64; 3]>,
    triangles: &mut Vec<[u32; 3]>,
    stack: &mut Vec<u32>,
) -> Result<()> {
    if stack.len() > 64 {
        return Err(Error::Document(
            "3MF: component nesting is too deep (hint: possible cycle)".into(),
        ));
    }
    if stack.contains(&id) {
        return Err(Error::Document(format!(
            "3MF: component cycle through object {id}"
        )));
    }
    let obj = objects.get(&id).ok_or_else(|| {
        Error::Document(format!(
            "3MF: build references object {id}, which does not exist"
        ))
    })?;
    for tri in &obj.triangles {
        let base = positions.len() as u32;
        for &vi in tri {
            let p = obj.positions[vi as usize];
            positions.push(match parent {
                Some(m) => apply_transform(&m, p),
                None => p,
            });
        }
        triangles.push([base, base + 1, base + 2]);
    }
    stack.push(id);
    for (child, transform) in &obj.components {
        let combined = match (parent, transform) {
            (None, None) => None,
            (Some(p), None) => Some(p),
            (None, Some(c)) => Some(*c),
            // Row-vector convention: p * component * item.
            (Some(p), Some(c)) => Some(mat_mul(c, &p)),
        };
        resolve_object(*child, combined, objects, positions, triangles, stack)?;
    }
    stack.pop();
    Ok(())
}

/// 3MF transforms are 4x3 row-major matrices with an implied last row
/// `0 0 0 1`; points are row vectors (`p' = p * M`).
fn apply_transform(m: &[f64; 12], p: [f64; 3]) -> [f64; 3] {
    [
        p[0] * m[0] + p[1] * m[3] + p[2] * m[6] + m[9],
        p[0] * m[1] + p[1] * m[4] + p[2] * m[7] + m[10],
        p[0] * m[2] + p[1] * m[5] + p[2] * m[8] + m[11],
    ]
}

/// Row-vector matrix product `a * b`, with the implied `[0,0,0,1]` column.
fn mat_mul(a: &[f64; 12], b: &[f64; 12]) -> [f64; 12] {
    let av = |r: usize, c: usize| if c == 3 {
        if r == 3 { 1.0 } else { 0.0 }
    } else {
        a[r * 3 + c]
    };
    let bv = |r: usize, c: usize| if c == 3 {
        if r == 3 { 1.0 } else { 0.0 }
    } else {
        b[r * 3 + c]
    };
    let mut out = [0.0; 12];
    for r in 0..4 {
        for c in 0..3 {
            let mut sum = 0.0;
            for k in 0..4 {
                sum += av(r, k) * bv(k, c);
            }
            out[r * 3 + c] = sum;
        }
    }
    out
}

fn parse_transform(raw: Option<&str>) -> Result<Option<[f64; 12]>> {
    let Some(raw) = raw else { return Ok(None) };
    let values: Vec<f64> = raw
        .split_ascii_whitespace()
        .map(|v| {
            v.parse::<f64>().map_err(|_| {
                Error::Document(format!("3MF: `{v}` in a transform is not a number"))
            })
        })
        .collect::<Result<_>>()?;
    if values.len() != 12 {
        return Err(Error::Document(format!(
            "3MF: a transform needs 12 numbers (got {}; hint: 4x3 row-major matrix)",
            values.len()
        )));
    }
    if values.iter().any(|v| !v.is_finite()) {
        return Err(Error::Document("3MF: a transform contains a non-finite number".into()));
    }
    let mut m = [0.0; 12];
    m.copy_from_slice(&values);
    Ok(Some(m))
}

fn unit_scale_of(unit: Option<&str>) -> Result<f64> {
    let unit = unit.unwrap_or("millimeter");
    match unit.to_ascii_lowercase().as_str() {
        "micron" => Ok(0.001),
        "millimeter" => Ok(1.0),
        "centimeter" => Ok(10.0),
        "inch" => Ok(25.4),
        "foot" => Ok(304.8),
        "meter" => Ok(1000.0),
        other => Err(Error::Document(format!(
            "3MF: unknown unit `{other}` (hint: micron, millimeter, centimeter, inch, foot or \
             meter)"
        ))),
    }
}

fn local_name(tag: &str) -> &str {
    tag.rsplit(':').next().unwrap_or(tag)
}

/// Parse one XML attribute from the raw attribute text of a start tag.
fn attr<'a>(attrs: &'a str, key: &str) -> Option<&'a str> {
    let mut rest = attrs;
    loop {
        rest = rest.trim_start();
        if rest.is_empty() {
            return None;
        }
        let eq = rest.find('=')?;
        let name = rest[..eq].trim_end();
        let after = rest[eq + 1..].trim_start();
        let (value, tail) = match after.as_bytes().first() {
            Some(b'"') => {
                let end = after[1..].find('"')? + 1;
                (&after[1..end], &after[end + 1..])
            },
            Some(b'\'') => {
                let end = after[1..].find('\'')? + 1;
                (&after[1..end], &after[end + 1..])
            },
            _ => return None,
        };
        if name == key {
            return Some(value);
        }
        rest = tail;
    }
}

fn attr_f64(attrs: &str, key: &str, context: &str) -> Result<f64> {
    let raw = attr(attrs, key)
        .ok_or_else(|| Error::Document(format!("3MF: <{context}> is missing `{key}`")))?;
    raw.parse::<f64>()
        .map_err(|_| Error::Document(format!("3MF: <{context}> `{key}=\"{raw}\"` is not a number")))
}

fn attr_u32(attrs: &str, key: &str, context: &str) -> Result<u32> {
    let raw = attr(attrs, key)
        .ok_or_else(|| Error::Document(format!("3MF: <{context}> is missing `{key}`")))?;
    raw.parse::<u32>().map_err(|_| {
        Error::Document(format!(
            "3MF: <{context}> `{key}=\"{raw}\"` is not a non-negative integer"
        ))
    })
}

/// Minimal XML start/end-tag scanner. 3MF model XML is regular enough that a
/// full parser would be dead weight; this skips declarations and comments and
/// hands each element's raw attribute text to `f`.
fn for_each_tag(
    xml: &str,
    mut f: impl FnMut(&str, bool, bool, &str) -> Result<()>,
) -> Result<()> {
    let bytes = xml.as_bytes();
    let mut i = 0;
    while let Some(found) = xml[i..].find('<') {
        let start = i + found;
        if xml[start..].starts_with("<!--") {
            i = xml[start + 4..]
                .find("-->")
                .map(|p| start + 4 + p + 3)
                .ok_or_else(|| Error::Document("3MF: unterminated XML comment".into()))?;
            continue;
        }
        if xml[start..].starts_with("<?") {
            i = xml[start + 2..]
                .find("?>")
                .map(|p| start + 2 + p + 2)
                .ok_or_else(|| Error::Document("3MF: unterminated XML declaration".into()))?;
            continue;
        }
        if xml[start..].starts_with("<!") {
            i = xml[start + 2..]
                .find('>')
                .map(|p| start + 2 + p + 1)
                .ok_or_else(|| Error::Document("3MF: unterminated XML declaration".into()))?;
            continue;
        }
        let mut j = start + 1;
        let mut quote = None;
        while j < bytes.len() {
            let c = bytes[j];
            if let Some(q) = quote {
                if c == q {
                    quote = None;
                }
            } else if c == b'"' || c == b'\'' {
                quote = Some(c);
            } else if c == b'>' {
                break;
            }
            j += 1;
        }
        if j >= bytes.len() {
            return Err(Error::Document(
                "3MF: unterminated XML element (hint: the model part is truncated)".into(),
            ));
        }
        let inner = &xml[start + 1..j];
        let closing = inner.starts_with('/');
        let inner = if closing { &inner[1..] } else { inner };
        let (inner, self_closing) = match inner.trim_end().strip_suffix('/') {
            Some(body) => (body, true),
            None => (inner, false),
        };
        let inner = inner.trim();
        if !inner.is_empty() {
            let name_end = inner
                .find(|c: char| c.is_ascii_whitespace())
                .unwrap_or(inner.len());
            f(&inner[..name_end], closing, self_closing, &inner[name_end..])?;
        }
        i = j + 1;
    }
    Ok(())
}

// --- ZIP reader (store and deflate) -----------------------------------------

fn zip_entries(bytes: &[u8]) -> Result<BTreeMap<String, Vec<u8>>> {
    let eocd = find_eocd(bytes).ok_or_else(|| {
        Error::Document(
            "3MF: not a ZIP package (hint: a 3MF file is an OPC/ZIP archive; no end-of-central-\
             directory record found)"
                .into(),
        )
    })?;
    let count = read_u16(bytes, eocd + 10)? as usize;
    let cd_offset = read_u32(bytes, eocd + 16)? as usize;
    if count == 0xffff || cd_offset == 0xffff_ffff {
        return Err(Error::Document("3MF: ZIP64 packages are not supported".into()));
    }

    let mut out = BTreeMap::new();
    let mut p = cd_offset;
    for _ in 0..count {
        if read_u32(bytes, p)? != 0x0201_4b50 {
            return Err(Error::Document(
                "3MF: corrupt ZIP central directory (hint: is this a valid 3MF package?)".into(),
            ));
        }
        let method = read_u16(bytes, p + 10)?;
        let compressed = read_u32(bytes, p + 20)? as usize;
        let uncompressed = read_u32(bytes, p + 24)? as usize;
        let name_len = read_u16(bytes, p + 28)? as usize;
        let extra_len = read_u16(bytes, p + 30)? as usize;
        let comment_len = read_u16(bytes, p + 32)? as usize;
        let local = read_u32(bytes, p + 42)? as usize;
        let name = bytes
            .get(p + 46..p + 46 + name_len)
            .map(|b| String::from_utf8_lossy(b).into_owned())
            .ok_or_else(|| Error::Document("3MF: truncated ZIP entry name".into()))?;
        p += 46 + name_len + extra_len + comment_len;
        if name.ends_with('/') {
            continue;
        }

        if read_u32(bytes, local)? != 0x0403_4b50 {
            return Err(Error::Document("3MF: corrupt ZIP local header".into()));
        }
        let local_name = read_u16(bytes, local + 26)? as usize;
        let local_extra = read_u16(bytes, local + 28)? as usize;
        let data_start = local + 30 + local_name + local_extra;
        let data = bytes
            .get(data_start..data_start + compressed)
            .ok_or_else(|| Error::Document("3MF: truncated ZIP entry data".into()))?;
        let data = match method {
            0 => data.to_vec(),
            8 => inflate(data, uncompressed)?,
            other => {
                return Err(Error::Document(format!(
                    "3MF: unsupported ZIP compression method {other} (hint: only store and \
                     deflate are supported)"
                )));
            },
        };
        if uncompressed != 0 && data.len() != uncompressed {
            return Err(Error::Document(
                "3MF: ZIP entry size does not match its central-directory record".into(),
            ));
        }
        out.insert(name, data);
    }
    Ok(out)
}

fn find_eocd(bytes: &[u8]) -> Option<usize> {
    if bytes.len() < 22 {
        return None;
    }
    let start = bytes.len().saturating_sub(22 + 65_535);
    (start..=bytes.len() - 22)
        .rev()
        .find(|&pos| bytes[pos..].starts_with(b"PK\x05\x06"))
}

fn inflate(data: &[u8], hint: usize) -> Result<Vec<u8>> {
    let mut out = Vec::with_capacity(hint);
    flate2::read::DeflateDecoder::new(data)
        .read_to_end(&mut out)
        .map_err(|e| Error::Document(format!("3MF: cannot inflate ZIP entry: {e}")))?;
    Ok(out)
}

fn read_u16(bytes: &[u8], at: usize) -> Result<u16> {
    let b = bytes
        .get(at..at + 2)
        .ok_or_else(|| Error::Document("3MF: truncated ZIP data".into()))?;
    Ok(u16::from_le_bytes([b[0], b[1]]))
}

fn read_u32(bytes: &[u8], at: usize) -> Result<u32> {
    let b = bytes
        .get(at..at + 4)
        .ok_or_else(|| Error::Document("3MF: truncated ZIP data".into()))?;
    Ok(u32::from_le_bytes([b[0], b[1], b[2], b[3]]))
}

fn model_part(entries: &BTreeMap<String, Vec<u8>>) -> Result<String> {
    if let Some(text) = entries
        .get("_rels/.rels")
        .and_then(|b| std::str::from_utf8(b).ok())
    {
        let mut target = None;
        if for_each_tag(text, |tag, closing, _self_closing, attrs| {
            if !closing && local_name(tag) == "Relationship" {
                if let (Some(kind), Some(path)) = (attr(attrs, "Type"), attr(attrs, "Target")) {
                    if kind.ends_with("/3dmodel") {
                        target = Some(path.trim_start_matches('/').replace('\\', "/"));
                    }
                }
            }
            Ok(())
        })
        .is_ok()
        {
            if let Some(t) = target.filter(|t| entries.contains_key(t)) {
                return Ok(t);
            }
        }
    }
    if entries.contains_key("3D/3dmodel.model") {
        return Ok("3D/3dmodel.model".into());
    }
    if let Some(key) = entries.keys().find(|k| k.ends_with(".model")) {
        return Ok(key.clone());
    }
    Err(Error::Document(
        "3MF: package has no model part (hint: expected 3D/3dmodel.model)".into(),
    ))
}

#[cfg(test)]
mod tests {
    use super::*;

    use std::io::Write;

    use crate::{geo, mat4, stl, threemf};

    fn temp(name: &str, ext: &str) -> std::path::PathBuf {
        std::env::temp_dir().join(format!(
            "wright_import_{}_{}.{}",
            std::process::id(),
            name,
            ext
        ))
    }

    fn box_stl() -> Vec<u8> {
        let mesh = geo::box_mesh([10.0, 20.0, 30.0]);
        stl::stl_bytes(&[(mesh, mat4::identity())]).0
    }

    fn triangles(mesh: &Mesh) -> Vec<[u32; 3]> {
        mesh.indices
            .chunks_exact(3)
            .map(|c| [c[0], c[1], c[2]])
            .collect()
    }

    fn mesh_volume(mesh: &Mesh) -> f64 {
        signed_volume(&mesh.positions, &triangles(mesh))
    }

    fn bounds(positions: &[[f32; 3]]) -> ([f32; 3], [f32; 3]) {
        let mut lo = [f32::MAX; 3];
        let mut hi = [f32::MIN; 3];
        for p in positions {
            for i in 0..3 {
                lo[i] = lo[i].min(p[i]);
                hi[i] = hi[i].max(p[i]);
            }
        }
        (lo, hi)
    }

    #[test]
    fn binary_stl_round_trips_through_export() {
        let path = temp("binary_box", "stl");
        std::fs::write(&path, box_stl()).unwrap();
        let imported = import_file(path.to_str().unwrap(), None, 1.0).unwrap();
        assert_eq!(imported.format, MeshFormat::Stl);
        assert_eq!(imported.triangles, 12);
        assert_eq!(imported.vertices, 8, "welding collapses the shared box corners");
        let (lo, hi) = bounds(&imported.positions);
        assert_eq!(lo, [-5.0, -10.0, -15.0]);
        assert_eq!(hi, [5.0, 10.0, 15.0]);

        let (mesh, _) = build(&imported.positions, &imported.indices, DEFAULT_SMOOTH_ANGLE_DEG).unwrap();
        assert!((mesh_volume(&mesh).abs() - 6000.0).abs() < 1e-3);
        let _ = std::fs::remove_file(path);
    }

    #[test]
    fn binary_stl_with_a_solid_header_is_parsed_as_binary() {
        let mut bytes = box_stl();
        bytes[..5].copy_from_slice(b"solid");
        let path = temp("binary_solid_header", "stl");
        std::fs::write(&path, &bytes).unwrap();
        let imported = import_file(path.to_str().unwrap(), Some(MeshFormat::Stl), 1.0).unwrap();
        assert_eq!(imported.triangles, 12);
        let _ = std::fs::remove_file(path);
    }

    #[test]
    fn ascii_stl_parses_and_names_the_solid() {
        let text = "solid tetra\n\
            facet normal 0 0 -1\n outer loop\n  vertex 0 0 0\n  vertex 0 1 0\n  vertex 1 0 0\n endloop\nendfacet\n\
            facet normal -1 0 0\n outer loop\n  vertex 0 0 0\n  vertex 0 0 1\n  vertex 0 1 0\n endloop\nendfacet\n\
            facet normal 0 -1 0\n outer loop\n  vertex 0 0 0\n  vertex 1 0 0\n  vertex 0 0 1\n endloop\nendfacet\n\
            facet normal 1 1 1\n outer loop\n  vertex 1 0 0\n  vertex 0 1 0\n  vertex 0 0 1\n endloop\nendfacet\n\
            endsolid tetra\n";
        let path = temp("ascii_tetra", "stl");
        std::fs::write(&path, text).unwrap();
        let imported = import_file(path.to_str().unwrap(), Some(MeshFormat::Stl), 1.0).unwrap();
        assert_eq!(imported.name, "tetra");
        assert_eq!(imported.triangles, 4);
        let (mesh, _) = build(&imported.positions, &imported.indices, DEFAULT_SMOOTH_ANGLE_DEG).unwrap();
        assert!((mesh_volume(&mesh) - 1.0 / 6.0).abs() < 1e-6);
        let _ = std::fs::remove_file(path);
    }

    #[test]
    fn obj_polygons_and_negative_indices() {
        let text = "o plate\n\
            v 0 0 0\nv 10 0 0\nv 10 10 0\nv 0 10 0\n\
            f 1 2 3 4\n\
            v 0 0 5\nv 10 0 5\nv 10 10 5\nv 0 10 5\n\
            f -4 -3 -2 -1\n";
        let path = temp("obj_plate", "obj");
        std::fs::write(&path, text).unwrap();
        let imported = import_file(path.to_str().unwrap(), None, 1.0).unwrap();
        assert_eq!(imported.name, "plate");
        assert_eq!(imported.triangles, 4, "two quads fan to two triangles each");
        assert_eq!(imported.vertices, 8);
        let _ = std::fs::remove_file(path);
    }

    #[test]
    fn three_mf_export_round_trips() {
        let mesh = geo::box_mesh([10.0, 20.0, 30.0]);
        let (bytes, tris) = threemf::threemf_bytes(
            &[threemf::Part {
                name: "box",
                mesh: &mesh,
                transform: mat4::identity(),
                color: [0.2, 0.4, 0.6, 1.0],
            }],
            "round trip",
        );
        assert_eq!(tris, 12);
        let path = temp("threemf_box", "3mf");
        std::fs::write(&path, &bytes).unwrap();
        let imported = import_file(path.to_str().unwrap(), None, 1.0).unwrap();
        assert_eq!(imported.format, MeshFormat::ThreeMf);
        assert_eq!(imported.triangles, 12);
        assert_eq!(imported.vertices, 8);
        let (lo, hi) = bounds(&imported.positions);
        assert_eq!(lo, [-5.0, -10.0, -15.0]);
        assert_eq!(hi, [5.0, 10.0, 15.0]);
        let _ = std::fs::remove_file(path);
    }

    #[test]
    fn three_mf_applies_build_transform_and_units() {
        let xml = concat!(
            "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n",
            "<model unit=\"inch\" xmlns=\"http://schemas.microsoft.com/3dmanufacturing/core/2015/02\">\n",
            "  <resources>\n",
            "    <object id=\"1\" name=\"tri\"><mesh><vertices>\n",
            "      <vertex x=\"0\" y=\"0\" z=\"0\"/>\n",
            "      <vertex x=\"1\" y=\"0\" z=\"0\"/>\n",
            "      <vertex x=\"0\" y=\"1\" z=\"0\"/>\n",
            "    </vertices><triangles>\n",
            "      <triangle v1=\"0\" v2=\"1\" v3=\"2\"/>\n",
            "    </triangles></mesh></object>\n",
            "  </resources>\n",
            "  <build><item objectid=\"1\" transform=\"1 0 0 0 1 0 0 0 1 10 20 30\"/></build>\n",
            "</model>\n"
        );
        let path = temp("threemf_transform", "3mf");
        std::fs::write(&path, zip_package(&[("3D/3dmodel.model", xml.as_bytes())], false)).unwrap();
        let imported = import_file(path.to_str().unwrap(), Some(MeshFormat::ThreeMf), 1.0).unwrap();
        assert_eq!(imported.name, "tri");
        let (lo, hi) = bounds(&imported.positions);
        // One inch = 25.4 mm; the item translates by (10,20,30) inches too.
        assert!((lo[0] - 254.0).abs() < 1e-3, "lo={lo:?}");
        assert!((lo[1] - 508.0).abs() < 1e-3, "lo={lo:?}");
        assert!((lo[2] - 762.0).abs() < 1e-3, "lo={lo:?}");
        assert!((hi[0] - 279.4).abs() < 1e-3, "hi={hi:?}");
        let _ = std::fs::remove_file(path);
    }

    #[test]
    fn three_mf_composes_component_and_build_transforms() {
        let xml = concat!(
            "<model unit=\"millimeter\"><resources>\n",
            "<object id=\"1\"><mesh><vertices>\n",
            "<vertex x=\"0\" y=\"0\" z=\"0\"/><vertex x=\"1\" y=\"0\" z=\"0\"/><vertex x=\"0\" y=\"1\" z=\"0\"/>\n",
            "</vertices><triangles><triangle v1=\"0\" v2=\"1\" v3=\"2\"/></triangles></mesh></object>\n",
            "<object id=\"2\"><components>\n",
            "<component objectid=\"1\" transform=\"1 0 0 0 1 0 0 0 1 0 0 10\"/>\n",
            "</components></object>\n",
            "</resources><build><item objectid=\"2\" transform=\"1 0 0 0 1 0 0 0 1 100 0 0\"/></build>\n</model>\n"
        );
        let path = temp("threemf_component", "3mf");
        std::fs::write(&path, zip_package(&[("3D/3dmodel.model", xml.as_bytes())], false)).unwrap();
        let imported = import_file(path.to_str().unwrap(), Some(MeshFormat::ThreeMf), 1.0).unwrap();
        assert_eq!(imported.triangles, 1);
        let (lo, hi) = bounds(&imported.positions);
        assert!((lo[0] - 100.0).abs() < 1e-6, "lo={lo:?}");
        assert!((lo[1] - 0.0).abs() < 1e-6, "lo={lo:?}");
        assert!((lo[2] - 10.0).abs() < 1e-6, "lo={lo:?}");
        assert!((hi[0] - 101.0).abs() < 1e-6, "hi={hi:?}");
        assert!((hi[1] - 1.0).abs() < 1e-6, "hi={hi:?}");
        let _ = std::fs::remove_file(path);
    }

    #[test]
    fn three_mf_reads_deflated_zip_entries() {
        let xml = concat!(
            "<model unit=\"millimeter\">\n<resources>\n",
            "<object id=\"1\"><mesh><vertices>\n",
            "<vertex x=\"0\" y=\"0\" z=\"0\"/><vertex x=\"1\" y=\"0\" z=\"0\"/><vertex x=\"0\" y=\"1\" z=\"0\"/>\n",
            "</vertices><triangles><triangle v1=\"0\" v2=\"1\" v3=\"2\"/></triangles></mesh></object>\n",
            "</resources><build><item objectid=\"1\"/></build>\n</model>\n"
        );
        let path = temp("threemf_deflate", "3mf");
        std::fs::write(&path, zip_package(&[("3D/3dmodel.model", xml.as_bytes())], true)).unwrap();
        let imported = import_file(path.to_str().unwrap(), Some(MeshFormat::ThreeMf), 1.0).unwrap();
        assert_eq!(imported.triangles, 1);
        let _ = std::fs::remove_file(path);
    }

    #[test]
    fn inward_winding_is_flipped_outward() {
        let mesh = geo::box_mesh([10.0, 10.0, 10.0]);
        let flipped = triangles(&mesh)
            .into_iter()
            .map(|t| [t[0], t[2], t[1]])
            .collect::<Vec<_>>();
        let cleaned = clean(&mesh.positions, &flipped).unwrap();
        assert!(signed_volume(&cleaned.positions, &cleaned.triangles) > 0.0);
    }

    #[test]
    fn degenerate_and_duplicate_triangles_are_dropped() {
        let positions = [
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
            [2.0, 0.0, 0.0],
        ];
        let triangles = [
            [0, 1, 2],
            [0, 0, 1],       // repeated corner
            [0, 1, 2],       // duplicate
            [1, 2, 3],
            [0, 1, 4],       // collinear but distinct: a seam triangle
        ];
        let cleaned = clean(&positions, &triangles).unwrap();
        assert_eq!(cleaned.triangles.len(), 3, "zero-area seam triangles survive");
        assert_eq!(cleaned.dropped, 2, "repeated-corner and duplicate triangles go");
    }

    #[test]
    fn crease_angle_splits_or_smooths_box_normals() {
        let mesh = geo::box_mesh([10.0, 10.0, 10.0]);
        let (hard, _) = build(&mesh.positions, &mesh.indices, 30.0).unwrap();
        assert_eq!(hard.positions.len(), 24, "six flat faces keep hard edges");
        for n in &hard.normals {
            let axis = n.iter().filter(|v| v.abs() > 0.99).count();
            assert_eq!(axis, 1, "a box face normal is an axis: {n:?}");
        }
        let (soft, _) = build(&mesh.positions, &mesh.indices, 179.0).unwrap();
        assert_eq!(soft.positions.len(), 8, "smoothing shares one normal per corner");
    }

    #[test]
    fn malformed_inputs_have_actionable_errors() {
        let path = temp("bad_obj", "obj");
        std::fs::write(&path, "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 9\n").unwrap();
        let err = import_file(path.to_str().unwrap(), None, 1.0).unwrap_err().to_string();
        assert!(err.contains("out of range"), "{err}");
        let _ = std::fs::remove_file(path);

        let err = build(&[[0.0; 3], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]], &[0, 1], 30.0)
            .unwrap_err()
            .to_string();
        assert!(err.contains("multiple of 3"), "{err}");
    }

    /// Minimal store-or-deflate ZIP writer for the tests. The reader ignores
    /// CRC, so a zero CRC keeps the helper small.
    fn zip_package(entries: &[(&str, &[u8])], deflate: bool) -> Vec<u8> {
        fn u16v(out: &mut Vec<u8>, v: u16) {
            out.extend_from_slice(&v.to_le_bytes());
        }
        fn u32v(out: &mut Vec<u8>, v: u32) {
            out.extend_from_slice(&v.to_le_bytes());
        }
        let mut out = Vec::new();
        let mut central = Vec::new();
        for (name, data) in entries {
            let (method, payload) = if deflate {
                let mut enc = flate2::write::DeflateEncoder::new(
                    Vec::new(),
                    flate2::Compression::default(),
                );
                enc.write_all(data).unwrap();
                (8u16, enc.finish().unwrap())
            } else {
                (0u16, data.to_vec())
            };
            let offset = out.len() as u32;
            u32v(&mut out, 0x0403_4b50);
            u16v(&mut out, 20);
            u16v(&mut out, 0x0800);
            u16v(&mut out, method);
            u16v(&mut out, 0);
            u16v(&mut out, 0x0021);
            u32v(&mut out, 0); // crc (not checked by our reader)
            u32v(&mut out, payload.len() as u32);
            u32v(&mut out, data.len() as u32);
            u16v(&mut out, name.len() as u16);
            u16v(&mut out, 0);
            out.extend_from_slice(name.as_bytes());
            out.extend_from_slice(&payload);

            u32v(&mut central, 0x0201_4b50);
            u16v(&mut central, 20);
            u16v(&mut central, 20);
            u16v(&mut central, 0x0800);
            u16v(&mut central, method);
            u16v(&mut central, 0);
            u16v(&mut central, 0x0021);
            u32v(&mut central, 0);
            u32v(&mut central, payload.len() as u32);
            u32v(&mut central, data.len() as u32);
            u16v(&mut central, name.len() as u16);
            u16v(&mut central, 0);
            u16v(&mut central, 0);
            u16v(&mut central, 0);
            u16v(&mut central, 0);
            u32v(&mut central, 0);
            u32v(&mut central, offset);
            central.extend_from_slice(name.as_bytes());
        }
        let offset = out.len() as u32;
        let size = central.len() as u32;
        out.extend_from_slice(&central);
        u32v(&mut out, 0x0605_4b50);
        u16v(&mut out, 0);
        u16v(&mut out, 0);
        u16v(&mut out, entries.len() as u16);
        u16v(&mut out, entries.len() as u16);
        u32v(&mut out, size);
        u32v(&mut out, offset);
        u16v(&mut out, 0);
        out
    }
}
