//! 3MF export: one OPC/ZIP package with a 3MF model containing every
//! selected solid as its own object.
//!
//! Unlike STL, 3MF keeps the things a slicer can use: the build is a list of
//! named objects, each with its own display color, and the model declares
//! its unit (millimeters). The package is written by hand — a store-only
//! ZIP with fixed timestamps — so the same scene exports to bit-identical
//! bytes and the core stays dependency-free.
//!
//! Format: OPC package with three parts — `[Content_Types].xml`,
//! `_rels/.rels` and `3D/3dmodel.model`. Geometry is baked into world space,
//! matching STL export and avoiding slicer-specific transform support.
//! Colors use the 3MF core `basematerials` material group; every object and
//! triangle references its base color.

use std::collections::BTreeMap;
use std::fmt::Write as _;

use agcad_protocol::{Mat4, Mesh, Rgba};

use crate::mat4;
use crate::Error;

/// One printable object for the 3MF build: an entity-local mesh, its world
/// placement, a name, and a display color.
pub struct Part<'a> {
    pub name: &'a str,
    pub mesh: &'a Mesh,
    pub transform: Mat4,
    pub color: Rgba,
}

/// Serialize the given parts into a 3MF package, returning `(bytes,
/// triangles)`. The caller supplies the document name used as the model
/// title.
pub fn threemf_bytes(parts: &[Part<'_>], title: &str) -> (Vec<u8>, usize) {
    let (model, tris) = model_xml(parts, title);
    let content_types = concat!(
        "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n",
        "<Types xmlns=\"http://schemas.openxmlformats.org/package/2006/content-types\">\n",
        "  <Default Extension=\"rels\" ContentType=\"application/vnd.openxmlformats-package.relationships+xml\"/>\n",
        "  <Default Extension=\"model\" ContentType=\"application/vnd.ms-package.3dmanufacturing-3dmodel+xml\"/>\n",
        "</Types>\n",
    );
    let rels = concat!(
        "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n",
        "<Relationships xmlns=\"http://schemas.openxmlformats.org/package/2006/relationships\">\n",
        "  <Relationship Target=\"/3D/3dmodel.model\" Id=\"rel0\" ",
        "Type=\"http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel\"/>\n",
        "</Relationships>\n",
    );
    let bytes = zip(&[
        ("[Content_Types].xml", content_types.as_bytes()),
        ("_rels/.rels", rels.as_bytes()),
        ("3D/3dmodel.model", model.as_bytes()),
    ]);
    (bytes, tris)
}

/// Write a 3MF file, returning `(path, objects, triangles, bytes)`.
pub fn write_3mf(
    path: &str,
    parts: &[Part<'_>],
    title: &str,
) -> Result<(String, usize, usize, usize), Error> {
    let (bytes, tris) = threemf_bytes(parts, title);
    std::fs::write(path, &bytes).map_err(|e| {
        Error::Invalid(format!(
            "cannot write 3MF to '{path}': {e} (hint: use an absolute path the server can write)"
        ))
    })?;
    let resolved = std::fs::canonicalize(path)
        .map(|p| p.to_string_lossy().into_owned())
        .unwrap_or_else(|_| path.to_string());
    Ok((resolved, parts.len(), tris, bytes.len()))
}

/// Build the `3D/3dmodel.model` part, returning `(xml, triangles)`.
fn model_xml(parts: &[Part<'_>], title: &str) -> (String, usize) {
    // Deduplicate colors in first-use order; each object then points at one
    // base in the single `basematerials` group.
    let mut colors: Vec<[u8; 4]> = Vec::new();
    let mut material_of = Vec::with_capacity(parts.len());
    for part in parts {
        let key = color_key(part.color);
        let index = match colors.iter().position(|c| *c == key) {
            Some(i) => i,
            None => {
                colors.push(key);
                colors.len() - 1
            },
        };
        material_of.push(index);
    }

    let mut xml = String::new();
    xml.push_str("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
    xml.push_str(
        "<model unit=\"millimeter\" xml:lang=\"en-US\" \
         xmlns=\"http://schemas.microsoft.com/3dmanufacturing/core/2015/02\">\n",
    );
    xml.push_str("  <metadata name=\"Application\">agcad</metadata>\n");
    xml.push_str("  <metadata name=\"Title\">");
    push_escaped(&mut xml, title);
    xml.push_str("</metadata>\n");
    xml.push_str("  <resources>\n");
    xml.push_str("    <basematerials id=\"1\">\n");
    for (i, c) in colors.iter().enumerate() {
        // 3MF display colors are #RRGGBBAA; alpha is opacity.
        writeln!(
            xml,
            "      <base name=\"Color {}\" displaycolor=\"#{:02X}{:02X}{:02X}{:02X}\"/>",
            i + 1,
            c[0],
            c[1],
            c[2],
            c[3]
        )
        .unwrap();
    }
    xml.push_str("    </basematerials>\n");

    let mut tris = 0usize;
    for (index, part) in parts.iter().enumerate() {
        let object_id = index + 2; // 1 is the basematerials group.
        let material = material_of[index];
        write!(
            xml,
            "    <object id=\"{object_id}\" type=\"model\" name=\""
        )
        .unwrap();
        push_escaped(&mut xml, part.name);
        writeln!(xml, "\" pid=\"1\" pindex=\"{material}\">").unwrap();
        xml.push_str("      <mesh>\n        <vertices>\n");
        let (positions, triangles) = welded_world(part.mesh, &part.transform);
        for w in &positions {
            xml.push_str("          <vertex x=\"");
            push_num(&mut xml, w[0]);
            xml.push_str("\" y=\"");
            push_num(&mut xml, w[1]);
            xml.push_str("\" z=\"");
            push_num(&mut xml, w[2]);
            xml.push_str("\"/>\n");
        }
        xml.push_str("        </vertices>\n        <triangles>\n");
        for tri in triangles.chunks_exact(3) {
            writeln!(
                xml,
                "          <triangle v1=\"{}\" v2=\"{}\" v3=\"{}\" pid=\"1\" p1=\"{material}\"/>",
                tri[0], tri[1], tri[2]
            )
            .unwrap();
            tris += 1;
        }
        xml.push_str("        </triangles>\n      </mesh>\n    </object>\n");
    }
    xml.push_str("  </resources>\n  <build>\n");
    for index in 0..parts.len() {
        writeln!(xml, "    <item objectid=\"{}\"/>", index + 2).unwrap();
    }
    xml.push_str("  </build>\n</model>\n");
    (xml, tris)
}

/// Transform a mesh into world space and weld coincident vertices by exact
/// `f32` bits (the same rule the boolean kernel and edge extractor use).
/// 3MF is an indexed format; welding collapses the per-triangle corner
/// duplicates that crease-normal meshes carry back into shared vertices.
fn welded_world(mesh: &Mesh, transform: &Mat4) -> (Vec<[f32; 3]>, Vec<u32>) {
    let mut ids: BTreeMap<[u32; 3], u32> = BTreeMap::new();
    let mut positions = Vec::new();
    let mut remap: Vec<u32> = Vec::with_capacity(mesh.positions.len());
    for p in &mesh.positions {
        let w = mat4::transform_point(transform, *p);
        let key = [w[0].to_bits(), w[1].to_bits(), w[2].to_bits()];
        let id = match ids.get(&key) {
            Some(&id) => id,
            None => {
                let id = positions.len() as u32;
                ids.insert(key, id);
                positions.push(w);
                id
            },
        };
        remap.push(id);
    }
    let triangles = mesh.indices.iter().map(|&i| remap[i as usize]).collect();
    (positions, triangles)
}

fn color_key(c: Rgba) -> [u8; 4] {
    fn channel(v: f32) -> u8 {
        (v.clamp(0.0, 1.0) * 255.0).round() as u8
    }
    [channel(c[0]), channel(c[1]), channel(c[2]), channel(c[3])]
}

/// Append an XML number. `f32`'s shortest round-trip form is valid in XML
/// (`1e-7` included); normalize negative zero and guard non-finite values.
fn push_num(out: &mut String, v: f32) {
    if !v.is_finite() || v == 0.0 {
        out.push('0');
    } else {
        write!(out, "{v}").unwrap();
    }
}

/// Escape text for XML element content or a double-quoted attribute.
fn push_escaped(out: &mut String, text: &str) {
    for ch in text.chars() {
        match ch {
            '&' => out.push_str("&amp;"),
            '<' => out.push_str("&lt;"),
            '>' => out.push_str("&gt;"),
            '"' => out.push_str("&quot;"),
            '\'' => out.push_str("&apos;"),
            // XML 1.0 forbids most C0 controls; keep names writable.
            c if (c as u32) < 0x20 && c != '\t' && c != '\n' && c != '\r' => out.push(' '),
            c => out.push(c),
        }
    }
}

// --- minimal deterministic ZIP writer -----------------------------------------

/// Fixed DOS timestamp: 1980-01-01 00:00:00. A real clock would make every
/// export byte-different; the timestamp carries no useful information.
const DOS_TIME: u16 = 0;
const DOS_DATE: u16 = 0x0021;

fn zip(entries: &[(&str, &[u8])]) -> Vec<u8> {
    let mut out = Vec::new();
    let mut central = Vec::new();
    for (name, data) in entries {
        let offset = out.len() as u32;
        let crc = crc32(data);
        let size = data.len() as u32;

        put_u32(&mut out, 0x0403_4b50); // local file header
        put_u16(&mut out, 20); // version needed
        put_u16(&mut out, 0x0800); // UTF-8 names
        put_u16(&mut out, 0); // stored, no compression
        put_u16(&mut out, DOS_TIME);
        put_u16(&mut out, DOS_DATE);
        put_u32(&mut out, crc);
        put_u32(&mut out, size);
        put_u32(&mut out, size);
        put_u16(&mut out, name.len() as u16);
        put_u16(&mut out, 0); // extra length
        out.extend_from_slice(name.as_bytes());
        out.extend_from_slice(data);

        put_u32(&mut central, 0x0201_4b50); // central directory header
        put_u16(&mut central, 20); // version made by
        put_u16(&mut central, 20); // version needed
        put_u16(&mut central, 0x0800);
        put_u16(&mut central, 0);
        put_u16(&mut central, DOS_TIME);
        put_u16(&mut central, DOS_DATE);
        put_u32(&mut central, crc);
        put_u32(&mut central, size);
        put_u32(&mut central, size);
        put_u16(&mut central, name.len() as u16);
        put_u16(&mut central, 0); // extra length
        put_u16(&mut central, 0); // comment length
        put_u16(&mut central, 0); // disk number
        put_u16(&mut central, 0); // internal attributes
        put_u32(&mut central, 0); // external attributes
        put_u32(&mut central, offset);
        central.extend_from_slice(name.as_bytes());
    }

    let central_offset = out.len() as u32;
    let central_size = central.len() as u32;
    out.extend_from_slice(&central);
    put_u32(&mut out, 0x0605_4b50); // end of central directory
    put_u16(&mut out, 0); // this disk
    put_u16(&mut out, 0); // disk with central directory
    put_u16(&mut out, entries.len() as u16);
    put_u16(&mut out, entries.len() as u16);
    put_u32(&mut out, central_size);
    put_u32(&mut out, central_offset);
    put_u16(&mut out, 0); // comment length
    out
}

fn put_u16(out: &mut Vec<u8>, v: u16) {
    out.extend_from_slice(&v.to_le_bytes());
}

fn put_u32(out: &mut Vec<u8>, v: u32) {
    out.extend_from_slice(&v.to_le_bytes());
}

const CRC_TABLE: [u32; 256] = {
    let mut table = [0u32; 256];
    let mut i = 0;
    while i < 256 {
        let mut c = i as u32;
        let mut bit = 0;
        while bit < 8 {
            c = if c & 1 != 0 { 0xEDB8_8320 ^ (c >> 1) } else { c >> 1 };
            bit += 1;
        }
        table[i] = c;
        i += 1;
    }
    table
};

fn crc32(data: &[u8]) -> u32 {
    let mut crc = 0xFFFF_FFFFu32;
    for &b in data {
        crc = CRC_TABLE[((crc ^ b as u32) & 0xFF) as usize] ^ (crc >> 8);
    }
    !crc
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geo;

    /// Extract one stored ZIP entry by name (tests only; exports are local
    /// bytes and small, so a linear scan is fine).
    fn entry<'a>(zip: &'a [u8], name: &str) -> &'a [u8] {
        let mut pos = 0usize;
        while pos + 30 <= zip.len() {
            let sig = u32::from_le_bytes(zip[pos..pos + 4].try_into().unwrap());
            if sig != 0x0403_4b50 {
                break;
            }
            let method = u16::from_le_bytes(zip[pos + 8..pos + 10].try_into().unwrap());
            assert_eq!(method, 0, "all entries are stored");
            let size = u32::from_le_bytes(zip[pos + 18..pos + 22].try_into().unwrap()) as usize;
            let name_len = u16::from_le_bytes(zip[pos + 26..pos + 28].try_into().unwrap()) as usize;
            let extra_len = u16::from_le_bytes(zip[pos + 28..pos + 30].try_into().unwrap()) as usize;
            let name_start = pos + 30;
            let data_start = name_start + name_len + extra_len;
            let entry_name = std::str::from_utf8(&zip[name_start..name_start + name_len]).unwrap();
            if entry_name == name {
                return &zip[data_start..data_start + size];
            }
            pos = data_start + size;
        }
        panic!("ZIP entry '{name}' not found");
    }

    fn box_part(name: &str, center: [f32; 3]) -> (Mesh, Mat4, String) {
        let mesh = geo::box_mesh([10.0, 10.0, 10.0]);
        let transform = mat4::translation([center[0] as f64, center[1] as f64, center[2] as f64]);
        (mesh, transform, name.to_string())
    }

    #[test]
    fn package_has_the_three_opc_parts() {
        let (mesh, transform, name) = box_part("cube", [0.0; 3]);
        let parts = [Part { name: &name, mesh: &mesh, transform, color: [1.0, 0.0, 0.0, 1.0] }];
        let (bytes, tris) = threemf_bytes(&parts, "test");
        assert_eq!(tris, 12);
        assert!(bytes.starts_with(b"PK\x03\x04"));
        for required in ["[Content_Types].xml", "_rels/.rels", "3D/3dmodel.model"] {
            assert!(!entry(&bytes, required).is_empty(), "{required} is empty");
        }
        let model = std::str::from_utf8(entry(&bytes, "3D/3dmodel.model")).unwrap();
        assert!(model.contains("unit=\"millimeter\""));
        assert!(model.contains("<object id=\"2\" type=\"model\" name=\"cube\""));
        assert!(model.contains("displaycolor=\"#FF0000FF\""));
        assert_eq!(model.matches("<vertex ").count(), 8, "box corners are welded");
        assert_eq!(model.matches("<triangle ").count(), 12);
    }

    #[test]
    fn each_object_keeps_its_name_and_color() {
        let (red_mesh, red_transform, red_name) =
            box_part("red base", [100.0, 0.0, 0.0]);
        let (gray_mesh, gray_transform, gray_name) =
            box_part("grey boss", [0.0, 0.0, 20.0]);
        let parts = [
            Part {
                name: &red_name,
                mesh: &red_mesh,
                transform: red_transform,
                color: [1.0, 0.0, 0.0, 1.0],
            },
            Part {
                name: &gray_name,
                mesh: &gray_mesh,
                transform: gray_transform,
                color: [0.5, 0.5, 0.5, 0.5],
            },
        ];
        let (bytes, tris) = threemf_bytes(&parts, "two parts");
        assert_eq!(tris, 24);
        let model = std::str::from_utf8(entry(&bytes, "3D/3dmodel.model")).unwrap();
        assert_eq!(model.matches("<object ").count(), 2);
        assert_eq!(model.matches("<item ").count(), 2);
        assert_eq!(model.matches("<base ").count(), 2);
        assert!(model.contains("displaycolor=\"#FF0000FF\""));
        assert!(model.contains("displaycolor=\"#80808080\""));
        assert!(model.contains("name=\"red base\" pid=\"1\" pindex=\"0\""));
        assert!(model.contains("name=\"grey boss\" pid=\"1\" pindex=\"1\""));
        // World-space vertices: the red box sits around x = 100.
        assert!(model.contains("x=\"95\"") && model.contains("x=\"105\""));
        assert!(model.contains("z=\"15\"") && model.contains("z=\"25\""));
    }

    #[test]
    fn identical_scenes_export_identical_bytes() {
        let (mesh, transform, name) = box_part("cube", [1.0, 2.0, 3.0]);
        let parts = [Part { name: &name, mesh: &mesh, transform, color: [0.2, 0.4, 0.6, 1.0] }];
        let a = threemf_bytes(&parts, "same").0;
        let b = threemf_bytes(&parts, "same").0;
        assert_eq!(a, b);
    }

    #[test]
    fn names_and_titles_are_xml_escaped() {
        let (mesh, transform, _name) = box_part("ignored", [0.0; 3]);
        let name = "a & b <c> \"quoted\"";
        let parts = [Part { name, mesh: &mesh, transform, color: [0.0, 0.0, 0.0, 1.0] }];
        let (bytes, _) = threemf_bytes(&parts, "t & t <1>");
        let model = std::str::from_utf8(entry(&bytes, "3D/3dmodel.model")).unwrap();
        assert!(model.contains("name=\"a &amp; b &lt;c&gt; &quot;quoted&quot;\""));
        assert!(model.contains("t &amp; t &lt;1&gt;"));
    }
}
