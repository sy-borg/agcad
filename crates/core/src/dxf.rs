//! ASCII DXF (R12 / AC1009) export of 2D drawings.
//!
//! R12 is deliberately old and small: every reader (AutoCAD, LibreCAD,
//! QCAD, Inkscape, CAM tools) understands it, and it needs no handles or
//! object dictionaries — only HEADER, TABLES, a BLOCKS section and ENTITIES.
//! Splines have no R12 entity, so they arrive as polylines (the same
//! interpolation the engine uses for profiles). Coordinates are sketch-local
//! millimeters and angles are degrees, counter-clockwise, as DXF expects.
//!
//! Bytes are deterministic: fixed header text, sorted layer order from the
//! drawing model, and `.6` decimal formatting with trailing zeros trimmed.

use crate::Error;
use crate::drawing::{self, CurveKind, Drawing, Role};

const LAYER_0: &str = "0";

fn color(role: Role) -> i32 {
    match role {
        Role::Drawing => 7,      // white / black
        Role::Construction => 8, // dark grey
        Role::Profile => 5,      // blue
    }
}

fn linetype(role: Role) -> &'static str {
    match role {
        Role::Construction => "DASHED",
        _ => "CONTINUOUS",
    }
}

/// Serialize a drawing as DXF text.
pub fn dxf_bytes(drawing: &Drawing) -> Result<Vec<u8>, Error> {
    let (lo, hi) = drawing.bounds().ok_or_else(|| {
        Error::Invalid("nothing to export: the drawing contains no curves".into())
    })?;

    let mut out = String::new();
    let mut emit = |code: i32, value: &str| {
        out.push_str(&code.to_string());
        out.push('\n');
        out.push_str(value);
        out.push('\n');
    };

    // --- HEADER -----------------------------------------------------------------
    emit(0, "SECTION");
    emit(2, "HEADER");
    emit(9, "$ACADVER");
    emit(1, "AC1009");
    emit(9, "$INSUNITS");
    emit(70, "4"); // millimeters
    emit(9, "$EXTMIN");
    emit(10, &drawing::number(lo[0]));
    emit(20, &drawing::number(lo[1]));
    emit(30, "0");
    emit(9, "$EXTMAX");
    emit(10, &drawing::number(hi[0]));
    emit(20, &drawing::number(hi[1]));
    emit(30, "0");
    emit(0, "ENDSEC");

    // --- TABLES: linetypes + layers --------------------------------------------
    emit(0, "SECTION");
    emit(2, "TABLES");

    emit(0, "TABLE");
    emit(2, "LTYPE");
    emit(70, "2");
    emit(0, "LTYPE");
    emit(2, "CONTINUOUS");
    emit(70, "0");
    emit(3, "Solid line");
    emit(72, "65");
    emit(73, "0");
    emit(40, "0");
    emit(0, "LTYPE");
    emit(2, "DASHED");
    emit(70, "0");
    emit(3, "Dashed __ __ __ __");
    emit(72, "65");
    emit(73, "2");
    emit(40, "1.5");
    emit(49, "1");
    emit(49, "-0.5");
    emit(0, "ENDTAB");

    emit(0, "TABLE");
    emit(2, "LAYER");
    emit(70, &(drawing.layers.len() + 1).to_string());
    emit(0, "LAYER");
    emit(2, LAYER_0);
    emit(70, "0");
    emit(62, "7");
    emit(6, "CONTINUOUS");
    for layer in &drawing.layers {
        emit(0, "LAYER");
        emit(2, &layer.name);
        emit(70, "0");
        emit(62, &color(layer.role).to_string());
        emit(6, linetype(layer.role));
    }
    emit(0, "ENDTAB");
    emit(0, "ENDSEC");

    // --- BLOCKS: none, but every DXF flavour expects the section ----------------
    emit(0, "SECTION");
    emit(2, "BLOCKS");
    emit(0, "ENDSEC");

    // --- ENTITIES ---------------------------------------------------------------
    emit(0, "SECTION");
    emit(2, "ENTITIES");
    for curve in &drawing.curves {
        let layer = &drawing.layers[curve.layer].name;
        match &curve.kind {
            CurveKind::Line { a, b } => {
                emit(0, "LINE");
                emit(8, layer);
                emit(10, &drawing::number(a[0]));
                emit(20, &drawing::number(a[1]));
                emit(30, "0");
                emit(11, &drawing::number(b[0]));
                emit(21, &drawing::number(b[1]));
                emit(31, "0");
            }
            CurveKind::Circle { center, radius } => {
                emit(0, "CIRCLE");
                emit(8, layer);
                emit(10, &drawing::number(center[0]));
                emit(20, &drawing::number(center[1]));
                emit(30, "0");
                emit(40, &drawing::number(*radius));
            }
            CurveKind::Arc {
                center,
                radius,
                start_deg,
                sweep_deg,
            } => {
                if *sweep_deg >= 359.999 {
                    emit(0, "CIRCLE");
                    emit(8, layer);
                    emit(10, &drawing::number(center[0]));
                    emit(20, &drawing::number(center[1]));
                    emit(30, "0");
                    emit(40, &drawing::number(*radius));
                } else {
                    let start = drawing::norm360(*start_deg);
                    let end = drawing::norm360(start + sweep_deg.max(0.0));
                    emit(0, "ARC");
                    emit(8, layer);
                    emit(10, &drawing::number(center[0]));
                    emit(20, &drawing::number(center[1]));
                    emit(30, "0");
                    emit(40, &drawing::number(*radius));
                    emit(50, &drawing::number(start));
                    emit(51, &drawing::number(end));
                }
            }
            CurveKind::Polyline { points, closed } => {
                if points.len() < 2 {
                    continue;
                }
                emit(0, "POLYLINE");
                emit(8, layer);
                emit(66, "1"); // vertices follow
                emit(70, if *closed { "1" } else { "0" });
                emit(10, "0");
                emit(20, "0");
                emit(30, "0");
                for p in points {
                    emit(0, "VERTEX");
                    emit(8, layer);
                    emit(10, &drawing::number(p[0]));
                    emit(20, &drawing::number(p[1]));
                    emit(30, "0");
                }
                emit(0, "SEQEND");
                emit(8, layer);
            }
        }
    }
    emit(0, "ENDSEC");
    emit(0, "EOF");

    Ok(out.into_bytes())
}

/// Write a DXF file, returning `(resolved path, bytes)`.
pub fn write_dxf(path: &str, drawing: &Drawing) -> Result<(String, usize), Error> {
    let bytes = dxf_bytes(drawing)?;
    std::fs::write(path, &bytes).map_err(|e| {
        Error::Invalid(format!(
            "cannot write DXF to '{path}': {e} (hint: use an absolute path the server can write)"
        ))
    })?;
    let resolved = std::fs::canonicalize(path)
        .map(|p| p.to_string_lossy().into_owned())
        .unwrap_or_else(|_| path.to_string());
    Ok((resolved, bytes.len()))
}
