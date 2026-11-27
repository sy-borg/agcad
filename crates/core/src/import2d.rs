//! 2D import (DXF/SVG) → sketch elements (M22).
//!
//! The importer is deliberately a parser, not a live file reference. It reads
//! a drawing once, converts it to analytic sketch curves, and hands the result
//! to `add_sketch_curves`; the `.wcad` then replays without the source file.
//!
//! Two formats, two very different jobs:
//!
//! * **DXF** (ASCII, R12-friendly) gives exact `LINE`, `CIRCLE`, `ARC` and
//!   polyline entities. `$INSUNITS` converts to millimeters. True splines are
//!   not reconstructed (R12 has none and NURBS control points are not
//!   interpolation points), so they are reported and skipped.
//! * **SVG** is an XML drawing with a Y-down frame. `line`, `circle`,
//!   `ellipse`, `rect`, `polyline`, `polygon` and `path` are supported;
//!   group transforms are composed, and `width`/`height`/`viewBox` give the
//!   millimeter scale. Cubic/quadratic curves are sampled to splines, while
//!   circular arcs stay analytic when the transform is a similarity.
//!
//! Optional inferred constraints make the result behave like authored
//! geometry: near-coincident endpoints are joined, near-axis-aligned lines
//! become exactly horizontal/vertical, and nearly tangent neighbours get a
//! tangency constraint. Interval choice is fixed and deterministic, never
//! based on file order luck.

use std::collections::BTreeMap;

use wright_protocol::{ConstraintKind, SketchConstraint, SketchElement, SketchRef};

use crate::Error;

/// Endpoints closer than this (mm) are treated as the same point. CAD
/// drawings routinely have sub-hundredth gaps where a drafter expected a
/// closed contour.
const COINCIDENT_TOL: f64 = 1e-2;
/// A line within this angle of an axis is snapped by a constraint.
const AXIS_DEG: f64 = 1.0;
/// Neighbouring curves within this angle count as tangent.
const TANGENT_DEG: f64 = 2.0;

/// A curve read from a 2D file, in drawing millimeters.
#[derive(Debug, Clone, PartialEq)]
pub enum Curve2d {
    Line { a: [f64; 2], b: [f64; 2] },
    Circle { center: [f64; 2], radius: f64 },
    /// Counter-clockwise arc from `start_deg` to `end_deg`.
    Arc {
        center: [f64; 2],
        radius: f64,
        start_deg: f64,
        end_deg: f64,
    },
    /// Interpolating spline (the same Catmull-Rom the sketch solver uses).
    Spline { points: Vec<[f64; 2]>, closed: bool },
}

/// The parsed contents of one drawing file.
#[derive(Debug)]
pub struct Imported2d {
    /// File stem, used as the default sketch name.
    pub name: String,
    pub bytes: usize,
    pub curves: Vec<Curve2d>,
    pub warnings: Vec<String>,
}

/// Read a DXF file and convert it to sketch curves.
pub fn import_dxf(path: &str, scale: f64) -> Result<Imported2d, Error> {
    check_scale(scale)?;
    let text = read_text(path, "DXF")?;
    let bytes = text.len();
    let (mut curves, mut warnings, unit_scale) = parse_dxf(&text)?;
    let total = unit_scale * scale;
    if total != 1.0 {
        for curve in &mut curves {
            scale_curve(curve, total);
        }
    }
    let name = file_stem(path);
    if curves.is_empty() {
        return Err(Error::Document(format!(
            "DXF '{path}' contains no supported drawing entities (hint: this importer reads \
             LINE, CIRCLE, ARC and POLYLINE/LWPOLYLINE in the ENTITIES section; blocks/INSERT \
             and SPLINE entities are not reconstructed)"
        )));
    }
    if !warnings.is_empty() {
        warnings.push(format!(
            "imported {} curve(s) at scale {:.4} mm/unit",
            curves.len(),
            total
        ));
    }
    Ok(Imported2d { name, bytes, curves, warnings })
}

/// Read an SVG file and convert it to sketch curves in millimeter,
/// Y-up coordinates.
pub fn import_svg(path: &str, scale: f64) -> Result<Imported2d, Error> {
    check_scale(scale)?;
    let text = read_text(path, "SVG")?;
    let bytes = text.len();
    let (curves, warnings) = parse_svg(&text, scale)?;
    if curves.is_empty() {
        return Err(Error::Document(format!(
            "SVG '{path}' contains no supported drawing shapes (hint: this importer reads line, \
             circle, ellipse, rect, polyline, polygon and path elements; text and images are not \
             geometry)"
        )));
    }
    Ok(Imported2d {
        name: file_stem(path),
        bytes,
        curves,
        warnings,
    })
}

fn check_scale(scale: f64) -> Result<(), Error> {
    if !scale.is_finite() || scale <= 0.0 {
        return Err(Error::Invalid(format!(
            "import scale must be a finite value > 0 (got {scale}; hint: use 25.4 for an \
             inch-unit drawing with no unit metadata)"
        )));
    }
    Ok(())
}

fn read_text(path: &str, what: &str) -> Result<String, Error> {
    std::fs::read_to_string(path).map_err(|e| {
        Error::Document(format!(
            "cannot read {what} '{path}': {e} (hint: use an absolute path the server can read)"
        ))
    })
}

fn file_stem(path: &str) -> String {
    std::path::Path::new(path)
        .file_stem()
        .map(|s| s.to_string_lossy().into_owned())
        .filter(|s| !s.trim().is_empty())
        .unwrap_or_else(|| "drawing".into())
}

fn scale_curve(curve: &mut Curve2d, s: f64) {
    match curve {
        Curve2d::Line { a, b } => {
            *a = [a[0] * s, a[1] * s];
            *b = [b[0] * s, b[1] * s];
        },
        Curve2d::Circle { center, radius } | Curve2d::Arc { center, radius, .. } => {
            *center = [center[0] * s, center[1] * s];
            *radius *= s;
        },
        Curve2d::Spline { points, .. } => {
            for p in points {
                *p = [p[0] * s, p[1] * s];
            }
        },
    }
}

// --- DXF ---------------------------------------------------------------------

/// Parse an ASCII DXF and return `(curves, warnings, unit_scale)`.
fn parse_dxf(text: &str) -> Result<(Vec<Curve2d>, Vec<String>, f64), Error> {
    let pairs = dxf_pairs(text)?;
    let unit_scale = dxf_unit_scale(&pairs);
    let mut curves = Vec::new();
    let mut warnings = Vec::new();

    let mut i = 0usize;
    let mut in_entities = false;
    let mut pending_poly: Option<(bool, Vec<[f64; 2]>)> = None;

    while i < pairs.len() {
        let (code, value) = (&pairs[i].0, &pairs[i].1);
        if *code == 0 && value == "SECTION" && i + 1 < pairs.len() && pairs[i + 1].0 == 2 {
            in_entities = pairs[i + 1].1 == "ENTITIES";
            i += 2;
            continue;
        }
        if !in_entities || *code != 0 {
            i += 1;
            continue;
        }
        if value == "ENDSEC" {
            // An unterminated POLYLINE is still usable: flush it.
            if let Some((closed, points)) = pending_poly.take() {
                emit_polyline(&mut curves, points, closed);
            }
            in_entities = false;
            i += 1;
            continue;
        }

        let kind = value.clone();
        let mut fields: Vec<(i32, String)> = Vec::new();
        i += 1;
        while i < pairs.len() && pairs[i].0 != 0 {
            fields.push(pairs[i].clone());
            i += 1;
        }

        match kind.as_str() {
            "VERTEX" => {
                if let Some((_, points)) = pending_poly.as_mut() {
                    if let Some(p) = dxf_point(&fields) {
                        points.push(p);
                    }
                }
            },
            "SEQEND" => {
                if let Some((closed, points)) = pending_poly.take() {
                    emit_polyline(&mut curves, points, closed);
                }
            },
            _ => {
                if let Some((closed, points)) = pending_poly.take() {
                    emit_polyline(&mut curves, points, closed);
                }
                match kind.as_str() {
                    "LINE" => {
                        if let (Some(a), Some(b)) = (dxf_point(&fields), dxf_second_point(&fields))
                        {
                            push_line(&mut curves, a, b);
                        }
                    },
                    "CIRCLE" => {
                        if let (Some(c), Some(r)) = (dxf_point(&fields), dxf_number(&fields, 40)) {
                            if r > 0.0 {
                                curves.push(Curve2d::Circle { center: c, radius: r });
                            }
                        }
                    },
                    "ARC" => {
                        let c = dxf_point(&fields);
                        let r = dxf_number(&fields, 40);
                        let start = dxf_number(&fields, 50);
                        let end = dxf_number(&fields, 51);
                        match (c, r, start, end) {
                            (Some(c), Some(r), Some(s), Some(e)) if r > 0.0 => {
                                curves.push(Curve2d::Arc {
                                    center: c,
                                    radius: r,
                                    start_deg: s,
                                    end_deg: e,
                                });
                            },
                            _ => warnings.push("skipped an ARC with missing geometry".into()),
                        }
                    },
                    "LWPOLYLINE" => {
                        let mut points = Vec::new();
                        for (code, value) in &fields {
                            match *code {
                                10 => points.push([parse_num(value, "LWPOLYLINE")?, 0.0]),
                                20 => {
                                    if let Some(last) = points.last_mut() {
                                        last[1] = parse_num(value, "LWPOLYLINE")?;
                                    }
                                },
                                _ => {},
                            }
                        }
                        let closed = dxf_flag(&fields, 70) & 1 != 0;
                        emit_polyline(&mut curves, points, closed);
                    },
                    "POLYLINE" => {
                        let closed = dxf_flag(&fields, 70) & 1 != 0;
                        pending_poly = Some((closed, Vec::new()));
                    },
                    "SPLINE" => warnings.push(
                        "skipped a SPLINE entity (hint: explode it to a polyline or use an SVG \
                         path, whose curves are sampled)"
                            .into(),
                    ),
                    "INSERT" => warnings.push(
                        "skipped an INSERT (block references are not expanded; hint: explode \
                         blocks in the source drawing before exporting DXF)"
                            .into(),
                    ),
                    "ELLIPSE" => warnings.push(
                        "skipped an ELLIPSE entity (hint: this importer supports LINE, CIRCLE, \
                         ARC and polyline geometry)"
                            .into(),
                    ),
                    other => warnings.push(format!(
                        "skipped unsupported DXF entity '{other}' (hint: supported are LINE, \
                         CIRCLE, ARC, POLYLINE and LWPOLYLINE)"
                    )),
                }
            },
        }
    }

    if let Some((closed, points)) = pending_poly {
        emit_polyline(&mut curves, points, closed);
    }
    Ok((curves, warnings, unit_scale))
}

fn dxf_pairs(text: &str) -> Result<Vec<(i32, String)>, Error> {
    let mut out = Vec::new();
    let mut lines = text.lines();
    let mut line_no = 0usize;
    while let Some(code_line) = lines.next() {
        line_no += 1;
        let code_text = code_line.trim();
        if code_text.is_empty() {
            continue;
        }
        let Some(value_line) = lines.next() else {
            return Err(Error::Document(format!(
                "DXF ended after group code {code_text} on line {line_no} (hint: group codes and \
                 values must come in pairs)"
            )));
        };
        line_no += 1;
        let code = code_text.parse::<i32>().map_err(|_| {
            Error::Document(format!(
                "DXF line {line_no}: expected an integer group code, got '{code_text}' (hint: \
                 this is an ASCII DXF, not a binary DXF)"
            ))
        })?;
        out.push((code, value_line.trim_end_matches('\r').to_string()));
    }
    Ok(out)
}

fn dxf_unit_scale(pairs: &[(i32, String)]) -> f64 {
    // $INSUNITS: 1 in, 2 ft, 4 mm, 5 cm, 6 m, 7 km. Everything else is
    // treated as already millimeters, the engine's native unit.
    for i in 0..pairs.len().saturating_sub(1) {
        if pairs[i].0 == 9 && pairs[i].1 == "$INSUNITS" {
            let mut j = i + 1;
            while j < pairs.len() && pairs[j].0 != 70 && pairs[j].0 != 9 {
                j += 1;
            }
            if j < pairs.len() && pairs[j].0 == 70 {
                return match pairs[j].1.trim().parse::<i32>().unwrap_or(4) {
                    1 => 25.4,
                    2 => 304.8,
                    4 => 1.0,
                    5 => 10.0,
                    6 => 1000.0,
                    7 => 1_000_000.0,
                    _ => 1.0,
                };
            }
        }
    }
    1.0
}

fn dxf_number(fields: &[(i32, String)], code: i32) -> Option<f64> {
    fields
        .iter()
        .find(|(c, _)| *c == code)
        .and_then(|(_, v)| v.trim().parse::<f64>().ok())
}

fn dxf_flag(fields: &[(i32, String)], code: i32) -> i32 {
    dxf_number(fields, code).unwrap_or(0.0) as i32
}

fn dxf_point(fields: &[(i32, String)]) -> Option<[f64; 2]> {
    let x = dxf_number(fields, 10)?;
    let y = dxf_number(fields, 20).unwrap_or(0.0);
    Some([x, y])
}

fn dxf_second_point(fields: &[(i32, String)]) -> Option<[f64; 2]> {
    let x = dxf_number(fields, 11)?;
    let y = dxf_number(fields, 21).unwrap_or(0.0);
    Some([x, y])
}

fn parse_num(value: &str, what: &str) -> Result<f64, Error> {
    value.trim().parse::<f64>().map_err(|_| {
        Error::Document(format!(
            "DXF {what}: '{value}' is not a number (hint: the file may be truncated or use a \
             decimal comma locale)"
        ))
    })
}

fn push_line(curves: &mut Vec<Curve2d>, a: [f64; 2], b: [f64; 2]) {
    if (a[0] - b[0]).abs() + (a[1] - b[1]).abs() > 1e-9 {
        curves.push(Curve2d::Line { a, b });
    }
}

fn emit_polyline(curves: &mut Vec<Curve2d>, points: Vec<[f64; 2]>, closed: bool) {
    if points.len() < 2 {
        return;
    }
    for pair in points.windows(2) {
        push_line(curves, pair[0], pair[1]);
    }
    if closed {
        let first = *points.first().unwrap();
        let last = *points.last().unwrap();
        if (first[0] - last[0]).abs() + (first[1] - last[1]).abs() > 1e-9 {
            push_line(curves, last, first);
        }
    }
}

// --- SVG ---------------------------------------------------------------------

type Matrix = [f64; 6];

const IDENTITY: Matrix = [1.0, 0.0, 0.0, 1.0, 0.0, 0.0];

fn mat_apply(m: Matrix, p: [f64; 2]) -> [f64; 2] {
    [m[0] * p[0] + m[2] * p[1] + m[4], m[1] * p[0] + m[3] * p[1] + m[5]]
}

fn mat_mul(a: Matrix, b: Matrix) -> Matrix {
    [
        a[0] * b[0] + a[2] * b[1],
        a[1] * b[0] + a[3] * b[1],
        a[0] * b[2] + a[2] * b[3],
        a[1] * b[2] + a[3] * b[3],
        a[0] * b[4] + a[2] * b[5] + a[4],
        a[1] * b[4] + a[3] * b[5] + a[5],
    ]
}

fn mat_scale(m: Matrix) -> f64 {
    (m[0].abs() + m[1].abs() + m[2].abs() + m[3].abs()) * 0.5
}

fn mat_is_identity(m: Matrix) -> bool {
    m.iter().zip(IDENTITY).all(|(a, b)| (a - b).abs() < 1e-12)
}

/// A group transform stack plus the SVG → sketch frame.
struct SvgFrame {
    /// Compose group transformations (SVG coordinates).
    stack: Vec<Matrix>,
    /// File units per viewBox unit, from width/height.
    unit: f64,
    /// ViewBox origin (SVG coordinates).
    vb: [f64; 3],
}

impl SvgFrame {
    fn matrix(&self) -> Matrix {
        *self.stack.last().unwrap_or(&IDENTITY)
    }

    /// Map an SVG point to sketch-local millimeters (Y up).
    fn point(&self, p: [f64; 2]) -> [f64; 2] {
        let q = mat_apply(self.matrix(), p);
        [(q[0] - self.vb[0]) * self.unit, (self.vb[1] + self.vb[2] - q[1]) * self.unit]
    }

    fn radius(&self, r: f64) -> f64 {
        r * mat_scale(self.matrix()) * self.unit
    }

    fn identity(&self) -> bool {
        mat_is_identity(self.matrix())
    }
}

struct Tag {
    name: String,
    attrs: BTreeMap<String, String>,
    closing: bool,
    self_closing: bool,
}

fn parse_svg(text: &str, scale: f64) -> Result<(Vec<Curve2d>, Vec<String>), Error> {
    let tags = svg_tags(text)?;
    let mut width_mm: Option<f64> = None;
    let mut height_mm: Option<f64> = None;
    let mut viewbox: Option<[f64; 4]> = None;
    for tag in &tags {
        if tag.name == "svg" {
            if let Some(w) = tag.attrs.get("width") {
                width_mm = svg_length(w);
            }
            if let Some(h) = tag.attrs.get("height") {
                height_mm = svg_length(h);
            }
            if let Some(vb) = tag.attrs.get("viewBox") {
                viewbox = parse_viewbox(vb);
            }
            break;
        }
    }

    let mut warnings = Vec::new();
    let (unit, vb) = svg_units(width_mm, height_mm, viewbox, &mut warnings);
    let unit = unit * scale;
    let mut frame = SvgFrame { stack: vec![IDENTITY], unit, vb };

    let mut curves = Vec::new();
    let mut skip_depth = 0usize;
    let skipped = |name: &str| {
        matches!(
            name,
            "defs" | "clipPath" | "mask" | "symbol" | "pattern" | "marker" | "style"
                | "title" | "desc" | "metadata"
        )
    };
    for tag in &tags {
        if skipped(&tag.name) {
            if tag.closing {
                skip_depth = skip_depth.saturating_sub(1);
            } else if !tag.self_closing {
                skip_depth += 1;
            }
            continue;
        }
        if skip_depth > 0 {
            continue;
        }
        if tag.name == "g" {
            if tag.closing {
                if frame.stack.len() > 1 {
                    frame.stack.pop();
                }
            } else if !tag.self_closing {
                let local = tag
                    .attrs
                    .get("transform")
                    .map(|t| parse_transform(t))
                    .unwrap_or(IDENTITY);
                let composed = mat_mul(frame.matrix(), local);
                frame.stack.push(composed);
            }
            continue;
        }
        if tag.closing || tag.name == "svg" || tag.name == "defs" || tag.name == "title"
            || tag.name == "desc" || tag.name == "style" || tag.name == "metadata"
        {
            continue;
        }
        match tag.name.as_str() {
            "line" => {
                let Some(a) = svg_point(&tag.attrs, "x1", "y1") else { continue };
                let Some(b) = svg_point(&tag.attrs, "x2", "y2") else { continue };
                push_svg_line(&mut curves, &frame, a, b);
            },
            "circle" => {
                let Some(c) = svg_point(&tag.attrs, "cx", "cy") else { continue };
                let Some(r) = svg_num(&tag.attrs, "r") else { continue };
                push_svg_circle(&mut curves, &frame, c, r);
            },
            "ellipse" => {
                let Some(c) = svg_point(&tag.attrs, "cx", "cy") else { continue };
                let Some(rx) = svg_num(&tag.attrs, "rx") else { continue };
                let Some(ry) = svg_num(&tag.attrs, "ry") else { continue };
                push_svg_ellipse(&mut curves, &frame, c, rx, ry);
            },
            "rect" => {
                let x = svg_num(&tag.attrs, "x").unwrap_or(0.0);
                let y = svg_num(&tag.attrs, "y").unwrap_or(0.0);
                let Some(w) = svg_num(&tag.attrs, "width") else { continue };
                let Some(h) = svg_num(&tag.attrs, "height") else { continue };
                if tag.attrs.contains_key("rx") || tag.attrs.contains_key("ry") {
                    warnings.push(
                        "a rounded rect was imported with square corners (rx/ry are ignored)"
                            .into(),
                    );
                }
                let points = vec![
                    [x, y],
                    [x + w, y],
                    [x + w, y + h],
                    [x, y + h],
                ];
                push_svg_polyline(&mut curves, &frame, &points, true);
            },
            "polyline" | "polygon" => {
                let Some(raw) = tag.attrs.get("points") else { continue };
                let points = parse_points(raw);
                if points.len() >= 2 {
                    push_svg_polyline(&mut curves, &frame, &points, tag.name == "polygon");
                }
            },
            "path" => {
                let Some(d) = tag.attrs.get("d") else { continue };
                match parse_path(d) {
                    Ok(subpaths) => {
                        for sub in subpaths {
                            emit_subpath(&mut curves, &frame, sub);
                        }
                    },
                    Err(message) => warnings.push(format!("skipped a path: {message}")),
                }
            },
            "text" | "image" | "use" => {},
            _ => {},
        }
    }

    Ok((curves, warnings))
}

fn svg_units(
    width: Option<f64>,
    height: Option<f64>,
    viewbox: Option<[f64; 4]>,
    warnings: &mut Vec<String>,
) -> (f64, [f64; 3]) {
    let Some(vb) = viewbox else {
        // No viewBox: coordinates are already in user units. A declared mm
        // width still gives the drawing a real-world scale only when a
        // viewBox ties the two together, so keep 1 unit = 1 mm.
        return (1.0, [0.0, 0.0, 0.0]);
    };
    let (_, _, w, h) = (vb[0], vb[1], vb[2], vb[3]);
    let sx = if w.abs() > 1e-12 { width.map(|v| v / w) } else { None };
    let sy = if h.abs() > 1e-12 { height.map(|v| v / h) } else { None };
    let unit = match (sx, sy) {
        (Some(x), Some(y)) => {
            if (x - y).abs() > 0.02 * x.abs().max(1e-9) {
                warnings.push(format!(
                    "SVG viewBox and width/height disagree on aspect ({x:.6} vs {y:.6}); using \
                     the X scale so circles stay circles"
                ));
            }
            x
        },
        (Some(x), None) => x,
        (None, Some(y)) => y,
        (None, None) => 1.0,
    };
    if !unit.is_finite() || unit <= 0.0 {
        return (1.0, [0.0, 0.0, 0.0]);
    }
    (unit, [vb[0], vb[1], h])
}

fn svg_length(raw: &str) -> Option<f64> {
    let s = raw.trim();
    let (number, unit) = split_unit(s);
    let value = number.parse::<f64>().ok()?;
    Some(match unit {
        "mm" => value,
        "cm" => value * 10.0,
        "m" => value * 1000.0,
        "in" => value * 25.4,
        "pt" => value * 25.4 / 72.0,
        "pc" => value * 25.4 / 6.0,
        "px" | "" => value * 25.4 / 96.0,
        "%" | "em" | "ex" | "vw" | "vh" => return None,
        _ => value,
    })
}

fn split_unit(s: &str) -> (&str, &str) {
    let split = s
        .find(|c: char| c.is_ascii_alphabetic())
        .unwrap_or(s.len());
    (&s[..split], s[split..].trim())
}

fn parse_viewbox(raw: &str) -> Option<[f64; 4]> {
    let parts: Vec<f64> = raw
        .split(|c: char| c == ',' || c.is_whitespace())
        .filter(|p| !p.is_empty())
        .filter_map(|p| p.parse().ok())
        .collect();
    (parts.len() == 4).then(|| [parts[0], parts[1], parts[2], parts[3]])
}

fn svg_num(attrs: &BTreeMap<String, String>, key: &str) -> Option<f64> {
    attrs.get(key).and_then(|v| split_unit(v.trim()).0.parse().ok())
}

fn svg_point(attrs: &BTreeMap<String, String>, x: &str, y: &str) -> Option<[f64; 2]> {
    Some([svg_num(attrs, x)?, svg_num(attrs, y)?])
}

fn parse_points(raw: &str) -> Vec<[f64; 2]> {
    let nums: Vec<f64> = raw
        .split(|c: char| c == ',' || c.is_whitespace())
        .filter(|p| !p.is_empty())
        .filter_map(|p| p.parse().ok())
        .collect();
    nums.chunks_exact(2).map(|p| [p[0], p[1]]).collect()
}

fn push_svg_line(curves: &mut Vec<Curve2d>, frame: &SvgFrame, a: [f64; 2], b: [f64; 2]) {
    push_line(curves, frame.point(a), frame.point(b));
}

fn push_svg_polyline(
    curves: &mut Vec<Curve2d>,
    frame: &SvgFrame,
    points: &[[f64; 2]],
    closed: bool,
) {
    let mapped: Vec<[f64; 2]> = points.iter().map(|p| frame.point(*p)).collect();
    emit_polyline(curves, mapped, closed);
}

fn push_svg_circle(curves: &mut Vec<Curve2d>, frame: &SvgFrame, center: [f64; 2], r: f64) {
    if r <= 0.0 {
        return;
    }
    let m = frame.matrix();
    let a = mat_apply(m, center);
    // Similarity test: uniform scale and perpendicular axes.
    let det = m[0] * m[3] - m[1] * m[2];
    let uniform = (m[0] - m[3]).abs() < 1e-9 && (m[1] + m[2]).abs() < 1e-9;
    if uniform && det > 0.0 {
        curves.push(Curve2d::Circle {
            center: [(a[0] - frame.vb[0]) * frame.unit, (frame.vb[1] + frame.vb[2] - a[1]) * frame.unit],
            radius: r * mat_scale(m) * frame.unit,
        });
    } else {
        let pts: Vec<[f64; 2]> = (0..64)
            .map(|i| {
                let t = i as f64 * std::f64::consts::TAU / 64.0;
                frame.point([center[0] + r * t.cos(), center[1] + r * t.sin()])
            })
            .collect();
        curves.push(Curve2d::Spline { points: pts, closed: true });
    }
}

fn push_svg_ellipse(
    curves: &mut Vec<Curve2d>,
    frame: &SvgFrame,
    center: [f64; 2],
    rx: f64,
    ry: f64,
) {
    if rx <= 0.0 || ry <= 0.0 {
        return;
    }
    let pts: Vec<[f64; 2]> = (0..64)
        .map(|i| {
            let t = i as f64 * std::f64::consts::TAU / 64.0;
            frame.point([center[0] + rx * t.cos(), center[1] + ry * t.sin()])
        })
        .collect();
    curves.push(Curve2d::Spline { points: pts, closed: true });
}

fn svg_tags(text: &str) -> Result<Vec<Tag>, Error> {
    let bytes = text.as_bytes();
    let mut out = Vec::new();
    let mut i = 0usize;
    while i < bytes.len() {
        if bytes[i] != b'<' {
            i += 1;
            continue;
        }
        if text[i..].starts_with("<!--") {
            let end = text[i..].find("-->").map(|p| i + p + 3).unwrap_or(bytes.len());
            i = end;
            continue;
        }
        if text[i..].starts_with("<?") {
            let end = text[i..].find("?>").map(|p| i + p + 2).unwrap_or(bytes.len());
            i = end;
            continue;
        }
        if text[i..].starts_with("<!") {
            let end = text[i..].find('>').map(|p| i + p + 1).unwrap_or(bytes.len());
            i = end;
            continue;
        }
        let mut j = i + 1;
        let mut quote: Option<u8> = None;
        while j < bytes.len() {
            let b = bytes[j];
            if let Some(q) = quote {
                if b == q {
                    quote = None;
                }
            } else if b == b'"' || b == b'\'' {
                quote = Some(b);
            } else if b == b'>' {
                break;
            }
            j += 1;
        }
        if j >= bytes.len() {
            break;
        }
        let raw = &text[i + 1..j];
        let mut inner = raw.trim();
        let self_closing = inner.ends_with('/');
        if self_closing {
            inner = inner[..inner.len() - 1].trim_end();
        }
        let closing = inner.starts_with('/');
        let inner = inner.trim_start_matches('/').trim_start();
        let name_end = inner
            .find(|c: char| c.is_whitespace())
            .unwrap_or(inner.len());
        let name = inner[..name_end].to_string();
        let attrs = parse_attrs(&inner[name_end..]);
        if !name.is_empty() {
            out.push(Tag { name, attrs, closing, self_closing });
        }
        i = j + 1;
    }
    Ok(out)
}

fn parse_attrs(raw: &str) -> BTreeMap<String, String> {
    let mut out = BTreeMap::new();
    let bytes = raw.as_bytes();
    let mut i = 0usize;
    while i < bytes.len() {
        while i < bytes.len() && (bytes[i] as char).is_whitespace() {
            i += 1;
        }
        let start = i;
        while i < bytes.len()
            && !(bytes[i] as char).is_whitespace()
            && bytes[i] != b'='
        {
            i += 1;
        }
        if i == start {
            i += 1;
            continue;
        }
        let key = raw[start..i].to_string();
        while i < bytes.len() && (bytes[i] as char).is_whitespace() {
            i += 1;
        }
        if i >= bytes.len() || bytes[i] != b'=' {
            out.insert(key, String::new());
            continue;
        }
        i += 1;
        while i < bytes.len() && (bytes[i] as char).is_whitespace() {
            i += 1;
        }
        if i >= bytes.len() {
            break;
        }
        let quote = bytes[i];
        if quote == b'"' || quote == b'\'' {
            i += 1;
            let value_start = i;
            while i < bytes.len() && bytes[i] != quote {
                i += 1;
            }
            out.insert(key, raw[value_start..i].to_string());
            if i < bytes.len() {
                i += 1;
            }
        } else {
            let value_start = i;
            while i < bytes.len() && !(bytes[i] as char).is_whitespace() {
                i += 1;
            }
            out.insert(key, raw[value_start..i].to_string());
        }
    }
    out
}

fn parse_transform(raw: &str) -> Matrix {
    let mut out = IDENTITY;
    let mut rest = raw.trim();
    while !rest.is_empty() {
        let Some(open) = rest.find('(') else { break };
        let name = rest[..open].trim();
        let Some(close_rel) = rest[open..].find(')') else { break };
        let close = open + close_rel;
        let args: Vec<f64> = rest[open + 1..close]
            .split(|c: char| c == ',' || c.is_whitespace())
            .filter(|p| !p.is_empty())
            .filter_map(|p| p.parse().ok())
            .collect();
        let local = match name {
            "translate" => [1.0, 0.0, 0.0, 1.0, args.first().copied().unwrap_or(0.0), args.get(1).copied().unwrap_or(0.0)],
            "scale" => {
                let sx = args.first().copied().unwrap_or(1.0);
                let sy = args.get(1).copied().unwrap_or(sx);
                [sx, 0.0, 0.0, sy, 0.0, 0.0]
            },
            "rotate" => {
                let a = args.first().copied().unwrap_or(0.0).to_radians();
                let (cx, cy) = (args.get(1).copied().unwrap_or(0.0), args.get(2).copied().unwrap_or(0.0));
                let r = [a.cos(), a.sin(), -a.sin(), a.cos(), 0.0, 0.0];
                let t1 = [1.0, 0.0, 0.0, 1.0, cx, cy];
                let t2 = [1.0, 0.0, 0.0, 1.0, -cx, -cy];
                mat_mul(mat_mul(t1, r), t2)
            },
            "matrix" => {
                if args.len() >= 6 {
                    [args[0], args[1], args[2], args[3], args[4], args[5]]
                } else {
                    IDENTITY
                }
            },
            _ => IDENTITY,
        };
        out = mat_mul(out, local);
        rest = rest[close + 1..].trim_start();
    }
    out
}

// --- SVG paths ---------------------------------------------------------------

#[derive(Debug)]
enum Piece {
    Line([f64; 2]),
    Arc {
        center: [f64; 2],
        radius: f64,
        start_deg: f64,
        end_deg: f64,
    },
    /// Sampled points, first point = current point. `closed` never set here:
    /// path closure is a separate line.
    Sample(Vec<[f64; 2]>),
}

#[derive(Debug)]
struct SubPath {
    pieces: Vec<Piece>,
    closed: bool,
}

/// Parse an SVG path `d` into subpaths (SVG coordinates, Y down).
fn parse_path(d: &str) -> Result<Vec<SubPath>, String> {
    let tokens = path_tokens(d)?;
    let mut i = 0usize;
    let mut out = Vec::new();
    let mut current = [0.0, 0.0];
    let mut start = [0.0, 0.0];
    let mut pieces: Vec<Piece> = Vec::new();
    let mut last_ctrl: Option<[f64; 2]> = None;
    let mut started = false;

    let flush = |pieces: &mut Vec<Piece>, closed: bool, out: &mut Vec<SubPath>| {
        if !pieces.is_empty() {
            out.push(SubPath { pieces: std::mem::take(pieces), closed });
        }
    };

    while i < tokens.len() {
        let Token::Command(cmd) = tokens[i] else {
            return Err("path data starts with a number, not a command".into());
        };
        i += 1;
        let relative = cmd.is_ascii_lowercase();
        let upper = cmd.to_ascii_uppercase();
        let mut first = true;
        let mut any = false;
        loop {
            let need = match upper {
                'M' | 'L' | 'T' => 2,
                'H' | 'V' => 1,
                'C' => 6,
                'S' | 'Q' => 4,
                'A' => 7,
                'Z' => 0,
                _ => return Err(format!("unsupported path command '{cmd}'")),
            };
            if upper == 'Z' {
                if !pieces.is_empty() {
                    if !points_equal(current, start) {
                        pieces.push(Piece::Line(start));
                    }
                    flush(&mut pieces, true, &mut out);
                    current = start;
                }
                any = true;
                break;
            }
            if i + need > tokens.len() {
                if first && !any {
                    break;
                }
                return Err(format!("path command '{cmd}' is missing coordinates"));
            }
            let mut args = [0.0f64; 7];
            for (k, slot) in args.iter_mut().enumerate().take(need) {
                match tokens[i + k] {
                    Token::Number(v) => *slot = v,
                    Token::Command(_) => {
                        return Err(format!("path command '{cmd}' is missing coordinates"));
                    },
                }
            }
            i += need;

            let point = |p: [f64; 2]| {
                if relative {
                    [current[0] + p[0], current[1] + p[1]]
                } else {
                    p
                }
            };
            match upper {
                'M' => {
                    let p = point([args[0], args[1]]);
                    flush(&mut pieces, false, &mut out);
                    current = p;
                    start = p;
                    started = true;
                    // Subsequent coordinate pairs after M are implicit L.
                    loop {
                        if i + 2 > tokens.len() {
                            break;
                        }
                        let Token::Number(x) = tokens[i] else { break };
                        let Token::Number(y) = tokens[i + 1] else { break };
                        let q = if relative { [current[0] + x, current[1] + y] } else { [x, y] };
                        pieces.push(Piece::Line(q));
                        current = q;
                        i += 2;
                    }
                    any = true;
                    break;
                },
                'L' => {
                    let p = point([args[0], args[1]]);
                    pieces.push(Piece::Line(p));
                    current = p;
                },
                'H' => {
                    let x = if relative { current[0] + args[0] } else { args[0] };
                    let p = [x, current[1]];
                    pieces.push(Piece::Line(p));
                    current = p;
                },
                'V' => {
                    let y = if relative { current[1] + args[0] } else { args[0] };
                    let p = [current[0], y];
                    pieces.push(Piece::Line(p));
                    current = p;
                },
                'C' | 'S' | 'Q' | 'T' => {
                    let (c1, c2, end) = match upper {
                        'C' => (
                            point([args[0], args[1]]),
                            point([args[2], args[3]]),
                            point([args[4], args[5]]),
                        ),
                        'S' => {
                            let c1 = last_ctrl.map_or(current, |c| {
                                [2.0 * current[0] - c[0], 2.0 * current[1] - c[1]]
                            });
                            (c1, point([args[0], args[1]]), point([args[2], args[3]]))
                        },
                        'Q' => (
                            point([args[0], args[1]]),
                            point([args[0], args[1]]),
                            point([args[2], args[3]]),
                        ),
                        _ => {
                            let c = last_ctrl.map_or(current, |c| {
                                [2.0 * current[0] - c[0], 2.0 * current[1] - c[1]]
                            });
                            (c, c, point([args[0], args[1]]))
                        },
                    };
                    let samples = if upper == 'C' || upper == 'S' {
                        sample_cubic(current, c1, c2, end)
                    } else {
                        sample_quadratic(current, c1, end)
                    };
                    let ctrl = if upper == 'C' || upper == 'S' { c2 } else { c1 };
                    pieces.push(Piece::Sample(samples));
                    last_ctrl = Some(ctrl);
                    current = end;
                },
                'A' => {
                    let end = point([args[5], args[6]]);
                    let (rx, ry, rot, large, sweep) =
                        (args[0], args[1], args[2], args[3] != 0.0, args[4] != 0.0);
                    if let Some(arc) = svg_arc(current, end, rx, ry, rot, large, sweep) {
                        pieces.push(arc);
                    } else {
                        pieces.push(Piece::Line(end));
                    }
                    current = end;
                },
                _ => unreachable!(),
            }
            any = true;
            first = false;
            // Commands without an explicit repeat stop when the next token is
            // a command (or input ends).
            if i >= tokens.len() || matches!(tokens[i], Token::Command(_)) {
                break;
            }
        }
        if !any && is_command(cmd) {
            break;
        }
    }

    if started || !pieces.is_empty() {
        flush(&mut pieces, false, &mut out);
    }
    Ok(out)
}

fn is_command(c: char) -> bool {
    "MmLlHhVvCcSsQqTtAaZz".contains(c)
}

fn points_equal(a: [f64; 2], b: [f64; 2]) -> bool {
    (a[0] - b[0]).abs() < 1e-9 && (a[1] - b[1]).abs() < 1e-9
}

enum Token {
    Command(char),
    Number(f64),
}

fn path_tokens(d: &str) -> Result<Vec<Token>, String> {
    let bytes = d.as_bytes();
    let mut out = Vec::new();
    let mut i = 0usize;
    while i < bytes.len() {
        let b = bytes[i] as char;
        if b.is_whitespace() || b == ',' {
            i += 1;
        } else if b.is_ascii_alphabetic() {
            if !is_command(b) {
                return Err(format!("unsupported path command '{b}'"));
            }
            out.push(Token::Command(b));
            i += 1;
        } else {
            let start = i;
            if b == '+' || b == '-' {
                i += 1;
            }
            while i < bytes.len() && (bytes[i] as char).is_ascii_digit() {
                i += 1;
            }
            if i < bytes.len() && bytes[i] == b'.' {
                i += 1;
                while i < bytes.len() && (bytes[i] as char).is_ascii_digit() {
                    i += 1;
                }
            }
            if i < bytes.len() && (bytes[i] == b'e' || bytes[i] == b'E') {
                i += 1;
                if i < bytes.len() && (bytes[i] == b'+' || bytes[i] == b'-') {
                    i += 1;
                }
                while i < bytes.len() && (bytes[i] as char).is_ascii_digit() {
                    i += 1;
                }
            }
            let text = &d[start..i];
            let v = text
                .parse::<f64>()
                .map_err(|_| format!("'{text}' is not a path number"))?;
            out.push(Token::Number(v));
        }
    }
    Ok(out)
}

fn sample_cubic(p0: [f64; 2], p1: [f64; 2], p2: [f64; 2], p3: [f64; 2]) -> Vec<[f64; 2]> {
    let mut out = Vec::with_capacity(13);
    for k in 0..=12 {
        let t = k as f64 / 12.0;
        let u = 1.0 - t;
        out.push([
            u * u * u * p0[0] + 3.0 * u * u * t * p1[0] + 3.0 * u * t * t * p2[0] + t * t * t * p3[0],
            u * u * u * p0[1] + 3.0 * u * u * t * p1[1] + 3.0 * u * t * t * p2[1] + t * t * t * p3[1],
        ]);
    }
    out
}

fn sample_quadratic(p0: [f64; 2], p1: [f64; 2], p2: [f64; 2]) -> Vec<[f64; 2]> {
    let mut out = Vec::with_capacity(13);
    for k in 0..=12 {
        let t = k as f64 / 12.0;
        let u = 1.0 - t;
        out.push([
            u * u * p0[0] + 2.0 * u * t * p1[0] + t * t * p2[0],
            u * u * p0[1] + 2.0 * u * t * p1[1] + t * t * p2[1],
        ]);
    }
    out
}

/// Convert an SVG endpoint arc to a center arc, or `None` for a degenerate
/// (zero-radius) arc, which the caller treats as a line.
fn svg_arc(
    p1: [f64; 2],
    p2: [f64; 2],
    rx: f64,
    ry: f64,
    rotation_deg: f64,
    large_arc: bool,
    sweep: bool,
) -> Option<Piece> {
    let (mut rx, mut ry) = (rx.abs(), ry.abs());
    if rx < 1e-12 || ry < 1e-12 {
        return None;
    }
    if points_equal(p1, p2) {
        return None;
    }
    let phi = rotation_deg.to_radians();
    let (sin_phi, cos_phi) = (phi.sin(), phi.cos());
    let dx = (p1[0] - p2[0]) / 2.0;
    let dy = (p1[1] - p2[1]) / 2.0;
    let x1p = cos_phi * dx + sin_phi * dy;
    let y1p = -sin_phi * dx + cos_phi * dy;
    let lambda = x1p * x1p / (rx * rx) + y1p * y1p / (ry * ry);
    if lambda > 1.0 {
        let s = lambda.sqrt();
        rx *= s;
        ry *= s;
    }
    let num = (rx * rx * ry * ry - rx * rx * y1p * y1p - ry * ry * x1p * x1p).max(0.0);
    let den = rx * rx * y1p * y1p + ry * ry * x1p * x1p;
    let coef = if den > 0.0 { (num / den).sqrt() } else { 0.0 };
    let sign = if large_arc != sweep { -1.0 } else { 1.0 };
    let cxp = sign * coef * (rx * y1p / ry);
    let cyp = sign * coef * (-ry * x1p / rx);
    let cx = cos_phi * cxp - sin_phi * cyp + (p1[0] + p2[0]) / 2.0;
    let cy = sin_phi * cxp + cos_phi * cyp + (p1[1] + p2[1]) / 2.0;

    let angle = |u: [f64; 2], v: [f64; 2]| -> f64 {
        let cross = u[0] * v[1] - u[1] * v[0];
        let dot = u[0] * v[0] + u[1] * v[1];
        cross.atan2(dot)
    };
    let u = [(x1p - cxp) / rx, (y1p - cyp) / ry];
    let v = [(-x1p - cxp) / rx, (-y1p - cyp) / ry];
    let theta1 = angle([1.0, 0.0], u);
    let mut dtheta = angle(u, v);
    if !sweep && dtheta > 0.0 {
        dtheta -= std::f64::consts::TAU;
    }
    if sweep && dtheta < 0.0 {
        dtheta += std::f64::consts::TAU;
    }

    if (rx - ry).abs() < 1e-9 && rotation_deg.abs() < 1e-9 {
        Some(Piece::Arc {
            center: [cx, cy],
            radius: rx,
            start_deg: theta1.to_degrees(),
            end_deg: (theta1 + dtheta).to_degrees(),
        })
    } else {
        let steps = ((dtheta.abs() / std::f64::consts::TAU) * 64.0).ceil().max(8.0) as usize;
        let mut pts = Vec::with_capacity(steps + 1);
        for k in 0..=steps {
            let t = theta1 + dtheta * k as f64 / steps as f64;
            pts.push([cx + rx * t.cos(), cy + ry * t.sin()]);
        }
        Some(Piece::Sample(pts))
    }
}

fn emit_subpath(curves: &mut Vec<Curve2d>, frame: &SvgFrame, sub: SubPath) {
    // Build the chain in sketch-local millimeters so lines, arcs and sampled
    // splines share one coordinate space.
    let mut prev: Option<[f64; 2]> = None;
    let mut items = Vec::new();
    for piece in &sub.pieces {
        match piece {
            Piece::Line(p) => {
                let q = frame.point(*p);
                if let Some(a) = prev {
                    if !points_equal(a, q) {
                        items.push(Item::Line(a, q));
                    }
                }
                prev = Some(q);
            },
            Piece::Arc {
                center,
                radius,
                start_deg,
                end_deg,
            } => {
                let sweep = ccw_sweep(*start_deg, *end_deg);
                let a = frame.point(arc_pt(*center, *radius, *start_deg));
                let b = frame.point(arc_pt(*center, *radius, *start_deg + sweep));
                if let Some(p) = prev {
                    if !points_equal(p, a) {
                        items.push(Item::Line(p, a));
                    }
                }
                if frame.identity() {
                    let sa = -start_deg;
                    let sb = -(start_deg + sweep);
                    // SVG increasing angle becomes decreasing after the Y
                    // flip; choose the CCW ordering that spans the same arc.
                    let (lo, hi) = if sb > sa { (sa, sb) } else { (sb, sa) };
                    items.push(Item::Arc {
                        center: frame.point(*center),
                        radius: frame.radius(*radius),
                        start_deg: lo,
                        end_deg: hi,
                    });
                } else {
                    let steps = ((sweep.abs().to_radians() / std::f64::consts::TAU) * 64.0)
                        .ceil()
                        .max(8.0) as usize;
                    let pts: Vec<[f64; 2]> = (0..=steps)
                        .map(|k| {
                            let t = start_deg + sweep * k as f64 / steps as f64;
                            frame.point(arc_pt(*center, *radius, t))
                        })
                        .collect();
                    items.push(Item::Spline(pts));
                }
                prev = Some(b);
            },
            Piece::Sample(points) => {
                if points.is_empty() {
                    continue;
                }
                let mapped: Vec<[f64; 2]> = points.iter().map(|p| frame.point(*p)).collect();
                if let Some(p) = prev {
                    if !points_equal(p, mapped[0]) {
                        items.push(Item::Line(p, mapped[0]));
                    }
                }
                if mapped.len() >= 3 {
                    items.push(Item::Spline(mapped.clone()));
                } else if mapped.len() == 2 {
                    items.push(Item::Line(mapped[0], mapped[1]));
                }
                prev = mapped.last().copied();
            },
        }
    }
    emit_items(curves, &items, sub.closed);
}

fn ccw_sweep(start: f64, end: f64) -> f64 {
    let mut s = (end % 360.0) - (start % 360.0);
    if s < 0.0 {
        s += 360.0;
    }
    if s <= 1e-9 {
        360.0
    } else {
        s
    }
}

fn arc_pt(center: [f64; 2], radius: f64, deg: f64) -> [f64; 2] {
    let t = deg.to_radians();
    [center[0] + radius * t.cos(), center[1] + radius * t.sin()]
}

enum Item {
    Line([f64; 2], [f64; 2]),
    Arc {
        center: [f64; 2],
        radius: f64,
        start_deg: f64,
        end_deg: f64,
    },
    Spline(Vec<[f64; 2]>),
}

fn emit_items(curves: &mut Vec<Curve2d>, items: &[Item], closed: bool) {
    let mut prev: Option<[f64; 2]> = None;
    let mut first: Option<[f64; 2]> = None;
    for item in items {
        match item {
            Item::Line(a, b) => {
                push_line(curves, *a, *b);
                first.get_or_insert(*a);
                prev = Some(*b);
            },
            Item::Arc {
                center,
                radius,
                start_deg,
                end_deg,
            } => {
                let sweep = ccw_sweep(*start_deg, *end_deg);
                let a = arc_pt(*center, *radius, *start_deg);
                let b = arc_pt(*center, *radius, *start_deg + sweep);
                if let Some(p) = prev {
                    if !points_equal(p, a) {
                        push_line(curves, p, a);
                    }
                }
                curves.push(Curve2d::Arc {
                    center: *center,
                    radius: *radius,
                    start_deg: *start_deg,
                    end_deg: *start_deg + sweep,
                });
                first.get_or_insert(a);
                prev = Some(b);
            },
            Item::Spline(points) => {
                if let Some(p) = prev {
                    if let Some(a) = points.first() {
                        if !points_equal(p, *a) {
                            push_line(curves, p, *a);
                        }
                    }
                }
                if points.len() >= 3 {
                    curves.push(Curve2d::Spline { points: points.clone(), closed: false });
                } else if points.len() == 2 {
                    push_line(curves, points[0], points[1]);
                }
                first.get_or_insert(points[0]);
                prev = points.last().copied();
            },
        }
    }
    if closed {
        if let (Some(a), Some(b)) = (prev, first) {
            if !points_equal(a, b) {
                push_line(curves, a, b);
            }
        }
    }
}

// --- sketch conversion + inferred constraints --------------------------------

/// Convert curves to sketch elements with 1-based ids.
pub fn to_elements(curves: &[Curve2d]) -> Vec<SketchElement> {
    let mut out = Vec::with_capacity(curves.len());
    let mut id = 1u64;
    for curve in curves {
        match curve {
            Curve2d::Line { a, b } => {
                if points_equal(*a, *b) {
                    continue;
                }
                out.push(SketchElement::Line {
                    id,
                    p1: *a,
                    p2: *b,
                    construction: false,
                });
                id += 1;
            },
            Curve2d::Circle { center, radius } => {
                if *radius <= 0.0 {
                    continue;
                }
                out.push(SketchElement::Circle {
                    id,
                    center: *center,
                    radius: *radius,
                    construction: false,
                });
                id += 1;
            },
            Curve2d::Arc {
                center,
                radius,
                start_deg,
                end_deg,
            } => {
                if *radius <= 0.0 || (end_deg - start_deg).abs() < 1e-9 {
                    continue;
                }
                out.push(SketchElement::Arc {
                    id,
                    center: *center,
                    radius: *radius,
                    start_deg: *start_deg,
                    end_deg: *end_deg,
                    construction: false,
                });
                id += 1;
            },
            Curve2d::Spline { points, closed } => {
                if points.len() < 2 || (*closed && points.len() < 3) {
                    continue;
                }
                let unique = dedupe_points(points, *closed);
                if unique.len() < 2 {
                    continue;
                }
                out.push(SketchElement::Spline {
                    id,
                    points: unique,
                    closed: *closed,
                    construction: false,
                });
                id += 1;
            },
        }
    }
    out
}

fn dedupe_points(points: &[[f64; 2]], closed: bool) -> Vec<[f64; 2]> {
    let mut out: Vec<[f64; 2]> = Vec::with_capacity(points.len());
    for p in points {
        if out.last().is_some_and(|l| points_equal(*l, *p)) {
            continue;
        }
        out.push(*p);
    }
    if closed && out.len() >= 3 && points_equal(out[0], *out.last().unwrap()) {
        out.pop();
    }
    out
}

#[derive(Clone, Copy)]
struct Endpoint {
    element: u64,
    point: u8,
    pos: [f64; 2],
}

/// Infer constraints for imported geometry: coincident endpoints,
/// horizontal/vertical lines and approximate tangency.
pub fn auto_constraints(elements: &[SketchElement]) -> Vec<SketchConstraint> {
    let mut constraints = Vec::new();
    let mut next = 1u64;
    let mut push = |kind: ConstraintKind, refs: Vec<SketchRef>, value: Option<f64>| {
        constraints.push(SketchConstraint { id: next, kind, refs, value });
        next += 1;
    };

    let endpoints = endpoints_of(elements);

    // Coincident: sort by X and union nearby endpoints, then link each group
    // in a star. The star is enough to keep the loop closed and avoids an
    // O(n²) pile of redundant equations.
    let mut order: Vec<usize> = (0..endpoints.len()).collect();
    order.sort_by(|&a, &b| {
        endpoints[a]
            .pos
            .partial_cmp(&endpoints[b].pos)
            .unwrap_or(std::cmp::Ordering::Equal)
    });
    let mut parent: Vec<usize> = (0..endpoints.len()).collect();
    fn find(parent: &mut [usize], i: usize) -> usize {
        let mut r = i;
        while parent[r] != r {
            r = parent[r];
        }
        let mut c = i;
        while parent[c] != c {
            let n = parent[c];
            parent[c] = r;
            c = n;
        }
        r
    }
    for oi in 0..order.len() {
        let i = order[oi];
        for &j in order.iter().skip(oi + 1) {
            let dx = endpoints[j].pos[0] - endpoints[i].pos[0];
            if dx > COINCIDENT_TOL {
                break;
            }
            let dy = endpoints[j].pos[1] - endpoints[i].pos[1];
            if dy.abs() <= COINCIDENT_TOL {
                let (a, b) = (find(&mut parent, i), find(&mut parent, j));
                if a != b {
                    parent[a] = b;
                }
            }
        }
    }
    let mut groups: BTreeMap<usize, Vec<usize>> = BTreeMap::new();
    for i in 0..endpoints.len() {
        let r = find(&mut parent, i);
        groups.entry(r).or_default().push(i);
    }
    for members in groups.values() {
        if members.len() < 2 {
            continue;
        }
        let first = endpoints[members[0]];
        for &m in &members[1..] {
            let other = endpoints[m];
            push(
                ConstraintKind::Coincident,
                vec![
                    SketchRef { element: first.element, point: Some(first.point) },
                    SketchRef { element: other.element, point: Some(other.point) },
                ],
                None,
            );
        }
    }

    // Axis-aligned lines. A line is horizontal when its unit direction's Y
    // component is within sin(AXIS_DEG).
    let sin_axis = AXIS_DEG.to_radians().sin();
    for e in elements {
        if let SketchElement::Line { id, p1, p2, construction: false } = e {
            let d = [p2[0] - p1[0], p2[1] - p1[1]];
            let len = (d[0] * d[0] + d[1] * d[1]).sqrt();
            if len < 1e-9 {
                continue;
            }
            let nx = d[0] / len;
            let ny = d[1] / len;
            if ny.abs() <= sin_axis {
                push(ConstraintKind::Horizontal, vec![SketchRef { element: *id, point: None }], None);
            } else if nx.abs() <= sin_axis {
                push(ConstraintKind::Vertical, vec![SketchRef { element: *id, point: None }], None);
            }
        }
    }

    // Approximate tangency between neighbouring lines/arcs.
    let cos_tan = TANGENT_DEG.to_radians().cos();
    for members in groups.values() {
        for a in 0..members.len() {
            for b in (a + 1)..members.len() {
                let ea = endpoints[members[a]];
                let eb = endpoints[members[b]];
                let Some(ta) = tangent_at(elements, ea) else { continue };
                let Some(tb) = tangent_at(elements, eb) else { continue };
                let dot = (ta[0] * tb[0] + ta[1] * tb[1]).abs();
                let la = (ta[0] * ta[0] + ta[1] * ta[1]).sqrt();
                let lb = (tb[0] * tb[0] + tb[1] * tb[1]).sqrt();
                if la < 1e-12 || lb < 1e-12 {
                    continue;
                }
                if dot / (la * lb) >= cos_tan {
                    push(
                        ConstraintKind::Tangent,
                        vec![
                            SketchRef { element: ea.element, point: None },
                            SketchRef { element: eb.element, point: None },
                        ],
                        None,
                    );
                }
            }
        }
    }

    constraints
}

fn endpoints_of(elements: &[SketchElement]) -> Vec<Endpoint> {
    let mut out = Vec::new();
    for e in elements {
        match e {
            SketchElement::Line { id, p1, p2, construction: false } => {
                out.push(Endpoint { element: *id, point: 0, pos: *p1 });
                out.push(Endpoint { element: *id, point: 1, pos: *p2 });
            },
            SketchElement::Arc {
                id,
                center,
                radius,
                start_deg,
                end_deg,
                construction: false,
            } => {
                out.push(Endpoint {
                    element: *id,
                    point: 1,
                    pos: arc_pt(*center, *radius, *start_deg),
                });
                out.push(Endpoint {
                    element: *id,
                    point: 2,
                    pos: arc_pt(*center, *radius, *start_deg + ccw_sweep(*start_deg, *end_deg)),
                });
            },
            SketchElement::Spline { id, points, closed: false, construction: false } => {
                if points.len() >= 2 {
                    out.push(Endpoint { element: *id, point: 0, pos: points[0] });
                    out.push(Endpoint {
                        element: *id,
                        point: (points.len() - 1) as u8,
                        pos: *points.last().unwrap(),
                    });
                }
            },
            _ => {},
        }
    }
    out
}

fn tangent_at(elements: &[SketchElement], endpoint: Endpoint) -> Option<[f64; 2]> {
    let e = elements.iter().find(|e| element_id(e) == endpoint.element)?;
    match e {
        SketchElement::Line { p1, p2, .. } => Some([p2[0] - p1[0], p2[1] - p1[1]]),
        SketchElement::Arc { center, radius, start_deg, end_deg, .. } => {
            let angle = if endpoint.point == 1 {
                *start_deg
            } else {
                *start_deg + ccw_sweep(*start_deg, *end_deg)
            };
            let t = angle.to_radians();
            let _ = center;
            Some([-radius * t.sin(), radius * t.cos()])
        },
        _ => None,
    }
}

fn element_id(e: &SketchElement) -> u64 {
    match e {
        SketchElement::Line { id, .. }
        | SketchElement::Circle { id, .. }
        | SketchElement::Arc { id, .. }
        | SketchElement::Spline { id, .. } => *id,
    }
}

/// Closed profile loop count for import reporting.
pub fn closed_loops(elements: &[SketchElement]) -> usize {
    crate::sketch::profile_loops(elements).map(|l| l.len()).unwrap_or(0)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn dxf_pairs_reject_a_truncated_file() {
        let err = dxf_pairs("0\nSECTION\n2").unwrap_err().to_string();
        assert!(err.contains("group codes and values"), "{err}");
    }

    #[test]
    fn parse_dxf_reads_lines_arcs_and_closed_polylines() {
        let dxf = "\
0\nSECTION\n2\nHEADER\n9\n$INSUNITS\n70\n4\n0\nENDSEC\n\
0\nSECTION\n2\nENTITIES\n\
0\nLINE\n10\n0\n20\n0\n11\n10\n21\n0\n\
0\nCIRCLE\n10\n5\n20\n5\n40\n2\n\
0\nARC\n10\n5\n20\n5\n40\n2\n50\n0\n51\n90\n\
0\nLWPOLYLINE\n70\n1\n10\n0\n20\n0\n10\n5\n20\n0\n10\n5\n20\n5\n0\nENDSEC\n0\nEOF\n";
        let (curves, warnings, scale) = parse_dxf(dxf).unwrap();
        assert_eq!(scale, 1.0);
        assert_eq!(curves.len(), 1 + 1 + 1 + 3);
        assert!(warnings.is_empty());
    }

    #[test]
    fn svg_path_arc_becomes_a_ccw_sketch_arc() {
        // A semicircle drawn clockwise in SVG (sweep=0) from (0,0) to (2,0)
        // around (1,0) arrives Y-up as the upper half.
        let (curves, _) = parse_svg(
            r#"<svg width="10mm" height="10mm" viewBox="0 0 10 10"><path d="M0 0 A1 1 0 0 0 2 0"/></svg>"#,
            1.0,
        )
        .unwrap();
        let arc = curves
            .iter()
            .find_map(|c| match c {
                Curve2d::Arc { start_deg, end_deg, .. } => Some((*start_deg, *end_deg)),
                _ => None,
            })
            .expect("arc");
        // Map to sketch: SVG (0,0) is the viewBox bottom-left → (0,10), and
        // (2,0) → (2,10); the arc bulges upward in the drawing, i.e. away
        // from the chord in -v? The exact angles matter less than the sweep.
        let sweep = ccw_sweep(arc.0, arc.1);
        assert!((sweep - 180.0).abs() < 1e-6, "sweep {sweep}");
    }

    #[test]
    fn auto_constraints_snap_an_almost_rectangular_loop() {
        let elements = to_elements(&[
            Curve2d::Line { a: [0.0, 0.0], b: [10.0, 0.005] },
            Curve2d::Line { a: [10.0, 0.005], b: [10.0, 5.0] },
            Curve2d::Line { a: [10.0, 5.0], b: [0.0, 5.0] },
            Curve2d::Line { a: [0.0, 5.0], b: [0.0, 0.0] },
        ]);
        let constraints = auto_constraints(&elements);
        assert!(constraints.iter().any(|c| c.kind == ConstraintKind::Horizontal));
        assert!(constraints.iter().any(|c| c.kind == ConstraintKind::Vertical));
        assert_eq!(
            constraints.iter().filter(|c| c.kind == ConstraintKind::Coincident).count(),
            4
        );
        assert_eq!(closed_loops(&elements), 1);
    }
}
