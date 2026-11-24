//! SVG export of 2D drawings.
//!
//! The SVG uses the drawing's millimeters as user units: `width`/`height`
//! carry `mm` so a browser or cutter sees the real size, while the viewBox
//! stays in sketch coordinates. SVG's Y axis points down, so the writer
//! flips it (and the arc sweep flag) to keep the drawing upright instead of
//! mirrored. Every layer becomes a group; construction geometry is dashed
//! and profile loops are a separate, filled-none outline.
//!
//! No fonts, timestamps or random ids: identical drawings produce
//! byte-identical files.

use crate::Error;
use crate::drawing::{self, CurveKind, Drawing, Role};

fn escape(s: &str) -> String {
    let mut out = String::with_capacity(s.len());
    for c in s.chars() {
        match c {
            '&' => out.push_str("&amp;"),
            '<' => out.push_str("&lt;"),
            '>' => out.push_str("&gt;"),
            '"' => out.push_str("&quot;"),
            '\'' => out.push_str("&apos;"),
            _ => out.push(c),
        }
    }
    out
}

fn style(role: Role) -> &'static str {
    match role {
        Role::Drawing => {
            r##"class="drawing" fill="none" stroke="#111111" stroke-width="0.2" stroke-linecap="round" stroke-linejoin="round""##
        }
        Role::Construction => {
            r##"class="construction" fill="none" stroke="#888888" stroke-width="0.15" stroke-dasharray="1 0.6" stroke-linecap="round" stroke-linejoin="round""##
        }
        Role::Profile => {
            r##"class="profile" fill="none" stroke="#0057b8" stroke-width="0.25" stroke-linecap="round" stroke-linejoin="round""##
        }
    }
}

/// Serialize a drawing as SVG text.
pub fn svg_bytes(drawing: &Drawing) -> Result<Vec<u8>, Error> {
    let (lo, hi) = drawing.bounds().ok_or_else(|| {
        Error::Invalid("nothing to export: the drawing contains no curves".into())
    })?;

    // A little breathing room, in millimeters, so strokes at the edge are
    // not clipped. The minimum keeps a zero-height drawing visible.
    let pad = ((hi[0] - lo[0]).max(hi[1] - lo[1]) * 0.02).max(1.0);
    let min_x = lo[0] - pad;
    let min_y = lo[1] - pad;
    let width = (hi[0] - lo[0]) + 2.0 * pad;
    let height = (hi[1] - lo[1]) + 2.0 * pad;
    // Flip Y: y_svg = min_y + max_y - y maps the padded box onto itself.
    let flip = |p: [f64; 2]| [p[0], lo[1] + hi[1] - p[1]];
    let n = drawing::number;

    let mut out = String::new();
    out.push_str("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
    out.push_str(&format!(
        "<svg xmlns=\"http://www.w3.org/2000/svg\" version=\"1.1\" width=\"{w}mm\" height=\"{h}mm\" viewBox=\"{x} {y} {w} {h}\">\n",
        w = n(width),
        h = n(height),
        x = n(min_x),
        y = n(min_y),
    ));
    out.push_str(&format!("  <title>{}</title>\n", escape(&drawing.name)));
    out.push_str(
        "  <desc>2D drawing exported by wright - sketch coordinates in millimeters</desc>\n",
    );

    for (li, layer) in drawing.layers.iter().enumerate() {
        out.push_str(&format!(
            "  <g id=\"layer-{}\" data-layer=\"{}\" {}>\n",
            li + 1,
            escape(&layer.name),
            style(layer.role)
        ));
        for curve in drawing.curves.iter().filter(|c| c.layer == li) {
            match &curve.kind {
                CurveKind::Line { a, b } => {
                    let a = flip(*a);
                    let b = flip(*b);
                    out.push_str(&format!(
                        "    <line x1=\"{}\" y1=\"{}\" x2=\"{}\" y2=\"{}\"/>\n",
                        n(a[0]),
                        n(a[1]),
                        n(b[0]),
                        n(b[1])
                    ));
                }
                CurveKind::Circle { center, radius } => {
                    let c = flip(*center);
                    out.push_str(&format!(
                        "    <circle cx=\"{}\" cy=\"{}\" r=\"{}\"/>\n",
                        n(c[0]),
                        n(c[1]),
                        n(*radius)
                    ));
                }
                CurveKind::Arc {
                    center,
                    radius,
                    start_deg,
                    sweep_deg,
                } => {
                    if *sweep_deg >= 359.999 {
                        let c = flip(*center);
                        out.push_str(&format!(
                            "    <circle cx=\"{}\" cy=\"{}\" r=\"{}\"/>\n",
                            n(c[0]),
                            n(c[1]),
                            n(*radius)
                        ));
                        continue;
                    }
                    let sweep = sweep_deg.max(0.0);
                    let start = drawing::point(*center, *radius, *start_deg);
                    let end = drawing::point(*center, *radius, *start_deg + sweep);
                    let a = flip(start);
                    let b = flip(end);
                    // A math-CCW arc becomes visually CCW after the Y flip,
                    // which is SVG's negative-angle direction (sweep-flag 0).
                    let large = if sweep > 180.0 { 1 } else { 0 };
                    out.push_str(&format!(
                        "    <path d=\"M {} {} A {} {} 0 {} 0 {} {}\"/>\n",
                        n(a[0]),
                        n(a[1]),
                        n(*radius),
                        n(*radius),
                        large,
                        n(b[0]),
                        n(b[1])
                    ));
                }
                CurveKind::Polyline { points, closed } => {
                    if points.len() < 2 {
                        continue;
                    }
                    out.push_str("    <path d=\"");
                    for (i, p) in points.iter().enumerate() {
                        let p = flip(*p);
                        out.push_str(if i == 0 { "M " } else { " L " });
                        out.push_str(&n(p[0]));
                        out.push(' ');
                        out.push_str(&n(p[1]));
                    }
                    if *closed {
                        out.push_str(" Z");
                    }
                    out.push_str("\"/>\n");
                }
            }
        }
        out.push_str("  </g>\n");
    }

    out.push_str("</svg>\n");
    Ok(out.into_bytes())
}

/// Write an SVG file, returning `(resolved path, bytes)`.
pub fn write_svg(path: &str, drawing: &Drawing) -> Result<(String, usize), Error> {
    let bytes = svg_bytes(drawing)?;
    std::fs::write(path, &bytes).map_err(|e| {
        Error::Invalid(format!(
            "cannot write SVG to '{path}': {e} (hint: use an absolute path the server can write)"
        ))
    })?;
    let resolved = std::fs::canonicalize(path)
        .map(|p| p.to_string_lossy().into_owned())
        .unwrap_or_else(|_| path.to_string());
    Ok((resolved, bytes.len()))
}
