//! A small 2D drawing model shared by the DXF and SVG writers.
//!
//! Sketches are inherently 2D: their element coordinates live in the sketch
//! plane's (u, v) frame, and the entity transform only places that frame in
//! 3D. A 2D export therefore keeps the sketch-local coordinates instead of
//! projecting to world XYZ — a face sketch on a vertical wall would otherwise
//! collapse to a line.
//!
//! Curves carry a layer index so both writers can group by intent: plain
//! drawing geometry, construction geometry (dashed), and closed profile loops
//! (the exact loops an extrude/revolve consumes).

/// What a layer means, which drives the DXF color/linetype and SVG styling.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Role {
    /// Solved sketch geometry.
    Drawing,
    /// Construction geometry: solved and exported, but not part of a profile.
    Construction,
    /// A closed profile loop.
    Profile,
}

/// One named layer. Names are sanitized for DXF on creation.
#[derive(Debug, Clone)]
pub struct Layer {
    pub name: String,
    pub role: Role,
}

/// A curve in sketch-local millimeters.
#[derive(Debug, Clone)]
pub enum CurveKind {
    Line {
        a: [f64; 2],
        b: [f64; 2],
    },
    Circle {
        center: [f64; 2],
        radius: f64,
    },
    /// Counter-clockwise arc from `start_deg` through `sweep_deg` (degrees).
    Arc {
        center: [f64; 2],
        radius: f64,
        start_deg: f64,
        sweep_deg: f64,
    },
    /// Open or closed polyline. A closed polyline does not repeat its first
    /// point at the end.
    Polyline {
        points: Vec<[f64; 2]>,
        closed: bool,
    },
}

#[derive(Debug, Clone)]
pub struct Curve {
    pub layer: usize,
    pub kind: CurveKind,
}

/// Everything a 2D writer needs, in deterministic entity order.
#[derive(Debug, Clone)]
pub struct Drawing {
    pub name: String,
    pub layers: Vec<Layer>,
    pub curves: Vec<Curve>,
}

impl Drawing {
    pub fn new(name: String) -> Self {
        Self {
            name,
            layers: Vec::new(),
            curves: Vec::new(),
        }
    }

    /// Add a layer (sanitized and de-duplicated) and return its index.
    pub fn layer(&mut self, name: &str, role: Role) -> usize {
        let base = sanitize_layer(name);
        let mut candidate = base.clone();
        let mut suffix = 2;
        while self
            .layers
            .iter()
            .any(|l| l.name.eq_ignore_ascii_case(&candidate))
        {
            candidate = format!("{base}_{suffix}");
            suffix += 1;
        }
        self.layers.push(Layer {
            name: candidate,
            role,
        });
        self.layers.len() - 1
    }

    /// Tight bounds of every curve, in drawing coordinates. Returns `None`
    /// when the drawing is empty.
    pub fn bounds(&self) -> Option<([f64; 2], [f64; 2])> {
        let mut lo = [f64::MAX; 2];
        let mut hi = [f64::MIN; 2];
        let mut grown = false;
        let mut grow = |p: [f64; 2]| {
            grown = true;
            for i in 0..2 {
                lo[i] = lo[i].min(p[i]);
                hi[i] = hi[i].max(p[i]);
            }
        };

        for curve in &self.curves {
            match &curve.kind {
                CurveKind::Line { a, b } => {
                    grow(*a);
                    grow(*b);
                }
                CurveKind::Circle { center, radius } => {
                    grow([center[0] - radius, center[1] - radius]);
                    grow([center[0] + radius, center[1] + radius]);
                }
                CurveKind::Arc {
                    center,
                    radius,
                    start_deg,
                    sweep_deg,
                } => {
                    // Endpoints plus any cardinal direction inside the sweep:
                    // extrema of a circle always sit on one of them.
                    let start = norm360(*start_deg);
                    let sweep = sweep_deg.max(0.0);
                    grow(point(*center, *radius, start));
                    grow(point(*center, *radius, start + sweep));
                    for k in 0..=4 {
                        let a = k as f64 * 90.0;
                        if norm360(a - start) <= sweep + 1e-9 {
                            grow(point(*center, *radius, a));
                        }
                    }
                }
                CurveKind::Polyline { points, .. } => {
                    for p in points {
                        grow(*p);
                    }
                }
            }
        }

        grown.then_some((lo, hi))
    }
}

/// A point on a circle at `deg` (counter-clockwise from +u).
pub fn point(center: [f64; 2], radius: f64, deg: f64) -> [f64; 2] {
    let t = deg.to_radians();
    [center[0] + radius * t.cos(), center[1] + radius * t.sin()]
}

/// Normalize degrees into `[0, 360)`.
pub fn norm360(mut a: f64) -> f64 {
    a %= 360.0;
    if a < 0.0 {
        a += 360.0;
    }
    a
}

/// Format a coordinate deterministically with trailing zeros trimmed.
/// Normalizes `-0` so identical geometry always produces identical bytes.
pub fn number(v: f64) -> String {
    let v = if v == 0.0 { 0.0 } else { v };
    let mut s = format!("{v:.6}");
    if s.contains('.') {
        while s.ends_with('0') {
            s.pop();
        }
        if s.ends_with('.') {
            s.pop();
        }
    }
    if s == "-0" {
        s = "0".to_string();
    }
    s
}

/// DXF symbol names allow letters, digits, spaces and `$ _ -`; everything
/// else becomes `_` so a sketch called `bracket <hot>` cannot break a file.
/// Names are capped to leave room for the role suffix and de-dup counter.
fn sanitize_layer(name: &str) -> String {
    let mut out: String = name
        .chars()
        .map(|c| {
            if c.is_ascii_alphanumeric() || matches!(c, ' ' | '-' | '_') {
                c
            } else {
                '_'
            }
        })
        .take(200)
        .collect();
    while out.ends_with(' ') {
        out.pop();
    }
    while out.starts_with(' ') {
        out.remove(0);
    }
    if out.is_empty() {
        out.push_str("layer");
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn layer_names_are_sanitized_and_unique() {
        let mut d = Drawing::new("doc".into());
        let a = d.layer("bracket <hot>", Role::Drawing);
        let b = d.layer("bracket <hot>", Role::Profile);
        // Both sanitize to the same base, so the second is de-duplicated.
        assert_eq!(d.layers[a].name, "bracket _hot_");
        assert_eq!(d.layers[b].name, "bracket _hot__2");
        let c = d.layer("  ", Role::Drawing);
        assert_eq!(d.layers[c].name, "layer");
        let zero = d.layer("-0.000001", Role::Drawing);
        assert!(d.layers[zero].name.starts_with('-'));
        assert!(!d.layers[zero].name.contains('<'));
    }

    #[test]
    fn bounds_are_exact_for_arcs() {
        let mut d = Drawing::new("doc".into());
        let l = d.layer("arc", Role::Drawing);
        // A quarter arc from 0° to 90° is exactly [0,1] × [0,1].
        d.curves.push(Curve {
            layer: l,
            kind: CurveKind::Arc {
                center: [0.0, 0.0],
                radius: 1.0,
                start_deg: 0.0,
                sweep_deg: 90.0,
            },
        });
        let (lo, hi) = d.bounds().unwrap();
        assert!((lo[0] - 0.0).abs() < 1e-12 && (lo[1] - 0.0).abs() < 1e-12);
        assert!((hi[0] - 1.0).abs() < 1e-12 && (hi[1] - 1.0).abs() < 1e-12);
    }

    #[test]
    fn number_is_stable_and_has_no_negative_zero() {
        assert_eq!(number(0.0), "0");
        assert_eq!(number(-0.0), "0");
        assert_eq!(number(1.5), "1.5");
        assert_eq!(number(10.0), "10");
        assert_eq!(number(1.2345678), "1.234568");
    }
}
