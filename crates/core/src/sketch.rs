//! 2D sketches: constraint-solved drawings that live inside an entity.
//!
//! Elements (lines, circles, arcs, splines) hold the current coordinates;
//! constraints declare relationships the solver must satisfy. Solving is a
//! Levenberg-Marquardt-flavored Gauss-Newton over all element variables —
//! pure f64, fixed iteration order, no RNG: **same sketch state + same
//! constraints → same solution**, which is what parametric replay depends
//! on.
//!
//! Constraints are equations over the variable vector; the Jacobian is
//! computed numerically (forward differences, fixed step) which keeps the
//! code uniform across constraint kinds. Residuals are in sketch units
//! (mm) — or degrees for the angle residual — so thresholds translate
//! directly to geometric accuracy.
//!
//! Constraint equation counts (used for the DOF report):
//! coincident/concentric/symmetric 2 · horizontal/vertical/parallel/
//! perpendicular/distance/radius/diameter/equal/angle/point_on_line/tangent 1.
//!
//! Construction elements are solved and displayed like any other, but they
//! are excluded from the closed profile that extrude/revolve consume.

use std::collections::BTreeMap;

use wright_protocol::{ConstraintKind, Mesh, SketchConstraint, SketchElement, SketchRef};

/// Solve until the worst residual is below this (mm). 1e-7 mm is far
/// below any manufacturing tolerance; the regularizer below sets the
/// practical floor around 1e-8.
const TOL: f64 = 1e-7;
/// Numerical Jacobian step.
const H: f64 = 1e-7;
/// Hard cap on LM outer iterations.
const MAX_ITERS: usize = 200;
/// Levenberg damping bounds.
const LAMBDA_MIN: f64 = 1e-12;
const LAMBDA_MAX: f64 = 1e12;
/// Small geometric epsilon for tessellation and intersection tests.
const EPS: f64 = 1e-9;
/// A constraint above this residual is reported as unsatisfied.
const UNSAT_TOL: f64 = 1e-5;

/// Outcome of a solve pass, mirrored into snapshots.
#[derive(Debug, Clone, PartialEq)]
pub struct SolveReport {
    pub solved: bool,
    pub residual: f64,
    pub dof: i64,
    /// The equations are redundant (rank deficient) even if they solve.
    pub over_constrained: bool,
    /// Unsatisfied or redundant constraints, named by id.
    pub diagnostics: Vec<String>,
}

impl SolveReport {
    fn clean(solved: bool, residual: f64, dof: i64) -> Self {
        Self { solved, residual, dof, over_constrained: false, diagnostics: Vec::new() }
    }
}

// --- element helpers ---------------------------------------------------------

pub fn element_id(e: &SketchElement) -> u64 {
    match e {
        SketchElement::Line { id, .. }
        | SketchElement::Circle { id, .. }
        | SketchElement::Arc { id, .. }
        | SketchElement::Spline { id, .. } => *id,
    }
}

pub fn is_construction(e: &SketchElement) -> bool {
    match e {
        SketchElement::Line { construction, .. }
        | SketchElement::Circle { construction, .. }
        | SketchElement::Arc { construction, .. }
        | SketchElement::Spline { construction, .. } => *construction,
    }
}

fn is_line(e: &SketchElement) -> bool {
    matches!(e, SketchElement::Line { .. })
}

fn is_curve(e: &SketchElement) -> bool {
    matches!(e, SketchElement::Circle { .. } | SketchElement::Arc { .. })
}

fn label(e: &SketchElement) -> &'static str {
    match e {
        SketchElement::Line { .. } => "line",
        SketchElement::Circle { .. } => "circle",
        SketchElement::Arc { .. } => "arc",
        SketchElement::Spline { .. } => "spline",
    }
}

fn find<'a>(elements: &'a [SketchElement], id: u64) -> Option<&'a SketchElement> {
    elements.iter().find(|e| element_id(e) == id)
}

// --- public constraint API ---------------------------------------------------

/// Validate a constraint against the sketch's current elements (arity,
/// ref shapes, target type, value presence). Returns a hint on failure.
pub fn validate(
    kind: ConstraintKind,
    elements: &[SketchElement],
    refs: &[SketchRef],
    value: Option<f64>,
) -> Result<(), String> {
    let is_line = |r: &SketchRef| matches!(find(elements, r.element), Some(e) if is_line(e));
    let is_curve = |r: &SketchRef| matches!(find(elements, r.element), Some(e) if is_curve(e));

    // Resolve refs first: "unknown element 7" is far more useful than
    // "needs a whole-element ref to a line" when the id is simply wrong.
    if let Some(bad) = refs.iter().find(|r| find(elements, r.element).is_none()) {
        return Err(format!(
            "sketch has no element {} (hint: ids come from add_sketch_line/circle/arc/spline; get_scene lists them)",
            bad.element
        ));
    }
    // Point refs must actually name a point on that element.
    for r in refs {
        let Some(e) = find(elements, r.element) else { continue };
        if let Some(p) = r.point {
            let ok = match e {
                SketchElement::Line { .. } => p <= 1,
                SketchElement::Circle { .. } => p == 0,
                SketchElement::Arc { .. } => p <= 2,
                SketchElement::Spline { points, .. } => (p as usize) < points.len(),
            };
            if !ok {
                return Err(format!(
                    "element {} ({}) has no point {p} (hint: line/arc points are 0=start,1=end; \
                     arc 0=center,1=start,2=end; circle 0=center; spline i=point i)",
                    r.element,
                    label(e)
                ));
            }
        }
    }

    let count_points = refs.iter().filter(|r| r.point.is_some()).count();
    let count_whole = refs.iter().filter(|r| r.point.is_none()).count();

    match kind {
        ConstraintKind::Coincident => {
            if refs.len() != 2 || count_points != 2 {
                return Err(
                    "coincident needs exactly 2 point refs (element ids with point: line 0/1, \
                     arc 1/2, circle/spline point)"
                        .into(),
                );
            }
        },
        ConstraintKind::Distance => {
            if refs.len() != 2 {
                return Err("distance needs exactly 2 refs: two points, or one point and one whole line".into());
            }
            let two_points = count_points == 2;
            let point_line = count_points == 1 && count_whole == 1 && refs.iter().any(|r| r.point.is_none() && is_line(r));
            if !two_points && !point_line {
                return Err(
                    "distance needs two point refs, or one point ref and one whole line ref"
                        .into(),
                );
            }
            check_len_value(kind, value)?;
        },
        ConstraintKind::Horizontal | ConstraintKind::Vertical => {
            if refs.len() != 1 || count_whole != 1 || !is_line(&refs[0]) {
                return Err(format!(
                    "{} needs exactly 1 whole-element ref to a line (omit 'point')",
                    kind.as_str()
                ));
            }
        },
        ConstraintKind::Parallel | ConstraintKind::Perpendicular | ConstraintKind::Angle => {
            if refs.len() != 2 || count_whole != 2 || !refs.iter().all(is_line) {
                return Err(format!(
                    "{} needs exactly 2 whole-element refs to lines (omit 'point')",
                    kind.as_str()
                ));
            }
            if kind == ConstraintKind::Angle {
                check_angle_value(value)?;
            }
        },
        ConstraintKind::Radius | ConstraintKind::Diameter => {
            if refs.len() != 1 || !is_curve(&refs[0]) {
                return Err(format!(
                    "{} needs exactly 1 ref to a circle or arc element",
                    kind.as_str()
                ));
            }
            check_len_value(kind, value)?;
        },
        ConstraintKind::Equal => {
            if refs.len() != 2 || count_whole != 2 {
                return Err("equal needs exactly 2 whole-element refs".into());
            }
            let both_lines = refs.iter().all(is_line);
            let both_curves = refs.iter().all(is_curve);
            if !both_lines && !both_curves {
                return Err("equal compares two lines (their lengths) or two circles/arcs (their radii); not a mix".into());
            }
        },
        ConstraintKind::Concentric => {
            if refs.len() != 2 || count_whole != 2 || !refs.iter().all(is_curve) {
                return Err("concentric needs exactly 2 circle/arc refs".into());
            }
        },
        ConstraintKind::Symmetric => {
            if refs.len() != 3 || count_points != 2 || count_whole != 1 || !is_line(&refs[2]) {
                return Err(
                    "symmetric needs refs [point, point, whole line] (the line is the mirror axis)"
                        .into(),
                );
            }
        },
        ConstraintKind::PointOnLine => {
            if refs.len() != 2 || count_points != 1 || count_whole != 1 || !refs.iter().any(|r| r.point.is_none() && is_line(r)) {
                return Err("point_on_line needs one point ref and one whole line ref".into());
            }
        },
        ConstraintKind::Tangent => {
            if refs.len() != 2 || count_whole != 2 {
                return Err("tangent needs exactly 2 whole-element refs".into());
            }
            let line_curve = (is_line(&refs[0]) && is_curve(&refs[1]))
                || (is_line(&refs[1]) && is_curve(&refs[0]));
            let curve_curve = is_curve(&refs[0]) && is_curve(&refs[1]);
            if !line_curve && !curve_curve {
                return Err("tangent needs a line and a circle/arc, or two circles/arcs".into());
            }
        },
    }
    Ok(())
}

fn check_len_value(kind: ConstraintKind, value: Option<f64>) -> Result<(), String> {
    let v = value.ok_or_else(|| {
        format!("{} needs a numeric 'value' (mm; may be an expression over parameters)", kind.as_str())
    })?;
    if !(v.is_finite()) || v < 0.0 {
        return Err(format!("{} value must be a finite, non-negative length (got {v})", kind.as_str()));
    }
    Ok(())
}

fn check_angle_value(value: Option<f64>) -> Result<(), String> {
    let v = value.ok_or_else(|| {
        "angle needs a numeric 'value' in degrees (may be an expression over parameters)".to_string()
    })?;
    if !v.is_finite() {
        return Err(format!("angle value must be finite (got {v})"));
    }
    Ok(())
}

// --- solver ------------------------------------------------------------------

/// Solve the sketch in place. On success the element coordinates are
/// updated to the solution; on failure they are left at the best point found
/// and the report names the constraints that block convergence.
pub fn solve(elements: &mut [SketchElement], constraints: &[SketchConstraint]) -> SolveReport {
    // Flatten element state into a variable vector:
    // line → (p1x, p1y, p2x, p2y), circle → (cx, cy, r),
    // arc → (cx, cy, r, start_deg, end_deg), spline → (x0, y0, x1, y1, …).
    let mut bases = Vec::with_capacity(elements.len());
    let mut n = 0usize;
    for e in elements.iter() {
        bases.push(n);
        n += match e {
            SketchElement::Line { .. } => 4,
            SketchElement::Circle { .. } => 3,
            SketchElement::Arc { .. } => 5,
            SketchElement::Spline { points, .. } => points.len() * 2,
        };
    }

    let mut x: Vec<f64> = Vec::with_capacity(n);
    for e in elements.iter() {
        match e {
            SketchElement::Line { p1, p2, .. } => x.extend_from_slice(&[p1[0], p1[1], p2[0], p2[1]]),
            SketchElement::Circle { center, radius, .. } => x.extend_from_slice(&[center[0], center[1], *radius]),
            SketchElement::Arc { center, radius, start_deg, end_deg, .. } => {
                x.extend_from_slice(&[center[0], center[1], *radius, *start_deg, *end_deg]);
            },
            SketchElement::Spline { points, .. } => {
                for p in points {
                    x.extend_from_slice(p);
                }
            },
        }
    }

    let eqs: i64 = constraints.iter().map(|c| c.kind.equations()).sum();
    let dof = n as i64 - eqs;

    if constraints.is_empty() {
        return SolveReport::clean(true, 0.0, dof);
    }

    let ctx = Ctx::new(&bases, constraints, elements);

    // Weak regularization toward the drawn positions. Constraint systems
    // are usually under-determined (and direction constraints are
    // scale-free), so without this the solver is free to translate, rotate
    // and stretch the drawing arbitrarily — technically a solution, but a
    // useless one. μ is tiny: it only breaks ties, it does not fight the
    // constraints.
    const MU: f64 = 1e-10;
    let x0 = x.clone();
    let merit = |x: &[f64]| -> f64 {
        let r = ctx.residuals(x);
        let rr: f64 = r.iter().map(|v| v * v).sum();
        let dd: f64 = x.iter().zip(&x0).map(|(a, b)| (a - b) * (a - b)).sum();
        rr + MU * dd
    };

    // Levenberg damping: start tiny, relax on success, tighten on failure.
    let mut lambda = 1e-9f64;
    let mut converged = false;
    for _ in 0..MAX_ITERS {
        let r = ctx.residuals(&x);
        let norm = r.iter().fold(0.0f64, |m, v| m.max(v.abs()));
        if norm < TOL {
            converged = true;
            break;
        }

        let j = numeric_jacobian(|v| ctx.residuals(v), &x, n);
        let jtj = mat_mul_t(&j); // JᵀJ
        let jtr = mat_t_vec(&j, &r); // Jᵀr
        let m0 = merit(&x);

        let mut accepted = false;
        while lambda <= LAMBDA_MAX {
            // Damped + regularized normal equations:
            //   (JᵀJ + λ·diag + μ·I) δ = −Jᵀr − μ(x − x0)
            let mut a = jtj.clone();
            let mut b: Vec<f64> = (0..n).map(|i| -jtr[i] - MU * (x[i] - x0[i])).collect();
            for i in 0..n {
                a[i][i] += lambda * jtj[i][i].abs().max(1e-12) + MU;
            }
            if let Some(delta) = solve_linear(&mut a, &mut b) {
                let x_try: Vec<f64> = x.iter().zip(&delta).map(|(xi, d)| xi + d).collect();
                let m1 = merit(&x_try);
                if m1.is_finite() && m1 < m0 {
                    x = x_try;
                    lambda = (lambda / 3.0).max(LAMBDA_MIN);
                    accepted = true;
                    break;
                }
            }
            lambda *= 10.0;
        }
        if !accepted {
            break;
        }
    }

    let residual = ctx.max_residual(&x);
    let solved = converged || residual < TOL;
    let (over_constrained, diagnostics) = diagnostics(&ctx, &x, solved, n, dof);

    // Always publish the best point found, even when it falls short of
    // TOL: the report says the sketch is unsolved, and the drawing should
    // show the closest configuration rather than silently stale geometry.
    for (e, base) in elements.iter_mut().zip(&bases) {
        match e {
            SketchElement::Line { p1, p2, .. } => {
                *p1 = [x[*base], x[base + 1]];
                *p2 = [x[base + 2], x[base + 3]];
            },
            SketchElement::Circle { center, radius, .. } => {
                *center = [x[*base], x[base + 1]];
                *radius = x[base + 2];
            },
            SketchElement::Arc { center, radius, start_deg, end_deg, .. } => {
                *center = [x[*base], x[base + 1]];
                *radius = x[base + 2];
                *start_deg = x[base + 3];
                *end_deg = x[base + 4];
            },
            SketchElement::Spline { points, .. } => {
                for (i, p) in points.iter_mut().enumerate() {
                    *p = [x[*base + 2 * i], x[*base + 2 * i + 1]];
                }
            },
        }
    }

    SolveReport {
        solved,
        residual: if solved { residual.min(TOL) } else { residual },
        dof,
        over_constrained,
        diagnostics,
    }
}

/// Context tying the variable vector back to element/constraint layout.
struct Ctx<'a> {
    bases: &'a [usize],
    constraints: &'a [SketchConstraint],
    elements: &'a [SketchElement],
    /// For circle-circle tangency, whether the initial drawing was internally
    /// tangent. Decided once so the residual does not switch branches mid-solve.
    tangent_internal: BTreeMap<u64, bool>,
}

impl<'a> Ctx<'a> {
    fn new(
        bases: &'a [usize],
        constraints: &'a [SketchConstraint],
        elements: &'a [SketchElement],
    ) -> Self {
        let mut tangent_internal = BTreeMap::new();
        for c in constraints {
            if c.kind != ConstraintKind::Tangent || c.refs.len() != 2 {
                continue;
            }
            let (Some(a), Some(b)) = (find(elements, c.refs[0].element), find(elements, c.refs[1].element))
            else {
                continue;
            };
            if let (Some((ca, ra)), Some((cb, rb))) = (center_radius(a), center_radius(b)) {
                let d = ((ca[0] - cb[0]).powi(2) + (ca[1] - cb[1]).powi(2)).sqrt();
                let internal = (d - (ra - rb).abs()).abs() < (d - (ra + rb)).abs();
                tangent_internal.insert(c.id, internal);
            }
        }
        Self { bases, constraints, elements, tangent_internal }
    }

    fn base_of(&self, id: u64) -> Option<usize> {
        self.elements
            .iter()
            .position(|e| element_id(e) == id)
            .map(|idx| self.bases[idx])
    }

    fn element_of(&self, id: u64) -> Option<&SketchElement> {
        self.elements.iter().find(|e| element_id(e) == id)
    }

    /// Point value of a ref, in sketch coordinates. Arc endpoints are derived
    /// from center/radius/angle; spline refs name control points directly.
    fn point(&self, r: &SketchRef, x: &[f64]) -> Option<[f64; 2]> {
        let base = self.base_of(r.element)?;
        let e = self.element_of(r.element)?;
        match (e, r.point) {
            (SketchElement::Line { .. }, Some(0)) => Some([x[base], x[base + 1]]),
            (SketchElement::Line { .. }, Some(1)) => Some([x[base + 2], x[base + 3]]),
            (SketchElement::Circle { .. }, Some(0) | None) => Some([x[base], x[base + 1]]),
            (SketchElement::Arc { .. }, Some(0) | None) => Some([x[base], x[base + 1]]),
            (SketchElement::Arc { .. }, Some(1)) => Some(arc_point(x[base], x[base + 1], x[base + 2], x[base + 3])),
            (SketchElement::Arc { .. }, Some(2)) => Some(arc_point(x[base], x[base + 1], x[base + 2], x[base + 4])),
            (SketchElement::Spline { points, .. }, Some(i)) if (i as usize) < points.len() => {
                Some([x[base + 2 * i as usize], x[base + 2 * i as usize + 1]])
            },
            _ => None,
        }
    }

    fn center(&self, r: &SketchRef, x: &[f64]) -> Option<[f64; 2]> {
        let base = self.base_of(r.element)?;
        match self.element_of(r.element)? {
            SketchElement::Circle { .. } | SketchElement::Arc { .. } => Some([x[base], x[base + 1]]),
            _ => None,
        }
    }

    fn radius(&self, r: &SketchRef, x: &[f64]) -> Option<f64> {
        let base = self.base_of(r.element)?;
        match self.element_of(r.element)? {
            SketchElement::Circle { .. } | SketchElement::Arc { .. } => Some(x[base + 2]),
            _ => None,
        }
    }

    /// Direction of a line element (length not normalized).
    fn line_dir(&self, r: &SketchRef, x: &[f64]) -> Option<[f64; 2]> {
        let base = self.base_of(r.element)?;
        match self.element_of(r.element)? {
            SketchElement::Line { .. } => {
                Some([x[base + 2] - x[base], x[base + 3] - x[base + 1]])
            },
            _ => None,
        }
    }

    fn is_line_ref(&self, r: &SketchRef) -> bool {
        matches!(self.element_of(r.element), Some(e) if is_line(e))
    }

    fn is_curve_ref(&self, r: &SketchRef) -> bool {
        matches!(self.element_of(r.element), Some(e) if is_curve(e))
    }

    fn constraint_residuals(&self, c: &SketchConstraint, x: &[f64]) -> Vec<f64> {
        let mut out = Vec::with_capacity(c.kind.equations() as usize);
        match c.kind {
            ConstraintKind::Coincident => {
                if let (Some(a), Some(b)) = (c.refs.first().and_then(|r| self.point(r, x)), c.refs.get(1).and_then(|r| self.point(r, x))) {
                    out.push(a[0] - b[0]);
                    out.push(a[1] - b[1]);
                }
            },
            ConstraintKind::Distance => {
                if c.refs.len() == 2 {
                    let target = c.value.unwrap_or(0.0);
                    let (pa, pb) = (self.point(&c.refs[0], x), self.point(&c.refs[1], x));
                    match (pa, pb) {
                        (Some(a), Some(b)) => {
                            out.push(((b[0] - a[0]).powi(2) + (b[1] - a[1]).powi(2)).sqrt() - target);
                        },
                        _ => {
                            // Point-to-line distance.
                            let (point_ref, line_ref) = if c.refs[0].point.is_some() {
                                (&c.refs[0], &c.refs[1])
                            } else {
                                (&c.refs[1], &c.refs[0])
                            };
                            if let (Some(p), Some(d)) = (self.point(point_ref, x), self.line_endpoints(line_ref, x)) {
                                out.push(point_line_distance(p, d.0, d.1).abs() - target);
                            }
                        },
                    }
                }
            },
            ConstraintKind::Horizontal => {
                if let Some(d) = self.line_dir(&c.refs[0], x) {
                    out.push(d[1]);
                }
            },
            ConstraintKind::Vertical => {
                if let Some(d) = self.line_dir(&c.refs[0], x) {
                    out.push(d[0]);
                }
            },
            ConstraintKind::Parallel | ConstraintKind::Perpendicular | ConstraintKind::Angle => {
                if let (Some(d1), Some(d2)) = (self.line_dir(&c.refs[0], x), self.line_dir(&c.refs[1], x)) {
                    let n1 = (d1[0] * d1[0] + d1[1] * d1[1]).sqrt().max(1e-12);
                    let n2 = (d2[0] * d2[0] + d2[1] * d2[1]).sqrt().max(1e-12);
                    let cross = (d1[0] * d2[1] - d1[1] * d2[0]) / (n1 * n2);
                    let dot = (d1[0] * d2[0] + d1[1] * d2[1]) / (n1 * n2);
                    match c.kind {
                        ConstraintKind::Parallel => out.push(cross),
                        ConstraintKind::Perpendicular => out.push(dot),
                        ConstraintKind::Angle => {
                            let angle = cross.atan2(dot);
                            let target = c.value.unwrap_or(0.0).to_radians();
                            out.push(wrap_pi(angle - target) * 180.0 / std::f64::consts::PI);
                        },
                        _ => {},
                    }
                }
            },
            ConstraintKind::Radius => {
                if let Some(r) = self.radius(&c.refs[0], x) {
                    out.push(r - c.value.unwrap_or(0.0));
                }
            },
            ConstraintKind::Diameter => {
                if let Some(r) = self.radius(&c.refs[0], x) {
                    out.push(2.0 * r - c.value.unwrap_or(0.0));
                }
            },
            ConstraintKind::Equal => {
                if c.refs.len() == 2 {
                    if self.is_line_ref(&c.refs[0]) && self.is_line_ref(&c.refs[1]) {
                        if let (Some(d1), Some(d2)) = (self.line_dir(&c.refs[0], x), self.line_dir(&c.refs[1], x)) {
                            out.push(norm(d1) - norm(d2));
                        }
                    } else if let (Some(r1), Some(r2)) = (self.radius(&c.refs[0], x), self.radius(&c.refs[1], x)) {
                        out.push(r1 - r2);
                    }
                }
            },
            ConstraintKind::Concentric => {
                if let (Some(a), Some(b)) = (self.center(&c.refs[0], x), self.center(&c.refs[1], x)) {
                    out.push(a[0] - b[0]);
                    out.push(a[1] - b[1]);
                }
            },
            ConstraintKind::Symmetric => {
                if c.refs.len() == 3 {
                    if let (Some(a), Some(b), Some(line)) = (
                        self.point(&c.refs[0], x),
                        self.point(&c.refs[1], x),
                        self.line_endpoints(&c.refs[2], x),
                    ) {
                        let d = [line.1[0] - line.0[0], line.1[1] - line.0[1]];
                        let n = norm(d).max(1e-12);
                        let mid = [(a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5];
                        out.push(cross(d, [mid[0] - line.0[0], mid[1] - line.0[1]]) / n);
                        out.push(dot([b[0] - a[0], b[1] - a[1]], d) / n);
                    }
                }
            },
            ConstraintKind::PointOnLine => {
                let (point_ref, line_ref) = if c.refs.first().is_some_and(|r| r.point.is_some()) {
                    (&c.refs[0], c.refs.get(1))
                } else {
                    (c.refs.get(1).unwrap_or(&c.refs[0]), c.refs.first())
                };
                if let (Some(p), Some(l)) = (self.point(point_ref, x), line_ref.and_then(|r| self.line_endpoints(r, x))) {
                    let d = [l.1[0] - l.0[0], l.1[1] - l.0[1]];
                    out.push(cross(d, [p[0] - l.0[0], p[1] - l.0[1]]) / norm(d).max(1e-12));
                }
            },
            ConstraintKind::Tangent => {
                {
                    let (r0, r1) = (&c.refs[0], &c.refs[1]);
                    let line_curve = if self.is_line_ref(r0) && self.is_curve_ref(r1) {
                        Some((r0, r1))
                    } else if self.is_line_ref(r1) && self.is_curve_ref(r0) {
                        Some((r1, r0))
                    } else {
                        None
                    };
                    if let Some((line, curve)) = line_curve {
                        if let (Some(l), Some(center), Some(radius)) = (
                            self.line_endpoints(line, x),
                            self.center(curve, x),
                            self.radius(curve, x),
                        ) {
                            out.push(point_line_distance(center, l.0, l.1).abs() - radius);
                        }
                    } else if let (Some(c1), Some(r1), Some(c2), Some(r2)) = (
                        self.center(r0, x),
                        self.radius(r0, x),
                        self.center(r1, x),
                        self.radius(r1, x),
                    ) {
                        let d = ((c1[0] - c2[0]).powi(2) + (c1[1] - c2[1]).powi(2)).sqrt();
                        let target = if *self.tangent_internal.get(&c.id).unwrap_or(&false) {
                            (r1 - r2).abs()
                        } else {
                            r1 + r2
                        };
                        out.push(d - target);
                    }
                }
            },
        }
        out
    }

    fn line_endpoints(&self, r: &SketchRef, x: &[f64]) -> Option<([f64; 2], [f64; 2])> {
        let base = self.base_of(r.element)?;
        match self.element_of(r.element)? {
            SketchElement::Line { .. } => Some(([x[base], x[base + 1]], [x[base + 2], x[base + 3]])),
            _ => None,
        }
    }

    fn residuals(&self, x: &[f64]) -> Vec<f64> {
        let mut out = Vec::with_capacity(self.constraints.len() * 2);
        for c in self.constraints {
            out.extend(self.constraint_residuals(c, x));
        }
        out
    }

    fn max_residual(&self, x: &[f64]) -> f64 {
        self.residuals(x).iter().fold(0.0f64, |m, v| m.max(v.abs()))
    }
}

fn center_radius(e: &SketchElement) -> Option<([f64; 2], f64)> {
    match e {
        SketchElement::Circle { center, radius, .. } => Some((*center, *radius)),
        SketchElement::Arc { center, radius, .. } => Some((*center, *radius)),
        _ => None,
    }
}

fn norm(v: [f64; 2]) -> f64 {
    (v[0] * v[0] + v[1] * v[1]).sqrt()
}

fn dot(a: [f64; 2], b: [f64; 2]) -> f64 {
    a[0] * b[0] + a[1] * b[1]
}

fn cross(a: [f64; 2], b: [f64; 2]) -> f64 {
    a[0] * b[1] - a[1] * b[0]
}

fn point_line_distance(p: [f64; 2], a: [f64; 2], b: [f64; 2]) -> f64 {
    let d = [b[0] - a[0], b[1] - a[1]];
    cross(d, [p[0] - a[0], p[1] - a[1]]) / norm(d).max(1e-12)
}

fn wrap_pi(mut a: f64) -> f64 {
    while a > std::f64::consts::PI {
        a -= std::f64::consts::TAU;
    }
    while a <= -std::f64::consts::PI {
        a += std::f64::consts::TAU;
    }
    a
}

/// True when the constraint equations are redundant, with diagnostics naming
/// the constraints to remove. Also reports unsatisfied constraints by id.
fn diagnostics(ctx: &Ctx<'_>, x: &[f64], solved: bool, nvars: usize, dof: i64) -> (bool, Vec<String>) {
    let mut out = Vec::new();
    let mut push_unique = |s: String| {
        if !out.iter().any(|d: &String| *d == s) {
            out.push(s);
        }
    };

    for c in ctx.constraints {
        let r = ctx.constraint_residuals(c, x);
        let m = r.iter().fold(0.0f64, |m, v| m.max(v.abs()));
        if m > UNSAT_TOL {
            push_unique(format!(
                "constraint {} ({}) is unsatisfied: residual {:.3e}",
                c.id,
                c.kind.as_str(),
                m
            ));
        }
    }

    // Named contradictions that do not need the solver to expose.
    for (i, a) in ctx.constraints.iter().enumerate() {
        for b in ctx.constraints.iter().skip(i + 1) {
            if a.kind != b.kind || !same_ref_set(&a.refs, &b.refs) {
                continue;
            }
            let vals_differ = match a.kind {
                ConstraintKind::Distance | ConstraintKind::Radius | ConstraintKind::Diameter => {
                    match (a.value, b.value) {
                        (Some(x), Some(y)) => (x - y).abs() > 1e-9,
                        _ => false,
                    }
                },
                ConstraintKind::Angle => match (a.value, b.value) {
                    (Some(x), Some(y)) => wrap_pi((x - y).to_radians()).abs() > 1e-9,
                    _ => false,
                },
                _ => false,
            };
            if vals_differ {
                push_unique(format!(
                    "constraints {} and {} are contradictory: both are {} on the same elements but ask for different values",
                    a.id,
                    b.id,
                    a.kind.as_str()
                ));
            }
        }
    }
    for (i, a) in ctx.constraints.iter().enumerate() {
        for b in ctx.constraints.iter().skip(i + 1) {
            let conflict = match (a.kind, b.kind) {
                (ConstraintKind::Horizontal, ConstraintKind::Vertical)
                | (ConstraintKind::Vertical, ConstraintKind::Horizontal) => same_ref_set(&a.refs, &b.refs),
                (ConstraintKind::Parallel, ConstraintKind::Perpendicular)
                | (ConstraintKind::Perpendicular, ConstraintKind::Parallel) => same_ref_set(&a.refs, &b.refs),
                _ => false,
            };
            if conflict {
                push_unique(format!(
                    "constraints {} ({}) and {} ({}) conflict on the same elements",
                    a.id,
                    a.kind.as_str(),
                    b.id,
                    b.kind.as_str()
                ));
            }
        }
    }

    let mut over = false;
    if solved {
        let j = numeric_jacobian(|v| ctx.residuals(v), x, nvars);
        let mut effective_rows = 0usize;
        let mut cursor = 0usize;
        let mut basis: Vec<Vec<f64>> = Vec::new();
        let mut rank = 0usize;
        for c in ctx.constraints {
            let rows = ctx.constraint_residuals(c, x).len();
            if rows == 0 {
                continue;
            }
            let slice = &j[cursor..(cursor + rows).min(j.len())];
            cursor += rows;
            // A singular configuration (e.g. distance 0 at coincident points)
            // gives an all-zero row that says nothing about redundancy.
            if slice.iter().all(|r| r.iter().all(|v| v.abs() <= 1e-9)) {
                continue;
            }
            effective_rows += rows;
            let mut trial = basis.clone();
            trial.extend_from_slice(slice);
            let r = rank_of(&trial, nvars);
            let added = r.saturating_sub(rank);
            if added == rows {
                basis = trial;
                rank = r;
            } else {
                over = true;
                if added == 0 {
                    push_unique(format!(
                        "constraint {} ({}) is redundant: its equation(s) are already implied by \
                         the other constraints (hint: remove it to clear the over-constraint)",
                        c.id,
                        c.kind.as_str()
                    ));
                } else {
                    push_unique(format!(
                        "constraint {} ({}) is partly redundant: only {added} of its {rows} \
                         equation(s) add independence (hint: relax or remove it)",
                        c.id,
                        c.kind.as_str()
                    ));
                }
                basis = trial;
                rank = r;
            }
        }
        if rank < effective_rows && !over {
            over = true;
            push_unique(format!(
                "sketch is over-constrained by {} dependent equation(s) (hint: remove a constraint)",
                effective_rows - rank
            ));
        }
        if !over && dof < 0 {
            over = true;
            push_unique(format!(
                "sketch is over-constrained by {} equation(s) (hint: remove a constraint)",
                -dof
            ));
        }
    }
    (over, out)
}

fn same_ref_set(a: &[SketchRef], b: &[SketchRef]) -> bool {
    if a.len() != b.len() {
        return false;
    }
    let key = |r: &SketchRef| (r.element, r.point.unwrap_or(u8::MAX));
    let mut ka: Vec<_> = a.iter().map(key).collect();
    let mut kb: Vec<_> = b.iter().map(key).collect();
    ka.sort();
    kb.sort();
    ka == kb
}

fn rank_of(rows: &[Vec<f64>], n: usize) -> usize {
    let mut m: Vec<Vec<f64>> = rows.to_vec();
    if m.is_empty() || n == 0 {
        return 0;
    }
    let scale = m
        .iter()
        .flat_map(|r| r.iter())
        .fold(0.0f64, |s, v| s.max(v.abs()))
        .max(1.0);
    let threshold = 1e-8 * scale;
    let mut rank = 0usize;
    let mut col = 0usize;
    while rank < m.len() && col < n {
        let (piv, val) = (rank..m.len()).fold((rank, 0.0f64), |best, r| {
            let v = m[r][col].abs();
            if v > best.1 { (r, v) } else { best }
        });
        if val <= threshold {
            col += 1;
            continue;
        }
        m.swap(rank, piv);
        let pv = m[rank][col];
        for r in 0..m.len() {
            if r == rank {
                continue;
            }
            let f = m[r][col] / pv;
            if f != 0.0 {
                for k in col..n {
                    m[r][k] -= f * m[rank][k];
                }
            }
        }
        rank += 1;
        col += 1;
    }
    rank
}

/// Forward-difference Jacobian, fixed step: rows = residuals, cols = vars.
fn numeric_jacobian(f: impl Fn(&[f64]) -> Vec<f64>, x: &[f64], n: usize) -> Vec<Vec<f64>> {
    let r0 = f(x);
    let mut j = vec![vec![0.0; n]; r0.len()];
    for k in 0..n {
        let mut xp = x.to_vec();
        xp[k] += H;
        let rp = f(&xp);
        for i in 0..r0.len() {
            j[i][k] = (rp.get(i).copied().unwrap_or(0.0) - r0.get(i).copied().unwrap_or(0.0)) / H;
        }
    }
    j
}

/// JᵀJ for a residuals×vars Jacobian.
fn mat_mul_t(j: &[Vec<f64>]) -> Vec<Vec<f64>> {
    let n = j.first().map_or(0, |r| r.len());
    let mut a = vec![vec![0.0; n]; n];
    for row in j {
        for i in 0..n {
            for k in i..n {
                a[i][k] += row[i] * row[k];
            }
        }
    }
    for i in 0..n {
        for k in 0..i {
            a[i][k] = a[k][i];
        }
    }
    a
}

fn mat_t_vec(j: &[Vec<f64>], r: &[f64]) -> Vec<f64> {
    let n = j.first().map_or(0, |row| row.len());
    let mut g = vec![0.0; n];
    for (row, &ri) in j.iter().zip(r) {
        for (gk, &rv) in g.iter_mut().zip(row) {
            *gk += rv * ri;
        }
    }
    g
}

/// Dense linear solve, Gaussian elimination with partial pivoting.
/// `a` is n×n, `b` length n; both are consumed. Returns None if singular.
fn solve_linear(a: &mut [Vec<f64>], b: &mut [f64]) -> Option<Vec<f64>> {
    let n = b.len();
    for col in 0..n {
        // Pivot: largest |a[row][col]| at or below the diagonal.
        let (piv, _) = (col..n).fold((col, a[col][col].abs()), |best, row| {
            let v = a[row][col].abs();
            if v > best.1 { (row, v) } else { best }
        });
        if a[piv][col].abs() < 1e-14 {
            return None;
        }
        a.swap(piv, col);
        b.swap(piv, col);

        let inv = 1.0 / a[col][col];
        for row in (col + 1)..n {
            let f = a[row][col] * inv;
            if f == 0.0 {
                continue;
            }
            for k in col..n {
                a[row][k] -= f * a[col][k];
            }
            b[row] -= f * b[col];
        }
    }

    let mut out = vec![0.0; n];
    for row in (0..n).rev() {
        let s: f64 = a[row][row + 1..n].iter().zip(&out[row + 1..n]).map(|(av, xv)| av * xv).sum();
        out[row] = (b[row] - s) / a[row][row];
    }
    Some(out)
}

// --- curve geometry ----------------------------------------------------------

fn norm360(mut a: f64) -> f64 {
    a %= 360.0;
    if a < 0.0 {
        a += 360.0;
    }
    a
}

/// Counter-clockwise sweep from `start_deg` to `end_deg`, in (0, 360].
pub fn arc_sweep(start_deg: f64, end_deg: f64) -> f64 {
    let s = norm360(end_deg) - norm360(start_deg);
    let s = norm360(s);
    if s <= 1e-9 { 360.0 } else { s }
}

fn arc_point(cx: f64, cy: f64, r: f64, deg: f64) -> [f64; 2] {
    let t = deg.to_radians();
    [cx + r * t.cos(), cy + r * t.sin()]
}

/// Number of tessellation segments for an arc of `sweep_deg` at `radius`.
fn arc_segments(radius: f64, sweep_deg: f64) -> usize {
    let full = ((radius * 60.0).sqrt().ceil() as usize).clamp(24, 256);
    ((full as f64 * sweep_deg / 360.0).ceil() as usize).clamp(2, 512)
}

/// Tessellate an arc into points start..=end (no duplicate closing point
/// unless the sweep is a full turn, in which case the first point is repeated
/// at the end).
fn arc_points(center: [f64; 2], radius: f64, start_deg: f64, end_deg: f64) -> Vec<[f64; 2]> {
    let sweep = arc_sweep(start_deg, end_deg);
    let n = arc_segments(radius, sweep);
    let mut out = Vec::with_capacity(n + 1);
    for i in 0..=n {
        let a = start_deg + sweep * i as f64 / n as f64;
        out.push(arc_point(center[0], center[1], radius, a));
    }
    out
}

/// Catmull-Rom interpolation through every control point. Open splines
/// duplicate their endpoints to define tangents; closed splines wrap.
fn spline_points(points: &[[f64; 2]], closed: bool) -> Vec<[f64; 2]> {
    let n = points.len();
    if n == 0 {
        return Vec::new();
    }
    if n == 1 {
        return vec![points[0]];
    }
    let get = |i: isize| -> [f64; 2] {
        if closed {
            points[((i % n as isize) + n as isize) as usize % n]
        } else {
            points[i.clamp(0, n as isize - 1) as usize]
        }
    };
    let spans = if closed { n } else { n - 1 };
    let mut out = Vec::new();
    for s in 0..spans {
        let p0 = get(s as isize - 1);
        let p1 = get(s as isize);
        let p2 = get(s as isize + 1);
        let p3 = get(s as isize + 2);
        let seg_len = norm([p2[0] - p1[0], p2[1] - p1[1]]);
        let steps = ((seg_len / 1.5).ceil() as usize).clamp(4, 48);
        let first = if s == 0 { 0 } else { 1 };
        for k in first..=steps {
            let t = k as f64 / steps as f64;
            let t2 = t * t;
            let t3 = t2 * t;
            let pt = [
                0.5 * (2.0 * p1[0]
                    + (-p0[0] + p2[0]) * t
                    + (2.0 * p0[0] - 5.0 * p1[0] + 4.0 * p2[0] - p3[0]) * t2
                    + (-p0[0] + 3.0 * p1[0] - 3.0 * p2[0] + p3[0]) * t3),
                0.5 * (2.0 * p1[1]
                    + (-p0[1] + p2[1]) * t
                    + (2.0 * p0[1] - 5.0 * p1[1] + 4.0 * p2[1] - p3[1]) * t2
                    + (-p0[1] + 3.0 * p1[1] - 3.0 * p2[1] + p3[1]) * t3),
            ];
            out.push(pt);
        }
    }
    if closed && out.len() > 1 {
        // The loop graph supplies the closing edge; drop the duplicate.
        out.pop();
    }
    out
}

// --- profiles ---------------------------------------------------------------

/// Extract closed profile loops (sketch-local 2D) for extrusion/revolve.
///
/// Circles, closed splines and full arcs are loops by themselves; lines,
/// open arcs and open splines connect endpoint-to-endpoint into simple closed
/// loops (every endpoint shared by exactly two elements). Construction
/// elements are ignored. Holes are *not* classified here — the solid builder
/// applies the even-odd rule, so nested loops become holes regardless of
/// drawing order or direction.
pub fn profile_loops(elements: &[SketchElement]) -> Result<Vec<Vec<[f64; 2]>>, String> {
    let printable: Vec<&SketchElement> = elements.iter().filter(|e| !is_construction(e)).collect();
    if printable.is_empty() {
        return Err(if elements.is_empty() {
            "sketch is empty (hint: add lines, arcs, a circle or a spline first)".into()
        } else {
            "sketch has only construction geometry (hint: construction elements guide constraints but \
             are excluded from profiles; add at least one printable element)"
                .to_string()
        });
    }

    // Endpoint matching tolerance. Solved coincidences agree to ~1e-8 mm;
    // scale the tolerance with the drawing so large sketches still match.
    let (mut lo, mut hi) = ([f64::MAX; 2], [f64::MIN; 2]);
    let mut grow = |p: [f64; 2]| {
        for i in 0..2 {
            lo[i] = lo[i].min(p[i]);
            hi[i] = hi[i].max(p[i]);
        }
    };
    for e in &printable {
        match e {
            SketchElement::Line { p1, p2, .. } => {
                grow(*p1);
                grow(*p2);
            },
            SketchElement::Circle { center, radius, .. } => {
                grow([center[0] - radius, center[1] - radius]);
                grow([center[0] + radius, center[1] + radius]);
            },
            SketchElement::Arc { center, radius, .. } => {
                grow([center[0] - radius, center[1] - radius]);
                grow([center[0] + radius, center[1] + radius]);
            },
            SketchElement::Spline { points, .. } => {
                for p in points {
                    grow(*p);
                }
            },
        }
    }
    let diag = ((hi[0] - lo[0]).powi(2) + (hi[1] - lo[1]).powi(2)).sqrt();
    let tol = (diag * 1e-6).max(1e-6);

    let mut loops: Vec<Vec<[f64; 2]>> = Vec::new();
    // Open chains: (points, element id) so an endpoint ends up connected to
    // exactly one other chain.
    let mut chains: Vec<(Vec<[f64; 2]>, u64)> = Vec::new();

    for e in &printable {
        match e {
            SketchElement::Circle { center, radius, .. } => {
                if *radius <= tol {
                    return Err(format!(
                        "circle at ({:.3}, {:.3}) has radius {radius:.4}; a profile needs positive radii",
                        center[0], center[1]
                    ));
                }
                let mut lp = arc_points(*center, *radius, 0.0, 360.0);
                lp.pop();
                loops.push(lp);
            },
            SketchElement::Arc { center, radius, start_deg, end_deg, id, .. } => {
                if *radius <= tol {
                    return Err(format!(
                        "arc at ({:.3}, {:.3}) has radius {radius:.4}; a profile needs positive radii",
                        center[0], center[1]
                    ));
                }
                let sweep = arc_sweep(*start_deg, *end_deg);
                let mut pts = arc_points(*center, *radius, *start_deg, *start_deg + sweep);
                if sweep >= 359.999 {
                    pts.pop();
                    loops.push(pts);
                } else {
                    chains.push((pts, *id));
                }
            },
            SketchElement::Spline { points, closed, id, .. } => {
                if points.len() < 2 {
                    return Err(format!(
                        "spline {id} has {} point(s); a spline needs at least 2 (3 when closed)",
                        points.len()
                    ));
                }
                let pts = spline_points(points, *closed);
                if pts.len() < 2 {
                    return Err(format!("spline {id} collapsed to fewer than 2 points"));
                }
                if *closed {
                    if signed_area(&pts).abs() <= tol * tol {
                        return Err(format!(
                            "closed spline {id} encloses no area (hint: spread its points out)"
                        ));
                    }
                    loops.push(pts);
                } else {
                    chains.push((pts, *id));
                }
            },
            SketchElement::Line { p1, p2, id, .. } => {
                if norm([p2[0] - p1[0], p2[1] - p1[1]]) > tol {
                    chains.push((vec![*p1, *p2], *id));
                }
            },
        }
    }

    if !chains.is_empty() {
        // Node graph over chain endpoints; each chain contributes one edge.
        let mut nodes: Vec<[f64; 2]> = Vec::new();
        let mut chain_nodes: Vec<[usize; 2]> = Vec::with_capacity(chains.len());
        for (pts, id) in &chains {
            let a = cluster(&mut nodes, pts[0], tol);
            let b = cluster(&mut nodes, *pts.last().unwrap(), tol);
            if a == b {
                // A chain that returns to its start is a loop even if it was
                // not explicitly marked closed.
                if pts.len() >= 3 {
                    let mut lp = pts.clone();
                    lp.pop();
                    if signed_area(&lp).abs() <= tol * tol {
                        return Err(format!(
                            "chain {id} returns to its start without enclosing area"
                        ));
                    }
                    loops.push(lp);
                    chain_nodes.push([usize::MAX, usize::MAX]);
                    continue;
                }
                return Err(format!(
                    "chain {id} starts and ends at ({:.3}, {:.3}) without enclosing area",
                    pts[0][0], pts[0][1]
                ));
            }
            chain_nodes.push([a, b]);
        }

        let mut adj: Vec<Vec<usize>> = vec![Vec::new(); nodes.len()];
        for (ci, [a, b]) in chain_nodes.iter().enumerate() {
            if *a != usize::MAX {
                adj[*a].push(ci);
                adj[*b].push(ci);
            }
        }
        for (ni, list) in adj.iter().enumerate() {
            if list.len() == 1 {
                return Err(format!(
                    "profile is open at ({:.3}, {:.3}): a curve end is not connected \
                     (hint: close the loop with lines/arcs or a coincident constraint)",
                    nodes[ni][0], nodes[ni][1]
                ));
            }
            if list.len() > 2 {
                return Err(format!(
                    "profile branches at ({:.3}, {:.3}): {} curve ends meet there \
                     (hint: profiles must be simple closed loops)",
                    nodes[ni][0], nodes[ni][1], list.len()
                ));
            }
        }

        let mut used = vec![false; chains.len()];
        for start in 0..chains.len() {
            if used[start] || chain_nodes[start][0] == usize::MAX {
                continue;
            }
            used[start] = true;
            let start_node = chain_nodes[start][0];
            let mut pts = chains[start].0.clone();
            let mut at = chain_nodes[start][1];
            loop {
                if at == start_node {
                    break;
                }
                let next = adj[at].iter().copied().find(|&s| !used[s]);
                let Some(next) = next else {
                    return Err(format!(
                        "profile is open at ({:.3}, {:.3}) (hint: close the loop)",
                        nodes[at][0], nodes[at][1]
                    ));
                };
                used[next] = true;
                let [a, b] = chain_nodes[next];
                let mut cp = chains[next].0.clone();
                if a == at {
                    pts.extend_from_slice(&cp[1..]);
                    at = b;
                } else {
                    cp.reverse();
                    pts.extend_from_slice(&cp[1..]);
                    at = a;
                }
            }
            // The walk appends the closing node; drop the duplicate.
            if pts.len() > 1 && norm([pts[0][0] - pts.last().unwrap()[0], pts[0][1] - pts.last().unwrap()[1]]) <= tol {
                pts.pop();
            }
            if pts.len() < 3 {
                return Err("profile loop has fewer than 3 corners (hint: remove duplicate curves)".into());
            }
            if signed_area(&pts).abs() <= tol * tol {
                return Err(format!(
                    "profile loop near ({:.3}, {:.3}) has zero area (hint: curves must enclose a region)",
                    pts[0][0], pts[0][1]
                ));
            }
            loops.push(pts);
        }
    }

    if loops.is_empty() {
        return Err(
            "sketch has no closed profile (hint: draw closed lines/arcs/spline or add a circle, then extrude)".into(),
        );
    }
    Ok(loops)
}

/// Find an existing node within `tol` of `p`, or add it. First match wins, so
/// the clustering is deterministic in element order.
fn cluster(nodes: &mut Vec<[f64; 2]>, p: [f64; 2], tol: f64) -> usize {
    for (i, q) in nodes.iter().enumerate() {
        if (q[0] - p[0]).hypot(q[1] - p[1]) <= tol {
            return i;
        }
    }
    nodes.push(p);
    nodes.len() - 1
}

/// Signed polygon area (positive = counter-clockwise).
fn signed_area(pts: &[[f64; 2]]) -> f64 {
    let mut a = 0.0;
    for i in 0..pts.len() {
        let p = pts[i];
        let q = pts[(i + 1) % pts.len()];
        a += p[0] * q[1] - q[0] * p[1];
    }
    a * 0.5
}

// --- display geometry --------------------------------------------------------

fn push_ribbon(m: &mut Mesh, a: [f64; 2], b: [f64; 2], pa: [f64; 2], pb: [f64; 2]) {
    let base = m.positions.len() as u32;
    let pts = [
        [a[0] + pa[0], a[1] + pa[1]],
        [b[0] + pb[0], b[1] + pb[1]],
        [b[0] - pb[0], b[1] - pb[1]],
        [a[0] - pa[0], a[1] - pa[1]],
    ];
    for p in pts {
        m.positions.push([p[0] as f32, p[1] as f32, 0.0]);
        m.normals.push([0.0, 0.0, 1.0]);
    }
    m.indices.extend_from_slice(&[base, base + 1, base + 2, base, base + 2, base + 3]);
}

fn push_polyline(m: &mut Mesh, pts: &[[f64; 2]], w: f64, closed: bool) {
    if pts.len() < 2 {
        return;
    }
    let n = if closed { pts.len() } else { pts.len() - 1 };
    for i in 0..n {
        let a = pts[i];
        let b = pts[(i + 1) % pts.len()];
        let d = [b[0] - a[0], b[1] - a[1]];
        let len = norm(d);
        if len < 1e-12 {
            continue;
        }
        let perp = [d[1] / len * w, -d[0] / len * w];
        push_ribbon(m, a, b, perp, perp);
    }
}

/// Tessellate a sketch into thin ribbon triangles so the existing viewer,
/// renderer and scene stats can display it unchanged. Ribbons lie in the
/// sketch plane (z = 0 local, normal +Z); width scales with the sketch
/// size so drawings stay visible at any scale. Construction elements are
/// drawn at half width.
pub fn display_mesh(elements: &[SketchElement]) -> Mesh {
    let mut m = Mesh::default();
    if elements.is_empty() {
        return m;
    }

    // Ribbon width relative to the drawing's extent.
    let mut min = [f64::MAX; 2];
    let mut max = [f64::MIN; 2];
    let mut grow = |p: [f64; 2]| {
        for i in 0..2 {
            min[i] = min[i].min(p[i]);
            max[i] = max[i].max(p[i]);
        }
    };
    for e in elements {
        match e {
            SketchElement::Line { p1, p2, .. } => {
                grow(*p1);
                grow(*p2);
            },
            SketchElement::Circle { center, radius, .. } => {
                grow([center[0] - radius, center[1] - radius]);
                grow([center[0] + radius, center[1] + radius]);
            },
            SketchElement::Arc { center, radius, .. } => {
                grow([center[0] - radius, center[1] - radius]);
                grow([center[0] + radius, center[1] + radius]);
            },
            SketchElement::Spline { points, .. } => {
                for p in points {
                    grow(*p);
                }
            },
        }
    }
    let diag = ((max[0] - min[0]).powi(2) + (max[1] - min[1]).powi(2)).sqrt();
    let base_w = (diag * 0.012).clamp(0.02, 5.0) * 0.5;

    for e in elements {
        let w = if is_construction(e) { base_w * 0.5 } else { base_w };
        match e {
            SketchElement::Line { p1, p2, .. } => {
                push_polyline(&mut m, &[*p1, *p2], w, false);
            },
            SketchElement::Circle { center, radius, .. } => {
                if *radius < 1e-12 {
                    continue;
                }
                let pts = arc_points(*center, *radius, 0.0, 360.0);
                push_polyline(&mut m, &pts, w, true);
            },
            SketchElement::Arc { center, radius, start_deg, end_deg, .. } => {
                if *radius < 1e-12 {
                    continue;
                }
                let sweep = arc_sweep(*start_deg, *end_deg);
                let pts = arc_points(*center, *radius, *start_deg, *start_deg + sweep);
                push_polyline(&mut m, &pts, w, sweep >= 359.999);
            },
            SketchElement::Spline { points, closed, .. } => {
                if points.len() < 2 {
                    continue;
                }
                let pts = spline_points(points, *closed);
                push_polyline(&mut m, &pts, w, *closed);
            },
        }
    }
    m
}

// --- trim / extend / offset --------------------------------------------------

/// Element ids touched by a sketch edit.
#[derive(Debug, Default, Clone)]
pub struct SketchEdit {
    pub created: Vec<u64>,
    pub removed: Vec<u64>,
    pub modified: Vec<u64>,
}

/// Trim the piece of `element` containing `at`. Returns the touched ids.
pub fn trim(
    elements: &mut Vec<SketchElement>,
    constraints: &mut Vec<SketchConstraint>,
    next_element: &mut u64,
    element: u64,
    at: [f64; 2],
) -> Result<SketchEdit, String> {
    let idx = elements
        .iter()
        .position(|e| element_id(e) == element)
        .ok_or_else(|| format!("sketch has no element {element}"))?;
    let mut params = intersection_params(elements, idx);
    params.retain(|t| t.is_finite() && *t >= -1e-9 && *t <= 1.0 + 1e-9);
    params.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    params.dedup_by(|a, b| (*a - *b).abs() < 1e-7);

    let target = elements[idx].clone();
    let mut edit = SketchEdit::default();

    // An intersection at an existing endpoint is not a cut: trimming runs
    // between interior crossings. Circles are closed, so every cut counts.
    if !matches!(target, SketchElement::Circle { .. }) {
        params.retain(|t| *t > 1e-7 && *t < 1.0 - 1e-7);
    }

    // Circle: needs two cuts to bound a removable piece.
    if let SketchElement::Circle { center, radius, construction, .. } = target {
        if params.len() < 2 {
            return Err(format!(
                "trim needs two intersections on circle {element} to bound the removed piece \
                 (found {}); add crossing elements or delete it explicitly",
                params.len()
            ));
        }
        let click = circle_param(center, radius, at);
        let n = params.len();
        let (mut lo, mut hi) = (params[n - 1] - 1.0, params[0]);
        for i in 0..n {
            let a = params[i];
            let b = if i + 1 < n { params[i + 1] } else { params[0] + 1.0 };
            if click >= a && click <= b {
                lo = a;
                hi = b;
                break;
            }
        }
        let keep_from = hi;
        let keep_to = lo + 1.0;
        let sweep = (keep_to - keep_from) * 360.0;
        if sweep <= 1e-7 {
            elements.remove(idx);
            constraints.retain(|c| c.refs.iter().all(|r| r.element != element));
            edit.removed.push(element);
            return Ok(edit);
        }
        let start = norm360(keep_from * 360.0);
        let end = norm360(keep_to * 360.0);
        elements[idx] = SketchElement::Arc {
            id: element,
            center,
            radius,
            start_deg: start,
            end_deg: end,
            construction,
        };
        // A circle only has a center point; keep center constraints.
        constraints.retain(|c| {
            c.refs
                .iter()
                .all(|r| r.element != element || r.point.is_none() || r.point == Some(0))
        });
        edit.modified.push(element);
        return Ok(edit);
    }

    let Some(click) = param_of_point(&target, at) else {
        return Err(format!(
            "element {element} cannot be trimmed (hint: trim works on lines, arcs and circles; \
             edit splines as control points)"
        ));
    };
    if params.is_empty() {
        return Err(format!(
            "trim found no intersection on element {element} (hint: trim needs another curve to cut against; \
             delete the element instead)"
        ));
    }

    let lo = params.iter().copied().filter(|t| *t < click - 1e-9).fold(0.0f64, f64::max);
    let hi = params.iter().copied().filter(|t| *t > click + 1e-9).fold(1.0f64, f64::min);
    let keep_start = lo > 1e-7;
    let keep_end = hi < 1.0 - 1e-7;

    let mut pieces: Vec<SketchElement> = Vec::new();
    if keep_start {
        if let Some(p) = sub_element(&target, element, 0.0, lo) {
            pieces.push(p);
        }
    }
    if keep_end {
        let id = if pieces.is_empty() {
            element
        } else {
            let id = *next_element;
            *next_element += 1;
            id
        };
        if let Some(p) = sub_element(&target, id, hi, 1.0) {
            if id != element {
                edit.created.push(id);
            }
            pieces.push(p);
        }
    }

    if pieces.is_empty() {
        elements.remove(idx);
        constraints.retain(|c| c.refs.iter().all(|r| r.element != element));
        edit.removed.push(element);
        return Ok(edit);
    }

    // The first piece keeps the original id so whole-element constraints
    // (direction, radius) still make sense. Point refs survive only when the
    // point itself survived *on that piece*: with both end pieces kept, the
    // original id is the start piece, so its end is a new cut.
    let original_keeps_start = keep_start;
    let original_keeps_end = !keep_start && keep_end;
    let keeps = kept_points(&target, original_keeps_start, original_keeps_end);
    constraints.retain(|c| {
        c.refs.iter().all(|r| {
            if r.element != element {
                return true;
            }
            match r.point {
                None => true,
                Some(p) => keeps.contains(&p),
            }
        })
    });

    elements.remove(idx);
    // The first surviving piece keeps the original id so whole-element
    // constraints (direction, radius) still make sense.
    elements.insert(idx, pieces.remove(0));
    edit.modified.push(element);
    elements.extend(pieces);
    Ok(edit)
}

/// Which point refs of `e` survive a trim that keeps [0,lo] and/or [hi,1].
fn kept_points(e: &SketchElement, keep_start: bool, keep_end: bool) -> Vec<u8> {
    match e {
        SketchElement::Line { .. } => {
            let mut v = Vec::new();
            if keep_start {
                v.push(0);
            }
            if keep_end {
                v.push(1);
            }
            v
        },
        SketchElement::Arc { .. } => {
            let mut v = vec![0];
            if keep_start {
                v.push(1);
            }
            if keep_end {
                v.push(2);
            }
            v
        },
        SketchElement::Circle { .. } => vec![0],
        SketchElement::Spline { .. } => Vec::new(),
    }
}

/// Build the sub-curve of `e` between element parameters `t0..t1`, using
/// `id` for the new element.
fn sub_element(e: &SketchElement, id: u64, t0: f64, t1: f64) -> Option<SketchElement> {
    match e {
        SketchElement::Line { p1, p2, construction, .. } => {
            let a = lerp(*p1, *p2, t0);
            let b = lerp(*p1, *p2, t1);
            if norm([b[0] - a[0], b[1] - a[1]]) <= EPS {
                return None;
            }
            Some(SketchElement::Line { id, p1: a, p2: b, construction: *construction })
        },
        SketchElement::Arc { center, radius, start_deg, end_deg, construction, .. } => {
            let sweep = arc_sweep(*start_deg, *end_deg);
            let a = *start_deg + sweep * t0;
            let b = *start_deg + sweep * t1;
            if (b - a).abs() <= 1e-9 {
                return None;
            }
            Some(SketchElement::Arc {
                id,
                center: *center,
                radius: *radius,
                start_deg: norm360(a),
                end_deg: norm360(b),
                construction: *construction,
            })
        },
        SketchElement::Circle { center, radius, construction, .. } => {
            if ((t1 - t0) * 360.0).abs() <= 1e-9 {
                return None;
            }
            Some(SketchElement::Arc {
                id,
                center: *center,
                radius: *radius,
                start_deg: norm360(t0 * 360.0),
                end_deg: norm360(t1 * 360.0),
                construction: *construction,
            })
        },
        SketchElement::Spline { .. } => None,
    }
}

fn lerp(a: [f64; 2], b: [f64; 2], t: f64) -> [f64; 2] {
    [a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t]
}

/// Parameter of the closest point on a line/arc/circle. Circles use [0,1)
/// over the full turn; lines and arcs use [0,1].
fn param_of_point(e: &SketchElement, p: [f64; 2]) -> Option<f64> {
    match e {
        SketchElement::Line { p1, p2, .. } => {
            let d = [p2[0] - p1[0], p2[1] - p1[1]];
            let len2 = dot(d, d);
            if len2 <= EPS {
                return None;
            }
            Some(dot([p[0] - p1[0], p[1] - p1[1]], d) / len2)
        },
        SketchElement::Circle { center, .. } => Some(circle_param(*center, 1.0, p)),
        SketchElement::Arc { center, radius, start_deg, end_deg, .. } => {
            let _ = radius;
            let sweep = arc_sweep(*start_deg, *end_deg);
            let angle = (p[1] - center[1]).atan2(p[0] - center[0]).to_degrees();
            Some(norm360(angle - *start_deg) / sweep)
        },
        SketchElement::Spline { .. } => None,
    }
}

fn circle_param(center: [f64; 2], _radius: f64, p: [f64; 2]) -> f64 {
    let angle = (p[1] - center[1]).atan2(p[0] - center[0]).to_degrees();
    norm360(angle) / 360.0
}

/// All intersections of element `idx` with every other non-construction
/// element, expressed as a parameter along the target.
fn intersection_params(elements: &[SketchElement], idx: usize) -> Vec<f64> {
    let target = &elements[idx];
    let mut out = Vec::new();
    for (oi, other) in elements.iter().enumerate() {
        if oi == idx {
            continue;
        }
        // Construction geometry is a valid trim boundary too: it is drawn as
        // a guide, so cutting against it is what the user sees.
        collect_intersections(target, other, &mut out);
    }
    out
}

fn collect_intersections(target: &SketchElement, other: &SketchElement, out: &mut Vec<f64>) {
    match (target, other) {
        (SketchElement::Line { p1, p2, .. }, SketchElement::Line { p1: q1, p2: q2, .. }) => {
            if let Some((s, _)) = line_line(*p1, *p2, *q1, *q2) {
                out.push(s);
            }
        },
        (SketchElement::Line { p1, p2, .. }, SketchElement::Circle { center, radius, .. }) => {
            for p in segment_circle(*p1, *p2, *center, *radius) {
                if let Some(s) = param_of_point(target, p) {
                    out.push(s);
                }
            }
        },
        (SketchElement::Line { p1, p2, .. }, SketchElement::Arc { center, radius, start_deg, end_deg, .. }) => {
            for p in segment_circle(*p1, *p2, *center, *radius) {
                if point_in_arc(p, *center, *start_deg, *end_deg) {
                    if let Some(s) = param_of_point(target, p) {
                        out.push(s);
                    }
                }
            }
        },
        (SketchElement::Circle { center, radius, .. }, SketchElement::Line { p1, p2, .. })
        | (SketchElement::Arc { center, radius, .. }, SketchElement::Line { p1, p2, .. }) => {
            for p in segment_circle(*p1, *p2, *center, *radius) {
                if target_accepts(target, p) {
                    if let Some(s) = param_of_point(target, p) {
                        out.push(s);
                    }
                }
            }
        },
        (SketchElement::Circle { center: c1, radius: r1, .. }, SketchElement::Circle { center: c2, radius: r2, .. }) => {
            for p in circle_circle(*c1, *r1, *c2, *r2) {
                if let Some(s) = param_of_point(target, p) {
                    out.push(s);
                }
            }
        },
        (SketchElement::Circle { center: c1, radius: r1, .. }, SketchElement::Arc { center: c2, radius: r2, start_deg, end_deg, .. }) => {
            for p in circle_circle(*c1, *r1, *c2, *r2) {
                if point_in_arc(p, *c2, *start_deg, *end_deg) && target_accepts(target, p) {
                    if let Some(s) = param_of_point(target, p) {
                        out.push(s);
                    }
                }
            }
        },
        (SketchElement::Arc { center: c1, radius: r1, .. }, SketchElement::Circle { center: c2, radius: r2, .. }) => {
            for p in circle_circle(*c1, *r1, *c2, *r2) {
                if target_accepts(target, p) {
                    if let Some(s) = param_of_point(target, p) {
                        out.push(s);
                    }
                }
            }
        },
        (SketchElement::Arc { center: c1, radius: r1, start_deg: a1, end_deg: b1, .. }, SketchElement::Arc { center: c2, radius: r2, start_deg: a2, end_deg: b2, .. }) => {
            for p in circle_circle(*c1, *r1, *c2, *r2) {
                if point_in_arc(p, *c2, *a2, *b2) && point_in_arc(p, *c1, *a1, *b1) {
                    if let Some(s) = param_of_point(target, p) {
                        out.push(s);
                    }
                }
            }
        },
        (_, SketchElement::Spline { points, closed, .. }) => {
            let poly = spline_points(points, *closed);
            for w in poly.windows(2) {
                if let (SketchElement::Line { p1, p2, .. }, _) = (target, w) {
                    if let Some((s, _)) = line_line(*p1, *p2, w[0], w[1]) {
                        out.push(s);
                    }
                } else if let (SketchElement::Circle { center, radius, .. }, _)
                | (SketchElement::Arc { center, radius, .. }, _) = (target, w)
                {
                    for p in segment_circle(w[0], w[1], *center, *radius) {
                        if target_accepts(target, p) {
                            if let Some(s) = param_of_point(target, p) {
                                out.push(s);
                            }
                        }
                    }
                }
            }
        },
        (SketchElement::Spline { .. }, _) => {
            // Target splines are not trimmable (handled before we get here).
        },
    }
}

fn target_accepts(target: &SketchElement, p: [f64; 2]) -> bool {
    match target {
        SketchElement::Circle { .. } => true,
        SketchElement::Arc { center, start_deg, end_deg, .. } => point_in_arc(p, *center, *start_deg, *end_deg),
        _ => true,
    }
}

fn point_in_arc(p: [f64; 2], center: [f64; 2], start_deg: f64, end_deg: f64) -> bool {
    let sweep = arc_sweep(start_deg, end_deg);
    if sweep >= 359.999 {
        return true;
    }
    let angle = norm360((p[1] - center[1]).atan2(p[0] - center[0]).to_degrees());
    let s = norm360(start_deg);
    let e = norm360(end_deg);
    if s <= e {
        angle >= s - 1e-7 && angle <= e + 1e-7
    } else {
        angle >= s - 1e-7 || angle <= e + 1e-7
    }
}

fn line_line(a: [f64; 2], b: [f64; 2], c: [f64; 2], d: [f64; 2]) -> Option<(f64, f64)> {
    let r = [b[0] - a[0], b[1] - a[1]];
    let s = [d[0] - c[0], d[1] - c[1]];
    let denom = cross(r, s);
    if denom.abs() < 1e-12 {
        return None;
    }
    let t = cross([c[0] - a[0], c[1] - a[1]], s) / denom;
    let u = cross([c[0] - a[0], c[1] - a[1]], r) / denom;
    if (-1e-9..=1.0 + 1e-9).contains(&t) && (-1e-9..=1.0 + 1e-9).contains(&u) {
        Some((t, u))
    } else {
        None
    }
}

fn segment_circle(a: [f64; 2], b: [f64; 2], center: [f64; 2], radius: f64) -> Vec<[f64; 2]> {
    let d = [b[0] - a[0], b[1] - a[1]];
    let f = [a[0] - center[0], a[1] - center[1]];
    let aa = dot(d, d);
    if aa < 1e-18 {
        return Vec::new();
    }
    let bb = 2.0 * dot(f, d);
    let cc = dot(f, f) - radius * radius;
    let disc = bb * bb - 4.0 * aa * cc;
    if disc < -1e-9 {
        return Vec::new();
    }
    let sq = disc.max(0.0).sqrt();
    let t1 = (-bb - sq) / (2.0 * aa);
    let t2 = (-bb + sq) / (2.0 * aa);
    let mut out = Vec::new();
    for t in [t1, t2] {
        if (-1e-9..=1.0 + 1e-9).contains(&t) {
            out.push(lerp(a, b, t));
        }
    }
    if out.len() == 2 && norm([out[0][0] - out[1][0], out[0][1] - out[1][1]]) < 1e-12 {
        out.pop();
    }
    out
}

fn circle_circle(c1: [f64; 2], r1: f64, c2: [f64; 2], r2: f64) -> Vec<[f64; 2]> {
    let d = [c2[0] - c1[0], c2[1] - c1[1]];
    let dist = norm(d);
    if dist < 1e-12 {
        return Vec::new();
    }
    if dist > r1 + r2 + 1e-9 || dist < (r1 - r2).abs() - 1e-9 {
        return Vec::new();
    }
    let a = (r1 * r1 - r2 * r2 + dist * dist) / (2.0 * dist);
    let h2 = r1 * r1 - a * a;
    if h2 < -1e-9 {
        return Vec::new();
    }
    let h = h2.max(0.0).sqrt();
    let mid = [c1[0] + d[0] * a / dist, c1[1] + d[1] * a / dist];
    if h < 1e-12 {
        return vec![mid];
    }
    let perp = [-d[1] / dist * h, d[0] / dist * h];
    let p1 = [mid[0] + perp[0], mid[1] + perp[1]];
    let p2 = [mid[0] - perp[0], mid[1] - perp[1]];
    if norm([p1[0] - p2[0], p1[1] - p2[1]]) < 1e-12 { vec![p1] } else { vec![p1, p2] }
}

/// Extend one line endpoint to the nearest intersection with another element.
pub fn extend(
    elements: &mut [SketchElement],
    element: u64,
    point: u8,
) -> Result<SketchEdit, String> {
    let idx = elements
        .iter()
        .position(|e| element_id(e) == element)
        .ok_or_else(|| format!("sketch has no element {element}"))?;
    let (p1, p2) = match &elements[idx] {
        SketchElement::Line { p1, p2, .. } => (*p1, *p2),
        other => {
            return Err(format!(
                "extend works on lines (element {element} is a {}); hint: edit the control points of arcs/splines",
                label(other)
            ));
        },
    };
    let (dir, start) = match point {
        0 => ([p1[0] - p2[0], p1[1] - p2[1]], p1),
        1 => ([p2[0] - p1[0], p2[1] - p1[1]], p2),
        _ => return Err(format!("line {element} has no point {point} (0 = start, 1 = end)")),
    };
    let len = norm(dir);
    if len < 1e-12 {
        return Err(format!("line {element} has zero length; nothing to extend"));
    }
    let unit = [dir[0] / len, dir[1] / len];
    let mut best: Option<f64> = None;
    for (oi, other) in elements.iter().enumerate() {
        if oi == idx {
            continue;
        }
        let hit = ray_intersections(start, unit, other);
        for t in hit {
            if t > 1e-7 && best.is_none_or(|b| t < b) {
                best = Some(t);
            }
        }
    }
    let Some(t) = best else {
        return Err(format!(
            "extend found no intersection for line {element} along ({:.3}, {:.3}) \
             (hint: draw another element in its path, or move the endpoint explicitly)",
            unit[0], unit[1]
        ));
    };
    let target = [start[0] + unit[0] * t, start[1] + unit[1] * t];
    match elements[idx] {
        SketchElement::Line { ref mut p1, ref mut p2, .. } => {
            if point == 0 {
                *p1 = target;
            } else {
                *p2 = target;
            }
        },
        _ => unreachable!(),
    }
    Ok(SketchEdit { created: Vec::new(), removed: Vec::new(), modified: vec![element] })
}

fn ray_intersections(origin: [f64; 2], dir: [f64; 2], other: &SketchElement) -> Vec<f64> {
    let far = [origin[0] + dir[0], origin[1] + dir[1]];
    let mut out = Vec::new();
    match other {
        SketchElement::Line { p1, p2, .. } => {
            // Ray-line: parameter along the ray is the distance because dir
            // is unit length.
            let s = [p2[0] - p1[0], p2[1] - p1[1]];
            let denom = cross(dir, s);
            if denom.abs() > 1e-12 {
                let q = [p1[0] - origin[0], p1[1] - origin[1]];
                let t = cross(q, s) / denom;
                let u = cross(q, dir) / denom;
                if t >= 0.0 && (-1e-9..=1.0 + 1e-9).contains(&u) {
                    out.push(t);
                }
            }
        },
        SketchElement::Circle { center, radius, .. } => {
            out.extend(ray_circle(origin, far, *center, *radius));
        },
        SketchElement::Arc { center, radius, start_deg, end_deg, .. } => {
            for t in ray_circle(origin, far, *center, *radius) {
                let p = [origin[0] + dir[0] * t, origin[1] + dir[1] * t];
                if point_in_arc(p, *center, *start_deg, *end_deg) {
                    out.push(t);
                }
            }
        },
        SketchElement::Spline { points, closed, .. } => {
            let poly = spline_points(points, *closed);
            for w in poly.windows(2) {
                let s = [w[1][0] - w[0][0], w[1][1] - w[0][1]];
                let denom = cross(dir, s);
                if denom.abs() > 1e-12 {
                    let q = [w[0][0] - origin[0], w[0][1] - origin[1]];
                    let t = cross(q, s) / denom;
                    let u = cross(q, dir) / denom;
                    if t >= 0.0 && (-1e-9..=1.0 + 1e-9).contains(&u) {
                        out.push(t);
                    }
                }
            }
        },
    }
    out
}

fn ray_circle(origin: [f64; 2], far: [f64; 2], center: [f64; 2], radius: f64) -> Vec<f64> {
    // `far` is origin + unit direction; the quadratic parameter is distance.
    let d = [far[0] - origin[0], far[1] - origin[1]];
    let f = [origin[0] - center[0], origin[1] - center[1]];
    let aa = dot(d, d);
    let bb = 2.0 * dot(f, d);
    let cc = dot(f, f) - radius * radius;
    let disc = bb * bb - 4.0 * aa * cc;
    if disc < -1e-9 || aa < 1e-18 {
        return Vec::new();
    }
    let sq = disc.max(0.0).sqrt();
    let mut out = Vec::new();
    for t in [(-bb - sq) / (2.0 * aa), (-bb + sq) / (2.0 * aa)] {
        if t >= 0.0 {
            out.push(t);
        }
    }
    out
}

/// Offset sketch curves, creating new elements.
pub fn offset(
    elements: &mut Vec<SketchElement>,
    next_element: &mut u64,
    selected: Option<&[u64]>,
    distance: f64,
    construction: Option<bool>,
) -> Result<SketchEdit, String> {
    if !distance.is_finite() || distance.abs() < 1e-12 {
        return Err(format!("offset distance must be a non-zero finite length (got {distance})"));
    }
    let ids: Vec<u64> = match selected {
        Some(ids) => {
            for id in ids {
                if !elements.iter().any(|e| element_id(e) == *id) {
                    return Err(format!("sketch has no element {id}"));
                }
            }
            let mut ids = ids.to_vec();
            ids.sort_unstable();
            ids.dedup();
            ids
        },
        None => elements
            .iter()
            .filter(|e| !is_construction(e))
            .map(element_id)
            .collect(),
    };
    if ids.is_empty() {
        return Err("offset selected no printable elements (hint: construction geometry is excluded unless named explicitly)".into());
    }

    let flag = |e: &SketchElement| construction.unwrap_or_else(|| is_construction(e));

    // A single closed loop of lines is mitered into a proper offset outline.
    // Positive distances move outward from the loop.
    if let Some(order) = line_loop_order(elements, &ids) {
        return offset_line_loop(elements, next_element, &order, distance, construction);
    }

    let mut created = Vec::new();
    for id in ids {
        let idx = elements.iter().position(|e| element_id(e) == id).unwrap();
        let source = elements[idx].clone();
        let id = *next_element;
        *next_element += 1;
        let construction = flag(&source);
        let new = match source {
            SketchElement::Line { p1, p2, .. } => {
                let d = [p2[0] - p1[0], p2[1] - p1[1]];
                let len = norm(d).max(1e-12);
                let n = [-d[1] / len * distance, d[0] / len * distance];
                SketchElement::Line {
                    id,
                    p1: [p1[0] + n[0], p1[1] + n[1]],
                    p2: [p2[0] + n[0], p2[1] + n[1]],
                    construction,
                }
            },
            SketchElement::Circle { center, radius, .. } => {
                let r = radius + distance;
                if r <= 1e-9 {
                    return Err(format!(
                        "offsetting circle {id} by {distance} would make its radius {r:.4}; \
                         choose a smaller magnitude"
                    ));
                }
                SketchElement::Circle { id, center, radius: r, construction }
            },
            SketchElement::Arc { center, radius, start_deg, end_deg, .. } => {
                let r = radius + distance;
                if r <= 1e-9 {
                    return Err(format!(
                        "offsetting arc {id} by {distance} would make its radius {r:.4}; \
                         choose a smaller magnitude"
                    ));
                }
                SketchElement::Arc {
                    id,
                    center,
                    radius: r,
                    start_deg,
                    end_deg,
                    construction,
                }
            },
            SketchElement::Spline { points, closed, .. } => {
                let pts = offset_spline_points(&points, closed, distance);
                SketchElement::Spline { id, points: pts, closed, construction }
            },
        };
        elements.push(new);
        created.push(id);
    }
    Ok(SketchEdit { created, removed: Vec::new(), modified: Vec::new() })
}

/// Offset a spline's control points along the local left normal. The
/// interpolating curve through those points is an approximation, unlike the
/// exact line/circle/arc offsets.
fn offset_spline_points(points: &[[f64; 2]], closed: bool, distance: f64) -> Vec<[f64; 2]> {
    let n = points.len();
    let mut out = Vec::with_capacity(n);
    for i in 0..n {
        let prev = if i == 0 {
            if closed { points[n - 1] } else { points[0] }
        } else {
            points[i - 1]
        };
        let next = if i + 1 == n {
            if closed { points[0] } else { points[n - 1] }
        } else {
            points[i + 1]
        };
        let d = [next[0] - prev[0], next[1] - prev[1]];
        let len = norm(d).max(1e-12);
        let nrm = [-d[1] / len, d[0] / len];
        out.push([points[i][0] + nrm[0] * distance, points[i][1] + nrm[1] * distance]);
    }
    out
}

/// Order a set of line elements into one closed loop, returning
/// `(element index, flipped)` in traversal order. `None` when the selection
/// is not exactly one closed loop of lines.
fn line_loop_order(elements: &[SketchElement], ids: &[u64]) -> Option<Vec<(usize, bool)>> {
    if ids.len() < 3 {
        return None;
    }
    let mut nodes: Vec<[f64; 2]> = Vec::new();
    let mut edges: Vec<(usize, [usize; 2])> = Vec::new();
    let tol = 1e-6;
    for id in ids {
        let idx = elements.iter().position(|e| element_id(e) == *id)?;
        let (p1, p2) = match &elements[idx] {
            SketchElement::Line { p1, p2, .. } => (*p1, *p2),
            _ => return None,
        };
        let a = cluster(&mut nodes, p1, tol);
        let b = cluster(&mut nodes, p2, tol);
        if a == b {
            return None;
        }
        edges.push((idx, [a, b]));
    }
    let mut adj: Vec<Vec<usize>> = vec![Vec::new(); nodes.len()];
    for (ei, (_, [a, b])) in edges.iter().enumerate() {
        adj[*a].push(ei);
        adj[*b].push(ei);
    }
    if adj.iter().any(|l| l.len() != 2) {
        return None;
    }
    let mut used = vec![false; edges.len()];
    let mut order = Vec::with_capacity(edges.len());
    let start = 0usize;
    used[start] = true;
    let (idx, [a, b]) = edges[start];
    order.push((idx, false));
    let mut at = b;
    let start_node = a;
    while order.len() < edges.len() {
        let next = adj[at].iter().copied().find(|&ei| !used[ei])?;
        used[next] = true;
        let (idx, [a, b]) = edges[next];
        if a == at {
            order.push((idx, false));
            at = b;
        } else {
            order.push((idx, true));
            at = a;
        }
    }
    if at != start_node {
        return None;
    }
    Some(order)
}

fn offset_line_loop(
    elements: &mut Vec<SketchElement>,
    next_element: &mut u64,
    order: &[(usize, bool)],
    distance: f64,
    construction: Option<bool>,
) -> Result<SketchEdit, String> {
    // Oriented endpoints along the loop.
    let mut oriented: Vec<([f64; 2], [f64; 2], bool)> = Vec::with_capacity(order.len());
    for (idx, flipped) in order {
        let (p1, p2, c) = match &elements[*idx] {
            SketchElement::Line { p1, p2, construction, .. } => (*p1, *p2, *construction),
            _ => unreachable!(),
        };
        oriented.push(if *flipped { (p2, p1, c) } else { (p1, p2, c) });
    }
    let area = signed_area(&oriented.iter().map(|(a, _, _)| *a).collect::<Vec<_>>());
    // Positive distance moves outward: right normal for a CCW loop, left for CW.
    let sign = if area >= 0.0 { 1.0 } else { -1.0 };
    let offset_line = |a: [f64; 2], b: [f64; 2]| -> ([f64; 2], [f64; 2]) {
        let d = [b[0] - a[0], b[1] - a[1]];
        let len = norm(d).max(1e-12);
        let n = [sign * d[1] / len * distance, -sign * d[0] / len * distance];
        ([a[0] + n[0], a[1] + n[1]], [b[0] + n[0], b[1] + n[1]])
    };
    let n = oriented.len();
    let mut corners = Vec::with_capacity(n);
    for i in 0..n {
        let (a1, b1, _) = oriented[i];
        let (a2, b2, _) = oriented[(i + 1) % n];
        let (oa1, ob1) = offset_line(a1, b1);
        let (oa2, ob2) = offset_line(a2, b2);
        let d1 = [ob1[0] - oa1[0], ob1[1] - oa1[1]];
        let d2 = [ob2[0] - oa2[0], ob2[1] - oa2[1]];
        let denom = cross(d1, d2);
        if denom.abs() < 1e-12 {
            return Err(format!(
                "cannot miter the offset at corner {i}: adjacent lines are parallel \
                 (hint: offset the lines individually with elements=[…])"
            ));
        }
        let q = [oa2[0] - oa1[0], oa2[1] - oa1[1]];
        let t = cross(q, d2) / denom;
        corners.push([oa1[0] + d1[0] * t, oa1[1] + d1[1] * t]);
    }
    let mut created = Vec::with_capacity(n);
    for i in 0..n {
        let id = *next_element;
        *next_element += 1;
        let c = construction.unwrap_or(oriented[i].2);
        elements.push(SketchElement::Line {
            id,
            p1: corners[(i + n - 1) % n],
            p2: corners[i],
            construction: c,
        });
        created.push(id);
    }
    Ok(SketchEdit { created, removed: Vec::new(), modified: Vec::new() })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn line(id: u64, x1: f64, y1: f64, x2: f64, y2: f64) -> SketchElement {
        SketchElement::Line { id, p1: [x1, y1], p2: [x2, y2], construction: false }
    }

    fn circle(id: u64, cx: f64, cy: f64, r: f64) -> SketchElement {
        SketchElement::Circle { id, center: [cx, cy], radius: r, construction: false }
    }

    fn arc(id: u64, cx: f64, cy: f64, r: f64, a0: f64, a1: f64) -> SketchElement {
        SketchElement::Arc {
            id,
            center: [cx, cy],
            radius: r,
            start_deg: a0,
            end_deg: a1,
            construction: false,
        }
    }

    fn pt(element: u64, point: u8) -> SketchRef {
        SketchRef { element, point: Some(point) }
    }

    fn whole(element: u64) -> SketchRef {
        SketchRef { element, point: None }
    }

    fn c(id: u64, kind: ConstraintKind, refs: Vec<SketchRef>, value: Option<f64>) -> SketchConstraint {
        SketchConstraint { id, kind, refs, value }
    }

    fn len(e: &SketchElement) -> f64 {
        match e {
            SketchElement::Line { p1, p2, .. } => ((p2[0] - p1[0]).powi(2) + (p2[1] - p1[1]).powi(2)).sqrt(),
            _ => panic!("not a line"),
        }
    }

    fn endpoints(e: &SketchElement) -> ([f64; 2], [f64; 2]) {
        match e {
            SketchElement::Line { p1, p2, .. } => (*p1, *p2),
            _ => panic!("not a line"),
        }
    }

    /// Four messy lines + coincidence/horizontal/vertical/distance must
    /// solve into a clean 40×30 rectangle.
    #[test]
    fn rectangle_solves_from_messy_start() {
        let mut elements = vec![
            line(1, 0.0, 0.0, 38.0, 3.0),
            line(2, 38.0, 3.0, 41.0, 31.0),
            line(3, 41.0, 31.0, -2.0, 28.0),
            line(4, -2.0, 28.0, 0.0, 0.0),
        ];
        let constraints = vec![
            c(1, ConstraintKind::Coincident, vec![pt(1, 1), pt(2, 0)], None),
            c(2, ConstraintKind::Coincident, vec![pt(2, 1), pt(3, 0)], None),
            c(3, ConstraintKind::Coincident, vec![pt(3, 1), pt(4, 0)], None),
            c(4, ConstraintKind::Coincident, vec![pt(4, 1), pt(1, 0)], None),
            c(5, ConstraintKind::Horizontal, vec![whole(1)], None),
            c(6, ConstraintKind::Horizontal, vec![whole(3)], None),
            c(7, ConstraintKind::Vertical, vec![whole(2)], None),
            c(8, ConstraintKind::Vertical, vec![whole(4)], None),
            c(9, ConstraintKind::Distance, vec![pt(1, 0), pt(1, 1)], Some(40.0)),
            c(10, ConstraintKind::Distance, vec![pt(2, 0), pt(2, 1)], Some(30.0)),
        ];
        let report = solve(&mut elements, &constraints);
        assert!(report.solved, "solver failed: {report:?}");
        assert!((len(&elements[0]) - 40.0).abs() < 1e-6, "bottom len {}", len(&elements[0]));
        assert!((len(&elements[1]) - 30.0).abs() < 1e-6, "right len {}", len(&elements[1]));
        assert!((len(&elements[2]) - 40.0).abs() < 1e-6, "top len {}", len(&elements[2]));
        assert!((len(&elements[3]) - 30.0).abs() < 1e-6, "left len {}", len(&elements[3]));
        // Right angles at every corner.
        for i in 0..4 {
            let (a1, a2) = endpoints(&elements[i]);
            let (b1, b2) = endpoints(&elements[(i + 1) % 4]);
            let d1 = [a2[0] - a1[0], a2[1] - a1[1]];
            let d2 = [b2[0] - b1[0], b2[1] - b1[1]];
            let dot = d1[0] * d2[0] + d1[1] * d2[1];
            let n1 = (d1[0] * d1[0] + d1[1] * d1[1]).sqrt();
            let n2 = (d2[0] * d2[0] + d2[1] * d2[1]).sqrt();
            assert!(dot.abs() / (n1 * n2) < 1e-6, "corner {i} not square: cos {}", dot / (n1 * n2));
        }
    }

    /// Radius + diameter constraints set the circle exactly.
    #[test]
    fn circle_radius_and_diameter() {
        let mut elements = vec![circle(1, 5.0, -3.0, 2.0), circle(2, 0.0, 0.0, 1.0)];
        let constraints = vec![
            c(1, ConstraintKind::Radius, vec![whole(1)], Some(10.0)),
            c(2, ConstraintKind::Diameter, vec![whole(2)], Some(25.0)),
        ];
        let report = solve(&mut elements, &constraints);
        assert!(report.solved, "{report:?}");
        match (&elements[0], &elements[1]) {
            (SketchElement::Circle { radius: r1, center: c1, .. }, SketchElement::Circle { radius: r2, .. }) => {
                assert!((r1 - 10.0).abs() < 1e-6, "radius {r1}");
                assert!((r2 - 12.5).abs() < 1e-6, "radius {r2}");
                assert!((c1[0] - 5.0).abs() < 1e-6 && (c1[1] + 3.0).abs() < 1e-6,
                    "radius constraint must not move the center: {c1:?}");
            },
            _ => panic!("expected circles"),
        }
    }

    /// Parallel and perpendicular are enforced on the line directions.
    #[test]
    fn parallel_and_perpendicular_directions() {
        let mut elements = vec![
            line(1, 0.0, 0.0, 10.0, 1.0),
            line(2, 0.0, 5.0, 9.0, 7.0),
            line(3, 0.0, -5.0, 2.0, -6.0),
        ];
        let constraints = vec![
            c(1, ConstraintKind::Parallel, vec![whole(1), whole(2)], None),
            c(2, ConstraintKind::Perpendicular, vec![whole(1), whole(3)], None),
            c(3, ConstraintKind::Horizontal, vec![whole(1)], None),
        ];
        let report = solve(&mut elements, &constraints);
        assert!(report.solved, "{report:?}");
        let dir = |e: &SketchElement| {
            let (a, b) = endpoints(e);
            [b[0] - a[0], b[1] - a[1]]
        };
        let d1 = dir(&elements[0]);
        let d2 = dir(&elements[1]);
        let d3 = dir(&elements[2]);
        let n1 = (d1[0] * d1[0] + d1[1] * d1[1]).sqrt();
        let n2 = (d2[0] * d2[0] + d2[1] * d2[1]).sqrt();
        let n3 = (d3[0] * d3[0] + d3[1] * d3[1]).sqrt();
        assert!((d1[0] * d2[1] - d1[1] * d2[0]).abs() / (n1 * n2) < 1e-6, "not parallel");
        assert!((d1[0] * d3[0] + d1[1] * d3[1]).abs() / (n1 * n3) < 1e-6, "not perpendicular");
        assert!(n1 < 100.0 && n2 < 100.0 && n3 < 100.0, "solution drifted: {n1} {n2} {n3}");
    }

    /// Contradictory distances must be reported, not silently "solved".
    #[test]
    fn over_constrained_is_reported() {
        let mut elements = vec![line(1, 0.0, 0.0, 10.0, 0.0)];
        let constraints = vec![
            c(1, ConstraintKind::Distance, vec![pt(1, 0), pt(1, 1)], Some(40.0)),
            c(2, ConstraintKind::Distance, vec![pt(1, 0), pt(1, 1)], Some(50.0)),
        ];
        let report = solve(&mut elements, &constraints);
        assert!(!report.solved, "contradiction must not report success: {report:?}");
        assert!(report.residual > 1.0, "residual should expose the conflict: {report:?}");
        assert!(report.diagnostics.iter().any(|d| d.contains("constraint 1")), "{:?}", report.diagnostics);
        assert!(report.diagnostics.iter().any(|d| d.contains("contradictory")), "{:?}", report.diagnostics);
    }

    /// A redundant but consistent constraint is named as over-constrained.
    #[test]
    fn redundant_constraint_is_named() {
        let mut elements = vec![line(1, 0.0, 0.0, 10.0, 0.0)];
        let constraints = vec![
            c(1, ConstraintKind::Distance, vec![pt(1, 0), pt(1, 1)], Some(40.0)),
            c(2, ConstraintKind::Distance, vec![pt(1, 0), pt(1, 1)], Some(40.0)),
        ];
        let report = solve(&mut elements, &constraints);
        assert!(report.solved, "{report:?}");
        assert!(report.over_constrained, "{report:?}");
        assert!(report.diagnostics.iter().any(|d| d.contains("constraint 2")), "{:?}", report.diagnostics);
    }

    /// Equal, concentric, symmetric, angle, point-on-line and tangent all
    /// reach their geometric targets.
    #[test]
    fn extended_constraint_vocabulary() {
        let mut elements = vec![
            line(1, 0.0, 0.0, 10.0, 0.0),
            line(2, 0.0, 5.0, 8.0, 5.0),
            line(3, 0.0, 10.0, 7.0, 10.0),
            circle(4, 0.0, 0.0, 3.0),
            circle(5, 4.0, 0.0, 2.0),
            line(6, 0.0, 20.0, 10.0, 20.0),
            circle(7, 4.0, 22.0, 1.0),
            circle(8, 0.0, 32.0, 2.0),
            line(9, 0.0, 30.0, 10.0, 30.0),
        ];
        let constraints = vec![
            c(1, ConstraintKind::Horizontal, vec![whole(1)], None),
            c(2, ConstraintKind::Equal, vec![whole(1), whole(2)], None),
            c(3, ConstraintKind::Angle, vec![whole(2), whole(3)], Some(30.0)),
            c(4, ConstraintKind::Concentric, vec![whole(4), whole(5)], None),
            c(5, ConstraintKind::Horizontal, vec![whole(6)], None),
            c(6, ConstraintKind::PointOnLine, vec![pt(7, 0), whole(6)], None),
            c(7, ConstraintKind::Horizontal, vec![whole(9)], None),
            c(8, ConstraintKind::Tangent, vec![whole(9), whole(8)], None),
        ];
        let report = solve(&mut elements, &constraints);
        assert!(report.solved, "{report:?}");
        assert!(report.residual < 1e-6, "{report:?}");
        assert!((len(&elements[0]) - len(&elements[1])).abs() < 1e-5, "equal lengths");
        match (&elements[3], &elements[4]) {
            (SketchElement::Circle { center: a, .. }, SketchElement::Circle { center: b, .. }) => {
                assert!((a[0] - b[0]).abs() < 1e-5 && (a[1] - b[1]).abs() < 1e-5, "concentric {a:?} {b:?}");
            },
            _ => panic!("expected circles"),
        }
        let d1 = match &elements[1] {
            SketchElement::Line { p1, p2, .. } => [p2[0] - p1[0], p2[1] - p1[1]],
            _ => unreachable!(),
        };
        let d2 = match &elements[2] {
            SketchElement::Line { p1, p2, .. } => [p2[0] - p1[0], p2[1] - p1[1]],
            _ => unreachable!(),
        };
        let angle = cross(d1, d2).atan2(dot(d1, d2)).to_degrees().abs();
        assert!((angle - 30.0).abs() < 1e-5, "angle {angle}");
        if let (SketchElement::Circle { center, .. }, SketchElement::Line { p1, p2, .. }) = (&elements[6], &elements[5]) {
            let d = [p2[0] - p1[0], p2[1] - p1[1]];
            let off = cross(d, [center[0] - p1[0], center[1] - p1[1]]) / norm(d);
            assert!(off.abs() < 1e-5, "point_on_line center {center:?} off line {:?}", (p1, p2));
        }
        if let (SketchElement::Circle { center, radius, .. }, SketchElement::Line { p1, .. }) =
            (&elements[7], &elements[8])
        {
            assert!((center[1] - p1[1]).abs() - radius.abs() < 1e-5, "tangent {center:?} {p1:?}");
        }
    }

    /// Symmetric mirrors two points about a line.
    #[test]
    fn symmetric_points_about_an_axis() {
        let mut elements = vec![
            line(1, 0.0, 0.0, 0.0, 10.0),
            line(2, -5.0, 4.0, -3.0, 6.0),
            line(3, 4.0, 7.0, 6.0, 5.0),
        ];
        let constraints = vec![
            c(1, ConstraintKind::Vertical, vec![whole(1)], None),
            c(2, ConstraintKind::Symmetric, vec![pt(2, 0), pt(3, 0), whole(1)], None),
            c(3, ConstraintKind::Symmetric, vec![pt(2, 1), pt(3, 1), whole(1)], None),
        ];
        let report = solve(&mut elements, &constraints);
        assert!(report.solved, "{report:?}");
        let (a0, a1) = endpoints(&elements[1]);
        let (b0, b1) = endpoints(&elements[2]);
        let axis = match &elements[0] {
            SketchElement::Line { p1, p2, .. } => [p2[0] - p1[0], p2[1] - p1[1]],
            _ => unreachable!(),
        };
        // Both midpoints lie on the axis line, and the connecting segments
        // are perpendicular to it.
        for (a, b) in [(a0, b0), (a1, b1)] {
            let mid = [(a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5];
            let m = match &elements[0] {
                SketchElement::Line { p1, .. } => [mid[0] - p1[0], mid[1] - p1[1]],
                _ => unreachable!(),
            };
            assert!(cross(axis, m).abs() / norm(axis) < 1e-5, "midpoint off the axis: {a:?} {b:?}");
            assert!(dot([b[0] - a[0], b[1] - a[1]], axis).abs() / norm(axis) < 1e-5, "not mirrored: {a:?} {b:?}");
        }
    }

    /// Arcs tessellate into a closed profile together with lines.
    #[test]
    fn arc_and_lines_form_a_closed_profile() {
        // Quarter disc: the arc runs from (10,0) to (0,10), and two lines
        // close the shape through the center.
        let elements = vec![
            line(1, 0.0, 10.0, 0.0, 0.0),
            line(2, 0.0, 0.0, 10.0, 0.0),
            arc(3, 0.0, 0.0, 10.0, 0.0, 90.0),
        ];
        let loops = profile_loops(&elements).unwrap();
        assert_eq!(loops.len(), 1);
        let area = signed_area(&loops[0]).abs();
        // Quarter disc area = π·100/4 ≈ 78.54.
        assert!(area > 76.0 && area < 80.0, "area {area}");
    }

    /// A closed spline is a profile on its own; construction elements are
    /// excluded.
    #[test]
    fn closed_spline_profile_and_construction_exclusion() {
        let spline = SketchElement::Spline {
            id: 1,
            points: vec![[0.0, 0.0], [20.0, 0.0], [20.0, 20.0], [0.0, 20.0]],
            closed: true,
            construction: false,
        };
        let guide = SketchElement::Line {
            id: 2,
            p1: [0.0, 0.0],
            p2: [5.0, 5.0],
            construction: true,
        };
        let loops = profile_loops(&[spline, guide]).unwrap();
        assert_eq!(loops.len(), 1);
        assert!(signed_area(&loops[0]).abs() > 300.0, "spline loop area {}", signed_area(&loops[0]).abs());

        // A sketch made only of construction geometry has no printable profile.
        let only_guide = SketchElement::Line {
            id: 1,
            p1: [0.0, 0.0],
            p2: [10.0, 0.0],
            construction: true,
        };
        let err = profile_loops(&[only_guide]).unwrap_err();
        assert!(err.contains("only construction"), "{err}");
    }

    /// Trim removes the clicked piece and keeps the rest.
    #[test]
    fn trim_line_and_circle() {
        let mut elements = vec![
            line(1, 0.0, 0.0, 10.0, 0.0),
            line(2, 5.0, -5.0, 5.0, 5.0),
            line(3, 3.0, -5.0, 3.0, 5.0),
        ];
        let mut constraints = Vec::new();
        let mut next = 4;
        let edit = trim(&mut elements, &mut constraints, &mut next, 1, [4.0, 0.0]).unwrap();
        assert_eq!(edit.modified, vec![1]);
        assert_eq!(edit.created.len(), 1);
        // One piece goes from x=0 to the first cut at x=3, the other from x=5
        // to x=10; the clicked piece 3..5 is gone.
        let lengths: Vec<f64> = elements
            .iter()
            .filter_map(|e| match e {
                SketchElement::Line { id, .. } if *id == 1 || edit.created.contains(id) => Some(len(e)),
                _ => None,
            })
            .collect();
        assert!(lengths.contains(&3.0), "lengths {lengths:?}");
        assert!(lengths.contains(&5.0), "lengths {lengths:?}");

        // A circle cut twice becomes an arc of the kept portion.
        let mut elements = vec![circle(1, 0.0, 0.0, 10.0), line(2, -20.0, 5.0, 20.0, 5.0)];
        let mut constraints = vec![c(1, ConstraintKind::Radius, vec![whole(1)], Some(10.0))];
        let mut next = 3;
        let edit = trim(&mut elements, &mut constraints, &mut next, 1, [0.0, 10.0]).unwrap();
        assert_eq!(edit.modified, vec![1]);
        assert!(matches!(elements[0], SketchElement::Arc { .. }), "{:?}", elements[0]);
        assert_eq!(constraints.len(), 1, "radius constraint survives");
    }

    /// Extend a line to the first boundary in its path.
    #[test]
    fn extend_line_to_a_boundary() {
        let mut elements = vec![line(1, 0.0, 0.0, 5.0, 0.0), line(2, 10.0, -5.0, 10.0, 5.0)];
        let edit = extend(&mut elements, 1, 1).unwrap();
        assert_eq!(edit.modified, vec![1]);
        let (_, end) = endpoints(&elements[0]);
        assert!((end[0] - 10.0).abs() < 1e-6 && end[1].abs() < 1e-6, "{end:?}");
    }

    /// Offset a rectangle outward; the copies are mitered into one loop.
    #[test]
    fn offset_closed_rectangle_mitered() {
        let mut elements = vec![
            line(1, 0.0, 0.0, 20.0, 0.0),
            line(2, 20.0, 0.0, 20.0, 10.0),
            line(3, 20.0, 10.0, 0.0, 10.0),
            line(4, 0.0, 10.0, 0.0, 0.0),
        ];
        let mut next = 5;
        let edit = offset(&mut elements, &mut next, None, 2.0, None).unwrap();
        assert_eq!(edit.created, vec![5, 6, 7, 8]);
        let loops = profile_loops(&elements).unwrap();
        let areas: Vec<f64> = loops.iter().map(|l| signed_area(l).abs()).collect();
        assert!(areas.iter().any(|a| (*a - 200.0).abs() < 1e-6), "{areas:?}");
        // 24 × 14 = 336 mm².
        assert!(areas.iter().any(|a| (*a - 336.0).abs() < 1e-6), "{areas:?}");
    }

    /// The validator accepts the new constraint shapes and rejects malformed
    /// references with an actionable hint.
    #[test]
    fn constraint_validation_accepts_the_new_kinds() {
        let elements = vec![
            line(1, 0.0, 0.0, 10.0, 0.0),
            line(2, 0.0, 5.0, 8.0, 5.0),
            line(3, 0.0, 10.0, 7.0, 10.0),
            circle(4, 0.0, 0.0, 3.0),
            circle(5, 4.0, 0.0, 2.0),
            arc(6, 0.0, 0.0, 4.0, 0.0, 90.0),
        ];
        assert!(validate(ConstraintKind::Equal, &elements, &[whole(1), whole(2)], None).is_ok());
        assert!(validate(ConstraintKind::Equal, &elements, &[whole(4), whole(5)], None).is_ok());
        assert!(validate(ConstraintKind::Concentric, &elements, &[whole(4), whole(6)], None).is_ok());
        assert!(validate(
            ConstraintKind::Symmetric,
            &elements,
            &[pt(1, 0), pt(2, 0), whole(3)],
            None
        )
        .is_ok());
        assert!(validate(ConstraintKind::Angle, &elements, &[whole(1), whole(2)], Some(45.0)).is_ok());
        assert!(validate(ConstraintKind::PointOnLine, &elements, &[pt(4, 0), whole(1)], None).is_ok());
        assert!(validate(ConstraintKind::Tangent, &elements, &[whole(1), whole(4)], None).is_ok());
        assert!(validate(ConstraintKind::Tangent, &elements, &[whole(4), whole(5)], None).is_ok());
        assert!(validate(
            ConstraintKind::Distance,
            &elements,
            &[pt(4, 0), whole(1)],
            Some(2.0)
        )
        .is_ok());

        // A point ref that cannot exist names the element kind in the hint.
        let err = validate(ConstraintKind::Coincident, &elements, &[pt(1, 7), pt(2, 0)], None)
            .unwrap_err();
        assert!(err.contains("line") && err.contains("point 7"), "{err}");
        // Mixing a line and a curve is not an equality.
        let err = validate(ConstraintKind::Equal, &elements, &[whole(1), whole(4)], None)
            .unwrap_err();
        assert!(err.contains("not a mix"), "{err}");
        // A single line cannot be tangent to itself.
        let err = validate(ConstraintKind::Tangent, &elements, &[whole(1), whole(2)], None)
            .unwrap_err();
        assert!(err.contains("tangent needs"), "{err}");
    }

    /// Trimming keeps whole-element constraints on the surviving piece but
    /// drops a point ref that the cut removed.
    #[test]
    fn trim_preserves_only_surviving_point_refs() {
        let mut elements = vec![
            line(1, 0.0, 0.0, 20.0, 0.0),
            line(2, 5.0, -5.0, 5.0, 5.0),
            line(3, 15.0, -5.0, 15.0, 5.0),
        ];
        let mut constraints = vec![
            c(1, ConstraintKind::Horizontal, vec![whole(1)], None),
            c(2, ConstraintKind::Distance, vec![pt(1, 0), pt(1, 1)], Some(20.0)),
            c(3, ConstraintKind::Distance, vec![pt(1, 0), pt(2, 0)], Some(5.0)),
            c(4, ConstraintKind::Distance, vec![pt(1, 1), pt(3, 0)], Some(5.0)),
        ];
        let mut next = 4;
        let edit = trim(&mut elements, &mut constraints, &mut next, 1, [10.0, 0.0]).unwrap();
        assert_eq!(edit.modified, vec![1]);
        assert_eq!(edit.created.len(), 1);
        assert!(constraints.iter().any(|c| c.id == 1), "whole-element constraint survives");
        assert!(constraints.iter().any(|c| c.id == 3), "start-point constraint survives");
        assert!(!constraints.iter().any(|c| c.id == 2), "original end point is now a cut");
        assert!(!constraints.iter().any(|c| c.id == 4), "original end point is now a cut");
    }


    #[test]
    fn display_mesh_is_wellformed() {
        let elements = vec![
            line(1, 0.0, 0.0, 10.0, 0.0),
            circle(2, 0.0, 0.0, 5.0),
            arc(3, 0.0, 0.0, 5.0, 0.0, 90.0),
            SketchElement::Spline {
                id: 4,
                points: vec![[0.0, 0.0], [5.0, 5.0], [10.0, 0.0]],
                closed: false,
                construction: true,
            },
        ];
        let m = display_mesh(&elements);
        assert_eq!(m.positions.len() % 4, 0);
        assert_eq!(m.indices.len(), m.positions.len() / 4 * 6);
        assert!(m.indices.iter().all(|&i| (i as usize) < m.positions.len()));
        assert!(m.positions.iter().all(|p| p[2] == 0.0), "ribbons must lie in the sketch plane");
        assert!(display_mesh(&[]).positions.is_empty());
    }
}
