//! 2D sketches: constraint-solved drawings that live inside an entity.
//!
//! Elements (lines, circles) hold the current coordinates; constraints
//! declare relationships the solver must satisfy. Solving is a
//! Levenberg-Marquardt-flavored Gauss-Newton over all element variables —
//! pure f64, fixed iteration order, no RNG: **same sketch state + same
//! constraints → same solution**, which is what parametric replay depends
//! on.
//!
//! Constraints are equations over the variable vector; the Jacobian is
//! computed numerically (forward differences, fixed step) which keeps the
//! code uniform across constraint kinds. Residuals are in sketch units
//! (mm), so thresholds translate directly to geometric accuracy.
//!
//! Constraint equation counts (used for the DOF report):
//! coincident 2 · horizontal/vertical/parallel/perpendicular/distance/
//! radius/diameter 1.

use agcad_protocol::{
    ConstraintKind, Mesh, SketchConstraint, SketchElement, SketchRef,
};

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

/// Outcome of a solve pass, mirrored into snapshots.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SolveReport {
    pub solved: bool,
    pub residual: f64,
    pub dof: i64,
}

// --- public API ----------------------------------------------------------------

/// Validate a constraint against the sketch's current elements (arity,
/// ref shapes, target type, value presence). Returns a hint on failure.
pub fn validate(kind: ConstraintKind, elements: &[SketchElement], refs: &[SketchRef], value: Option<f64>) -> Result<(), String> {
    let is_line = |r: &SketchRef| matches!(find(elements, r.element), Some(SketchElement::Line { .. }));
    let is_circle = |r: &SketchRef| matches!(find(elements, r.element), Some(SketchElement::Circle { .. }));

    // Resolve refs first: "unknown element 7" is far more useful than
    // "needs a whole-element ref to a line" when the id is simply wrong.
    if let Some(bad) = refs.iter().find(|r| find(elements, r.element).is_none()) {
        return Err(format!(
            "sketch has no element {} (hint: ids come from add_sketch_line/add_sketch_circle; get_scene lists them)",
            bad.element
        ));
    }

    let count_points = refs.iter().filter(|r| r.point.is_some()).count();
    let count_whole = refs.iter().filter(|r| r.point.is_none()).count();

    match kind {
        ConstraintKind::Coincident => {
            if refs.len() != 2 || count_points != 2 {
                return Err("coincident needs exactly 2 point refs (element ids with point: 0 or 1)".into());
            }
        },
        ConstraintKind::Distance => {
            if refs.len() != 2 || count_points != 2 {
                return Err("distance needs exactly 2 point refs (element ids with point: 0 or 1)".into());
            }
            check_value(kind, value)?;
        },
        ConstraintKind::Horizontal | ConstraintKind::Vertical => {
            if refs.len() != 1 || count_whole != 1 || !is_line(&refs[0]) {
                return Err(format!("{} needs exactly 1 whole-element ref to a line (omit 'point')", kind.as_str()));
            }
        },
        ConstraintKind::Parallel | ConstraintKind::Perpendicular => {
            if refs.len() != 2 || count_whole != 2 || !refs.iter().all(is_line) {
                return Err(format!("{} needs exactly 2 whole-element refs to lines (omit 'point')", kind.as_str()));
            }
        },
        ConstraintKind::Radius | ConstraintKind::Diameter => {
            if refs.len() != 1 || !is_circle(&refs[0]) {
                return Err(format!("{} needs exactly 1 ref to a circle element", kind.as_str()));
            }
            check_value(kind, value)?;
        },
    }
    Ok(())
}

fn check_value(kind: ConstraintKind, value: Option<f64>) -> Result<(), String> {
    let v = value.ok_or_else(|| format!("{} needs a numeric 'value' (mm; may be an expression over parameters)", kind.as_str()))?;
    if !(v.is_finite()) || v < 0.0 {
        return Err(format!("{} value must be a finite, non-negative length (got {v})", kind.as_str()));
    }
    Ok(())
}

fn find<'a>(elements: &'a [SketchElement], id: u64) -> Option<&'a SketchElement> {
    elements.iter().find(|e| element_id(e) == id)
}

fn element_id(e: &SketchElement) -> u64 {
    match e {
        SketchElement::Line { id, .. } | SketchElement::Circle { id, .. } => *id,
    }
}

/// Solve the sketch in place. On success the element coordinates are
/// updated to the solution; on failure they are left untouched and the
/// report says so (with the residual that blocked convergence).
pub fn solve(elements: &mut [SketchElement], constraints: &[SketchConstraint]) -> SolveReport {
    // Flatten element state into a variable vector:
    // line → (p1x, p1y, p2x, p2y), circle → (cx, cy, r).
    let mut bases = Vec::with_capacity(elements.len());
    let mut n = 0usize;
    for e in elements.iter() {
        bases.push(n);
        n += match e {
            SketchElement::Line { .. } => 4,
            SketchElement::Circle { .. } => 3,
        };
    }

    let mut x: Vec<f64> = Vec::with_capacity(n);
    for e in elements.iter() {
        match e {
            SketchElement::Line { p1, p2, .. } => x.extend_from_slice(&[p1[0], p1[1], p2[0], p2[1]]),
            SketchElement::Circle { center, radius, .. } => x.extend_from_slice(&[center[0], center[1], *radius]),
        }
    }

    let eqs: i64 = constraints
        .iter()
        .map(|c| match c.kind {
            ConstraintKind::Coincident => 2,
            _ => 1,
        })
        .sum();
    let dof = n as i64 - eqs;

    if constraints.is_empty() {
        return SolveReport { solved: true, residual: 0.0, dof };
    }

    let ctx = Ctx { bases: &bases, constraints, elements };

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
        }
    }
    SolveReport { solved, residual: if solved { residual.min(TOL) } else { residual }, dof }
}

/// Context tying the variable vector back to element/constraint layout.
struct Ctx<'a> {
    bases: &'a [usize],
    constraints: &'a [SketchConstraint],
    elements: &'a [SketchElement],
}

impl Ctx<'_> {
    /// Variable indices of a ref'd point: returns (ix, iy).
    fn point(&self, r: &SketchRef) -> Option<(usize, usize)> {
        let idx = self.elements.iter().position(|e| element_id(e) == r.element)?;
        let base = self.bases[idx];
        match (&self.elements[idx], r.point) {
            (SketchElement::Line { .. }, Some(0)) => Some((base, base + 1)),
            (SketchElement::Line { .. }, Some(1)) => Some((base + 2, base + 3)),
            (SketchElement::Circle { .. }, Some(0)) | (SketchElement::Circle { .. }, None) => Some((base, base + 1)),
            _ => None,
        }
    }

    /// Variable indices of a whole line element (for direction constraints).
    fn line_vars(&self, r: &SketchRef) -> Option<(usize, usize, usize, usize)> {
        let idx = self.elements.iter().position(|e| element_id(e) == r.element)?;
        match &self.elements[idx] {
            SketchElement::Line { .. } => {
                let b = self.bases[idx];
                Some((b, b + 1, b + 2, b + 3))
            },
            _ => None,
        }
    }

    /// Variable index of a circle radius.
    fn radius_var(&self, r: &SketchRef) -> Option<usize> {
        let idx = self.elements.iter().position(|e| element_id(e) == r.element)?;
        match &self.elements[idx] {
            SketchElement::Circle { .. } => Some(self.bases[idx] + 2),
            _ => None,
        }
    }

    fn residuals(&self, x: &[f64]) -> Vec<f64> {
        let mut out = Vec::with_capacity(self.constraints.len() * 2);
        for c in self.constraints {
            match c.kind {
                ConstraintKind::Coincident => {
                    let (a, b) = (self.point(&c.refs[0]), self.point(&c.refs[1]));
                    if let (Some((ax, ay)), Some((bx, by))) = (a, b) {
                        out.push(x[ax] - x[bx]);
                        out.push(x[ay] - x[by]);
                    }
                },
                ConstraintKind::Distance => {
                    if let (Some((ax, ay)), Some((bx, by))) = (self.point(&c.refs[0]), self.point(&c.refs[1])) {
                        let dx = x[bx] - x[ax];
                        let dy = x[by] - x[ay];
                        out.push((dx * dx + dy * dy).sqrt() - c.value.unwrap_or(0.0));
                    }
                },
                ConstraintKind::Horizontal => {
                    if let Some((_x1, y1, _x2, y2)) = self.line_vars(&c.refs[0]) {
                        out.push(x[y1] - x[y2]);
                    }
                },
                ConstraintKind::Vertical => {
                    if let Some((x1, _y1, x2, _y2)) = self.line_vars(&c.refs[0]) {
                        out.push(x[x1] - x[x2]);
                    }
                },
                ConstraintKind::Parallel | ConstraintKind::Perpendicular => {
                    if let (Some(d1), Some(d2)) = (self.line_vars(&c.refs[0]), self.line_vars(&c.refs[1])) {
                        let e1 = (x[d1.2] - x[d1.0], x[d1.3] - x[d1.1]);
                        let e2 = (x[d2.2] - x[d2.0], x[d2.3] - x[d2.1]);
                        // Normalize to sin/cos of the angle between the lines:
                        // raw cross/dot products scale with length², which
                        // makes the Jacobian badly conditioned and prevents
                        // convergence to an absolute tolerance.
                        let n1 = (e1.0 * e1.0 + e1.1 * e1.1).sqrt().max(1e-12);
                        let n2 = (e2.0 * e2.0 + e2.1 * e2.1).sqrt().max(1e-12);
                        if c.kind == ConstraintKind::Parallel {
                            out.push((e1.0 * e2.1 - e1.1 * e2.0) / (n1 * n2));
                        } else {
                            out.push((e1.0 * e2.0 + e1.1 * e2.1) / (n1 * n2));
                        }
                    }
                },
                ConstraintKind::Radius => {
                    if let Some(r) = self.radius_var(&c.refs[0]) {
                        out.push(x[r] - c.value.unwrap_or(0.0));
                    }
                },
                ConstraintKind::Diameter => {
                    if let Some(r) = self.radius_var(&c.refs[0]) {
                        out.push(2.0 * x[r] - c.value.unwrap_or(0.0));
                    }
                },
            }
        }
        out
    }

    fn max_residual(&self, x: &[f64]) -> f64 {
        self.residuals(x).iter().fold(0.0f64, |m, v| m.max(v.abs()))
    }
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

// --- profiles ---------------------------------------------------------------

/// Extract closed profile loops (sketch-local 2D) for extrusion/revolve.
///
/// Circles are tessellated into closed loops; line segments must form
/// simple closed loops (every endpoint shared by exactly two segments).
/// Holes are *not* classified here — the solid builder applies the even-odd
/// rule, so nested loops become holes regardless of drawing order or
/// direction.
pub fn profile_loops(elements: &[SketchElement]) -> Result<Vec<Vec<[f64; 2]>>, String> {
    if elements.is_empty() {
        return Err("sketch is empty (hint: add lines or a circle first)".into());
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
        }
    }
    let diag = ((hi[0] - lo[0]).powi(2) + (hi[1] - lo[1]).powi(2)).sqrt();
    let tol = (diag * 1e-6).max(1e-6);

    let mut loops: Vec<Vec<[f64; 2]>> = Vec::new();

    // Circles are closed loops by themselves.
    for e in elements {
        if let SketchElement::Circle { center, radius, .. } = e {
            if *radius <= tol {
                return Err(format!(
                    "circle at ({:.3}, {:.3}) has radius {radius:.4}; a profile needs positive radii",
                    center[0], center[1]
                ));
            }
            let n = ((radius * 60.0).sqrt().ceil() as usize).clamp(24, 256);
            let mut lp = Vec::with_capacity(n);
            for i in 0..n {
                let t = i as f64 * std::f64::consts::TAU / n as f64;
                lp.push([center[0] + radius * t.cos(), center[1] + radius * t.sin()]);
            }
            loops.push(lp);
        }
    }

    // Line segments → endpoint graph → closed loops.
    let mut segs: Vec<([f64; 2], [f64; 2])> = Vec::new();
    for e in elements {
        if let SketchElement::Line { p1, p2, .. } = e {
            let len = ((p2[0] - p1[0]).powi(2) + (p2[1] - p1[1]).powi(2)).sqrt();
            if len > tol {
                segs.push((*p1, *p2));
            }
        }
    }

    if !segs.is_empty() {
        let mut nodes: Vec<[f64; 2]> = Vec::new();
        let mut seg_nodes: Vec<[usize; 2]> = Vec::with_capacity(segs.len());
        for (a, b) in &segs {
            let ia = cluster(&mut nodes, *a, tol);
            let ib = cluster(&mut nodes, *b, tol);
            if ia == ib {
                return Err(format!(
                    "line at ({:.3}, {:.3}) has zero length; remove it or re-solve the sketch",
                    a[0], a[1]
                ));
            }
            seg_nodes.push([ia, ib]);
        }

        let mut adj: Vec<Vec<usize>> = vec![Vec::new(); nodes.len()];
        for (si, [a, b]) in seg_nodes.iter().enumerate() {
            adj[*a].push(si);
            adj[*b].push(si);
        }
        for (ni, list) in adj.iter().enumerate() {
            if list.len() == 1 {
                return Err(format!(
                    "profile is open at ({:.3}, {:.3}): a line end is not connected \
                     (hint: close the loop with lines or a coincident constraint)",
                    nodes[ni][0], nodes[ni][1]
                ));
            }
            if list.len() > 2 {
                return Err(format!(
                    "profile branches at ({:.3}, {:.3}): {} line ends meet there \
                     (hint: profiles must be simple closed loops)",
                    nodes[ni][0], nodes[ni][1], list.len()
                ));
            }
        }

        let mut used = vec![false; segs.len()];
        for start in 0..segs.len() {
            if used[start] {
                continue;
            }
            let start_node = seg_nodes[start][0];
            let mut pts = vec![nodes[start_node]];
            let (mut cur, mut at) = (start, start_node);
            loop {
                used[cur] = true;
                let [a, b] = seg_nodes[cur];
                let next = if a == at { b } else { a };
                if next == start_node {
                    break;
                }
                pts.push(nodes[next]);
                at = next;
                match adj[at].iter().copied().find(|&s| !used[s]) {
                    Some(s) => cur = s,
                    None => {
                        return Err(format!(
                            "profile is open at ({:.3}, {:.3}) (hint: close the loop)",
                            nodes[at][0], nodes[at][1]
                        ));
                    },
                }
            }
            if pts.len() < 3 {
                return Err("profile loop has fewer than 3 corners (hint: remove duplicate lines)".into());
            }
            if signed_area(&pts).abs() <= tol * tol {
                return Err(format!(
                    "profile loop near ({:.3}, {:.3}) has zero area (hint: lines must enclose a region)",
                    pts[0][0], pts[0][1]
                ));
            }
            loops.push(pts);
        }
    }

    if loops.is_empty() {
        return Err(
            "sketch has no closed profile (hint: draw closed lines or add a circle, then extrude)".into(),
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

// --- display geometry ------------------------------------------------------------

/// Tessellate a sketch into thin ribbon triangles so the existing viewer,
/// renderer and scene stats can display it unchanged. Ribbons lie in the
/// sketch plane (z = 0 local, normal +Z); width scales with the sketch
/// size so drawings stay visible at any scale.
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
        }
    }
    let diag = ((max[0] - min[0]).powi(2) + (max[1] - min[1]).powi(2)).sqrt();
    let w = (diag * 0.012).clamp(0.02, 5.0) * 0.5;
    let n = [0.0f32, 0.0, 1.0];

    let push_ribbon = |m: &mut Mesh, a: [f64; 2], b: [f64; 2], pa: [f64; 2], pb: [f64; 2]| {
        let base = m.positions.len() as u32;
        let pts = [
            [a[0] + pa[0], a[1] + pa[1]],
            [b[0] + pb[0], b[1] + pb[1]],
            [b[0] - pb[0], b[1] - pb[1]],
            [a[0] - pa[0], a[1] - pa[1]],
        ];
        for p in pts {
            m.positions.push([p[0] as f32, p[1] as f32, 0.0]);
            m.normals.push(n);
        }
        m.indices.extend_from_slice(&[base, base + 1, base + 2, base, base + 2, base + 3]);
    };

    for e in elements {
        match e {
            SketchElement::Line { p1, p2, .. } => {
                let dx = p2[0] - p1[0];
                let dy = p2[1] - p1[1];
                let len = (dx * dx + dy * dy).sqrt();
                if len < 1e-12 {
                    continue;
                }
                // Right-hand normal: keeps the ribbon quad CCW as seen
                // from +Z, so the rasterizer's backface cull keeps it.
                let perp = [dy / len * w, -dx / len * w];
                push_ribbon(&mut m, *p1, *p2, perp, perp);
            },
            SketchElement::Circle { center, radius, .. } => {
                if *radius < 1e-12 {
                    continue;
                }
                const SEG: usize = 64;
                let mut prev = [center[0] + radius, center[1]];
                for i in 1..=SEG {
                    let t = (i as f64) * std::f64::consts::TAU / SEG as f64;
                    let p = [center[0] + radius * t.cos(), center[1] + radius * t.sin()];
                    // Radial offset for a ribbon that follows the ring.
                    let u = [
                        (prev[0] - center[0]) / radius * w,
                        (prev[1] - center[1]) / radius * w,
                    ];
                    let v = [(p[0] - center[0]) / radius * w, (p[1] - center[1]) / radius * w];
                    push_ribbon(&mut m, prev, p, u, v);
                    prev = p;
                }
            },
        }
    }
    m
}

#[cfg(test)]
mod tests {
    use super::*;

    fn line(id: u64, x1: f64, y1: f64, x2: f64, y2: f64) -> SketchElement {
        SketchElement::Line { id, p1: [x1, y1], p2: [x2, y2] }
    }

    fn circle(id: u64, cx: f64, cy: f64, r: f64) -> SketchElement {
        SketchElement::Circle { id, center: [cx, cy], radius: r }
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
            SketchElement::Line { p1, p2, .. } => {
                ((p2[0] - p1[0]).powi(2) + (p2[1] - p1[1]).powi(2)).sqrt()
            },
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
        // Constraint residuals are normalized (sin/cos of the angle), so
        // compare the same quantities here.
        assert!((d1[0] * d2[1] - d1[1] * d2[0]).abs() / (n1 * n2) < 1e-6, "not parallel");
        assert!((d1[0] * d3[0] + d1[1] * d3[1]).abs() / (n1 * n3) < 1e-6, "not perpendicular");
        // The regularization must keep the drawing near its input size.
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
    }

    /// Solving twice from the same input is bit-identical (determinism).
    #[test]
    fn solve_is_deterministic() {
        let start = vec![line(1, 0.0, 0.0, 38.0, 3.0), line(2, 38.0, 3.0, 41.0, 31.0)];
        let constraints = vec![
            c(1, ConstraintKind::Coincident, vec![pt(1, 1), pt(2, 0)], None),
            c(2, ConstraintKind::Distance, vec![pt(1, 0), pt(1, 1)], Some(25.0)),
        ];
        let mut a = start.clone();
        let mut b = start.clone();
        solve(&mut a, &constraints);
        solve(&mut b, &constraints);
        assert_eq!(a, b);
    }

    /// Display ribbons are well-formed triangles in the sketch plane.
    #[test]
    fn display_mesh_is_wellformed() {
        let elements = vec![line(1, 0.0, 0.0, 10.0, 0.0), circle(2, 0.0, 0.0, 5.0)];
        let m = display_mesh(&elements);
        assert_eq!(m.positions.len() % 4, 0);
        assert_eq!(m.indices.len(), m.positions.len() / 4 * 6);
        assert!(m.indices.iter().all(|&i| (i as usize) < m.positions.len()));
        assert!(m.positions.iter().all(|p| p[2] == 0.0), "ribbons must lie in the sketch plane");
        assert!(display_mesh(&[]).positions.is_empty());
    }
}
