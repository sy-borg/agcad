//! Multi-view engineering drawings.
//!
//! This is the *drawing* side of M19, distinct from the sketch-local 2D
//! exports in [`crate::drawing`]. It projects world geometry onto
//! orthographic (and one isometric) views, removes hidden lines, dimensions
//! each view's overall size, and can add part balloons, a BOM table and a
//! title block.
//!
//! Design choices worth knowing:
//!
//! * **Feature + silhouette edges.** A triangle mesh has no faces, so edges
//!   are rebuilt by welding vertices, pairing triangles, and keeping edges
//!   whose dihedral angle exceeds a threshold — plus edges where the two
//!   adjacent faces face opposite directions *for this view* (the silhouette
//!   of a cylinder). Boundary edges of an open mesh are drawn too.
//! * **Hidden lines are sampled.** Each candidate edge is sampled along its
//!   length and every sample is depth-tested against a deterministic 2D grid
//!   of projected triangles. Runs of visible samples become solid polylines,
//!   runs of occluded samples become dashed ones. Sampling is coarse enough
//!   to stay fast and fine enough for a readable sheet.
//! * **Sheet millimeters.** Coordinates are millimeters, `scale` scales the
//!   geometry on the sheet, and dimension text always reports the real model
//!   length. Nothing random, no timestamps: identical scenes produce
//!   identical bytes.
//!
//! The resulting [`Drawing`] is handed to the existing SVG/DXF writers, so
//! both formats share styling, layer names and number formatting.

use std::collections::{BTreeMap, BTreeSet};

use wright_protocol::{
    CommandResult, DocId, DrawingAnnotation, DrawingFormat, DrawingViewSpec, Mat4, Vec3, ViewKind,
};

use crate::drawing::{self, Curve, CurveKind, Drawing, Role};
use crate::mat4;
use crate::{assembly, Document, Error, Result};

/// Everything the sheet builder needs from an export command.
pub struct SheetOptions<'a> {
    pub title: Option<&'a str>,
    pub views: Option<&'a [DrawingViewSpec]>,
    pub scale: Option<f64>,
    pub hidden_lines: Option<bool>,
    pub dimensions: Option<bool>,
    pub balloons: Option<bool>,
    pub bom: Option<bool>,
    pub annotations: &'a [DrawingAnnotation],
}

const SHARP_DEG: f64 = 30.0;
const VIEW_GAP: f64 = 26.0;
const DIM_OFFSET: f64 = 12.0;
const MARGIN: f64 = 14.0;
const TITLE_W: f64 = 120.0;
const TITLE_H: f64 = 26.0;

// --- projection frames --------------------------------------------------------

#[derive(Debug, Clone, Copy)]
struct Frame {
    /// Viewing direction (from the camera into the scene).
    dir: Vec3,
    right: Vec3,
    up: Vec3,
}

impl Frame {
    fn uv(&self, p: Vec3) -> [f64; 2] {
        [dot(p, self.right), dot(p, self.up)]
    }

    fn depth(&self, p: Vec3) -> f64 {
        dot(p, self.dir)
    }
}

fn dot(a: Vec3, b: Vec3) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn cross(a: Vec3, b: Vec3) -> Vec3 {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

fn normalize(v: Vec3) -> Vec3 {
    let len = dot(v, v).sqrt();
    if !len.is_finite() || len < 1e-12 {
        [0.0; 3]
    } else {
        [v[0] / len, v[1] / len, v[2] / len]
    }
}

fn frame(kind: ViewKind) -> Frame {
    let (dir, up_world) = match kind {
        ViewKind::Front => ([0.0, 1.0, 0.0], [0.0, 0.0, 1.0]),
        ViewKind::Back => ([0.0, -1.0, 0.0], [0.0, 0.0, 1.0]),
        // Third-angle: the top view above the front has the front face at
        // its bottom edge, so screen up is +Y.
        ViewKind::Top => ([0.0, 0.0, -1.0], [0.0, 1.0, 0.0]),
        ViewKind::Bottom => ([0.0, 0.0, 1.0], [0.0, -1.0, 0.0]),
        // The right view's left edge is the front face, so screen right is +Y.
        ViewKind::Right => ([-1.0, 0.0, 0.0], [0.0, 0.0, 1.0]),
        ViewKind::Left => ([1.0, 0.0, 0.0], [0.0, 0.0, 1.0]),
        ViewKind::Iso => ([-1.0, 1.0, -1.0], [0.0, 0.0, 1.0]),
    };
    let dir = normalize(dir);
    let mut right = normalize(cross(dir, up_world));
    if right == [0.0; 3] {
        right = [1.0, 0.0, 0.0];
    }
    let up = normalize(cross(right, dir));
    Frame { dir, right, up }
}

// --- world geometry -----------------------------------------------------------

struct WorldItem {
    instance: Option<u64>,
    positions: Vec<Vec3>,
    topo: Topo,
}

#[derive(Debug)]
struct Topo {
    /// Welded positions. `tris` and `edges` index this array, not the
    /// entity's original (crease-duplicated) vertex buffer.
    positions: Vec<Vec3>,
    tris: Vec<[u32; 3]>,
    edges: Vec<TopoEdge>,
}

#[derive(Debug, Clone, Copy)]
struct TopoEdge {
    a: u32,
    b: u32,
    n1: Vec3,
    n2: Vec3,
    /// Triangles that use this edge, including zero-area seam triangles.
    count: usize,
    /// Non-degenerate face normals available (0, 1 or 2). A degenerate
    /// neighbour still closes the edge pairing so the seam does not become a
    /// false boundary.
    normals: usize,
}

fn build_topo(positions: &[Vec3], indices: &[u32]) -> Topo {
    // Weld by exact f32 bits: the same rule the boolean kernel and sharp-edge
    // extraction use, so a seam does not become a false boundary.
    let mut map: BTreeMap<[u32; 3], u32> = BTreeMap::new();
    let mut welded: Vec<Vec3> = Vec::new();
    let mut remap: Vec<u32> = Vec::with_capacity(positions.len());
    for p in positions {
        let key = [
            (p[0] as f32).to_bits(),
            (p[1] as f32).to_bits(),
            (p[2] as f32).to_bits(),
        ];
        let next = welded.len() as u32;
        let id = *map.entry(key).or_insert_with(|| {
            welded.push(*p);
            next
        });
        remap.push(id);
    }

    let mut tris = Vec::new();
    struct EdgeRec {
        count: usize,
        normals: Vec<Vec3>,
    }
    let mut edges: BTreeMap<(u32, u32), EdgeRec> = BTreeMap::new();
    for tri in indices.chunks_exact(3) {
        let (i0, i1, i2) = (
            remap[tri[0] as usize],
            remap[tri[1] as usize],
            remap[tri[2] as usize],
        );
        if i0 == i1 || i1 == i2 || i2 == i0 {
            continue;
        }
        let p = [
            welded[i0 as usize],
            welded[i1 as usize],
            welded[i2 as usize],
        ];
        let n = normalize(cross(
            [p[1][0] - p[0][0], p[1][1] - p[0][1], p[1][2] - p[0][2]],
            [p[2][0] - p[0][0], p[2][1] - p[0][1], p[2][2] - p[0][2]],
        ));
        let degenerate = dot(n, n) < 0.5;
        if !degenerate {
            // Hidden-line tests use real faces only; zero-area slivers add
            // nothing to the image.
            tris.push([i0, i1, i2]);
        }
        for (a, b) in [(i0, i1), (i1, i2), (i2, i0)] {
            let key = if a < b { (a, b) } else { (b, a) };
            let entry = edges.entry(key).or_insert(EdgeRec {
                count: 0,
                normals: Vec::new(),
            });
            entry.count += 1;
            if !degenerate && entry.normals.len() < 2 {
                entry.normals.push(n);
            }
        }
    }

    let edges = edges
        .into_iter()
        .map(|((a, b), rec)| TopoEdge {
            a,
            b,
            n1: rec.normals.first().copied().unwrap_or([0.0; 3]),
            n2: rec.normals.get(1).copied().unwrap_or([0.0; 3]),
            count: rec.count,
            normals: rec.normals.len(),
        })
        .collect();
    Topo {
        positions: welded,
        tris,
        edges,
    }
}

fn collect_items(d: &Document) -> Vec<WorldItem> {
    let mut out = Vec::new();
    for e in d.entities().values() {
        if matches!(e.shape, crate::Shape::Sketch { .. }) || e.mesh.positions.is_empty() {
            continue;
        }
        let model: Mat4 = e.transform();
        let positions: Vec<Vec3> = e
            .mesh
            .positions
            .iter()
            .map(|&p| {
                let w = mat4::transform_point(&model, p);
                [w[0] as f64, w[1] as f64, w[2] as f64]
            })
            .collect();
        let topo = build_topo(&positions, &e.mesh.indices);
        if topo.tris.is_empty() {
            continue;
        }
        let instance = match &e.shape {
            crate::Shape::PartMember { instance, .. } => Some(*instance),
            _ => None,
        };
        out.push(WorldItem {
            instance,
            positions,
            topo,
        });
    }
    out
}

// --- hidden-line removal ------------------------------------------------------

struct ProjectedTri {
    uv: [[f64; 2]; 3],
    depth: [f64; 3],
}

struct Grid {
    lo: [f64; 2],
    cell: f64,
    nx: usize,
    ny: usize,
    cells: Vec<Vec<u32>>,
}

impl Grid {
    fn build(tris: &[ProjectedTri]) -> Grid {
        let mut lo = [f64::MAX; 2];
        let mut hi = [f64::MIN; 2];
        for t in tris {
            for p in t.uv {
                lo[0] = lo[0].min(p[0]);
                lo[1] = lo[1].min(p[1]);
                hi[0] = hi[0].max(p[0]);
                hi[1] = hi[1].max(p[1]);
            }
        }
        if !lo[0].is_finite() {
            return Grid {
                lo: [0.0; 2],
                cell: 1.0,
                nx: 1,
                ny: 1,
                cells: vec![Vec::new()],
            };
        }
        let extent = (hi[0] - lo[0]).max(hi[1] - lo[1]).max(1.0);
        let cell = (extent / 160.0).max(0.5);
        let nx = (((hi[0] - lo[0]) / cell).floor() as usize + 1).max(1);
        let ny = (((hi[1] - lo[1]) / cell).floor() as usize + 1).max(1);
        let mut cells = vec![Vec::new(); nx * ny];
        for (i, t) in tris.iter().enumerate() {
            let mut tlo = [f64::MAX; 2];
            let mut thi = [f64::MIN; 2];
            for p in t.uv {
                tlo[0] = tlo[0].min(p[0]);
                tlo[1] = tlo[1].min(p[1]);
                thi[0] = thi[0].max(p[0]);
                thi[1] = thi[1].max(p[1]);
            }
            let x0 = (((tlo[0] - lo[0]) / cell).floor().max(0.0) as usize).min(nx - 1);
            let x1 = (((thi[0] - lo[0]) / cell).floor().max(0.0) as usize).min(nx - 1);
            let y0 = (((tlo[1] - lo[1]) / cell).floor().max(0.0) as usize).min(ny - 1);
            let y1 = (((thi[1] - lo[1]) / cell).floor().max(0.0) as usize).min(ny - 1);
            for y in y0..=y1 {
                for x in x0..=x1 {
                    cells[y * nx + x].push(i as u32);
                }
            }
        }
        Grid {
            lo,
            cell,
            nx,
            ny,
            cells,
        }
    }

    fn cell_of(&self, p: [f64; 2]) -> (usize, usize) {
        let x = (((p[0] - self.lo[0]) / self.cell).floor().max(0.0) as usize).min(self.nx - 1);
        let y = (((p[1] - self.lo[1]) / self.cell).floor().max(0.0) as usize).min(self.ny - 1);
        (x, y)
    }

    fn candidates(&self, p: [f64; 2]) -> &[u32] {
        let (x, y) = self.cell_of(p);
        &self.cells[y * self.nx + x]
    }
}

/// Barycentric coverage of a projected point. Returns the three weights when
/// the point is inside (with a small tolerance for samples on an edge).
fn barycentric(t: &ProjectedTri, p: [f64; 2]) -> Option<[f64; 3]> {
    let a = t.uv[0];
    let b = t.uv[1];
    let c = t.uv[2];
    let area = (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]);
    if area.abs() < 1e-12 {
        return None;
    }
    let w0 = ((b[0] - p[0]) * (c[1] - p[1]) - (b[1] - p[1]) * (c[0] - p[0])) / area;
    let w1 = ((c[0] - p[0]) * (a[1] - p[1]) - (c[1] - p[1]) * (a[0] - p[0])) / area;
    let w2 = 1.0 - w0 - w1;
    let eps = -1e-9;
    if w0 >= eps && w1 >= eps && w2 >= eps {
        Some([w0, w1, w2])
    } else {
        None
    }
}

struct Hlr {
    tris: Vec<ProjectedTri>,
    grid: Grid,
    eps: f64,
}

impl Hlr {
    fn new(items: &[WorldItem], frame: &Frame) -> Hlr {
        let mut tris = Vec::new();
        let mut diag = 1.0f64;
        for item in items {
            for tri in &item.topo.tris {
                let p = [
                    item.topo.positions[tri[0] as usize],
                    item.topo.positions[tri[1] as usize],
                    item.topo.positions[tri[2] as usize],
                ];
                let uv = [frame.uv(p[0]), frame.uv(p[1]), frame.uv(p[2])];
                let depth = [frame.depth(p[0]), frame.depth(p[1]), frame.depth(p[2])];
                let (lo, hi) = uv_bounds(&uv);
                diag = diag.max((hi[0] - lo[0]).abs()).max((hi[1] - lo[1]).abs());
                tris.push(ProjectedTri { uv, depth });
            }
        }
        let eps = (diag * 1e-4).max(1e-3);
        let grid = Grid::build(&tris);
        Hlr { tris, grid, eps }
    }

    fn visible(&self, uv: [f64; 2], depth: f64) -> bool {
        for &i in self.grid.candidates(uv) {
            let t = &self.tris[i as usize];
            if let Some(w) = barycentric(t, uv) {
                let d = w[0] * t.depth[0] + w[1] * t.depth[1] + w[2] * t.depth[2];
                if d < depth - self.eps {
                    return false;
                }
            }
        }
        true
    }
}

fn uv_bounds(uv: &[[f64; 2]; 3]) -> ([f64; 2], [f64; 2]) {
    let mut lo = [f64::MAX; 2];
    let mut hi = [f64::MIN; 2];
    for p in uv {
        lo[0] = lo[0].min(p[0]);
        lo[1] = lo[1].min(p[1]);
        hi[0] = hi[0].max(p[0]);
        hi[1] = hi[1].max(p[1]);
    }
    (lo, hi)
}

// --- views --------------------------------------------------------------------

struct ViewBuilt {
    kind: ViewKind,
    /// Sheet origin of the projected bbox corner (set by layout).
    origin: [f64; 2],
    /// Projected bbox size in sheet millimeters.
    size: [f64; 2],
    /// Projected bbox size in model millimeters (dimension text).
    model_size: [f64; 2],
    visible: Vec<Vec<[f64; 2]>>,
    hidden: Vec<Vec<[f64; 2]>>,
    /// Bounds in model-view coordinates, for balloons.
    min_uv: [f64; 2],
    scale: f64,
}

/// Drop interior points of a straight run. Hidden-line sampling splits every
/// edge into many short segments; a visible straight edge should reach the
/// writer as two points.
fn simplify(points: Vec<[f64; 2]>) -> Vec<[f64; 2]> {
    if points.len() <= 2 {
        return points;
    }
    let mut out = Vec::with_capacity(points.len());
    out.push(points[0]);
    for i in 1..points.len() - 1 {
        let a = out[out.len() - 1];
        let b = points[i];
        let c = points[i + 1];
        let ab = [b[0] - a[0], b[1] - a[1]];
        let bc = [c[0] - b[0], c[1] - b[1]];
        let cross = ab[0] * bc[1] - ab[1] * bc[0];
        let dot = ab[0] * bc[0] + ab[1] * bc[1];
        let scale = (ab[0] * ab[0] + ab[1] * ab[1]).sqrt() * (bc[0] * bc[0] + bc[1] * bc[1]).sqrt();
        if cross.abs() > 1e-7 * scale.max(1.0) || dot < 0.0 {
            out.push(b);
        }
    }
    out.push(points[points.len() - 1]);
    out
}

fn build_view(items: &[WorldItem], spec: &DrawingViewSpec, scale: f64, hidden: bool) -> ViewBuilt {
    let frame = frame(spec.view);
    let mut lo = [f64::MAX; 2];
    let mut hi = [f64::MIN; 2];
    for item in items {
        for p in &item.positions {
            let uv = frame.uv(*p);
            lo[0] = lo[0].min(uv[0]);
            lo[1] = lo[1].min(uv[1]);
            hi[0] = hi[0].max(uv[0]);
            hi[1] = hi[1].max(uv[1]);
        }
    }
    if !lo[0].is_finite() {
        lo = [0.0; 2];
        hi = [0.0; 2];
    }
    let min_uv = lo;
    let model_size = [hi[0] - lo[0], hi[1] - lo[1]];
    let size = [model_size[0] * scale, model_size[1] * scale];

    let hlr = Hlr::new(items, &frame);
    let mut visible = Vec::new();
    let mut hidden_runs = Vec::new();

    for item in items {
        let mut seen: BTreeSet<(u32, u32)> = BTreeSet::new();
        for edge in &item.topo.edges {
            let key = if edge.a < edge.b {
                (edge.a, edge.b)
            } else {
                (edge.b, edge.a)
            };
            if !seen.insert(key) {
                continue;
            }
            let sharp = match edge.normals {
                2 => dot(edge.n1, edge.n2).clamp(-1.0, 1.0).acos().to_degrees() >= SHARP_DEG,
                _ => edge.count == 1,
            };
            let silhouette = edge.normals == 2 && {
                let a = dot(edge.n1, frame.dir);
                let b = dot(edge.n2, frame.dir);
                (a > 1e-9 && b < -1e-9) || (a < -1e-9 && b > 1e-9)
            };
            if !(sharp || silhouette) {
                continue;
            }
            let pa = item.topo.positions[edge.a as usize];
            let pb = item.topo.positions[edge.b as usize];
            let uva = frame.uv(pa);
            let uvb = frame.uv(pb);
            let len = ((uvb[0] - uva[0]).powi(2) + (uvb[1] - uva[1]).powi(2)).sqrt();
            let steps = ((len * scale / 0.7).ceil() as usize).clamp(2, 96);
            let mut vis = Vec::new();
            for i in 0..=steps {
                let t = i as f64 / steps as f64;
                let p = [
                    pa[0] + (pb[0] - pa[0]) * t,
                    pa[1] + (pb[1] - pa[1]) * t,
                    pa[2] + (pb[2] - pa[2]) * t,
                ];
                let uv = frame.uv(p);
                let d = frame.depth(p);
                vis.push((uv, hlr.visible(uv, d)));
            }
            let mut run: Vec<[f64; 2]> = Vec::new();
            let mut run_visible = false;
            for (i, (uv, v)) in vis.iter().enumerate() {
                if i > 0 && *v != run_visible {
                    // Keep the boundary sample in both runs so the split sits
                    // exactly on the visibility change.
                    if run.len() >= 2 {
                        if run_visible {
                            visible.push(std::mem::take(&mut run));
                        } else {
                            hidden_runs.push(std::mem::take(&mut run));
                        }
                    } else {
                        run.clear();
                    }
                }
                run.push(*uv);
                run_visible = *v;
            }
            if run.len() >= 2 {
                if run_visible {
                    visible.push(run);
                } else {
                    hidden_runs.push(run);
                }
            }
        }
    }

    // Local sheet coordinates: bbox min at zero.
    let to_local = |p: [f64; 2]| [(p[0] - min_uv[0]) * scale, (p[1] - min_uv[1]) * scale];
    let visible = visible
        .into_iter()
        .map(|run| simplify(run.into_iter().map(to_local).collect()))
        .filter(|run: &Vec<[f64; 2]>| run.len() >= 2)
        .collect();
    let hidden = if hidden {
        hidden_runs
            .into_iter()
            .map(|run| simplify(run.into_iter().map(to_local).collect()))
            .filter(|run: &Vec<[f64; 2]>| run.len() >= 2)
            .collect()
    } else {
        Vec::new()
    };

    ViewBuilt {
        kind: spec.view,
        origin: [0.0; 2],
        size,
        model_size,
        visible,
        hidden,
        min_uv,
        scale,
    }
}

fn layout(views: &mut [ViewBuilt], specs: &[DrawingViewSpec]) {
    // Explicit placements first.
    for (v, spec) in views.iter_mut().zip(specs.iter()) {
        if let Some(at) = spec.at {
            v.origin = [at[0] - v.size[0] * 0.5, at[1] - v.size[1] * 0.5];
        }
    }
    let front = views.iter().position(|v| v.kind == ViewKind::Front);
    let Some(front) = front else {
        // No front view: lay views out in a row.
        let mut x = 0.0;
        for v in views.iter_mut() {
            if let Some(_) = specs.iter().find(|s| s.view == v.kind).and_then(|s| s.at) {
                continue;
            }
            v.origin = [x, 0.0];
            x += v.size[0] + VIEW_GAP;
        }
        return;
    };
    let (fw, fh) = (views[front].size[0], views[front].size[1]);
    let (fx, fy) = (views[front].origin[0], views[front].origin[1]);
    let mut max_right = fx + fw;
    for (i, v) in views.iter_mut().enumerate() {
        if i == front || specs[i].at.is_some() {
            max_right = max_right.max(v.origin[0] + v.size[0]);
            continue;
        }
        match v.kind {
            ViewKind::Top => v.origin = [fx + (fw - v.size[0]) * 0.5, fy + fh + VIEW_GAP],
            ViewKind::Right => v.origin = [fx + fw + VIEW_GAP, fy + (fh - v.size[1]) * 0.5],
            ViewKind::Iso => v.origin = [fx + fw + VIEW_GAP, fy + fh + VIEW_GAP],
            _ => {
                v.origin = [max_right + VIEW_GAP, fy];
            }
        }
        max_right = max_right.max(v.origin[0] + v.size[0]);
    }
}

// --- sheet furniture ----------------------------------------------------------

fn line(d: &mut Drawing, layer: usize, a: [f64; 2], b: [f64; 2]) {
    if (a[0] - b[0]).abs() < 1e-9 && (a[1] - b[1]).abs() < 1e-9 {
        return;
    }
    d.curves.push(Curve {
        layer,
        kind: CurveKind::Line { a, b },
    });
}

fn polyline(d: &mut Drawing, layer: usize, points: &[[f64; 2]]) {
    if points.len() >= 2 {
        d.curves.push(Curve {
            layer,
            kind: CurveKind::Polyline {
                points: points.to_vec(),
                closed: false,
            },
        });
    }
}

fn text(d: &mut Drawing, layer: usize, at: [f64; 2], s: impl Into<String>, height: f64) {
    d.curves.push(Curve {
        layer,
        kind: CurveKind::Text {
            at,
            text: s.into(),
            height,
        },
    });
}

fn centered_text(d: &mut Drawing, layer: usize, center: [f64; 2], s: &str, height: f64) {
    let w = drawing::text_width(s, height);
    text(d, layer, [center[0] - w * 0.5, center[1]], s, height);
}

fn rect(d: &mut Drawing, layer: usize, lo: [f64; 2], hi: [f64; 2]) {
    line(d, layer, lo, [hi[0], lo[1]]);
    line(d, layer, [hi[0], lo[1]], hi);
    line(d, layer, hi, [lo[0], hi[1]]);
    line(d, layer, [lo[0], hi[1]], lo);
}

fn arrowhead(d: &mut Drawing, layer: usize, tip: [f64; 2], dir: [f64; 2]) {
    // Two short barbs behind the tip, 25° off the line.
    let len = 1.8;
    let angle = 25.0f64.to_radians();
    for s in [-1.0, 1.0] {
        let ca = (angle * s).cos();
        let sa = (angle * s).sin();
        let vx = -(dir[0] * ca - dir[1] * sa);
        let vy = -(dir[0] * sa + dir[1] * ca);
        line(d, layer, tip, [tip[0] + vx * len, tip[1] + vy * len]);
    }
}

fn dimension_text(v: f64) -> String {
    // Two decimals are plenty for a drawing, and trailing zeros are trimmed.
    let rounded = (v * 100.0).round() / 100.0;
    drawing::number(rounded)
}

fn horizontal_dimension(d: &mut Drawing, layer: usize, x0: f64, x1: f64, y: f64, value: f64) {
    let lo = x0.min(x1);
    let hi = x0.max(x1);
    if hi - lo < 1e-6 {
        return;
    }
    line(d, layer, [lo, y], [hi, y]);
    // Extension lines rise toward the geometry.
    line(d, layer, [lo, y - 2.0], [lo, y + 2.0]);
    line(d, layer, [hi, y - 2.0], [hi, y + 2.0]);
    arrowhead(d, layer, [lo, y], [1.0, 0.0]);
    arrowhead(d, layer, [hi, y], [-1.0, 0.0]);
    let s = dimension_text(value);
    let w = drawing::text_width(&s, 3.0);
    text(d, layer, [(lo + hi) * 0.5 - w * 0.5, y + 1.2], s, 3.0);
}

fn vertical_dimension(d: &mut Drawing, layer: usize, y0: f64, y1: f64, x: f64, value: f64) {
    let lo = y0.min(y1);
    let hi = y0.max(y1);
    if hi - lo < 1e-6 {
        return;
    }
    line(d, layer, [x, lo], [x, hi]);
    line(d, layer, [x - 2.0, lo], [x + 2.0, lo]);
    line(d, layer, [x - 2.0, hi], [x + 2.0, hi]);
    arrowhead(d, layer, [x, lo], [0.0, 1.0]);
    arrowhead(d, layer, [x, hi], [0.0, -1.0]);
    let s = dimension_text(value);
    let w = drawing::text_width(&s, 3.0);
    text(d, layer, [x - 3.0 - w, (lo + hi) * 0.5 - 1.5], s, 3.0);
}

fn add_view_label(d: &mut Drawing, layer: usize, v: &ViewBuilt) {
    let label = v.kind.as_str().to_uppercase();
    text(
        d,
        layer,
        [v.origin[0], v.origin[1] + v.size[1] + 2.5],
        label,
        3.5,
    );
}

fn scale_label(scale: f64) -> String {
    if (scale - 1.0).abs() < 1e-9 {
        "1:1".into()
    } else if scale > 1.0 {
        format!("{}:1", drawing::number(scale))
    } else {
        format!("1:{}", drawing::number(1.0 / scale))
    }
}

fn sheet_bbox(d: &Drawing) -> Option<([f64; 2], [f64; 2])> {
    d.bounds()
}

// --- public entry points ------------------------------------------------------

pub fn build(d: &Document, opts: &SheetOptions<'_>) -> Result<(Drawing, usize)> {
    let items = collect_items(d);
    if items.is_empty() {
        return Err(Error::Invalid(
            "nothing to draw: the document has no printable solids (hint: extrude a sketch \
             or add a solid first)"
                .into(),
        ));
    }
    let scale = opts.scale.unwrap_or(1.0);
    if !scale.is_finite() || scale <= 0.0 {
        return Err(Error::Invalid("drawing scale must be > 0".into()));
    }
    let default_hidden = true;
    let hidden = opts.hidden_lines.unwrap_or(default_hidden);
    let default_dims = true;
    let dimensions = opts.dimensions.unwrap_or(default_dims);
    let has_instances = !d.instances().is_empty();
    let balloons = opts.balloons.unwrap_or(has_instances);
    let bom = opts.bom.unwrap_or(has_instances);

    let default_specs: Vec<DrawingViewSpec> = [
        ViewKind::Front,
        ViewKind::Top,
        ViewKind::Right,
        ViewKind::Iso,
    ]
    .iter()
    .map(|&view| DrawingViewSpec {
        view,
        at: None,
        dimensions: None,
    })
    .collect();
    let specs: &[DrawingViewSpec] = opts.views.unwrap_or(&default_specs);
    if specs.is_empty() {
        return Err(Error::Invalid("the drawing needs at least one view".into()));
    }

    let mut views: Vec<ViewBuilt> = specs
        .iter()
        .map(|spec| build_view(&items, spec, scale, hidden))
        .collect();
    layout(&mut views, specs);

    let mut title = opts.title.unwrap_or(&d.name).to_string();
    if title.chars().count() > 44 {
        title = title.chars().take(41).collect::<String>() + "...";
    }
    let mut out = Drawing::new(format!("{title} - drawing"));
    // Hidden lines first: the visible layer then paints over coincident
    // tangent seams instead of a dashed line landing on top of a solid one.
    let l_hidden = out.layer("hidden", Role::Hidden);
    let l_visible = out.layer("visible", Role::Visible);
    let l_dim = out.layer("dimensions", Role::Dimension);
    let l_note = out.layer("annotations", Role::Annotation);
    let l_sheet = out.layer("sheet", Role::Sheet);
    let l_bom = out.layer("bom", Role::Bom);

    for v in &views {
        for run in &v.visible {
            let pts: Vec<[f64; 2]> = run
                .iter()
                .map(|p| [v.origin[0] + p[0], v.origin[1] + p[1]])
                .collect();
            polyline(&mut out, l_visible, &pts);
        }
        for run in &v.hidden {
            let pts: Vec<[f64; 2]> = run
                .iter()
                .map(|p| [v.origin[0] + p[0], v.origin[1] + p[1]])
                .collect();
            polyline(&mut out, l_hidden, &pts);
        }
        add_view_label(&mut out, l_note, v);
        let want_dims = specs
            .iter()
            .find(|s| s.view == v.kind)
            .and_then(|s| s.dimensions)
            .unwrap_or(dimensions);
        if want_dims && v.kind.is_orthographic() {
            let (x, y) = (v.origin[0], v.origin[1]);
            horizontal_dimension(
                &mut out,
                l_dim,
                x,
                x + v.size[0],
                y - DIM_OFFSET,
                v.model_size[0],
            );
            vertical_dimension(
                &mut out,
                l_dim,
                y,
                y + v.size[1],
                x - DIM_OFFSET,
                v.model_size[1],
            );
        }
    }

    let rows = assembly::bill_of_materials(d);
    let item_of: BTreeMap<&str, u32> = rows.iter().map(|r| (r.part.as_str(), r.item)).collect();

    // Balloons in the first orthographic view. Anchors are projected
    // instance bboxes; balloons are staggered horizontally so two stacked
    // instances do not hide each other.
    if balloons && !rows.is_empty() {
        if let Some(v) = views.iter().find(|v| v.kind.is_orthographic()) {
            let f = frame(v.kind);
            let mut balloons_found = Vec::new();
            for (iid, inst) in d.instances() {
                let mut lo = [f64::MAX; 3];
                let mut hi = [f64::MIN; 3];
                let mut any = false;
                for item in items.iter().filter(|i| i.instance == Some(*iid)) {
                    for p in &item.positions {
                        any = true;
                        for k in 0..3 {
                            lo[k] = lo[k].min(p[k]);
                            hi[k] = hi[k].max(p[k]);
                        }
                    }
                }
                if !any {
                    continue;
                }
                let mut uv_lo = [f64::MAX; 2];
                let mut uv_hi = [f64::MIN; 2];
                for cx in [lo[0], hi[0]] {
                    for cy in [lo[1], hi[1]] {
                        for cz in [lo[2], hi[2]] {
                            let uv = f.uv([cx, cy, cz]);
                            uv_lo[0] = uv_lo[0].min(uv[0]);
                            uv_lo[1] = uv_lo[1].min(uv[1]);
                            uv_hi[0] = uv_hi[0].max(uv[0]);
                            uv_hi[1] = uv_hi[1].max(uv[1]);
                        }
                    }
                }
                let cx = (uv_lo[0] + uv_hi[0]) * 0.5;
                let top = uv_hi[1];
                let sheet = |p: [f64; 2]| {
                    [
                        v.origin[0] + (p[0] - v.min_uv[0]) * v.scale,
                        v.origin[1] + (p[1] - v.min_uv[1]) * v.scale,
                    ]
                };
                let anchor = sheet([cx, top]);
                let item = item_of.get(inst.part.as_str()).copied().unwrap_or(0);
                balloons_found.push((*iid, anchor, item));
            }
            balloons_found.sort_by(|a, b| {
                a.1[0]
                    .partial_cmp(&b.1[0])
                    .unwrap_or(std::cmp::Ordering::Equal)
                    .then(a.0.cmp(&b.0))
            });
            let radius = 3.2;
            let mut last_x = f64::MIN;
            let floor = v.origin[0] + v.size[0] * 0.5 + 4.0;
            for (_, anchor, item) in balloons_found {
                let mut cx = anchor[0].max(floor);
                if last_x.is_finite() && cx < last_x + radius * 2.4 {
                    cx = last_x + radius * 2.4;
                }
                last_x = cx;
                let center = [cx, anchor[1] + 12.0];
                let mut circle = Vec::with_capacity(17);
                for k in 0..16 {
                    let a = k as f64 / 16.0 * std::f64::consts::TAU;
                    circle.push([center[0] + radius * a.cos(), center[1] + radius * a.sin()]);
                }
                circle.push(circle[0]);
                polyline(&mut out, l_note, &circle);
                line(&mut out, l_note, anchor, [center[0], center[1] - radius]);
                centered_text(
                    &mut out,
                    l_note,
                    [center[0], center[1] - 1.4],
                    &item.to_string(),
                    3.2,
                );
            }
        }
    }

    for a in opts.annotations {
        if a.text.trim().is_empty() {
            continue;
        }
        text(&mut out, l_note, a.at, &a.text, 3.5);
    }

    // Content bounds (views, dimensions, labels, balloons, notes).
    let (content_lo, content_hi) = sheet_bbox(&out).unwrap_or(([0.0; 2], [100.0; 2]));

    let bom_rows_h = if bom && !rows.is_empty() {
        6.0 * (rows.len() as f64 + 1.0) + 2.0
    } else {
        0.0
    };
    let right = content_hi[0] + MARGIN;
    let left = (content_lo[0] - MARGIN).min(right - TITLE_W - MARGIN * 2.0);
    let bottom = content_lo[1] - MARGIN - TITLE_H - bom_rows_h - 8.0;
    let top = content_hi[1] + MARGIN;
    rect(&mut out, l_sheet, [left, bottom], [right, top]);

    // Title block, bottom-right.
    let tx = right - MARGIN - TITLE_W;
    let ty = bottom + 8.0;
    rect(&mut out, l_sheet, [tx, ty], [tx + TITLE_W, ty + TITLE_H]);
    line(
        &mut out,
        l_sheet,
        [tx, ty + TITLE_H - 9.0],
        [tx + TITLE_W, ty + TITLE_H - 9.0],
    );
    line(
        &mut out,
        l_sheet,
        [tx + 78.0, ty],
        [tx + 78.0, ty + TITLE_H - 9.0],
    );
    text(
        &mut out,
        l_sheet,
        [tx + 3.0, ty + TITLE_H - 4.0],
        &title,
        4.2,
    );
    text(&mut out, l_sheet, [tx + 3.0, ty + 5.5], "WRIGHT CAD", 3.0);
    text(
        &mut out,
        l_sheet,
        [tx + 82.0, ty + 11.0],
        format!("SCALE {}", scale_label(scale)),
        3.0,
    );
    text(&mut out, l_sheet, [tx + 82.0, ty + 5.5], "UNITS mm", 3.0);

    // BOM table above the title block, right-aligned.
    if bom && !rows.is_empty() {
        let cols = [14.0, 66.0, 14.0];
        let table_w: f64 = cols.iter().sum();
        let bx = right - MARGIN - table_w;
        let by = ty + TITLE_H + 4.0;
        let mut y = by + 6.0 * (rows.len() as f64 + 1.0);
        // Header at the top, rows below it.
        line(&mut out, l_bom, [bx, y], [bx + table_w, y]);
        let mut x = bx;
        for (col, name) in cols.iter().zip(["ITEM", "PART", "QTY"]) {
            text(&mut out, l_bom, [x + 1.5, y - 4.0], name, 2.8);
            x += col;
            line(&mut out, l_bom, [x, y], [x, y - 6.0]);
        }
        y -= 6.0;
        for row in &rows {
            text(
                &mut out,
                l_bom,
                [bx + 1.5, y - 4.0],
                row.item.to_string(),
                2.8,
            );
            let mut part = row.part.clone();
            if part.chars().count() > 30 {
                part = part.chars().take(27).collect::<String>() + "...";
            }
            text(&mut out, l_bom, [bx + cols[0] + 1.5, y - 4.0], part, 2.8);
            text(
                &mut out,
                l_bom,
                [bx + cols[0] + cols[1] + 1.5, y - 4.0],
                row.quantity.to_string(),
                2.8,
            );
            y -= 6.0;
            line(&mut out, l_bom, [bx, y + 6.0], [bx + table_w, y + 6.0]);
        }
        line(&mut out, l_bom, [bx, by], [bx + table_w, by]);
        line(
            &mut out,
            l_bom,
            [bx, by],
            [bx, by + 6.0 * (rows.len() as f64 + 1.0)],
        );
        line(
            &mut out,
            l_bom,
            [bx + table_w, by],
            [bx + table_w, by + 6.0 * (rows.len() as f64 + 1.0)],
        );
    }

    Ok((out, views.len()))
}

pub fn export(
    d: &Document,
    doc: DocId,
    path: &str,
    format: DrawingFormat,
    opts: SheetOptions<'_>,
) -> Result<CommandResult> {
    let (drawing, views) = build(d, &opts)?;
    let curves = drawing.curves.len();
    let (resolved, bytes) = match format {
        DrawingFormat::Svg => crate::svg::write_svg(path, &drawing)?,
        DrawingFormat::Dxf => crate::dxf::write_dxf(path, &drawing)?,
    };
    Ok(CommandResult::DrawingExported {
        doc,
        path: resolved,
        format: format.as_str().to_string(),
        views,
        curves,
        bytes,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::atomic::{AtomicUsize, Ordering};

    use wright_protocol::{Command, Scalar};

    use crate::Engine;

    static NEXT_FILE: AtomicUsize = AtomicUsize::new(0);

    fn temp_file(tag: &str, ext: &str) -> String {
        let n = NEXT_FILE.fetch_add(1, Ordering::Relaxed);
        std::env::temp_dir()
            .join(format!(
                "wright_m19_draw_{tag}_{}_{n}.{ext}",
                std::process::id()
            ))
            .to_string_lossy()
            .into_owned()
    }

    fn create(e: &mut Engine, name: &str) -> DocId {
        match e
            .execute(Command::CreateDocument {
                name: name.into(),
                doc: None,
            })
            .unwrap()
        {
            CommandResult::DocumentCreated { doc } => doc,
            r => panic!("unexpected {r:?}"),
        }
    }

    fn add_box(e: &mut Engine, doc: DocId, name: &str, center: [f64; 3], size: [f64; 3]) {
        e.execute(Command::AddBox {
            doc,
            name: name.into(),
            center: center.map(Scalar::Number),
            size: size.map(Scalar::Number),
            color: None,
        })
        .unwrap();
    }

    fn export(
        e: &mut Engine,
        doc: DocId,
        path: &str,
        format: DrawingFormat,
        hidden: Option<bool>,
    ) -> CommandResult {
        e.execute(Command::ExportDrawing {
            doc,
            path: path.into(),
            format,
            views: None,
            scale: None,
            hidden_lines: hidden,
            dimensions: None,
            balloons: None,
            bom: None,
            title: Some("Test sheet".into()),
            annotations: None,
        })
        .unwrap()
    }

    #[test]
    fn a_box_drawing_has_views_dimensions_and_a_title_block() {
        let mut e = Engine::new();
        let doc = create(&mut e, "block");
        add_box(&mut e, doc, "block", [0.0, 0.0, 2.5], [20.0, 10.0, 5.0]);
        let path = temp_file("box", "svg");
        let result = export(&mut e, doc, &path, DrawingFormat::Svg, None);
        match result {
            CommandResult::DrawingExported { views, curves, .. } => {
                assert_eq!(views, 4);
                assert!(curves > 20, "expected real geometry, got {curves} curves");
            }
            r => panic!("unexpected {r:?}"),
        }
        let svg = std::fs::read_to_string(&path).unwrap();
        for needle in [
            "data-layer=\"visible\"",
            "data-layer=\"hidden\"",
            "data-layer=\"dimensions\"",
            "data-layer=\"annotations\"",
            "data-layer=\"sheet\"",
            "FRONT",
            "TOP",
            "RIGHT",
            "ISO",
            "Test sheet",
            "SCALE 1:1",
            "UNITS mm",
        ] {
            assert!(svg.contains(needle), "missing {needle} in SVG\n{svg}");
        }
        // Overall dimensions: width 20 appears in more than one view.
        assert!(
            svg.matches(">20</text>").count() >= 2,
            "width dimension missing"
        );
        assert!(svg.contains(">10</text>"), "depth dimension missing");
        assert!(svg.contains(">5</text>"), "height dimension missing");

        // Same scene + options -> identical bytes.
        let path2 = temp_file("box2", "svg");
        export(&mut e, doc, &path2, DrawingFormat::Svg, None);
        assert_eq!(
            std::fs::read(&path).unwrap(),
            std::fs::read(&path2).unwrap()
        );
        let _ = std::fs::remove_file(path);
        let _ = std::fs::remove_file(path2);
    }

    #[test]
    fn crease_duplicated_vertices_do_not_create_diagonal_edges() {
        // A boolean result carries one vertex per triangle corner (for crease
        // normals). Drawing topology must index the *welded* array; using the
        // raw buffer draws long diagonals between unrelated corners.
        let mut e = Engine::new();
        let doc = create(&mut e, "bored plate");
        add_box(&mut e, doc, "body", [0.0, 0.0, 2.5], [60.0, 30.0, 5.0]);
        for x in [-18.0, 18.0] {
            e.execute(Command::AddCylinder {
                doc,
                name: "hole".into(),
                center: [Scalar::Number(x), Scalar::Number(0.0), Scalar::Number(2.5)],
                radius: Scalar::Number(5.0),
                height: Scalar::Number(15.0),
                segments: Some(32),
                color: None,
            })
            .unwrap();
        }
        e.execute(Command::Boolean {
            doc,
            op: wright_protocol::BooleanOp::Difference,
            entities: vec![1, 2, 3],
            name: Some("plate".into()),
            color: None,
        })
        .unwrap();
        for id in [1, 2, 3] {
            e.execute(Command::Delete { doc, entity: id }).unwrap();
        }
        let d = e.doc_for_test(doc);
        let items = collect_items(d);
        assert_eq!(items.len(), 1);
        let spec = DrawingViewSpec {
            view: ViewKind::Top,
            at: None,
            dimensions: None,
        };
        let view = build_view(&items, &spec, 1.0, true);
        for run in view.visible.iter().chain(view.hidden.iter()) {
            let a = run[0];
            let b = run[run.len() - 1];
            let dx = (b[0] - a[0]).abs();
            let dy = (b[1] - a[1]).abs();
            let long = ((dx * dx + dy * dy).sqrt()) > 12.0;
            assert!(
                !long || dx < 1e-6 || dy < 1e-6,
                "unexpected diagonal edge {a:?} -> {b:?} (dx={dx}, dy={dy})"
            );
        }
    }

    #[test]
    fn hidden_lines_can_be_turned_off() {
        let mut e = Engine::new();
        let doc = create(&mut e, "hidden");
        add_box(&mut e, doc, "a", [0.0, 0.0, 2.5], [20.0, 10.0, 5.0]);
        add_box(&mut e, doc, "b", [10.0, 0.0, 7.5], [20.0, 10.0, 5.0]);
        let on = temp_file("hidden_on", "svg");
        let off = temp_file("hidden_off", "svg");
        let on_bytes = match export(&mut e, doc, &on, DrawingFormat::Svg, Some(true)) {
            CommandResult::DrawingExported { bytes, .. } => bytes,
            r => panic!("unexpected {r:?}"),
        };
        let off_bytes = match export(&mut e, doc, &off, DrawingFormat::Svg, Some(false)) {
            CommandResult::DrawingExported { bytes, .. } => bytes,
            r => panic!("unexpected {r:?}"),
        };
        assert!(
            off_bytes < on_bytes,
            "hiding hidden lines should shorten the sheet"
        );
        let svg = std::fs::read_to_string(&off).unwrap();
        assert!(svg.contains("data-layer=\"hidden\""));
        // The hidden group is empty when the option is off.
        let start = svg.find("data-layer=\"hidden\"").unwrap();
        let end = svg[start..].find("</g>").unwrap() + start;
        let group = &svg[start..end];
        assert!(
            !group.contains("<path") && !group.contains("<line"),
            "{group}"
        );
        let _ = std::fs::remove_file(on);
        let _ = std::fs::remove_file(off);
    }

    #[test]
    fn dxf_drawing_carries_text_and_named_layers() {
        let mut e = Engine::new();
        let doc = create(&mut e, "dxf block");
        add_box(&mut e, doc, "block", [0.0, 0.0, 2.5], [20.0, 10.0, 5.0]);
        let path = temp_file("box", "dxf");
        export(&mut e, doc, &path, DrawingFormat::Dxf, None);
        let dxf = std::fs::read_to_string(&path).unwrap();
        assert!(dxf.contains("\nTEXT\n"));
        assert!(dxf.contains("Test sheet"));
        assert!(dxf.contains("\nvisible\n"));
        assert!(dxf.contains("\ndimensions\n"));
        assert!(dxf.contains("\nsheet\n"));
        assert!(dxf.contains("\nFRONT\n"));
        let _ = std::fs::remove_file(path);
    }

    #[test]
    fn assembly_drawings_get_balloons_and_a_bom() {
        let mut e = Engine::new();
        let part = create(&mut e, "screw part");
        add_box(&mut e, part, "body", [0.0, 0.0, 5.0], [6.0, 6.0, 10.0]);
        let asm = create(&mut e, "assembly");
        e.execute(Command::ImportPart {
            doc: asm,
            name: Some("screw".into()),
            path: None,
            from: Some(part),
        })
        .unwrap();
        for (i, name) in ["s1", "s2"].iter().enumerate() {
            e.execute(Command::AddInstance {
                doc: asm,
                part: "screw".into(),
                name: (*name).into(),
                translation: Some([
                    Scalar::Number(i as f64 * 20.0),
                    Scalar::Number(0.0),
                    Scalar::Number(0.0),
                ]),
                rotation_deg: None,
                scale: None,
                parameters: None,
            })
            .unwrap();
        }
        let path = temp_file("assembly", "svg");
        export(&mut e, asm, &path, DrawingFormat::Svg, None);
        let svg = std::fs::read_to_string(&path).unwrap();
        for needle in ["ITEM", "PART", "QTY", "screw", ">2</text>"] {
            assert!(svg.contains(needle), "missing {needle}");
        }
        // Two instances -> two balloons (item 1) in the front view.
        assert!(svg.matches(">1</text>").count() >= 2);
        let _ = std::fs::remove_file(path);
    }

    #[test]
    fn drawing_views_and_annotations_are_configurable() {
        let mut e = Engine::new();
        let doc = create(&mut e, "custom");
        add_box(&mut e, doc, "block", [0.0, 0.0, 2.5], [20.0, 10.0, 5.0]);
        let path = temp_file("custom", "svg");
        e.execute(Command::ExportDrawing {
            doc,
            path: path.clone(),
            format: DrawingFormat::Svg,
            views: Some(vec![
                DrawingViewSpec {
                    view: ViewKind::Front,
                    at: Some([50.0, 50.0]),
                    dimensions: Some(false),
                },
                DrawingViewSpec {
                    view: ViewKind::Iso,
                    at: Some([150.0, 50.0]),
                    dimensions: None,
                },
            ]),
            scale: Some(2.0),
            hidden_lines: Some(false),
            dimensions: None,
            balloons: Some(false),
            bom: Some(false),
            title: Some("Custom".into()),
            annotations: Some(vec![DrawingAnnotation {
                text: "CHECK ALL EDGES".into(),
                at: [20.0, 20.0],
            }]),
        })
        .unwrap();
        let svg = std::fs::read_to_string(&path).unwrap();
        assert!(svg.contains("FRONT"));
        assert!(svg.contains("ISO"));
        assert!(!svg.contains("TOP"));
        assert!(svg.contains("CHECK ALL EDGES"));
        assert!(svg.contains("SCALE 2:1"));
        // The front view's dimensions are disabled, so no 20 dimension text.
        assert!(!svg.contains(">20</text>"));
        let _ = std::fs::remove_file(path);
    }

    #[test]
    fn drawing_is_a_session_command_and_needs_solids() {
        let mut e = Engine::new();
        let doc = create(&mut e, "empty");
        let path = temp_file("empty", "svg");
        let err = e
            .execute(Command::ExportDrawing {
                doc,
                path: path.clone(),
                format: DrawingFormat::Svg,
                views: None,
                scale: None,
                hidden_lines: None,
                dimensions: None,
                balloons: None,
                bom: None,
                title: None,
                annotations: None,
            })
            .unwrap_err()
            .to_string();
        assert!(err.contains("no printable solids"), "{err}");
        assert!(!std::path::Path::new(&path).exists());

        add_box(&mut e, doc, "block", [0.0, 0.0, 2.5], [20.0, 10.0, 5.0]);
        let before = match e.execute(Command::GetProgram { doc }).unwrap() {
            CommandResult::Program { program, .. } => program.len(),
            r => panic!("unexpected {r:?}"),
        };
        export(&mut e, doc, &path, DrawingFormat::Svg, None);
        let after = match e.execute(Command::GetProgram { doc }).unwrap() {
            CommandResult::Program { program, .. } => program.len(),
            r => panic!("unexpected {r:?}"),
        };
        assert_eq!(before, after, "drawing export must not enter the program");
        let _ = std::fs::remove_file(path);
    }
}
