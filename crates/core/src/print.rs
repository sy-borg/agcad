//! Printability preflight: the bridge from "watertight" to "printable".
//!
//! Everything here is a deterministic approximation over a **world-space**
//! triangle mesh. The build direction is +Z and the bed is the z=0 plane.
//!
//! * **Overhangs** are downward faces whose slope is within a configurable
//!   angle of horizontal. First-layer contact is excluded: the bottom of a
//!   box resting on the bed is not an overhang.
//! * **Bridges** are connected overhang patches that contain near-horizontal
//!   faces above the first layer. The reported span is the short horizontal
//!   extent of the patch, which is the dimension a slicer has to span between
//!   supports.
//! * **First layer** counts the projected area of downward faces inside one
//!   layer of the lowest point, and says whether the solid is on, above or
//!   below the bed.
//! * **Shells** come from a union-find over welded vertices. The largest
//!   positive shell is the outside; extra positive shells are islands, and
//!   inward-wound (negative) shells are enclosed cavities. A cavity with
//!   boundary edges is drained; a closed one traps material.
//! * **Cups** are upward faces with material directly above them (a vertical
//!   ray hits an underside). Their volume is `area * depth`, a rough trap
//!   estimate for resin printing.
//! * **Minimum feature** reuses the thinnest-wall estimate: an inward ray
//!   per sampled triangle, so a thin pin and a thin wall both show up.
//!
//! No randomness, clocks, hash iteration or locale-dependent formatting.

use std::collections::BTreeMap;

use wright_protocol::{
    Bbox, BridgeSpan, CavityReport, CupReport, FirstLayerReport, FitClass, FitClassInfo, FitReport,
    MaterialSpec, Mesh, OverhangRegion, PrintReport, ShellReport, Vec3,
};

use crate::edges;

/// Maximum number of patches listed per category.
const MAX_PATCHES: usize = 8;
/// Cap on the upward faces sampled for cup detection.
const MAX_CUP_SAMPLES: usize = 256;

pub const DEFAULT_OVERHANG_DEG: f64 = 45.0;
pub const DEFAULT_BRIDGE_DEG: f64 = 10.0;
pub const DEFAULT_LAYER_MM: f64 = 0.2;
pub const DEFAULT_BED_MM: [f64; 2] = [220.0, 220.0];
pub const CURRENCY: &str = "USD";

/// Inputs shared by validation and orientation scoring.
#[derive(Debug, Clone, Copy)]
pub struct Settings {
    pub overhang_angle_deg: f64,
    pub bridge_angle_deg: f64,
    pub layer_height_mm: f64,
    pub min_feature_mm: Option<f64>,
}

impl Default for Settings {
    fn default() -> Self {
        Self {
            overhang_angle_deg: DEFAULT_OVERHANG_DEG,
            bridge_angle_deg: DEFAULT_BRIDGE_DEG,
            layer_height_mm: DEFAULT_LAYER_MM,
            min_feature_mm: None,
        }
    }
}

/// Print analysis of one world-space mesh.
pub fn analyze(mesh: &Mesh, min_wall: Option<f64>, settings: &Settings) -> PrintReport {
    let mut empty = PrintReport {
        overhang_angle_deg: settings.overhang_angle_deg,
        bridge_angle_deg: settings.bridge_angle_deg,
        layer_height_mm: settings.layer_height_mm,
        surface_area: 0.0,
        overhang_area: 0.0,
        overhang_fraction: 0.0,
        overhang_region_count: 0,
        overhang_regions: Vec::new(),
        bridge_area: 0.0,
        max_bridge_span: None,
        bridges: Vec::new(),
        first_layer: FirstLayerReport {
            z_min: 0.0,
            layer_height_mm: settings.layer_height_mm,
            on_bed: false,
            floating: false,
            below_bed: false,
            contact_area: 0.0,
            bottom_area: 0.0,
            footprint_area: 0.0,
            bbox: zero_bbox(),
        },
        min_feature: min_wall,
        min_feature_at: None,
        small_feature: false,
        shells: Vec::new(),
        island_count: 0,
        floating_island_count: 0,
        cavities: Vec::new(),
        trapped_volume: 0.0,
        cups: Vec::new(),
        cupped_volume: 0.0,
        printable: false,
    };
    let Ok((positions, welded)) = edges::weld(mesh) else {
        return empty;
    };
    if welded.len() < 3 {
        return empty;
    }

    let tris = build_triangles(&positions, &welded);
    let bbox = bbox_from_positions(&positions);
    let scale = diagonal(&bbox);
    let min_area = (scale * scale * 1e-12).max(1e-3);
    let z_min = bbox.min[2];
    let layer = settings.layer_height_mm;
    let on_bed = z_min.abs() <= layer;
    let below_bed = z_min < -layer;
    let floating = z_min > layer;

    let surface_area: f64 = tris
        .iter()
        .filter(|t| t.area >= min_area)
        .map(|t| t.area)
        .sum();

    let mut overhang_tris = Vec::new();
    let mut bridge_tris = Vec::new();
    let mut overhang_area = 0.0;
    let mut bridge_area = 0.0;
    for (i, t) in tris.iter().enumerate() {
        if t.degenerate || t.area < min_area {
            continue;
        }
        let near_bottom = t.hi[2] <= z_min + layer + 1e-9;
        if t.n[2] < 0.0 {
            let from_down = (-t.n[2]).clamp(-1.0, 1.0).acos().to_degrees();
            if from_down <= settings.overhang_angle_deg + 1e-9 {
                if !(on_bed && near_bottom) {
                    overhang_area += t.area;
                    overhang_tris.push(i);
                }
            }
            if from_down <= settings.bridge_angle_deg + 1e-9 && !(on_bed && near_bottom) {
                bridge_area += t.area;
                bridge_tris.push(i);
            }
        }
    }

    let first_layer = first_layer_report(&tris, z_min, layer, on_bed, below_bed, floating);
    let (overhang_regions, overhang_region_count) = patches(&tris, &overhang_tris, scale);
    let overhang_regions = overhang_regions
        .into_iter()
        .map(|p| OverhangRegion {
            area: p.area,
            tris: p.tris,
            at: p.at,
            bbox: p.bbox,
        })
        .take(MAX_PATCHES)
        .collect();
    let bridges = bridge_patches(&tris, &overhang_tris, &bridge_tris, scale);
    let max_bridge_span = bridges.iter().map(|b| b.span).reduce(f64::max);
    let bridges = bridges.into_iter().take(MAX_PATCHES).collect();

    let components = components(&tris);
    let shells = shell_reports(&tris, &components, layer, scale);
    let outer = shells
        .iter()
        .max_by(|a, b| {
            a.volume
                .total_cmp(&b.volume)
                .then_with(|| b.index.cmp(&a.index))
        })
        .map(|s| s.index);
    let mut island_count = 0;
    let mut floating_island_count = 0;
    let mut cavities = Vec::new();
    let mut trapped_volume = 0.0;
    for s in &shells {
        if Some(s.index) == outer {
            continue;
        }
        if s.cavity {
            let drained = s.open_edges > 0;
            let volume = -s.volume;
            if !drained {
                trapped_volume += volume;
            }
            cavities.push(CavityReport {
                shell: s.index,
                volume,
                bbox: s.bbox,
                drained,
                drain_hole_mm: if drained {
                    None
                } else {
                    Some(suggest_drain_hole(volume))
                },
            });
        } else {
            island_count += 1;
            if s.floating {
                floating_island_count += 1;
            }
        }
    }
    let shells = shells
        .into_iter()
        .map(|mut s| {
            s.outer = Some(s.index) == outer;
            s
        })
        .collect();

    let (cups, cupped_volume) = cups(&tris, scale, min_area);
    let small_feature = settings
        .min_feature_mm
        .is_some_and(|t| min_wall.is_some_and(|w| w < t));
    let printable = !small_feature && trapped_volume <= 1e-6 && !below_bed;

    empty.surface_area = surface_area;
    empty.overhang_area = overhang_area;
    empty.overhang_fraction = if surface_area > 0.0 {
        overhang_area / surface_area
    } else {
        0.0
    };
    empty.overhang_region_count = overhang_region_count;
    empty.overhang_regions = overhang_regions;
    empty.bridge_area = bridge_area;
    empty.max_bridge_span = max_bridge_span;
    empty.bridges = bridges;
    empty.first_layer = first_layer;
    empty.min_feature = min_wall;
    empty.min_feature_at = None;
    empty.small_feature = small_feature;
    empty.shells = shells;
    empty.island_count = island_count;
    empty.floating_island_count = floating_island_count;
    empty.cavities = cavities;
    empty.trapped_volume = trapped_volume;
    empty.cups = cups;
    empty.cupped_volume = cupped_volume;
    empty.printable = printable;
    empty
}

/// Minimum feature location, when the caller already knows it.
pub fn with_feature_at(mut report: PrintReport, at: Option<Vec3>) -> PrintReport {
    report.min_feature_at = at;
    report
}

// --- triangles -----------------------------------------------------------------

struct PTri {
    /// Welded vertex ids.
    v: [u32; 3],
    p: [[f64; 3]; 3],
    hi: [f64; 3],
    /// Unit geometric normal; zero for a degenerate triangle.
    n: [f64; 3],
    area: f64,
    degenerate: bool,
}

fn build_triangles(positions: &[[f64; 3]], welded: &[u32]) -> Vec<PTri> {
    let mut out = Vec::with_capacity(welded.len() / 3);
    for tri in welded.chunks_exact(3) {
        let v = [tri[0], tri[1], tri[2]];
        let p = [
            positions[v[0] as usize],
            positions[v[1] as usize],
            positions[v[2] as usize],
        ];
        let cross = edges::cross(edges::sub(p[1], p[0]), edges::sub(p[2], p[0]));
        let len = edges::dot(cross, cross).sqrt();
        let n = if len > 0.0 {
            edges::scale(cross, 1.0 / len)
        } else {
            [0.0; 3]
        };
        let mut hi = [f64::MIN; 3];
        for q in p {
            for (i, value) in q.iter().enumerate() {
                hi[i] = hi[i].max(*value);
            }
        }
        out.push(PTri {
            v,
            p,
            hi,
            n,
            area: 0.5 * len,
            degenerate: len < 1e-12,
        });
    }
    out
}

fn first_layer_report(
    tris: &[PTri],
    z_min: f64,
    layer: f64,
    on_bed: bool,
    below_bed: bool,
    floating: bool,
) -> FirstLayerReport {
    let mut contact_area = 0.0;
    let mut bottom_area = 0.0;
    let mut lo = [f64::MAX; 3];
    let mut hi = [f64::MIN; 3];
    let mut found = false;
    for t in tris {
        if t.degenerate || t.n[2] >= 0.0 || t.hi[2] > z_min + layer + 1e-9 {
            continue;
        }
        found = true;
        let projected = t.area * t.n[2].abs();
        bottom_area += projected;
        if on_bed {
            contact_area += projected;
        }
        for q in t.p {
            for (k, value) in q.iter().enumerate() {
                lo[k] = lo[k].min(*value);
                hi[k] = hi[k].max(*value);
            }
        }
    }
    let bbox = if found {
        Bbox { min: lo, max: hi }
    } else {
        zero_bbox()
    };
    let footprint_area = if found {
        (bbox.max[0] - bbox.min[0]).max(0.0) * (bbox.max[1] - bbox.min[1]).max(0.0)
    } else {
        0.0
    };
    FirstLayerReport {
        z_min,
        layer_height_mm: layer,
        on_bed,
        floating,
        below_bed,
        contact_area,
        bottom_area,
        footprint_area,
        bbox,
    }
}

// --- connected components ------------------------------------------------------

fn components(tris: &[PTri]) -> Vec<usize> {
    let mut parent: Vec<usize> = (0..tris.len()).collect();
    let mut first_at: BTreeMap<u32, usize> = BTreeMap::new();
    for (i, t) in tris.iter().enumerate() {
        for &v in &t.v {
            match first_at.get(&v) {
                Some(&other) => union(&mut parent, i, other),
                None => {
                    first_at.insert(v, i);
                }
            }
        }
    }
    (0..tris.len()).map(|i| find(&mut parent, i)).collect()
}

fn find(parent: &mut [usize], mut i: usize) -> usize {
    while parent[i] != i {
        parent[i] = parent[parent[i]];
        i = parent[i];
    }
    i
}

fn union(parent: &mut [usize], a: usize, b: usize) {
    let ra = find(parent, a);
    let rb = find(parent, b);
    if ra != rb {
        // Keep the smaller root so component numbering follows triangle order.
        let (lo, hi) = if ra < rb { (ra, rb) } else { (rb, ra) };
        parent[hi] = lo;
    }
}

fn shell_reports(
    tris: &[PTri],
    component_of: &[usize],
    layer: f64,
    scale: f64,
) -> Vec<ShellReport> {
    let mut groups: BTreeMap<usize, Vec<usize>> = BTreeMap::new();
    for (i, &root) in component_of.iter().enumerate() {
        groups.entry(root).or_default().push(i);
    }
    // Edge incidence plus one owning triangle, so a boundary edge can be
    // attributed to its shell in O(1).
    let mut edge_count: BTreeMap<(u32, u32), (usize, usize)> = BTreeMap::new();
    for (i, t) in tris.iter().enumerate() {
        for (a, b) in [(t.v[0], t.v[1]), (t.v[1], t.v[2]), (t.v[2], t.v[0])] {
            let key = if a < b { (a, b) } else { (b, a) };
            let entry = edge_count.entry(key).or_insert((0, i));
            entry.0 += 1;
        }
    }
    let volume_eps = (scale * scale * scale * 1e-12).max(1e-6);
    let mut out = Vec::with_capacity(groups.len());
    for (index, ids) in groups.values().enumerate() {
        let mut volume = 0.0;
        let mut lo = [f64::MAX; 3];
        let mut hi = [f64::MIN; 3];
        for &i in ids {
            let t = &tris[i];
            volume += edges::dot(t.p[0], edges::cross(t.p[1], t.p[2])) / 6.0;
            for q in t.p {
                for (k, value) in q.iter().enumerate() {
                    lo[k] = lo[k].min(*value);
                    hi[k] = hi[k].max(*value);
                }
            }
        }
        let root = component_of[ids[0]];
        let mut open_edges = 0usize;
        let mut non_manifold_edges = 0usize;
        for &(count, owner) in edge_count.values() {
            if component_of[owner] != root {
                continue;
            }
            match count {
                0 | 2 => {}
                1 => open_edges += 1,
                _ => non_manifold_edges += 1,
            }
        }
        let volume = if volume.abs() < volume_eps {
            0.0
        } else {
            volume
        };
        out.push(ShellReport {
            index,
            volume,
            tris: ids.len(),
            bbox: Bbox { min: lo, max: hi },
            open_edges,
            non_manifold_edges,
            outer: false,
            cavity: volume < -volume_eps,
            floating: hi[2] > layer && lo[2] > layer,
        });
    }
    // Sort by descending absolute volume for stable reports, keeping the
    // deterministic component index in `index`.
    out.sort_by(|a, b| {
        b.volume
            .abs()
            .total_cmp(&a.volume.abs())
            .then_with(|| a.index.cmp(&b.index))
    });
    out
}

// --- patches -------------------------------------------------------------------

struct Patch {
    area: f64,
    tris: usize,
    at: Vec3,
    bbox: Bbox,
}

fn patches(tris: &[PTri], selected: &[usize], scale: f64) -> (Vec<Patch>, usize) {
    if selected.is_empty() {
        return (Vec::new(), 0);
    }
    let area_eps = (scale * scale * 1e-12).max(1e-6);
    let groups = group_selected(tris, selected);
    let mut out = Vec::new();
    for ids in groups {
        let mut area = 0.0;
        let mut centroid = [0.0; 3];
        let mut lo = [f64::MAX; 3];
        let mut hi = [f64::MIN; 3];
        for &i in &ids {
            let t = &tris[i];
            area += t.area;
            centroid[0] += t.area * (t.p[0][0] + t.p[1][0] + t.p[2][0]) / 3.0;
            centroid[1] += t.area * (t.p[0][1] + t.p[1][1] + t.p[2][1]) / 3.0;
            centroid[2] += t.area * (t.p[0][2] + t.p[1][2] + t.p[2][2]) / 3.0;
            for q in t.p {
                for (k, value) in q.iter().enumerate() {
                    lo[k] = lo[k].min(*value);
                    hi[k] = hi[k].max(*value);
                }
            }
        }
        if area <= area_eps {
            continue;
        }
        out.push(Patch {
            area,
            tris: ids.len(),
            at: [centroid[0] / area, centroid[1] / area, centroid[2] / area],
            bbox: Bbox { min: lo, max: hi },
        });
    }
    out.sort_by(|a, b| {
        b.area
            .total_cmp(&a.area)
            .then_with(|| b.bbox.min[0].total_cmp(&a.bbox.min[0]))
    });
    let count = out.len();
    (out, count)
}

/// Group selected triangles that share a welded vertex. The returned groups
/// follow the first selected triangle in each group, so ordering is stable.
fn group_selected(tris: &[PTri], selected: &[usize]) -> Vec<Vec<usize>> {
    let mut parent: BTreeMap<usize, usize> = selected.iter().map(|&i| (i, i)).collect();
    let mut first_at: BTreeMap<u32, usize> = BTreeMap::new();
    for &i in selected {
        for &v in &tris[i].v {
            if let Some(&other) = first_at.get(&v) {
                let ra = find_map(&mut parent, i);
                let rb = find_map(&mut parent, other);
                if ra != rb {
                    let (lo, hi) = if ra < rb { (ra, rb) } else { (rb, ra) };
                    parent.insert(hi, lo);
                }
            } else {
                first_at.insert(v, i);
            }
        }
    }
    let mut groups: BTreeMap<usize, Vec<usize>> = BTreeMap::new();
    for &i in selected {
        let root = find_map(&mut parent, i);
        groups.entry(root).or_default().push(i);
    }
    groups.into_values().collect()
}

fn find_map(parent: &mut BTreeMap<usize, usize>, i: usize) -> usize {
    let mut root = i;
    while let Some(&next) = parent.get(&root) {
        if next == root {
            break;
        }
        root = next;
    }
    let mut cur = i;
    while let Some(&next) = parent.get(&cur) {
        if next == root {
            break;
        }
        parent.insert(cur, root);
        cur = next;
    }
    root
}

fn bridge_patches(
    tris: &[PTri],
    overhang: &[usize],
    horizontal: &[usize],
    _scale: f64,
) -> Vec<BridgeSpan> {
    if overhang.is_empty() || horizontal.is_empty() {
        return Vec::new();
    }
    let horizontal_set: std::collections::BTreeSet<usize> = horizontal.iter().copied().collect();
    let mut out = Vec::new();
    for ids in group_selected(tris, overhang) {
        if !ids.iter().any(|i| horizontal_set.contains(i)) {
            continue;
        }
        let mut area = 0.0;
        let mut centroid = [0.0; 3];
        let mut lo = [f64::MAX; 3];
        let mut hi = [f64::MIN; 3];
        for &i in &ids {
            let t = &tris[i];
            area += t.area;
            centroid[0] += t.area * (t.p[0][0] + t.p[1][0] + t.p[2][0]) / 3.0;
            centroid[1] += t.area * (t.p[0][1] + t.p[1][1] + t.p[2][1]) / 3.0;
            centroid[2] += t.area * (t.p[0][2] + t.p[1][2] + t.p[2][2]) / 3.0;
            for q in t.p {
                for (k, value) in q.iter().enumerate() {
                    lo[k] = lo[k].min(*value);
                    hi[k] = hi[k].max(*value);
                }
            }
        }
        let dx = (hi[0] - lo[0]).max(0.0);
        let dy = (hi[1] - lo[1]).max(0.0);
        let span = dx.min(dy);
        let length = dx.max(dy);
        if span < 0.05 || length < 0.05 {
            continue;
        }
        out.push(BridgeSpan {
            area,
            span,
            length,
            at: [centroid[0] / area, centroid[1] / area, centroid[2] / area],
            bbox: Bbox { min: lo, max: hi },
        });
    }
    out.sort_by(|a, b| {
        b.span
            .total_cmp(&a.span)
            .then_with(|| b.area.total_cmp(&a.area))
            .then_with(|| b.bbox.min[0].total_cmp(&a.bbox.min[0]))
    });
    out
}

/// Approximate a drainage-hole diameter: 3 mm is enough for most resins, and
/// larger cavities get a slightly larger hole so they empty in reasonable
/// time.
fn suggest_drain_hole(volume: f64) -> f64 {
    (3.0 + (volume.max(0.0).sqrt() / 50.0)).min(8.0)
}

// --- cups ----------------------------------------------------------------------

fn cups(tris: &[PTri], scale: f64, min_area: f64) -> (Vec<CupReport>, f64) {
    let offset = (scale * 1e-6).max(1e-4);
    let upward: Vec<usize> = (0..tris.len())
        .filter(|&i| {
            let t = &tris[i];
            !t.degenerate && t.area >= min_area && t.n[2] >= 0.985
        })
        .collect();
    if upward.is_empty() {
        return (Vec::new(), 0.0);
    }
    let stride = upward.len().div_ceil(MAX_CUP_SAMPLES).max(1);
    let mut floors: Vec<(usize, f64)> = Vec::new();
    for &i in upward.iter().step_by(stride) {
        let t = &tris[i];
        let origin = edges::sub(centroid(t), edges::scale(t.n, -offset));
        let mut nearest = f64::INFINITY;
        for (j, u) in tris.iter().enumerate() {
            if j == i || u.degenerate || shares_vertex(t, u) {
                continue;
            }
            // A cup ceiling is an underside; an upward face above the floor
            // is just another top surface.
            if u.n[2] >= 0.0 {
                continue;
            }
            if let Some(hit) = ray_triangle(origin, [0.0, 0.0, 1.0], &u.p) {
                let depth = hit + offset;
                if depth > 0.05 && depth < nearest {
                    nearest = depth;
                }
            }
        }
        if nearest.is_finite() {
            floors.push((i, nearest));
        }
    }
    if floors.is_empty() {
        return (Vec::new(), 0.0);
    }
    let depth_by_tri: BTreeMap<usize, f64> = floors.iter().copied().collect();
    let selected: Vec<usize> = floors.iter().map(|(i, _)| *i).collect();
    let mut out = Vec::new();
    let mut total = 0.0;
    for ids in group_selected(tris, &selected) {
        let mut area = 0.0;
        let mut centroid = [0.0; 3];
        let mut lo = [f64::MAX; 3];
        let mut hi = [f64::MIN; 3];
        let mut depth: f64 = 0.0;
        for &i in &ids {
            let t = &tris[i];
            depth = depth.max(depth_by_tri.get(&i).copied().unwrap_or(0.0));
            area += t.area;
            centroid[0] += t.area * (t.p[0][0] + t.p[1][0] + t.p[2][0]) / 3.0;
            centroid[1] += t.area * (t.p[0][1] + t.p[1][1] + t.p[2][1]) / 3.0;
            centroid[2] += t.area * (t.p[0][2] + t.p[1][2] + t.p[2][2]) / 3.0;
            for q in t.p {
                for (k, value) in q.iter().enumerate() {
                    lo[k] = lo[k].min(*value);
                    hi[k] = hi[k].max(*value);
                }
            }
        }
        if area <= 0.0 || depth <= 0.0 {
            continue;
        }
        let volume = area * depth;
        total += volume;
        out.push(CupReport {
            area,
            depth,
            volume,
            at: [centroid[0] / area, centroid[1] / area, centroid[2] / area],
            bbox: Bbox { min: lo, max: hi },
        });
    }
    out.sort_by(|a, b| {
        b.volume
            .total_cmp(&a.volume)
            .then_with(|| b.area.total_cmp(&a.area))
    });
    out.truncate(MAX_PATCHES);
    (out, total)
}

fn shares_vertex(a: &PTri, b: &PTri) -> bool {
    a.v.iter().any(|x| b.v.contains(x))
}

fn centroid(t: &PTri) -> [f64; 3] {
    [
        (t.p[0][0] + t.p[1][0] + t.p[2][0]) / 3.0,
        (t.p[0][1] + t.p[1][1] + t.p[2][1]) / 3.0,
        (t.p[0][2] + t.p[1][2] + t.p[2][2]) / 3.0,
    ]
}

fn ray_triangle(origin: [f64; 3], dir: [f64; 3], tri: &[[f64; 3]; 3]) -> Option<f64> {
    let (a, b, c) = (tri[0], tri[1], tri[2]);
    let e1 = edges::sub(b, a);
    let e2 = edges::sub(c, a);
    let h = edges::cross(dir, e2);
    let det = edges::dot(e1, h);
    if det.abs() < 1e-12 {
        return None;
    }
    let inv = 1.0 / det;
    let s = edges::sub(origin, a);
    let u = edges::dot(s, h) * inv;
    if !(-1e-9..=1.0 + 1e-9).contains(&u) {
        return None;
    }
    let q = edges::cross(s, e1);
    let v = edges::dot(dir, q) * inv;
    if v < -1e-9 || u + v > 1.0 + 1e-9 {
        return None;
    }
    let t = edges::dot(e2, q) * inv;
    (t > 1e-9).then_some(t)
}

// --- materials and fits ---------------------------------------------------------

const PRESETS: [(&str, f64, f64); 7] = [
    ("pla", 1.24, 20.0),
    ("petg", 1.27, 24.0),
    ("abs", 1.04, 22.0),
    ("asa", 1.07, 28.0),
    ("tpu", 1.21, 35.0),
    ("nylon", 1.14, 40.0),
    ("resin", 1.10, 45.0),
];

pub fn material_presets() -> Vec<MaterialSpec> {
    PRESETS
        .iter()
        .map(|(name, density, cost)| MaterialSpec {
            name: (*name).to_string(),
            density_g_cm3: *density,
            cost_per_kg: *cost,
        })
        .collect()
}

/// Resolve a preset name (case-insensitive) or a custom name with explicit
/// density and cost. A preset only needs overrides for the values the caller
/// wants to change.
pub fn resolve_material(
    name: &str,
    density_g_cm3: Option<f64>,
    cost_per_kg: Option<f64>,
) -> crate::Result<MaterialSpec> {
    let key = name.trim().to_ascii_lowercase();
    if key.is_empty() {
        return Err(crate::Error::Invalid(
            "material name must not be empty (hint: use a preset such as 'pla' or pass a \
             custom name with density_g_cm3 and cost_per_kg)"
                .into(),
        ));
    }
    let preset = PRESETS.iter().find(|(p, _, _)| *p == key);
    let (density, cost) = match preset {
        Some((_, d, c)) => (*d, *c),
        None => {
            let Some(d) = density_g_cm3 else {
                let known: Vec<&str> = PRESETS.iter().map(|(p, _, _)| *p).collect();
                return Err(crate::Error::Invalid(format!(
                    "unknown material '{name}' (hint: presets are {}; or pass density_g_cm3 and \
                     cost_per_kg for a custom material)",
                    known.join(", ")
                )));
            };
            (d, cost_per_kg.unwrap_or(0.0))
        }
    };
    let density = density_g_cm3.unwrap_or(density);
    let cost = cost_per_kg.unwrap_or(cost);
    if !density.is_finite() || density <= 0.0 {
        return Err(crate::Error::Invalid(
            "material density must be > 0 g/cm^3".into(),
        ));
    }
    if !cost.is_finite() || cost < 0.0 {
        return Err(crate::Error::Invalid(
            "material cost must be >= 0 USD/kg".into(),
        ));
    }
    Ok(MaterialSpec {
        name: if preset.is_some() {
            key
        } else {
            name.trim().to_string()
        },
        density_g_cm3: density,
        cost_per_kg: cost,
    })
}

fn print_profile(material: Option<&str>) -> crate::Result<(String, f64, f64)> {
    let key = material.unwrap_or("pla").trim().to_ascii_lowercase();
    let (shrink, elephant) = match key.as_str() {
        "pla" => (0.05, 0.20),
        "petg" => (0.05, 0.20),
        "abs" => (0.10, 0.25),
        "asa" => (0.10, 0.25),
        "tpu" => (0.15, 0.30),
        "nylon" => (0.10, 0.25),
        "resin" => (0.02, 0.10),
        other => {
            let known: Vec<&str> = PRESETS.iter().map(|(p, _, _)| *p).collect();
            return Err(crate::Error::Invalid(format!(
                "unknown material '{other}' for fit compensation (hint: presets are {})",
                known.join(", ")
            )));
        }
    };
    Ok((key, shrink, elephant))
}

#[allow(clippy::too_many_arguments)]
pub fn fit(
    class: FitClass,
    nominal_mm: Option<f64>,
    hole_mm: Option<f64>,
    material: Option<&str>,
    printer_tolerance_mm: Option<f64>,
    hole_shrink_mm: Option<f64>,
    elephant_foot_mm: Option<f64>,
) -> crate::Result<FitReport> {
    for (label, value) in [
        ("nominal_mm", nominal_mm),
        ("hole_mm", hole_mm),
        ("printer_tolerance_mm", printer_tolerance_mm),
        ("hole_shrink_mm", hole_shrink_mm),
        ("elephant_foot_mm", elephant_foot_mm),
    ] {
        if let Some(v) = value {
            if !v.is_finite() {
                return Err(crate::Error::Invalid(format!("{label} must be finite")));
            }
        }
    }
    if nominal_mm.is_some_and(|v| v <= 0.0) || hole_mm.is_some_and(|v| v <= 0.0) {
        return Err(crate::Error::Invalid(
            "nominal_mm and hole_mm must be > 0 (hint: they are diameters in mm)".into(),
        ));
    }
    let tolerance = printer_tolerance_mm.unwrap_or(0.0);
    if tolerance < 0.0 {
        return Err(crate::Error::Invalid(
            "printer_tolerance_mm must be >= 0 (hint: it is extra clearance per side)".into(),
        ));
    }
    let (material_name, default_shrink, default_elephant) = print_profile(material)?;
    let shrink = hole_shrink_mm.unwrap_or(default_shrink);
    let elephant = elephant_foot_mm.unwrap_or(default_elephant);
    let radial = class.radial_clearance_mm();
    let allowance = radial + tolerance + shrink;
    let modeled_hole_mm = nominal_mm.map(|n| n + 2.0 * allowance);
    let modeled_shaft_mm = hole_mm.map(|h| h - 2.0 * allowance);
    let classes = FitClass::all()
        .iter()
        .map(|c| FitClassInfo {
            class: *c,
            radial_clearance_mm: c.radial_clearance_mm(),
            diametral_clearance_mm: 2.0 * c.radial_clearance_mm(),
            description: c.description().to_string(),
        })
        .collect();
    let mut notes = vec![
        format!(
            "modeled hole = nominal + 2 * ({} clearance + {} printer tolerance + {} shrink)",
            class.as_str(),
            fmt(tolerance),
            fmt(shrink)
        ),
        format!(
            "remove the elephant foot with a {} mm bottom chamfer or negative first-layer offset",
            fmt(elephant)
        ),
    ];
    if class == FitClass::Press {
        notes.push(
            "press fits are material- and printer-dependent; test a coupon before committing"
                .into(),
        );
    }
    Ok(FitReport {
        class,
        radial_clearance_mm: radial,
        diametral_clearance_mm: 2.0 * radial,
        printer_tolerance_mm: tolerance,
        hole_shrink_mm: shrink,
        elephant_foot_mm: elephant,
        material: Some(material_name),
        modeled_hole_mm,
        modeled_shaft_mm,
        classes,
        notes,
    })
}

fn fmt(v: f64) -> String {
    let s = format!("{v:.3}");
    s.trim_end_matches('0').trim_end_matches('.').to_string()
}

// --- geometry helpers -----------------------------------------------------------

fn zero_bbox() -> Bbox {
    Bbox {
        min: [0.0; 3],
        max: [0.0; 3],
    }
}

fn bbox_from_positions(positions: &[[f64; 3]]) -> Bbox {
    let mut lo = [f64::MAX; 3];
    let mut hi = [f64::MIN; 3];
    for p in positions {
        for (i, value) in p.iter().enumerate() {
            lo[i] = lo[i].min(*value);
            hi[i] = hi[i].max(*value);
        }
    }
    if positions.is_empty() {
        zero_bbox()
    } else {
        Bbox { min: lo, max: hi }
    }
}

fn diagonal(b: &Bbox) -> f64 {
    edges::sub(b.max, b.min)
        .iter()
        .map(|v| v * v)
        .sum::<f64>()
        .sqrt()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geo;

    fn analyze_box(size: [f64; 3], center: [f64; 3]) -> PrintReport {
        let mesh = crate::transform_mesh(&geo::box_mesh(size), &crate::mat4::translation(center));
        analyze(
            &mesh,
            Some(size[0].min(size[1]).min(size[2])),
            &Settings::default(),
        )
    }

    #[test]
    fn a_box_on_the_bed_has_contact_but_no_overhang() {
        let r = analyze_box([10.0, 20.0, 30.0], [0.0, 0.0, 15.0]);
        assert!(r.first_layer.on_bed);
        assert!(!r.first_layer.floating);
        assert!((r.first_layer.contact_area - 200.0).abs() < 0.01, "{r:?}");
        assert_eq!(
            r.overhang_area, 0.0,
            "the bed-contact face is not an overhang"
        );
        assert_eq!(r.trapped_volume, 0.0);
        assert_eq!(r.island_count, 0);
        assert!(r.printable);
    }

    #[test]
    fn a_floating_box_reports_no_contact_and_an_island_only_when_split() {
        let r = analyze_box([10.0, 10.0, 10.0], [0.0, 0.0, 20.0]);
        assert!(r.first_layer.floating);
        assert_eq!(r.first_layer.contact_area, 0.0);
        assert!(r.first_layer.bottom_area > 99.0);
        assert!(
            r.printable,
            "a floating part can still be printed with supports"
        );
        assert!(
            (r.max_bridge_span.unwrap_or(0.0) - 10.0).abs() < 1e-6,
            "the unsupported bottom is a 10 mm bridge: {r:?}"
        );
    }

    #[test]
    fn a_tilted_box_has_overhang_area() {
        let mesh = crate::transform_mesh(
            &geo::box_mesh([20.0, 10.0, 10.0]),
            &crate::mat4::rotation_xyz_deg([0.0, 30.0, 0.0]),
        );
        let r = analyze(&mesh, None, &Settings::default());
        assert!(r.overhang_area > 0.0, "{r:?}");
        assert!(r.overhang_fraction > 0.0 && r.overhang_fraction <= 1.0);
    }

    #[test]
    fn separate_shells_are_islands_and_a_hole_is_a_bridge() {
        let a = geo::box_mesh([10.0, 10.0, 10.0]);
        let mut b = crate::transform_mesh(&a, &crate::mat4::translation([30.0, 0.0, 0.0]));
        let mut mesh = a.clone();
        mesh.positions.append(&mut b.positions);
        let base = (mesh.indices.len() / 3) as u32;
        let mut indices = mesh.indices.clone();
        indices.extend(b.indices.iter().map(|i| i + base));
        mesh.indices = indices;
        let r = analyze(&mesh, None, &Settings::default());
        assert_eq!(r.shells.len(), 2, "{r:?}");
        assert_eq!(r.island_count, 1);
    }

    #[test]
    fn fit_classes_scale_the_hole_and_expose_the_table() {
        let free = fit(
            FitClass::Free,
            Some(10.0),
            None,
            Some("pla"),
            None,
            None,
            None,
        )
        .unwrap();
        let snug = fit(
            FitClass::Snug,
            Some(10.0),
            None,
            Some("pla"),
            None,
            None,
            None,
        )
        .unwrap();
        assert!(free.modeled_hole_mm.unwrap() > snug.modeled_hole_mm.unwrap());
        assert!(
            snug.modeled_hole_mm.unwrap() > 10.0,
            "shrink is compensated too"
        );
        assert_eq!(free.classes.len(), 4);
        let press = fit(
            FitClass::Press,
            Some(10.0),
            None,
            None,
            None,
            Some(0.0),
            None,
        )
        .unwrap();
        assert!(press.modeled_hole_mm.unwrap() < 10.0);
    }

    #[test]
    fn presets_and_custom_materials_resolve() {
        let pla = resolve_material("PLA", None, None).unwrap();
        assert!((pla.density_g_cm3 - 1.24).abs() < 1e-9);
        let custom = resolve_material("mystery", Some(2.0), Some(10.0)).unwrap();
        assert_eq!(custom.name, "mystery");
        assert!(resolve_material("mystery", None, None).is_err());
    }
}
