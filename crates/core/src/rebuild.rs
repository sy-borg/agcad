//! Incremental rebuild support: content keys and a per-feature geometry cache.
//!
//! A knob change still replays the program — that is the correctness oracle —
//! but the expensive part is regenerating tessellated geometry. Every
//! geometry-producing program step gets a [`GeomKey`] folded from its resolved
//! arguments *and from the keys of the geometry it consumes* (boolean
//! operands, the solid a fillet reshapes, the sketch profile an extrude
//! sweeps). The cache stores each step's result under that key, so:
//!
//! * an unchanged feature produces the same key and reuses its mesh,
//! * a feature downstream of a changed one sees a new upstream key and
//!   rebuilds,
//! * an independent feature keeps its key and stays cached.
//!
//! Keys are deterministic hashes of exact bits (f32/f64 bit patterns), never
//! of iteration order or pointers, and the cached bytes are exactly what a
//! cold replay would have computed. The cache is derived state: it is not
//! saved, not part of undo snapshots, and clearing it only costs speed.

use wright_protocol::{BooleanOp, EdgeFilter, Mesh, RevolveAxis, SketchElement, Vec3};

use crate::fillet;
use crate::pattern::CopyKind;

/// Content key of one geometry result: two independent 64-bit hashes.
/// Collisions would mean stale geometry, so this is deliberately 128 bits
/// wide even though an entity signature only needs 64.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct GeomKey {
    a: u64,
    b: u64,
}

/// A geometry result worth memoizing. `volume`, `edges` and `center` carry the
/// values a live command reports in its result so a cache hit can answer the
/// same way without touching the kernel.
#[derive(Debug, Clone, Default)]
pub struct CachedGeometry {
    pub mesh: Mesh,
    pub volume: Option<f64>,
    pub edges: Option<u32>,
    /// Bounding-box center for boolean results (the entity translation).
    pub center: Option<Vec3>,
}

impl CachedGeometry {
    pub fn mesh(mesh: Mesh) -> Self {
        Self { mesh, ..Self::default() }
    }
}

/// Geometry results indexed by program step. One entry per feature: a knob
/// change replaces only the entries whose key moved, so memory is bounded by
/// the program size, not by the number of knob turns.
#[derive(Debug, Default)]
pub struct GeometryCache {
    entries: Vec<Option<(GeomKey, CachedGeometry)>>,
    /// Features whose geometry was regenerated during the current replay.
    pub rebuilt: usize,
    /// Features served from the cache during the current replay.
    pub reused: usize,
}

impl GeometryCache {
    /// Start a replay: counters reset, meshes stay.
    pub fn begin(&mut self) {
        self.rebuilt = 0;
        self.reused = 0;
    }

    /// Look up a feature's previous result. Only a hit inside a replay counts
    /// as reused; a live command that happens to match an old entry is just a
    /// free answer.
    pub fn get(&mut self, step: Option<usize>, key: GeomKey) -> Option<CachedGeometry> {
        let step = step?;
        match self.entries.get(step) {
            Some(Some((cached, geometry))) if *cached == key => {
                self.reused += 1;
                Some(geometry.clone())
            },
            _ => None,
        }
    }

    /// Store a freshly built result for this program step.
    pub fn put(
        &mut self,
        step: Option<usize>,
        key: GeomKey,
        geometry: CachedGeometry,
    ) -> CachedGeometry {
        if let Some(step) = step {
            if self.entries.len() <= step {
                self.entries.resize_with(step + 1, || None);
            }
            self.entries[step] = Some((key, geometry.clone()));
            self.rebuilt += 1;
        }
        geometry
    }

    /// Drop every memoized mesh. The next replay is the cold oracle.
    #[cfg(test)]
    pub fn clear(&mut self) {
        self.entries.clear();
        self.rebuilt = 0;
        self.reused = 0;
    }
}

/// Little-endian FNV-1a in two lanes with different primes; feeding the same
/// bytes into both gives a 128-bit content key.
#[derive(Default)]
pub struct Hasher {
    a: u64,
    b: u64,
    /// Set on first use so an empty key still differs from [`GeomKey::NONE`].
    started: bool,
}

const FNV_A_OFFSET: u64 = 0xcbf2_9ce4_8422_2325;
const FNV_A_PRIME: u64 = 0x0000_0100_0000_01b3;
const FNV_B_OFFSET: u64 = 0x8422_2325_cbf2_9ce4;
const FNV_B_PRIME: u64 = 0x9e37_79b1_85eb_ca87;

impl Hasher {
    pub fn new() -> Self {
        Self { a: FNV_A_OFFSET, b: FNV_B_OFFSET, started: false }
    }

    fn mix(&mut self, bytes: &[u8]) {
        self.started = true;
        for &byte in bytes {
            self.a ^= u64::from(byte);
            self.a = self.a.wrapping_mul(FNV_A_PRIME);
            self.b = self.b.rotate_left(5) ^ u64::from(byte);
            self.b = self.b.wrapping_mul(FNV_B_PRIME);
        }
    }

    pub fn str(&mut self, s: &str) {
        self.mix(s.as_bytes());
        self.mix(&[0]);
    }

    pub fn u64(&mut self, v: u64) {
        self.mix(&v.to_le_bytes());
    }

    pub fn u32(&mut self, v: u32) {
        self.mix(&v.to_le_bytes());
    }

    pub fn f64(&mut self, v: f64) {
        self.u64(v.to_bits());
    }

    pub fn vec3(&mut self, v: Vec3) {
        for x in v {
            self.f64(x);
        }
    }

    pub fn xy(&mut self, v: [f64; 2]) {
        self.f64(v[0]);
        self.f64(v[1]);
    }

    pub fn placement(&mut self, translation: Vec3, rotation_deg: Vec3, scale: Vec3) {
        self.vec3(translation);
        self.vec3(rotation_deg);
        self.vec3(scale);
    }

    pub fn key(&mut self, key: GeomKey) {
        self.u64(key.a);
        self.u64(key.b);
    }

    pub fn finish(self) -> GeomKey {
        // A hash of anything is never the empty placeholder. `str` already
        // mixes a tag, and `started` guards an accidental empty hasher.
        if self.started {
            GeomKey { a: self.a, b: self.b }
        } else {
            GeomKey { a: FNV_A_OFFSET, b: FNV_B_OFFSET }
        }
    }
}

fn start(tag: &str) -> Hasher {
    let mut h = Hasher::new();
    h.str("wright/geom/1");
    h.str(tag);
    h
}

pub fn box_key(size: Vec3) -> GeomKey {
    let mut h = start("box");
    h.vec3(size);
    h.finish()
}

pub fn sphere_key(radius: f64, segments: u32) -> GeomKey {
    let mut h = start("sphere");
    h.f64(radius);
    h.u32(segments);
    h.finish()
}

pub fn cylinder_key(radius: f64, height: f64, segments: u32) -> GeomKey {
    let mut h = start("cylinder");
    h.f64(radius);
    h.f64(height);
    h.u32(segments);
    h.finish()
}

/// An imported or client-supplied mesh has no parameters: its raw triangles
/// and crease angle are its key. Computed before tessellation, so a replay can
/// reuse the welded mesh and its derived normals.
pub fn add_mesh_key(
    positions: &[[f32; 3]],
    indices: &[u32],
    smooth_angle_deg: f64,
) -> GeomKey {
    let mut h = start("mesh");
    h.f64(smooth_angle_deg);
    h.u64(positions.len() as u64);
    for p in positions {
        for v in p {
            h.u32(v.to_bits());
        }
    }
    h.u64(indices.len() as u64);
    for &i in indices {
        h.u32(i);
    }
    h.finish()
}

/// Sketch display mesh depends on its solved elements, but solids key off the
/// profile loops they are built from. This key makes a boolean that consumes a
/// sketch follow edits too.
pub fn sketch_key(elements: &[SketchElement]) -> GeomKey {
    let mut h = start("sketch");
    h.u64(elements.len() as u64);
    for element in elements {
        match element {
            SketchElement::Line { id, p1, p2, construction } => {
                h.str("line");
                h.u64(*id);
                h.u64(u64::from(*construction));
                h.xy(*p1);
                h.xy(*p2);
            },
            SketchElement::Circle { id, center, radius, construction } => {
                h.str("circle");
                h.u64(*id);
                h.u64(u64::from(*construction));
                h.xy(*center);
                h.f64(*radius);
            },
            SketchElement::Arc { id, center, radius, start_deg, end_deg, construction } => {
                h.str("arc");
                h.u64(*id);
                h.u64(u64::from(*construction));
                h.xy(*center);
                h.f64(*radius);
                h.f64(*start_deg);
                h.f64(*end_deg);
            },
            SketchElement::Spline { id, points, closed, construction } => {
                h.str("spline");
                h.u64(*id);
                h.u64(u64::from(*construction));
                h.u64(u64::from(*closed));
                h.u64(points.len() as u64);
                for p in points {
                    h.xy(*p);
                }
            },
        }
    }
    h.finish()
}

pub fn boolean_key(
    op: BooleanOp,
    operands: &[(GeomKey, Vec3, Vec3, Vec3)],
) -> GeomKey {
    let mut h = start("boolean");
    h.str(op.as_str());
    h.u64(operands.len() as u64);
    for (key, translation, rotation_deg, scale) in operands {
        h.key(*key);
        // Boolean operands enter the kernel in world space, so placement is
        // part of this feature's input even though the key only describes the
        // local mesh.
        h.placement(*translation, *rotation_deg, *scale);
    }
    h.finish()
}

fn loops(h: &mut Hasher, loops: &[Vec<[f64; 2]>]) {
    h.u64(loops.len() as u64);
    for lp in loops {
        h.u64(lp.len() as u64);
        for p in lp {
            h.xy(*p);
        }
    }
}

pub fn extrude_key(profile: &[Vec<[f64; 2]>], distance: f64) -> GeomKey {
    let mut h = start("extrude");
    loops(&mut h, profile);
    h.f64(distance);
    h.finish()
}

pub fn revolve_key(
    profile: &[Vec<[f64; 2]>],
    angle_deg: f64,
    axis: RevolveAxis,
    segments: Option<u32>,
) -> GeomKey {
    let mut h = start("revolve");
    loops(&mut h, profile);
    h.f64(angle_deg);
    h.str(axis.as_str());
    match segments {
        Some(n) => {
            h.str("fixed");
            h.u32(n);
        },
        None => h.str("auto"),
    }
    h.finish()
}

pub fn reshape_key(
    input: GeomKey,
    placement: (Vec3, Vec3, Vec3),
    kind: fillet::Kind,
    filter: &EdgeFilter,
    min_angle_deg: f64,
) -> GeomKey {    let mut h = start("reshape");
    h.key(input);
    h.placement(placement.0, placement.1, placement.2);
    let (name, size) = match kind {
        fillet::Kind::Fillet(v) => ("fillet", v),
        fillet::Kind::Chamfer(v) => ("chamfer", v),
    };
    h.str(name);
    h.f64(size);
    h.f64(min_angle_deg);

    // EdgeFilter is a geometric selection; every field changes which edges
    // are cut, so every field belongs in the key.
    match filter.direction {
        Some(v) => {
            h.str("dir");
            h.vec3(v);
        },
        None => h.str("dir:none"),
    }
    match filter.angle_tol_deg {
        Some(v) => h.f64(v),
        None => h.f64(-1.0),
    }
    match filter.near {
        Some(v) => {
            h.str("near");
            h.vec3(v);
        },
        None => h.str("near:none"),
    }
    match filter.near_radius {
        Some(v) => h.f64(v),
        None => h.f64(-1.0),
    }
    match &filter.normals {
        Some(normals) => {
            h.str("normals");
            h.u64(normals.len() as u64);
            for n in normals {
                h.vec3(*n);
            }
        },
        None => h.str("normals:none"),
    }
    match filter.convex {
        Some(v) => {
            h.str(if v { "convex" } else { "concave" });
        },
        None => h.str("convex:any"),
    }
    h.finish()
}

/// The placement and mesh of a pattern copy, or the reflected copy.
pub fn copy_key(input: GeomKey, placement: (Vec3, Vec3, Vec3), kind: CopyKind) -> GeomKey {
    let mut h = start("copy");
    h.key(input);
    h.placement(placement.0, placement.1, placement.2);
    match kind {
        CopyKind::Linear { direction, spacing, index } => {
            h.str("linear");
            h.vec3(direction);
            h.f64(spacing);
            h.u32(index);
        },
        CopyKind::Circular { center, axis, step_deg, index } => {
            h.str("circular");
            h.vec3(center);
            h.vec3(axis);
            h.f64(step_deg);
            h.u32(index);
        },
        CopyKind::Mirror { origin, normal } => {
            h.str("mirror");
            h.vec3(origin);
            h.vec3(normal);
        },
    }
    h.finish()
}

pub fn shell_key(
    input: GeomKey,
    placement: (Vec3, Vec3, Vec3),
    thickness: f64,
    open: &[Vec3],
    angle_tol_deg: f64,
) -> GeomKey {
    let mut h = start("shell");
    h.key(input);
    h.placement(placement.0, placement.1, placement.2);
    h.f64(thickness);
    h.f64(angle_tol_deg);
    h.u64(open.len() as u64);
    for n in open {
        h.vec3(*n);
    }
    h.finish()
}

/// Hole inputs: the solid plus every resolved cutter dimension. The center and
/// normal are world-space, so they belong in the key even though the mesh is
/// local.
pub fn hole_key(input: GeomKey, placement: (Vec3, Vec3, Vec3), spec: &crate::hole::HoleSpec) -> GeomKey {
    let mut h = start("hole");
    h.key(input);
    h.placement(placement.0, placement.1, placement.2);
    h.vec3(spec.center);
    h.vec3(spec.normal);
    h.f64(spec.diameter);
    hole_opt(&mut h, spec.depth);
    hole_opt(&mut h, spec.counterbore_diameter);
    hole_opt(&mut h, spec.counterbore_depth);
    hole_opt(&mut h, spec.countersink_angle_deg);
    match spec.segments {
        Some(n) => {
            h.str("seg");
            h.u32(n);
        },
        None => h.str("seg:auto"),
    }
    h.finish()
}

fn hole_opt(h: &mut Hasher, v: Option<f64>) {
    match v {
        Some(v) => {
            h.str("some");
            h.f64(v);
        },
        None => h.str("none"),
    }
}

pub fn sweep_key(profile: &[Vec<[f64; 2]>], x_ref: Vec3, path: &[Vec3]) -> GeomKey {
    let mut h = start("sweep");
    loops(&mut h, profile);
    h.vec3(x_ref);
    h.u64(path.len() as u64);
    for p in path {
        h.vec3(*p);
    }
    h.finish()
}

pub fn loft_key(rings: &[Vec<Vec3>], segments: u32) -> GeomKey {
    let mut h = start("loft");
    h.u64(rings.len() as u64);
    for ring in rings {
        h.u64(ring.len() as u64);
        for p in ring {
            h.vec3(*p);
        }
    }
    h.u32(segments);
    h.finish()
}

pub fn thread_key(spec: &crate::thread::ThreadSpec) -> GeomKey {
    let mut h = start("thread");
    h.f64(spec.major_diameter);
    h.f64(spec.minor_diameter);
    h.f64(spec.pitch);
    h.f64(spec.length);
    h.str(spec.hand.as_str());
    h.f64(spec.flank_angle_deg);
    h.u32(spec.segments_per_turn);
    h.vec3(spec.origin);
    h.finish()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn keys_are_content_addressed() {
        assert_eq!(box_key([1.0, 2.0, 3.0]), box_key([1.0, 2.0, 3.0]));
        assert_ne!(box_key([1.0, 2.0, 3.0]), box_key([1.0, 2.0, 4.0]));
        assert_ne!(box_key([1.0, 2.0, 3.0]), sphere_key(1.0, 3));
    }

    #[test]
    fn cache_only_reuses_an_exact_key() {
        let mut cache = GeometryCache::default();
        let key = box_key([1.0; 3]);
        let other = box_key([2.0; 3]);
        assert!(cache.get(Some(0), key).is_none());
        cache.put(Some(0), key, CachedGeometry::mesh(Mesh::default()));
        cache.begin();
        assert!(cache.get(Some(0), key).is_some());
        assert_eq!(cache.reused, 1);
        assert!(cache.get(Some(0), other).is_none());
        assert!(cache.get(Some(1), key).is_none());
        assert_eq!(cache.reused, 1);
    }

    #[test]
    fn live_commands_do_not_pollute_replay_counters() {
        let mut cache = GeometryCache::default();
        let key = sphere_key(4.0, 32);
        cache.put(None, key, CachedGeometry::mesh(Mesh::default()));
        assert_eq!(cache.rebuilt, 0);
        assert!(cache.get(None, key).is_none());
        assert_eq!(cache.reused, 0);

        cache.begin();
        cache.put(Some(2), key, CachedGeometry::mesh(Mesh::default()));
        assert_eq!(cache.rebuilt, 1);
        assert!(cache.get(Some(2), key).is_some());
        assert_eq!(cache.reused, 1);
    }
}
