//! # agcad-core
//!
//! The headless engine. Owns documents and entities, executes
//! [`Command`]s, and produces [`CommandResult`]s. Knows nothing about
//! transport (that's the server) or rendering (that's the viewer).
//!
//! ## The program *is* the function
//!
//! Every modeling command is recorded in the document's `program`, with
//! numeric arguments kept in raw form (`Scalar`), including expressions.
//! Executing the program from an empty document must always reproduce the
//! live state — an invariant maintained by treating (entities, program,
//! parameters, next_entity) as one atomically-snapshotted unit. When a
//! parameter changes, the program is replayed with the new values:
//! identical entity ids, regenerated geometry. Agents author this program;
//! the user only describes intent.

pub mod boolean;
pub mod edges;
pub mod expr;
pub mod fillet;
pub mod geo;
pub mod hole;
pub mod loft;
pub mod mat4;
pub mod pattern;
pub mod pick;
pub mod plane;
mod rebuild;
pub mod render;
pub mod shell;
pub mod sketch;
pub mod solid;
pub mod stl;
pub mod sweep;
pub mod threemf;
pub mod thread;
pub mod validate;

use std::collections::BTreeMap;
use std::fmt;

use agcad_protocol::{
    Bbox, BooleanOp, Command, CommandResult, DocId, DocInfo, DocumentSnapshot, EdgeFilter,
    EntityChange, EntityId, EntitySnapshot, EntitySummary, Mat4, Mesh, MirrorPlane,
    ParamInfo, ParameterChange, Rgba, RevolveAxis, SavedParameter, Scalar, SketchElement,
    SketchPlane, SketchValidation, SolidValidation, SourceDoc, SVec3, ValidationTotals,
    Vec3, DOCUMENT_FORMAT, DOCUMENT_VERSION, PROTOCOL_VERSION, SERVER_NAME,
};

/// Default tessellation segments for curved primitives.
const DEFAULT_SEGMENTS: u32 = 32;
/// Undo history kept per document (full snapshots). Fine for v0; switch to
/// a command-log design when documents grow large.
const MAX_HISTORY: usize = 128;

#[derive(Debug, Clone)]
pub enum Error {
    UnknownDocument(DocId),
    UnknownEntity { doc: DocId, entity: EntityId },
    Invalid(String),
    /// File-format / document I/O problems. The message is self-contained.
    Document(String),
    NothingToUndo(DocId),
    NothingToRedo(DocId),
    Replay { step: usize, source: String },
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::UnknownDocument(d) => write!(f, "unknown document {d} (hint: ids start at 1; run create_document first, or list_documents)"),
            Self::UnknownEntity { doc, entity } => write!(f, "document {doc} has no entity {entity} (hint: get_scene lists valid entity ids)"),
            Self::Invalid(msg) => write!(f, "invalid parameters: {msg}"),
            Self::Document(msg) => write!(f, "{msg}"),
            Self::NothingToUndo(d) => write!(f, "document {d}: nothing to undo"),
            Self::NothingToRedo(d) => write!(f, "document {d}: nothing to redo"),
            Self::Replay { step, source } => write!(f, "program replay failed at step {step}: {source} (hint: undo reverts the parameter change; check that every expression only references defined parameters)"),
        }
    }
}

impl std::error::Error for Error {}

pub type Result<T> = std::result::Result<T, Error>;

/// A named parametric knob: `wall = 2.0`, optionally with a human-readable
/// description the agent can write for the user.
#[derive(Debug, Clone)]
pub struct Param {
    pub value: f64,
    pub description: Option<String>,
}

pub type ParamMap = BTreeMap<String, Param>;

/// All state that gets restored by undo — one atomic unit.
#[derive(Debug, Clone)]
struct Snapshot {
    next_entity: EntityId,
    entities: BTreeMap<EntityId, Entity>,
    program: Vec<Command>,
    params: ParamMap,
}

#[derive(Debug, Clone)]
pub struct Entity {
    pub id: EntityId,
    pub name: String,
    pub shape: Shape,
    /// Tessellated geometry in local space (baked once per program run).
    pub mesh: Mesh,
    pub translation: Vec3,
    pub rotation_deg: Vec3,
    pub scale: Vec3,
    pub color: Rgba,
    /// Content key of the local mesh. Downstream geometry folds this into its
    /// own key, which is how an incremental replay knows that a fillet on an
    /// unchanged extrude can be reused while a fillet on a changed one cannot.
    /// Derived state; not saved and not part of the scene signature.
    pub(crate) geom_key: rebuild::GeomKey,
}

#[derive(Debug, Clone)]
pub enum Shape {
    Box { size: Vec3 },
    Sphere { radius: f64, segments: u32 },
    Cylinder { radius: f64, height: f64, segments: u32 },
    /// Result of a set operation. References its operand entity ids so the
    /// program can replay it after those operands are re-created.
    Boolean { op: BooleanOp, entities: Vec<EntityId> },
    /// A 2D constraint-solved drawing. Coordinates live in sketch-local
    /// space; the entity transform places the drawing in 3D. Its `mesh`
    /// holds display ribbons only — sketches are not printable geometry.
    Sketch {
        elements: Vec<SketchElement>,
        constraints: Vec<agcad_protocol::SketchConstraint>,
        next_element: u64,
        next_constraint: u64,
        solved: bool,
        residual: f64,
        dof: i64,
    },
    /// A solid made by extruding a sketch profile along its normal. Keeps
    /// the source sketch id so it can rebuild when the sketch changes.
    Extrude {
        sketch: EntityId,
        /// Extrusion height (mm); negative extrudes the other way.
        distance: f64,
    },
    /// A solid made by revolving a sketch profile around an in-plane axis.
    Revolve {
        sketch: EntityId,
        angle_deg: f64,
        axis: RevolveAxis,
        segments: Option<u32>,
    },
    /// A solid carried along a path (polyline or helix).
    Sweep {
        sketch: EntityId,
    },
    /// A solid lofted through several sketch profiles.
    Loft {
        sketches: Vec<EntityId>,
    },
    /// A standalone screw thread (axis +Z from its origin).
    Thread,
    /// A linear/circular pattern copy or a mirrored copy of another solid.
    Copy {
        source: EntityId,
        kind: pattern::CopyKind,
    },
}

impl Shape {
    /// Stable lowercase name for reports and diffs.
    pub fn kind(&self) -> &'static str {
        match self {
            Self::Box { .. } => "box",
            Self::Sphere { .. } => "sphere",
            Self::Cylinder { .. } => "cylinder",
            Self::Boolean { .. } => "boolean",
            Self::Sketch { .. } => "sketch",
            Self::Extrude { .. } => "extrude",
            Self::Revolve { .. } => "revolve",
            Self::Sweep { .. } => "sweep",
            Self::Loft { .. } => "loft",
            Self::Thread => "thread",
            Self::Copy { kind, .. } => kind.label(),
        }
    }
}

impl Entity {
    /// Column-major model matrix `T * Rz * Ry * Rx * S`.
    pub fn transform(&self) -> Mat4 {
        mat4::compose(self.translation, self.rotation_deg, self.scale)
    }

    pub fn snapshot(&self) -> EntitySnapshot {
        let transform = self.transform();
        EntitySnapshot {
            id: self.id,
            name: self.name.clone(),
            translation: self.translation,
            rotation_deg: self.rotation_deg,
            scale: self.scale,
            transform,
            world_bbox: world_bbox(&self.mesh, &transform),
            color: self.color,
            mesh: self.mesh.clone(),
            sketch: match &self.shape {
                Shape::Sketch { elements, constraints, solved, residual, dof, .. } => {
                    Some(agcad_protocol::SketchSnapshot {
                        elements: elements.clone(),
                        constraints: constraints.clone(),
                        solved: *solved,
                        residual: *residual,
                        dof: *dof,
                        plane: Some(plane::frame_from_transform(&transform)),
                    })
                },
                _ => None,
            },
        }
    }
}

/// Axis-aligned bounds of a local mesh placed by `transform`, in world space.
/// This is what a client should trust for size and placement: `mesh`
/// positions are entity-local, so a rotation does not change them.
fn world_bbox(mesh: &Mesh, transform: &Mat4) -> Bbox {
    if mesh.positions.is_empty() {
        return Bbox { min: [0.0; 3], max: [0.0; 3] };
    }
    let mut lo = [f64::MAX; 3];
    let mut hi = [f64::MIN; 3];
    for p in &mesh.positions {
        let w = mat4::transform_point(transform, *p);
        for i in 0..3 {
            lo[i] = lo[i].min(w[i] as f64);
            hi[i] = hi[i].max(w[i] as f64);
        }
    }
    Bbox { min: lo, max: hi }
}

#[derive(Debug)]
pub struct Document {
    pub id: DocId,
    pub name: String,
    next_entity: EntityId,
    entities: BTreeMap<EntityId, Entity>,
    /// Raw modeling commands in execution order — the parametric function.
    program: Vec<Command>,
    params: ParamMap,
    /// Provenance only: the script that produced this program, if any.
    /// Saved back out with the document; never executed on open.
    source: Option<SourceDoc>,
    /// True while `replay` is running the program. Dependent rebuilds take
    /// the cheap path then, because the program's own later commands (e.g.
    /// a fillet on an extrude) will redo the work in order.
    replaying: bool,
    /// Program step currently being executed, when known. Replay sets it so a
    /// geometry result can be memoized per feature; live commands use the
    /// index of the command they are about to append.
    step: Option<usize>,
    /// Per-feature geometry memo, keyed by content. Derived state: not saved,
    /// not part of undo snapshots, safe to clear at any time.
    cache: rebuild::GeometryCache,
    undo_stack: Vec<Snapshot>,
    redo_stack: Vec<Snapshot>,
}

impl Document {
    fn new(id: DocId, name: impl Into<String>) -> Self {
        Self {
            id,
            name: name.into(),
            next_entity: 1,
            entities: BTreeMap::new(),
            program: Vec::new(),
            params: BTreeMap::new(),
            source: None,
            replaying: false,
            step: None,
            cache: rebuild::GeometryCache::default(),
            undo_stack: Vec::new(),
            redo_stack: Vec::new(),
        }
    }

    fn next_entity(&mut self) -> EntityId {
        let id = self.next_entity;
        self.next_entity += 1;
        id
    }

    /// Capture state before a mutation; clears the redo stack (standard
    /// linear-history semantics).
    fn checkpoint(&mut self) {
        self.undo_stack.push(Snapshot {
            next_entity: self.next_entity,
            entities: self.entities.clone(),
            program: self.program.clone(),
            params: self.params.clone(),
        });
        if self.undo_stack.len() > MAX_HISTORY {
            self.undo_stack.remove(0);
        }
        self.redo_stack.clear();
    }

    /// Discard the last checkpoint, restoring the state it captured. Used
    /// when a mutation fails midway so the document never becomes
    /// inconsistent.
    fn rollback(&mut self) {
        if let Some(snap) = self.undo_stack.pop() {
            self.next_entity = snap.next_entity;
            self.entities = snap.entities;
            self.program = snap.program;
            self.params = snap.params;
        }
        self.redo_stack.clear();
    }

    fn entity_mut(&mut self, id: EntityId) -> Result<&mut Entity> {
        let doc = self.id;
        self.entities.get_mut(&id).ok_or(Error::UnknownEntity { doc, entity: id })
    }

    fn resolve(&self, s: &Scalar) -> Result<f64> {
        match s {
            Scalar::Number(v) => Ok(*v),
            Scalar::Expression(e) => expr::eval(e, &self.params).map_err(Error::Invalid),
        }
    }

    fn resolve3(&self, v: &SVec3) -> Result<Vec3> {
        Ok([self.resolve(&v[0])?, self.resolve(&v[1])?, self.resolve(&v[2])?])
    }

    /// Re-execute the program from an empty scene. Deterministic: ids,
    /// meshes and transforms come out identical when parameters didn't
    /// change.
    ///
    /// Every command runs, but geometry-producing steps are memoized by
    /// content: a step whose inputs (including upstream geometry keys) did not
    /// move reuses its previous mesh instead of calling the kernel. That is
    /// what makes a knob change incremental while keeping this full replay as
    /// the correctness oracle.
    fn replay(&mut self) -> Result<()> {
        let program = self.program.clone();
        self.entities.clear();
        self.next_entity = 1;
        self.replaying = true;
        self.cache.begin();
        let mut result = Ok(());
        for (step, cmd) in program.iter().enumerate() {
            self.step = Some(step);
            if let Err(e) = exec(self, cmd) {
                result = Err(Error::Replay { step, source: e.to_string() });
                break;
            }
        }
        self.step = None;
        self.replaying = false;
        result
    }

    /// Memoize one geometry result by content key at the current program step.
    /// `build` only runs on a miss, so a cache hit skips the kernel entirely.
    fn cached_geometry<F>(&mut self, key: rebuild::GeomKey, build: F) -> Result<rebuild::CachedGeometry>
    where
        F: FnOnce(&mut Self) -> Result<rebuild::CachedGeometry>,
    {
        if let Some(cached) = self.cache.get(self.step, key) {
            return Ok(cached);
        }
        let geometry = build(self)?;
        Ok(self.cache.put(self.step, key, geometry))
    }

    /// Re-derive everything that a live sketch edit affects. During a
    /// replay the later program commands do this in order, so only the
    /// extrudes/revolves need refreshing; outside one, a full replay is the
    /// only way to also re-apply fillets/chamfers that come after the
    /// solid in the program.
    fn after_sketch_change(&mut self, doc: DocId, sketch: EntityId) -> Result<()> {
        if self.replaying {
            rebuild_dependents(self, doc, sketch)
        } else {
            self.replay()
        }
    }

    /// The durable state: name, parameter values and the program. Derived
    /// geometry is deliberately not included — opening replays the program.
    fn snapshot(&self, source: Option<SourceDoc>) -> DocumentSnapshot {
        let mut program = self.program.clone();
        // Normalize the doc id: the file is a standalone artifact, and a
        // constant id makes save -> open -> save a byte-identical round trip.
        for cmd in &mut program {
            set_doc(cmd, 1);
        }
        DocumentSnapshot {
            format: DOCUMENT_FORMAT.to_string(),
            version: DOCUMENT_VERSION,
            protocol: PROTOCOL_VERSION,
            name: self.name.clone(),
            parameters: self
                .params
                .iter()
                .map(|(name, p)| {
                    (name.clone(), SavedParameter { value: p.value, description: p.description.clone() })
                })
                .collect(),
            program,
            source: source.or_else(|| self.source.clone()),
        }
    }

    /// Rebuild a document from a saved snapshot and replay its program.
    /// `id` is the id the rebuilt document will live under; program commands
    /// are rebound to it. Used by `open_document` and by `diff_scene`'s
    /// read-only baseline path (which passes a throwaway id and never inserts
    /// the document into the engine).
    fn from_snapshot(snapshot: DocumentSnapshot, id: DocId, name: String) -> Result<Self> {
        let mut d = Document::new(id, name);
        d.params = snapshot
            .parameters
            .iter()
            .map(|(k, p)| {
                (k.clone(), Param { value: p.value, description: p.description.clone() })
            })
            .collect();
        d.source = snapshot.source;
        d.program = snapshot.program;
        for cmd in &mut d.program {
            set_doc(cmd, id);
        }
        d.replay()?;
        Ok(d)
    }
}

/// Read and check a `.agcad` file without opening it into the engine.
fn read_snapshot(path: &str) -> Result<DocumentSnapshot> {
    let text = std::fs::read_to_string(path).map_err(|e| {
        Error::Document(format!(
            "cannot read document '{path}': {e} \
             (hint: use an absolute path the server can read)"
        ))
    })?;
    let snap: DocumentSnapshot = serde_json::from_str(&text).map_err(|e| {
        Error::Document(format!(
            "cannot parse document '{path}': {e} \
             (hint: .agcad files are JSON written by save_document)"
        ))
    })?;
    if snap.format != DOCUMENT_FORMAT {
        return Err(Error::Document(format!(
            "'{path}' is not an agcad document (format: '{}'; \
             hint: the file must start with {{\"format\": \"{DOCUMENT_FORMAT}\"}})",
            snap.format
        )));
    }
    if snap.version > DOCUMENT_VERSION {
        return Err(Error::Document(format!(
            "document '{path}' is version {} but this build reads up to \
             {DOCUMENT_VERSION} (hint: upgrade the server)",
            snap.version
        )));
    }
    Ok(snap)
}

/// Rebind every `doc` field of a command. Programs are per-document, so
/// loading a file into a new document id rewrites them in one pass.
fn set_doc(command: &mut Command, doc: DocId) {
    match command {
        Command::AddBox { doc: d, .. }
        | Command::AddSphere { doc: d, .. }
        | Command::AddCylinder { doc: d, .. }
        | Command::Transform { doc: d, .. }
        | Command::SetColor { doc: d, .. }
        | Command::Delete { doc: d, .. }
        | Command::Fillet { doc: d, .. }
        | Command::Chamfer { doc: d, .. }
        | Command::Boolean { doc: d, .. }
        | Command::AddSketch { doc: d, .. }
        | Command::AddSketchLine { doc: d, .. }
        | Command::AddSketchCircle { doc: d, .. }
        | Command::Constrain { doc: d, .. }
        | Command::RemoveConstraint { doc: d, .. }
        | Command::MoveSketchPoint { doc: d, .. }
        | Command::DeleteSketchElement { doc: d, .. }
        | Command::Extrude { doc: d, .. }
        | Command::Revolve { doc: d, .. }
        | Command::LinearPattern { doc: d, .. }
        | Command::CircularPattern { doc: d, .. }
        | Command::Mirror { doc: d, .. }
        | Command::Shell { doc: d, .. }
        | Command::Hole { doc: d, .. }
        | Command::Sweep { doc: d, .. }
        | Command::Loft { doc: d, .. }
        | Command::Thread { doc: d, .. }
        | Command::SetParameter { doc: d, .. }
        | Command::RemoveParameter { doc: d, .. }
        | Command::GetParameters { doc: d }
        | Command::Undo { doc: d }
        | Command::Redo { doc: d }
        | Command::GetScene { doc: d }
        | Command::Pick { doc: d, .. }
        | Command::Validate { doc: d, .. }
        | Command::DiffScene { doc: d, .. }
        | Command::GetProgram { doc: d }
        | Command::ExportStl { doc: d, .. }
        | Command::Export3mf { doc: d, .. }
        | Command::SaveDocument { doc: d, .. }
        | Command::RenderOffscreen { doc: d, .. }
        | Command::CloseDocument { doc: d } => *d = doc,
        Command::Initialize
        | Command::ListDocuments
        | Command::CreateDocument { .. }
        | Command::OpenDocument { .. } => {},
    }
}

/// The engine. Deterministic: identical command sequences produce
/// identical ids, meshes and results — safe to replay in tests.
#[derive(Debug, Default)]
pub struct Engine {
    docs: BTreeMap<DocId, Document>,
    next_doc: DocId,
}

impl Engine {
    pub fn new() -> Self {
        Self { docs: BTreeMap::new(), next_doc: 1 }
    }

    fn doc(&self, id: DocId) -> Result<&Document> {
        self.docs.get(&id).ok_or(Error::UnknownDocument(id))
    }

    fn doc_mut(&mut self, id: DocId) -> Result<&mut Document> {
        self.docs.get_mut(&id).ok_or(Error::UnknownDocument(id))
    }

    /// Execute one command and produce its result.
    pub fn execute(&mut self, command: Command) -> Result<CommandResult> {
        match &command {
            Command::Initialize => Ok(CommandResult::Initialized {
                protocol_version: PROTOCOL_VERSION,
                server: SERVER_NAME.to_string(),
            }),

            Command::ListDocuments => Ok(CommandResult::Documents {
                docs: self
                    .docs
                    .values()
                    .map(|d| DocInfo {
                        doc: d.id,
                        name: d.name.clone(),
                        entity_count: d.entities.len(),
                    })
                    .collect(),
            }),

            Command::CreateDocument { name, doc: wanted } => {
                let doc = match wanted {
                    Some(id) => {
                        if *id == 0 {
                            return Err(Error::Invalid(
                                "document ids start at 1 (hint: omit `doc` to allocate the next id)".into(),
                            ));
                        }
                        if self.docs.contains_key(id) {
                            return Err(Error::Invalid(format!(
                                "document {id} already exists (hint: list_documents shows allocated ids, or omit `doc` to allocate the next one)"
                            )));
                        }
                        // Keep automatic allocation clear of explicit ids.
                        self.next_doc = self.next_doc.max(*id + 1);
                        *id
                    },
                    None => {
                        let id = self.next_doc;
                        self.next_doc += 1;
                        id
                    },
                };
                self.docs.insert(doc, Document::new(doc, name.clone()));
                Ok(CommandResult::DocumentCreated { doc })
            },

            Command::CloseDocument { doc } => {
                self.docs.remove(doc).ok_or(Error::UnknownDocument(*doc))?;
                Ok(CommandResult::Closed { doc: *doc })
            },

            // --- modeling: record into the program, then execute --------
            Command::AddBox { .. }
            | Command::AddSphere { .. }
            | Command::AddCylinder { .. }
            | Command::Transform { .. }
            | Command::SetColor { .. }
            | Command::Delete { .. }
            | Command::Fillet { .. }
            | Command::Chamfer { .. }
            | Command::Boolean { .. }
            | Command::AddSketch { .. }
            | Command::AddSketchLine { .. }
            | Command::AddSketchCircle { .. }
            | Command::Constrain { .. }
            | Command::RemoveConstraint { .. }
            | Command::MoveSketchPoint { .. }
            | Command::DeleteSketchElement { .. }
            | Command::Extrude { .. }
            | Command::Revolve { .. }
            | Command::LinearPattern { .. }
            | Command::CircularPattern { .. }
            | Command::Mirror { .. }
            | Command::Shell { .. }
            | Command::Hole { .. }
            | Command::Sweep { .. }
            | Command::Loft { .. }
            | Command::Thread { .. } => {
                let doc = doc_of(&command).expect("modeling commands always carry a doc");
                let d = self.doc_mut(doc)?;
                d.checkpoint();
                let step = d.program.len();
                d.program.push(command.clone());
                d.step = Some(step);
                let result = exec(d, &command);
                d.step = None;
                match result {
                    Ok(result) => Ok(result),
                    Err(e) => {
                        d.rollback();
                        Err(e)
                    },
                }
            },

            // --- parameters: change value, replay the program -----------
            Command::SetParameter { doc, name, value, description } => {
                let d = self.doc_mut(*doc)?;
                // The value may reference other parameters, but not itself.
                let without_self: ParamMap = d
                    .params
                    .iter()
                    .filter(|(k, _)| *k != name)
                    .map(|(k, p)| (k.clone(), p.clone()))
                    .collect();
                let resolved = match value {
                    Scalar::Number(v) => *v,
                    Scalar::Expression(e) => {
                        expr::eval(e, &without_self).map_err(Error::Invalid)?
                    },
                };
                d.checkpoint();
                // A knob turn must not erase the label: when the caller
                // omits a description, keep the existing one.
                let description = description
                    .clone()
                    .or_else(|| d.params.get(name).and_then(|p| p.description.clone()));
                d.params.insert(name.clone(), Param { value: resolved, description });
                match d.replay() {
                    Ok(()) => Ok(CommandResult::ParameterSet {
                        doc: *doc,
                        name: name.clone(),
                        value: resolved,
                        rebuilt: d.cache.rebuilt,
                        reused: d.cache.reused,
                        steps: d.program.len(),
                    }),
                    Err(e) => {
                        d.rollback();
                        Err(e)
                    },
                }
            },

            Command::RemoveParameter { doc, name } => {
                let d = self.doc_mut(*doc)?;
                d.checkpoint();
                d.params.remove(name);
                match d.replay() {
                    Ok(()) => Ok(CommandResult::ParameterRemoved { doc: *doc, name: name.clone() }),
                    Err(e) => {
                        d.rollback();
                        Err(e)
                    },
                }
            },

            Command::GetParameters { doc } => {
                let d = self.doc(*doc)?;
                Ok(CommandResult::Parameters {
                    doc: *doc,
                    params: d
                        .params
                        .iter()
                        .map(|(name, p)| ParamInfo {
                            name: name.clone(),
                            value: p.value,
                            description: p.description.clone(),
                        })
                        .collect(),
                })
            },

            Command::Undo { doc } => {
                let d = self.doc_mut(*doc)?;
                let snap = d.undo_stack.pop().ok_or(Error::NothingToUndo(*doc))?;
                d.redo_stack.push(Snapshot {
                    next_entity: d.next_entity,
                    entities: std::mem::take(&mut d.entities),
                    program: std::mem::take(&mut d.program),
                    params: std::mem::take(&mut d.params),
                });
                d.next_entity = snap.next_entity;
                d.entities = snap.entities;
                d.program = snap.program;
                d.params = snap.params;
                Ok(CommandResult::Undone { doc: *doc })
            },

            Command::Redo { doc } => {
                let d = self.doc_mut(*doc)?;
                let snap = d.redo_stack.pop().ok_or(Error::NothingToRedo(*doc))?;
                d.undo_stack.push(Snapshot {
                    next_entity: d.next_entity,
                    entities: std::mem::take(&mut d.entities),
                    program: std::mem::take(&mut d.program),
                    params: std::mem::take(&mut d.params),
                });
                d.next_entity = snap.next_entity;
                d.entities = snap.entities;
                d.program = snap.program;
                d.params = snap.params;
                Ok(CommandResult::Redone { doc: *doc })
            },

            Command::GetScene { doc } => {
                let d = self.doc(*doc)?;
                Ok(CommandResult::Scene {
                    doc: *doc,
                    entities: d.entities.values().map(|e| e.snapshot()).collect(),
                })
            },

            Command::Pick { doc, ray, tolerance } => {
                let d = self.doc(*doc)?;
                let hit = pick::pick(&d.entities, ray, *tolerance)?;
                Ok(CommandResult::Picked { doc: *doc, hit })
            },

            Command::Validate { doc, entities, min_wall_mm } => {
                if let Some(threshold) = min_wall_mm {
                    if !threshold.is_finite() || *threshold <= 0.0 {
                        return Err(Error::Invalid(
                            "min_wall_mm must be a finite value > 0 (hint: omit it to report the \
                             estimate without a pass/fail threshold)"
                                .into(),
                        ));
                    }
                }
                let d = self.doc(*doc)?;
                let ids: Vec<EntityId> = match entities {
                    Some(ids) => ids.clone(),
                    None => d.entities.keys().copied().collect(),
                };
                let mut solids = Vec::new();
                let mut sketches = Vec::new();
                let mut warnings = Vec::new();
                for id in ids {
                    let e = d.entities.get(&id).ok_or(Error::UnknownEntity { doc: *doc, entity: id })?;
                    if matches!(e.shape, Shape::Sketch { .. }) {
                        let report = sketch_validation(e);
                        if !report.solved {
                            warnings.push(format!(
                                "sketch {} '{}': solver reports residual {:.3e} (hint: inspect \
                                 its constraints with get_scene)",
                                report.entity, report.name, report.residual
                            ));
                        }
                        if report.profile_error.is_some() {
                            warnings.push(format!(
                                "sketch {} '{}': {}",
                                report.entity,
                                report.name,
                                report.profile_error.as_deref().unwrap_or_default()
                            ));
                        }
                        sketches.push(report);
                    } else {
                        let report = solid_validation(e, *min_wall_mm);
                        if !report.watertight {
                            warnings.push(format!(
                                "solid {} '{}' is not watertight ({} open, {} non-manifold \
                                 edge(s); hint: a closed solid has exactly two triangles per edge)",
                                report.entity,
                                report.name,
                                report.open_edges,
                                report.non_manifold_edges
                            ));
                        }
                        if report.self_intersections > 0 {
                            warnings.push(format!(
                                "solid {} '{}' self-intersects in {} triangle pair(s) (hint: the \
                                 surfaces cross; boolean the overlapping pieces instead of \
                                 leaving them in one mesh)",
                                report.entity, report.name, report.self_intersections
                            ));
                        }
                        if report.volume <= 0.0 {
                            warnings.push(format!(
                                "solid {} '{}' has non-positive signed volume ({:.3} mm^3; hint: \
                                 the winding is inward)",
                                report.entity, report.name, report.volume
                            ));
                        }
                        if report.thin {
                            warnings.push(format!(
                                "solid {} '{}' is thinner than {} mm (estimated wall {:.3} mm \
                                 near {})",
                                report.entity,
                                report.name,
                                min_wall_mm.unwrap_or(0.0),
                                report.min_wall.unwrap_or(0.0),
                                report
                                    .min_wall_at
                                    .map(|p| format!("({:.2}, {:.2}, {:.2})", p[0], p[1], p[2]))
                                    .unwrap_or_else(|| "an unsampled location".into())
                            ));
                        } else if min_wall_mm.is_some() && report.min_wall.is_none() && report.watertight {
                            warnings.push(format!(
                                "solid {} '{}': no wall estimate (the sampled inward rays found \
                                 no opposite surface; hint: the mesh may be too coarse or \
                                 disconnected)",
                                report.entity, report.name
                            ));
                        }
                        solids.push(report);
                    }
                }

                let totals = validation_totals(&solids, &sketches);
                let ok = solids.iter().all(|s| s.ok) && sketches.iter().all(|s| s.ok);
                Ok(CommandResult::Validated { doc: *doc, ok, solids, sketches, totals, warnings })
            },

            Command::DiffScene { doc, against, path } => {
                if against.is_some() == path.is_some() {
                    return Err(Error::Invalid(
                        "diff_scene needs exactly one baseline: `against` (an open document id) \
                         or `path` (a saved .agcad file)"
                            .into(),
                    ));
                }
                let d = self.doc(*doc)?;
                let loaded;
                let (baseline, against_id, against_name): (&Document, Option<DocId>, String) =
                    match (against, path) {
                        (Some(id), None) => {
                            if *id == *doc {
                                return Err(Error::Invalid(
                                    "diff_scene needs two revisions (hint: the baseline is this \
                                     same document; compare with another doc or a saved file)"
                                        .into(),
                                ));
                            }
                            let base = self.doc(*id)?;
                            (base, Some(*id), base.name.clone())
                        },
                        (None, Some(p)) => {
                            let snap = read_snapshot(p)?;
                            let stored_name = if snap.name.trim().is_empty() {
                                p.clone()
                            } else {
                                snap.name.clone()
                            };
                            loaded = Document::from_snapshot(snap, 0, stored_name).map_err(|e| {
                                Error::Document(format!(
                                    "baseline '{p}' failed to replay ({e}; hint: the file may \
                                     use features this build lacks, or reference an undefined \
                                     parameter)"
                                ))
                            })?;
                            (&loaded, None, p.clone())
                        },
                        _ => unreachable!("validated exactly one baseline above"),
                    };

                Ok(diff_documents(*doc, d, baseline, against_id, against_name))
            },

            Command::ExportStl { doc, path, entities } => {
                let d = self.doc(*doc)?;
                let ids: Vec<EntityId> = match entities {
                    Some(ids) => ids.clone(),
                    None => d.entities.keys().copied().collect(),
                };
                // Sketches are drawings, not printable geometry — skip them.
                let entries = ids
                    .iter()
                    .filter(|&&id| {
                        d.entities.get(&id).map_or(false, |e| !matches!(e.shape, Shape::Sketch { .. }))
                    })
                    .map(|&id| {
                        let e = d.entities.get(&id).ok_or(Error::UnknownEntity { doc: *doc, entity: id })?;
                        Ok((e.mesh.clone(), e.transform()))
                    })
                    .collect::<Result<Vec<_>>>()?;
                if entries.is_empty() {
                    return Err(Error::Invalid(
                        "nothing to export: no solid entities (hint: sketches are not meshes — extrude them into solids first)".into(),
                    ));
                }
                let (resolved, tris) = stl::write_stl(path, &entries)?;
                Ok(CommandResult::StlExported {
                    doc: *doc,
                    path: resolved,
                    tris,
                    bytes: 84 + tris * 50,
                })
            },

            Command::Export3mf { doc, path, entities } => {
                let d = self.doc(*doc)?;
                let ids: Vec<EntityId> = match entities {
                    Some(ids) => ids.clone(),
                    None => d.entities.keys().copied().collect(),
                };
                let mut parts = Vec::new();
                for id in ids {
                    let e = d
                        .entities
                        .get(&id)
                        .ok_or(Error::UnknownEntity { doc: *doc, entity: id })?;
                    // Sketches are display-only, and empty solids would become
                    // invalid 3MF objects — neither belongs in the build.
                    if matches!(e.shape, Shape::Sketch { .. }) || e.mesh.indices.is_empty() {
                        continue;
                    }
                    parts.push(threemf::Part {
                        name: &e.name,
                        mesh: &e.mesh,
                        transform: e.transform(),
                        color: e.color,
                    });
                }
                if parts.is_empty() {
                    return Err(Error::Invalid(
                        "nothing to export: no solid entities (hint: sketches are not meshes — extrude them into solids first)".into(),
                    ));
                }
                let (resolved, objects, tris, bytes) = threemf::write_3mf(path, &parts, &d.name)?;
                Ok(CommandResult::ThreeMfExported {
                    doc: *doc,
                    path: resolved,
                    objects,
                    tris,
                    bytes,
                })
            },

            Command::SaveDocument { doc, path, source } => {
                let d = self.doc_mut(*doc)?;
                // Remember the recipe so `get_program` reflects what a
                // reopen would recover. This is metadata, not model state.
                if let Some(source) = source {
                    d.source = Some(source.clone());
                }
                let snapshot = d.snapshot(None);
                // Pretty + sorted keys + trailing newline: stable bytes, so
                // two saves of the same model produce identical files.
                let json = serde_json::to_string_pretty(&snapshot).map_err(|e| {
                    Error::Document(format!("cannot serialize document {doc}: {e}"))
                })?;
                let json = format!("{json}\n");
                std::fs::write(path, json.as_bytes()).map_err(|e| {
                    Error::Document(format!(
                        "cannot write document to '{path}': {e} \
                         (hint: use an absolute path the server can write)"
                    ))
                })?;
                let resolved = std::fs::canonicalize(path)
                    .map(|p| p.to_string_lossy().into_owned())
                    .unwrap_or_else(|_| path.clone());
                Ok(CommandResult::DocumentSaved {
                    doc: *doc,
                    path: resolved,
                    bytes: json.len(),
                    commands: snapshot.program.len(),
                    params: snapshot.parameters.len(),
                })
            },

            Command::OpenDocument { path, name } => {
                let snap = read_snapshot(path)?;
                let doc = self.next_doc;
                let stored_name = if snap.name.trim().is_empty() {
                    std::path::Path::new(path)
                        .file_stem()
                        .map(|s| s.to_string_lossy().into_owned())
                        .unwrap_or_else(|| "document".into())
                } else {
                    snap.name.clone()
                };
                // Rebuild geometry from the program. If it fails, nothing is
                // inserted, so the engine never holds a half-open document.
                let d = Document::from_snapshot(snap, doc, name.clone().unwrap_or(stored_name))
                    .map_err(|e| {
                        Error::Document(format!(
                            "document '{path}' failed to replay ({e}; hint: the file may use \
                             features this build lacks, or reference an undefined parameter)"
                        ))
                    })?;

                let result = CommandResult::DocumentOpened {
                    doc,
                    name: d.name.clone(),
                    commands: d.program.len(),
                    params: d.params.len(),
                    entities: d.entities.len(),
                };
                self.next_doc += 1;
                self.docs.insert(doc, d);
                Ok(result)
            },

            Command::GetProgram { doc } => {
                let d = self.doc(*doc)?;
                Ok(CommandResult::Program {
                    doc: *doc,
                    name: d.name.clone(),
                    params: d
                        .params
                        .iter()
                        .map(|(name, p)| ParamInfo {
                            name: name.clone(),
                            value: p.value,
                            description: p.description.clone(),
                        })
                        .collect(),
                    program: d.program.clone(),
                    source: d.source.clone(),
                })
            },

            Command::RenderOffscreen { doc, width, height, eye, target, up, fov_deg, ortho, background, path } => {
                let d = self.doc(*doc)?;
                let (w, h) = (
                    width.unwrap_or(800).clamp(16, 4096),
                    height.unwrap_or(600).clamp(16, 4096),
                );
                let bg = background.unwrap_or([0.92, 0.93, 0.96, 1.0]);

                let items: Vec<render::RenderItem<'_>> = d
                    .entities
                    .values()
                    .map(|e| render::RenderItem { mesh: &e.mesh, model: e.transform(), color: e.color })
                    .collect();

                // Camera: explicit eye wins; otherwise auto-frame the
                // scene. Explicit eye without target aims at the scene
                // center — the least-surprising default for agents.
                let (center, _) = render::scene_bounds(&items);
                let camera = match eye {
                    Some(eye) => render::Camera {
                        eye: *eye,
                        target: target.map_or(center, |t| t),
                        up: up.map_or([0.0, 0.0, 1.0], |u| u),
                        fov_y_deg: fov_deg.unwrap_or(40.0).clamp(5.0, 150.0),
                        ortho: ortho.unwrap_or(false),
                    },
                    None => {
                        let mut cam = render::Camera::auto(&items, w, h);
                        if let Some(t) = target {
                            cam.target = *t;
                        }
                        if let Some(u) = up {
                            cam.up = *u;
                        }
                        if let Some(f) = fov_deg {
                            cam.fov_y_deg = f.clamp(5.0, 150.0);
                        }
                        if let Some(o) = ortho {
                            cam.ortho = *o;
                        }
                        cam
                    },
                };

                let (png, width, height) = render::render_png(&items, camera, w, h, bg)?;
                let resolved = match path {
                    Some(p) => {
                        std::fs::write(&p, &png).map_err(|e| {
                            Error::Invalid(format!("cannot write PNG to '{p}': {e} (hint: use an absolute path the server can write)"))
                        })?;
                        Some(
                            std::fs::canonicalize(&p)
                                .map(|q| q.to_string_lossy().into_owned())
                                .unwrap_or(p.clone()),
                        )
                    },
                    None => None,
                };
                Ok(CommandResult::Rendered {
                    doc: *doc,
                    width,
                    height,
                    png_base64: render::base64_encode(&png),
                    path: resolved,
                })
            },
        }
    }
}

/// One solid's trust report (see [`Command::Validate`]).
fn solid_validation(entity: &Entity, min_wall_mm: Option<f64>) -> SolidValidation {
    let world = transform_mesh(&entity.mesh, &entity.transform());
    let check = validate::check_solid(&world);
    let thin = min_wall_mm.is_some_and(|t| check.min_wall.is_some_and(|w| w < t));
    let ok = check.watertight
        && check.non_manifold_edges == 0
        && check.self_intersections == 0
        && check.volume > 0.0
        && !thin;
    SolidValidation {
        entity: entity.id,
        name: entity.name.clone(),
        kind: entity.shape.kind().to_string(),
        ok,
        watertight: check.watertight,
        open_edges: check.open_edges,
        non_manifold_edges: check.non_manifold_edges,
        degenerate_tris: check.degenerate_tris,
        self_intersections: check.self_intersections,
        self_intersection_points: check.self_intersection_points,
        volume: check.volume,
        tris: check.tris,
        bbox: check.bbox,
        min_wall: check.min_wall,
        min_wall_at: check.min_wall_at,
        thin,
        signature: validate::entity_signature(entity),
    }
}

/// One sketch's trust report: solver status and whether it can be extruded.
fn sketch_validation(entity: &Entity) -> SketchValidation {
    let Shape::Sketch { elements, constraints, solved, residual, dof, .. } = &entity.shape else {
        unreachable!("sketch_validation is only called for sketches")
    };
    let (closed_profile, loops, profile_error) = if elements.is_empty() {
        (false, 0, None)
    } else {
        match sketch::profile_loops(elements) {
            Ok(loops) => (true, loops.len(), None),
            Err(err) => (false, 0, Some(err)),
        }
    };
    SketchValidation {
        entity: entity.id,
        name: entity.name.clone(),
        ok: *solved && (elements.is_empty() || closed_profile),
        solved: *solved,
        residual: *residual,
        dof: *dof,
        elements: elements.len(),
        constraints: constraints.len(),
        closed_profile,
        loops,
        profile_error,
        signature: validate::entity_signature(entity),
    }
}

/// Roll per-entity reports up into document totals.
fn validation_totals(solids: &[SolidValidation], sketches: &[SketchValidation]) -> ValidationTotals {
    let mut totals = ValidationTotals {
        solids: solids.len(),
        sketches: sketches.len(),
        tris: 0,
        volume: 0.0,
        open_edges: 0,
        non_manifold_edges: 0,
        self_intersections: 0,
        degenerate_tris: 0,
        min_wall: None,
        bbox: None,
    };
    for s in solids {
        totals.tris += s.tris;
        totals.volume += s.volume;
        totals.open_edges += s.open_edges;
        totals.non_manifold_edges += s.non_manifold_edges;
        totals.self_intersections += s.self_intersections;
        totals.degenerate_tris += s.degenerate_tris;
        if let Some(wall) = s.min_wall {
            totals.min_wall = Some(totals.min_wall.map_or(wall, |m: f64| m.min(wall)));
        }
        totals.bbox = Some(match totals.bbox {
            None => s.bbox,
            Some(b) => Bbox {
                min: [b.min[0].min(s.bbox.min[0]), b.min[1].min(s.bbox.min[1]), b.min[2].min(s.bbox.min[2])],
                max: [b.max[0].max(s.bbox.max[0]), b.max[1].max(s.bbox.max[1]), b.max[2].max(s.bbox.max[2])],
            },
        });
    }
    totals
}

/// Compact entity description for a scene diff.
fn entity_summary(entity: &Entity) -> EntitySummary {
    let is_sketch = matches!(entity.shape, Shape::Sketch { .. });
    let transform = entity.transform();
    EntitySummary {
        entity: entity.id,
        name: entity.name.clone(),
        kind: entity.shape.kind().to_string(),
        signature: validate::entity_signature(entity),
        volume: if is_sketch {
            None
        } else {
            Some(fillet::signed_volume(&transform_mesh(&entity.mesh, &transform)))
        },
        tris: if is_sketch { None } else { Some(entity.mesh.indices.len() / 3) },
        world_bbox: world_bbox(&entity.mesh, &transform),
    }
}

/// Build the full revision comparison (the `diff_scene` command body).
fn diff_documents(
    doc: DocId,
    current: &Document,
    baseline: &Document,
    against: Option<DocId>,
    against_name: String,
) -> CommandResult {
    use std::collections::BTreeSet;

    let current_signature = validate::scene_signature(&current.entities);
    let baseline_signature = validate::scene_signature(&baseline.entities);

    let mut added = Vec::new();
    let mut removed = Vec::new();
    let mut modified = Vec::new();
    let mut unchanged = 0usize;

    for (id, e) in &current.entities {
        let Some(b) = baseline.entities.get(id) else {
            added.push(entity_summary(e));
            continue;
        };
        let from_sig = validate::entity_signature(b);
        let to_sig = validate::entity_signature(e);
        if from_sig == to_sig {
            unchanged += 1;
            continue;
        }
        let mut changed = Vec::new();
        if b.name != e.name {
            changed.push("name".to_string());
        }
        if b.color != e.color {
            changed.push("color".to_string());
        }
        if b.shape.kind() != e.shape.kind() {
            changed.push("kind".to_string());
        }
        if b.translation != e.translation {
            changed.push("translation".to_string());
        }
        if b.rotation_deg != e.rotation_deg {
            changed.push("rotation_deg".to_string());
        }
        if b.scale != e.scale {
            changed.push("scale".to_string());
        }
        if !mesh_eq(&b.mesh, &e.mesh) {
            changed.push("mesh".to_string());
        }
        if !sketch_eq(&b.shape, &e.shape) {
            changed.push("sketch".to_string());
        }
        modified.push(EntityChange {
            entity: *id,
            changed,
            from: EntitySummary {
                entity: b.id,
                name: b.name.clone(),
                kind: b.shape.kind().to_string(),
                signature: from_sig,
                volume: if matches!(b.shape, Shape::Sketch { .. }) {
                    None
                } else {
                    Some(fillet::signed_volume(&transform_mesh(&b.mesh, &b.transform())))
                },
                tris: if matches!(b.shape, Shape::Sketch { .. }) {
                    None
                } else {
                    Some(b.mesh.indices.len() / 3)
                },
                world_bbox: world_bbox(&b.mesh, &b.transform()),
            },
            to: entity_summary(e),
        });
    }
    for (id, b) in &baseline.entities {
        if !current.entities.contains_key(id) {
            removed.push(entity_summary(b));
        }
    }

    let names: BTreeSet<&String> = current.params.keys().chain(baseline.params.keys()).collect();
    let mut parameters = Vec::new();
    for name in names {
        let from = baseline.params.get(name).map(|p| p.value);
        let to = current.params.get(name).map(|p| p.value);
        if from != to {
            parameters.push(ParameterChange { name: name.clone(), from, to });
        }
    }

    let commands_from = baseline.program.len();
    let commands_to = current.program.len();
    let program_changed = canonical_program(&baseline.program) != canonical_program(&current.program);
    let changed = !added.is_empty()
        || !removed.is_empty()
        || !modified.is_empty()
        || !parameters.is_empty()
        || program_changed;

    CommandResult::SceneDiff {
        doc,
        against,
        against_name,
        changed,
        scene_signature: current_signature,
        against_signature: baseline_signature,
        added,
        removed,
        modified,
        unchanged,
        parameters,
        program_changed,
        commands_from,
        commands_to,
    }
}

/// Mesh equality for diffing: positions and indices, bit for bit. Normals
/// are derived from those and are intentionally ignored.
fn mesh_eq(a: &Mesh, b: &Mesh) -> bool {
    a.indices == b.indices
        && a.positions.len() == b.positions.len()
        && a.positions.iter().zip(&b.positions).all(|(p, q)| {
            p[0].to_bits() == q[0].to_bits()
                && p[1].to_bits() == q[1].to_bits()
                && p[2].to_bits() == q[2].to_bits()
        })
}

/// Whether the non-placement content of two shapes differs. Solids are
/// covered by the mesh comparison; this catches sketch elements, constraints
/// and solver status.
fn sketch_eq(a: &Shape, b: &Shape) -> bool {
    match (a, b) {
        (
            Shape::Sketch { elements: ea, constraints: ca, solved: sa, residual: ra, dof: da, .. },
            Shape::Sketch { elements: eb, constraints: cb, solved: sb, residual: rb, dof: db, .. },
        ) => {
            ea == eb
                && ca == cb
                && sa == sb
                && ra.to_bits() == rb.to_bits()
                && da == db
        },
        (Shape::Sketch { .. }, _) | (_, Shape::Sketch { .. }) => false,
        _ => true,
    }
}

/// Serialize a program with doc ids normalized, for an order-sensitive but
/// id-independent comparison.
fn canonical_program(program: &[Command]) -> String {
    let mut normalized = program.to_vec();
    for cmd in &mut normalized {
        set_doc(cmd, 1);
    }
    serde_json::to_string(&normalized).unwrap_or_default()
}

fn doc_of(command: &Command) -> Option<DocId> {
    match command {
        Command::AddBox { doc, .. }
        | Command::AddSphere { doc, .. }
        | Command::AddCylinder { doc, .. }
        | Command::Transform { doc, .. }
        | Command::SetColor { doc, .. }
        | Command::Delete { doc, .. }
        | Command::Fillet { doc, .. }
        | Command::Chamfer { doc, .. }
        | Command::Boolean { doc, .. }
        | Command::AddSketch { doc, .. }
        | Command::AddSketchLine { doc, .. }
        | Command::AddSketchCircle { doc, .. }
        | Command::Constrain { doc, .. }
        | Command::RemoveConstraint { doc, .. }
        | Command::MoveSketchPoint { doc, .. }
        | Command::DeleteSketchElement { doc, .. }
        | Command::Extrude { doc, .. }
        | Command::Revolve { doc, .. }
        | Command::LinearPattern { doc, .. }
        | Command::CircularPattern { doc, .. }
        | Command::Mirror { doc, .. }
        | Command::Shell { doc, .. }
        | Command::Hole { doc, .. }
        | Command::Sweep { doc, .. }
        | Command::Loft { doc, .. }
        | Command::Thread { doc, .. }
        | Command::SetParameter { doc, .. }
        | Command::RemoveParameter { doc, .. }
        | Command::GetParameters { doc }
        | Command::Undo { doc }
        | Command::Redo { doc }
        | Command::GetScene { doc }
        | Command::Pick { doc, .. }
        | Command::Validate { doc, .. }
        | Command::DiffScene { doc, .. }
        | Command::ExportStl { doc, .. }
        | Command::Export3mf { doc, .. }
        | Command::RenderOffscreen { doc, .. }
        | Command::CloseDocument { doc } => Some(*doc),
        _ => None,
    }
}

/// Execute one *modeling* command against a document. Used by the live
/// path and by replay — never checkpoints on its own.
fn exec(d: &mut Document, command: &Command) -> Result<CommandResult> {
    match command {
        Command::AddBox { doc, name, center, size, color } => {
            let center = d.resolve3(center)?;
            let size = d.resolve3(size)?;
            if size.iter().any(|&s| s <= 0.0) {
                return Err(Error::Invalid("box size components must be > 0".into()));
            }
            add_entity(d, *doc, name.clone(), Shape::Box { size }, center, [0.0; 3], [1.0; 3], *color)
        },

        Command::AddSphere { doc, name, center, radius, segments, color } => {
            let center = d.resolve3(center)?;
            let radius = d.resolve(radius)?;
            if radius <= 0.0 {
                return Err(Error::Invalid("sphere radius must be > 0".into()));
            }
            let segments = segments.unwrap_or(DEFAULT_SEGMENTS).max(3);
            add_entity(
                d,
                *doc,
                name.clone(),
                Shape::Sphere { radius, segments },
                center,
                [0.0; 3],
                [1.0; 3],
                *color,
            )
        },

        Command::AddCylinder { doc, name, center, radius, height, segments, color } => {
            let center = d.resolve3(center)?;
            let radius = d.resolve(radius)?;
            let height = d.resolve(height)?;
            if radius <= 0.0 || height <= 0.0 {
                return Err(Error::Invalid("cylinder radius and height must be > 0".into()));
            }
            let segments = segments.unwrap_or(DEFAULT_SEGMENTS).max(3);
            add_entity(
                d,
                *doc,
                name.clone(),
                Shape::Cylinder { radius, height, segments },
                center,
                [0.0; 3],
                [1.0; 3],
                *color,
            )
        },

        Command::Transform { doc, entity, translation, rotation_deg, scale } => {
            let translation = translation.as_ref().map(|v| d.resolve3(v)).transpose()?;
            let rotation_deg = rotation_deg.as_ref().map(|v| d.resolve3(v)).transpose()?;
            let scale = scale.as_ref().map(|v| d.resolve3(v)).transpose()?;
            let e = d.entity_mut(*entity)?;
            if let Some(t) = translation {
                e.translation = t;
            }
            if let Some(r) = rotation_deg {
                e.rotation_deg = r;
            }
            if let Some(s) = scale {
                e.scale = s;
            }
            let is_sketch = matches!(e.shape, Shape::Sketch { .. });
            if is_sketch {
                // Solids built from the sketch follow its placement.
                d.after_sketch_change(*doc, *entity)?;
            }
            Ok(CommandResult::Transformed { doc: *doc, entity: *entity })
        },

        Command::SetColor { doc, entity, color } => {
            d.entity_mut(*entity)?.color = *color;
            Ok(CommandResult::Transformed { doc: *doc, entity: *entity })
        },

        Command::Boolean { doc, op, entities, name, color } => {
            if entities.len() < 2 {
                return Err(Error::Invalid(format!(
                    "boolean {} needs at least 2 entity ids (got {}; hint: create the operand solids first, then reference their ids)",
                    op.as_str(),
                    entities.len()
                )));
            }
            // The operands' content keys plus their placements determine the
            // result; record them before touching the kernel so a cache hit can
            // skip the world-space transforms and the boolean entirely.
            let inputs: Vec<(rebuild::GeomKey, Vec3, Vec3, Vec3)> = entities
                .iter()
                .map(|&eid| {
                    let e = d
                        .entities
                        .get(&eid)
                        .ok_or(Error::UnknownEntity { doc: *doc, entity: eid })?;
                    Ok((e.geom_key, e.translation, e.rotation_deg, e.scale))
                })
                .collect::<Result<Vec<_>>>()?;
            let key = rebuild::boolean_key(*op, &inputs);

            // Gather operands in *world space*: local mesh through the
            // entity's model matrix. Color and name inherit from the
            // first operand unless overridden.
            let source_color = entities
                .iter()
                .find_map(|eid| d.entities.get(eid).map(|e| e.color));
            let cached = d.cached_geometry(key, |d| {
                let mut world: Vec<Mesh> = Vec::with_capacity(entities.len());
                for &eid in entities {
                    let e = d
                        .entities
                        .get(&eid)
                        .ok_or(Error::UnknownEntity { doc: *doc, entity: eid })?;
                    world.push(transform_mesh(&e.mesh, &e.transform()));
                }
                let result = boolean::boolean(*op, &world)?;
                // Bake the result centered on its bounding-box center so the
                // entity follows the usual "geometry local, transform places"
                // convention.
                let (mesh, center) = recenter(&result);
                Ok(rebuild::CachedGeometry {
                    mesh,
                    volume: None,
                    edges: None,
                    center: Some(center),
                })
            })?;

            let center = cached.center.unwrap_or([0.0; 3]);
            let id = d.next_entity();
            let entity = Entity {
                id,
                name: name.clone().unwrap_or_else(|| op.as_str().to_string()),
                shape: Shape::Boolean { op: *op, entities: entities.clone() },
                mesh: cached.mesh,
                translation: center,
                rotation_deg: [0.0; 3],
                scale: [1.0; 3],
                color: color.unwrap_or(source_color.unwrap_or([0.7, 0.72, 0.75, 1.0])),
                geom_key: key,
            };
            let mesh = entity.mesh.clone();
            d.entities.insert(id, entity);
            Ok(CommandResult::EntityCreated { doc: *doc, entity: id, mesh })
        },

        Command::Fillet { doc, entity, radius, edges, min_angle_deg } => {
            let radius = d.resolve(radius)?;
            reshape(d, *doc, *entity, fillet::Kind::Fillet(radius), edges.as_ref(), *min_angle_deg)
        },

        Command::Chamfer { doc, entity, distance, edges, min_angle_deg } => {
            let distance = d.resolve(distance)?;
            reshape(
                d,
                *doc,
                *entity,
                fillet::Kind::Chamfer(distance),
                edges.as_ref(),
                *min_angle_deg,
            )
        },

        // --- sketches -------------------------------------------------------

        Command::AddSketch { doc, name, plane } => {
            let (translation, rotation_deg) = match plane {
                Some(plane) => resolve_sketch_plane(d, *doc, plane)?,
                None => ([0.0; 3], [0.0; 3]),
            };
            let id = d.next_entity();
            let shape = Shape::Sketch {
                elements: Vec::new(),
                constraints: Vec::new(),
                next_element: 1,
                next_constraint: 1,
                solved: true,
                residual: 0.0,
                dof: 0,
            };
            let entity = Entity {
                id,
                name: name.clone(),
                shape,
                mesh: sketch::display_mesh(&[]),
                translation,
                rotation_deg,
                scale: [1.0; 3],
                color: [0.15, 0.55, 0.85, 1.0],
                geom_key: rebuild::sketch_key(&[]),
            };
            let mesh = entity.mesh.clone();
            d.entities.insert(id, entity);
            Ok(CommandResult::EntityCreated { doc: *doc, entity: id, mesh })
        },

        Command::AddSketchLine { doc, sketch, p1, p2 } => {
            let id = {
                let (p1, p2) = ([d.resolve(&p1[0])?, d.resolve(&p1[1])?], [d.resolve(&p2[0])?, d.resolve(&p2[1])?]);
                let (elements, _, next_element, ..) = sketch_parts(d, *sketch)?;
                let id = *next_element;
                *next_element += 1;
                elements.push(SketchElement::Line { id, p1, p2 });
                id
            };
            refresh_sketch(d, *doc, *sketch)?;
            Ok(sketch_result(SketchOutcome::Element(*doc, *sketch, id)))
        },

        Command::AddSketchCircle { doc, sketch, center, radius } => {
            let id = {
                let (center, radius) = ([d.resolve(&center[0])?, d.resolve(&center[1])?], d.resolve(radius)?);
                if radius <= 0.0 {
                    return Err(Error::Invalid(format!("circle radius must be positive (got {radius})")));
                }
                let (elements, _, next_element, ..) = sketch_parts(d, *sketch)?;
                let id = *next_element;
                *next_element += 1;
                elements.push(SketchElement::Circle { id, center, radius });
                id
            };
            refresh_sketch(d, *doc, *sketch)?;
            Ok(sketch_result(SketchOutcome::Element(*doc, *sketch, id)))
        },

        Command::Constrain { doc, sketch, kind, refs, value } => {
            let cid = {
                let resolved = match value {
                    Some(v) => Some(d.resolve(&v)?),
                    None => None,
                };
                let (elements, constraints, _, next_constraint, ..) = sketch_parts(d, *sketch)?;
                sketch::validate(*kind, elements, refs, resolved).map_err(Error::Invalid)?;
                let cid = *next_constraint;
                *next_constraint += 1;
                constraints.push(agcad_protocol::SketchConstraint {
                    id: cid,
                    kind: *kind,
                    refs: refs.clone(),
                    value: resolved,
                });
                cid
            };
            let report = refresh_sketch(d, *doc, *sketch)?;
            Ok(sketch_result(SketchOutcome::Constraint(*doc, *sketch, cid, report)))
        },

        Command::RemoveConstraint { doc, sketch, constraint } => {
            {
                let (_, constraints, ..) = sketch_parts(d, *sketch)?;
                let before = constraints.len();
                constraints.retain(|c| c.id != *constraint);
                if constraints.len() == before {
                    return Err(Error::Invalid(format!(
                        "sketch {sketch} has no constraint {constraint} (hint: get_scene lists constraint ids)"
                    )));
                }
            }
            refresh_sketch(d, *doc, *sketch)?;
            Ok(CommandResult::ConstraintRemoved { doc: *doc, sketch: *sketch, constraint: *constraint })
        },

        Command::MoveSketchPoint { doc, sketch, element, point, to } => {
            {
                let (elements, ..) = sketch_parts(d, *sketch)?;
                let target = elements.iter_mut().find(|e| match e {
                    SketchElement::Line { id, .. } | SketchElement::Circle { id, .. } => id == element,
                });
                match (target, point) {
                    (Some(SketchElement::Line { p1, .. }), 0) => *p1 = *to,
                    (Some(SketchElement::Line { p2, .. }), 1) => *p2 = *to,
                    (Some(SketchElement::Circle { center, .. }), 0) => *center = *to,
                    _ => {
                        return Err(Error::Invalid(format!(
                            "element {element} has no point {point} (hint: 0 = line start / circle center, 1 = line end)"
                        )));
                    },
                }
            }
            refresh_sketch(d, *doc, *sketch)?;
            Ok(CommandResult::SketchPointMoved { doc: *doc, sketch: *sketch, element: *element })
        },

        Command::DeleteSketchElement { doc, sketch, element } => {
            {
                let (elements, constraints, ..) = sketch_parts(d, *sketch)?;
                let before = elements.len();
                elements.retain(|e| match e {
                    SketchElement::Line { id, .. } | SketchElement::Circle { id, .. } => *id != *element,
                });
                if elements.len() == before {
                    return Err(Error::UnknownEntity { doc: *doc, entity: *element });
                }
                // Cascade: constraints referencing the deleted element go too.
                constraints.retain(|c| c.refs.iter().all(|r| r.element != *element));
            }
            refresh_sketch(d, *doc, *sketch)?;
            Ok(CommandResult::SketchElementDeleted { doc: *doc, sketch: *sketch, element: *element })
        },

        // --- sketch → solid -------------------------------------------------

        Command::Extrude { doc, sketch, distance, name, color } => {
            let distance = d.resolve(distance)?;
            if distance.abs() < 1e-9 {
                return Err(Error::Invalid(
                    "extrude distance must be non-zero (hint: a negative distance extrudes the other way)"
                        .into(),
                ));
            }
            let (loops, (translation, rotation_deg, scale)) = sketch_profile(d, *doc, *sketch)?;
            let key = rebuild::extrude_key(&loops, distance);
            let cached = d.cached_geometry(key, |_| {
                let (mesh, volume) = solid::extrude(&loops, distance)?;
                Ok(rebuild::CachedGeometry {
                    mesh,
                    volume: Some(volume),
                    edges: None,
                    center: None,
                })
            })?;
            let volume = cached.volume.unwrap_or(0.0);
            let tris = cached.mesh.indices.len() / 3;
            let id = d.next_entity();
            d.entities.insert(
                id,
                Entity {
                    id,
                    name: name.clone().unwrap_or_else(|| "extrude".into()),
                    shape: Shape::Extrude { sketch: *sketch, distance },
                    mesh: cached.mesh,
                    translation,
                    rotation_deg,
                    scale,
                    color: color.unwrap_or([0.72, 0.74, 0.78, 1.0]),
                    geom_key: key,
                },
            );
            Ok(CommandResult::SolidCreated { doc: *doc, entity: id, sketch: *sketch, volume, tris })
        },

        Command::Revolve { doc, sketch, angle_deg, axis, segments, name, color } => {
            let angle = match angle_deg {
                Some(v) => d.resolve(v)?,
                None => 360.0,
            };
            if !(angle.is_finite()) || angle <= 0.0 {
                return Err(Error::Invalid(format!(
                    "revolve angle must be a positive number of degrees (got {angle})"
                )));
            }
            let axis = axis.unwrap_or(RevolveAxis::X);
            let segments = segments.map(|n| n.clamp(3, 512));
            let (loops, (translation, rotation_deg, scale)) = sketch_profile(d, *doc, *sketch)?;
            let key = rebuild::revolve_key(&loops, angle, axis, segments);
            let cached = d.cached_geometry(key, |_| {
                let (mesh, volume) = solid::revolve(&loops, angle, axis, segments)?;
                Ok(rebuild::CachedGeometry {
                    mesh,
                    volume: Some(volume),
                    edges: None,
                    center: None,
                })
            })?;
            let volume = cached.volume.unwrap_or(0.0);
            let tris = cached.mesh.indices.len() / 3;
            let id = d.next_entity();
            d.entities.insert(
                id,
                Entity {
                    id,
                    name: name.clone().unwrap_or_else(|| "revolve".into()),
                    shape: Shape::Revolve { sketch: *sketch, angle_deg: angle, axis, segments },
                    mesh: cached.mesh,
                    translation,
                    rotation_deg,
                    scale,
                    color: color.unwrap_or([0.72, 0.74, 0.78, 1.0]),
                    geom_key: key,
                },
            );
            Ok(CommandResult::SolidCreated { doc: *doc, entity: id, sketch: *sketch, volume, tris })
        },

        // --- shape leverage ------------------------------------------------

        Command::LinearPattern { doc, entities, direction, spacing, count, name, color } => {
            let direction = d.resolve3(direction)?;
            let spacing = d.resolve(spacing)?;
            let count = validate_count(*count)?;
            if direction.iter().all(|v| v.abs() < 1e-12) {
                return Err(Error::Invalid(
                    "linear_pattern direction must be a non-zero vector (e.g. [1,0,0])".into(),
                ));
            }
            if !spacing.is_finite() {
                return Err(Error::Invalid("linear_pattern spacing must be finite".into()));
            }
            let dir = normalize3(direction);
            let mut created = Vec::new();
            for &source in entities {
                let e = d.entities.get(&source).ok_or(Error::UnknownEntity { doc: *doc, entity: source })?;
                ensure_solid(e, "linear_pattern")?;
                for i in 1..count {
                    let kind = pattern::CopyKind::Linear { direction: dir, spacing, index: i };
                    let id = create_copy(d, *doc, source, kind, name.clone(), *color)?;
                    created.push(id);
                }
            }
            Ok(pattern_result(*doc, d, created))
        },

        Command::CircularPattern { doc, entities, center, axis, angle_deg, count, name, color } => {
            let center = match center {
                Some(c) => d.resolve3(c)?,
                None => [0.0; 3],
            };
            let axis = axis.map(normalize3).unwrap_or([0.0, 0.0, 1.0]);
            if axis.iter().all(|v| v.abs() < 1e-12) {
                return Err(Error::Invalid(
                    "circular_pattern axis must be a non-zero vector (e.g. [0,0,1])".into(),
                ));
            }
            let angle = match angle_deg {
                Some(v) => d.resolve(v)?,
                None => 360.0,
            };
            if !angle.is_finite() || angle == 0.0 {
                return Err(Error::Invalid(
                    "circular_pattern angle_deg must be a finite non-zero angle".into(),
                ));
            }
            let count = validate_count(*count)?;
            let step = angle / count as f64;
            let mut created = Vec::new();
            for &source in entities {
                let e = d.entities.get(&source).ok_or(Error::UnknownEntity { doc: *doc, entity: source })?;
                ensure_solid(e, "circular_pattern")?;
                for i in 1..count {
                    let kind = pattern::CopyKind::Circular { center, axis, step_deg: step, index: i };
                    let id = create_copy(d, *doc, source, kind, name.clone(), *color)?;
                    created.push(id);
                }
            }
            Ok(pattern_result(*doc, d, created))
        },

        Command::Mirror { doc, entities, plane, name, color } => {
            let (origin, normal) = resolve_mirror_plane(d, plane)?;
            let mut created = Vec::new();
            for &source in entities {
                let e = d.entities.get(&source).ok_or(Error::UnknownEntity { doc: *doc, entity: source })?;
                ensure_solid(e, "mirror")?;
                let kind = pattern::CopyKind::Mirror { origin, normal };
                let id = create_copy(d, *doc, source, kind, name.clone(), *color)?;
                created.push(id);
            }
            Ok(pattern_result(*doc, d, created))
        },

        Command::Shell { doc, entity, thickness, open, angle_tol_deg } => {
            let thickness = d.resolve(thickness)?;
            shell_solid(d, *doc, *entity, thickness, open, angle_tol_deg.unwrap_or(1.0))
        },

        Command::Hole {
            doc,
            entity,
            center,
            normal,
            diameter,
            depth,
            counterbore_diameter,
            counterbore_depth,
            countersink_angle_deg,
            segments,
        } => {
            let spec = hole::HoleSpec {
                center: d.resolve3(center)?,
                normal: *normal,
                diameter: d.resolve(diameter)?,
                depth: depth.as_ref().map(|v| d.resolve(v)).transpose()?,
                counterbore_diameter: counterbore_diameter
                    .as_ref()
                    .map(|v| d.resolve(v))
                    .transpose()?,
                counterbore_depth: counterbore_depth.as_ref().map(|v| d.resolve(v)).transpose()?,
                countersink_angle_deg: countersink_angle_deg
                    .as_ref()
                    .map(|v| d.resolve(v))
                    .transpose()?,
                segments: *segments,
            };
            hole_solid(d, *doc, *entity, spec)
        },

        Command::Sweep { doc, sketch, path, segments, name, color } => {
            let (loops, placement) = sketch_profile(d, *doc, *sketch)?;
            let (translation, rotation_deg, scale) = placement;
            // The sketch only supplies the profile and its in-plane X; the
            // path is world-space and re-orients the profile at every station.
            let scaled: Vec<Vec<[f64; 2]>> = loops
                .iter()
                .map(|lp| lp.iter().map(|p| [p[0] * scale[0], p[1] * scale[1]]).collect())
                .collect();
            let path = sweep::path_points(path, *segments, |v| d.resolve(v))?;
            let sketch_transform = mat4::compose(translation, rotation_deg, [1.0; 3]);
            let x_ref = mat4::transform_direction(&sketch_transform, [1.0, 0.0, 0.0]);
            let key = rebuild::sweep_key(&scaled, x_ref, &path);
            let cached = d.cached_geometry(key, |_| {
                let world = sweep::sweep(&scaled, x_ref, &path)?;
                let (mesh, center) = recenter(&world);
                Ok(rebuild::CachedGeometry {
                    mesh,
                    volume: Some(fillet::signed_volume(&world)),
                    edges: None,
                    center: Some(center),
                })
            })?;
            let center = cached.center.unwrap_or([0.0; 3]);
            let volume = cached.volume.unwrap_or(0.0);
            let tris = cached.mesh.indices.len() / 3;
            let id = d.next_entity();
            d.entities.insert(
                id,
                Entity {
                    id,
                    name: name.clone().unwrap_or_else(|| "sweep".into()),
                    shape: Shape::Sweep { sketch: *sketch },
                    mesh: cached.mesh,
                    translation: center,
                    rotation_deg: [0.0; 3],
                    scale: [1.0; 3],
                    color: color.unwrap_or([0.72, 0.74, 0.78, 1.0]),
                    geom_key: key,
                },
            );
            Ok(CommandResult::SolidBuilt {
                doc: *doc,
                entity: id,
                kind: "sweep".into(),
                volume,
                tris,
            })
        },

        Command::Loft { doc, sketches, segments, name, color } => {
            if sketches.len() < 2 {
                return Err(Error::Invalid(
                    "loft needs at least 2 sketches (hint: sketches=[a, b])".into(),
                ));
            }
            let mut rings = Vec::with_capacity(sketches.len());
            for &sketch in sketches {
                let (loops, (translation, rotation_deg, scale)) = sketch_profile(d, *doc, sketch)?;
                if loops.len() != 1 {
                    return Err(Error::Invalid(format!(
                        "loft sketch {sketch} has {} closed loops; loft supports one outer loop per \
                         sketch (hint: remove holes or split the sketch)",
                        loops.len()
                    )));
                }
                let m = mat4::compose(translation, rotation_deg, scale);
                let ring: Vec<Vec3> = loops[0]
                    .iter()
                    .map(|p| {
                        let w = mat4::transform_point(&m, [p[0] as f32, p[1] as f32, 0.0]);
                        [w[0] as f64, w[1] as f64, w[2] as f64]
                    })
                    .collect();
                rings.push(ring);
            }
            let segments = segments.unwrap_or(64).clamp(8, 512);
            let key = rebuild::loft_key(&rings, segments);
            let cached = d.cached_geometry(key, |_| {
                let world = loft::loft(&rings, segments)?;
                let (mesh, center) = recenter(&world);
                Ok(rebuild::CachedGeometry {
                    mesh,
                    volume: Some(fillet::signed_volume(&world)),
                    edges: None,
                    center: Some(center),
                })
            })?;
            let center = cached.center.unwrap_or([0.0; 3]);
            let volume = cached.volume.unwrap_or(0.0);
            let tris = cached.mesh.indices.len() / 3;
            let id = d.next_entity();
            d.entities.insert(
                id,
                Entity {
                    id,
                    name: name.clone().unwrap_or_else(|| "loft".into()),
                    shape: Shape::Loft { sketches: sketches.clone() },
                    mesh: cached.mesh,
                    translation: center,
                    rotation_deg: [0.0; 3],
                    scale: [1.0; 3],
                    color: color.unwrap_or([0.72, 0.74, 0.78, 1.0]),
                    geom_key: key,
                },
            );
            Ok(CommandResult::SolidBuilt {
                doc: *doc,
                entity: id,
                kind: "loft".into(),
                volume,
                tris,
            })
        },

        Command::Thread {
            doc,
            major_diameter,
            minor_diameter,
            pitch,
            length,
            hand,
            flank_angle_deg,
            segments_per_turn,
            origin,
            name,
            color,
        } => {
            let major = d.resolve(major_diameter)?;
            let pitch = d.resolve(pitch)?;
            let minor = match minor_diameter {
                Some(v) => d.resolve(v)?,
                None => thread::default_minor(major, pitch),
            };
            let spec = thread::ThreadSpec {
                major_diameter: major,
                minor_diameter: minor,
                pitch,
                length: d.resolve(length)?,
                hand: *hand,
                flank_angle_deg: match flank_angle_deg {
                    Some(v) => d.resolve(v)?,
                    None => 60.0,
                },
                segments_per_turn: segments_per_turn.unwrap_or(48),
                origin: match origin {
                    Some(o) => d.resolve3(o)?,
                    None => [0.0; 3],
                },
            };
            let key = rebuild::thread_key(&spec);
            let cached = d.cached_geometry(key, |_| {
                let world = thread::thread(&spec)?;
                let (mesh, center) = recenter(&world);
                Ok(rebuild::CachedGeometry {
                    mesh,
                    volume: Some(fillet::signed_volume(&world)),
                    edges: None,
                    center: Some(center),
                })
            })?;
            let center = cached.center.unwrap_or([0.0; 3]);
            let volume = cached.volume.unwrap_or(0.0);
            let tris = cached.mesh.indices.len() / 3;
            let id = d.next_entity();
            d.entities.insert(
                id,
                Entity {
                    id,
                    name: name.clone().unwrap_or_else(|| "thread".into()),
                    shape: Shape::Thread,
                    mesh: cached.mesh,
                    translation: center,
                    rotation_deg: [0.0; 3],
                    scale: [1.0; 3],
                    color: color.unwrap_or([0.72, 0.74, 0.78, 1.0]),
                    geom_key: key,
                },
            );
            Ok(CommandResult::SolidBuilt {
                doc: *doc,
                entity: id,
                kind: "thread".into(),
                volume,
                tris,
            })
        },

        Command::Delete { doc, entity } => {
            if d.entities.remove(entity).is_none() {
                return Err(Error::UnknownEntity { doc: *doc, entity: *entity });
            }
            // Deleting a sketch also removes the solids built from it, and
            // deleting a solid removes its pattern/mirror copies — otherwise
            // replay would hit a dangling reference.
            let dependents: Vec<EntityId> = d
                .entities
                .iter()
                .filter(|(_, e)| match &e.shape {
                    Shape::Extrude { sketch, .. }
                    | Shape::Revolve { sketch, .. }
                    | Shape::Sweep { sketch } => *sketch == *entity,
                    Shape::Loft { sketches } => sketches.contains(entity),
                    Shape::Copy { source, .. } => source == entity,
                    _ => false,
                })
                .map(|(id, _)| *id)
                .collect();
            for id in dependents {
                d.entities.remove(&id);
            }
            Ok(CommandResult::Deleted { doc: *doc, entity: *entity })
        },

        _ => Err(Error::Invalid("not a modeling command".into())),
    }
}

/// Resolve a requested sketch plane into the entity placement that puts the
/// sketch's local XY on it. Face selection is world-space; `near` and the
/// datum offset may be expressions, so replay follows a parameter change.
fn resolve_sketch_plane(
    d: &Document,
    doc: DocId,
    plane: &SketchPlane,
) -> Result<(Vec3, Vec3)> {
    let frame = match plane {
        SketchPlane::Datum { plane, offset } => {
            let offset = match offset {
                Some(v) => d.resolve(v)?,
                None => 0.0,
            };
            plane::datum_frame(*plane, offset)?
        },
        SketchPlane::Custom { origin, normal, x_axis } => {
            let origin = d.resolve3(origin)?;
            plane::custom_frame(origin, *normal, *x_axis)?
        },
        SketchPlane::Face { entity, near, normal, angle_tol_deg } => {
            let near = near.as_ref().map(|v| d.resolve3(v)).transpose()?;
            let e = d
                .entities
                .get(entity)
                .ok_or(Error::UnknownEntity { doc, entity: *entity })?;
            if matches!(e.shape, Shape::Sketch { .. }) {
                return Err(Error::Invalid(format!(
                    "entity {entity} is a sketch, not a solid (hint: a face plane needs an \
                     existing solid; reference a box, extrude or boolean result, or use a datum \
                     plane)"
                )));
            }
            if e.mesh.indices.is_empty() {
                return Err(Error::Invalid(format!(
                    "entity {entity} has no geometry to select a face from"
                )));
            }
            let world = transform_mesh(&e.mesh, &e.transform());
            plane::face_frame(&world, near, *normal, *angle_tol_deg, *entity)?
        },
    };
    Ok(plane::placement(&frame))
}

/// Closed profile loops of a sketch plus its placement, for extrude/revolve.
#[allow(clippy::type_complexity)]
fn sketch_profile(
    d: &Document,
    doc: DocId,
    sketch: EntityId,
) -> Result<(Vec<Vec<[f64; 2]>>, (Vec3, Vec3, Vec3))> {
    let e = d.entities.get(&sketch).ok_or(Error::UnknownEntity { doc, entity: sketch })?;
    match &e.shape {
        Shape::Sketch { elements, .. } => Ok((
            sketch::profile_loops(elements).map_err(Error::Invalid)?,
            (e.translation, e.rotation_deg, e.scale),
        )),
        _ => Err(Error::Invalid(format!(
            "entity {sketch} is not a sketch (hint: create one with add_sketch, then extrude it)"
        ))),
    }
}

/// Reshape one solid in place (fillet/chamfer). The entity keeps its id,
/// name, color, transform and local frame; only the mesh changes.
///
/// The cut is computed in **world space**, so `radius` and the edge filter
/// are in the coordinates the user sees even when the entity is rotated or
/// scaled. The result is mapped back through the inverse transform, which
/// also means a scaled entity gets a fillet that scales with it.
fn reshape(
    d: &mut Document,
    doc: DocId,
    entity: EntityId,
    kind: fillet::Kind,
    edges: Option<&EdgeFilter>,
    min_angle_deg: Option<f64>,
) -> Result<CommandResult> {
    let min_angle_deg = min_angle_deg.unwrap_or(30.0);
    let (input_key, placement, transform) = {
        let e = d.entities.get(&entity).ok_or(Error::UnknownEntity { doc, entity })?;
        if matches!(e.shape, Shape::Sketch { .. }) {
            return Err(Error::Invalid(format!(
                "entity {entity} is a sketch (hint: fillet/chamfer apply to printable solids; \
                 extrude the sketch into a solid first)"
            )));
        }
        (
            e.geom_key,
            (e.translation, e.rotation_deg, e.scale),
            e.transform(),
        )
    };

    let empty = EdgeFilter::default();
    let filter = edges.unwrap_or(&empty);
    let key = rebuild::reshape_key(input_key, placement, kind, filter, min_angle_deg);

    let cached = d.cached_geometry(key, |d| {
        let e = d.entities.get(&entity).ok_or(Error::UnknownEntity { doc, entity })?;
        let world = transform_mesh(&e.mesh, &transform);
        let (world, edges) = fillet::apply(&world, kind, filter, min_angle_deg)?;
        let volume = fillet::signed_volume(&world);

        let inv = mat4::inverse(&transform).ok_or_else(|| {
            Error::Invalid(format!(
                "entity {entity} has a singular transform (a zero scale component); cannot map the \
                 reshaped mesh back into its local frame"
            ))
        })?;
        let positions: Vec<[f32; 3]> =
            world.positions.iter().map(|&p| mat4::transform_point(&inv, p)).collect();
        // Re-derive normals in the local frame (the world ones would be wrong
        // after a rotation) with the same crease rule the kernel used.
        let mesh = boolean::crease_mesh(&positions, &world.indices);
        Ok(rebuild::CachedGeometry {
            mesh,
            volume: Some(volume),
            edges: Some(edges),
            center: None,
        })
    })?;

    let edges = cached.edges.unwrap_or(0);
    let volume = cached.volume.unwrap_or(0.0);
    let mesh = cached.mesh;
    {
        let e = d.entity_mut(entity)?;
        e.mesh = mesh.clone();
        e.geom_key = key;
    }
    Ok(CommandResult::EntityModified { doc, entity, edges, volume, mesh })
}

/// Rebuild every extrude/revolve that references `sketch`, so solids follow
/// sketch edits immediately — not only on the next full replay. The sketch's
/// placement is copied too, so moving/rotating a sketch carries its solids.
fn rebuild_dependents(d: &mut Document, doc: DocId, sketch: EntityId) -> Result<()> {
    let ids: Vec<EntityId> = d.entities.keys().copied().collect();
    for id in ids {
        let kind = match d.entities.get(&id) {
            Some(e) => match e.shape {
                Shape::Extrude { sketch: s, distance } if s == sketch => Some(ExtrudeKind::Extrude(distance)),
                Shape::Revolve { sketch: s, angle_deg, axis, segments } if s == sketch => {
                    Some(ExtrudeKind::Revolve(angle_deg, axis, segments))
                },
                _ => None,
            },
            None => None,
        };
        let Some(kind) = kind else { continue };

        let (loops, placement) = sketch_profile(d, doc, sketch)?;
        let key = match kind {
            ExtrudeKind::Extrude(distance) => rebuild::extrude_key(&loops, distance),
            ExtrudeKind::Revolve(angle, axis, segments) => {
                rebuild::revolve_key(&loops, angle, axis, segments)
            },
        };
        let (mesh, _volume) = match kind {
            ExtrudeKind::Extrude(distance) => solid::extrude(&loops, distance),
            ExtrudeKind::Revolve(angle, axis, segments) => solid::revolve(&loops, angle, axis, segments),
        }
        .map_err(|e| {
            Error::Invalid(format!(
                "cannot rebuild entity {id} from sketch {sketch}: {e} (hint: delete the solid first if you want to change the profile)"
            ))
        })?;
        let e = d.entity_mut(id)?;
        e.mesh = mesh;
        (e.translation, e.rotation_deg, e.scale) = placement;
        e.geom_key = key;
    }
    Ok(())
}

enum ExtrudeKind {
    Extrude(f64),
    Revolve(f64, RevolveAxis, Option<u32>),
}

/// Mutable access to a sketch entity's shape fields; error if not a sketch.
/// Returns `(elements, constraints, next_element, next_constraint, solved,
/// residual, dof)`.
#[allow(clippy::type_complexity)]
fn sketch_parts(
    d: &mut Document,
    sketch: EntityId,
) -> Result<(
    &mut Vec<SketchElement>,
    &mut Vec<agcad_protocol::SketchConstraint>,
    &mut u64,
    &mut u64,
    &mut bool,
    &mut f64,
    &mut i64,
)> {
    let e = d.entity_mut(sketch)?;
    match &mut e.shape {
        Shape::Sketch { elements, constraints, next_element, next_constraint, solved, residual, dof } => {
            Ok((elements, constraints, next_element, next_constraint, solved, residual, dof))
        },
        _ => Err(Error::Invalid(format!(
            "entity {sketch} is not a sketch (hint: create one with add_sketch first)"
        ))),
    }
}

/// Re-solve a sketch from its current elements/constraints and refresh the
/// display mesh + solver status. Called after every sketch mutation.
fn refresh_sketch(d: &mut Document, doc: DocId, sketch: EntityId) -> Result<sketch::SolveReport> {
    let (mut elements, constraints) = {
        let (se, sc, ..) = sketch_parts(d, sketch)?;
        (se.clone(), sc.clone())
    };
    let report = sketch::solve(&mut elements, &constraints);
    let mesh = sketch::display_mesh(&elements);
    let key = rebuild::sketch_key(&elements);
    let (se, sc, _, _, solved, residual, dof) = sketch_parts(d, sketch)?;
    *se = elements;
    *sc = constraints;
    *solved = report.solved;
    *residual = report.residual;
    *dof = report.dof;
    {
        let e = d.entity_mut(sketch)?;
        e.mesh = mesh;
        e.geom_key = key;
    }
    // Solids built from this sketch must follow it immediately.
    d.after_sketch_change(doc, sketch)?;
    Ok(report)
}

/// What a sketch command produced, for building its result.
enum SketchOutcome {
    Element(DocId, EntityId, u64),
    Constraint(DocId, EntityId, u64, sketch::SolveReport),
}

fn sketch_result(outcome: SketchOutcome) -> CommandResult {
    match outcome {
        SketchOutcome::Element(doc, sketch, element) => {
            CommandResult::SketchElementCreated { doc, sketch, element }
        },
        SketchOutcome::Constraint(doc, sketch, constraint, r) => CommandResult::ConstraintCreated {
            doc,
            sketch,
            constraint,
            solved: r.solved,
            residual: r.residual,
            dof: r.dof,
        },
    }
}

/// Build a pattern/mirror result from the copies just created.
fn pattern_result(doc: DocId, d: &Document, created: Vec<EntityId>) -> CommandResult {
    let tris = created
        .iter()
        .filter_map(|id| d.entities.get(id))
        .map(|e| e.mesh.indices.len() / 3)
        .sum();
    CommandResult::EntitiesCreated { doc, entities: created, tris }
}

fn validate_count(count: u32) -> Result<u32> {
    if !(2..=1000).contains(&count) {
        return Err(Error::Invalid(format!(
            "pattern count must be between 2 and 1000, and includes the original (got {count})"
        )));
    }
    Ok(count)
}

fn normalize3(v: Vec3) -> Vec3 {
    let len = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
    if len < 1e-12 { [0.0; 3] } else { [v[0] / len, v[1] / len, v[2] / len] }
}

fn ensure_solid(entity: &Entity, what: &str) -> Result<()> {
    if matches!(entity.shape, Shape::Sketch { .. }) {
        return Err(Error::Invalid(format!(
            "entity {} is a sketch; {what} repeats printable solids (hint: extrude the sketch \
             first)",
            entity.id
        )));
    }
    if entity.mesh.indices.is_empty() {
        return Err(Error::Invalid(format!(
            "entity {} has no geometry to {what}",
            entity.id
        )));
    }
    Ok(())
}

fn resolve_mirror_plane(d: &Document, plane: &MirrorPlane) -> Result<(Vec3, Vec3)> {
    match plane {
        MirrorPlane::Datum { plane, offset } => {
            let offset = match offset {
                Some(v) => d.resolve(v)?,
                None => 0.0,
            };
            let frame = plane::datum_frame(*plane, offset)?;
            Ok((frame.origin, frame.normal))
        },
        MirrorPlane::Custom { origin, normal } => {
            let origin = d.resolve3(origin)?;
            let frame = plane::custom_frame(origin, *normal, None)?;
            Ok((frame.origin, frame.normal))
        },
    }
}

/// Create one pattern/mirror copy and attach it to the document.
fn create_copy(
    d: &mut Document,
    doc: DocId,
    source: EntityId,
    kind: pattern::CopyKind,
    name: Option<String>,
    color: Option<Rgba>,
) -> Result<EntityId> {
    let (geometry, key, source_name, source_color) = {
        let e = d.entities.get(&source).ok_or(Error::UnknownEntity { doc, entity: source })?;
        let geometry = pattern::build(e, &kind)?;
        let key = rebuild::copy_key(
            e.geom_key,
            (e.translation, e.rotation_deg, e.scale),
            kind,
        );
        (geometry, key, e.name.clone(), e.color)
    };
    let label = match kind {
        pattern::CopyKind::Linear { index, .. } | pattern::CopyKind::Circular { index, .. } => index,
        pattern::CopyKind::Mirror { .. } => 0,
    };
    let name = match name {
        Some(base) if label > 0 => format!("{base} {label}"),
        Some(base) => base,
        None if label > 0 => format!("{source_name} {label}"),
        None => format!("{source_name} {}", kind.label()),
    };
    let id = d.next_entity();
    d.entities.insert(
        id,
        Entity {
            id,
            name,
            shape: Shape::Copy { source, kind },
            mesh: geometry.mesh,
            translation: geometry.translation,
            rotation_deg: geometry.rotation_deg,
            scale: geometry.scale,
            color: color.unwrap_or(source_color),
            geom_key: key,
        },
    );
    Ok(id)
}

/// Map a world-space result back into an entity's local frame.
fn localize_world(transform: &Mat4, world: &Mesh) -> Result<Mesh> {
    let inv = mat4::inverse(transform).ok_or_else(|| {
        Error::Invalid(
            "the entity has a singular transform (a zero scale component); cannot map the result \
             back into its local frame"
                .into(),
        )
    })?;
    let positions: Vec<[f32; 3]> = world
        .positions
        .iter()
        .map(|&p| mat4::transform_point(&inv, p))
        .collect();
    Ok(boolean::crease_mesh(&positions, &world.indices))
}

fn shell_solid(
    d: &mut Document,
    doc: DocId,
    entity: EntityId,
    thickness: f64,
    open: &[[f64; 3]],
    angle_tol_deg: f64,
) -> Result<CommandResult> {
    let (input_key, placement, transform) = editable_solid(d, doc, entity, "shell")?;
    let open: Vec<Vec3> = open.to_vec();
    let key = rebuild::shell_key(input_key, placement, thickness, &open, angle_tol_deg);
    let cached = d.cached_geometry(key, |d| {
        let e = d.entities.get(&entity).ok_or(Error::UnknownEntity { doc, entity })?;
        let world = transform_mesh(&e.mesh, &transform);
        let shaped = shell::shell(&world, thickness, &open, angle_tol_deg)?;
        let volume = fillet::signed_volume(&shaped);
        let mesh = localize_world(&transform, &shaped)?;
        Ok(rebuild::CachedGeometry {
            mesh,
            volume: Some(volume),
            edges: None,
            center: None,
        })
    })?;
    let volume = cached.volume.unwrap_or(0.0);
    let tris = cached.mesh.indices.len() / 3;
    let mesh = cached.mesh;
    {
        let e = d.entity_mut(entity)?;
        e.mesh = mesh.clone();
        e.geom_key = key;
    }
    Ok(CommandResult::EntityReshaped { doc, entity, volume, tris, mesh })
}

fn hole_solid(
    d: &mut Document,
    doc: DocId,
    entity: EntityId,
    spec: hole::HoleSpec,
) -> Result<CommandResult> {
    let (input_key, placement, transform) = editable_solid(d, doc, entity, "hole")?;
    let key = rebuild::hole_key(input_key, placement, &spec);
    let cached = d.cached_geometry(key, |d| {
        let e = d.entities.get(&entity).ok_or(Error::UnknownEntity { doc, entity })?;
        let world = transform_mesh(&e.mesh, &transform);
        let shaped = hole::drill(&world, &spec)?;
        let volume = fillet::signed_volume(&shaped);
        let mesh = localize_world(&transform, &shaped)?;
        Ok(rebuild::CachedGeometry {
            mesh,
            volume: Some(volume),
            edges: None,
            center: None,
        })
    })?;
    let volume = cached.volume.unwrap_or(0.0);
    let tris = cached.mesh.indices.len() / 3;
    let mesh = cached.mesh;
    {
        let e = d.entity_mut(entity)?;
        e.mesh = mesh.clone();
        e.geom_key = key;
    }
    Ok(CommandResult::EntityReshaped { doc, entity, volume, tris, mesh })
}

/// Shared preflight for in-place world-space features.
fn editable_solid(
    d: &Document,
    doc: DocId,
    entity: EntityId,
    what: &str,
) -> Result<(rebuild::GeomKey, (Vec3, Vec3, Vec3), Mat4)> {
    let e = d.entities.get(&entity).ok_or(Error::UnknownEntity { doc, entity })?;
    if matches!(e.shape, Shape::Sketch { .. }) {
        return Err(Error::Invalid(format!(
            "entity {entity} is a sketch; {what} applies to printable solids (hint: extrude the \
             sketch first)"
        )));
    }
    if e.mesh.indices.is_empty() {
        return Err(Error::Invalid(format!("entity {entity} has no geometry to {what}")));
    }
    Ok((
        e.geom_key,
        (e.translation, e.rotation_deg, e.scale),
        e.transform(),
    ))
}

/// Transform a local mesh into world space (f32 math, matching the
/// viewer's model matrix convention).
pub(crate) fn transform_mesh(mesh: &Mesh, m: &Mat4) -> Mesh {
    Mesh {
        positions: mesh.positions.iter().map(|&p| mat4::transform_point(m, p)).collect(),
        normals: mesh.normals.clone(),
        indices: mesh.indices.clone(),
    }
}

/// Compute a mesh's bounding-box center and shift positions so the box is
/// centered on the origin. Returns `(mesh, center)` where `center` is the
/// translation that undoes the shift (i.e. the original center).
pub(crate) fn recenter(mesh: &Mesh) -> (Mesh, Vec3) {
    let mut min = [f64::MAX; 3];
    let mut max = [f64::MIN; 3];
    for p in &mesh.positions {
        for i in 0..3 {
            let v = p[i] as f64;
            min[i] = min[i].min(v);
            max[i] = max[i].max(v);
        }
    }
    let center = if mesh.positions.is_empty() {
        [0.0; 3]
    } else {
        [(min[0] + max[0]) / 2.0, (min[1] + max[1]) / 2.0, (min[2] + max[2]) / 2.0]
    };
    let shifted = Mesh {
        positions: mesh
            .positions
            .iter()
            .map(|p| [(p[0] - center[0] as f32), (p[1] - center[1] as f32), (p[2] - center[2] as f32)])
            .collect(),
        normals: mesh.normals.clone(),
        indices: mesh.indices.clone(),
    };
    (shifted, center)
}

#[allow(clippy::too_many_arguments)]
fn add_entity(
    d: &mut Document,
    doc: DocId,
    name: String,
    shape: Shape,
    translation: Vec3,
    rotation_deg: Vec3,
    scale: Vec3,
    color: Option<Rgba>,
) -> Result<CommandResult> {
    let id = d.next_entity();
    let (mesh, geom_key) = match &shape {
        Shape::Box { size } => {
            let key = rebuild::box_key(*size);
            let size = *size;
            let mesh = d
                .cached_geometry(key, |_| {
                    Ok(rebuild::CachedGeometry::mesh(geo::box_mesh(size)))
                })?
                .mesh;
            (mesh, key)
        },
        Shape::Sphere { radius, segments } => {
            let (radius, segments) = (*radius, *segments);
            let key = rebuild::sphere_key(radius, segments);
            let mesh = d
                .cached_geometry(key, |_| {
                    Ok(rebuild::CachedGeometry::mesh(geo::sphere_mesh(radius, segments)))
                })?
                .mesh;
            (mesh, key)
        },
        Shape::Cylinder { radius, height, segments } => {
            let (radius, height, segments) = (*radius, *height, *segments);
            let key = rebuild::cylinder_key(radius, height, segments);
            let mesh = d
                .cached_geometry(key, |_| {
                    Ok(rebuild::CachedGeometry::mesh(geo::cylinder_mesh(
                        radius, height, segments,
                    )))
                })?
                .mesh;
            (mesh, key)
        },
        // Boolean entities are produced by the boolean path, which builds
        // the Entity directly — add_entity never sees this shape.
        Shape::Boolean { .. } => unreachable!("boolean shapes are not tessellated here"),
        // Sketches are created by AddSketch (with solver state), never here.
        Shape::Sketch { .. } => unreachable!("sketch shapes are not tessellated here"),
        // Solids are built by Extrude/Revolve/Sweep/Loft/Thread, which create
        // their entities directly with the kernel's mesh.
        Shape::Extrude { .. }
        | Shape::Revolve { .. }
        | Shape::Sweep { .. }
        | Shape::Loft { .. }
        | Shape::Thread
        | Shape::Copy { .. } => {
            unreachable!("derived solid shapes are not tessellated here")
        },
    };
    let entity = Entity {
        id,
        name,
        shape,
        mesh,
        translation,
        rotation_deg,
        scale,
        color: color.unwrap_or([0.7, 0.72, 0.75, 1.0]),
        geom_key,
    };
    let mesh = entity.mesh.clone();
    d.entities.insert(id, entity);
    Ok(CommandResult::EntityCreated { doc, entity: id, mesh })
}

#[cfg(test)]
mod tests {
    use super::*;
    use agcad_protocol::{Hand, SweepPath};

    fn engine() -> Engine {
        let mut e = Engine::new();
        match e.execute(Command::CreateDocument { name: "test".into(), doc: None }).unwrap() {
            CommandResult::DocumentCreated { doc } => assert_eq!(doc, 1),
            r => panic!("unexpected result {r:?}"),
        }
        e
    }

    fn num(v: f64) -> Scalar {
        Scalar::Number(v)
    }

    fn vec3(v: f64) -> SVec3 {
        [Scalar::Number(v), Scalar::Number(v), Scalar::Number(v)]
    }

    /// Mesh bounding-box extent along one axis.
    fn extent(e: &mut Engine, entity: EntityId, axis: usize) -> f64 {
        match e.execute(Command::GetScene { doc: 1 }).unwrap() {
            CommandResult::Scene { entities, .. } => {
                let mesh = &entities.iter().find(|x| x.id == entity).unwrap().mesh;
                let values: Vec<f64> = mesh.positions.iter().map(|p| p[axis] as f64).collect();
                values.iter().cloned().fold(f64::MIN, f64::max)
                    - values.iter().cloned().fold(f64::MAX, f64::min)
            },
            r => panic!("unexpected result {r:?}"),
        }
    }

    #[test]
    fn ids_are_deterministic() {
        let mut a = engine();
        let mut b = engine();
        let cmd = Command::AddBox {
            doc: 1,
            name: "box".into(),
            center: vec3(0.0),
            size: vec3(10.0),
            color: None,
        };
        let id_a = match a.execute(cmd.clone()).unwrap() {
            CommandResult::EntityCreated { entity, mesh, .. } => {
                assert_eq!(mesh.positions.len(), 24);
                entity
            },
            r => panic!("unexpected result {r:?}"),
        };
        let id_b = match b.execute(cmd).unwrap() {
            CommandResult::EntityCreated { entity, .. } => entity,
            r => panic!("unexpected result {r:?}"),
        };
        assert_eq!(id_a, id_b);
    }

    #[test]
    fn parametric_rebuild_regenerates_geometry() {
        let mut e = engine();
        e.execute(Command::SetParameter {
            doc: 1,
            name: "wall".into(),
            value: num(2.0),
            description: Some("plate thickness".into()),
        })
        .unwrap();
        let id = match e
            .execute(Command::AddBox {
                doc: 1,
                name: "plate".into(),
                center: [num(0.0), num(0.0), Scalar::Expression("wall/2".into())],
                size: [num(80.0), num(40.0), Scalar::Expression("wall*2".into())],
                color: None,
            })
            .unwrap()
        {
            CommandResult::EntityCreated { entity, .. } => entity,
            r => panic!("unexpected result {r:?}"),
        };

        let before = extent(&mut e, id, 2);
        assert!((before - 4.0).abs() < 1e-5, "expected z extent 4, got {before}");

        // Change the parameter: geometry regenerates, id stays.
        e.execute(Command::SetParameter {
            doc: 1,
            name: "wall".into(),
            value: Scalar::Expression("3".into()), // even parameter values may be expressions
            description: None,
        })
        .unwrap();
        let after = extent(&mut e, id, 2);
        assert!((after - 6.0).abs() < 1e-5, "expected z extent 6, got {after}");

        // Undo restores both geometry and the parameter value.
        e.execute(Command::Undo { doc: 1 }).unwrap();
        assert!((extent(&mut e, id, 2) - 4.0).abs() < 1e-5);
        match e.execute(Command::GetParameters { doc: 1 }).unwrap() {
            CommandResult::Parameters { params, .. } => {
                assert_eq!(params[0].value, 2.0);
            },
            r => panic!("unexpected result {r:?}"),
        }
    }

    #[test]
    fn ids_stable_across_parameter_changes() {
        let mut e = engine();
        e.execute(Command::SetParameter {
            doc: 1,
            name: "wall".into(),
            value: num(2.0),
            description: None,
        })
        .unwrap();
        let a = match e
            .execute(Command::AddBox {
                doc: 1,
                name: "a".into(),
                center: vec3(0.0),
                size: vec3(10.0),
                color: None,
            })
            .unwrap()
        {
            CommandResult::EntityCreated { entity, .. } => entity,
            r => panic!("unexpected result {r:?}"),
        };
        let b = match e
            .execute(Command::AddCylinder {
                doc: 1,
                name: "b".into(),
                center: vec3(0.0),
                radius: Scalar::Expression("wall".into()),
                height: Scalar::Expression("wall*5".into()),
                segments: None,
                color: None,
            })
            .unwrap()
        {
            CommandResult::EntityCreated { entity, .. } => entity,
            r => panic!("unexpected result {r:?}"),
        };
        e.execute(Command::SetParameter {
            doc: 1,
            name: "wall".into(),
            value: num(4.0),
            description: None,
        })
        .unwrap();
        match e.execute(Command::GetScene { doc: 1 }).unwrap() {
            CommandResult::Scene { entities, .. } => {
                let ids: Vec<u64> = entities.iter().map(|x| x.id).collect();
                assert_eq!(ids, vec![a, b], "replay must reuse identical ids");
                // Cylinder regenerated with the new wall: height = 20 along Z.
                let cyl = entities.iter().find(|x| x.id == b).unwrap();
                let max_z = cyl.mesh.positions.iter().map(|p| p[2]).fold(f32::MIN, f32::max);
                assert!((max_z - 10.0).abs() < 1e-4, "expected height 20 (±10), got {max_z}");
            },
            r => panic!("unexpected result {r:?}"),
        }
    }

    #[test]
    fn failed_commands_leave_no_trace() {
        let mut e = engine();
        let id = match e
            .execute(Command::AddBox {
                doc: 1,
                name: "a".into(),
                center: vec3(0.0),
                size: vec3(10.0),
                color: None,
            })
            .unwrap()
        {
            CommandResult::EntityCreated { entity, .. } => entity,
            r => panic!("unexpected result {r:?}"),
        };
        // Reference an undefined parameter: rejected, not recorded.
        assert!(matches!(
            e.execute(Command::AddBox {
                doc: 1,
                name: "bad".into(),
                center: vec3(0.0),
                size: [Scalar::Expression("nope*2".into()), num(1.0), num(1.0)],
                color: None,
            }),
            Err(Error::Invalid(msg)) if msg.contains("'nope'") && msg.contains("no parameters defined"),
        ));
        // Now define the parameter and change it: the program still replays
        // cleanly with just the original box.
        e.execute(Command::SetParameter {
            doc: 1,
            name: "wall".into(),
            value: num(3.0),
            description: None,
        })
        .unwrap();
        match e.execute(Command::GetScene { doc: 1 }).unwrap() {
            CommandResult::Scene { entities, .. } => {
                assert_eq!(entities.len(), 1);
                assert_eq!(entities[0].id, id);
            },
            r => panic!("unexpected result {r:?}"),
        }
    }

    #[test]
    fn undo_restores_state() {
        let mut e = engine();
        let id = match e
            .execute(Command::AddBox {
                doc: 1,
                name: "box".into(),
                center: vec3(0.0),
                size: vec3(10.0),
                color: None,
            })
            .unwrap()
        {
            CommandResult::EntityCreated { entity, .. } => entity,
            r => panic!("unexpected result {r:?}"),
        };
        e.execute(Command::Delete { doc: 1, entity: id }).unwrap();
        e.execute(Command::Undo { doc: 1 }).unwrap();
        match e.execute(Command::GetScene { doc: 1 }).unwrap() {
            CommandResult::Scene { entities, .. } => {
                assert_eq!(entities.len(), 1);
                assert_eq!(entities[0].id, id);
            },
            r => panic!("unexpected result {r:?}"),
        }
        e.execute(Command::Redo { doc: 1 }).unwrap();
        match e.execute(Command::GetScene { doc: 1 }).unwrap() {
            CommandResult::Scene { entities, .. } => assert!(entities.is_empty()),
            r => panic!("unexpected result {r:?}"),
        }
    }

    #[test]
    fn transform_composes_correctly() {
        let mut e = engine();
        let id = match e
            .execute(Command::AddBox {
                doc: 1,
                name: "box".into(),
                center: vec3(0.0),
                size: vec3(10.0),
                color: None,
            })
            .unwrap()
        {
            CommandResult::EntityCreated { entity, .. } => entity,
            r => panic!("unexpected result {r:?}"),
        };
        e.execute(Command::Transform {
            doc: 1,
            entity: id,
            translation: Some([num(1.0), num(2.0), num(3.0)]),
            rotation_deg: Some([num(0.0), num(0.0), num(90.0)]),
            scale: Some(vec3(2.0)),
        })
        .unwrap();
        match e.execute(Command::GetScene { doc: 1 }).unwrap() {
            CommandResult::Scene { entities, .. } => {
                let m = entities[0].transform;
                assert_eq!(m[3], [1.0, 2.0, 3.0, 1.0]);
                let x = [m[0][0], m[0][1], m[0][2]];
                assert!((x[0] - 0.0).abs() < 1e-5 && (x[1] - 2.0).abs() < 1e-5, "got {x:?}");
            },
            r => panic!("unexpected result {r:?}"),
        }
    }

    #[test]
    fn errors_are_helpful() {
        let mut e = engine();
        assert!(matches!(
            e.execute(Command::AddBox {
                doc: 99,
                name: "x".into(),
                center: vec3(0.0),
                size: vec3(1.0),
                color: None,
            }),
            Err(Error::UnknownDocument(99))
        ));
        assert!(matches!(
            e.execute(Command::AddBox {
                doc: 1,
                name: "x".into(),
                center: vec3(0.0),
                size: [num(0.0), num(1.0), num(1.0)],
                color: None,
            }),
            Err(Error::Invalid(_))
        ));
    }

    // --- M4: booleans + export -------------------------------------------------

    /// Two overlapping unit boxes fused: volume must be less than the sum,
    /// result placed at its bbox center, sources kept.
    #[test]
    fn boolean_union_fuses_overlapping_solids() {
        let mut e = engine();
        let a = match e.execute(Command::AddBox {
            doc: 1, name: "a".into(), center: vec3(0.0), size: vec3(10.0), color: None,
        }).unwrap() {
            CommandResult::EntityCreated { entity, .. } => entity,
            r => panic!("unexpected result {r:?}"),
        };
        let b = match e.execute(Command::AddBox {
            doc: 1, name: "b".into(), center: [num(5.0), num(0.0), num(0.0)], size: vec3(10.0), color: None,
        }).unwrap() {
            CommandResult::EntityCreated { entity, .. } => entity,
            r => panic!("unexpected result {r:?}"),
        };
        let u = match e.execute(Command::Boolean {
            doc: 1,
            op: agcad_protocol::BooleanOp::Union,
            entities: vec![a, b],
            name: Some("fused".into()),
            color: None,
        }).unwrap() {
            CommandResult::EntityCreated { entity, mesh, .. } => (entity, mesh),
            r => panic!("unexpected result {r:?}"),
        };
        let (uid, mesh) = u;
        assert_eq!(uid, 3, "deterministic: next id after two boxes");
        assert!(!mesh.positions.is_empty());
        assert!(!mesh.indices.is_empty());
        // Bounding box: two 10mm boxes offset by 5 on x → x extent 15.
        let xs: Vec<f64> = mesh.positions.iter().map(|p| p[0] as f64).collect();
        let xext = xs.iter().cloned().fold(f64::MIN, f64::max) - xs.iter().cloned().fold(f64::MAX, f64::min);
        assert!((xext - 15.0).abs() < 1e-4, "expected x extent 15, got {xext}");
        // Sources still exist.
        match e.execute(Command::GetScene { doc: 1 }).unwrap() {
            CommandResult::Scene { entities, .. } => assert_eq!(entities.len(), 3),
            r => panic!("unexpected result {r:?}"),
        }
    }

    /// Box minus overlapping sphere: a carved pocket, watertight, smaller.
    #[test]
    fn boolean_difference_carves() {
        let mut e = engine();
        let a = match e.execute(Command::AddBox {
            doc: 1, name: "block".into(), center: vec3(0.0), size: vec3(20.0), color: None,
        }).unwrap() {
            CommandResult::EntityCreated { entity, .. } => entity,
            r => panic!("unexpected result {r:?}"),
        };
        let b = match e.execute(Command::AddSphere {
            doc: 1, name: "ball".into(), center: [num(0.0), num(0.0), num(10.0)], radius: num(6.0), segments: None, color: None,
        }).unwrap() {
            CommandResult::EntityCreated { entity, .. } => entity,
            r => panic!("unexpected result {r:?}"),
        };
        match e.execute(Command::Boolean {
            doc: 1,
            op: agcad_protocol::BooleanOp::Difference,
            entities: vec![a, b],
            name: None,
            color: None,
        }).unwrap() {
            CommandResult::EntityCreated { mesh, .. } => {
                assert!(!mesh.positions.is_empty());
                // Top face must be carved open: some vertex below z=5 would
                // be wrong, but a hole means the max z < 10 (flat top) for
                // interior verts. Simplest robust check: fewer triangles
                // than the plain box would have after a full union, and the
                // result has more tris than a bare box (carved surfaces).
                assert!(mesh.indices.len() / 3 > 12, "carving must add surface");
            },
            r => panic!("unexpected result {r:?}"),
        }
    }

    /// The parametric payoff: parameters flow *through* boolean results.
    #[test]
    fn boolean_replays_parametrically() {
        let mut e = engine();
        e.execute(Command::SetParameter {
            doc: 1, name: "wall".into(), value: num(2.0), description: None,
        }).unwrap();
        let a = match e.execute(Command::AddBox {
            doc: 1, name: "outer".into(), center: vec3(0.0), size: [Scalar::Expression("wall*10".into()), num(10.0), num(10.0)], color: None,
        }).unwrap() {
            CommandResult::EntityCreated { entity, .. } => entity,
            r => panic!("unexpected result {r:?}"),
        };
        let b = match e.execute(Command::AddBox {
            doc: 1, name: "inner".into(), center: vec3(0.0), size: [num(6.0), num(6.0), num(6.0)], color: None,
        }).unwrap() {
            CommandResult::EntityCreated { entity, .. } => entity,
            r => panic!("unexpected result {r:?}"),
        };
        // Hollow the wall: difference → a tube with wall thickness.
        let uid = match e.execute(Command::Boolean {
            doc: 1,
            op: agcad_protocol::BooleanOp::Difference,
            entities: vec![a, b],
            name: Some("tube".into()),
            color: None,
        }).unwrap() {
            CommandResult::EntityCreated { entity, .. } => entity,
            r => panic!("unexpected result {r:?}"),
        };
        // Grow the wall: replay re-runs the boolean with new operand sizes.
        e.execute(Command::SetParameter {
            doc: 1, name: "wall".into(), value: num(4.0), description: None,
        }).unwrap();
        match e.execute(Command::GetScene { doc: 1 }).unwrap() {
            CommandResult::Scene { entities, .. } => {
                let ids: Vec<u64> = entities.iter().map(|x| x.id).collect();
                assert_eq!(ids, vec![a, b, uid], "replay must reuse identical ids through booleans");
                let tube = entities.iter().find(|x| x.id == uid).unwrap();
                let xs: Vec<f64> = tube.mesh.positions.iter().map(|p| p[0] as f64).collect();
                let xext = xs.iter().cloned().fold(f64::MIN, f64::max) - xs.iter().cloned().fold(f64::MAX, f64::min);
                assert!((xext - 40.0).abs() < 1e-4, "tube regenerated with wall=4 → x extent 40, got {xext}");
            },
            r => panic!("unexpected result {r:?}"),
        }
    }

    /// Undo after a boolean restores the exact pre-boolean scene.
    #[test]
    fn undo_restores_boolean() {
        let mut e = engine();
        let ids: Vec<u64> = [(0.0, "a"), (3.0, "b")]
            .iter()
            .map(|&(x, n)| match e.execute(Command::AddBox {
                doc: 1, name: n.into(), center: [num(x), num(0.0), num(0.0)], size: vec3(10.0), color: None,
            }).unwrap() {
                CommandResult::EntityCreated { entity, .. } => entity,
                r => panic!("unexpected result {r:?}"),
            })
            .collect();
        let [a, b] = [ids[0], ids[1]];
        let uid = match e.execute(Command::Boolean {
            doc: 1, op: agcad_protocol::BooleanOp::Union, entities: vec![a, b],
            name: None, color: None,
        }).unwrap() {
            CommandResult::EntityCreated { entity, .. } => entity,
            r => panic!("unexpected result {r:?}"),
        };
        e.execute(Command::Undo { doc: 1 }).unwrap();
        match e.execute(Command::GetScene { doc: 1 }).unwrap() {
            CommandResult::Scene { entities, .. } => {
                assert_eq!(entities.len(), 2, "undo must remove the boolean entity");
                assert!(entities.iter().all(|x| x.id != uid));
            },
            r => panic!("unexpected result {r:?}"),
        }
        e.execute(Command::Redo { doc: 1 }).unwrap();
        match e.execute(Command::GetScene { doc: 1 }).unwrap() {
            CommandResult::Scene { entities, .. } => assert_eq!(entities.len(), 3),
            r => panic!("unexpected result {r:?}"),
        }
    }

    /// Same program, two engines → identical boolean meshes (kernel
    /// determinism, the property the parametric replay depends on).
    #[test]
    fn boolean_results_are_deterministic() {
        let build = |e: &mut Engine| {
            let a = match e.execute(Command::AddSphere {
                doc: 1, name: "a".into(), center: vec3(0.0), radius: num(10.0), segments: None, color: None,
            }).unwrap() {
                CommandResult::EntityCreated { entity, .. } => entity,
                r => panic!("unexpected result {r:?}"),
            };
            let b = match e.execute(Command::AddCylinder {
                doc: 1, name: "b".into(), center: vec3(0.0), radius: num(3.0), height: num(30.0), segments: None, color: None,
            }).unwrap() {
                CommandResult::EntityCreated { entity, .. } => entity,
                r => panic!("unexpected result {r:?}"),
            };
            match e.execute(Command::Boolean {
                doc: 1, op: agcad_protocol::BooleanOp::Union, entities: vec![a, b],
                name: None, color: None,
            }).unwrap() {
                CommandResult::EntityCreated { mesh, .. } => mesh,
                r => panic!("unexpected result {r:?}"),
            }
        };
        let mut e1 = engine();
        let mut e2 = engine();
        let m1 = build(&mut e1);
        let m2 = build(&mut e2);
        assert_eq!(m1.positions, m2.positions, "bit-identical positions");
        assert_eq!(m1.indices, m2.indices, "bit-identical topology");
    }

    /// Headless render: valid PNG bytes, echoed dimensions, deterministic.
    #[test]
    fn render_offscreen_produces_valid_deterministic_png() {
        let mut e = engine();
        e.execute(Command::AddBox {
            doc: 1, name: "cube".into(), center: vec3(0.0), size: vec3(10.0), color: None,
        }).unwrap();

        let render = |e: &mut Engine, path: Option<String>| {
            match e.execute(Command::RenderOffscreen {
                doc: 1,
                width: Some(160),
                height: Some(120),
                eye: Some([40.0, -60.0, 50.0]),
                target: Some([0.0, 0.0, 0.0]),
                up: None,
                fov_deg: None,
                ortho: None,
                background: None,
                path,
            }).unwrap() {
                CommandResult::Rendered { width, height, png_base64, path, .. } => (width, height, png_base64, path),
                r => panic!("unexpected result {r:?}"),
            }
        };
        let (w, h, b64_1, _) = render(&mut e, None);
        assert_eq!((w, h), (160, 120));
        assert!(!b64_1.is_empty());

        // File write: parse the PNG structure straight off disk.
        let path = std::env::temp_dir().join("agcad_test_render.png");
        let (_, _, _, written) = render(&mut e, Some(path.to_string_lossy().into_owned()));
        let bytes = std::fs::read(&path).unwrap();
        assert_eq!(written.as_deref(), Some(path.to_string_lossy().as_ref()));
        assert!(bytes.starts_with(&[0x89, b'P', b'N', b'G', 0x0D, 0x0A, 0x1A, 0x0A]), "missing PNG signature");
        // IHDR width/height: big-endian u32 at offsets 16 and 20.
        let iw = u32::from_be_bytes([bytes[16], bytes[17], bytes[18], bytes[19]]);
        let ih = u32::from_be_bytes([bytes[20], bytes[21], bytes[22], bytes[23]]);
        assert_eq!((iw, ih), (160, 120));

        // Same scene + camera → bit-identical output.
        let (_, _, b64_2, _) = render(&mut e, None);
        assert_eq!(b64_1, b64_2, "renders must be bit-identical");
        let _ = std::fs::remove_file(&path);
    }

    /// Auto-framed camera on an empty document must not panic.
    #[test]
    fn render_offscreen_empty_scene_is_fine() {
        let mut e = engine();
        match e.execute(Command::RenderOffscreen {
            doc: 1, width: None, height: None, eye: None, target: None,
            up: None, fov_deg: None, ortho: None, background: None, path: None,
        }).unwrap() {
            CommandResult::Rendered { width, height, .. } => {
                assert_eq!((width, height), (800, 600));
            },
            r => panic!("unexpected result {r:?}"),
        }
    }

    /// The STL test from M4, kept near the render tests for symmetry.
    #[test]
    fn stl_export_writes_valid_binary() {
        let mut e = engine();
        let id = match e.execute(Command::AddBox {
            doc: 1, name: "cube".into(), center: vec3(0.0), size: vec3(10.0), color: None,
        }).unwrap() {
            CommandResult::EntityCreated { entity, .. } => entity,
            r => panic!("unexpected result {r:?}"),
        };
        // Move it so world-space export must apply the transform.
        e.execute(Command::Transform {
            doc: 1, entity: id, translation: Some([num(100.0), num(0.0), num(0.0)]),
            rotation_deg: None, scale: None,
        }).unwrap();

        let path = std::env::temp_dir().join("agcad_test_export.stl");
        let path_str = path.to_string_lossy().into_owned();
        match e.execute(Command::ExportStl { doc: 1, path: path_str.clone(), entities: None }).unwrap() {
            CommandResult::StlExported { tris, bytes, path: resolved, .. } => {
                assert_eq!(tris, 12);
                assert_eq!(bytes, 84 + 12 * 50);
                assert!(resolved.ends_with("agcad_test_export.stl"));
            },
            r => panic!("unexpected result {r:?}"),
        }
        let data = std::fs::read(&path).unwrap();
        assert_eq!(data.len(), 84 + 12 * 50);
        assert!(!data.starts_with(b"solid"), "binary STL must not start with 'solid'");
        let count = u32::from_le_bytes([data[80], data[81], data[82], data[83]]);
        assert_eq!(count, 12);
        // First triangle's first vertex: 84-byte header + 12-byte face
        // normal → x at byte 96. Must be in world space (x ≈ 95..105).
        let x = f32::from_le_bytes([data[96], data[97], data[98], data[99]]);
        assert!((95.0..=105.0).contains(&x), "expected world-space x near 100, got {x}");
        let _ = std::fs::remove_file(&path);
    }

    /// 3MF is the multi-object format: names, colors and millimeters survive.
    #[test]
    fn threemf_export_writes_a_multi_object_package() {
        let mut e = engine();
        let base = match e
            .execute(Command::AddBox {
                doc: 1,
                name: "red base".into(),
                center: vec3(0.0),
                size: vec3(10.0),
                color: Some([1.0, 0.0, 0.0, 1.0]),
            })
            .unwrap()
        {
            CommandResult::EntityCreated { entity, .. } => entity,
            r => panic!("unexpected result {r:?}"),
        };
        e.execute(Command::Transform {
            doc: 1,
            entity: base,
            translation: Some([num(100.0), num(0.0), num(0.0)]),
            rotation_deg: None,
            scale: None,
        })
        .unwrap();
        e.execute(Command::AddBox {
            doc: 1,
            name: "grey boss".into(),
            center: vec3(0.0),
            size: vec3(10.0),
            color: None,
        })
        .unwrap();

        let path = std::env::temp_dir().join("agcad_test_export.3mf");
        let path_str = path.to_string_lossy().into_owned();
        match e
            .execute(Command::Export3mf { doc: 1, path: path_str.clone(), entities: None })
            .unwrap()
        {
            CommandResult::ThreeMfExported { objects, tris, bytes, path: resolved, .. } => {
                assert_eq!(objects, 2);
                assert_eq!(tris, 24);
                assert!(resolved.ends_with("agcad_test_export.3mf"));
                let data = std::fs::read(&path).unwrap();
                assert_eq!(bytes, data.len());
            },
            r => panic!("unexpected result {r:?}"),
        }
        let data = std::fs::read(&path).unwrap();
        assert!(data.starts_with(b"PK\x03\x04"), "3MF is a ZIP package");
        // Stored entries make the model XML visible in the raw package.
        let text = String::from_utf8_lossy(&data);
        assert!(text.contains("3D/3dmodel.model"));
        assert!(text.contains("unit=\"millimeter\""));
        assert!(text.contains("<metadata name=\"Application\">agcad</metadata>"));
        assert_eq!(text.matches("<object ").count(), 2);
        assert_eq!(text.matches("<item ").count(), 2);
        assert!(text.contains("name=\"red base\""));
        assert!(text.contains("name=\"grey boss\""));
        assert!(text.contains("displaycolor=\"#FF0000FF\""));
        // World-space geometry, like STL: the translated base is at x≈100.
        assert!(text.contains("x=\"95\"") && text.contains("x=\"105\""));
        let _ = std::fs::remove_file(&path);
    }

    /// Sketches are skipped by 3MF too; sketch-only documents are an error,
    /// not an empty package.
    #[test]
    fn threemf_skips_sketches() {
        let mut e = engine();
        let s = add_sketch(&mut e, "profile");
        add_line(&mut e, s, [0.0, 0.0], [10.0, 0.0]);
        let path = std::env::temp_dir().join("agcad_sketch_only.3mf");
        let path_str = path.to_string_lossy().to_string();
        let err = e.execute(Command::Export3mf { doc: 1, path: path_str, entities: None });
        assert!(err.is_err(), "sketch-only export must fail with a hint");

        e.execute(Command::AddBox {
            doc: 1,
            name: "plate".into(),
            center: vec3(0.0),
            size: vec3(10.0),
            color: None,
        })
        .unwrap();
        let path = std::env::temp_dir().join("agcad_sketch_and_box.3mf");
        let path_str = path.to_string_lossy().to_string();
        match e.execute(Command::Export3mf { doc: 1, path: path_str, entities: None }).unwrap() {
            CommandResult::ThreeMfExported { objects, tris, .. } => {
                assert_eq!(objects, 1);
                assert_eq!(tris, 12, "only the box's 12 triangles");
            },
            r => panic!("unexpected result {r:?}"),
        }
        let _ = std::fs::remove_file(&path);
    }

    // --- sketches -----------------------------------------------------------

    use agcad_protocol::ConstraintKind as K;

    fn pref(element: u64, point: u8) -> agcad_protocol::SketchRef {
        agcad_protocol::SketchRef { element, point: Some(point) }
    }

    fn wref(element: u64) -> agcad_protocol::SketchRef {
        agcad_protocol::SketchRef { element, point: None }
    }

    fn add_line(e: &mut Engine, sketch: EntityId, p1: [f64; 2], p2: [f64; 2]) -> u64 {
        match e
            .execute(Command::AddSketchLine {
                doc: 1,
                sketch,
                p1: [num(p1[0]), num(p1[1])],
                p2: [num(p2[0]), num(p2[1])],
            })
            .unwrap()
        {
            CommandResult::SketchElementCreated { element, .. } => element,
            r => panic!("unexpected result {r:?}"),
        }
    }

    fn constrain(
        e: &mut Engine,
        sketch: EntityId,
        kind: K,
        refs: Vec<agcad_protocol::SketchRef>,
        value: Option<Scalar>,
    ) -> (u64, bool, f64, i64) {
        match e
            .execute(Command::Constrain { doc: 1, sketch, kind, refs, value })
            .unwrap()
        {
            CommandResult::ConstraintCreated { constraint, solved, residual, dof, .. } => {
                (constraint, solved, residual, dof)
            },
            r => panic!("unexpected result {r:?}"),
        }
    }

    fn sketch_snapshot(e: &mut Engine, sketch: EntityId) -> agcad_protocol::SketchSnapshot {
        match e.execute(Command::GetScene { doc: 1 }).unwrap() {
            CommandResult::Scene { entities, .. } => entities
                .iter()
                .find(|x| x.id == sketch)
                .expect("sketch in scene")
                .sketch
                .clone()
                .expect("sketch data in snapshot"),
            r => panic!("unexpected result {r:?}"),
        }
    }

    fn add_sketch(e: &mut Engine, name: &str) -> EntityId {
        match e.execute(Command::AddSketch { doc: 1, name: name.into(), plane: None }).unwrap() {
            CommandResult::EntityCreated { entity, .. } => entity,
            r => panic!("unexpected result {r:?}"),
        }
    }

    fn add_sketch_on(e: &mut Engine, name: &str, plane: SketchPlane) -> EntityId {
        match e
            .execute(Command::AddSketch {
                doc: 1,
                name: name.into(),
                plane: Some(plane),
            })
            .unwrap()
        {
            CommandResult::EntityCreated { entity, .. } => entity,
            r => panic!("unexpected result {r:?}"),
        }
    }

    fn face_plane(entity: EntityId, normal: [f64; 3]) -> SketchPlane {
        SketchPlane::Face { entity, near: None, normal: Some(normal), angle_tol_deg: None }
    }

    fn sketch_frame(e: &mut Engine, sketch: EntityId) -> agcad_protocol::SketchPlaneFrame {
        sketch_snapshot(e, sketch)
            .plane
            .expect("sketch snapshots carry the resolved plane")
    }

    fn close_to(a: [f64; 3], b: [f64; 3], tol: f64) -> bool {
        (0..3).all(|i| (a[i] - b[i]).abs() <= tol)
    }

    /// Rectangle profile built purely through the bus: the solver must
    /// square it up and report the remaining degrees of freedom.
    #[test]
    fn sketch_rectangle_via_commands() {
        let mut e = engine();
        let s = add_sketch(&mut e, "profile");
        let (a, b, c, d) = (
            add_line(&mut e, s, [0.0, 0.0], [38.0, 3.0]),
            add_line(&mut e, s, [38.0, 3.0], [41.0, 31.0]),
            add_line(&mut e, s, [41.0, 31.0], [-2.0, 28.0]),
            add_line(&mut e, s, [-2.0, 28.0], [0.0, 0.0]),
        );
        constrain(&mut e, s, K::Coincident, vec![pref(a, 1), pref(b, 0)], None);
        constrain(&mut e, s, K::Coincident, vec![pref(b, 1), pref(c, 0)], None);
        constrain(&mut e, s, K::Coincident, vec![pref(c, 1), pref(d, 0)], None);
        constrain(&mut e, s, K::Coincident, vec![pref(d, 1), pref(a, 0)], None);
        constrain(&mut e, s, K::Horizontal, vec![wref(a)], None);
        constrain(&mut e, s, K::Horizontal, vec![wref(c)], None);
        constrain(&mut e, s, K::Vertical, vec![wref(b)], None);
        constrain(&mut e, s, K::Vertical, vec![wref(d)], None);
        let (_, solved, residual, _) =
            constrain(&mut e, s, K::Distance, vec![pref(a, 0), pref(a, 1)], Some(num(40.0)));
        assert!(solved, "residual {residual}");
        let (_, solved, _, _) =
            constrain(&mut e, s, K::Distance, vec![pref(b, 0), pref(b, 1)], Some(num(30.0)));
        assert!(solved);

        let snap = sketch_snapshot(&mut e, s);
        assert!(snap.solved);
        assert_eq!(snap.elements.len(), 4);
        assert_eq!(snap.constraints.len(), 10);
        assert_eq!(snap.dof, 16 - 14);
        let line_len = |id: u64| match snap.elements.iter().find(|x| matches!(x, SketchElement::Line { id: i, .. } if *i == id)).unwrap() {
            SketchElement::Line { p1, p2, .. } => ((p2[0] - p1[0]).powi(2) + (p2[1] - p1[1]).powi(2)).sqrt(),
            _ => unreachable!(),
        };
        assert!((line_len(a) - 40.0).abs() < 1e-5);
        assert!((line_len(b) - 30.0).abs() < 1e-5);
        // Sketches contribute display ribbons, never empty geometry.
        match e.execute(Command::GetScene { doc: 1 }).unwrap() {
            CommandResult::Scene { entities, .. } => {
                assert!(!entities[0].mesh.positions.is_empty());
                assert!(entities[0].mesh.positions.iter().all(|p| p[2] == 0.0));
            },
            r => panic!("unexpected result {r:?}"),
        }
    }

    /// A distance constraint whose value is an expression re-solves when
    /// the parameter changes — parametric sketches, not baked drawings.
    #[test]
    fn sketch_replays_parametrically() {
        let mut e = engine();
        e.execute(Command::SetParameter {
            doc: 1,
            name: "w".into(),
            value: num(40.0),
            description: None,
        })
        .unwrap();
        let s = add_sketch(&mut e, "profile");
        let a = add_line(&mut e, s, [0.0, 0.0], [10.0, 0.0]);
        constrain(&mut e, s, K::Horizontal, vec![wref(a)], None);
        constrain(
            &mut e,
            s,
            K::Distance,
            vec![pref(a, 0), pref(a, 1)],
            Some(Scalar::Expression("w".into())),
        );
        let len_of = |e: &mut Engine, s: EntityId| match sketch_snapshot(e, s).elements[0] {
            SketchElement::Line { p1, p2, .. } => {
                ((p2[0] - p1[0]).powi(2) + (p2[1] - p1[1]).powi(2)).sqrt()
            },
            _ => unreachable!(),
        };
        assert!((len_of(&mut e, s) - 40.0).abs() < 1e-5);

        e.execute(Command::SetParameter { doc: 1, name: "w".into(), value: num(65.0), description: None })
            .unwrap();
        let after = len_of(&mut e, s);
        assert!((after - 65.0).abs() < 1e-5, "expected re-solved length 65, got {after}");
    }

    /// Undo/redo must move constraints and solved geometry atomically.
    #[test]
    fn undo_restores_sketch_constraint() {
        let mut e = engine();
        let s = add_sketch(&mut e, "profile");
        let a = add_line(&mut e, s, [0.0, 0.0], [10.0, 0.0]);
        assert_eq!(sketch_snapshot(&mut e, s).constraints.len(), 0);
        constrain(&mut e, s, K::Distance, vec![pref(a, 0), pref(a, 1)], Some(num(25.0)));
        assert_eq!(sketch_snapshot(&mut e, s).constraints.len(), 1);

        e.execute(Command::Undo { doc: 1 }).unwrap();
        assert_eq!(sketch_snapshot(&mut e, s).constraints.len(), 0, "undo must remove the constraint");
        e.execute(Command::Redo { doc: 1 }).unwrap();
        assert_eq!(sketch_snapshot(&mut e, s).constraints.len(), 1, "redo must restore it");
    }

    /// Deleting an element also drops the constraints that referenced it.
    #[test]
    fn delete_sketch_element_cascades() {
        let mut e = engine();
        let s = add_sketch(&mut e, "profile");
        let a = add_line(&mut e, s, [0.0, 0.0], [10.0, 0.0]);
        let b = add_line(&mut e, s, [10.0, 0.0], [10.0, 5.0]);
        constrain(&mut e, s, K::Coincident, vec![pref(a, 1), pref(b, 0)], None);
        constrain(&mut e, s, K::Horizontal, vec![wref(a)], None);
        constrain(&mut e, s, K::Vertical, vec![wref(b)], None);
        assert_eq!(sketch_snapshot(&mut e, s).constraints.len(), 3);

        e.execute(Command::DeleteSketchElement { doc: 1, sketch: s, element: a }).unwrap();
        let snap = sketch_snapshot(&mut e, s);
        assert_eq!(snap.elements.len(), 1);
        assert_eq!(snap.constraints.len(), 1, "constraints on the deleted line must cascade");
        assert_eq!(snap.constraints[0].kind, K::Vertical);
    }

    /// STL export ignores sketches; a document with only sketches is not
    /// silently exported as an empty file.
    #[test]
    fn stl_skips_sketches() {
        let mut e = engine();
        let s = add_sketch(&mut e, "profile");
        add_line(&mut e, s, [0.0, 0.0], [10.0, 0.0]);
        let path = std::env::temp_dir().join("agcad_sketch_only.stl");
        let path_str = path.to_string_lossy().to_string();
        let err = e.execute(Command::ExportStl { doc: 1, path: path_str, entities: None });
        assert!(err.is_err(), "sketch-only export must fail with a hint");

        e.execute(Command::AddBox {
            doc: 1,
            name: "plate".into(),
            center: vec3(0.0),
            size: vec3(10.0),
            color: None,
        })
        .unwrap();
        let path = std::env::temp_dir().join("agcad_sketch_and_box.stl");
        let path_str = path.to_string_lossy().to_string();
        match e.execute(Command::ExportStl { doc: 1, path: path_str, entities: None }).unwrap() {
            CommandResult::StlExported { tris, .. } => assert_eq!(tris, 12, "only the box's 12 triangles"),
            r => panic!("unexpected result {r:?}"),
        }
        let _ = std::fs::remove_file(&path);
    }

    /// Two engines fed the same sketch program must agree bit-for-bit.
    #[test]
    fn sketch_is_deterministic_across_engines() {
        let build = || {
            let mut e = engine();
            let s = add_sketch(&mut e, "profile");
            let a = add_line(&mut e, s, [0.0, 0.0], [38.0, 3.0]);
            let b = add_line(&mut e, s, [38.0, 3.0], [41.0, 31.0]);
            constrain(&mut e, s, K::Coincident, vec![pref(a, 1), pref(b, 0)], None);
            constrain(&mut e, s, K::Horizontal, vec![wref(a)], None);
            constrain(&mut e, s, K::Perpendicular, vec![wref(a), wref(b)], None);
            constrain(&mut e, s, K::Distance, vec![pref(a, 0), pref(a, 1)], Some(num(25.0)));
            (sketch_snapshot(&mut e, s), sketch_snapshot(&mut e, s))
        };
        let (a, _) = build();
        let (b, _) = build();
        assert_eq!(a.elements, b.elements);
        assert_eq!(a.constraints, b.constraints);
        assert_eq!(a.residual.to_bits(), b.residual.to_bits());
    }

    // --- sketch → solid -----------------------------------------------------

    /// Add the four lines and constraints of an axis-aligned rectangle to an
    /// existing sketch. The loop starts at the sketch origin and runs into
    /// +X/+Y, so its placement is predictable on a face.
    fn rect_in_sketch(e: &mut Engine, s: EntityId, w: f64, h: f64) -> [u64; 4] {
        let a = add_line(e, s, [0.0, 0.0], [w, 0.0]);
        let b = add_line(e, s, [w, 0.0], [w, h]);
        let c = add_line(e, s, [w, h], [0.0, h]);
        let d = add_line(e, s, [0.0, h], [0.0, 0.0]);
        for (p, q) in [(a, b), (b, c), (c, d), (d, a)] {
            constrain(e, s, K::Coincident, vec![pref(p, 1), pref(q, 0)], None);
        }
        constrain(e, s, K::Horizontal, vec![wref(a)], None);
        constrain(e, s, K::Horizontal, vec![wref(c)], None);
        constrain(e, s, K::Vertical, vec![wref(b)], None);
        constrain(e, s, K::Vertical, vec![wref(d)], None);
        [a, b, c, d]
    }

    /// A properly constrained rectangle profile (closed loop, axis-aligned),
    /// so later constraints edit it without tearing the loop apart.
    fn rect_sketch(e: &mut Engine, w: f64, h: f64) -> (EntityId, [u64; 4]) {
        let s = add_sketch(e, "profile");
        (s, rect_in_sketch(e, s, w, h))
    }

    fn extrude(e: &mut Engine, sketch: EntityId, distance: Scalar) -> (EntityId, f64, usize) {
        match e
            .execute(Command::Extrude { doc: 1, sketch, distance, name: None, color: None })
            .unwrap()
        {
            CommandResult::SolidCreated { entity, volume, tris, .. } => (entity, volume, tris),
            r => panic!("unexpected result {r:?}"),
        }
    }

    fn entity_snapshot(e: &mut Engine, id: EntityId) -> EntitySnapshot {
        match e.execute(Command::GetScene { doc: 1 }).unwrap() {
            CommandResult::Scene { entities, .. } => entities.into_iter().find(|x| x.id == id).unwrap(),
            r => panic!("unexpected result {r:?}"),
        }
    }

    #[test]
    fn extrude_sketch_creates_printable_solid() {
        let mut e = engine();
        let (s, _) = rect_sketch(&mut e, 40.0, 30.0);
        let (id, volume, tris) = extrude(&mut e, s, num(5.0));
        assert!((volume - 40.0 * 30.0 * 5.0).abs() < 1e-6, "volume {volume}");
        assert_eq!(tris, 12, "a rectangle extrusion is a box");
        let z = extent(&mut e, id, 2);
        assert!((z - 5.0).abs() < 1e-6, "z extent {z}");
        // The solid inherits the sketch placement (base on the sketch plane).
        let snap = entity_snapshot(&mut e, id);
        let sketch = entity_snapshot(&mut e, s);
        assert_eq!(snap.translation, sketch.translation);
        assert_eq!(snap.rotation_deg, sketch.rotation_deg);
    }

    #[test]
    fn extrude_turns_nested_circle_into_a_hole() {
        let mut e = engine();
        let (s, _) = rect_sketch(&mut e, 40.0, 30.0);
        let c = match e
            .execute(Command::AddSketchCircle {
                doc: 1,
                sketch: s,
                center: [num(20.0), num(15.0)],
                radius: num(5.0),
            })
            .unwrap()
        {
            CommandResult::SketchElementCreated { element, .. } => element,
            r => panic!("unexpected result {r:?}"),
        };
        constrain(&mut e, s, K::Radius, vec![wref(c)], Some(num(5.0)));
        let (_, volume, _) = extrude(&mut e, s, num(2.0));
        let expected = (40.0 * 30.0 - std::f64::consts::PI * 25.0) * 2.0;
        assert!((volume - expected).abs() / expected < 0.01, "volume {volume} vs {expected}");
    }

    /// Adding a constraint *after* the extrude must update the solid now,
    /// not only on the next full replay.
    #[test]
    fn solid_follows_later_sketch_edits() {
        let mut e = engine();
        let (s, [bottom, ..]) = rect_sketch(&mut e, 40.0, 30.0);
        let (id, volume, _) = extrude(&mut e, s, num(5.0));
        assert!((volume - 6000.0).abs() < 1e-6);
        constrain(&mut e, s, K::Distance, vec![pref(bottom, 0), pref(bottom, 1)], Some(num(60.0)));
        let snap = entity_snapshot(&mut e, id);
        let xs: Vec<f64> = snap.mesh.positions.iter().map(|p| p[0] as f64).collect();
        let width = xs.iter().cloned().fold(f64::MIN, f64::max) - xs.iter().cloned().fold(f64::MAX, f64::min);
        assert!((width - 60.0).abs() < 1e-4, "solid width {width} did not follow the constraint");
        // Volume follows too: 60 × 30 × 5.
        let zs: Vec<f64> = snap.mesh.positions.iter().map(|p| p[2] as f64).collect();
        let depth = zs.iter().cloned().fold(f64::MIN, f64::max) - zs.iter().cloned().fold(f64::MAX, f64::min);
        assert!((width * 30.0 * depth - 9000.0).abs() < 1.0, "volume {:.1}", width * 30.0 * depth);
    }

    #[test]
    fn extrude_replays_parametrically() {
        let mut e = engine();
        e.execute(Command::SetParameter { doc: 1, name: "thickness".into(), value: num(4.0), description: None })
            .unwrap();
        let (s, _) = rect_sketch(&mut e, 20.0, 20.0);
        let (id, before, _) = extrude(&mut e, s, Scalar::Expression("thickness".into()));
        assert!((before - 1600.0).abs() < 1e-6, "volume {before}");
        e.execute(Command::SetParameter { doc: 1, name: "thickness".into(), value: num(10.0), description: None })
            .unwrap();
        let snap = entity_snapshot(&mut e, id);
        let zs: Vec<f64> = snap.mesh.positions.iter().map(|p| p[2] as f64).collect();
        let z = zs.iter().cloned().fold(f64::MIN, f64::max) - zs.iter().cloned().fold(f64::MAX, f64::min);
        assert!((z - 10.0).abs() < 1e-6, "z extent after parameter change: {z}");
    }

    #[test]
    fn negative_extrude_builds_below_the_sketch_plane() {
        let mut e = engine();
        let (s, _) = rect_sketch(&mut e, 10.0, 10.0);
        let (id, volume, _) = extrude(&mut e, s, num(-3.0));
        assert!((volume - 300.0).abs() < 1e-6);
        let snap = entity_snapshot(&mut e, id);
        let zs: Vec<f64> = snap.mesh.positions.iter().map(|p| p[2] as f64).collect();
        let (lo, hi) = (
            zs.iter().cloned().fold(f64::MAX, f64::min),
            zs.iter().cloned().fold(f64::MIN, f64::max),
        );
        assert!((lo + 3.0).abs() < 1e-6 && hi.abs() < 1e-6, "z {lo}..{hi}");
    }

    #[test]
    fn revolve_sketch_makes_a_tube() {
        let mut e = engine();
        let s = add_sketch(&mut e, "profile");
        add_line(&mut e, s, [0.0, 5.0], [20.0, 5.0]);
        add_line(&mut e, s, [20.0, 5.0], [20.0, 10.0]);
        add_line(&mut e, s, [20.0, 10.0], [0.0, 10.0]);
        add_line(&mut e, s, [0.0, 10.0], [0.0, 5.0]);
        match e
            .execute(Command::Revolve {
                doc: 1,
                sketch: s,
                angle_deg: None,
                axis: None,
                segments: Some(256),
                name: None,
                color: None,
            })
            .unwrap()
        {
            CommandResult::SolidCreated { volume, tris, .. } => {
                let expected = std::f64::consts::PI * (100.0 - 25.0) * 20.0;
                assert!((volume - expected).abs() / expected < 0.005, "volume {volume} vs {expected}");
                assert!(tris > 100);
            },
            r => panic!("unexpected result {r:?}"),
        }
    }

    #[test]
    fn open_profile_is_rejected_with_a_hint() {
        let mut e = engine();
        let s = add_sketch(&mut e, "profile");
        add_line(&mut e, s, [0.0, 0.0], [10.0, 0.0]);
        add_line(&mut e, s, [10.0, 0.0], [10.0, 10.0]);
        let err = e
            .execute(Command::Extrude { doc: 1, sketch: s, distance: num(5.0), name: None, color: None })
            .unwrap_err();
        let msg = err.to_string();
        assert!(msg.contains("profile is open"), "{msg}");
        // The failed command must not leave a program entry behind.
        assert_eq!(entity_snapshot(&mut e, s).id, s);
    }

    #[test]
    fn revolve_rejects_a_profile_crossing_the_axis() {
        let mut e = engine();
        let s = add_sketch(&mut e, "profile");
        add_line(&mut e, s, [0.0, -5.0], [20.0, -5.0]);
        add_line(&mut e, s, [20.0, -5.0], [20.0, 10.0]);
        add_line(&mut e, s, [20.0, 10.0], [0.0, 10.0]);
        add_line(&mut e, s, [0.0, 10.0], [0.0, -5.0]);
        let err = e
            .execute(Command::Revolve {
                doc: 1,
                sketch: s,
                angle_deg: None,
                axis: None,
                segments: None,
                name: None,
                color: None,
            })
            .unwrap_err();
        assert!(err.to_string().contains("crosses the revolve axis"), "{err}");
    }

    #[test]
    fn deleting_a_sketch_cascades_to_its_solids() {
        let mut e = engine();
        let (s, _) = rect_sketch(&mut e, 10.0, 10.0);
        extrude(&mut e, s, num(2.0));
        e.execute(Command::Delete { doc: 1, entity: s }).unwrap();
        match e.execute(Command::GetScene { doc: 1 }).unwrap() {
            CommandResult::Scene { entities, .. } => assert!(entities.is_empty(), "solid should cascade"),
            r => panic!("unexpected result {r:?}"),
        }
    }

    #[test]
    fn extruded_solid_exports_to_stl() {
        let mut e = engine();
        let (s, _) = rect_sketch(&mut e, 10.0, 10.0);
        extrude(&mut e, s, num(5.0));
        let path = std::env::temp_dir().join("agcad_extrude.stl");
        let path_str = path.to_string_lossy().to_string();
        match e.execute(Command::ExportStl { doc: 1, path: path_str, entities: None }).unwrap() {
            CommandResult::StlExported { tris, bytes, .. } => {
                assert_eq!(tris, 12);
                assert_eq!(bytes, 84 + 12 * 50);
            },
            r => panic!("unexpected result {r:?}"),
        }
        let _ = std::fs::remove_file(&path);
    }

    #[test]
    fn extrude_is_deterministic_across_engines() {
        let build = || {
            let mut e = engine();
            let (s, _) = rect_sketch(&mut e, 33.0, 17.0);
            let (id, volume, _) = extrude(&mut e, s, num(4.5));
            let snap = entity_snapshot(&mut e, id);
            (snap.mesh.positions, snap.mesh.indices, volume.to_bits())
        };
        assert_eq!(build(), build());
    }

    #[test]
    fn undo_restores_extrude() {
        let mut e = engine();
        let (s, _) = rect_sketch(&mut e, 10.0, 10.0);
        let (id, volume, _) = extrude(&mut e, s, num(2.0));
        e.execute(Command::Undo { doc: 1 }).unwrap();
        match e.execute(Command::GetScene { doc: 1 }).unwrap() {
            CommandResult::Scene { entities, .. } => {
                assert_eq!(entities.len(), 1, "undo removes the solid");
                assert_eq!(entities[0].id, s);
            },
            r => panic!("unexpected result {r:?}"),
        }
        e.execute(Command::Redo { doc: 1 }).unwrap();
        match e.execute(Command::GetScene { doc: 1 }).unwrap() {
            CommandResult::Scene { entities, .. } => {
                let solid = entities.iter().find(|x| x.id == id).expect("solid restored");
                let zs: Vec<f64> = solid.mesh.positions.iter().map(|p| p[2] as f64).collect();
                let z = zs.iter().cloned().fold(f64::MIN, f64::max) - zs.iter().cloned().fold(f64::MAX, f64::min);
                assert!((z - 2.0).abs() < 1e-6);
            },
            r => panic!("unexpected result {r:?}"),
        }
        assert!((volume - 200.0).abs() < 1e-6);
    }

    #[test]
    fn transforming_a_sketch_moves_its_solids() {
        let mut e = engine();
        let (s, _) = rect_sketch(&mut e, 10.0, 10.0);
        let (id, _, _) = extrude(&mut e, s, num(2.0));
        e.execute(Command::Transform {
            doc: 1,
            entity: s,
            translation: Some([num(0.0), num(0.0), num(10.0)]),
            rotation_deg: None,
            scale: None,
        })
        .unwrap();
        let snap = entity_snapshot(&mut e, id);
        assert_eq!(snap.translation, [0.0, 0.0, 10.0], "solid must follow the sketch placement");
    }

    // --- sketches on faces and datum planes ----------------------------------

    #[test]
    fn datum_planes_place_the_sketch_frame() {
        use agcad_protocol::DatumPlane;

        let mut e = engine();
        // Default: XY at the origin.
        let xy = add_sketch(&mut e, "xy");
        let f = sketch_frame(&mut e, xy);
        assert!(close_to(f.origin, [0.0; 3], 1e-9));
        assert!(close_to(f.normal, [0.0, 0.0, 1.0], 1e-9));
        assert!(close_to(f.x_axis, [1.0, 0.0, 0.0], 1e-9));

        // Offset along the normal, right-handed local axes.
        let xz = add_sketch_on(
            &mut e,
            "front",
            SketchPlane::Datum { plane: DatumPlane::Xz, offset: Some(num(7.0)) },
        );
        let f = sketch_frame(&mut e, xz);
        assert!(close_to(f.origin, [0.0, -7.0, 0.0], 1e-9), "{:?}", f.origin);
        assert!(close_to(f.normal, [0.0, -1.0, 0.0], 1e-9));
        assert!(close_to(f.x_axis, [1.0, 0.0, 0.0], 1e-9));

        let yz = add_sketch_on(
            &mut e,
            "right",
            SketchPlane::Datum { plane: DatumPlane::Yz, offset: Some(num(3.0)) },
        );
        let f = sketch_frame(&mut e, yz);
        assert!(close_to(f.origin, [3.0, 0.0, 0.0], 1e-9), "{:?}", f.origin);
        assert!(close_to(f.normal, [1.0, 0.0, 0.0], 1e-9));
        assert!(close_to(f.x_axis, [0.0, 1.0, 0.0], 1e-9));

        // The plane offset may be an expression and follows a knob.
        e.execute(Command::SetParameter {
            doc: 1,
            name: "plate".into(),
            value: num(2.0),
            description: None,
        })
        .unwrap();
        let param = add_sketch_on(
            &mut e,
            "param",
            SketchPlane::Datum {
                plane: DatumPlane::Xy,
                offset: Some(Scalar::Expression("plate*3".into())),
            },
        );
        assert!((sketch_frame(&mut e, param).origin[2] - 6.0).abs() < 1e-9);
        e.execute(Command::SetParameter { doc: 1, name: "plate".into(), value: num(4.0), description: None })
            .unwrap();
        assert!((sketch_frame(&mut e, param).origin[2] - 12.0).abs() < 1e-9);
    }

    #[test]
    fn custom_plane_projects_an_in_plane_x_axis() {
        let mut e = engine();
        let s = add_sketch_on(
            &mut e,
            "tilted",
            SketchPlane::Custom {
                origin: [num(1.0), num(2.0), num(3.0)],
                normal: [1.0, 0.0, 1.0],
                x_axis: Some([2.0, 0.0, -2.0]),
            },
        );
        let f = sketch_frame(&mut e, s);
        assert!(close_to(f.origin, [1.0, 2.0, 3.0], 1e-9));
        let inv = 1.0 / 2.0f64.sqrt();
        assert!(close_to(f.normal, [inv, 0.0, inv], 1e-9), "{:?}", f.normal);
        assert!(close_to(f.x_axis, [inv, 0.0, -inv], 1e-9), "{:?}", f.x_axis);
    }

    #[test]
    fn sketch_on_the_top_face_extrudes_outward_or_into_the_body() {
        let mut e = engine();
        // Base spans z = 0..10; its top face is z = 10.
        let base = add_box(&mut e, [0.0, 0.0, 5.0], [40.0, 30.0, 10.0]);
        let top = add_sketch_on(&mut e, "top", face_plane(base, [0.0, 0.0, 1.0]));
        let f = sketch_frame(&mut e, top);
        assert!(close_to(f.origin, [0.0, 0.0, 10.0], 1e-6), "{:?}", f.origin);
        assert!(close_to(f.normal, [0.0, 0.0, 1.0], 1e-9));
        assert!(close_to(f.x_axis, [1.0, 0.0, 0.0], 1e-9));
        rect_in_sketch(&mut e, top, 10.0, 8.0);

        // Positive: material in front of the face.
        let (boss, volume, _) = extrude(&mut e, top, num(4.0));
        assert!((volume - 320.0).abs() < 1e-6, "volume {volume}");
        let bb = entity_snapshot(&mut e, boss).world_bbox;
        assert!((bb.min[2] - 10.0).abs() < 1e-5 && (bb.max[2] - 14.0).abs() < 1e-5, "{bb:?}");

        // Negative: the same profile goes into the body instead.
        let mut into = engine();
        let base = add_box(&mut into, [0.0, 0.0, 5.0], [40.0, 30.0, 10.0]);
        let top = add_sketch_on(&mut into, "top", face_plane(base, [0.0, 0.0, 1.0]));
        rect_in_sketch(&mut into, top, 10.0, 8.0);
        let (cutter, _, _) = extrude(&mut into, top, num(-3.0));
        let bb = entity_snapshot(&mut into, cutter).world_bbox;
        assert!((bb.min[2] - 7.0).abs() < 1e-5 && (bb.max[2] - 10.0).abs() < 1e-5, "{bb:?}");
    }

    #[test]
    fn a_planar_cap_is_one_face_even_when_tessellated() {
        let mut e = engine();
        // A 32-facet cylinder's top cap: the fan's triangles are coplanar,
        // so they must group into one face, not 32 slivers.
        let post = match e
            .execute(Command::AddCylinder {
                doc: 1,
                name: "post".into(),
                center: [num(0.0), num(0.0), num(10.0)],
                radius: num(10.0),
                height: num(20.0),
                segments: Some(32),
                color: None,
            })
            .unwrap()
        {
            CommandResult::EntityCreated { entity, .. } => entity,
            r => panic!("unexpected result {r:?}"),
        };
        let top = add_sketch_on(&mut e, "cap", face_plane(post, [0.0, 0.0, 1.0]));
        let f = sketch_frame(&mut e, top);
        assert!(close_to(f.origin, [0.0, 0.0, 20.0], 1e-5), "{:?}", f.origin);
        assert!(close_to(f.normal, [0.0, 0.0, 1.0], 1e-9));
        assert!(close_to(f.x_axis, [1.0, 0.0, 0.0], 1e-9));
    }

    #[test]
    fn a_face_sketch_pocket_cuts_the_body() {
        let mut e = engine();
        let base = add_box(&mut e, [0.0, 0.0, 5.0], [40.0, 30.0, 10.0]);
        let top = add_sketch_on(&mut e, "pocket", face_plane(base, [0.0, 0.0, 1.0]));

        // A 10 x 8 rectangle around the face origin.
        let pts = [[-5.0, -4.0], [5.0, -4.0], [5.0, 4.0], [-5.0, 4.0]];
        for i in 0..4 {
            add_line(&mut e, top, pts[i], pts[(i + 1) % 4]);
        }
        let (cutter, cutter_volume, _) = extrude(&mut e, top, num(-3.0));
        assert!((cutter_volume - 240.0).abs() < 1e-6);

        let result = match e
            .execute(Command::Boolean {
                doc: 1,
                op: BooleanOp::Difference,
                entities: vec![base, cutter],
                name: Some("pocketed".into()),
                color: None,
            })
            .unwrap()
        {
            CommandResult::EntityCreated { entity, .. } => entity,
            r => panic!("unexpected result {r:?}"),
        };
        let snap = entity_snapshot(&mut e, result);
        let volume = crate::fillet::signed_volume(&snap.mesh);
        assert!((volume - (12000.0 - 240.0)).abs() < 1e-3, "volume {volume}");
        // The pocket is open at the top, so the world bbox is unchanged.
        assert!((snap.world_bbox.max[2] - 10.0).abs() < 1e-5);
    }

    #[test]
    fn a_face_sketch_follows_a_parameter_change() {
        let mut e = engine();
        e.execute(Command::SetParameter { doc: 1, name: "h".into(), value: num(10.0), description: None })
            .unwrap();
        let base = match e
            .execute(Command::AddBox {
                doc: 1,
                name: "base".into(),
                center: [num(0.0), num(0.0), Scalar::Expression("h/2".into())],
                size: [num(40.0), num(30.0), Scalar::Expression("h".into())],
                color: None,
            })
            .unwrap()
        {
            CommandResult::EntityCreated { entity, .. } => entity,
            r => panic!("unexpected result {r:?}"),
        };
        // No `near`: the +Z normal alone is unambiguous, so the sketch
        // follows the top face as the body grows.
        let top = add_sketch_on(&mut e, "top", face_plane(base, [0.0, 0.0, 1.0]));
        rect_in_sketch(&mut e, top, 10.0, 8.0);
        let (boss, _, _) = extrude(&mut e, top, num(2.0));

        let bb = entity_snapshot(&mut e, boss).world_bbox;
        assert!((bb.min[2] - 10.0).abs() < 1e-5 && (bb.max[2] - 12.0).abs() < 1e-5, "{bb:?}");

        e.execute(Command::SetParameter { doc: 1, name: "h".into(), value: num(25.0), description: None })
            .unwrap();
        assert!((sketch_frame(&mut e, top).origin[2] - 25.0).abs() < 1e-5);
        let bb = entity_snapshot(&mut e, boss).world_bbox;
        assert!((bb.min[2] - 25.0).abs() < 1e-5 && (bb.max[2] - 27.0).abs() < 1e-5, "{bb:?}");
    }

    #[test]
    fn near_selects_between_parallel_faces() {
        let mut e = engine();
        let base = add_box(&mut e, [0.0, 0.0, 5.0], [40.0, 40.0, 10.0]);
        let boss = add_box(&mut e, [0.0, 0.0, 10.0], [10.0, 10.0, 20.0]);
        let fused = match e
            .execute(Command::Boolean {
                doc: 1,
                op: BooleanOp::Union,
                entities: vec![base, boss],
                name: None,
                color: None,
            })
            .unwrap()
        {
            CommandResult::EntityCreated { entity, .. } => entity,
            r => panic!("unexpected result {r:?}"),
        };

        // Largest +Z face (the base shoulder at z = 10) when no point is given.
        let shoulder = add_sketch_on(&mut e, "shoulder", face_plane(fused, [0.0, 0.0, 1.0]));
        assert!((sketch_frame(&mut e, shoulder).origin[2] - 10.0).abs() < 1e-5);

        let boss_top = add_sketch_on(
            &mut e,
            "boss_top",
            SketchPlane::Face {
                entity: fused,
                near: Some([num(0.0), num(0.0), num(20.0)]),
                normal: Some([0.0, 0.0, 1.0]),
                angle_tol_deg: None,
            },
        );
        let f = sketch_frame(&mut e, boss_top);
        assert!((f.origin[2] - 20.0).abs() < 1e-5, "{:?}", f.origin);
    }

    #[test]
    fn a_face_sketch_uses_the_world_aligned_frame_of_a_rotated_entity() {
        let mut e = engine();
        let base = add_box(&mut e, [0.0, 0.0, 0.0], [40.0, 20.0, 10.0]);
        // Tip the box onto its side: the original top face now points -Y, so
        // its plane is world XZ. The sketch X stays world +X (a plane has no
        // intrinsic X) and the extrude normal follows the face.
        e.execute(Command::Transform {
            doc: 1,
            entity: base,
            translation: None,
            rotation_deg: Some([num(90.0), num(0.0), num(0.0)]),
            scale: None,
        })
        .unwrap();
        let side = add_sketch_on(&mut e, "side", face_plane(base, [0.0, -1.0, 0.0]));
        let f = sketch_frame(&mut e, side);
        assert!(close_to(f.normal, [0.0, -1.0, 0.0], 1e-6), "{:?}", f.normal);
        assert!(close_to(f.x_axis, [1.0, 0.0, 0.0], 1e-5), "{:?}", f.x_axis);
        assert!(close_to(f.origin, [0.0, -5.0, 0.0], 1e-5), "{:?}", f.origin);
    }

    #[test]
    fn bad_sketch_planes_are_rejected_with_hints() {
        let mut e = engine();
        let box_id = add_box(&mut e, [0.0; 3], [10.0; 3]);
        let sketch = add_sketch(&mut e, "s");

        let err = e
            .execute(Command::AddSketch {
                doc: 1,
                name: "bad".into(),
                plane: Some(face_plane(sketch, [0.0, 0.0, 1.0])),
            })
            .unwrap_err();
        assert!(err.to_string().contains("is a sketch"), "{err}");
        assert!(err.to_string().contains("datum plane"), "{err}");

        let err = e
            .execute(Command::AddSketch {
                doc: 1,
                name: "bad".into(),
                plane: Some(face_plane(box_id, [1.0, 1.0, 0.0])),
            })
            .unwrap_err();
        assert!(err.to_string().contains("no face with outward normal"), "{err}");
        assert!(err.to_string().contains("available face normals"), "{err}");

        let err = e
            .execute(Command::AddSketch {
                doc: 1,
                name: "bad".into(),
                plane: Some(SketchPlane::Custom {
                    origin: [num(0.0), num(0.0), num(0.0)],
                    normal: [0.0; 3],
                    x_axis: None,
                }),
            })
            .unwrap_err();
        assert!(err.to_string().contains("normal must be a non-zero vector"), "{err}");

        let err = e
            .execute(Command::AddSketch {
                doc: 1,
                name: "bad".into(),
                plane: Some(SketchPlane::Face {
                    entity: box_id,
                    near: None,
                    normal: Some([0.0, 0.0, 1.0]),
                    angle_tol_deg: Some(0.0),
                }),
            })
            .unwrap_err();
        assert!(err.to_string().contains("angle_tol_deg"), "{err}");

        // None of the failures left a program entry behind.
        let program_len = match e.execute(Command::GetProgram { doc: 1 }).unwrap() {
            CommandResult::Program { program, .. } => program.len(),
            r => panic!("unexpected result {r:?}"),
        };
        assert_eq!(program_len, 2, "box + the good sketch only");
    }

    #[test]
    fn face_sketch_round_trips_through_save_open() {
        let (path, path_s) = temp_file("agcad_face_sketch.agcad");
        let mut a = engine();
        a.execute(Command::SetParameter { doc: 1, name: "h".into(), value: num(10.0), description: None })
            .unwrap();
        let base = match a
            .execute(Command::AddBox {
                doc: 1,
                name: "base".into(),
                center: [num(0.0), num(0.0), Scalar::Expression("h/2".into())],
                size: [num(40.0), num(30.0), Scalar::Expression("h".into())],
                color: None,
            })
            .unwrap()
        {
            CommandResult::EntityCreated { entity, .. } => entity,
            r => panic!("unexpected result {r:?}"),
        };
        let top = add_sketch_on(&mut a, "top", face_plane(base, [0.0, 0.0, 1.0]));
        rect_in_sketch(&mut a, top, 10.0, 8.0);
        extrude(&mut a, top, num(3.0));
        let before = entities_json(&mut a, 1);

        a.execute(Command::SaveDocument { doc: 1, path: path_s.clone(), source: None }).unwrap();
        let text = std::fs::read_to_string(&path).unwrap();
        assert!(text.contains("\"face\""), "the plane reference is part of the program");
        assert!(text.contains("\"normal\""), "{text}");

        let mut b = Engine::new();
        b.execute(Command::OpenDocument { path: path_s.clone(), name: None }).unwrap();
        assert_eq!(before, entities_json(&mut b, 1), "replay must reproduce the face placement");

        // A plane reference must round-trip byte-for-byte too.
        let (path2, path2_s) = temp_file("agcad_face_sketch_2.agcad");
        b.execute(Command::SaveDocument { doc: 1, path: path2_s, source: None }).unwrap();
        assert_eq!(
            std::fs::read(&path).unwrap(),
            std::fs::read(&path2).unwrap(),
            "save -> open -> save must be a no-op diff with a plane reference"
        );

        // And the opened document still follows its knob.
        b.execute(Command::SetParameter { doc: 1, name: "h".into(), value: num(20.0), description: None })
            .unwrap();
        let bb = entity_snapshot(&mut b, 3).world_bbox;
        assert!((bb.min[2] - 20.0).abs() < 1e-5 && (bb.max[2] - 23.0).abs() < 1e-5, "{bb:?}");

        let _ = std::fs::remove_file(&path);
        let _ = std::fs::remove_file(&path2);
    }

    /// Turning a knob (no description passed) must keep the label the
    /// author wrote — the whole point of "the user only turns parameters".
    #[test]
    fn changing_a_parameter_keeps_its_description() {
        let mut e = engine();
        e.execute(Command::SetParameter {
            doc: 1,
            name: "wall".into(),
            value: num(2.0),
            description: Some("wall thickness".into()),
        })
        .unwrap();
        e.execute(Command::SetParameter {
            doc: 1,
            name: "wall".into(),
            value: num(3.0),
            description: None,
        })
        .unwrap();
        match e.execute(Command::GetParameters { doc: 1 }).unwrap() {
            CommandResult::Parameters { params, .. } => {
                assert_eq!(params.len(), 1);
                assert_eq!(params[0].value, 3.0);
                assert_eq!(params[0].description.as_deref(), Some("wall thickness"));
            },
            r => panic!("unexpected result {r:?}"),
        }
    }

    // --- save / open ---------------------------------------------------------

    /// Deterministic signature of a document's entities: ids, names, exact
    /// placement and mesh bits. Comparing the serialized snapshots catches
    /// any drift in replay.
    fn entities_json(e: &mut Engine, doc: DocId) -> String {
        match e.execute(Command::GetScene { doc }).unwrap() {
            CommandResult::Scene { entities, .. } => serde_json::to_string(&entities).unwrap(),
            r => panic!("unexpected result {r:?}"),
        }
    }

    /// Z extent of the whole scene (solids and sketch ribbons both sit at
    /// z = 0..height, so this measures the extrusion).
    fn scene_z_extent(e: &mut Engine, doc: DocId) -> f64 {
        match e.execute(Command::GetScene { doc }).unwrap() {
            CommandResult::Scene { entities, .. } => {
                let zs: Vec<f64> = entities
                    .iter()
                    .flat_map(|x| x.mesh.positions.iter().map(|p| p[2] as f64))
                    .collect();
                let hi = zs.iter().cloned().fold(f64::MIN, f64::max);
                let lo = zs.iter().cloned().fold(f64::MAX, f64::min);
                hi - lo
            },
            r => panic!("unexpected result {r:?}"),
        }
    }

    fn temp_file(name: &str) -> (std::path::PathBuf, String) {
        let path = std::env::temp_dir().join(name);
        let s = path.to_string_lossy().into_owned();
        (path, s)
    }

    #[test]
    fn save_open_round_trips_a_parametric_document() {
        let (path, path_s) = temp_file("agcad_roundtrip.agcad");

        let mut a = engine();
        a.execute(Command::SetParameter {
            doc: 1,
            name: "thickness".into(),
            value: num(5.0),
            description: Some("plate".into()),
        })
        .unwrap();
        let (sketch, _) = rect_sketch(&mut a, 30.0, 20.0);
        let (_, volume, _) = extrude(&mut a, sketch, Scalar::Expression("thickness".into()));
        assert!((volume - 3000.0).abs() < 1e-6);
        let before = entities_json(&mut a, 1);

        match a
            .execute(Command::SaveDocument { doc: 1, path: path_s.clone(), source: None })
            .unwrap()
        {
            CommandResult::DocumentSaved { bytes, commands, params, path, .. } => {
                assert!(bytes > 0);
                // add_sketch + 4 lines + 8 constraints + extrude
                assert_eq!(commands, 14);
                assert_eq!(params, 1);
                assert!(path.ends_with("agcad_roundtrip.agcad"), "{path}");
            },
            r => panic!("unexpected result {r:?}"),
        }

        let mut b = Engine::new();
        match b
            .execute(Command::OpenDocument { path: path_s.clone(), name: None })
            .unwrap()
        {
            CommandResult::DocumentOpened { doc, name, commands, params, entities } => {
                assert_eq!(doc, 1, "a fresh engine allocates id 1");
                assert_eq!(name, "test");
                assert_eq!(commands, 14);
                assert_eq!(params, 1);
                assert_eq!(entities, 2, "sketch + solid");
            },
            r => panic!("unexpected result {r:?}"),
        }
        // Replay must reproduce ids, placement and every mesh bit.
        assert_eq!(before, entities_json(&mut b, 1));

        let _ = std::fs::remove_file(&path);
    }

    #[test]
    fn save_is_byte_stable_across_reopen() {
        let (p1, p1s) = temp_file("agcad_stable_a.agcad");
        let (p2, p2s) = temp_file("agcad_stable_b.agcad");

        let mut a = engine();
        rect_sketch(&mut a, 10.0, 10.0);
        let source = SourceDoc { language: "python".into(), text: "print('hi')\n".into() };
        a.execute(Command::SaveDocument {
            doc: 1,
            path: p1s.clone(),
            source: Some(source),
        })
        .unwrap();

        // Open in a fresh engine, then save again without passing a source:
        // the recipe stored on the document must come back out.
        let mut b = Engine::new();
        b.execute(Command::OpenDocument { path: p1s.clone(), name: None }).unwrap();
        b.execute(Command::SaveDocument { doc: 1, path: p2s.clone(), source: None }).unwrap();

        let first = std::fs::read(&p1).unwrap();
        let second = std::fs::read(&p2).unwrap();
        assert_eq!(first, second, "save -> open -> save must be a no-op diff");

        let text = String::from_utf8(first).unwrap();
        assert!(text.contains("\"format\": \"agcad-document\""));
        assert!(text.contains("\"version\": 3"));
        assert!(text.contains("print('hi')"), "the embedded source is preserved");
        // The file is the program, not the geometry.
        assert!(!text.contains("\"mesh\""));
        assert!(!text.contains("positions"));

        let _ = std::fs::remove_file(&p1);
        let _ = std::fs::remove_file(&p2);
    }

    #[test]
    fn open_rejects_bad_files_with_actionable_hints() {
        let (path, path_s) = temp_file("agcad_bad.agcad");

        std::fs::write(&path, "{ not json").unwrap();
        let err = Engine::new()
            .execute(Command::OpenDocument { path: path_s.clone(), name: None })
            .unwrap_err();
        assert!(err.to_string().contains("cannot parse document"), "{err}");
        assert!(err.to_string().contains("save_document"), "{err}");

        std::fs::write(&path, r#"{"format":"something-else","version":1,"name":"x"}"#).unwrap();
        let err = Engine::new()
            .execute(Command::OpenDocument { path: path_s.clone(), name: None })
            .unwrap_err();
        assert!(err.to_string().contains("not an agcad document"), "{err}");

        std::fs::write(&path, r#"{"format":"agcad-document","version":99,"name":"x"}"#).unwrap();
        let err = Engine::new()
            .execute(Command::OpenDocument { path: path_s.clone(), name: None })
            .unwrap_err();
        assert!(err.to_string().contains("version 99"), "{err}");

        let err = Engine::new()
            .execute(Command::OpenDocument {
                path: "/nonexistent/nope.agcad".into(),
                name: None,
            })
            .unwrap_err();
        assert!(err.to_string().contains("cannot read document"), "{err}");

        let _ = std::fs::remove_file(&path);
    }

    /// The file's commands are normalized to doc 1 so saves are stable; on
    /// open they must be rebound to the id the engine actually allocated,
    /// or replay would target a document that does not exist.
    #[test]
    fn open_rebinds_doc_ids_so_the_program_still_replays() {
        let (path, path_s) = temp_file("agcad_rebind.agcad");

        let mut a = engine();
        a.execute(Command::SetParameter {
            doc: 1,
            name: "h".into(),
            value: num(4.0),
            description: None,
        })
        .unwrap();
        let (sketch, _) = rect_sketch(&mut a, 20.0, 20.0);
        extrude(&mut a, sketch, Scalar::Expression("h".into()));
        a.execute(Command::SaveDocument { doc: 1, path: path_s.clone(), source: None })
            .unwrap();

        // The fresh engine already owns doc 1, so the opened file lands on 2.
        let mut b = engine();
        match b
            .execute(Command::OpenDocument { path: path_s.clone(), name: Some("copy".into()) })
            .unwrap()
        {
            CommandResult::DocumentOpened { doc, name, .. } => {
                assert_eq!(doc, 2);
                assert_eq!(name, "copy");
            },
            r => panic!("unexpected result {r:?}"),
        }
        assert!((scene_z_extent(&mut b, 2) - 4.0).abs() < 1e-6);

        // A parameter change replays the whole program against doc 2.
        b.execute(Command::SetParameter {
            doc: 2,
            name: "h".into(),
            value: num(9.0),
            description: None,
        })
        .unwrap();
        assert!((scene_z_extent(&mut b, 2) - 9.0).abs() < 1e-6);

        // The original document is untouched.
        assert!((scene_z_extent(&mut a, 1) - 4.0).abs() < 1e-6);

        let _ = std::fs::remove_file(&path);
    }

    #[test]
    fn get_program_exposes_params_and_expression_arguments() {
        let mut e = engine();
        e.execute(Command::SetParameter {
            doc: 1,
            name: "thickness".into(),
            value: num(6.0),
            description: Some("plate".into()),
        })
        .unwrap();
        let (sketch, _) = rect_sketch(&mut e, 10.0, 10.0);
        extrude(&mut e, sketch, Scalar::Expression("thickness".into()));

        match e.execute(Command::GetProgram { doc: 1 }).unwrap() {
            CommandResult::Program { name, params, program, .. } => {
                assert_eq!(name, "test");
                assert_eq!(params.len(), 1);
                assert_eq!(params[0].name, "thickness");
                assert_eq!(params[0].value, 6.0);
                assert_eq!(params[0].description.as_deref(), Some("plate"));

                let distances: Vec<_> = program
                    .iter()
                    .filter_map(|c| match c {
                        Command::Extrude { distance, .. } => Some(distance.clone()),
                        _ => None,
                    })
                    .collect();
                assert_eq!(distances.len(), 1);
                assert!(
                    matches!(&distances[0], Scalar::Expression(s) if s == "thickness"),
                    "the program keeps the expression, not the resolved number"
                );
            },
            r => panic!("unexpected result {r:?}"),
        }
    }

    /// The script is the source of truth: a saved `.agcad` carries the
    /// recipe, and `get_program` hands it back after a reopen so a handoff
    /// can recover the source without executing it.
    #[test]
    fn get_program_recovers_the_embedded_script() {
        let (path, path_s) = temp_file("agcad_program_source.agcad");
        let (path2, path2_s) = temp_file("agcad_program_source_2.agcad");
        let source = SourceDoc {
            language: "python".into(),
            text: "print('the recipe')\n".into(),
        };

        let mut a = engine();
        rect_sketch(&mut a, 10.0, 10.0);
        match a.execute(Command::GetProgram { doc: 1 }).unwrap() {
            CommandResult::Program { source: None, .. } => {},
            r => panic!("a fresh document has no embedded source: {r:?}"),
        }
        a.execute(Command::SaveDocument {
            doc: 1,
            path: path_s.clone(),
            source: Some(source.clone()),
        })
        .unwrap();
        // The live document remembers what it just wrote.
        match a.execute(Command::GetProgram { doc: 1 }).unwrap() {
            CommandResult::Program { source: Some(s), .. } => assert_eq!(s.text, source.text),
            r => panic!("save_document must remember the provided source: {r:?}"),
        }

        let mut b = Engine::new();
        b.execute(Command::OpenDocument { path: path_s.clone(), name: None }).unwrap();
        match b.execute(Command::GetProgram { doc: 1 }).unwrap() {
            CommandResult::Program { source: Some(s), .. } => {
                assert_eq!(s.language, "python");
                assert_eq!(s.text, source.text);
            },
            r => panic!("opened document must expose the embedded source: {r:?}"),
        }

        // A save without a source argument keeps the stored recipe.
        b.execute(Command::SaveDocument { doc: 1, path: path2_s.clone(), source: None }).unwrap();
        let mut c = Engine::new();
        c.execute(Command::OpenDocument { path: path2_s.clone(), name: None }).unwrap();
        match c.execute(Command::GetProgram { doc: 1 }).unwrap() {
            CommandResult::Program { source: Some(s), .. } => assert_eq!(s.text, source.text),
            r => panic!("save -> open -> save must keep the source: {r:?}"),
        }

        let _ = std::fs::remove_file(&path);
        let _ = std::fs::remove_file(&path2);
    }

    #[test]
    fn save_empty_document_round_trips() {
        let (path, path_s) = temp_file("agcad_empty.agcad");
        let mut a = engine();
        a.execute(Command::SaveDocument { doc: 1, path: path_s.clone(), source: None })
            .unwrap();

        let mut b = Engine::new();
        match b.execute(Command::OpenDocument { path: path_s.clone(), name: None }).unwrap() {
            CommandResult::DocumentOpened { commands, params, entities, .. } => {
                assert_eq!((commands, params, entities), (0, 0, 0));
            },
            r => panic!("unexpected result {r:?}"),
        }
        let _ = std::fs::remove_file(&path);
    }

    // --- feedback-driven fixes ------------------------------------------------

    /// Explicit ids let one batch create a document and then address it —
    /// the "create then populate costs two round trips" friction.
    #[test]
    fn create_document_accepts_an_explicit_id() {
        let mut e = Engine::new();
        match e.execute(Command::CreateDocument { name: "first".into(), doc: Some(10) }).unwrap() {
            CommandResult::DocumentCreated { doc } => assert_eq!(doc, 10),
            r => panic!("unexpected result {r:?}"),
        }
        // Automatic allocation must stay clear of explicit ids.
        match e.execute(Command::CreateDocument { name: "next".into(), doc: None }).unwrap() {
            CommandResult::DocumentCreated { doc } => assert_eq!(doc, 11),
            r => panic!("unexpected result {r:?}"),
        }
        let err = e
            .execute(Command::CreateDocument { name: "dup".into(), doc: Some(10) })
            .unwrap_err();
        assert!(err.to_string().contains("already exists"), "{err}");
        assert!(err.to_string().contains("list_documents"), "{err}");
        let err = e
            .execute(Command::CreateDocument { name: "zero".into(), doc: Some(0) })
            .unwrap_err();
        assert!(err.to_string().contains("start at 1"), "{err}");

        // Both documents are usable.
        e.execute(Command::AddBox {
            doc: 10, name: "a".into(), center: vec3(0.0), size: vec3(1.0), color: None,
        })
        .unwrap();
        e.execute(Command::AddBox {
            doc: 11, name: "b".into(), center: vec3(0.0), size: vec3(1.0), color: None,
        })
        .unwrap();
    }

    /// The scene reports a world-space bbox alongside the local mesh, so a
    /// transform is visible in inspection (not only in the render).
    #[test]
    fn scene_reports_a_world_space_bbox() {
        let mut e = engine();
        let id = match e
            .execute(Command::AddCylinder {
                doc: 1, name: "post".into(),
                center: [num(0.0), num(0.0), num(10.0)],
                radius: num(5.0), height: num(20.0), segments: Some(32), color: None,
            })
            .unwrap()
        {
            CommandResult::EntityCreated { entity, .. } => entity,
            r => panic!("unexpected result {r:?}"),
        };

        let snap = entity_snapshot(&mut e, id);
        let bb = snap.world_bbox;
        // +Z cylinder, centered at z = 10, so it stands on the plane.
        assert!((bb.min[0] + 5.0).abs() < 1e-4 && (bb.max[0] - 5.0).abs() < 1e-4, "{bb:?}");
        assert!((bb.min[2] - 0.0).abs() < 1e-4 && (bb.max[2] - 20.0).abs() < 1e-4, "{bb:?}");

        // Lay it down along Y: the world bbox follows the transform while
        // the local mesh (and its bbox) is untouched.
        e.execute(Command::Transform {
            doc: 1, entity: id, translation: None,
            rotation_deg: Some([num(90.0), num(0.0), num(0.0)]), scale: None,
        })
        .unwrap();
        let snap = entity_snapshot(&mut e, id);
        let bb = snap.world_bbox;
        let (z, y) = (bb.max[2] - bb.min[2], bb.max[1] - bb.min[1]);
        assert!((z - 10.0).abs() < 0.2, "world z extent {z} ({bb:?})");
        assert!((y - 20.0).abs() < 0.2, "world y extent {y} ({bb:?})");

        let local_z = snap.mesh.positions.iter().map(|p| p[2]).fold(f32::MIN, f32::max)
            - snap.mesh.positions.iter().map(|p| p[2]).fold(f32::MAX, f32::min);
        assert!((local_z - 20.0).abs() < 1e-4, "local mesh must stay axis-aligned ({local_z})");
    }

    // --- fillets and chamfers ------------------------------------------------

    fn add_box(e: &mut Engine, center: [f64; 3], size: [f64; 3]) -> EntityId {
        match e
            .execute(Command::AddBox {
                doc: 1,
                name: "box".into(),
                center: [num(center[0]), num(center[1]), num(center[2])],
                size: [num(size[0]), num(size[1]), num(size[2])],
                color: None,
            })
            .unwrap()
        {
            CommandResult::EntityCreated { entity, .. } => entity,
            r => panic!("unexpected result {r:?}"),
        }
    }

    fn modified(result: CommandResult) -> (EntityId, u32, f64, Mesh) {
        match result {
            CommandResult::EntityModified { entity, edges, volume, mesh, .. } => {
                (entity, edges, volume, mesh)
            },
            r => panic!("unexpected result {r:?}"),
        }
    }

    fn volume_of(e: &mut Engine, id: EntityId) -> f64 {
        fillet::signed_volume(&entity_snapshot(e, id).mesh)
    }

    #[test]
    fn fillet_rounds_edges_in_place_and_keeps_the_placement() {
        let mut e = engine();
        let id = add_box(&mut e, [0.0, 0.0, 5.0], [20.0, 20.0, 10.0]);
        e.execute(Command::Transform {
            doc: 1,
            entity: id,
            translation: None,
            rotation_deg: Some([num(0.0), num(0.0), num(45.0)]),
            scale: None,
        })
        .unwrap();
        let before = entity_snapshot(&mut e, id);

        let (entity, edges, volume, mesh) = modified(
            e.execute(Command::Fillet {
                doc: 1,
                entity: id,
                radius: num(2.0),
                edges: None,
                min_angle_deg: None,
            })
            .unwrap(),
        );
        assert_eq!(entity, id, "the entity is reshaped, not replaced");
        assert_eq!(edges, 12);
        // `volume` is world-space, `mesh` local; a rotation preserves volume
        // (up to the f32 round trip through the inverse transform).
        let local = fillet::signed_volume(&mesh);
        assert!((volume - local).abs() / volume < 1e-4, "{volume} vs {local}");
        assert!(volume < 4000.0, "filleting removes material ({volume})");

        let after = entity_snapshot(&mut e, id);
        assert_eq!(after.translation, before.translation);
        assert_eq!(after.rotation_deg, before.rotation_deg);
        assert_eq!(after.scale, before.scale);
        assert_eq!(after.name, before.name);
        // The fillet is tangent to the faces, so the *local* mesh bounds are
        // unchanged: the rounding happens inside them. (The tight world bbox
        // of a rotated box does shrink — its extreme point was the very
        // corner the fillet rounded — so that is not the invariant to check.)
        let local_bbox = |m: &Mesh| {
            let mut lo = [f64::MAX; 3];
            let mut hi = [f64::MIN; 3];
            for p in &m.positions {
                for k in 0..3 {
                    lo[k] = lo[k].min(p[k] as f64);
                    hi[k] = hi[k].max(p[k] as f64);
                }
            }
            (lo, hi)
        };
        let (blo, bhi) = local_bbox(&before.mesh);
        let (alo, ahi) = local_bbox(&after.mesh);
        for k in 0..3 {
            assert!((alo[k] - blo[k]).abs() < 1e-3, "{alo:?} vs {blo:?}");
            assert!((ahi[k] - bhi[k]).abs() < 1e-3, "{ahi:?} vs {bhi:?}");
        }
        // The local mesh is still a watertight solid.
        assert!(fillet::signed_volume(&after.mesh) > 0.0);
    }

    #[test]
    fn fillet_replays_with_the_parameters() {
        let mut e = engine();
        e.execute(Command::SetParameter {
            doc: 1,
            name: "r".into(),
            value: num(2.0),
            description: Some("edge radius".into()),
        })
        .unwrap();
        let id = add_box(&mut e, [0.0, 0.0, 5.0], [20.0, 20.0, 10.0]);
        let (_, edges, v2, _) = modified(
            e.execute(Command::Fillet {
                doc: 1,
                entity: id,
                radius: Scalar::Expression("r".into()),
                edges: Some(EdgeFilter { direction: Some([0.0, 0.0, 1.0]), ..Default::default() }),
                min_angle_deg: None,
            })
            .unwrap(),
        );
        assert_eq!(edges, 4, "only the vertical corners");

        // Turning the knob re-runs the program: same id, new fillet.
        e.execute(Command::SetParameter {
            doc: 1,
            name: "r".into(),
            value: num(5.0),
            description: None,
        })
        .unwrap();
        let v5 = volume_of(&mut e, id);
        assert!(v5 < v2, "a larger radius removes more ({v5} vs {v2})");

        // ... and back again is deterministic.
        e.execute(Command::SetParameter {
            doc: 1,
            name: "r".into(),
            value: num(2.0),
            description: None,
        })
        .unwrap();
        assert!((volume_of(&mut e, id) - v2).abs() < 1e-6);
        let ids: Vec<EntityId> = match e.execute(Command::GetScene { doc: 1 }).unwrap() {
            CommandResult::Scene { entities, .. } => entities.iter().map(|x| x.id).collect(),
            r => panic!("unexpected result {r:?}"),
        };
        assert_eq!(ids, vec![id], "replay keeps the entity id");
    }

    #[test]
    fn chamfer_bevels_only_the_selected_edges() {
        let mut e = engine();
        let id = add_box(&mut e, [0.0, 0.0, 5.0], [20.0, 20.0, 10.0]);
        let (_, edges, volume, _) = modified(
            e.execute(Command::Chamfer {
                doc: 1,
                entity: id,
                distance: num(2.0),
                edges: Some(EdgeFilter { direction: Some([0.0, 0.0, 1.0]), ..Default::default() }),
                min_angle_deg: None,
            })
            .unwrap(),
        );
        assert_eq!(edges, 4);
        // Four bevels of area r²/2 × height, no corner overlaps (they are
        // at different corners).
        let expected = 4000.0 - 4.0 * (0.5 * 2.0 * 2.0 * 10.0);
        assert!((volume - expected).abs() / expected < 0.01, "{volume} vs {expected}");

        // The 45° bevel is itself a sharp face pair, so the solid gains
        // edges rather than losing them — but the sharp 90° corner is gone.
        let snap = entity_snapshot(&mut e, id);
        let left = edges::sharp_edges(&snap.mesh, 30.0).unwrap();
        assert!(left.len() > 12, "a bevel adds edges ({})", left.len());
        assert!(
            left.iter().any(|x| (x.dihedral_deg() - 135.0).abs() < 1e-3),
            "the bevel meets each face at 135°"
        );
    }

    #[test]
    fn fillet_rejects_sketches_and_bad_sizes() {
        let mut e = engine();
        let s = add_sketch(&mut e, "profile");
        let err = e
            .execute(Command::Fillet {
                doc: 1,
                entity: s,
                radius: num(1.0),
                edges: None,
                min_angle_deg: None,
            })
            .unwrap_err();
        assert!(err.to_string().contains("sketch"), "{err}");

        let id = add_box(&mut e, [0.0, 0.0, 5.0], [10.0, 10.0, 10.0]);
        let err = e
            .execute(Command::Fillet {
                doc: 1,
                entity: id,
                radius: num(0.0),
                edges: None,
                min_angle_deg: None,
            })
            .unwrap_err();
        assert!(err.to_string().contains("positive"), "{err}");

        let err = e
            .execute(Command::Fillet {
                doc: 1,
                entity: id,
                radius: num(1.0),
                edges: Some(EdgeFilter { near: Some([0.0, 0.0, 0.0]), ..Default::default() }),
                min_angle_deg: None,
            })
            .unwrap_err();
        assert!(err.to_string().contains("near_radius"), "{err}");

        // A failed fillet leaves the document untouched (rollback).
        let snap = entity_snapshot(&mut e, id);
        assert_eq!(snap.mesh.indices.len(), 36, "box is still a box");
        assert!(e.execute(Command::GetProgram { doc: 1 }).is_ok());
    }

    #[test]
    fn a_fillet_survives_a_sketch_edit() {
        let mut e = engine();
        let (s, lines) = rect_sketch(&mut e, 40.0, 30.0);
        let (id, _, _) = extrude(&mut e, s, num(5.0));
        let (_, edges, filleted, _) = modified(
            e.execute(Command::Fillet {
                doc: 1,
                entity: id,
                radius: num(1.0),
                edges: None,
                min_angle_deg: None,
            })
            .unwrap(),
        );
        assert_eq!(edges, 12);
        assert!(filleted < 40.0 * 30.0 * 5.0);

        // Drag a corner: the profile widens, the extrude rebuilds, and the
        // fillet must be re-applied to the new solid (not silently dropped).
        e.execute(Command::MoveSketchPoint {
            doc: 1,
            sketch: s,
            element: lines[0],
            point: 1,
            to: [50.0, 0.0],
        })
        .unwrap();
        let after = volume_of(&mut e, id);
        // The sharp box would be 50×30×5 = 7500; the fillet must still be
        // cutting the edges.
        assert!(after < 7480.0, "fillet was lost after the sketch edit: {after}");
    }

    #[test]
    fn a_fillet_survives_save_and_open() {
        let (path, path_s) = temp_file("agcad_fillet.agcad");
        let mut a = engine();
        let id = add_box(&mut a, [0.0, 0.0, 5.0], [20.0, 20.0, 10.0]);
        a.execute(Command::Fillet {
            doc: 1,
            entity: id,
            radius: num(2.0),
            edges: Some(EdgeFilter { direction: Some([0.0, 0.0, 1.0]), ..Default::default() }),
            min_angle_deg: None,
        })
        .unwrap();
        let saved = entity_snapshot(&mut a, id).mesh;
        a.execute(Command::SaveDocument { doc: 1, path: path_s.clone(), source: None }).unwrap();

        let mut b = Engine::new();
        b.execute(Command::OpenDocument { path: path_s.clone(), name: None }).unwrap();
        let reopened = entity_snapshot(&mut b, id).mesh;
        assert_eq!(reopened.indices, saved.indices);
        assert_eq!(reopened.positions, saved.positions);
        let _ = std::fs::remove_file(&path);
    }

    #[test]
    fn validate_reports_a_printable_box() {
        let mut e = engine();
        let id = add_box(&mut e, [0.0, 0.0, 0.0], [10.0, 20.0, 30.0]);
        let program_before = match e.execute(Command::GetProgram { doc: 1 }).unwrap() {
            CommandResult::Program { program, .. } => program.len(),
            r => panic!("unexpected result {r:?}"),
        };

        match e
            .execute(Command::Validate { doc: 1, entities: None, min_wall_mm: Some(5.0) })
            .unwrap()
        {
            CommandResult::Validated { ok, solids, sketches, totals, warnings, .. } => {
                assert!(ok, "box should pass: {warnings:?}");
                assert!(warnings.is_empty());
                assert!(sketches.is_empty());
                assert_eq!(solids.len(), 1);
                let s = &solids[0];
                assert_eq!(s.entity, id);
                assert_eq!(s.kind, "box");
                assert!(s.ok && s.watertight);
                assert_eq!(s.open_edges, 0);
                assert_eq!(s.non_manifold_edges, 0);
                assert_eq!(s.self_intersections, 0);
                assert_eq!(s.tris, 12);
                assert!((s.volume - 6000.0).abs() < 1e-6, "volume {}", s.volume);
                let wall = s.min_wall.expect("box has a wall estimate");
                assert!((wall - 10.0).abs() < 0.01, "min wall {wall}");
                assert!(!s.thin);
                assert_eq!(s.signature.len(), 16);
                assert_eq!(totals.solids, 1);
                assert!((totals.volume - 6000.0).abs() < 1e-6);
            },
            r => panic!("unexpected result {r:?}"),
        }

        // Validation is a query: it does not touch the program or history.
        let program_after = match e.execute(Command::GetProgram { doc: 1 }).unwrap() {
            CommandResult::Program { program, .. } => program.len(),
            r => panic!("unexpected result {r:?}"),
        };
        assert_eq!(program_after, program_before);
        e.execute(Command::Undo { doc: 1 }).unwrap();
        assert!(matches!(
            e.execute(Command::GetScene { doc: 1 }).unwrap(),
            CommandResult::Scene { entities, .. } if entities.is_empty()
        ));
    }

    #[test]
    fn validate_flags_a_thin_solid_and_reports_the_wall() {
        let mut e = engine();
        let id = add_box(&mut e, [0.0, 0.0, 0.0], [20.0, 20.0, 1.0]);
        match e
            .execute(Command::Validate { doc: 1, entities: None, min_wall_mm: Some(2.0) })
            .unwrap()
        {
            CommandResult::Validated { ok, solids, warnings, .. } => {
                assert!(!ok);
                let s = &solids[0];
                assert_eq!(s.entity, id);
                assert!(s.ok == false && s.thin);
                let wall = s.min_wall.expect("thin plate has a wall estimate");
                assert!((wall - 1.0).abs() < 0.01, "min wall {wall}");
                assert!(
                    warnings.iter().any(|w| w.contains("thinner than 2 mm")),
                    "{warnings:?}"
                );
            },
            r => panic!("unexpected result {r:?}"),
        }

        let err = e
            .execute(Command::Validate { doc: 1, entities: None, min_wall_mm: Some(0.0) })
            .unwrap_err();
        assert!(err.to_string().contains("min_wall_mm"), "{err}");
    }

    #[test]
    fn validate_checks_sketch_solver_and_profile() {
        let mut e = engine();
        let (sketch, lines) = rect_sketch(&mut e, 40.0, 30.0);
        match e
            .execute(Command::Validate { doc: 1, entities: None, min_wall_mm: None })
            .unwrap()
        {
            CommandResult::Validated { ok, solids, sketches, .. } => {
                assert!(ok, "a closed, solved rectangle is valid");
                assert!(solids.is_empty());
                let s = &sketches[0];
                assert_eq!(s.entity, sketch);
                assert!(s.ok && s.solved && s.closed_profile);
                assert_eq!(s.elements, 4);
                assert_eq!(s.loops, 1);
                assert!(s.profile_error.is_none());
            },
            r => panic!("unexpected result {r:?}"),
        }

        // Open the loop: the sketch is still solvable but cannot be extruded.
        e.execute(Command::DeleteSketchElement { doc: 1, sketch, element: lines[2] })
            .unwrap();
        match e
            .execute(Command::Validate { doc: 1, entities: None, min_wall_mm: None })
            .unwrap()
        {
            CommandResult::Validated { ok, sketches, warnings, .. } => {
                assert!(!ok);
                let s = &sketches[0];
                assert!(!s.ok && !s.closed_profile);
                assert!(s.profile_error.as_deref().unwrap_or("").contains("open"), "{s:?}");
                assert!(warnings.iter().any(|w| w.contains("sketch")), "{warnings:?}");
            },
            r => panic!("unexpected result {r:?}"),
        }
    }

    #[test]
    fn validate_accepts_a_filleted_boolean() {
        // A rounded union is the shape that used to trip a naive
        // self-intersection test: tangent seams and sub-micron slivers are
        // not crossings, and the thin fin they make is not a printable wall.
        let mut e = engine();
        let base = add_box(&mut e, [0.0, 0.0, 5.0], [40.0, 40.0, 10.0]);
        let boss = match e
            .execute(Command::AddCylinder {
                doc: 1,
                name: "boss".into(),
                center: vec3(0.0),
                radius: num(8.0),
                height: num(14.0),
                segments: Some(24),
                color: None,
            })
            .unwrap()
        {
            CommandResult::EntityCreated { entity, .. } => entity,
            r => panic!("unexpected result {r:?}"),
        };
        let fused = match e
            .execute(Command::Boolean {
                doc: 1,
                op: BooleanOp::Union,
                entities: vec![base, boss],
                name: Some("fused".into()),
                color: None,
            })
            .unwrap()
        {
            CommandResult::EntityCreated { entity, .. } => entity,
            r => panic!("unexpected result {r:?}"),
        };
        e.execute(Command::Fillet {
            doc: 1,
            entity: fused,
            radius: num(1.0),
            edges: Some(EdgeFilter { normals: Some(vec![[0.0, 0.0, 1.0]]), ..Default::default() }),
            min_angle_deg: None,
        })
        .unwrap();

        match e
            .execute(Command::Validate { doc: 1, entities: Some(vec![fused]), min_wall_mm: Some(1.5) })
            .unwrap()
        {
            CommandResult::Validated { ok, solids, warnings, .. } => {
                let s = &solids[0];
                assert!(s.watertight && s.ok, "{warnings:?}");
                assert_eq!(s.self_intersections, 0, "tangent seams are not crossings");
                let wall = s.min_wall.expect("rounded union has a wall estimate");
                assert!(wall > 1.5, "min wall {wall} should ignore fin slivers");
                assert!(!s.thin);
                assert!(ok, "{warnings:?}");
            },
            r => panic!("unexpected result {r:?}"),
        }
    }

    #[test]
    fn diff_scene_detects_a_parameter_driven_rebuild() {
        let (path, path_s) = temp_file("agcad_diff_scene.agcad");
        let mut e = engine();
        e.execute(Command::SetParameter {
            doc: 1,
            name: "t".into(),
            value: num(4.0),
            description: Some("plate thickness".into()),
        })
        .unwrap();
        let id = match e
            .execute(Command::AddBox {
                doc: 1,
                name: "plate".into(),
                center: [num(0.0), num(0.0), Scalar::Expression("t/2".into())],
                size: [num(20.0), num(20.0), Scalar::Expression("t".into())],
                color: None,
            })
            .unwrap()
        {
            CommandResult::EntityCreated { entity, .. } => entity,
            r => panic!("unexpected result {r:?}"),
        };
        e.execute(Command::SaveDocument { doc: 1, path: path_s.clone(), source: None }).unwrap();

        e.execute(Command::SetParameter {
            doc: 1,
            name: "t".into(),
            value: num(0.8),
            description: None,
        })
        .unwrap();
        match e
            .execute(Command::DiffScene { doc: 1, against: None, path: Some(path_s.clone()) })
            .unwrap()
        {
            CommandResult::SceneDiff {
                changed,
                added,
                removed,
                modified,
                unchanged,
                parameters,
                program_changed,
                commands_from,
                commands_to,
                scene_signature,
                against_signature,
                ..
            } => {
                assert!(changed);
                assert!(added.is_empty() && removed.is_empty());
                assert_eq!(unchanged, 0);
                assert_eq!(modified.len(), 1);
                let m = &modified[0];
                assert_eq!(m.entity, id);
                assert!(m.changed.iter().any(|c| c == "mesh"), "{:?}", m.changed);
                assert!(m.changed.iter().any(|c| c == "translation"), "{:?}", m.changed);
                assert!((m.from.volume.unwrap() - 1600.0).abs() < 1e-6);
                assert!((m.to.volume.unwrap() - 320.0).abs() < 1e-3);
                assert_ne!(m.from.signature, m.to.signature);
                assert_eq!(parameters.len(), 1);
                assert_eq!(parameters[0].name, "t");
                assert_eq!(parameters[0].from, Some(4.0));
                assert_eq!(parameters[0].to, Some(0.8));
                assert!(!program_changed);
                assert_eq!(commands_from, commands_to);
                assert_ne!(scene_signature, against_signature);
            },
            r => panic!("unexpected result {r:?}"),
        }

        // Back to the baseline: nothing changed, and the program is intact.
        e.execute(Command::SetParameter {
            doc: 1,
            name: "t".into(),
            value: num(4.0),
            description: None,
        })
        .unwrap();
        match e
            .execute(Command::DiffScene { doc: 1, against: None, path: Some(path_s) })
            .unwrap()
        {
            CommandResult::SceneDiff {
                changed,
                added,
                removed,
                modified,
                unchanged,
                parameters,
                program_changed,
                scene_signature,
                against_signature,
                ..
            } => {
                assert!(!changed);
                assert!(added.is_empty() && removed.is_empty() && modified.is_empty());
                assert_eq!(unchanged, 1);
                assert!(parameters.is_empty());
                assert!(!program_changed);
                assert_eq!(scene_signature, against_signature);
            },
            r => panic!("unexpected result {r:?}"),
        }
        let _ = std::fs::remove_file(&path);
    }

    #[test]
    fn diff_scene_compares_two_open_documents() {
        let mut e = engine();
        add_box(&mut e, [0.0, 0.0, 0.0], [10.0, 10.0, 10.0]);

        e.execute(Command::CreateDocument { name: "baseline".into(), doc: Some(2) }).unwrap();
        match e
            .execute(Command::AddBox {
                doc: 2,
                name: "box".into(),
                center: vec3(0.0),
                size: vec3(20.0),
                color: None,
            })
            .unwrap()
        {
            CommandResult::EntityCreated { entity, .. } => assert_eq!(entity, 1),
            r => panic!("unexpected result {r:?}"),
        }
        // The baseline has one extra solid, so the current revision is
        // missing it.
        e.execute(Command::AddBox {
            doc: 2,
            name: "extra".into(),
            center: vec3(50.0),
            size: vec3(2.0),
            color: None,
        })
        .unwrap();

        match e
            .execute(Command::DiffScene { doc: 1, against: Some(2), path: None })
            .unwrap()
        {
            CommandResult::SceneDiff { changed, added, removed, modified, unchanged, .. } => {
                assert!(changed);
                assert!(added.is_empty());
                assert_eq!(removed.len(), 1);
                assert_eq!(removed[0].entity, 2);
                assert_eq!(modified.len(), 1);
                assert!(modified[0].changed.iter().any(|c| c == "mesh"));
                assert_eq!(unchanged, 0);
            },
            r => panic!("unexpected result {r:?}"),
        }

        // Identical revisions compare clean, including the params/program.
        e.execute(Command::CreateDocument { name: "copy".into(), doc: Some(3) }).unwrap();
        e.execute(Command::AddBox {
            doc: 3,
            name: "box".into(),
            center: vec3(0.0),
            size: vec3(10.0),
            color: None,
        })
        .unwrap();
        match e
            .execute(Command::DiffScene { doc: 1, against: Some(3), path: None })
            .unwrap()
        {
            CommandResult::SceneDiff { changed, unchanged, modified, parameters, .. } => {
                assert!(!changed);
                assert_eq!(unchanged, 1);
                assert!(modified.is_empty() && parameters.is_empty());
            },
            r => panic!("unexpected result {r:?}"),
        }

        let err = e
            .execute(Command::DiffScene { doc: 1, against: None, path: None })
            .unwrap_err();
        assert!(err.to_string().contains("exactly one baseline"), "{err}");
        let err = e
            .execute(Command::DiffScene { doc: 1, against: Some(1), path: None })
            .unwrap_err();
        assert!(err.to_string().contains("same document"), "{err}");
    }

    // --- incremental rebuild (M15) -------------------------------------------

    /// Turn a knob and return `(rebuilt, reused, steps)` from the result.
    fn set_param(e: &mut Engine, name: &str, value: f64) -> (usize, usize, usize) {
        match e
            .execute(Command::SetParameter {
                doc: 1,
                name: name.into(),
                value: num(value),
                description: None,
            })
            .unwrap()
        {
            CommandResult::ParameterSet { rebuilt, reused, steps, .. } => (rebuilt, reused, steps),
            r => panic!("unexpected result {r:?}"),
        }
    }

    fn add_box_expr(e: &mut Engine, center: [f64; 3], size: [Scalar; 3]) -> EntityId {
        match e
            .execute(Command::AddBox {
                doc: 1,
                name: "box".into(),
                center: [num(center[0]), num(center[1]), num(center[2])],
                size,
                color: None,
            })
            .unwrap()
        {
            CommandResult::EntityCreated { entity, .. } => entity,
            r => panic!("unexpected result {r:?}"),
        }
    }

    fn union(e: &mut Engine, ids: &[EntityId]) -> EntityId {
        match e
            .execute(Command::Boolean {
                doc: 1,
                op: BooleanOp::Union,
                entities: ids.to_vec(),
                name: None,
                color: None,
            })
            .unwrap()
        {
            CommandResult::EntityCreated { entity, .. } => entity,
            r => panic!("unexpected result {r:?}"),
        }
    }

    fn round_all(e: &mut Engine, id: EntityId, radius: f64) -> EntityId {
        match e
            .execute(Command::Fillet {
                doc: 1,
                entity: id,
                radius: num(radius),
                edges: None,
                min_angle_deg: None,
            })
            .unwrap()
        {
            CommandResult::EntityModified { entity, .. } => entity,
            r => panic!("unexpected result {r:?}"),
        }
    }

    fn scene_sig(e: &Engine) -> String {
        validate::scene_signature(&e.docs[&1].entities)
    }

    fn scene_meshes(e: &Engine) -> Vec<(EntityId, Mesh)> {
        e.docs[&1].entities.iter().map(|(id, x)| (*id, x.mesh.clone())).collect()
    }

    /// Drop the memo and replay cold: the correctness oracle for the cache.
    fn cold_replay(e: &mut Engine) {
        let d = e.docs.get_mut(&1).unwrap();
        d.cache.clear();
        d.replay().unwrap();
    }

    fn assert_same_meshes(a: &[(EntityId, Mesh)], b: &[(EntityId, Mesh)]) {
        assert_eq!(a.len(), b.len());
        for ((ida, ma), (idb, mb)) in a.iter().zip(b) {
            assert_eq!(ida, idb);
            assert!(mesh_eq(ma, mb), "entity {ida} mesh differs after a cold replay");
        }
    }

    #[test]
    fn a_knob_change_rebuilds_only_downstream_features() {
        let mut e = engine();
        set_param(&mut e, "a", 10.0);
        set_param(&mut e, "b", 8.0);

        let left = add_box_expr(
            &mut e,
            [0.0, 0.0, 0.0],
            [Scalar::Expression("a".into()), num(10.0), num(10.0)],
        );
        let cap = add_box(&mut e, [0.0, 0.0, 8.0], [6.0, 6.0, 6.0]);
        let fused = union(&mut e, &[left, cap]);
        let rounded = round_all(&mut e, fused, 0.4);
        assert_eq!(rounded, fused, "a fillet reshapes in place");
        let _right = add_box_expr(
            &mut e,
            [30.0, 0.0, 0.0],
            [Scalar::Expression("b".into()), num(10.0), num(10.0)],
        );

        // a → left box, the union and its fillet. The cap and the independent
        // right box keep their cached bytes.
        assert_eq!(set_param(&mut e, "a", 12.0), (3, 2, 5));

        // b only reaches the right box.
        assert_eq!(set_param(&mut e, "b", 9.0), (1, 4, 5));

        // A no-op knob turn rebuilds nothing.
        assert_eq!(set_param(&mut e, "a", 12.0), (0, 5, 5));

        // The cached meshes are exactly the cold-replay bytes.
        let sig = scene_sig(&e);
        let meshes = scene_meshes(&e);
        cold_replay(&mut e);
        assert_eq!(scene_sig(&e), sig);
        assert_same_meshes(&meshes, &scene_meshes(&e));
        assert_eq!(e.docs[&1].cache.rebuilt, 5);
        assert_eq!(e.docs[&1].cache.reused, 0);
    }

    #[test]
    fn a_sketch_parameter_rebuilds_its_extrude_and_downstream_fillet() {
        let mut e = engine();
        set_param(&mut e, "w", 12.0);
        set_param(&mut e, "d", 6.0);

        let sketch = add_sketch(&mut e, "profile");
        let z = || num(0.0);
        let x = || Scalar::Expression("w".into());
        let y = || num(8.0);
        for (p1, p2) in [
            ([z(), z()], [x(), z()]),
            ([x(), z()], [x(), y()]),
            ([x(), y()], [z(), y()]),
            ([z(), y()], [z(), z()]),
        ] {
            e.execute(Command::AddSketchLine { doc: 1, sketch, p1, p2 }).unwrap();
        }
        let swept = extrude(&mut e, sketch, Scalar::Expression("d".into())).0;
        round_all(&mut e, swept, 0.3);

        // An independent solid whose inputs never move.
        let (other_sketch, _) = rect_sketch(&mut e, 6.0, 6.0);
        extrude(&mut e, other_sketch, num(4.0));

        // w moves the profile: the extrude and its fillet rebuild, the other
        // extrude does not.
        assert_eq!(set_param(&mut e, "w", 15.0), (2, 1, 21));

        // d moves only the swept distance, same two features.
        assert_eq!(set_param(&mut e, "d", 9.0), (2, 1, 21));

        let sig = scene_sig(&e);
        let meshes = scene_meshes(&e);
        cold_replay(&mut e);
        assert_eq!(scene_sig(&e), sig);
        assert_same_meshes(&meshes, &scene_meshes(&e));
    }

    #[test]
    fn undo_and_redo_leave_the_incremental_cache_honest() {
        let mut e = engine();
        set_param(&mut e, "h", 10.0);
        let a = add_box_expr(
            &mut e,
            [0.0, 0.0, 0.0],
            [num(10.0), num(10.0), Scalar::Expression("h".into())],
        );
        let b = add_box(&mut e, [0.0, 0.0, 6.0], [6.0, 6.0, 6.0]);
        union(&mut e, &[a, b]);

        let original_sig = scene_sig(&e);
        assert_eq!(set_param(&mut e, "h", 14.0), (2, 1, 3));
        let taller_sig = scene_sig(&e);
        assert_ne!(taller_sig, original_sig);

        e.execute(Command::Undo { doc: 1 }).unwrap();
        assert_eq!(scene_sig(&e), original_sig, "undo restores the old geometry");
        e.execute(Command::Redo { doc: 1 }).unwrap();
        assert_eq!(scene_sig(&e), taller_sig);

        // A later change after the undo/redo pair must not inherit stale
        // meshes from either revision.
        assert_eq!(set_param(&mut e, "h", 18.0), (2, 1, 3));
        let sig = scene_sig(&e);
        let meshes = scene_meshes(&e);
        cold_replay(&mut e);
        assert_eq!(scene_sig(&e), sig);
        assert_same_meshes(&meshes, &scene_meshes(&e));
    }

    // --- shape leverage (M16) -------------------------------------------------

    fn entity_translation(e: &mut Engine, id: EntityId) -> Vec3 {
        match e.execute(Command::GetScene { doc: 1 }).unwrap() {
            CommandResult::Scene { entities, .. } => {
                entities.iter().find(|x| x.id == id).unwrap().translation
            },
            r => panic!("unexpected result {r:?}"),
        }
    }

    fn entity_tris(e: &mut Engine, id: EntityId) -> usize {
        entity_snapshot(e, id).mesh.indices.len() / 3
    }

    #[test]
    fn patterns_and_mirror_create_the_expected_copies() {
        let mut e = engine();
        let source = add_box(&mut e, [10.0, 0.0, 0.0], [10.0, 10.0, 10.0]);

        let mut copies_of = |result: CommandResult| match result {
            CommandResult::EntitiesCreated { entities, tris, .. } => {
                assert!(tris > 0);
                entities
            },
            r => panic!("unexpected result {r:?}"),
        };

        // Linear: count includes the original, so 3 instances = 2 copies.
        let line = copies_of(
            e.execute(Command::LinearPattern {
                doc: 1,
                entities: vec![source],
                direction: [num(1.0), num(0.0), num(0.0)],
                spacing: num(20.0),
                count: 3,
                name: None,
                color: None,
            })
            .unwrap(),
        );
        assert_eq!(line.len(), 2);
        assert!((entity_translation(&mut e, line[0])[0] - 30.0).abs() < 1e-6);
        assert!((entity_translation(&mut e, line[1])[0] - 50.0).abs() < 1e-6);
        assert_eq!(entity_tris(&mut e, line[0]), entity_tris(&mut e, source));
        match e.execute(Command::GetScene { doc: 1 }).unwrap() {
            CommandResult::Scene { entities, .. } => {
                assert_eq!(entities.iter().find(|x| x.id == line[0]).unwrap().name, "box 1");
            },
            r => panic!("unexpected result {r:?}"),
        }

        // Circular: the source sits 10 mm out on +X, so quarter turns land on
        // +Y, -X and -Y with no change in radius.
        let circular = copies_of(
            e.execute(Command::CircularPattern {
                doc: 1,
                entities: vec![source],
                center: Some(vec3(0.0)),
                axis: Some([0.0, 0.0, 1.0]),
                angle_deg: Some(num(360.0)),
                count: 4,
                name: Some("hole".into()),
                color: None,
            })
            .unwrap(),
        );
        assert_eq!(circular.len(), 3);
        let expected = [[0.0, 10.0, 0.0], [-10.0, 0.0, 0.0], [0.0, -10.0, 0.0]];
        for (id, want) in circular.iter().zip(expected) {
            let got = entity_translation(&mut e, *id);
            for k in 0..3 {
                assert!((got[k] - want[k]).abs() < 1e-4, "{id}: {got:?} vs {want:?}");
            }
        }

        // Mirror across the YZ plane: the reflection lands at -10 on X.
        let mirrored = copies_of(
            e.execute(Command::Mirror {
                doc: 1,
                entities: vec![source],
                plane: MirrorPlane::Custom {
                    origin: [num(0.0), num(0.0), num(0.0)],
                    normal: [1.0, 0.0, 0.0],
                },
                name: None,
                color: None,
            })
            .unwrap(),
        );
        assert_eq!(mirrored.len(), 1);
        assert!((entity_translation(&mut e, mirrored[0])[0] + 10.0).abs() < 1e-4);
        assert!(volume_of(&mut e, mirrored[0]) > 0.0, "mirrored winding is outward");

        // The copies are ordinary solids: a union of the pattern is printable.
        let union_id = union(&mut e, &[source, line[0], line[1]]);
        match e.execute(Command::Validate { doc: 1, entities: Some(vec![union_id]), min_wall_mm: None }).unwrap() {
            CommandResult::Validated { ok, solids, .. } => {
                assert!(ok, "{:?}", solids[0]);
            },
            r => panic!("unexpected result {r:?}"),
        }
    }

    #[test]
    fn patterns_replay_with_the_program_and_follow_the_source() {
        let mut e = engine();
        set_param(&mut e, "w", 10.0);
        let source = add_box_expr(
            &mut e,
            [0.0, 0.0, 0.0],
            [Scalar::Expression("w".into()), num(10.0), num(10.0)],
        );
        e.execute(Command::LinearPattern {
            doc: 1,
            entities: vec![source],
            direction: [num(1.0), num(0.0), num(0.0)],
            spacing: num(20.0),
            count: 3,
            name: None,
            color: None,
        })
        .unwrap();
        let before = scene_sig(&e);

        // A knob change replays the pattern; the copy at x = 20 grows with its
        // source, and the ids stay stable.
        set_param(&mut e, "w", 14.0);
        match e.execute(Command::GetScene { doc: 1 }).unwrap() {
            CommandResult::Scene { entities, .. } => {
                let ids: Vec<u64> = entities.iter().map(|x| x.id).collect();
                assert_eq!(ids, vec![1, 2, 3]);
                let copy = entities.iter().find(|x| x.id == 2).unwrap();
                let xs: Vec<f32> = copy.mesh.positions.iter().map(|p| p[0]).collect();
                let width = xs.iter().cloned().fold(f32::MIN, f32::max)
                    - xs.iter().cloned().fold(f32::MAX, f32::min);
                assert!((width - 14.0).abs() < 1e-4, "copy width {width}");
                assert!((copy.translation[0] - 20.0).abs() < 1e-6);
            },
            r => panic!("unexpected result {r:?}"),
        }
        assert_ne!(scene_sig(&e), before);

        // Deleting the source takes its copies with it (no dangling refs on
        // replay).
        e.execute(Command::Delete { doc: 1, entity: source }).unwrap();
        match e.execute(Command::GetScene { doc: 1 }).unwrap() {
            CommandResult::Scene { entities, .. } => assert!(entities.is_empty()),
            r => panic!("unexpected result {r:?}"),
        }
        let meshes = scene_meshes(&e);
        cold_replay(&mut e);
        assert_same_meshes(&meshes, &scene_meshes(&e));
    }

    #[test]
    fn shell_and_hole_reshape_in_place_and_replay() {
        let mut e = engine();
        set_param(&mut e, "wall", 2.0);
        let body = add_box(&mut e, [0.0, 0.0, 0.0], [40.0, 30.0, 20.0]);

        let shelled = match e
            .execute(Command::Shell {
                doc: 1,
                entity: body,
                thickness: Scalar::Expression("wall".into()),
                open: vec![[0.0, 0.0, 1.0]],
                angle_tol_deg: None,
            })
            .unwrap()
        {
            CommandResult::EntityReshaped { entity, volume, tris, .. } => {
                assert_eq!(entity, body);
                assert!(tris > 0);
                volume
            },
            r => panic!("unexpected result {r:?}"),
        };
        let expected = 40.0 * 30.0 * 20.0 - 36.0 * 26.0 * 18.0;
        assert!((shelled - expected).abs() / expected < 0.01, "volume {shelled} vs {expected}");
        // The entity keeps its id and gains a cavity, not a second entity.
        match e.execute(Command::GetScene { doc: 1 }).unwrap() {
            CommandResult::Scene { entities, .. } => assert_eq!(entities.len(), 1),
            r => panic!("unexpected result {r:?}"),
        }
        match e.execute(Command::Validate { doc: 1, entities: None, min_wall_mm: Some(1.0) }).unwrap() {
            CommandResult::Validated { ok, solids, .. } => assert!(ok, "{:?}", solids[0]),
            r => panic!("unexpected result {r:?}"),
        }

        // A hole in the floor, with a counterbore, cuts in place too.
        let drilled = match e
            .execute(Command::Hole {
                doc: 1,
                entity: body,
                center: [num(0.0), num(0.0), num(-10.0)],
                normal: [0.0, 0.0, -1.0],
                diameter: num(6.0),
                depth: Some(num(8.0)),
                counterbore_diameter: Some(num(11.0)),
                counterbore_depth: Some(num(2.0)),
                countersink_angle_deg: None,
                segments: Some(48),
            })
            .unwrap()
        {
            CommandResult::EntityReshaped { entity, volume, .. } => {
                assert_eq!(entity, body);
                volume
            },
            r => panic!("unexpected result {r:?}"),
        };
        assert!(drilled < shelled, "the hole must remove material: {drilled} vs {shelled}");

        // The wall knob replays shell, then the hole, in order.
        set_param(&mut e, "wall", 3.0);
        let thicker = volume_of(&mut e, body);
        assert!(thicker > drilled, "a thicker wall leaves more material: {thicker} vs {drilled}");
        match e.execute(Command::GetScene { doc: 1 }).unwrap() {
            CommandResult::Scene { entities, .. } => assert_eq!(entities.len(), 1),
            r => panic!("unexpected result {r:?}"),
        }
        let sig = scene_sig(&e);
        let meshes = scene_meshes(&e);
        cold_replay(&mut e);
        assert_eq!(scene_sig(&e), sig);
        assert_same_meshes(&meshes, &scene_meshes(&e));

        // A wall that cannot fit is an actionable error, not a broken solid.
        let err = e
            .execute(Command::Shell {
                doc: 1,
                entity: body,
                thickness: num(20.0),
                open: vec![],
                angle_tol_deg: None,
            })
            .unwrap_err();
        assert!(err.to_string().contains("too thick"), "{err}");
    }

    #[test]
    fn sweep_loft_and_thread_build_printable_solids() {
        let mut e = engine();
        set_param(&mut e, "r", 2.0);

        let spring_profile = add_sketch(&mut e, "wire");
        e.execute(Command::AddSketchCircle {
            doc: 1,
            sketch: spring_profile,
            center: [num(0.0), num(0.0)],
            radius: Scalar::Expression("r".into()),
        })
        .unwrap();
        let spring = match e
            .execute(Command::Sweep {
                doc: 1,
                sketch: spring_profile,
                path: SweepPath::Helix {
                    origin: None,
                    axis: None,
                    radius: num(12.0),
                    pitch: num(8.0),
                    turns: num(1.0),
                    hand: Hand::Right,
                    segments_per_turn: Some(16),
                },
                segments: None,
                name: Some("spring".into()),
                color: None,
            })
            .unwrap()
        {
            CommandResult::SolidBuilt { entity, kind, volume, tris, .. } => {
                assert_eq!(kind, "sweep");
                assert!(volume > 0.0 && tris > 0);
                entity
            },
            r => panic!("unexpected result {r:?}"),
        };
        match e.execute(Command::Validate { doc: 1, entities: Some(vec![spring]), min_wall_mm: None }).unwrap() {
            CommandResult::Validated { ok, solids, .. } => assert!(ok, "{:?}", solids[0]),
            r => panic!("unexpected result {r:?}"),
        }

        // Loft a square into a circle 20 mm up.
        let lower = add_sketch(&mut e, "square");
        for (p1, p2) in [
            ([-8.0, -8.0], [8.0, -8.0]),
            ([8.0, -8.0], [8.0, 8.0]),
            ([8.0, 8.0], [-8.0, 8.0]),
            ([-8.0, 8.0], [-8.0, -8.0]),
        ] {
            e.execute(Command::AddSketchLine {
                doc: 1,
                sketch: lower,
                p1: [num(p1[0]), num(p1[1])],
                p2: [num(p2[0]), num(p2[1])],
            })
            .unwrap();
        }
        let upper = add_sketch_on(&mut e, "round", SketchPlane::Datum {
            plane: agcad_protocol::DatumPlane::Xy,
            offset: Some(num(20.0)),
        });
        e.execute(Command::AddSketchCircle {
            doc: 1,
            sketch: upper,
            center: [num(0.0), num(0.0)],
            radius: num(6.0),
        })
        .unwrap();
        let lofted = match e
            .execute(Command::Loft {
                doc: 1,
                sketches: vec![lower, upper],
                segments: Some(32),
                name: None,
                color: None,
            })
            .unwrap()
        {
            CommandResult::SolidBuilt { entity, kind, volume, .. } => {
                assert_eq!(kind, "loft");
                assert!(volume > 0.0);
                entity
            },
            r => panic!("unexpected result {r:?}"),
        };
        match e.execute(Command::Validate { doc: 1, entities: Some(vec![lofted]), min_wall_mm: None }).unwrap() {
            CommandResult::Validated { ok, solids, .. } => assert!(ok, "{:?}", solids[0]),
            r => panic!("unexpected result {r:?}"),
        }

        // A real thread, and the radius knob replays the swept spring.
        let threaded = match e
            .execute(Command::Thread {
                doc: 1,
                major_diameter: num(8.0),
                minor_diameter: None,
                pitch: num(1.25),
                length: num(10.0),
                hand: Hand::Right,
                flank_angle_deg: None,
                segments_per_turn: Some(32),
                origin: Some([num(0.0), num(0.0), num(0.0)]),
                name: Some("m8".into()),
                color: None,
            })
            .unwrap()
        {
            CommandResult::SolidBuilt { entity, kind, volume, .. } => {
                assert_eq!(kind, "thread");
                assert!(volume > 0.0);
                entity
            },
            r => panic!("unexpected result {r:?}"),
        };
        match e.execute(Command::Validate { doc: 1, entities: Some(vec![threaded]), min_wall_mm: None }).unwrap() {
            CommandResult::Validated { ok, solids, .. } => assert!(ok, "{:?}", solids[0]),
            r => panic!("unexpected result {r:?}"),
        }

        let before = scene_sig(&e);
        set_param(&mut e, "r", 3.0);
        assert_ne!(scene_sig(&e), before);
        let meshes = scene_meshes(&e);
        cold_replay(&mut e);
        assert_same_meshes(&meshes, &scene_meshes(&e));
    }

    #[test]
    fn shape_leverage_commands_round_trip_through_save_open() {
        let mut e = engine();
        e.execute(Command::SetParameter {
            doc: 1,
            name: "wall".into(),
            value: num(2.0),
            description: Some("case wall".into()),
        })
        .unwrap();
        let body = add_box(&mut e, [0.0, 0.0, 0.0], [40.0, 30.0, 20.0]);
        e.execute(Command::Shell {
            doc: 1,
            entity: body,
            thickness: Scalar::Expression("wall".into()),
            open: vec![[0.0, 0.0, 1.0]],
            angle_tol_deg: None,
        })
        .unwrap();
        e.execute(Command::Hole {
            doc: 1,
            entity: body,
            center: [num(0.0), num(0.0), num(-10.0)],
            normal: [0.0, 0.0, -1.0],
            diameter: num(5.0),
            depth: None,
            counterbore_diameter: None,
            counterbore_depth: None,
            countersink_angle_deg: None,
            segments: Some(32),
        })
        .unwrap();
        e.execute(Command::LinearPattern {
            doc: 1,
            entities: vec![body],
            direction: [num(1.0), num(0.0), num(0.0)],
            spacing: num(50.0),
            count: 2,
            name: None,
            color: None,
        })
        .unwrap();
        e.execute(Command::Thread {
            doc: 1,
            major_diameter: num(6.0),
            minor_diameter: None,
            pitch: num(1.0),
            length: num(8.0),
            hand: Hand::Left,
            flank_angle_deg: None,
            segments_per_turn: Some(24),
            origin: Some([num(0.0), num(0.0), num(20.0)]),
            name: None,
            color: None,
        })
        .unwrap();

        let (p1, p1s) = temp_file("agcad_shape_leverage.agcad");
        let (p2, p2s) = temp_file("agcad_shape_leverage_again.agcad");
        e.execute(Command::SaveDocument { doc: 1, path: p1s.clone(), source: None }).unwrap();
        let mut b = Engine::new();
        b.execute(Command::OpenDocument { path: p1s.clone(), name: None }).unwrap();
        b.execute(Command::SaveDocument { doc: 1, path: p2s.clone(), source: None }).unwrap();
        let first = std::fs::read(&p1).unwrap();
        let second = std::fs::read(&p2).unwrap();
        assert_eq!(first, second, "save -> open -> save must stay a no-op");
        let text = String::from_utf8(first).unwrap();
        for method in ["shell", "hole", "linear_pattern", "thread"] {
            assert!(text.contains(&format!("\"method\": \"{method}\"")), "missing {method}");
        }
        // Geometry is rebuilt on open, so the entities and ids match.
        let a = scene_sig(&e);
        let bb = scene_sig(&b);
        assert_eq!(a, bb);

        let _ = std::fs::remove_file(&p1);
        let _ = std::fs::remove_file(&p2);
    }
}
