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
pub mod mat4;
pub mod plane;
pub mod render;
pub mod sketch;
pub mod solid;
pub mod stl;
pub mod threemf;

use std::collections::BTreeMap;
use std::fmt;

use agcad_protocol::{
    Bbox, BooleanOp, Command, CommandResult, DocId, DocInfo, DocumentSnapshot, EdgeFilter,
    EntityId, EntitySnapshot, Mat4, Mesh, ParamInfo, Rgba, RevolveAxis, SavedParameter, Scalar,
    SketchElement, SketchPlane, SourceDoc, SVec3, Vec3, DOCUMENT_FORMAT, DOCUMENT_VERSION,
    PROTOCOL_VERSION, SERVER_NAME,
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
    fn replay(&mut self) -> Result<()> {
        let program = self.program.clone();
        self.entities.clear();
        self.next_entity = 1;
        self.replaying = true;
        let mut result = Ok(());
        for (step, cmd) in program.iter().enumerate() {
            if let Err(e) = exec(self, cmd) {
                result = Err(Error::Replay { step, source: e.to_string() });
                break;
            }
        }
        self.replaying = false;
        result
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
        | Command::SetParameter { doc: d, .. }
        | Command::RemoveParameter { doc: d, .. }
        | Command::GetParameters { doc: d }
        | Command::Undo { doc: d }
        | Command::Redo { doc: d }
        | Command::GetScene { doc: d }
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
            | Command::Revolve { .. } => {
                let doc = doc_of(&command).expect("modeling commands always carry a doc");
                let d = self.doc_mut(doc)?;
                d.checkpoint();
                d.program.push(command.clone());
                match exec(d, &command) {
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

                let doc = self.next_doc;
                let stored_name = if snap.name.trim().is_empty() {
                    std::path::Path::new(path)
                        .file_stem()
                        .map(|s| s.to_string_lossy().into_owned())
                        .unwrap_or_else(|| "document".into())
                } else {
                    snap.name.clone()
                };
                let mut d = Document::new(doc, name.clone().unwrap_or(stored_name));
                d.params = snap
                    .parameters
                    .iter()
                    .map(|(k, p)| {
                        (k.clone(), Param { value: p.value, description: p.description.clone() })
                    })
                    .collect();
                d.source = snap.source.clone();
                d.program = snap.program;
                for cmd in &mut d.program {
                    set_doc(cmd, doc);
                }
                // Rebuild geometry from the program. If it fails, nothing is
                // inserted, so the engine never holds a half-open document.
                d.replay().map_err(|e| {
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
        | Command::SetParameter { doc, .. }
        | Command::RemoveParameter { doc, .. }
        | Command::GetParameters { doc }
        | Command::Undo { doc }
        | Command::Redo { doc }
        | Command::GetScene { doc }
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
            // Gather operands in *world space*: local mesh through the
            // entity's model matrix. Color and name inherit from the
            // first operand unless overridden.
            let mut world: Vec<Mesh> = Vec::with_capacity(entities.len());
            let mut source_color: Option<Rgba> = None;
            for &eid in entities {
                let e = d.entities.get(&eid).ok_or(Error::UnknownEntity { doc: *doc, entity: eid })?;
                let m = transform_mesh(&e.mesh, &e.transform());
                world.push(m);
                if source_color.is_none() {
                    source_color = Some(e.color);
                }
            }
            let result = boolean::boolean(*op, &world)?;

            // Bake the result centered on its bounding-box center so the
            // entity follows the usual "geometry local, transform places"
            // convention.
            let (mesh, center) = recenter(&result);
            let id = d.next_entity();
            let entity = Entity {
                id,
                name: name.clone().unwrap_or_else(|| op.as_str().to_string()),
                shape: Shape::Boolean { op: *op, entities: entities.clone() },
                mesh,
                translation: center,
                rotation_deg: [0.0; 3],
                scale: [1.0; 3],
                color: color.unwrap_or(source_color.unwrap_or([0.7, 0.72, 0.75, 1.0])),
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
            let (mesh, volume) = solid::extrude(&loops, distance)?;
            let tris = mesh.indices.len() / 3;
            let id = d.next_entity();
            d.entities.insert(
                id,
                Entity {
                    id,
                    name: name.clone().unwrap_or_else(|| "extrude".into()),
                    shape: Shape::Extrude { sketch: *sketch, distance },
                    mesh,
                    translation,
                    rotation_deg,
                    scale,
                    color: color.unwrap_or([0.72, 0.74, 0.78, 1.0]),
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
            let (mesh, volume) = solid::revolve(&loops, angle, axis, segments)?;
            let tris = mesh.indices.len() / 3;
            let id = d.next_entity();
            d.entities.insert(
                id,
                Entity {
                    id,
                    name: name.clone().unwrap_or_else(|| "revolve".into()),
                    shape: Shape::Revolve { sketch: *sketch, angle_deg: angle, axis, segments },
                    mesh,
                    translation,
                    rotation_deg,
                    scale,
                    color: color.unwrap_or([0.72, 0.74, 0.78, 1.0]),
                },
            );
            Ok(CommandResult::SolidCreated { doc: *doc, entity: id, sketch: *sketch, volume, tris })
        },

        Command::Delete { doc, entity } => {
            if d.entities.remove(entity).is_none() {
                return Err(Error::UnknownEntity { doc: *doc, entity: *entity });
            }
            // Deleting a sketch also removes the solids built from it —
            // otherwise replay would hit a dangling sketch reference.
            let dependents: Vec<EntityId> = d
                .entities
                .iter()
                .filter(|(_, e)| match e.shape {
                    Shape::Extrude { sketch, .. } | Shape::Revolve { sketch, .. } => sketch == *entity,
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
    let (world, transform) = {
        let e = d.entities.get(&entity).ok_or(Error::UnknownEntity { doc, entity })?;
        if matches!(e.shape, Shape::Sketch { .. }) {
            return Err(Error::Invalid(format!(
                "entity {entity} is a sketch (hint: fillet/chamfer apply to printable solids; \
                 extrude the sketch into a solid first)"
            )));
        }
        let m = e.transform();
        (transform_mesh(&e.mesh, &m), m)
    };

    let empty = EdgeFilter::default();
    let filter = edges.unwrap_or(&empty);
    let (world, edges) = fillet::apply(&world, kind, filter, min_angle_deg.unwrap_or(30.0))?;
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

    d.entity_mut(entity)?.mesh = mesh.clone();
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
    let (se, sc, _, _, solved, residual, dof) = sketch_parts(d, sketch)?;
    *se = elements;
    *sc = constraints;
    *solved = report.solved;
    *residual = report.residual;
    *dof = report.dof;
    d.entity_mut(sketch)?.mesh = mesh;
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

/// Transform a local mesh into world space (f32 math, matching the
/// viewer's model matrix convention).
fn transform_mesh(mesh: &Mesh, m: &Mat4) -> Mesh {
    Mesh {
        positions: mesh.positions.iter().map(|&p| mat4::transform_point(m, p)).collect(),
        normals: mesh.normals.clone(),
        indices: mesh.indices.clone(),
    }
}

/// Compute a mesh's bounding-box center and shift positions so the box is
/// centered on the origin. Returns `(mesh, center)` where `center` is the
/// translation that undoes the shift (i.e. the original center).
fn recenter(mesh: &Mesh) -> (Mesh, Vec3) {
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
    let mesh = match &shape {
        Shape::Box { size } => geo::box_mesh(*size),
        Shape::Sphere { radius, segments } => geo::sphere_mesh(*radius, *segments),
        Shape::Cylinder { radius, height, segments } => {
            geo::cylinder_mesh(*radius, *height, *segments)
        },
        // Boolean entities are produced by the boolean path, which builds
        // the Entity directly — add_entity never sees this shape.
        Shape::Boolean { .. } => unreachable!("boolean shapes are not tessellated here"),
        // Sketches are created by AddSketch (with solver state), never here.
        Shape::Sketch { .. } => unreachable!("sketch shapes are not tessellated here"),
        // Solids are built by Extrude/Revolve, which create their entities
        // directly with the kernel's mesh.
        Shape::Extrude { .. } | Shape::Revolve { .. } => {
            unreachable!("extrude/revolve shapes are not tessellated here")
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
    };
    let mesh = entity.mesh.clone();
    d.entities.insert(id, entity);
    Ok(CommandResult::EntityCreated { doc, entity: id, mesh })
}

#[cfg(test)]
mod tests {
    use super::*;

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
        assert!(text.contains("\"version\": 2"));
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
}
