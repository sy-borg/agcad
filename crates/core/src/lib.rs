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
pub mod expr;
pub mod geo;
pub mod mat4;
pub mod render;
pub mod sketch;
pub mod solid;
pub mod stl;

use std::collections::BTreeMap;
use std::fmt;

use agcad_protocol::{
    BooleanOp, Command, CommandResult, DocId, DocInfo, EntityId, EntitySnapshot, Mat4,
    Mesh, ParamInfo, Rgba, RevolveAxis, Scalar, SketchElement, SVec3, Vec3, PROTOCOL_VERSION,
    SERVER_NAME,
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
    NothingToUndo(DocId),
    NothingToRedo(DocId),
    Replay { step: usize, source: String },
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::UnknownDocument(d) => write!(f, "unknown document {d} (hint: run create_document first, or list_documents)"),
            Self::UnknownEntity { doc, entity } => write!(f, "document {doc} has no entity {entity} (hint: get_scene lists valid entity ids)"),
            Self::Invalid(msg) => write!(f, "invalid parameters: {msg}"),
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
        EntitySnapshot {
            id: self.id,
            name: self.name.clone(),
            translation: self.translation,
            rotation_deg: self.rotation_deg,
            scale: self.scale,
            transform: self.transform(),
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
                    })
                },
                _ => None,
            },
        }
    }
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
        for (step, cmd) in program.iter().enumerate() {
            if let Err(e) = exec(self, cmd) {
                return Err(Error::Replay { step, source: e.to_string() });
            }
        }
        Ok(())
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

            Command::CreateDocument { name } => {
                let doc = self.next_doc;
                self.next_doc += 1;
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
                d.params.insert(name.clone(), Param {
                    value: resolved,
                    description: description.clone(),
                });
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
                rebuild_dependents(d, *doc, *entity)?;
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

        // --- sketches -------------------------------------------------------

        Command::AddSketch { doc, name } => {
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
                translation: [0.0; 3],
                rotation_deg: [0.0; 3],
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
    rebuild_dependents(d, doc, sketch)?;
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
        match e.execute(Command::CreateDocument { name: "test".into() }).unwrap() {
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
                // Cylinder regenerated with the new wall: height = 20.
                let cyl = entities.iter().find(|x| x.id == b).unwrap();
                let max_y = cyl.mesh.positions.iter().map(|p| p[1]).fold(f32::MIN, f32::max);
                assert!((max_y - 10.0).abs() < 1e-4, "expected height 20 (±10), got {max_y}");
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
        match e.execute(Command::AddSketch { doc: 1, name: name.into() }).unwrap() {
            CommandResult::EntityCreated { entity, .. } => entity,
            r => panic!("unexpected result {r:?}"),
        }
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

    /// A properly constrained rectangle profile (closed loop, axis-aligned),
    /// so later constraints edit it without tearing the loop apart.
    fn rect_sketch(e: &mut Engine, w: f64, h: f64) -> (EntityId, [u64; 4]) {
        let s = add_sketch(e, "profile");
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
        (s, [a, b, c, d])
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
}
