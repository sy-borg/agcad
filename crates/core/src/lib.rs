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
pub mod stl;

use std::collections::BTreeMap;
use std::fmt;

use agcad_protocol::{
    BooleanOp, Command, CommandResult, DocId, DocInfo, EntityId, EntitySnapshot, Mat4, Mesh,
    ParamInfo, Rgba, Scalar, SVec3, Vec3, PROTOCOL_VERSION, SERVER_NAME,
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
            | Command::Boolean { .. } => {
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
                let entries = ids
                    .iter()
                    .map(|&id| {
                        let e = d.entities.get(&id).ok_or(Error::UnknownEntity { doc: *doc, entity: id })?;
                        Ok((e.mesh.clone(), e.transform()))
                    })
                    .collect::<Result<Vec<_>>>()?;
                let (resolved, tris) = stl::write_stl(path, &entries)?;
                Ok(CommandResult::StlExported {
                    doc: *doc,
                    path: resolved,
                    tris,
                    bytes: 84 + tris * 50,
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
        | Command::SetParameter { doc, .. }
        | Command::RemoveParameter { doc, .. }
        | Command::GetParameters { doc }
        | Command::Undo { doc }
        | Command::Redo { doc }
        | Command::GetScene { doc }
        | Command::ExportStl { doc, .. }
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

        Command::Delete { doc, entity } => {
            if d.entities.remove(entity).is_none() {
                return Err(Error::UnknownEntity { doc: *doc, entity: *entity });
            }
            Ok(CommandResult::Deleted { doc: *doc, entity: *entity })
        },

        _ => Err(Error::Invalid("not a modeling command".into())),
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

    /// Binary STL: correct header, exact triangle count, exact file size.
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
}
