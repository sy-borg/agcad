//! # agcad-core
//!
//! The headless engine. Owns documents and entities, executes
//! [`Command`]s, and produces [`CommandResult`]s. Knows nothing about
//! transport (that's the server) or rendering (that's the viewer) — the
//! same engine serves human UIs, agents and tests.

pub mod geo;
pub mod mat4;

use std::collections::BTreeMap;
use std::fmt;

use agcad_protocol::{
    Command, CommandResult, DocId, DocInfo, EntityId, EntitySnapshot, Mat4, Mesh, Rgba, Vec3,
    PROTOCOL_VERSION, SERVER_NAME,
};

/// Default tessellation segments for curved primitives.
const DEFAULT_SEGMENTS: u32 = 32;
/// Undo history kept per document (snapshots). Fine for v0; switch to a
/// command-log design when documents grow large.
const MAX_HISTORY: usize = 128;

#[derive(Debug, Clone)]
pub enum Error {
    UnknownDocument(DocId),
    UnknownEntity { doc: DocId, entity: EntityId },
    Invalid(String),
    NothingToUndo(DocId),
    NothingToRedo(DocId),
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::UnknownDocument(d) => write!(f, "unknown document {d} (hint: run create_document first, or list_documents)"),
            Self::UnknownEntity { doc, entity } => write!(f, "document {doc} has no entity {entity} (hint: get_scene lists valid entity ids)"),
            Self::Invalid(msg) => write!(f, "invalid parameters: {msg}"),
            Self::NothingToUndo(d) => write!(f, "document {d}: nothing to undo"),
            Self::NothingToRedo(d) => write!(f, "document {d}: nothing to redo"),
        }
    }
}

impl std::error::Error for Error {}

pub type Result<T> = std::result::Result<T, Error>;

/// All state that gets restored by undo, captured before each mutating
/// command.
#[derive(Debug, Clone)]
struct Snapshot {
    next_entity: EntityId,
    entities: BTreeMap<EntityId, Entity>,
}

#[derive(Debug, Clone)]
pub struct Entity {
    pub id: EntityId,
    pub name: String,
    pub shape: Shape,
    /// Tessellated geometry in local space (baked once, immutable in v0).
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
        });
        if self.undo_stack.len() > MAX_HISTORY {
            self.undo_stack.remove(0);
        }
        self.redo_stack.clear();
    }

    fn entity_mut(&mut self, id: EntityId) -> Result<&mut Entity> {
        let doc = self.id;
        self.entities.get_mut(&id).ok_or(Error::UnknownEntity { doc, entity: id })
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
        let next = self.docs.get_mut(&id);
        next.ok_or(Error::UnknownDocument(id))
    }

    /// Execute one command and produce its result.
    pub fn execute(&mut self, command: Command) -> Result<CommandResult> {
        match command {
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
                self.docs.insert(doc, Document::new(doc, name));
                Ok(CommandResult::DocumentCreated { doc })
            },

            Command::CloseDocument { doc } => {
                self.docs.remove(&doc).ok_or(Error::UnknownDocument(doc))?;
                Ok(CommandResult::Closed { doc })
            },

            Command::AddBox { doc, name, center, size, color } => {
                if size.iter().any(|&s| s <= 0.0) {
                    return Err(Error::Invalid("box size components must be > 0".into()));
                }
                self.add_entity(doc, name, Shape::Box { size }, center, [0.0; 3], [1.0; 3], color)
            },

            Command::AddSphere { doc, name, center, radius, segments, color } => {
                if radius <= 0.0 {
                    return Err(Error::Invalid("sphere radius must be > 0".into()));
                }
                let segments = segments.unwrap_or(DEFAULT_SEGMENTS).max(3);
                self.add_entity(
                    doc,
                    name,
                    Shape::Sphere { radius, segments },
                    center,
                    [0.0; 3],
                    [1.0; 3],
                    color,
                )
            },

            Command::AddCylinder { doc, name, center, radius, height, segments, color } => {
                if radius <= 0.0 || height <= 0.0 {
                    return Err(Error::Invalid("cylinder radius and height must be > 0".into()));
                }
                let segments = segments.unwrap_or(DEFAULT_SEGMENTS).max(3);
                self.add_entity(
                    doc,
                    name,
                    Shape::Cylinder { radius, height, segments },
                    center,
                    [0.0; 3],
                    [1.0; 3],
                    color,
                )
            },

            Command::Transform { doc, entity, translation, rotation_deg, scale } => {
                let d = self.doc_mut(doc)?;
                d.checkpoint();
                let e = d.entity_mut(entity)?;
                if let Some(t) = translation {
                    e.translation = t;
                }
                if let Some(r) = rotation_deg {
                    e.rotation_deg = r;
                }
                if let Some(s) = scale {
                    e.scale = s;
                }
                Ok(CommandResult::Transformed { doc, entity })
            },

            Command::SetColor { doc, entity, color } => {
                let d = self.doc_mut(doc)?;
                d.checkpoint();
                let e = d.entity_mut(entity)?;
                e.color = color;
                Ok(CommandResult::Transformed { doc, entity })
            },

            Command::Delete { doc, entity } => {
                let d = self.doc_mut(doc)?;
                d.checkpoint();
                d.entities.remove(&entity).ok_or(Error::UnknownEntity { doc, entity })?;
                Ok(CommandResult::Deleted { doc, entity })
            },

            Command::Undo { doc } => {
                let d = self.doc_mut(doc)?;
                let snap = d.undo_stack.pop().ok_or(Error::NothingToUndo(doc))?;
                d.redo_stack.push(Snapshot {
                    next_entity: d.next_entity,
                    entities: std::mem::take(&mut d.entities),
                });
                d.next_entity = snap.next_entity;
                d.entities = snap.entities;
                Ok(CommandResult::Undone { doc })
            },

            Command::Redo { doc } => {
                let d = self.doc_mut(doc)?;
                let snap = d.redo_stack.pop().ok_or(Error::NothingToRedo(doc))?;
                d.undo_stack.push(Snapshot {
                    next_entity: d.next_entity,
                    entities: std::mem::take(&mut d.entities),
                });
                d.next_entity = snap.next_entity;
                d.entities = snap.entities;
                Ok(CommandResult::Redone { doc })
            },

            Command::GetScene { doc } => {
                let d = self.doc(doc)?;
                Ok(CommandResult::Scene {
                    doc,
                    entities: d.entities.values().map(|e| e.snapshot()).collect(),
                })
            },
        }
    }

    #[allow(clippy::too_many_arguments)]
    fn add_entity(
        &mut self,
        doc: DocId,
        name: String,
        shape: Shape,
        translation: Vec3,
        rotation_deg: Vec3,
        scale: Vec3,
        color: Option<Rgba>,
    ) -> Result<CommandResult> {
        let d = self.doc_mut(doc)?;
        d.checkpoint();
        let id = d.next_entity();
        let mesh = match &shape {
            Shape::Box { size } => geo::box_mesh(*size),
            Shape::Sphere { radius, segments } => geo::sphere_mesh(*radius, *segments),
            Shape::Cylinder { radius, height, segments } => {
                geo::cylinder_mesh(*radius, *height, *segments)
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

    fn add_box(e: &mut Engine) -> EntityId {
        match e
            .execute(Command::AddBox {
                doc: 1,
                name: "box".into(),
                center: [0.0, 0.0, 5.0],
                size: [10.0, 10.0, 10.0],
                color: None,
            })
            .unwrap()
        {
            CommandResult::EntityCreated { entity, mesh, .. } => {
                assert_eq!(mesh.positions.len(), 24, "box must tessellate to 24 verts");
                entity
            },
            r => panic!("unexpected result {r:?}"),
        }
    }

    #[test]
    fn ids_are_deterministic() {
        let mut a = engine();
        let mut b = engine();
        assert_eq!(add_box(&mut a), add_box(&mut b));
    }

    #[test]
    fn undo_restores_state() {
        let mut e = engine();
        let id = add_box(&mut e);
        e.execute(Command::Delete { doc: 1, entity: id }).unwrap();
        e.execute(Command::Undo { doc: 1 }).unwrap();
        match e.execute(Command::GetScene { doc: 1 }).unwrap() {
            CommandResult::Scene { entities, .. } => {
                assert_eq!(entities.len(), 1);
                assert_eq!(entities[0].id, id);
            },
            r => panic!("unexpected result {r:?}"),
        }
        // Redo deletes it again.
        e.execute(Command::Redo { doc: 1 }).unwrap();
        match e.execute(Command::GetScene { doc: 1 }).unwrap() {
            CommandResult::Scene { entities, .. } => assert!(entities.is_empty()),
            r => panic!("unexpected result {r:?}"),
        }
    }

    #[test]
    fn transform_composes_correctly() {
        let mut e = engine();
        let id = add_box(&mut e);
        e.execute(Command::Transform {
            doc: 1,
            entity: id,
            translation: Some([1.0, 2.0, 3.0]),
            rotation_deg: Some([0.0, 0.0, 90.0]),
            scale: Some([2.0, 2.0, 2.0]),
        })
        .unwrap();
        match e.execute(Command::GetScene { doc: 1 }).unwrap() {
            CommandResult::Scene { entities, .. } => {
                let m = entities[0].transform;
                // Column 3 is the translation.
                assert_eq!(m[3], [1.0, 2.0, 3.0, 1.0]);
                // 90° Z rotation: unit X becomes unit Y (allow float noise).
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
                center: [0.0; 3],
                size: [1.0; 3],
                color: None,
            }),
            Err(Error::UnknownDocument(99))
        ));
        assert!(matches!(
            e.execute(Command::AddBox {
                doc: 1,
                name: "x".into(),
                center: [0.0; 3],
                size: [0.0, 1.0, 1.0],
                color: None,
            }),
            Err(Error::Invalid(_))
        ));
    }
}
