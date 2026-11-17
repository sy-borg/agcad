//! # agcad-protocol
//!
//! Wire types shared by every agcad component: core, server, viewer and
//! clients (Python, Rust, anything that can speak JSON).
//!
//! ## Transport
//!
//! Line-delimited JSON-RPC 2.0 over stdio or TCP. One request per line,
//! one response per line.
//!
//! ```json
//! {"jsonrpc":"2.0","id":1,"method":"add_box","params":{"doc":1,"name":"base","center":[0,0,5],"size":[80,40,"wall*2"]}}
//! ```
//!
//! The server merges `method` into `params` and deserializes the result
//! into a [`Command`] (internally tagged on `method`). Responses carry a
//! [`CommandResult`] (externally tagged, e.g. `{"entity_created":{...}}`)
//! or an [`RpcError`].
//!
//! ## Parametric model
//!
//! Modeling commands are recorded in the document's program. Numeric
//! arguments may be [`Scalar::Expression`]s referencing named parameters
//! ("wall", "plate*0.5"). When a parameter changes, the program is
//! replayed from scratch — deterministically, with identical entity ids.
//! The program *is* the function; agents author and tune it.
//!
//! ## Conventions
//!
//! * All IDs are small deterministic integers — agents must be able to
//!   predict and reuse them.
//! * All lengths are millimeters, angles degrees, colors RGBA floats 0..1.
//! * Commands are reversible where it makes sense; `undo`/`redo` are
//!   per-document and restore parameters + program + geometry atomically.

use serde::{Deserialize, Serialize};

pub const PROTOCOL_VERSION: u32 = 1;
pub const SERVER_NAME: &str = "agcad";

pub type DocId = u64;
pub type EntityId = u64;
pub type Vec3 = [f64; 3];
pub type Rgba = [f32; 4];
/// Column-major 4x4 transform: `m[col][row]`.
pub type Mat4 = [[f32; 4]; 4];
/// A 3-vector whose components may be expressions over document parameters.
pub type SVec3 = [Scalar; 3];

/// A numeric command argument: a plain number, or an expression string
/// referencing document parameters (`"wall*2"`, `"plate/2 + 5"`).
/// Evaluated at execution time.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(untagged)]
pub enum Scalar {
    Number(f64),
    Expression(String),
}

/// A single request envelope.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RpcRequest {
    #[serde(default = "default_jsonrpc")]
    pub jsonrpc: String,
    pub id: u64,
    pub method: String,
    #[serde(default)]
    pub params: serde_json::Value,
}

fn default_jsonrpc() -> String {
    "2.0".to_string()
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RpcError {
    pub code: i32,
    pub message: String,
}

impl RpcError {
    pub fn parse(message: impl Into<String>) -> Self {
        Self { code: -32700, message: message.into() }
    }
    pub fn method_not_found(message: impl Into<String>) -> Self {
        Self { code: -32601, message: message.into() }
    }
    pub fn invalid_params(message: impl Into<String>) -> Self {
        Self { code: -32602, message: message.into() }
    }
    pub fn internal(message: impl Into<String>) -> Self {
        Self { code: -32603, message: message.into() }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RpcResponse {
    #[serde(default = "default_jsonrpc")]
    pub jsonrpc: String,
    pub id: u64,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub result: Option<CommandResult>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub error: Option<RpcError>,
}

impl RpcResponse {
    pub fn ok(id: u64, result: CommandResult) -> Self {
        Self { jsonrpc: "2.0".into(), id, result: Some(result), error: None }
    }

    pub fn err(id: u64, error: RpcError) -> Self {
        Self { jsonrpc: "2.0".into(), id, result: None, error: Some(error) }
    }
}

/// Every operation the bus understands. `method` is the JSON-RPC method
/// name; all remaining variant fields live inside `params`.
///
/// Modeling commands (Add*, Transform, SetColor, Delete) are appended to
/// the document program and replayed when parameters change. Session
/// commands (initialize, undo/redo, get_scene, ...) are not part of the
/// program.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "method", rename_all = "snake_case")]
pub enum Command {
    Initialize,
    ListDocuments,
    CreateDocument { name: String },
    CloseDocument { doc: DocId },

    AddBox {
        doc: DocId,
        name: String,
        /// Box center in world space.
        center: SVec3,
        size: SVec3,
        #[serde(default)]
        color: Option<Rgba>,
    },
    AddSphere {
        doc: DocId,
        name: String,
        center: SVec3,
        radius: Scalar,
        #[serde(default)]
        segments: Option<u32>,
        #[serde(default)]
        color: Option<Rgba>,
    },
    AddCylinder {
        doc: DocId,
        name: String,
        center: SVec3,
        radius: Scalar,
        /// Along local +Y.
        height: Scalar,
        #[serde(default)]
        segments: Option<u32>,
        #[serde(default)]
        color: Option<Rgba>,
    },

    /// Replaces only the components that are provided (`None` keeps the
    /// current value). Rotation is extrinsic XYZ, applied X, then Y, then Z,
    /// around the entity's own origin.
    Transform {
        doc: DocId,
        entity: EntityId,
        #[serde(default)]
        translation: Option<SVec3>,
        #[serde(default)]
        rotation_deg: Option<SVec3>,
        #[serde(default)]
        scale: Option<SVec3>,
    },
    SetColor { doc: DocId, entity: EntityId, color: Rgba },
    Delete { doc: DocId, entity: EntityId },

    /// Define or update a parameter. The value itself may be an expression
    /// over *other* parameters (self-reference is rejected). Changing a
    /// parameter replays the document program.
    SetParameter {
        doc: DocId,
        name: String,
        value: Scalar,
        #[serde(default)]
        description: Option<String>,
    },
    RemoveParameter { doc: DocId, name: String },
    GetParameters { doc: DocId },

    Undo { doc: DocId },
    Redo { doc: DocId },

    GetScene { doc: DocId },
}

/// Result of a successfully executed [`Command`]. Externally tagged:
/// `{"entity_created":{"doc":1,"entity":2,"mesh":{...}}}`.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CommandResult {
    Initialized { protocol_version: u32, server: String },
    Documents { docs: Vec<DocInfo> },
    DocumentCreated { doc: DocId },
    Closed { doc: DocId },
    EntityCreated { doc: DocId, entity: EntityId, mesh: Mesh },
    Transformed { doc: DocId, entity: EntityId },
    Deleted { doc: DocId, entity: EntityId },
    ParameterSet { doc: DocId, name: String, value: f64 },
    ParameterRemoved { doc: DocId, name: String },
    Parameters { doc: DocId, params: Vec<ParamInfo> },
    Undone { doc: DocId },
    Redone { doc: DocId },
    Scene { doc: DocId, entities: Vec<EntitySnapshot> },
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DocInfo {
    pub doc: DocId,
    pub name: String,
    pub entity_count: usize,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ParamInfo {
    pub name: String,
    pub value: f64,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub description: Option<String>,
}

/// Triangle mesh in entity-local space (millimeters). Geometry is baked
/// centered on the entity origin; world placement comes from the entity
/// transform.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct Mesh {
    pub positions: Vec<[f32; 3]>,
    pub normals: Vec<[f32; 3]>,
    pub indices: Vec<u32>,
}

/// Everything a renderer needs to draw one entity.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EntitySnapshot {
    pub id: EntityId,
    pub name: String,
    pub translation: Vec3,
    pub rotation_deg: Vec3,
    pub scale: Vec3,
    /// Model matrix `T * Rz * Ry * Rx * S`, column-major.
    pub transform: Mat4,
    pub color: Rgba,
    pub mesh: Mesh,
}
