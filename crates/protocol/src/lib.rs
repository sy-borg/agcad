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
//! {"jsonrpc":"2.0","id":1,"method":"add_box","params":{"doc":1,"name":"base","center":[0,0,0],"size":[10,10,10]}}
//! ```
//!
//! The server merges `method` into `params` and deserializes the result
//! into a [`Command`] (internally tagged on `method`). Responses carry a
//! [`CommandResult`] (externally tagged, e.g. `{"entity_created":{...}}`)
//! or an [`RpcError`].
//!
//! ## Conventions
//!
//! * All IDs are small deterministic integers — agents must be able to
//!   predict and reuse them.
//! * All lengths are millimeters, angles degrees, colors RGBA floats 0..1.
//! * Commands are reversible where it makes sense; `undo`/`redo` are
//!   per-document.

use serde::{Deserialize, Serialize};

pub const PROTOCOL_VERSION: u32 = 1;
pub const SERVER_NAME: &str = "agcad";

pub type DocId = u64;
pub type EntityId = u64;
pub type Vec3 = [f64; 3];
pub type Rgba = [f32; 4];
/// Column-major 4x4 transform: `m[col][row]`.
pub type Mat4 = [[f32; 4]; 4];

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
        center: Vec3,
        size: Vec3,
        #[serde(default)]
        color: Option<Rgba>,
    },
    AddSphere {
        doc: DocId,
        name: String,
        center: Vec3,
        radius: f64,
        #[serde(default)]
        segments: Option<u32>,
        #[serde(default)]
        color: Option<Rgba>,
    },
    AddCylinder {
        doc: DocId,
        name: String,
        center: Vec3,
        radius: f64,
        /// Along local +Y.
        height: f64,
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
        translation: Option<Vec3>,
        #[serde(default)]
        rotation_deg: Option<Vec3>,
        #[serde(default)]
        scale: Option<Vec3>,
    },
    SetColor { doc: DocId, entity: EntityId, color: Rgba },
    Delete { doc: DocId, entity: EntityId },

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

/// Triangle mesh in entity-local space (meters: millimeters). Geometry is
/// baked centered on the entity origin; world placement comes from the
/// entity transform.
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
