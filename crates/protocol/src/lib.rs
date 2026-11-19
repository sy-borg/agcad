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
/// Modeling commands (Add*, Transform, SetColor, Delete, Boolean) are
/// appended to the document program and replayed when parameters change.
/// Session commands (initialize, undo/redo, get_scene, export_stl, ...)
/// are not part of the program.
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

    /// Boolean set operation over existing entities, evaluated in
    /// **world space** (each operand's transform is applied first). The
    /// result becomes a new entity, placed at the center of its bounding
    /// box with identity rotation/scale. Sources are kept — delete them
    /// explicitly if you only want the fused result.
    Boolean {
        doc: DocId,
        /// `union`, `difference` or `intersection`.
        op: BooleanOp,
        /// Operand entity ids, in order. For `difference` the first id is
        /// the solid everything else is subtracted from.
        entities: Vec<EntityId>,
        #[serde(default)]
        name: Option<String>,
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

    /// Write a binary STL of the selected entities' **world-space**
    /// geometry (default: every entity in the document). A session
    /// command, not part of the program — exporting changes no state.
    ExportStl {
        doc: DocId,
        /// Filesystem path the *server* writes to; absolute paths avoid
        /// surprises about the server's working directory.
        path: String,
        #[serde(default)]
        entities: Option<Vec<EntityId>>,
    },

    /// Headless software render of the document's world-space scene to a
    /// PNG, returned inline as base64 (and optionally written to `path`).
    /// A session command — rendering changes no state. Deterministic:
    /// same scene + same camera → bit-identical bytes.
    RenderOffscreen {
        doc: DocId,
        #[serde(default)]
        width: Option<u32>,
        #[serde(default)]
        height: Option<u32>,
        /// Camera eye; omit for an auto-framed 3/4 view of the scene.
        #[serde(default)]
        eye: Option<Vec3>,
        #[serde(default)]
        target: Option<Vec3>,
        #[serde(default)]
        up: Option<Vec3>,
        #[serde(default)]
        fov_deg: Option<f64>,
        /// Parallel (orthographic) projection instead of perspective. With
        /// `fov_deg` this keeps the same framing, but sizes no longer change
        /// with depth — what standard front/top/side CAD views want.
        #[serde(default)]
        ortho: Option<bool>,
        #[serde(default)]
        background: Option<Rgba>,
        #[serde(default)]
        path: Option<String>,
    },

    /// Create an empty sketch: a 2D constraint-solved drawing placed in 3D
    /// by the entity transform (default: the XY plane at the origin).
    /// Sketches are recorded in the program like any other modeling
    /// command, so they replay parametrically — coordinates and constraint
    /// values may be expressions over parameters.
    AddSketch { doc: DocId, name: String },

    /// Append a line segment to a sketch. 2D coordinates in sketch-local
    /// units (mm), may be expressions.
    AddSketchLine {
        doc: DocId,
        sketch: EntityId,
        p1: [Scalar; 2],
        p2: [Scalar; 2],
    },

    /// Append a circle to a sketch.
    AddSketchCircle {
        doc: DocId,
        sketch: EntityId,
        center: [Scalar; 2],
        radius: Scalar,
    },

    /// Add a geometric/numeric constraint to a sketch and re-solve. The
    /// solver adjusts free element coordinates to satisfy all constraints;
    /// if it cannot, the constraint is still recorded and the failure is
    /// reported (over-constrained sketches stay inspectable, not hidden).
    Constrain {
        doc: DocId,
        sketch: EntityId,
        kind: ConstraintKind,
        /// Element references, meaning depends on `kind`:
        /// coincident/distance → two point refs; horizontal/vertical → one
        /// whole-element ref; parallel/perpendicular → two element refs;
        /// radius/diameter → one circle ref.
        refs: Vec<SketchRef>,
        /// Required by distance/radius/diameter (mm). May be an expression.
        #[serde(default)]
        value: Option<Scalar>,
    },

    /// Remove a constraint by id and re-solve.
    RemoveConstraint { doc: DocId, sketch: EntityId, constraint: u64 },

    /// Drag a sketch point to new 2D coordinates, then re-solve. Useful
    /// for steering the solver into a different configuration.
    MoveSketchPoint {
        doc: DocId,
        sketch: EntityId,
        element: u64,
        /// 0 = line start / circle center, 1 = line end.
        point: u8,
        to: [f64; 2],
    },

    /// Delete a sketch element; constraints referencing it are removed
    /// too (cascade), then the sketch re-solves.
    DeleteSketchElement { doc: DocId, sketch: EntityId, element: u64 },

    /// Extrude a closed sketch profile into a printable solid along the
    /// sketch normal. The sketch must be made of closed loops (lines that
    /// form loops, and/or circles); holes are handled (a circle inside a
    /// loop becomes a through-hole). The result is a watertight solid
    /// placed by the sketch's transform, and it follows later sketch edits.
    Extrude {
        doc: DocId,
        sketch: EntityId,
        /// Height along the sketch normal (mm). Negative extrudes the other
        /// way. May be an expression.
        distance: Scalar,
        #[serde(default)]
        name: Option<String>,
        #[serde(default)]
        color: Option<Rgba>,
    },

    /// Revolve a closed sketch profile around an in-plane axis into a
    /// printable solid. The profile must lie entirely on one side of the
    /// axis (touching it is fine).
    Revolve {
        doc: DocId,
        sketch: EntityId,
        /// Sweep angle in degrees (default 360). May be an expression.
        #[serde(default)]
        angle_deg: Option<Scalar>,
        /// Axis to revolve around, in sketch-local coordinates
        /// (default: the sketch's X axis).
        #[serde(default)]
        axis: Option<RevolveAxis>,
        /// Segments around the revolution; omit for an automatic count.
        #[serde(default)]
        segments: Option<u32>,
        #[serde(default)]
        name: Option<String>,
        #[serde(default)]
        color: Option<Rgba>,
    },
}

/// Which set operation a [`Command::Boolean`] performs.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum BooleanOp {
    Union,
    Difference,
    Intersection,
}

impl BooleanOp {
    pub fn as_str(&self) -> &'static str {
        match self {
            Self::Union => "union",
            Self::Difference => "difference",
            Self::Intersection => "intersection",
        }
    }
}

impl std::fmt::Display for BooleanOp {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(self.as_str())
    }
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
    StlExported { doc: DocId, path: String, tris: usize, bytes: usize },
    Rendered {
        doc: DocId,
        width: u32,
        height: u32,
        /// PNG bytes, base64-encoded (the bus is line-delimited JSON).
        png_base64: String,
        /// Set when the command also wrote the PNG to a file.
        #[serde(skip_serializing_if = "Option::is_none")]
        path: Option<String>,
    },
    SketchElementCreated { doc: DocId, sketch: EntityId, element: u64 },
    SketchElementDeleted { doc: DocId, sketch: EntityId, element: u64 },
    ConstraintCreated {
        doc: DocId,
        sketch: EntityId,
        constraint: u64,
        /// Whether the solver satisfied every constraint after this change.
        solved: bool,
        /// Worst constraint violation remaining (mm-ish units).
        residual: f64,
        /// Approximate degrees of freedom left (vars − equations).
        dof: i64,
    },
    ConstraintRemoved { doc: DocId, sketch: EntityId, constraint: u64 },
    SketchPointMoved { doc: DocId, sketch: EntityId, element: u64 },
    /// A sketch was turned into a solid (extrude/revolve). `volume` is the
    /// enclosed volume in mm³ — a quick printability sanity check.
    SolidCreated {
        doc: DocId,
        entity: EntityId,
        sketch: EntityId,
        volume: f64,
        tris: usize,
    },
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DocInfo {
    pub doc: DocId,
    pub name: String,
    pub entity_count: usize,
}

/// Reference to a point (or a whole element) inside a sketch.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct SketchRef {
    /// Sketch element id (from add_sketch_line / add_sketch_circle).
    pub element: u64,
    /// `Some(0)` = line start / circle center, `Some(1)` = line end,
    /// `None` = the whole element (its direction, for line constraints).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub point: Option<u8>,
}

/// Which relationship a constraint enforces.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ConstraintKind {
    Coincident,
    Horizontal,
    Vertical,
    Parallel,
    Perpendicular,
    Distance,
    Radius,
    Diameter,
}

impl ConstraintKind {
    pub fn as_str(&self) -> &'static str {
        match self {
            Self::Coincident => "coincident",
            Self::Horizontal => "horizontal",
            Self::Vertical => "vertical",
            Self::Parallel => "parallel",
            Self::Perpendicular => "perpendicular",
            Self::Distance => "distance",
            Self::Radius => "radius",
            Self::Diameter => "diameter",
        }
    }
}

/// Which in-plane axis a revolve turns around.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum RevolveAxis {
    /// The sketch's local X axis (profile revolves in the XZ plane).
    X,
    /// The sketch's local Y axis (profile revolves in the YZ plane).
    Y,
}

impl RevolveAxis {
    pub fn as_str(&self) -> &'static str {
        match self {
            Self::X => "x",
            Self::Y => "y",
        }
    }
}

/// A 2D element of a sketch. Coordinates are sketch-local (mm); the
/// sketch entity's transform places them in 3D.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum SketchElement {
    Line { id: u64, p1: [f64; 2], p2: [f64; 2] },
    Circle { id: u64, center: [f64; 2], radius: f64 },
}

/// One constraint as stored in the sketch (value already resolved).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SketchConstraint {
    pub id: u64,
    pub kind: ConstraintKind,
    pub refs: Vec<SketchRef>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub value: Option<f64>,
}

/// Solver status + contents of a sketch, as reported in snapshots.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SketchSnapshot {
    pub elements: Vec<SketchElement>,
    pub constraints: Vec<SketchConstraint>,
    /// Whether the solver satisfied every constraint.
    pub solved: bool,
    /// Worst remaining violation across constraints.
    pub residual: f64,
    /// Approximate degrees of freedom left (variables − equations).
    pub dof: i64,
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
    /// For sketch entities: elements, constraints and solver status. The
    /// `mesh` of a sketch holds display ribbons, not printable geometry.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub sketch: Option<SketchSnapshot>,
}
