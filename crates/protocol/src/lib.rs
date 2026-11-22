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

/// Magic value in a saved document's `format` field.
pub const DOCUMENT_FORMAT: &str = "agcad-document";
/// File format version written by this build. Bump when the on-disk shape
/// changes incompatibly; readers reject versions they do not know.
///
/// 2: `add_sketch` may carry a `plane` (a reader that ignores it would
/// silently place the sketch on XY, so the change is not downgrade-safe).
pub const DOCUMENT_VERSION: u32 = 2;

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
/// Session commands (initialize, undo/redo, get_scene, export_stl,
/// export_3mf, ...)
/// are not part of the program.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "method", rename_all = "snake_case")]
pub enum Command {
    Initialize,
    ListDocuments,
    CreateDocument {
        name: String,
        /// Optional explicit id (≥ 1, unused). Lets a single batch create a
        /// document and populate it: `create_document{doc:10}` followed by
        /// commands addressing doc 10. Omit it to allocate the next id.
        #[serde(default)]
        doc: Option<DocId>,
    },
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
        /// Extruded along local +Z (Z is up, like every primitive).
        height: Scalar,
        #[serde(default)]
        segments: Option<u32>,
        #[serde(default)]
        color: Option<Rgba>,
    },

    /// Round the selected sharp edges of one solid. The entity keeps its
    /// id, name, color and transform; its mesh is replaced by the filleted
    /// result (in the same local frame). Convex edges are cut back to a
    /// cylindrical surface of `radius`, concave edges are filled in.
    /// Replays with the program, so a parameter change re-fillets the new
    /// geometry.
    Fillet {
        doc: DocId,
        entity: EntityId,
        /// Fillet radius (mm). May be an expression.
        radius: Scalar,
        /// Which sharp edges to round. Omit for every sharp edge.
        #[serde(default)]
        edges: Option<EdgeFilter>,
        /// Only edges whose adjacent faces differ by at least this much
        /// count as sharp (degrees, default 30). Tessellation seams of
        /// curved surfaces are below this, real edges above it.
        #[serde(default)]
        min_angle_deg: Option<f64>,
    },

    /// Bevel the selected sharp edges of one solid with a flat face: each
    /// adjacent face is set back by `distance` from the edge. Convex edges
    /// lose material, concave edges gain it. Same entity semantics as
    /// [`Command::Fillet`].
    Chamfer {
        doc: DocId,
        entity: EntityId,
        /// Setback along each adjacent face (mm). May be an expression.
        distance: Scalar,
        #[serde(default)]
        edges: Option<EdgeFilter>,
        #[serde(default)]
        min_angle_deg: Option<f64>,
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

    /// Cast a world-space ray at the document's solids and report what it
    /// hits, with a ready-made geometric selector for the face and (when
    /// the hit is near one) the closest sharp edge. A session command: it
    /// changes no state and is not part of the program. Sketches are
    /// display-only and are skipped, so a sketch ribbon never blocks a
    /// pick.
    Pick {
        doc: DocId,
        ray: Ray,
        /// Edge snap radius in mm. A hit within this distance of a sharp
        /// edge also reports an `edge` selector. Defaults to 1% of the
        /// picked entity's bbox diagonal (min 0.01 mm). The viewer passes
        /// its click radius in pixels converted to world units.
        #[serde(default)]
        tolerance: Option<f64>,
    },

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

    /// Write a 3MF package of the selected entities' **world-space**
    /// geometry (default: every entity in the document). Unlike STL, the
    /// build is multi-object and keeps per-object names and colors, with
    /// the model's unit set to millimeters. A session command, not part of
    /// the program — exporting changes no state.
    #[serde(rename = "export_3mf")]
    Export3mf {
        doc: DocId,
        /// Filesystem path the *server* writes to; absolute paths avoid
        /// surprises about the server's working directory.
        path: String,
        #[serde(default)]
        entities: Option<Vec<EntityId>>,
    },

    /// Write the document to a `.agcad` file: its program plus the current
    /// parameter values (see [`DocumentSnapshot`]). Geometry is not stored;
    /// opening replays the program. The file is pretty-printed and
    /// byte-stable, so save → open → save is a no-op diff.
    ///
    /// Saving changes no model state. A provided `source` is remembered on
    /// the document, so `get_program` can hand the recipe back without a
    /// reopen; it is never executed.
    SaveDocument {
        doc: DocId,
        path: String,
        /// Optional authoring source to embed. It rides along as the
        /// canonical recipe; opening a document never executes it.
        #[serde(default)]
        source: Option<SourceDoc>,
    },

    /// Open a saved document as a new document (new id) and replay its
    /// program. Returns the new id and counts. The stored name wins unless
    /// `name` is given.
    OpenDocument {
        path: String,
        #[serde(default)]
        name: Option<String>,
    },

    /// Read-only view of the document's program and parameters — what a
    /// save would write. Useful for clients that render their own language
    /// (e.g. the Python client's `export_python`); the engine stays
    /// language-agnostic.
    GetProgram { doc: DocId },

    /// Check the document's printable solids and sketches. For every solid
    /// this reports watertightness (open/non-manifold edges), degenerate
    /// triangles, self-intersections, signed world volume, world bbox and an
    /// approximate thinnest wall; for every sketch it reports solver status
    /// and whether the profile is closed. A session command: it changes no
    /// state and is not part of the program.
    ///
    /// `ok` is the short answer: every solid is closed, manifold,
    /// self-intersection free, has positive volume and meets `min_wall_mm`
    /// when given; every sketch is solved and (if it has elements) closed.
    Validate {
        doc: DocId,
        /// Restrict the check to these entities. Omit for the whole scene.
        #[serde(default)]
        entities: Option<Vec<EntityId>>,
        /// Flag solids whose estimated thinnest wall is below this value
        /// (mm). Omit to report the estimate without failing on it.
        #[serde(default)]
        min_wall_mm: Option<f64>,
    },

    /// Compare two revisions by entity signature. The baseline is either
    /// another open document (`against`) or a saved `.agcad` file (`path`),
    /// which is replayed privately and discarded — exactly one must be given.
    /// The diff classifies entities as added, removed, modified (with the
    /// changed fields and volume/triangle deltas) or unchanged, and includes
    /// parameter and program changes. A session command: it changes no state.
    DiffScene {
        doc: DocId,
        /// Compare against this open document.
        #[serde(default)]
        against: Option<DocId>,
        /// Compare against this `.agcad` file. Read-only: the document is
        /// not opened into the document list.
        #[serde(default)]
        path: Option<String>,
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
    /// by the entity transform. `plane` chooses where it sits — a standard
    /// datum plane (the default is XY at the origin), an explicit
    /// origin/normal, or a planar face of an existing solid. Sketches are
    /// recorded in the program like any other modeling command, so they
    /// replay parametrically — coordinates and constraint values may be
    /// expressions over parameters.
    AddSketch {
        doc: DocId,
        name: String,
        /// Placement frame. Omit for the XY datum plane at the origin.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        plane: Option<SketchPlane>,
    },

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
    ///
    /// On a sketch placed on a solid face, the sketch normal is the face's
    /// outward normal: a positive distance adds material in front of the
    /// face and a negative one cuts into the body (use `boolean{difference}`
    /// to remove the cutter).
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

/// Which sharp edges a fillet/chamfer applies to, in **world space**.
///
/// Every condition that is set must hold (logical AND); with no conditions
/// every sharp edge is selected. Selection is geometric on purpose: entity
/// ids and edge indices are stable across replay, but a parameter change
/// moves edges, and "the edges parallel to Z" keeps meaning the same thing.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
#[serde(default)]
pub struct EdgeFilter {
    /// Keep edges parallel to this direction (either orientation).
    pub direction: Option<[f64; 3]>,
    /// Angular tolerance for `direction` and `normals`, in degrees
    /// (default 1).
    pub angle_tol_deg: Option<f64>,
    /// Keep edges whose midpoint is within `near_radius` of this point.
    pub near: Option<[f64; 3]>,
    /// Radius around `near` (mm). Required together with `near`.
    pub near_radius: Option<f64>,
    /// Keep edges that touch a face with one of these outward normals.
    /// One normal selects every edge of that face — `[[0,0,1]]` is "the
    /// whole top rim". Two select the edge between exactly those faces, in
    /// either order — `[[0,0,1],[1,0,0]]` is "where the top meets the +X
    /// side".
    pub normals: Option<Vec<[f64; 3]>>,
    /// Keep only convex (`true`) or concave (`false`) edges.
    pub convex: Option<bool>,
}

/// A world-space ray. `direction` is normalized by the engine, so the
/// reported hit distance is in millimeters. A ray that misses is a valid
/// result (`hit: null`), not an error.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Ray {
    pub origin: Vec3,
    /// Non-zero; need not be unit length.
    pub direction: Vec3,
}

/// The result of a [`Command::Pick`]: the nearest solid hit, or `None`.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PickHit {
    pub entity: EntityId,
    pub name: String,
    /// Distance along the ray in mm.
    pub distance: f64,
    /// The world-space hit point on the solid surface.
    pub point: Vec3,
    /// Outward normal of the hit face (the whole planar patch, not just
    /// the hit triangle).
    pub normal: Vec3,
    /// Immediately usable as `add_sketch{plane: face}`.
    pub face: FaceSelector,
    /// Present when the hit landed within the snap tolerance of a sharp
    /// edge; immediately usable as `fillet`/`chamfer`'s `edges` filter.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub edge: Option<EdgeSelector>,
}

/// A planar face reference, shaped exactly like
/// [`SketchPlane::Face`] so it can be passed straight back to the engine.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct FaceSelector {
    pub entity: EntityId,
    /// Outward face normal in world space.
    pub normal: Vec3,
    /// A world-space point on the face; selects between parallel faces
    /// and becomes the sketch origin (projected onto the face plane).
    pub near: Vec3,
}

/// A sharp-edge reference, shaped exactly like [`EdgeFilter`] so it can be
/// passed straight to `fillet`/`chamfer` as `edges`.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct EdgeSelector {
    /// Edge direction in world space.
    pub direction: Vec3,
    /// Midpoint of the picked edge segment.
    pub near: Vec3,
    /// A quarter of the edge length plus the snap tolerance. `near` is
    /// exact, so this radius also tracks the edge through modest parameter
    /// changes without swallowing the parallel edge on the other side of a
    /// thin wall.
    pub near_radius: f64,
}

/// Validation of one printable solid (see [`Command::Validate`]).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SolidValidation {
    pub entity: EntityId,
    pub name: String,
    /// `box`, `sphere`, `cylinder`, `boolean`, `extrude` or `revolve`.
    pub kind: String,
    /// Watertight, manifold, no self-intersections, positive volume and (when
    /// a threshold was requested) not thinner than it.
    pub ok: bool,
    pub watertight: bool,
    /// Edges used by exactly one triangle — a hole in the surface.
    pub open_edges: usize,
    /// Edges used by more than two triangles.
    pub non_manifold_edges: usize,
    /// Zero-area triangles (tolerated on tangent seams, reported for trust).
    pub degenerate_tris: usize,
    /// Pairs of non-adjacent triangles that intersect. A cheap check with a
    /// deterministic spatial sweep; 0 means none found.
    pub self_intersections: usize,
    /// Up to a few representative locations (midpoints between the offending
    /// triangle centroids), for diagnosis.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub self_intersection_points: Vec<Vec3>,
    /// Signed volume in world space (mm³); positive means outward winding.
    pub volume: f64,
    pub tris: usize,
    pub bbox: Bbox,
    /// Approximate thinnest wall in mm, from inward ray casts at sampled
    /// triangles. `None` when the mesh is not closed or no opposite surface
    /// was found.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub min_wall: Option<f64>,
    /// Where the thinnest wall was measured (a surface point), if any.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub min_wall_at: Option<Vec3>,
    /// Whether `min_wall` fell below the requested `min_wall_mm` threshold.
    pub thin: bool,
    /// Content signature (name, placement, color and geometry).
    pub signature: String,
}

/// Validation of one sketch: solver status plus whether it can be extruded.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SketchValidation {
    pub entity: EntityId,
    pub name: String,
    /// Solved, and closed when it has any elements. An empty sketch is a
    /// valid scratch drawing, not a failure.
    pub ok: bool,
    pub solved: bool,
    /// Worst remaining constraint violation.
    pub residual: f64,
    /// Approximate degrees of freedom left.
    pub dof: i64,
    pub elements: usize,
    pub constraints: usize,
    /// Whether the elements form a closed profile that extrude/revolve can
    /// consume. Empty sketches are not closed.
    pub closed_profile: bool,
    pub loops: usize,
    /// Why the profile is not closed, when it has elements but cannot be
    /// extruded.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub profile_error: Option<String>,
    pub signature: String,
}

/// Document-level totals for [`Command::Validate`].
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ValidationTotals {
    pub solids: usize,
    pub sketches: usize,
    pub tris: usize,
    /// Sum of the solids' signed world volumes (mm³).
    pub volume: f64,
    pub open_edges: usize,
    pub non_manifold_edges: usize,
    pub self_intersections: usize,
    pub degenerate_tris: usize,
    /// Thinnest wall found across every solid.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub min_wall: Option<f64>,
    /// Union of every solid's world bbox.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub bbox: Option<Bbox>,
}

/// Compact entity description used by [`Command::DiffScene`].
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EntitySummary {
    pub entity: EntityId,
    pub name: String,
    /// `box`, `sphere`, `cylinder`, `boolean`, `sketch`, `extrude` or `revolve`.
    pub kind: String,
    pub signature: String,
    /// Signed world volume (mm³); `None` for sketches.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub volume: Option<f64>,
    /// Triangle count; `None` for sketches (their mesh is display ribbons).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub tris: Option<usize>,
    pub world_bbox: Bbox,
}

/// One entity that exists in both revisions but differs.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EntityChange {
    pub entity: EntityId,
    /// The changed fields, in a stable order: `name`, `color`, `kind`,
    /// `translation`, `rotation_deg`, `scale`, `mesh`, `sketch`.
    pub changed: Vec<String>,
    pub from: EntitySummary,
    pub to: EntitySummary,
}

/// One parameter whose resolved value differs between revisions.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ParameterChange {
    pub name: String,
    /// `None` when the parameter did not exist in the baseline (`to` is set),
    /// or was removed from the current revision (`from` is set).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub from: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub to: Option<f64>,
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
    /// A solid was reshaped in place (fillet/chamfer). `edges` is how many
    /// sharp edges the selection matched, `volume` the new world-space
    /// volume in mm³, and `mesh` the new local mesh.
    EntityModified {
        doc: DocId,
        entity: EntityId,
        edges: u32,
        volume: f64,
        mesh: Mesh,
    },
    Transformed { doc: DocId, entity: EntityId },
    Deleted { doc: DocId, entity: EntityId },
    ParameterSet { doc: DocId, name: String, value: f64 },
    ParameterRemoved { doc: DocId, name: String },
    Parameters { doc: DocId, params: Vec<ParamInfo> },
    Undone { doc: DocId },
    Redone { doc: DocId },
    Scene { doc: DocId, entities: Vec<EntitySnapshot> },
    /// A ray query (see [`Command::Pick`]). `hit` is `None` when the ray
    /// missed every solid; that is a valid answer, not an error.
    Picked { doc: DocId, hit: Option<PickHit> },
    /// Trust report for the document's solids and sketches (see
    /// [`Command::Validate`]). `ok` is false when any solid is not printable
    /// or any sketch is unsolved/open; the per-entity reports say why.
    Validated {
        doc: DocId,
        ok: bool,
        solids: Vec<SolidValidation>,
        sketches: Vec<SketchValidation>,
        totals: ValidationTotals,
        #[serde(default, skip_serializing_if = "Vec::is_empty")]
        warnings: Vec<String>,
    },
    /// Revision comparison by entity signature (see [`Command::DiffScene`]).
    SceneDiff {
        doc: DocId,
        /// The baseline document id, when compared with `against`.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        against: Option<DocId>,
        /// Baseline name (document name or `.agcad` path).
        against_name: String,
        /// True when anything differs (entities, parameters or program).
        changed: bool,
        /// Content fingerprint of the current scene.
        scene_signature: String,
        /// Content fingerprint of the baseline scene.
        against_signature: String,
        added: Vec<EntitySummary>,
        removed: Vec<EntitySummary>,
        modified: Vec<EntityChange>,
        /// Entities present in both revisions with identical signatures.
        unchanged: usize,
        parameters: Vec<ParameterChange>,
        program_changed: bool,
        commands_from: usize,
        commands_to: usize,
    },
    StlExported { doc: DocId, path: String, tris: usize, bytes: usize },
    /// A 3MF package was written. `objects` is how many build items it
    /// contains, `tris` the total triangle count.
    #[serde(rename = "3mf_exported")]
    ThreeMfExported {
        doc: DocId,
        path: String,
        objects: usize,
        tris: usize,
        bytes: usize,
    },
    DocumentSaved {
        doc: DocId,
        path: String,
        bytes: usize,
        commands: usize,
        params: usize,
    },
    DocumentOpened {
        doc: DocId,
        name: String,
        commands: usize,
        params: usize,
        entities: usize,
    },
    Program {
        doc: DocId,
        name: String,
        params: Vec<ParamInfo>,
        program: Vec<Command>,
        /// The authoring source embedded in the document, when it was saved
        /// with one. The script on disk is the source of truth; this is the
        /// copy that rode along in the `.agcad` snapshot, so a handoff can
        /// recover the recipe without executing anything.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        source: Option<SourceDoc>,
    },
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

/// The on-disk form of a document (`.agcad`, JSON).
///
/// It stores the *parametric function* — the program and the current
/// parameter values — and nothing derived. Geometry is rebuilt by replaying
/// the program on open, which is deterministic (same program + parameters →
/// identical ids and meshes, bit for bit). Storing meshes would let the file
/// drift from the program; not storing them keeps the file small and
/// diffable.
///
/// `program` commands are normalized to `doc: 1` on write and rebound to the
/// document's real id on open, so save → open → save is byte-identical.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DocumentSnapshot {
    /// Must equal [`DOCUMENT_FORMAT`].
    pub format: String,
    /// File format version ([`DOCUMENT_VERSION`] when written).
    pub version: u32,
    /// Protocol version that produced the file (diagnostics).
    #[serde(default)]
    pub protocol: u32,
    pub name: String,
    /// Current parameter values, sorted by name.
    #[serde(default)]
    pub parameters: std::collections::BTreeMap<String, SavedParameter>,
    /// Modeling commands in execution order.
    #[serde(default)]
    pub program: Vec<Command>,
    /// Authoring source: the canonical recipe that produced this instance.
    /// Opening a document replays `program`; it never executes `source`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub source: Option<SourceDoc>,
}

/// One parameter in a saved document: the engine stores the resolved value.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SavedParameter {
    pub value: f64,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub description: Option<String>,
}

/// The authoring script for a document: the durable source of truth. It rides
/// along in the saved `.agcad` so a handoff can recover the recipe; opening a
/// document never executes it.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SourceDoc {
    pub language: String,
    pub text: String,
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

/// A standard datum plane. Each is right-handed with the listed local axes,
/// so the sketch's local +Z is the extrusion direction:
///
/// * `xy` — X = world +X, Y = +Y, normal = +Z
/// * `xz` — X = world +X, Y = +Z, normal = -Y (the front view looks along +Y)
/// * `yz` — X = world +Y, Y = +Z, normal = +X
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DatumPlane {
    Xy,
    Xz,
    Yz,
}

impl DatumPlane {
    pub fn as_str(&self) -> &'static str {
        match self {
            Self::Xy => "xy",
            Self::Xz => "xz",
            Self::Yz => "yz",
        }
    }
}

/// Where a new sketch plane sits, in world space.
///
/// Wire forms (externally tagged):
///
/// ```json
/// {"datum": {"plane": "xy", "offset": 5}}
/// {"custom": {"origin": [0,0,10], "normal": [0,0,1], "x_axis": [1,0,0]}}
/// {"face": {"entity": 1, "normal": [0,0,1], "near": [0,0,10]}}
/// ```
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SketchPlane {
    /// A standard datum plane, optionally offset along its normal.
    Datum {
        plane: DatumPlane,
        /// Offset along the plane normal (mm). May be an expression.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        offset: Option<Scalar>,
    },
    /// Any plane given by a world-space origin and normal. The sketch's
    /// local +X is derived (world +X projected onto the plane, or world +Y
    /// when the plane is nearly perpendicular to X) unless `x_axis` is
    /// given; it is projected onto the plane either way.
    Custom {
        origin: SVec3,
        /// Non-zero plane normal (does not need to be unit length).
        normal: [f64; 3],
        #[serde(default, skip_serializing_if = "Option::is_none")]
        x_axis: Option<[f64; 3]>,
    },
    /// A planar face of an existing solid. The sketch normal is the face's
    /// outward normal, so a positive extrude adds material in front of the
    /// face and a negative one cuts into the body. The in-plane X follows
    /// the same world-axis rule as [`SketchPlane::Custom`] — a plane has no
    /// intrinsic X, so a rotated body still gets a stable, predictable one.
    Face {
        entity: EntityId,
        /// World-space point used to pick between parallel faces with the
        /// same normal. The sketch origin is this point projected onto the
        /// chosen face plane; omit it to use the face's center. May be an
        /// expression, so the origin can follow a parameter.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        near: Option<SVec3>,
        /// Required outward normal of the face (world space). Omit to pick
        /// the largest matching face.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        normal: Option<[f64; 3]>,
        /// Angular tolerance for `normal`, in degrees (default 1).
        #[serde(default, skip_serializing_if = "Option::is_none")]
        angle_tol_deg: Option<f64>,
    },
}

/// The resolved placement of a sketch, reported in snapshots. Sketch-local
/// coordinates map to world as `origin + x_axis*x + (normal × x_axis)*y`;
/// `normal` is the extrude direction.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct SketchPlaneFrame {
    pub origin: Vec3,
    pub normal: Vec3,
    pub x_axis: Vec3,
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
    /// Resolved world-space plane: origin, outward normal (the extrude
    /// direction) and in-plane X. Derived from the entity transform.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub plane: Option<SketchPlaneFrame>,
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

/// Axis-aligned bounding box, in millimeters.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Bbox {
    pub min: Vec3,
    pub max: Vec3,
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
    /// Bounds of the mesh **after** `transform`, i.e. world space. The
    /// `mesh.positions` inside are local; trust this for placement and size.
    pub world_bbox: Bbox,
    pub color: Rgba,
    pub mesh: Mesh,
    /// For sketch entities: elements, constraints and solver status. The
    /// `mesh` of a sketch holds display ribbons, not printable geometry.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub sketch: Option<SketchSnapshot>,
}
