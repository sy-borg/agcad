//! # wright-protocol
//!
//! Wire types shared by every wright component: core, server, viewer and
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
pub const SERVER_NAME: &str = "wright";

/// Magic value in a saved document's `format` field.
pub const DOCUMENT_FORMAT: &str = "wright-document";
/// File format version written by this build. Bump when the on-disk shape
/// changes incompatibly; readers reject versions they do not know.
///
/// 2: `add_sketch` may carry a `plane` (a reader that ignores it would
/// silently place the sketch on XY, so the change is not downgrade-safe).
///
/// 3: M16 shape-leverage commands (`linear_pattern`, `circular_pattern`,
/// `mirror`, `shell`, `hole`, `sweep`, `loft`, `thread`). Older readers fail
/// to parse them, so the change is not downgrade-safe either.
///
/// 4: M19 assembly commands (`add_part`, `add_instance`, `add_connector`,
/// `mate`, `move_instance`). `add_part` embeds a complete part document in
/// the program, so an assembly is self-contained just like an imported mesh;
/// older readers cannot parse the step and would silently drop the parts.
///
/// 5: M21 program editing. Pattern `count` may be an expression (older
/// readers reject a string there), and the snapshot carries the suppressed
/// program steps. An older reader would run suppressed features, so the
/// omission is not downgrade-safe.
///
/// 6: M22 import → parametric. New program steps (`add_sketch_curves`,
/// `section`, `project`, `repair_mesh`) carry imported geometry as data, so
/// an older reader would silently drop the sketch/profile or leave a mesh
/// unrepaired.
pub const DOCUMENT_VERSION: u32 = 6;

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

/// One edit to a document's program (see [`Command::PatchProgram`]).
///
/// Steps are 0-based indexes into the current program, exactly as
/// `get_program` returns them. Ops are applied in order to a working copy, so
/// later indexes see the effect of earlier ones. Commands inserted or
/// replaced address entities by their **current** ids; the engine replays the
/// patch and rewrites every reference when id allocation shifts, so a program
/// keeps its meaning.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "op", rename_all = "snake_case")]
pub enum ProgramOp {
    /// Insert `command` before step `at` (`at == steps` appends).
    Insert { at: usize, command: Command },
    /// Remove the step at `at`.
    Remove { at: usize },
    /// Replace the step at `at`. The step keeps its suppression flag and is
    /// treated as the same feature for id remapping, so a downstream fillet
    /// follows the replacement.
    Replace { at: usize, command: Command },
    /// Move the step at `from` so it ends up at index `to` (0-based, in the
    /// patched program). Suppression follows the step.
    Reorder { from: usize, to: usize },
}

/// Every operation the bus understands. `method` is the JSON-RPC method
/// name; all remaining variant fields live inside `params`.
///
/// Modeling commands (Add*, Transform, SetColor, Delete, Boolean,
/// AddInstance, Mate, ...) are appended to the document program and replayed
/// when parameters change. Session commands (initialize, undo/redo, get_scene,
/// export_stl, export_3mf, export_step, export_dxf, export_svg,
/// export_drawing, bill_of_materials, check_interference, ...) are not part of
/// the program.
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

    /// Read a triangle mesh (STL binary or ASCII, OBJ, or 3MF) from the
    /// server's filesystem and add it as a solid entity.
    ///
    /// Coordinates are millimeters: 3MF's declared unit is converted and
    /// `scale` multiplies the result. The parsed geometry is recorded as an
    /// [`Command::AddMesh`] program step, so a `.wcad` save replays without
    /// the source file. The importer welds vertices by exact position, drops
    /// degenerate and duplicate triangles, flips a globally inward winding,
    /// and computes crease-aware shading normals. A non-watertight import is
    /// still allowed — `validate` reports it, and booleans may reject it.
    ImportMesh {
        doc: DocId,
        /// Path the *server* reads; absolute paths avoid surprises about the
        /// server's working directory.
        path: String,
        /// Entity name; defaults to the file stem.
        #[serde(default)]
        name: Option<String>,
        /// Uniform scale applied after unit conversion (default 1).
        #[serde(default)]
        scale: Option<f64>,
        /// Force a format instead of detecting it from the extension and
        /// contents.
        #[serde(default)]
        format: Option<MeshFormat>,
        /// Crease angle for shading normals in degrees (default 30).
        /// Adjacent faces meeting at a sharper angle keep a hard edge.
        #[serde(default)]
        smooth_angle_deg: Option<f64>,
        #[serde(default)]
        color: Option<Rgba>,
    },

    /// Add a solid from raw triangles — the program step `import_mesh`
    /// records. `positions` are millimeters in the source frame; like every
    /// other solid, the mesh is baked around its bounding-box center and the
    /// entity translation restores the world position. `indices` is flat,
    /// three per triangle. The same cleanup as import applies (welding,
    /// degenerate/duplicate removal, outward winding).
    AddMesh {
        doc: DocId,
        name: String,
        positions: Vec<[f32; 3]>,
        indices: Vec<u32>,
        /// Crease angle for shading normals in degrees (default 30).
        #[serde(default)]
        smooth_angle_deg: Option<f64>,
        #[serde(default)]
        color: Option<Rgba>,
    },

    /// Read a 2D DXF drawing from the server's filesystem and bring its
    /// curves in as a sketch (M22). Lines stay lines, circles and arcs stay
    /// analytic, and polylines become connected line segments; the result is
    /// recorded as an `add_sketch_curves` program step, so a `.wcad` save
    /// replays without the source file. `auto_constraints` (default true)
    /// adds coincident, horizontal/vertical and approximate-tangency
    /// constraints so the imported drawing behaves like authored geometry
    /// when a knob changes.
    ImportDxf {
        doc: DocId,
        /// Path the *server* reads; absolute paths avoid surprises.
        path: String,
        /// Sketch name; defaults to the file stem.
        #[serde(default)]
        name: Option<String>,
        /// Placement frame (default: the XY datum plane).
        #[serde(default)]
        plane: Option<SketchPlane>,
        /// Uniform scale applied to the file's coordinates (default 1).
        /// DXF `$INSUNITS` is converted to millimeters automatically.
        #[serde(default)]
        scale: Option<f64>,
        /// Add inferred constraints (default true).
        #[serde(default)]
        auto_constraints: Option<bool>,
    },

    /// Read an SVG drawing from the server's filesystem and bring its curves
    /// in as a sketch (M22). Supports `line`, `circle`, `ellipse`, `rect`,
    /// `polyline`, `polygon` and `path` (M/L/H/V/C/S/Q/T/A/Z), nested group
    /// transforms, and common `width`/`height`/`viewBox` units. SVG's Y axis
    /// points down, so geometry is flipped to the sketch's Y-up frame.
    ImportSvg {
        doc: DocId,
        /// Path the *server* reads; absolute paths avoid surprises.
        path: String,
        #[serde(default)]
        name: Option<String>,
        #[serde(default)]
        plane: Option<SketchPlane>,
        /// Uniform scale applied after the SVG unit conversion (default 1).
        #[serde(default)]
        scale: Option<f64>,
        #[serde(default)]
        auto_constraints: Option<bool>,
    },

    /// Create a sketch from a complete set of elements and constraints — the
    /// program step `import_dxf`/`import_svg` records. Elements must carry
    /// their own ids (1-based by convention); constraints reference them.
    /// The sketch is solved on creation like any other.
    AddSketchCurves {
        doc: DocId,
        name: String,
        #[serde(default)]
        plane: Option<SketchPlane>,
        elements: Vec<SketchElement>,
        /// Inferred or explicit constraints, referencing the element ids.
        #[serde(default, skip_serializing_if = "Vec::is_empty")]
        constraints: Vec<SketchConstraint>,
    },

    /// Cut a solid or imported mesh with a plane and put the intersection
    /// curves into a new sketch (M22). Closed loops come out as connected
    /// line runs, so the sketch can be extruded straight away. A modeling
    /// command: it replays with the program, and parameter changes regenerate
    /// the section from the rebuilt solid.
    Section {
        doc: DocId,
        entity: EntityId,
        #[serde(default)]
        name: Option<String>,
        /// Cutting plane (default: the XY datum plane).
        #[serde(default)]
        plane: Option<WorkPlane>,
    },

    /// Project a solid or imported mesh onto a plane as a sketch (M22).
    /// Boundary, sharp and silhouette edges are projected, giving an outline
    /// an agent can dimension, offset or extrude. A modeling command.
    Project {
        doc: DocId,
        entity: EntityId,
        #[serde(default)]
        name: Option<String>,
        /// Projection plane (default: the XY datum plane). The plane normal
        /// is the viewing direction.
        #[serde(default)]
        plane: Option<WorkPlane>,
        /// Sharp-edge threshold in degrees (default 30).
        #[serde(default)]
        min_angle_deg: Option<f64>,
    },

    /// Enumerate the analytic primitives of a solid or imported mesh (M22):
    /// planar faces, cylindrical surfaces (axis + radius, inner or outer) and
    /// the hole candidates those inner cylinders describe. A session command:
    /// it changes no state and is not part of the program. Use it to re-author
    /// an imported part parametrically instead of only booleaning onto it.
    Primitives { doc: DocId, entity: EntityId },

    /// Repair a triangle mesh in place (M22): weld and clean, orient each
    /// connected shell consistently and outward, drop triangles that make an
    /// edge non-manifold, and fill closed boundary loops. The entity keeps
    /// its id, name, color and transform; a modeling command, so the repair
    /// replays parametrically and can make a non-watertight import safe for
    /// booleans.
    RepairMesh {
        doc: DocId,
        entity: EntityId,
        /// Fill closed boundary loops (default true).
        #[serde(default)]
        fill_holes: Option<bool>,
        /// Re-orient each connected shell consistently and outward
        /// (default true).
        #[serde(default)]
        fix_winding: Option<bool>,
        /// Drop triangles that make an edge belong to more than two
        /// triangles (default true).
        #[serde(default)]
        remove_non_manifold: Option<bool>,
        /// Crease angle for shading normals in degrees (default 30).
        #[serde(default)]
        smooth_angle_deg: Option<f64>,
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

    /// Read a part document and make it available to `add_instance` under
    /// `name`. Exactly one of `path` (a `.wcad` file on the server) or
    /// `from` (another open document) must be given. The part's complete
    /// program is embedded in the assembly program as an [`Command::AddPart`]
    /// step, so the assembly remains self-contained: the source file (or
    /// document) is not needed again, including after save/open. Nothing is
    /// added to the scene yet — call `add_instance` to place one.
    ImportPart {
        doc: DocId,
        /// Part name. Defaults to the document/file name.
        #[serde(default)]
        name: Option<String>,
        /// A `.wcad` file the *server* reads; absolute paths avoid surprises.
        #[serde(default)]
        path: Option<String>,
        /// Snapshot this open document as the part.
        #[serde(default)]
        from: Option<DocId>,
    },

    /// Register an embedded part definition. This is the self-contained
    /// program step `import_part` records; agents normally use
    /// `import_part`. Replaying a part document here never executes its
    /// optional authoring source.
    AddPart {
        doc: DocId,
        name: String,
        /// The part document (program + parameter values + source). It is
        /// replayed privately when an instance is placed.
        part: DocumentSnapshot,
    },

    /// Place a part in the assembly. Every printable solid of the part
    /// becomes an ordinary entity in this document, named
    /// `"<instance>.<member>"`, with `Shape::PartMember` linking it back to
    /// the instance. Sketches are display-only and are not copied. The
    /// instance's placement is rigid (rotation + uniform scale); `parameters`
    /// override the part's saved parameter values for this instance only.
    AddInstance {
        doc: DocId,
        /// A part registered by `import_part` / `add_part`.
        part: String,
        /// Assembly-unique instance name (also the member-name prefix).
        name: String,
        /// Placement of the part origin in assembly space. May be expressions.
        #[serde(default)]
        translation: Option<SVec3>,
        /// Extrinsic XYZ rotation in degrees. May be expressions.
        #[serde(default)]
        rotation_deg: Option<SVec3>,
        /// Uniform scale (default 1). May be an expression.
        #[serde(default)]
        scale: Option<Scalar>,
        /// Per-instance part parameter overrides. Values may be expressions
        /// over the *assembly* parameters (resolved before the part replays).
        #[serde(default)]
        parameters: Option<std::collections::BTreeMap<String, Scalar>>,
    },

    /// Add a named connector to an instance: a point (and optional axis) in
    /// instance-local coordinates. Mates reference connectors, so a screw's
    /// tip and a hole's mouth can be defined once and reused.
    AddConnector {
        doc: DocId,
        instance: u64,
        name: String,
        /// Connector point in instance-local coordinates. May be expressions.
        point: SVec3,
        /// Optional connector axis in instance-local coordinates (non-zero).
        /// Used by `distance`, `parallel` and `angle` mates.
        #[serde(default)]
        axis: Option<[f64; 3]>,
    },

    /// Move one instance (and every member entity) to a new placement. A
    /// modeling command, so it replays in program order.
    MoveInstance {
        doc: DocId,
        instance: u64,
        #[serde(default)]
        translation: Option<SVec3>,
        #[serde(default)]
        rotation_deg: Option<SVec3>,
        #[serde(default)]
        scale: Option<Scalar>,
    },

    /// Constrain instance `b` to instance `a` through two connectors, then
    /// move `b` (and all its members) to satisfy the mate. Mates are solved
    /// sequentially in program order; each one is a rigid motion of `b`.
    ///
    /// * `coincident` — `b`'s connector point goes to `a`'s point; when both
    ///   have axes they are anti-parallel (mating faces face each other).
    /// * `distance` — like coincident, with `b`'s point offset along `a`'s
    ///   axis by `offset` mm (both connectors need an axis).
    /// * `parallel` — only rotates `b` so its axis is parallel to `a`'s,
    ///   keeping the connector point where it is.
    /// * `angle` — puts `b`'s point on `a`'s point and rotates `b`'s axis to
    ///   `offset` degrees from `a`'s axis (offset 0 = parallel, 180 =
    ///   anti-parallel). The tilt direction is derived deterministically
    ///   from `a`'s axis.
    Mate {
        doc: DocId,
        kind: MateKind,
        a: MateRef,
        b: MateRef,
        /// Distance (mm) for `distance`, degrees for `angle`.
        #[serde(default)]
        offset: Option<Scalar>,
    },

    /// Bill of materials: one row per part used in the assembly, with the
    /// instance count, entity count and total world volume. A session
    /// command — no program entry, no state change.
    BillOfMaterials { doc: DocId },

    /// Check pairs of placed instances (default) or explicitly named solids
    /// for volumetric interference. Uses AABB pruning and the boolean
    /// kernel's intersection; reports the overlapping volume in mm³. A
    /// session command — no program entry, no state change.
    CheckInterference {
        doc: DocId,
        /// Instances to check against each other. Omit for every instance.
        /// Ignored when `entities` is given.
        #[serde(default)]
        instances: Option<Vec<u64>>,
        /// Solids to check pairwise. Defaults to cross-instance member pairs;
        /// use this to check arbitrary solids in one document.
        #[serde(default)]
        entities: Option<Vec<EntityId>>,
        /// Ignore overlaps at or below this volume (mm³, default 0.001).
        #[serde(default)]
        tolerance: Option<f64>,
    },

    /// Write a multi-view engineering drawing of the document (or assembly)
    /// as SVG or DXF. Views are orthographic projections with feature and
    /// silhouette edges, hidden-line removal, overall dimensions, part
    /// balloons and an optional BOM table + title block. Coordinates on the
    /// sheet are millimeters; `scale` scales the geometry but the dimension
    /// numbers stay true model lengths. A session command — no program entry,
    /// no state change.
    #[serde(rename = "export_drawing")]
    ExportDrawing {
        doc: DocId,
        path: String,
        #[serde(default)]
        format: DrawingFormat,
        /// Views and their sheet placement. Omit for front, top, right and
        /// an isometric view, auto-laid-out in third-angle style.
        #[serde(default)]
        views: Option<Vec<DrawingViewSpec>>,
        /// Drawing scale as drawn/model (default 1). Dimensions still show
        /// the real model length.
        #[serde(default)]
        scale: Option<f64>,
        /// Draw occluded edges as dashed hidden lines (default true).
        #[serde(default)]
        hidden_lines: Option<bool>,
        /// Add overall width/height dimensions to orthographic views
        /// (default true). A view can override this.
        #[serde(default)]
        dimensions: Option<bool>,
        /// Number part instances with BOM balloons in the first orthographic
        /// view (default: true when the document has instances).
        #[serde(default)]
        balloons: Option<bool>,
        /// Draw a BOM table above the title block (default: true when the
        /// document has instances).
        #[serde(default)]
        bom: Option<bool>,
        /// Sheet title; defaults to the document name.
        #[serde(default)]
        title: Option<String>,
        /// Free annotation text placed in sheet millimeters.
        #[serde(default)]
        annotations: Option<Vec<DrawingAnnotation>>,
    },

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

    /// Write a 2D ASCII DXF (R12) drawing of sketches. Coordinates are the
    /// **sketch-local** (u, v) millimeters — the drawing's own 2D frame, not
    /// world XYZ. `entities` defaults to every sketch; explicitly naming an
    /// extrude/revolve/sweep/loft solid exports its source sketch profile
    /// instead, so a part's footprint can be exported without knowing the
    /// sketch id. A session command, not part of the program.
    #[serde(rename = "export_dxf")]
    ExportDxf {
        doc: DocId,
        /// Filesystem path the *server* writes to; absolute paths avoid
        /// surprises about the server's working directory.
        path: String,
        #[serde(default)]
        entities: Option<Vec<EntityId>>,
        /// Which curves to write. Defaults to the solved sketch elements;
        /// `profiles` writes only the closed loops an extrude/revolve
        /// consumes, and `both` writes both on separate layers.
        #[serde(default)]
        curves: CurveMode,
    },

    /// Write an SVG of sketches, with the same selection and coordinate
    /// rules as [`Command::ExportDxf`]: sketch-local millimeters, a viewBox
    /// around the drawing, construction geometry dashed and profile loops
    /// on their own group. A session command, not part of the program.
    #[serde(rename = "export_svg")]
    ExportSvg {
        doc: DocId,
        /// Filesystem path the *server* writes to; absolute paths avoid
        /// surprises about the server's working directory.
        path: String,
        #[serde(default)]
        entities: Option<Vec<EntityId>>,
        #[serde(default)]
        curves: CurveMode,
    },

    /// Write a STEP (ISO 10303-21, AP214 `AUTOMOTIVE_DESIGN`) file of the
    /// selected entities' **world-space** geometry (default: every solid in
    /// the document). wright is a tessellation kernel, so the file is an
    /// explicit **faceted B-rep**: one planar `FACE_SURFACE` per triangle,
    /// welded `CARTESIAN_POINT`s shared between neighbours, and one
    /// `FACETED_BREP` per closed edge-manifold piece. Open or non-manifold
    /// pieces are written as `SHELL_BASED_SURFACE_MODEL`s instead of being
    /// rejected or mislabelled as solids. Analytic surfaces (true cylinders,
    /// cones, splines) are not reconstructed. A session command, not part of
    /// the program — exporting changes no state.
    #[serde(rename = "export_step")]
    ExportStep {
        doc: DocId,
        /// Filesystem path the *server* writes to; absolute paths avoid
        /// surprises about the server's working directory.
        path: String,
        #[serde(default)]
        entities: Option<Vec<EntityId>>,
    },

    /// Write the document to a `.wcad` file: its program plus the current
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

    /// Edit the program in place: insert, remove, replace or reorder steps.
    ///
    /// Commands inside the ops address entities by their current ids. The
    /// engine replays the patched program against the current parameters and
    /// rewrites references whenever id allocation moves, so an inserted step
    /// that shifts later ids does not silently retarget a downstream fillet.
    /// The result reports surviving and renumbered entity ids, entities the
    /// edit removed, entities it created, and (for a dry run) a
    /// [`SceneDiffReport`] against the pre-edit scene. A dry run changes
    /// nothing; a real patch is one undo step.
    ///
    /// `suppressed` steps are carried through the edit as step properties, and
    /// a removed step's flag disappears with it.
    PatchProgram {
        doc: DocId,
        ops: Vec<ProgramOp>,
        /// Replay the patched program and report, but do not commit.
        #[serde(default)]
        dry_run: bool,
    },

    /// Turn program steps on or off without deleting them. A suppressed step
    /// is skipped by replay but keeps its place, so a variant can be flipped
    /// back with `suppress{suppress:false}`. Entity ids before the suppressed
    /// step are unchanged; ids after it may shift and the result reports the
    /// remapping. Suppressing a feature that later steps use is rejected with
    /// the referencing step named (suppress those too for an A/B variant).
    Suppress {
        doc: DocId,
        /// 0-based program step indexes, as `get_program` returns them.
        steps: Vec<usize>,
        /// `true` turns suppression on (the default), `false` clears it.
        #[serde(default = "default_true")]
        suppress: bool,
    },

    /// Run several commands as one unit. With `atomic: true` a failing
    /// command rolls every document, parameter and program change back to the
    /// state before the batch (undo history included); files already written
    /// are not retracted. Results come back in execution order.
    Batch {
        commands: Vec<Command>,
        #[serde(default)]
        atomic: bool,
    },

    /// Enumerate the planar faces of one solid with readable descriptors,
    /// stable ids, fingerprints and ready-made `add_sketch` selectors. A
    /// session command: it changes no state and is not part of the program.
    ///
    /// Faces are listed in deterministic mesh order. `fingerprint` identifies
    /// the face across rebuilds (normal + rank among parallel faces);
    /// `geometry` is the exact vertex hash and changes when a parameter
    /// resizes the face. Use the selector to address a face without guessing
    /// a `near` point.
    Faces { doc: DocId, entity: EntityId },

    /// Enumerate the logical sharp edges of one solid: collinear or tangent
    /// runs are grouped, so a tessellated cylinder rim is one edge with 32
    /// segments rather than 32 separate candidates. Each group carries its
    /// endpoints/direction (or curve midpoint), adjacent face ids, a
    /// fingerprint and a ready-to-use `fillet`/`chamfer` selector. A session
    /// command: it changes no state and is not part of the program.
    Edges {
        doc: DocId,
        entity: EntityId,
        /// Sharp-edge threshold in degrees (default 30), the same value
        /// `fillet`/`chamfer` use.
        #[serde(default)]
        min_angle_deg: Option<f64>,
    },

    /// Check the document's printable solids and sketches. For every solid
    /// this reports watertightness (open/non-manifold edges), degenerate
    /// triangles, self-intersections, signed world volume, world bbox and an
    /// approximate thinnest wall; for every sketch it reports solver status
    /// and whether the profile is closed. Stored selectors are also checked
    /// against their authored binding (M20): `ok`, `moved`, `ambiguous`,
    /// `missing`, `drift` or `unbound`. A session command: it changes no
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
    /// another open document (`against`) or a saved `.wcad` file (`path`),
    /// which is replayed privately and discarded — exactly one must be given.
    /// The diff classifies entities as added, removed, modified (with the
    /// changed fields and volume/triangle deltas) or unchanged, and includes
    /// parameter and program changes plus selectors whose logical binding
    /// changed. A session command: it changes no state.
    DiffScene {
        doc: DocId,
        /// Compare against this open document.
        #[serde(default)]
        against: Option<DocId>,
        /// Compare against this `.wcad` file. Read-only: the document is
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
    /// units (mm), may be expressions. A construction line guides constraints
    /// but is excluded from the extrudable profile.
    AddSketchLine {
        doc: DocId,
        sketch: EntityId,
        p1: [Scalar; 2],
        p2: [Scalar; 2],
        /// Construction geometry: solved and displayed, but never part of a
        /// closed profile. Defaults to false.
        #[serde(default, skip_serializing_if = "is_false")]
        construction: bool,
    },

    /// Append a circle to a sketch.
    AddSketchCircle {
        doc: DocId,
        sketch: EntityId,
        center: [Scalar; 2],
        radius: Scalar,
        #[serde(default, skip_serializing_if = "is_false")]
        construction: bool,
    },

    /// Append a circular arc to a sketch. Angles are degrees, measured
    /// counter-clockwise from the sketch's local +X; the arc runs from
    /// `start_deg` to `end_deg` counter-clockwise (a sweep of 360 is a full
    /// circle, so use `add_sketch_circle` for that).
    AddSketchArc {
        doc: DocId,
        sketch: EntityId,
        center: [Scalar; 2],
        radius: Scalar,
        start_deg: Scalar,
        end_deg: Scalar,
        #[serde(default, skip_serializing_if = "is_false")]
        construction: bool,
    },

    /// Append an interpolating spline through the given points (Catmull-Rom;
    /// the curve passes through every point). Open by default: its two
    /// endpoints connect to other sketch elements like a line. An open spline
    /// needs at least 2 points, a closed one at least 3.
    AddSketchSpline {
        doc: DocId,
        sketch: EntityId,
        points: Vec<[Scalar; 2]>,
        #[serde(default, skip_serializing_if = "is_false")]
        closed: bool,
        #[serde(default, skip_serializing_if = "is_false")]
        construction: bool,
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
        /// coincident → two point refs; distance → two point refs or one
        /// point and one whole line; horizontal/vertical → one whole line;
        /// parallel/perpendicular/angle → two whole lines; radius/diameter →
        /// one circle/arc; equal → two lines or two curves; concentric → two
        /// curves; symmetric → point, point, whole line (the mirror axis);
        /// point_on_line → one point and one whole line; tangent → one line
        /// and one curve, or two curves.
        refs: Vec<SketchRef>,
        /// Required by distance/radius/diameter (mm) and angle (degrees).
        /// May be an expression.
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
        /// Point index: line 0=start,1=end; circle 0=center; arc
        /// 0=center,1=start,2=end; spline i=control point i.
        point: u8,
        to: [f64; 2],
    },

    /// Delete a sketch element; constraints referencing it are removed
    /// too (cascade), then the sketch re-solves.
    DeleteSketchElement { doc: DocId, sketch: EntityId, element: u64 },

    /// Trim a line/arc/circle at `at`: the piece containing that point is
    /// removed, bounded by its nearest intersections with other elements
    /// (construction guides included). Trim keeps the rest as one or two
    /// elements (new ids are returned); a circle with one cut becomes an arc.
    /// Constraints that still make sense for the kept geometry survive.
    /// Splines cannot be trimmed.
    TrimSketch {
        doc: DocId,
        sketch: EntityId,
        element: u64,
        /// A point near the piece to remove, in sketch-local coordinates.
        at: [f64; 2],
    },

    /// Extend one line endpoint (0 = start, 1 = end) along its own direction
    /// to the nearest intersection with another element (construction guides
    /// included). The line must point at something: with no intersection the
    /// command fails rather than guessing a length.
    ExtendSketch { doc: DocId, sketch: EntityId, element: u64, point: u8 },

    /// Offset sketch curves by `distance` (mm) to the left of each element's
    /// direction, creating new elements. Lines, circles, arcs and splines are
    /// supported. Select `elements` to copy just those, or omit it to offset
    /// every printable element — when that selection is one closed loop of
    /// lines, the copies are mitered into a proper offset outline (positive
    /// distance then moves outward).
    OffsetSketch {
        doc: DocId,
        sketch: EntityId,
        /// Elements to offset. Omit for all non-construction elements.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        elements: Option<Vec<u64>>,
        distance: Scalar,
        /// Mark the new elements as construction (default: copy the source).
        #[serde(default, skip_serializing_if = "Option::is_none")]
        construction: Option<bool>,
    },

    /// Extrude a closed sketch profile into a printable solid along the
    /// sketch normal. The sketch must be made of closed loops (lines, arcs and
    /// open splines that form loops, and/or circles or closed splines); holes
    /// are handled (a circle inside a loop becomes a through-hole). The result
    /// is a watertight solid placed by the sketch's transform, and it follows
    /// later sketch edits.
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

    /// Repeat solids along a straight line, in world space. `count` includes
    /// the originals, so `count: 4` creates three copies per source. Copies
    /// are new entities with the source's mesh and color (unless overridden)
    /// and are kept in sync with the source while the program replays.
    LinearPattern {
        doc: DocId,
        /// Source solids to repeat. Sketches are display-only: extrude first.
        entities: Vec<EntityId>,
        /// Direction of the repetition (non-zero). May be expressions.
        direction: SVec3,
        /// Distance between adjacent copies (mm). May be an expression.
        spacing: Scalar,
        /// Number of instances including the original (2..=1000). May be an
        /// expression, so a knob changes the number of copies. A structural
        /// change renumbers downstream entities; the result reports the map.
        count: Scalar,
        #[serde(default)]
        name: Option<String>,
        #[serde(default)]
        color: Option<Rgba>,
    },

    /// Repeat solids around an axis, in world space. `count` includes the
    /// originals, and the copies are spread evenly over `angle_deg` (default
    /// 360): step = angle / count. Copies rotate with the pattern.
    CircularPattern {
        doc: DocId,
        entities: Vec<EntityId>,
        /// A point on the axis (default: the world origin). May be expressions.
        #[serde(default)]
        center: Option<SVec3>,
        /// Axis direction (default: +Z). Non-zero.
        #[serde(default)]
        axis: Option<[f64; 3]>,
        /// Total angle covered by the pattern (default 360). May be an
        /// expression.
        #[serde(default)]
        angle_deg: Option<Scalar>,
        /// Number of instances including the original (2..=1000). May be an
        /// expression.
        count: Scalar,
        #[serde(default)]
        name: Option<String>,
        #[serde(default)]
        color: Option<Rgba>,
    },

    /// Mirror solids across a plane, in world space. Creates one reflected
    /// copy per source (the originals are kept).
    Mirror {
        doc: DocId,
        entities: Vec<EntityId>,
        plane: MirrorPlane,
        #[serde(default)]
        name: Option<String>,
        #[serde(default)]
        color: Option<Rgba>,
    },

    /// Hollow a solid in place, leaving a wall of `thickness` around every
    /// face. `open` lists outward face normals (world space) to leave open,
    /// so `[[0,0,1]]` turns a closed hollow box into a cup. The entity keeps
    /// its id, name, color and transform; only the mesh changes. Replays
    /// parametrically.
    Shell {
        doc: DocId,
        entity: EntityId,
        /// Wall thickness (mm). May be an expression; must be > 0.
        thickness: Scalar,
        /// Outward normals of faces to remove (e.g. `[[0,0,1]]` for the top).
        /// Omit for a closed hollow body.
        #[serde(default)]
        open: Vec<[f64; 3]>,
        /// Angular tolerance when matching `open` normals, in degrees
        /// (default 1).
        #[serde(default)]
        angle_tol_deg: Option<f64>,
    },

    /// Drill a hole into a solid, in place. `center` is a point on the entry
    /// surface and `normal` points out of the material; the hole runs along
    /// `-normal`. `counterbore_*` adds a larger cylindrical recess and
    /// `countersink_angle_deg` turns the recess into a cone, so the usual
    /// screw seats are one command. The entity keeps its id; only the mesh
    /// changes.
    Hole {
        doc: DocId,
        entity: EntityId,
        /// A point on the entry surface (world space). May be expressions.
        center: SVec3,
        /// Outward surface normal at the entry point (default +Z).
        #[serde(default = "default_up")]
        normal: [f64; 3],
        /// Hole diameter (mm). May be an expression.
        diameter: Scalar,
        /// Depth into the material (mm). Omit or use a negative value for a
        /// through-hole (it cuts past the far side).
        #[serde(default)]
        depth: Option<Scalar>,
        /// Counterbore / countersink outer diameter (mm).
        #[serde(default)]
        counterbore_diameter: Option<Scalar>,
        /// Counterbore depth (mm). Required for a cylindrical recess.
        #[serde(default)]
        counterbore_depth: Option<Scalar>,
        /// Included countersink angle in degrees (e.g. 82 or 90). When set,
        /// `counterbore_diameter` becomes the cone's surface diameter.
        #[serde(default)]
        countersink_angle_deg: Option<Scalar>,
        /// Segments around the hole; omit for an automatic count.
        #[serde(default)]
        segments: Option<u32>,
    },

    /// Sweep a closed sketch profile along a 3D path into a printable solid.
    /// The profile plane is re-oriented at every path station, so a sketch on
    /// XY works for a path that runs in any direction. A helix path turns the
    /// same command into a spring, a worm or a screw thread.
    Sweep {
        doc: DocId,
        sketch: EntityId,
        path: SweepPath,
        /// Optional override for the number of stations along a helix. Ignored
        /// for a polyline (its points are the stations).
        #[serde(default)]
        segments: Option<u32>,
        #[serde(default)]
        name: Option<String>,
        #[serde(default)]
        color: Option<Rgba>,
    },

    /// Loft between two or more closed sketch profiles: a solid whose
    /// cross-section morphs from one profile to the next. Each sketch must
    /// have a single closed outer loop (holes are not lofted yet). Profiles
    /// are resampled to a common vertex count and rotated to the best
    /// correspondence, so a square can loft smoothly into a circle.
    Loft {
        doc: DocId,
        /// At least two sketches, in order.
        sketches: Vec<EntityId>,
        /// Resampled vertices per profile (default 64, minimum 8).
        #[serde(default)]
        segments: Option<u32>,
        #[serde(default)]
        name: Option<String>,
        #[serde(default)]
        color: Option<Rgba>,
    },

    /// A real screw thread as a standalone solid: a core cylinder with a
    /// helical V-shaped ridge swept around it (axis +Z, base at `origin`).
    /// Union it with a boss for a screw, or subtract it from a body for a
    /// threaded hole. `flank_angle_deg` is the included angle between the two
    /// flanks (default 60, the ISO metric value).
    Thread {
        doc: DocId,
        /// Major (crest) diameter (mm). May be an expression.
        major_diameter: Scalar,
        /// Minor (root) diameter (mm); default is the ISO metric
        /// `major - 1.0825 * pitch`.
        #[serde(default)]
        minor_diameter: Option<Scalar>,
        /// Distance between adjacent threads (mm).
        pitch: Scalar,
        /// Threaded length along +Z (mm).
        length: Scalar,
        #[serde(default)]
        hand: Hand,
        /// Included angle between the flanks (degrees, default 60).
        #[serde(default)]
        flank_angle_deg: Option<Scalar>,
        /// Stations around one full turn (default 48).
        #[serde(default)]
        segments_per_turn: Option<u32>,
        /// Base point of the thread axis (default: the world origin).
        #[serde(default)]
        origin: Option<SVec3>,
        #[serde(default)]
        name: Option<String>,
        #[serde(default)]
        color: Option<Rgba>,
    },
}

fn default_up() -> [f64; 3] {
    [0.0, 0.0, 1.0]
}

fn default_true() -> bool {
    true
}

/// Which sharp edges a fillet/chamfer applies to, in **world space**.
///
/// Every condition that is set must hold (logical AND); with no conditions
/// every sharp edge is selected. Selection is geometric on purpose: entity
/// ids and edge indices are stable across replay, but a parameter change
/// moves edges, and "the edges parallel to Z" keeps meaning the same thing.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
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
    /// What this filter bound to when it was authored. The engine fills this
    /// in on the first live run and compares against it on later replays, so
    /// a selector that starts choosing different geometry is reported as
    /// drift instead of silently re-filleting another edge. Omit it to bind
    /// on first use.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub expect: Option<RefExpect>,
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

/// What a selector bound to when it resolved successfully (M20).
///
/// The engine writes this into the program step on the first live run. It is
/// reporting metadata: it never changes which geometry a selector matches.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct RefExpect {
    /// Identity fingerprint of the resolved group (or set of groups). Stable
    /// while the selector keeps binding to the same feature, even when a
    /// parameter moves or resizes it.
    pub fingerprint: String,
    /// Exact-geometry fingerprint of the same binding. It changes whenever
    /// the feature moves, so a reference can be reported as "same feature,
    /// different geometry" instead of drift.
    pub geometry: String,
    /// Number of logical groups that matched.
    pub matched: usize,
    /// Readable description of the binding, captured at authoring time.
    pub descriptor: String,
}

/// Why a bound selector no longer matches its stored [`RefExpect`].
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ReferenceDrift {
    /// Fingerprint recorded when the selector was authored.
    pub expected: String,
    /// Fingerprint of the geometry it resolves to now.
    pub actual: String,
    pub expected_descriptor: String,
    pub actual_descriptor: String,
}

/// How one selector resolved against live geometry (M20).
///
/// Every modeling command that takes a geometric selector returns one of
/// these, and `validate`/`diff_scene` classify the stored binding to report
/// movement and drift after a rebuild.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ReferenceReport {
    /// Number of logical groups (faces or edges) the selector matched.
    pub matched: usize,
    /// 1-based group ids in the corresponding `faces`/`edges` listing,
    /// ascending. For a face selector, `chosen` is the one that was used.
    pub groups: Vec<u32>,
    /// More than one logical group matched. Intentional broad selectors
    /// (for example "every vertical edge" or no edge filter at all) are
    /// common; the report makes the breadth visible rather than failing.
    pub ambiguous: bool,
    /// The group actually used, when the operation consumes exactly one
    /// (face planes). `None` for selectors that act on every match.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub chosen: Option<u32>,
    /// Underlying sharp segments (edges) or triangles (faces) touched.
    pub elements: usize,
    /// Identity fingerprint of the binding (the chosen face or the matched
    /// set of edges). `None` when nothing matched.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub fingerprint: Option<String>,
    /// Exact-geometry fingerprint of the same binding.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub geometry: Option<String>,
    /// Readable descriptions of the matched groups (capped for large sets).
    #[serde(default)]
    pub descriptors: Vec<String>,
    /// Set when the selector carried a prior [`RefExpect`] and now binds to a
    /// different logical feature.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub drift: Option<ReferenceDrift>,
    /// The same feature, with different exact geometry (a knob resized or
    /// moved it). This is expected for parametric edits and is not drift.
    #[serde(default)]
    pub moved: bool,
}

impl ReferenceReport {
    /// A report for something that is not a geometric selector (a connector
    /// or a datum-derived plane): one logical match, no ambiguity.
    pub fn fixed(fingerprint: impl Into<String>, descriptor: impl Into<String>) -> Self {
        let descriptor = descriptor.into();
        Self {
            matched: 1,
            groups: Vec::new(),
            ambiguous: false,
            chosen: None,
            elements: 1,
            fingerprint: Some(fingerprint.into()),
            geometry: None,
            descriptors: vec![descriptor],
            drift: None,
            moved: false,
        }
    }
}

/// One enumerated planar face of a solid (see [`Command::Faces`]).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct FaceGroupInfo {
    /// 1-based, deterministic within one `faces` call.
    pub id: u32,
    /// `planar` today; reserved for curved surfaces.
    pub kind: String,
    /// Area-weighted outward normal in world space.
    pub normal: Vec3,
    /// Area-weighted centroid (a point on the face plane).
    pub center: Vec3,
    /// World-space area in mm².
    pub area: f64,
    pub bbox: Bbox,
    pub tris: usize,
    /// Adjacent logical edge ids, ascending.
    pub edges: Vec<u32>,
    /// Human-readable description (`planar face #2: normal +Z, area ...`).
    pub descriptor: String,
    /// Identity fingerprint (stable while this face stays the same feature).
    pub fingerprint: String,
    /// Exact-geometry fingerprint.
    pub geometry: String,
    /// Ready to pass as `add_sketch{plane: ...}`.
    pub selector: FaceSelector,
}

/// One enumerated logical edge of a solid (see [`Command::Edges`]).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct EdgeGroupInfo {
    /// 1-based, deterministic within one `edges` call.
    pub id: u32,
    /// `line` (straight run) or `curve` (a tessellated rim or freeform run).
    pub kind: String,
    /// Endpoints for a `line`; omitted for a closed or curved run.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub a: Option<Vec3>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub b: Option<Vec3>,
    /// Length-weighted midpoint (on a circular rim this is the circle
    /// center, not a rim point).
    pub midpoint: Vec3,
    /// Unit direction for a `line`; omitted for a curved run.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub direction: Option<Vec3>,
    /// Total length in mm.
    pub length: f64,
    /// Convex (`true`) or concave (`false`); omitted when a tessellated run
    /// mixes both.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub convex: Option<bool>,
    /// Interior dihedral angle in degrees, when the run is uniform.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub dihedral_deg: Option<f64>,
    /// Sharp segments that make up this logical edge.
    pub segments: usize,
    /// Adjacent planar face ids, ascending.
    pub faces: Vec<u32>,
    pub descriptor: String,
    pub fingerprint: String,
    pub geometry: String,
    /// Ready to pass to `fillet`/`chamfer` as `edges`.
    pub selector: EdgeFilter,
}

/// One cylindrical surface found by [`Command::Primitives`] (M22).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CylinderInfo {
    /// 1-based, deterministic within one `primitives` call.
    pub id: u32,
    /// Unit cylinder axis in world space, canonicalized so its largest
    /// component is positive.
    pub axis: Vec3,
    /// A point on the axis (the mid-height point).
    pub center: Vec3,
    pub radius: f64,
    /// Axial extent of the surface.
    pub height: f64,
    /// `true` for a concave/inner surface (a hole), `false` for a boss or
    /// shaft.
    pub inner: bool,
    /// Surface area in mm².
    pub area: f64,
    pub bbox: Bbox,
    pub tris: usize,
    pub descriptor: String,
    /// Identity fingerprint (axis, inner/outer, rank) that survives a
    /// parameter change which resizes the cylinder.
    pub fingerprint: String,
}

/// One hole candidate derived from an inner cylindrical surface (M22).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct HoleCandidate {
    /// 1-based, deterministic within one `primitives` call.
    pub id: u32,
    /// The [`CylinderInfo::id`] this hole came from.
    pub cylinder: u32,
    /// Mid-height point on the axis.
    pub center: Vec3,
    /// Canonical unit axis (same as the cylinder's).
    pub axis: Vec3,
    pub radius: f64,
    pub depth: f64,
    /// `true` when the cylinder spans essentially the whole solid along its
    /// axis (a through-hole rather than a blind pocket).
    pub through: bool,
    pub descriptor: String,
}

/// The result of [`Command::RepairMesh`]: what the repair changed.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct RepairReport {
    pub vertices_before: usize,
    pub vertices_after: usize,
    pub triangles_before: usize,
    pub triangles_after: usize,
    /// Closed boundary loops that were filled.
    pub holes_filled: usize,
    /// Triangles added by those fills.
    pub triangles_added: usize,
    /// Triangles dropped as degenerate, duplicate or non-manifold junk.
    pub triangles_removed: usize,
    /// Triangles whose winding was flipped to make the shell consistent.
    pub flipped: usize,
    /// Boundary edges remaining after the repair (0 means watertight).
    pub open_edges: usize,
    /// Edges still used by more than two triangles.
    pub non_manifold_edges: usize,
    /// The repaired mesh is closed and manifold.
    pub watertight: bool,
}

/// One stored selector checked by [`Command::Validate`].
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ReferenceCheck {
    /// 0-based program step that carries the selector.
    pub step: usize,
    /// `fillet`, `chamfer`, `add_sketch`, `hole`, `mirror` or `mate`.
    pub method: String,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub entity: Option<EntityId>,
    /// `ok`, `moved`, `ambiguous`, `missing`, `drift` or `unbound`.
    pub status: String,
    pub matched: usize,
    pub ambiguous: bool,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub fingerprint: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub expected: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub expected_descriptor: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub actual_descriptor: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub message: Option<String>,
}

/// One selector whose binding changed between two revisions (see
/// [`Command::DiffScene`]).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ReferenceChange {
    pub step: usize,
    pub method: String,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub entity: Option<EntityId>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub from_fingerprint: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub to_fingerprint: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub from_descriptor: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub to_descriptor: Option<String>,
    pub matched_from: usize,
    pub matched_to: usize,
}

/// Validation of one printable solid (see [`Command::Validate`]).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SolidValidation {
    pub entity: EntityId,
    pub name: String,
    /// `box`, `sphere`, `cylinder`, `boolean`, `extrude`, `revolve`, `sweep`,
    /// `loft`, `thread`, `pattern` or `mirror`.
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
    /// The constraint set is redundant (over-constrained): removing a named
    /// constraint in `diagnostics` restores independence.
    #[serde(default)]
    pub over_constrained: bool,
    /// Unsatisfied or redundant constraints, named by id and reason.
    #[serde(default)]
    pub diagnostics: Vec<String>,
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
    /// `box`, `sphere`, `cylinder`, `boolean`, `sketch`, `extrude`, `revolve`,
    /// `sweep`, `loft`, `thread`, `pattern` or `mirror`.
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

/// The full revision comparison behind `diff_scene` and the dry run of
/// `patch_program`.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SceneDiffReport {
    pub doc: DocId,
    /// The baseline document id, when compared with `against`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub against: Option<DocId>,
    /// Baseline name (document name or `.wcad` path).
    pub against_name: String,
    /// True when anything differs (entities, parameters or program).
    pub changed: bool,
    /// Content fingerprint of the current scene.
    pub scene_signature: String,
    /// Content fingerprint of the baseline scene.
    pub against_signature: String,
    pub added: Vec<EntitySummary>,
    pub removed: Vec<EntitySummary>,
    pub modified: Vec<EntityChange>,
    /// Entities present in both revisions with identical signatures.
    pub unchanged: usize,
    pub parameters: Vec<ParameterChange>,
    pub program_changed: bool,
    pub commands_from: usize,
    pub commands_to: usize,
    /// Selectors whose binding changed between the revisions, even when the
    /// volume barely moved.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub references: Vec<ReferenceChange>,
}

/// One entity id that moved because a program edit changed id allocation.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct IdChange {
    pub from: EntityId,
    pub to: EntityId,
    /// The entity's name after the change, for readable reports.
    pub name: String,
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

/// A triangle-mesh interchange format accepted by [`Command::ImportMesh`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum MeshFormat {
    #[serde(rename = "stl")]
    Stl,
    #[serde(rename = "obj")]
    Obj,
    #[serde(rename = "3mf")]
    ThreeMf,
}

impl MeshFormat {
    pub fn as_str(&self) -> &'static str {
        match self {
            Self::Stl => "stl",
            Self::Obj => "obj",
            Self::ThreeMf => "3mf",
        }
    }

    /// Recognize a filename extension (without the dot, case-insensitive).
    pub fn from_extension(ext: &str) -> Option<Self> {
        match ext.to_ascii_lowercase().as_str() {
            "stl" => Some(Self::Stl),
            "obj" => Some(Self::Obj),
            "3mf" => Some(Self::ThreeMf),
            _ => None,
        }
    }
}

impl std::fmt::Display for MeshFormat {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(self.as_str())
    }
}

/// Which curves a 2D drawing export writes.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CurveMode {
    /// The solved sketch elements: lines, circles, arcs and splines (the
    /// latter as polylines, since DXF R12 has no spline entity).
    Elements,
    /// Only the closed profile loops an extrude/revolve consumes, as
    /// polylines — e.g. a laser-cutter outline.
    Profiles,
    /// Both: elements on the sketch layer, profile loops on a separate one.
    Both,
}

impl Default for CurveMode {
    fn default() -> Self {
        Self::Elements
    }
}

/// How an assembly mate positions instance `b` relative to instance `a`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum MateKind {
    Coincident,
    Distance,
    Parallel,
    Angle,
}

impl MateKind {
    pub fn as_str(&self) -> &'static str {
        match self {
            Self::Coincident => "coincident",
            Self::Distance => "distance",
            Self::Parallel => "parallel",
            Self::Angle => "angle",
        }
    }
}

impl std::fmt::Display for MateKind {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(self.as_str())
    }
}

/// One side of a mate: an instance and one of its named connectors.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct MateRef {
    pub instance: u64,
    pub connector: String,
}

/// A drawing sheet format. Both reuse the existing deterministic 2D writers.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DrawingFormat {
    #[default]
    Svg,
    Dxf,
}

impl DrawingFormat {
    pub fn as_str(&self) -> &'static str {
        match self {
            Self::Svg => "svg",
            Self::Dxf => "dxf",
        }
    }
}

/// Which orthographic (or isometric) projection a drawing view uses.
///
/// Third-angle placement: the front view looks along +Y (screen right +X,
/// up +Z), the top view looks down -Z (right +X, up +Y) and belongs above the
/// front view, and the right view looks along -X (right +Y, up +Z) and
/// belongs to the right of the front view.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ViewKind {
    Front,
    Back,
    Top,
    Bottom,
    Left,
    Right,
    Iso,
}

impl ViewKind {
    pub fn as_str(&self) -> &'static str {
        match self {
            Self::Front => "front",
            Self::Back => "back",
            Self::Top => "top",
            Self::Bottom => "bottom",
            Self::Left => "left",
            Self::Right => "right",
            Self::Iso => "iso",
        }
    }

    /// True for the six axis-aligned views; the isometric view is excluded
    /// from automatic dimensions.
    pub fn is_orthographic(&self) -> bool {
        !matches!(self, Self::Iso)
    }
}

impl std::fmt::Display for ViewKind {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(self.as_str())
    }
}

/// One view on a drawing sheet. `at` is the sheet position (millimeters) of
/// the projected geometry's bounding-box center; omit it for the automatic
/// third-angle layout. `dimensions` overrides the sheet-wide default for
/// this view.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DrawingViewSpec {
    pub view: ViewKind,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub at: Option<[f64; 2]>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub dimensions: Option<bool>,
}

/// Free text on a drawing sheet, at sheet coordinates (millimeters).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DrawingAnnotation {
    pub text: String,
    pub at: [f64; 2],
}

/// One BOM line (see [`Command::BillOfMaterials`]).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BomRow {
    /// Stable item number, assigned in part-name order from 1.
    pub item: u32,
    pub part: String,
    /// The part document's name, when it has one.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub description: Option<String>,
    /// Number of placed instances.
    pub quantity: u32,
    /// Member entities currently present.
    pub entities: usize,
    /// Total signed world volume of the present members (mm³).
    pub volume: f64,
    /// Instance names, in creation order.
    pub instances: Vec<String>,
}

/// One overlapping pair reported by [`Command::CheckInterference`].
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct InterferenceHit {
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub a_instance: Option<u64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub b_instance: Option<u64>,
    pub a_name: String,
    pub b_name: String,
    pub a_entity: EntityId,
    pub b_entity: EntityId,
    /// Intersection volume in mm³.
    pub volume: f64,
    /// World-space bounds of the intersection.
    pub bbox: Bbox,
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

/// Which way a helix or thread winds.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Hand {
    #[default]
    Right,
    Left,
}

impl Hand {
    pub fn as_str(&self) -> &'static str {
        match self {
            Self::Right => "right",
            Self::Left => "left",
        }
    }

    /// +1 for a right-hand helix, -1 for a left-hand one.
    pub fn sign(&self) -> f64 {
        match self {
            Self::Right => 1.0,
            Self::Left => -1.0,
        }
    }
}

/// The plane a [`Command::Mirror`] reflects across, in world space.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum MirrorPlane {
    /// A standard datum plane, optionally offset along its normal. The
    /// `xy` plane is Z = offset, `xz` is Y = -offset, `yz` is X = offset
    /// (the same frames sketches use).
    Datum {
        plane: DatumPlane,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        offset: Option<Scalar>,
    },
    /// Any plane given by an origin and a non-zero normal.
    Custom { origin: SVec3, normal: [f64; 3] },
}

/// The path a [`Command::Sweep`] follows.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SweepPath {
    /// A polyline through world-space points. `closed` joins the last point
    /// back to the first.
    Polyline {
        points: Vec<SVec3>,
        #[serde(default)]
        closed: bool,
    },
    /// A helix around `axis` through `origin`. The path starts at angle 0
    /// (on the frame's in-plane X axis) and advances `pitch` along the axis
    /// per turn.
    Helix {
        #[serde(default)]
        origin: Option<SVec3>,
        /// Axis direction (default +Z). Non-zero.
        #[serde(default)]
        axis: Option<[f64; 3]>,
        radius: Scalar,
        pitch: Scalar,
        turns: Scalar,
        #[serde(default)]
        hand: Hand,
        /// Stations per full turn. Omit for an automatic count.
        #[serde(default)]
        segments_per_turn: Option<u32>,
    },
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
    EntityCreated {
        doc: DocId,
        entity: EntityId,
        mesh: Mesh,
        /// How the sketch's face-plane selector resolved, when it had one.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        reference: Option<ReferenceReport>,
    },
    /// A part was registered for `add_instance` (see [`Command::ImportPart`]).
    /// `solids` is the number of printable solids an instance will create and
    /// `bytes` the embedded snapshot size.
    PartImported {
        doc: DocId,
        name: String,
        description: String,
        solids: usize,
        parameters: usize,
        bytes: usize,
    },
    /// A bare embedded part definition was registered (`add_part`).
    PartDefined { doc: DocId, name: String, solids: usize, parameters: usize },
    /// A part was placed; `entities` lists its member solids in creation order.
    InstanceCreated {
        doc: DocId,
        instance: u64,
        name: String,
        part: String,
        entities: Vec<EntityId>,
    },
    ConnectorAdded { doc: DocId, instance: u64, name: String },
    InstanceMoved { doc: DocId, instance: u64 },
    /// A mate was solved by moving instance `b` (see [`Command::Mate`]).
    MateSolved {
        doc: DocId,
        kind: MateKind,
        a_instance: u64,
        b_instance: u64,
        /// The connectors the mate resolved to.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        reference: Option<ReferenceReport>,
    },
    /// Bill of materials (see [`Command::BillOfMaterials`]).
    Bom { doc: DocId, rows: Vec<BomRow> },
    /// Interference report (see [`Command::CheckInterference`]). `ok` is true
    /// when no pair overlapped above the tolerance. `checked` counts the pairs
    /// whose world AABBs overlapped and therefore reached the kernel.
    Interference {
        doc: DocId,
        ok: bool,
        checked: usize,
        tolerance: f64,
        hits: Vec<InterferenceHit>,
    },
    /// A multi-view engineering drawing was written (see
    /// [`Command::ExportDrawing`]).
    DrawingExported {
        doc: DocId,
        path: String,
        format: String,
        views: usize,
        curves: usize,
        bytes: usize,
    },
    /// A mesh file was imported as a solid (see [`Command::ImportMesh`]).
    /// `vertices` and `triangles` are the cleaned counts, and `dropped`
    /// counts degenerate or duplicate input triangles that were discarded.
    MeshImported {
        doc: DocId,
        entity: EntityId,
        name: String,
        format: MeshFormat,
        path: String,
        bytes: usize,
        vertices: usize,
        triangles: usize,
        dropped: usize,
    },
    /// A 2D drawing file was imported as a sketch (see
    /// [`Command::ImportDxf`] and [`Command::ImportSvg`]).
    CurveImported {
        doc: DocId,
        sketch: EntityId,
        name: String,
        format: DrawingFormat,
        path: String,
        bytes: usize,
        /// Curves read from the file.
        curves: usize,
        /// Sketch elements written to the program (a polyline becomes lines).
        elements: usize,
        /// Inferred constraints (0 when auto-constraints are off).
        constraints: usize,
        /// Closed profile loops in the imported drawing.
        closed_loops: usize,
        solved: bool,
        #[serde(default, skip_serializing_if = "Vec::is_empty")]
        warnings: Vec<String>,
    },
    /// A sketch was created with a complete set of elements (see
    /// [`Command::AddSketchCurves`], `section` and `project`).
    SketchBuilt {
        doc: DocId,
        sketch: EntityId,
        name: String,
        elements: usize,
        constraints: usize,
        /// Closed profile loops the elements form (0 for an open drawing).
        closed_loops: usize,
        solved: bool,
        residual: f64,
        dof: i64,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        reference: Option<ReferenceReport>,
    },
    /// Analytic primitives of one solid (see [`Command::Primitives`]).
    Primitives {
        doc: DocId,
        entity: EntityId,
        name: String,
        planar_faces: Vec<FaceGroupInfo>,
        cylinders: Vec<CylinderInfo>,
        holes: Vec<HoleCandidate>,
    },
    /// A mesh was repaired in place (see [`Command::RepairMesh`]).
    MeshRepaired {
        doc: DocId,
        entity: EntityId,
        /// New signed world volume (mm³).
        volume: f64,
        tris: usize,
        mesh: Mesh,
        report: RepairReport,
    },
    /// A solid was reshaped in place (fillet/chamfer). `edges` is how many
    /// sharp edges the selection matched, `volume` the new world-space
    /// volume in mm³, and `mesh` the new local mesh.
    EntityModified {
        doc: DocId,
        entity: EntityId,
        edges: u32,
        volume: f64,
        mesh: Mesh,
        /// How the edge selector resolved: matched logical edges, ambiguity
        /// and the identity fingerprint (M20).
        #[serde(default, skip_serializing_if = "Option::is_none")]
        reference: Option<ReferenceReport>,
    },
    /// Several entities created at once (linear/circular patterns and
    /// mirrors). `entities` lists the copies in creation order.
    EntitiesCreated {
        doc: DocId,
        entities: Vec<EntityId>,
        tris: usize,
        /// The mirror plane or pattern definition the copies came from.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        reference: Option<ReferenceReport>,
    },
    /// A solid was reshaped in place without an edge count (shell, hole).
    /// `volume` is the new world-space volume and `mesh` the new local mesh.
    EntityReshaped {
        doc: DocId,
        entity: EntityId,
        volume: f64,
        tris: usize,
        mesh: Mesh,
        /// How the entry face or opened faces resolved (hole/shell).
        #[serde(default, skip_serializing_if = "Option::is_none")]
        reference: Option<ReferenceReport>,
    },
    /// A solid was built from profiles or a path (sweep, loft, thread).
    /// `kind` is `sweep`, `loft` or `thread`.
    SolidBuilt {
        doc: DocId,
        entity: EntityId,
        kind: String,
        volume: f64,
        tris: usize,
    },
    Transformed { doc: DocId, entity: EntityId },
    Deleted { doc: DocId, entity: EntityId },
    ParameterSet {
        doc: DocId,
        name: String,
        value: f64,
        /// Geometry features that regenerated their mesh during this replay.
        #[serde(default)]
        rebuilt: usize,
        /// Geometry features whose mesh came from the content cache because
        /// neither they nor their upstream inputs changed.
        #[serde(default)]
        reused: usize,
        /// Program steps replayed (every step still runs; only geometry is
        /// memoized).
        #[serde(default)]
        steps: usize,
        /// Stored selectors whose logical binding changed during this replay
        /// (M20). Inspect `validate` for the details and descriptors.
        #[serde(default)]
        references_drifted: usize,
        /// Entities renumbered by a structural change (a pattern count
        /// expression that changed). Empty for ordinary knob turns.
        #[serde(default, skip_serializing_if = "Vec::is_empty")]
        renumbered: Vec<IdChange>,
        /// Entities a structural change added (extra pattern copies).
        #[serde(default, skip_serializing_if = "Vec::is_empty")]
        created: Vec<EntityId>,
        /// Entities a structural change removed (fewer pattern copies).
        #[serde(default, skip_serializing_if = "Vec::is_empty")]
        removed: Vec<EntityId>,
    },
    ParameterRemoved { doc: DocId, name: String },
    Parameters { doc: DocId, params: Vec<ParamInfo> },
    Undone { doc: DocId },
    Redone { doc: DocId },
    Scene { doc: DocId, entities: Vec<EntitySnapshot> },
    /// A ray query (see [`Command::Pick`]). `hit` is `None` when the ray
    /// missed every solid; that is a valid answer, not an error.
    Picked { doc: DocId, hit: Option<PickHit> },
    /// Enumerated planar faces (see [`Command::Faces`]).
    FacesResolved { doc: DocId, entity: EntityId, name: String, faces: Vec<FaceGroupInfo> },
    /// Enumerated logical edges (see [`Command::Edges`]).
    EdgesResolved {
        doc: DocId,
        entity: EntityId,
        name: String,
        min_angle_deg: f64,
        edges: Vec<EdgeGroupInfo>,
    },
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
        /// Stored selectors re-resolved against the current geometry.
        #[serde(default, skip_serializing_if = "Vec::is_empty")]
        references: Vec<ReferenceCheck>,
    },
    /// Revision comparison by entity signature (see [`Command::DiffScene`]).
    SceneDiff(SceneDiffReport),
    /// A program edit (see [`Command::PatchProgram`]). `diff` is present on a
    /// dry run: the patched scene compared with the current one, so an agent
    /// can prove the edit before committing it.
    ProgramPatched {
        doc: DocId,
        dry_run: bool,
        /// Steps in the patched program.
        steps: usize,
        /// Suppressed step indexes after the edit.
        #[serde(default)]
        suppressed: Vec<usize>,
        /// Entities that still exist after the edit.
        surviving: usize,
        /// Old ids that now answer to a different id, with the entity name.
        #[serde(default, skip_serializing_if = "Vec::is_empty")]
        renumbered: Vec<IdChange>,
        /// Old entities the edit took out of the scene.
        #[serde(default, skip_serializing_if = "Vec::is_empty")]
        removed: Vec<EntityId>,
        /// New entities the edit added.
        #[serde(default, skip_serializing_if = "Vec::is_empty")]
        created: Vec<EntityId>,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        diff: Option<SceneDiffReport>,
    },
    /// Suppression changed (see [`Command::Suppress`]).
    SuppressionChanged {
        doc: DocId,
        /// Suppressed step indexes, ascending.
        suppressed: Vec<usize>,
        /// Program steps replayed (every step still exists; `steps - suppressed`
        /// execute).
        steps: usize,
        /// Entities that disappeared because their step is now suppressed.
        #[serde(default, skip_serializing_if = "Vec::is_empty")]
        removed: Vec<EntityId>,
        /// Entities that came back or moved id.
        #[serde(default, skip_serializing_if = "Vec::is_empty")]
        renumbered: Vec<IdChange>,
        /// Entities that reappeared when a step was switched back on.
        #[serde(default, skip_serializing_if = "Vec::is_empty")]
        created: Vec<EntityId>,
    },
    /// A batch ran (see [`Command::Batch`]). `results` are the individual
    /// command results, in execution order.
    BatchExecuted { commands: usize, atomic: bool, results: Vec<CommandResult> },
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
    /// A 2D DXF drawing was written. `entities` counts the source entities
    /// exported, `curves` the primitives in the file.
    DxfExported {
        doc: DocId,
        path: String,
        entities: usize,
        curves: usize,
        bytes: usize,
    },
    /// A 2D SVG drawing was written (same fields as [`Self::DxfExported`]).
    SvgExported {
        doc: DocId,
        path: String,
        entities: usize,
        curves: usize,
        bytes: usize,
    },
    /// A STEP file was written. `entities` counts the source entities,
    /// `solids` the closed `FACETED_BREP` pieces, `surfaces` the open
    /// `SHELL_BASED_SURFACE_MODEL` pieces and `faces` the triangles written.
    StepExported {
        doc: DocId,
        path: String,
        entities: usize,
        solids: usize,
        surfaces: usize,
        faces: usize,
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
        /// Program steps skipped on replay, ascending (see
        /// [`Command::Suppress`]).
        #[serde(default, skip_serializing_if = "Vec::is_empty")]
        suppressed: Vec<usize>,
        /// The authoring source embedded in the document, when it was saved
        /// with one. The script on disk is the source of truth; this is the
        /// copy that rode along in the `.wcad` snapshot, so a handoff can
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
        /// Redundant or unsatisfied constraints, named by id.
        #[serde(default)]
        diagnostics: Vec<String>,
        /// True when the equations are redundant even though they solve.
        #[serde(default)]
        over_constrained: bool,
    },
    ConstraintRemoved { doc: DocId, sketch: EntityId, constraint: u64 },
    SketchPointMoved { doc: DocId, sketch: EntityId, element: u64 },
    /// Sketch geometry was edited in place (trim, extend, offset).
    SketchEdited {
        doc: DocId,
        sketch: EntityId,
        /// New element ids, in creation order (empty when nothing was added).
        created: Vec<u64>,
        /// Element ids removed by the edit (empty when nothing was removed).
        removed: Vec<u64>,
        /// Existing element ids whose geometry changed.
        modified: Vec<u64>,
        solved: bool,
        residual: f64,
        dof: i64,
        #[serde(default, skip_serializing_if = "Vec::is_empty")]
        diagnostics: Vec<String>,
    },
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

/// The on-disk form of a document (`.wcad`, JSON).
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
    /// Program steps that replay skips, by 0-based index. Kept sorted; an
    /// empty set is not written, so unchanged documents keep their shape.
    #[serde(default, skip_serializing_if = "std::collections::BTreeSet::is_empty")]
    pub suppressed: std::collections::BTreeSet<usize>,
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
/// along in the saved `.wcad` so a handoff can recover the recipe; opening a
/// document never executes it.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SourceDoc {
    pub language: String,
    pub text: String,
}

/// Reference to a point (or a whole element) inside a sketch.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct SketchRef {
    /// Sketch element id (from add_sketch_line / add_sketch_circle /
    /// add_sketch_arc / add_sketch_spline).
    pub element: u64,
    /// Point index, interpreted per element kind:
    /// line `0` = start, `1` = end; circle `0` = center;
    /// arc `0` = center, `1` = start, `2` = end; spline `i` = point `i`.
    /// `None` = the whole element (its direction/radius, for whole-element
    /// constraints).
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
    /// Two lines of equal length, or two circles/arcs of equal radius.
    Equal,
    /// Two circles/arcs share a center point.
    Concentric,
    /// Two points mirrored about a line (refs: point, point, whole line).
    Symmetric,
    /// The angle between two lines, in degrees (`value`).
    Angle,
    /// A point lies on the supporting line of a line element.
    PointOnLine,
    /// A line is tangent to a circle/arc, or two circles/arcs are externally
    /// tangent.
    Tangent,
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
            Self::Equal => "equal",
            Self::Concentric => "concentric",
            Self::Symmetric => "symmetric",
            Self::Angle => "angle",
            Self::PointOnLine => "point_on_line",
            Self::Tangent => "tangent",
        }
    }

    /// How many scalar equations this constraint contributes to the solver.
    pub fn equations(&self) -> i64 {
        match self {
            Self::Coincident | Self::Concentric | Self::Symmetric => 2,
            _ => 1,
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
        /// What this face reference bound to when it was authored. Filled in
        /// by the engine on the first live run and compared on later
        /// replays. Omit it to bind on first use.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        expect: Option<RefExpect>,
    },
}

/// A world-space working plane for `section` and `project` (M22). Unlike a
/// sketch plane there is no face variant: a cut or projection needs a plane,
/// not a solid's surface reference.
///
/// Wire forms:
///
/// ```json
/// {"datum": {"plane": "xy", "offset": 5}}
/// {"custom": {"origin": [0,0,10], "normal": [0,0,1], "x_axis": [1,0,0]}}
/// ```
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum WorkPlane {
    /// A standard datum plane, optionally offset along its normal.
    Datum {
        plane: DatumPlane,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        offset: Option<Scalar>,
    },
    /// Any plane given by a world-space origin and normal.
    Custom {
        origin: SVec3,
        /// Non-zero plane normal (does not need to be unit length).
        normal: [f64; 3],
        #[serde(default, skip_serializing_if = "Option::is_none")]
        x_axis: Option<[f64; 3]>,
    },
}

impl Default for WorkPlane {
    fn default() -> Self {
        Self::Datum { plane: DatumPlane::Xy, offset: None }
    }
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
/// sketch entity's transform places them in 3D. `construction` elements are
/// solved and displayed but excluded from the extrudable profile.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum SketchElement {
    Line {
        id: u64,
        p1: [f64; 2],
        p2: [f64; 2],
        #[serde(default, skip_serializing_if = "is_false")]
        construction: bool,
    },
    Circle {
        id: u64,
        center: [f64; 2],
        radius: f64,
        #[serde(default, skip_serializing_if = "is_false")]
        construction: bool,
    },
    /// Counter-clockwise arc from `start_deg` to `end_deg` (degrees).
    Arc {
        id: u64,
        center: [f64; 2],
        radius: f64,
        start_deg: f64,
        end_deg: f64,
        #[serde(default, skip_serializing_if = "is_false")]
        construction: bool,
    },
    /// Interpolating Catmull-Rom spline through `points`.
    Spline {
        id: u64,
        points: Vec<[f64; 2]>,
        #[serde(default, skip_serializing_if = "is_false")]
        closed: bool,
        #[serde(default, skip_serializing_if = "is_false")]
        construction: bool,
    },
}

fn is_false(b: &bool) -> bool {
    !*b
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
    /// True when the constraint equations are redundant (rank deficient) even
    /// if a solution exists. The diagnostics name the constraints to remove.
    #[serde(default)]
    pub over_constrained: bool,
    /// Conflict diagnostics: unsatisfied or redundant constraints named by id,
    /// with the reason. Empty when the sketch is clean.
    #[serde(default)]
    pub diagnostics: Vec<String>,
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
    /// For a solid placed by `add_instance`: the owning instance id. The
    /// instance moves every entity that shares it.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub instance: Option<u64>,
    /// For a part member: the part name it was replayed from.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub part: Option<String>,
}
