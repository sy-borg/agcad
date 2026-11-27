"""Minimal Model Context Protocol (MCP) server for Wright CAD — stdlib only.

Speaks newline-delimited JSON-RPC 2.0 on stdio (the MCP stdio transport)
and forwards tool calls to the wright command bus. Deliberately depends on
nothing but the `wright` client package: the protocol surface we need
(initialize, tools/list, tools/call, ping) is small, and zero deps beats
SDK version drift.

Design rules:
- One tool per *intent*, not per command: `wright_execute` runs a batch of
  bus commands in one call (MCP round-trips are expensive).
- Token economy: mesh payloads are summarized to stats (bbox, tri count).
  Full meshes are available via the Python client, not the model.
- The `instructions` field in the initialize result teaches the agent the
  command language — hosts like opencode inject it into the system prompt.
"""
from __future__ import annotations

import json
import os
import sys
from typing import Any

from .client import Wright, WrightError

SERVER_INFO = {"name": "wright", "version": "0.5.0"}
DEFAULT_PROTOCOL_VERSION = "2025-06-18"
MAX_BATCH = 500

INSTRUCTIONS = """Wright CAD is a headless parametric CAD for 3D printing.
A document holds a program: every modeling command is recorded, dimensions may be
expressions over named parameters ("wall*2"). set_parameter replays the program
deterministically (entity ids stay stable unless a pattern count expression
changes) and regenerates only the geometry downstream of the change; unchanged
features reuse memoized meshes. set_parameter reports any renumbered ids.
undo/redo revert one command each and restore parameters + program + geometry
atomically. patch_program inserts/removes/replaces/reorders program steps with a
dry run that reports surviving and renumbered ids plus a scene diff; suppress
turns a step off without deleting it for A/B variants; batch{commands,
atomic:true} rolls a failed multi-command batch back.
Units: millimeters and degrees. Z is up. Colors are [r,g,b,a] floats 0..1.
Cylinders extrude along +Z. Document ids start at 1; create_document takes an
optional explicit `doc` id, so create-then-populate fits in one batch.
Workflow: create_document -> set_parameter(...) -> add primitives with expression
dimensions -> transform -> boolean ops for printable solids -> wright_scene to
verify -> validate to prove watertightness/printability and diff_scene to see
exactly what a knob changed -> suggest_orientation/arrange to put a part on the
bed -> wright_render to SEE the result -> export_stl/
export_3mf for slicing, export_step for a faceted STEP B-rep in other CAD, and
export_dxf/export_svg for 2D sketch drawings.
Sketches are
2D constraint-solved drawings placed in 3D by the entity transform: add_sketch ->
add_sketch_line/add_sketch_circle/add_sketch_arc/add_sketch_spline ->
constrain(...) re-solves after every change -> trim_sketch/extend_sketch/
offset_sketch edit the drawing -> extrude/revolve turns a closed profile into a
printable solid. Construction elements guide constraints and display but are
excluded from the profile. Sketch planes can
be the XY/XZ/YZ datum planes, an explicit origin/normal, or a planar face of an
existing solid; a face sketch's normal is the outward face normal, so a positive
extrude adds a boss and a negative extrude cuts into the body (boolean difference
removes it). Fillets and
chamfers reshape a solid in place (same entity id): fillet{doc,entity,radius,edges?}
or chamfer{doc,entity,distance,edges?}, where `edges` is a geometric filter
({direction}, {near,near_radius}, {normals}, {convex}) instead of an index — omit
it to take every sharp edge. The Python script is the source of truth;
save_document -> .wcad is a replayable snapshot (program + current parameter
values + the embedded script). get_program exposes all three, so a handoff can
recover the recipe without executing it. For durable work keep the script and
pass it as save_document{source:{language:"python",text:...}}; an MCP-only
document with no script is scratch state. A human working in the viewer can
label entities and click a face or edge; the click arrives as a pick result
whose face/edge selectors you pass straight to add_sketch/fillet/chamfer.
Shape leverage: linear_pattern/circular_pattern repeat solids (count includes the
original and may be an expression), mirror reflects them across a plane, shell
hollows a solid in place
(open faces listed by outward normal), hole cuts a through/blind/counterbore/
countersink hole in place, sweep carries a profile along a polyline or helix,
loft morphs between two or more profiles, and thread builds a real screw thread
(union it for a screw, subtract it for a threaded hole).
Assemblies reuse whole parts: import_part (a .wcad file or an open document)
embeds the part's program, add_instance places it (with per-instance parameter
overrides), add_connector names attachment points and mate joins two instances
(coincident/distance/parallel/angle). bill_of_materials and check_interference
answer the assembly questions, and export_drawing writes an SVG or DXF sheet
with front/top/right/iso views, hidden lines, dimensions, balloons and a BOM.
Printability preflight: validate also reports overhang area, bridge spans,
first-layer contact, minimum feature, shells/floating islands, closed cavities
with a suggested drainage hole, and cups; suggest_orientation scores stable
build orientations from planar faces, arrange shelf-packs solids onto the bed,
set_material assigns a preset/custom material, and fit turns a nominal shaft or
hole into a modeled size (free/sliding/snug/press) with hole-shrink and
elephant-foot compensation.
Import → parametric: import_dxf/import_svg bring a drawing in as a
constraint-solved sketch, section/project cut or project a solid into sketch
geometry, primitives lists planar faces, cylinder axes/radii and hole
candidates so an imported part can be re-authored, and repair_mesh fills holes,
fixes winding and removes non-manifold junk. Documents live in the engine
process, not in this server, so a restarted backend
starts empty. When a tool result carries `reconnected`, the client rebuilt the
connection: read its `open_documents` list, replay your script if your document
is gone, and never assume old entity ids survived. The viewer reconnects and
refreshes on its own.
Failed commands report a hint; fix the named issue and retry."""

EXECUTE_DESCRIPTION = """Batch-execute wright commands. Each command is an object:
{"method": "<name>", ...params} or {"method": "<name>", "params": {...}}.
Commands: initialize{}, list_documents{}, create_document{name,doc?}
  (doc is an unused id >= 1; omit it to allocate the next one — pass it to
  create+populate in a single batch), close_document{doc},
set_parameter{doc,name,value(number|string expression),description?} (returns
  rebuilt/reused geometry-feature counts and total program steps),
remove_parameter{doc,name}, get_parameters{doc},
add_box{doc,name,center:[x,y,z],size:[w,d,h],color?},
add_sphere{doc,name,center,radius,segments?,color?},
add_cylinder{doc,name,center,radius,height,segments?,color?} (extruded along
  +Z, spanning center.z ± height/2 — no rotation needed to stand it up),
import_mesh{doc,path,name?,scale?,format?,smooth_angle_deg?,color?} (read an
  STL (binary/ASCII), OBJ or 3MF from the SERVER filesystem and add it as a
  solid; `path` should be absolute. 3MF units convert to mm automatically;
  `scale` multiplies after that. The parser welds vertices, drops degenerate
  or duplicate triangles, flips a globally inward winding, and computes
  crease-aware normals (default 30°). It records an add_mesh program step
  containing the baking triangles, so save/open does NOT need the source
  file; a non-watertight import is allowed, validate reports it. format
  overrides extension/contents detection; name defaults to the file stem),
add_mesh{doc,name,positions:[[x,y,z],...],indices:[i0,i1,i2,...],
  smooth_angle_deg?,color?} (raw triangle solid, three indices per triangle;
  normally produced by import_mesh, but useful when a client already has mesh
  data),
import_dxf{doc,path,name?,plane?,scale?,auto_constraints?} (read an ASCII DXF
  (LINE/CIRCLE/ARC/POLYLINE/LWPOLYLINE) into a SKETCH; $INSUNITS converts to
  mm, plane is a sketch plane (default XY); records an add_sketch_curves step
  so save/open needs no source file. auto_constraints (default true) infers
  coincident, horizontal/vertical and approximate tangency),
import_svg{doc,path,name?,plane?,scale?,auto_constraints?} (same for SVG:
  line/circle/ellipse/rect/polyline/polygon/path M/L/H/V/C/S/Q/T/A/Z, group
  transforms, width/height/viewBox; Y is flipped upright),
add_sketch_curves{doc,name,elements,constraints?,plane?} (low-level baked
  sketch step; elements carry ids and use the wire sketch-element shape),
section{doc,entity,plane?,name?} (cut a solid/imported mesh with a plane and
  return a new SKETCH of closed loops ready to extrude; plane is
  {datum:{plane:"xy"|"xz"|"yz",offset?}} or {custom:{origin,normal,x_axis?}},
  default XY. Modeling step, replays),
project{doc,entity,plane?,min_angle_deg?,name?} (project boundary/sharp/
  silhouette edges onto a plane as a sketch outline; plane normal is the view
  direction. Modeling step),
primitives{doc,entity} (session query: planar_faces, fitted cylinders with
  axis/center/radius/height/inner, and hole candidates with depth + through/
  blind guess — use it to re-author an imported part instead of only
  booleaning onto it),
repair_mesh{doc,entity,fill_holes?,fix_winding?,remove_non_manifold?,
  smooth_angle_deg?} (IN PLACE: weld, orient each shell outward, drop
  non-manifold fins and fill closed boundary loops; returns holes_filled,
  flipped/removed counts and final watertightness; modeling step, replays),
transform{doc,entity,translation?,rotation_deg?,scale?} (only provided parts change),
boolean{doc,op:"union"|"difference"|"intersection",entities:[ids],name?,color?}
  (world-space set op; creates a NEW entity, sources are kept — delete them if
  you only want the fused result; boolean entities replay parametrically),
fillet{doc,entity,radius,edges?,min_angle_deg?} (rounds the selected sharp edges
  of a solid IN PLACE — the entity keeps its id, name, color and transform;
  radius may be an expression, so a knob re-fillets the rebuilt geometry; returns
  edges matched, world-space volume and the new local mesh),
chamfer{doc,entity,distance,edges?,min_angle_deg?} (flat bevel, same in-place
  semantics; convex edges lose material, concave edges gain it),
  edges: a filter object, all set conditions ANDed; omit it for every sharp edge:
    {direction:[x,y,z],angle_tol_deg?} — edges parallel to a direction,
    {near:[x,y,z],near_radius} — edges whose midpoint is near a point,
    {normals:[[..]]} — ONE outward face normal = every edge of that face
      ([[0,0,1]] is the whole top rim); TWO = the edge between those faces,
      either order ([[0,0,1],[1,0,0]]),
    {convex:true|false} — only outside corners / only inside corners,
  min_angle_deg (default 30) is how sharp a face pair must be to count as an edge;
  it keeps tessellation seams of curved surfaces out. Omitting `edges` rounds
  every sharp edge in one pass, inside corners included, and treats a
  tessellated rim (a cylinder's edge circle) as one continuous band. When treating rims and
  vertical edges with separate commands, apply rim treatments first: the later
  vertical fillet trims the rim band cleanly, while re-cutting an already
  rounded rim is slow and noisy.
set_color{doc,entity,color}, delete{doc,entity}, undo{doc}, redo{doc},
set_material{doc,entity,material,density_g_cm3?,cost_per_kg?} (assign a print
  material; presets pla/petg/abs/asa/tpu/nylon/resin, or a custom name with
  density and price; a program step, so it replays and feeds BOM mass/cost),
list_materials{} (built-in density (g/cm^3) and price (USD/kg) presets),
add_sketch{doc,name,plane?} (empty 2D sketch; plane may be omitted for the XY
  datum plane at the origin. Other forms:
    {datum:{plane:"xy"|"xz"|"yz",offset?}} — a standard datum plane
      (xy: X=+X,Y=+Y,normal=+Z; xz: X=+X,Y=+Z,normal=-Y;
       yz: X=+Y,Y=+Z,normal=+X; offset shifts along the normal),
    {custom:{origin:[x,y,z],normal:[x,y,z],x_axis?:[x,y,z]}} — any plane,
    {face:{entity:id,normal?:[x,y,z],near?:[x,y,z],angle_tol_deg?}} — a planar
      face of an existing solid; normal is the OUTWARD face normal, near picks
      between parallel faces and becomes the sketch origin (projected onto the
      face; face center when omitted), omit normal to take the largest face. Positive
      extrude adds material in front of the face, negative cuts into the body
      (then boolean{difference} to remove it). The resolved frame is reported
      as sketch.plane in get_scene.),
add_sketch_line{doc,sketch,p1:[x,y],p2:[x,y],construction?},
add_sketch_circle{doc,sketch,center:[x,y],radius,construction?},
add_sketch_arc{doc,sketch,center:[x,y],radius,start_deg,end_deg,construction?}
  (counter-clockwise from start to end; angles in degrees from local +X),
add_sketch_spline{doc,sketch,points:[[x,y],...],closed?,construction?}
  (interpolating Catmull-Rom through every point; open 2+ points, closed 3+),
constrain{doc,sketch,kind,refs,value?} (re-solves; kind is coincident,
  horizontal, vertical, parallel, perpendicular, distance, radius, diameter,
  equal, concentric, symmetric, angle, point_on_line or tangent; refs are
  {"element":id,"point":0|1|...} for points or {"element":id} for whole
  elements. Points: line 0=start,1=end; circle 0=center; arc 0=center,
  1=start,2=end; spline i=point i. coincident/distance take 2 points (distance
  also takes point+line), horizontal/vertical 1 line, parallel/perpendicular/
  angle 2 lines, radius/diameter 1 circle/arc + value, equal 2 lines or 2
  curves, concentric 2 curves, symmetric [point,point,whole line], point_on_line
  [point,whole line], tangent [line,curve] or [curve,curve]. The result reports
  solved/residual/dof plus diagnostics naming unsatisfied or redundant
  constraints by id; over_constrained flags rank-deficient equations),
remove_constraint{doc,sketch,constraint},
move_sketch_point{doc,sketch,element,point,to:[x,y]} (point refs as above;
  steers the solver, then re-solves),
delete_sketch_element{doc,sketch,element} (constraints on it cascade away),
trim_sketch{doc,sketch,element,at:[x,y]} (remove the piece containing `at`,
  bounded by the nearest intersections; a circle needs two cuts and becomes an
  arc; returns created/removed/modified ids),
extend_sketch{doc,sketch,element,point:0|1} (extend a line endpoint to the
  nearest boundary in its path; errors if nothing is in the way),
offset_sketch{doc,sketch,distance,elements?,construction?} (new elements;
  positive distance is left of each element's direction, except one closed loop
  of lines is mitered with positive = outward; lines/circles/arcs exact,
  splines offset their control points),
extrude{doc,sketch,distance,name?,color?} (closed profile → watertight solid along
  the sketch normal; negative distance extrudes the other way; nested loops become
  holes; the solid follows later sketch edits; returns volume + tris),
revolve{doc,sketch,angle_deg?,axis?:"x"|"y",segments?,name?,color?} (profile →
  solid of revolution around an in-plane axis, default 360°; profile must stay on
  one side of the axis),
linear_pattern{doc,entities:[ids],direction:[x,y,z],spacing,count,name?,color?}
  (count INCLUDES the original, so count:4 creates 3 copies; both count and
  spacing may be expressions, so a parameter drives the number of copies. New
  solids are returned in `entities`; a structural count change renumbers
  downstream entities and set_parameter reports the map),
circular_pattern{doc,entities,count,center?=[0,0,0],axis?=[0,0,1],angle_deg?=360,
  name?,color?} (copies rotate with the pattern and share the angle evenly;
  count may be an expression),
mirror{doc,entities,plane,name?,color?} (plane is {datum:{plane:"xy"|"xz"|"yz",
  offset?}} or {custom:{origin,normal}}; one reflected copy per source),
shell{doc,entity,thickness,open?,angle_tol_deg?} (hollow IN PLACE; `open` is a
  list of outward face normals to remove, e.g. [[0,0,1]] for a cup; omit for a
  sealed hollow), hole{doc,entity,center,normal?=[0,0,1],diameter,depth?,
  counterbore_diameter?,counterbore_depth?,countersink_angle_deg?,segments?}
  (IN PLACE; center is on the entry surface, the hole runs along -normal; omit
  depth for through; give counterbore diameter+depth or a countersink included
  angle), sweep{doc,sketch,path,segments?,name?,color?} (path is
  {polyline:{points:[[x,y,z],...],closed?}} or {helix:{radius,pitch,turns,
  hand?:"right"|"left",origin?,axis?,segments_per_turn?}}; the profile plane is
  re-oriented along the path), loft{doc,sketches:[ids],segments?,name?,color?}
  (one closed outer loop per sketch; profiles may be on different planes),
thread{doc,major_diameter,minor_diameter?,pitch,length,hand?:"right"|"left",
  flank_angle_deg?=60,segments_per_turn?,origin?,name?,color?} (a real threaded
  solid along +Z from origin; union for a screw, boolean difference for a
  threaded hole),
import_part{doc,path?|from?:doc_id,name?} (register a part for assembly: `path`
  is a .wcad file on the server, `from` snapshots an open document. Give one.
  The part program is embedded in this document, so save/open never needs the
  source again; nothing is placed yet. Replays the part once to validate it),
add_part{doc,name,part} (register an already-embedded part snapshot — the
  program step import_part records; agents normally use import_part),
add_instance{doc,part,name,translation?,rotation_deg?,scale?=1,parameters?}
  (place a part; every printable solid becomes an entity named
  "<instance>.<member>" and returns its ids. parameters overrides the part's
  saved knob values for this instance only; values may be expressions over
  the ASSEMBLY parameters. Instances move as a rigid group),
add_connector{doc,instance,name,point:[x,y,z],axis?:[x,y,z]} (named attachment
  point in instance-local coordinates; mates use these),
move_instance{doc,instance,translation?,rotation_deg?,scale?} (move one
  instance and all its members; omitted parts stay),
mate{doc,kind:"coincident"|"distance"|"parallel"|"angle",a:{instance,connector},
  b:{instance,connector},offset?} (moves instance b so its connector meets a's:
  coincident = points together + axes anti-parallel; distance = same with a gap
  along a's axis (mm); parallel = align axes only; angle = points together and
  b's axis offset degrees from a's. Mates solve sequentially in program order),
bill_of_materials{doc} (rows per part: item number, quantity, current entities,
  total world volume, materials, mass_g and cost (USD); plus document totals;
  session query),
check_interference{doc,instances?,entities?,tolerance?=0.001} (pairwise
  volumetric overlap: by default across placed instances; or pass `entities`
  to check specific solids. Returns ok, checked and hits with intersection
  volume/bbox; session query),
export_drawing{doc,path,format?:"svg"|"dxf",views?,scale?,hidden_lines?,
  dimensions?,balloons?,bom?,title?,annotations?} (multi-view engineering
  drawing: default front/top/right/iso in third-angle layout, feature +
  silhouette edges, hidden-line removal, overall dimensions in true model mm,
  part balloons, BOM table, title block. A view is "front"/"top"/"right"/
  "left"/"back"/"bottom"/"iso" or {view,at:[x,y],dimensions?} in sheet mm;
  annotations are {text,at:[x,y]}; session command),
save_document{doc,path,source?} (writes a .wcad JSON file: the program plus
  the current parameter values; geometry is rebuilt by replay, so the file stays
  small and diffable. source={language,text} embeds the authoring script as the
  canonical recipe copy; it is never executed. save -> open -> save is a no-op
  diff; session command, changes no model state. A provided source is
  remembered on the document, so get_program returns it immediately too),
open_document{path,name?} (loads a .wcad file as a NEW document and replays it;
  returns doc, name, counts; the stored name is used unless name overrides),
get_program{doc} (read-only program + parameters + embedded source — the
  complete handoff: the source is the canonical script; the program is the
  replayable IR. `suppressed` lists step indexes replay skips),
patch_program{doc,ops,dry_run?} (edit the program in place. Each op is
  {op:"insert",at,command}, {op:"remove",at}, {op:"replace",at,command} or
  {op:"reorder",from,to}; indexes are 0-based and ops apply in order. Commands
  address CURRENT entity ids; the replay rewrites references as ids shift.
  dry_run:true returns surviving/renumbered/removed/created ids and a full
  scene `diff` without changing the document. A real patch is one undo step.
  Direct edits do not rewrite an embedded source; export_python reconstructs
  the current program as a runnable script),
suppress{doc,steps,suppress?=true} (turn 0-based program steps off/on without
  deleting them. Saved with the document; ids before the step are unchanged,
  later ids may shift and the result reports `renumbered`/`removed`. A feature
  later steps still use is rejected — suppress those steps too),
batch{commands,atomic?=false} (run raw bus commands as one unit; atomic:true
  rolls every document change back if any command fails. Results come back in
  order; files already written are not retracted),
pick{doc,ray:{origin:[x,y,z],direction:[x,y,z]},tolerance?} (world-space ray
  at the solids; answers {hit:{entity,name,distance,point,normal,face,edge?}}
  or hit:null on a miss. The selectors are ready to pass straight back:
  face -> add_sketch{plane:{face:<face>}} (its body is exactly
  {entity,normal,near}; the Python client's plane= takes it directly);
  edge -> fillet/chamfer{edges:<edge>} (exactly {direction,near,near_radius},
  present only when the hit is within tolerance mm of a sharp edge). This is
  how a human click in the viewer reaches the agent; sketches are skipped),
faces{doc,entity} (list the planar faces of one solid: id, descriptor,
  normal/area/center, adjacent edge ids, a stable fingerprint, an exact
  geometry hash and a selector ready for add_sketch{plane:...}; session query),
edges{doc,entity,min_angle_deg?} (list logical sharp edges: collinear/tangent
  runs are one edge, so a tessellated rim is one curve with `segments`, not 32
  candidates. Each carries endpoints/direction or midpoint, adjacent face ids,
  fingerprint/geometry and a selector ready for fillet/chamfer{edges:...};
  session query),
validate{doc,entities?,min_wall_mm?,overhang_angle_deg?,layer_height_mm?,
  min_feature_mm?,bridge_angle_deg?} (trust + printability check, changes no
  state). For every solid: watertight/open/non-manifold edges, degenerate
  triangles, self-intersections, signed world volume, world bbox and an
  approximate thinnest wall (inward rays; sub-0.05 mm contact is treated as
  numerical noise). Its `print` report adds overhang area/regions at
  overhang_angle_deg (default 45), bridge spans (bridge_angle_deg, default 10),
  first-layer contact (layer_height_mm, default 0.2), minimum feature, connected
  shells/islands, closed cavities with a suggested drainage-hole size, and cups.
  For every sketch: solver status, over-constrained/diagnostics and whether the
  profile is closed.
  Returns ok (mesh trust), printable (print checks), per-entity reports, totals,
  warnings and references. Print warnings appear only when a print parameter is
  supplied. Each stored selector (fillet, chamfer, face sketch plane, hole,
  mirror, mate) is re-checked: ok, moved (same feature, new coordinates),
  ambiguous, missing, drift (now binds a different feature) or unbound.
  entities restricts the check; min_wall_mm sets the thin-wall pass/fail
  threshold (omit to report the estimate without failing),
suggest_orientation{doc,entities?,overhang_angle_deg?,layer_height_mm?,
  bed_width_mm?,bed_height_mm?,limit?} (score stable orientations: machine axes
  + planar faces; each candidate carries absolute rotation_deg and translation
  for transform, height, footprint, contact/overhang area, bridge span and a
  lower-is-better score; session query),
arrange{doc,entities?,bed_width_mm?,bed_height_mm?,spacing_mm?,center?} (lay
  solids on the bed with their lowest point at z=0, shelf-packed with spacing and
  centered; records ordinary transform steps, so it undoes as one action),
fit{doc,class:"free"|"sliding"|"snug"|"press",nominal_mm?,hole_mm?,material?,
  printer_tolerance_mm?,hole_shrink_mm?,elephant_foot_mm?} (convert a nominal
  shaft or hole into a modeled hole/shaft with clearance, material hole-shrink
  and an elephant-foot recommendation; session query),
diff_scene{doc,against?:doc_id,path?:".wcad"} (compare two revisions by
  entity signature; pass exactly one baseline: another open document or a
  saved file, which is replayed read-only and not added to the document list.
  Returns changed + added/removed/modified with the changed fields and
  volume/triangle deltas, unchanged count, parameter changes, program change,
  references whose binding changed, and stable scene signatures. Use it to
  prove a rebuild did what you meant),
export_stl{doc,path,entities?} (binary STL, world space, absolute path — a
  slicer-ready file; not part of the program, changes no state; sketches are
  display-only and skipped),
export_3mf{doc,path,entities?} (3MF package, world space, absolute path — one
  named object per solid with its display color, millimeter unit; prefer it
  over STL when the build has several parts or colors; session command),
export_dxf{doc,path,entities?,curves?: "elements"|"profiles"|"both"} (2D ASCII
  DXF R12; sketch-local (u,v) millimeters, NOT world XYZ. entities defaults to
  every sketch; naming an extrude/revolve/sweep/loft solid exports its source
  sketch's closed profile loops, so a part's footprint needs no sketch id.
  curves defaults to "elements"; "profiles" writes only the closed loops a
  cutter needs, "both" separates them onto a profile layer; session command),
export_svg{doc,path,entities?,curves?} (same selection and coordinates as
  export_dxf, but an SVG with an mm viewBox: Y flipped upright, construction
  geometry dashed, profile loops on their own group; session command),
export_step{doc,path,entities?} (STEP ISO 10303-21 AP214, world space: an
  explicit FACETED B-rep, one planar FACE_SURFACE per triangle with welded
  shared points. Closed edge-manifold pieces become FACETED_BREP solids; open
  or non-manifold pieces become SHELL_BASED_SURFACE_MODEL surfaces. Returns
  entities/solids/surfaces/faces/bytes. Analytic cylinders/cones are NOT
  reconstructed, and STEP import is not supported — use STL/OBJ/3MF for
  incoming meshes; session command),
render_offscreen{doc,width?,height?,eye?,target?,up?,fov_deg?,ortho?,background?,path?}
  (headless PNG; prefer the wright_render tool, which returns the image inline.
  ortho:true gives a parallel projection for front/top/side views).
Numeric arguments accept expression strings over parameters. Commands run in order;
execution stops at the first error (reported with a recovery hint)."""

SCENE_DESCRIPTION = """Inspect an wright document: parameters and entities with
placement, a world-space bounding box per entity (meshes are entity-local;
world_bbox already includes translation/rotation/scale), vertex/triangle
counts, and for sketches a compact solver + resolved plane summary (origin,
outward normal, in-plane X) — no raw meshes. Part members also carry their
owning `instance` id and `part` name (their name is `<instance>.<member>`)."""

RENDER_DESCRIPTION = """Render the document to a PNG image, returned inline as an
image content block — so you can SEE the scene and compare it against what the
user asked for, then iterate. Args: doc, width? (default 800), height? (600),
eye?/target?/up? as [x,y,z] camera vectors (omit eye for an auto-framed 3/4
view of the whole scene), fov_deg? (default 40), path? to also save the PNG.
Headless software render, deterministic; changes no state."""

TOOLS = [
    {
        "name": "wright_execute",
        "description": EXECUTE_DESCRIPTION,
        "inputSchema": {
            "type": "object",
            "properties": {
                "commands": {
                    "type": "array",
                    "items": {"type": "object"},
                    "description": "Command objects, executed in order",
                },
            },
            "required": ["commands"],
        },
    },
    {
        "name": "wright_scene",
        "description": SCENE_DESCRIPTION,
        "inputSchema": {
            "type": "object",
            "properties": {
                "doc": {"type": "integer", "description": "Document id from create_document"},
            },
            "required": ["doc"],
        },
    },
    {
        "name": "wright_render",
        "description": RENDER_DESCRIPTION,
        "inputSchema": {
            "type": "object",
            "properties": {
                "doc": {"type": "integer", "description": "Document id from create_document"},
                "width": {"type": "integer", "description": "Image width, default 800"},
                "height": {"type": "integer", "description": "Image height, default 600"},
                "eye": {"type": "array", "items": {"type": "number"}, "minItems": 3, "maxItems": 3,
                        "description": "Camera position; omit for auto-framed 3/4 view"},
                "target": {"type": "array", "items": {"type": "number"}, "minItems": 3, "maxItems": 3},
                "up": {"type": "array", "items": {"type": "number"}, "minItems": 3, "maxItems": 3},
                "fov_deg": {"type": "number", "description": "Vertical field of view, default 40"},
                "path": {"type": "string", "description": "Also save the PNG to this absolute path"},
            },
            "required": ["doc"],
        },
    },
]


# --- result summarization (token economy) ------------------------------------


def mesh_stats(mesh: dict) -> dict:
    """Stats for a mesh payload. `local_bbox` is in entity-local space — use
    the entity's `world_bbox` for placement and size."""
    positions = mesh.get("positions") or []
    if positions:
        mins = [min(p[a] for p in positions) for a in range(3)]
        maxs = [max(p[a] for p in positions) for a in range(3)]
        local_bbox = [[round(v, 3) for v in mins], [round(v, 3) for v in maxs]]
    else:
        local_bbox = None
    return {
        "verts": len(positions),
        "tris": len(mesh.get("indices") or []) // 3,
        "local_bbox": local_bbox,
    }


def summarize(obj: Any) -> Any:
    """Replace every `mesh` payload with its stats, recursively. Inline PNG
    payloads are replaced with a pointer to `wright_render` (images belong
    in the vision loop, not in a text dump)."""
    if isinstance(obj, dict):
        out = {}
        for k, v in obj.items():
            if k == "mesh" and isinstance(v, dict):
                out[k] = mesh_stats(v)
            elif k == "png_base64" and isinstance(v, str):
                out[k] = f"<{len(v)} base64 chars — call wright_render to get this as an image>"
            else:
                out[k] = summarize(v)
        return out
    if isinstance(obj, list):
        return [summarize(v) for v in obj]
    return obj


# --- the server ---------------------------------------------------------------


class McpServer:
    def __init__(self) -> None:
        self._cad: Wright | None = None
        # Last transport generation reported to the agent. The Python client
        # bumps its generation when it reconnects; a change means the server
        # may have restarted (and lost its in-memory documents).
        self._generation = 0

    @property
    def cad(self) -> Wright:
        # Spawn the engine lazily so `tools/list` works even without a binary.
        if self._cad is None:
            attach = os.environ.get("WRIGHT_CONNECT")
            if attach:
                # Live mode: draw into a shared backend (e.g. the one a
                # viewer is watching) instead of a private engine.
                host, sep, port = attach.rpartition(":")
                if not sep:
                    host, port = "127.0.0.1", attach
                self._cad = Wright.connect(host, int(port))
                print(f"wright-mcp: attached to shared backend {host}:{port} (live preview)", file=sys.stderr)
            else:
                self._cad = Wright.spawn()
                print(
                    "wright-mcp: spawned a private engine (set WRIGHT_CONNECT=host:port "
                    "to draw into a shared backend a viewer can watch)",
                    file=sys.stderr,
                )
        return self._cad

    def _reconnect_note(self) -> dict | None:
        """Report a transport rebuild once per generation.

        Documents live in the engine process, so a restarted server is empty.
        Telling the agent explicitly beats a bare ``unknown document`` error:
        the notice carries the documents that actually survived and tells the
        agent to replay its script when they are gone.
        """
        cad = self._cad
        if cad is None or cad.generation == self._generation:
            return None

        docs: list[dict] | None
        try:
            docs = cad.call("list_documents", {})["docs"]
        except Exception:  # noqa: BLE001 — the notice must never fail a tool call
            docs = None
        # list_documents may itself have reconnected; report the latest state.
        self._generation = cad.generation

        if docs is None:
            message = (
                "The client reconnected to the wright server, but it is not "
                "answering again yet. Retry; if the server restarted, its "
                "document space is empty."
            )
            return {"generation": self._generation, "open_documents": None, "message": message}

        ids = [d["doc"] for d in docs if isinstance(d, dict) and "doc" in d]
        if ids:
            message = (
                "The client reconnected to the wright server. Open documents: "
                f"{ids}. A restarted server starts empty — if your document id "
                "is missing, re-create it and replay your script; previous "
                "entity ids are gone."
            )
        else:
            message = (
                "The client reconnected to the wright server, but it has no open "
                "documents: a restarted engine starts empty and cannot restore "
                "in-memory documents. Re-create the document and replay your "
                "script (create_document + commands, or open_document from a "
                "saved .wcad) before continuing; previous entity ids are gone."
            )
        return {"generation": self._generation, "open_documents": ids, "message": message}

    def _note_on_error(self, text: str) -> str:
        """Append a reconnect explanation to a failed tool message."""
        note = self._reconnect_note()
        return f"{text}\n{note['message']}" if note is not None else text

    # -- MCP methods ---------------------------------------------------------

    def handle(self, msg: dict) -> dict | None:
        method = msg.get("method")
        rid = msg.get("id")
        if rid is None:
            return None  # notifications get no response
        try:
            if method == "initialize":
                requested = (msg.get("params") or {}).get("protocolVersion")
                result = {
                    # Echo the client's revision when offered (spec-safe:
                    # answering with a supported version is what counts).
                    "protocolVersion": requested or DEFAULT_PROTOCOL_VERSION,
                    "capabilities": {"tools": {"listChanged": False}},
                    "serverInfo": SERVER_INFO,
                    "instructions": INSTRUCTIONS,
                }
            elif method == "tools/list":
                result = {"tools": TOOLS}
            elif method == "tools/call":
                params = msg.get("params") or {}
                result = self._dispatch_tool(params.get("name"), params.get("arguments") or {})
            elif method == "ping":
                result = {}
            else:
                return self._error(rid, -32601, f"method not found: {method}")
        except Exception as e:  # noqa: BLE001 — protocol boundary
            return self._error(rid, -32603, f"{type(e).__name__}: {e}")
        return {"jsonrpc": "2.0", "id": rid, "result": result}

    @staticmethod
    def _error(rid: Any, code: int, message: str) -> dict:
        return {"jsonrpc": "2.0", "id": rid, "error": {"code": code, "message": message}}

    # -- tools -----------------------------------------------------------------

    def _dispatch_tool(self, name: Any, args: dict) -> dict:
        if name == "wright_execute":
            return self._tool_execute(args)
        if name == "wright_scene":
            return self._tool_scene(args)
        if name == "wright_render":
            return self._tool_render(args)
        return {
            "content": [{"type": "text", "text": f"unknown tool: {name}"}],
            "isError": True,
        }

    def _tool_execute(self, args: dict) -> dict:
        commands = args.get("commands")
        if not isinstance(commands, list):
            return {"content": [{"type": "text", "text": "commands must be an array"}], "isError": True}
        if len(commands) > MAX_BATCH:
            return {
                "content": [{"type": "text", "text": f"batch too large: {len(commands)} > {MAX_BATCH}"}],
                "isError": True,
            }

        results: list[dict] = []
        error: str | None = None
        for i, cmd in enumerate(commands):
            if not isinstance(cmd, dict) or "method" not in cmd:
                error = f"step {i}: command must be an object with a 'method' key"
                break
            method = cmd["method"]
            params = cmd.get("params")
            if params is None:
                params = {k: v for k, v in cmd.items() if k != "method"}
            try:
                res = self.cad.call(method, params)
                results.append({"step": i, "method": method, "result": summarize(res)})
            except WrightError as e:
                results.append({"step": i, "method": method, "error": str(e)})
                error = str(e)
                break
            except Exception as e:  # noqa: BLE001
                results.append({"step": i, "method": method, "error": f"{type(e).__name__}: {e}"})
                error = "unexpected failure, see last step"
                break

        note = self._reconnect_note()
        payload: dict[str, Any] = {"executed": len(results), "results": results}
        if note is not None:
            payload["reconnected"] = note
        if error is not None:
            payload["stopped_early"] = True
            payload["remaining"] = len(commands) - len(results)
            hint = "fix the error in the last result, then re-send the remaining commands"
            if note is not None and "unknown document" in error:
                # A restart is the usual reason a previously valid doc id
                # vanished; say so instead of sending the agent in circles.
                hint = note["message"]
            payload["hint"] = hint
        return {"content": [{"type": "text", "text": json.dumps(payload, separators=(",", ":"))}]}

    def _tool_scene(self, args: dict) -> dict:
        try:
            doc = int(args["doc"])
        except (KeyError, TypeError, ValueError):
            return {"content": [{"type": "text", "text": "integer 'doc' is required"}], "isError": True}
        try:
            scene = self.cad.call("get_scene", {"doc": doc})
            params = self.cad.call("get_parameters", {"doc": doc})
        except WrightError as e:
            return {"content": [{"type": "text", "text": self._note_on_error(str(e))}], "isError": True}

        entities = []
        for e in scene["entities"]:
            stats = mesh_stats(e["mesh"])
            entry = {
                "id": e["id"],
                "name": e["name"],
                "translation": e["translation"],
                "rotation_deg": e["rotation_deg"],
                "scale": e["scale"],
                # World space, after the entity transform — trust this
                # one for placement. `stats.local_bbox` is entity-local.
                # .get so an older attached engine (pre-world_bbox) does
                # not crash inspection.
                "world_bbox": e.get("world_bbox"),
                "color": e["color"],
                **stats,
            }
            # Sketches carry a compact solver + plane summary: enough to
            # verify where the drawing sits without shipping every line.
            sketch = e.get("sketch")
            if sketch:
                entry["sketch"] = {
                    "solved": sketch.get("solved"),
                    "residual": sketch.get("residual"),
                    "dof": sketch.get("dof"),
                    "over_constrained": sketch.get("over_constrained", False),
                    "diagnostics": sketch.get("diagnostics", []),
                    "elements": len(sketch.get("elements", [])),
                    "constraints": len(sketch.get("constraints", [])),
                    "plane": sketch.get("plane"),
                }
            entities.append(entry)
        payload = {"doc": doc, "parameters": params["params"], "entities": entities}
        note = self._reconnect_note()
        if note is not None:
            payload["reconnected"] = note
        return {"content": [{"type": "text", "text": json.dumps(payload, separators=(",", ":"))}]}

    def _tool_render(self, args: dict) -> dict:
        params = {k: v for k, v in args.items() if v is not None}
        try:
            r = self.cad.call("render_offscreen", params)
        except WrightError as e:
            return {"content": [{"type": "text", "text": self._note_on_error(str(e))}], "isError": True}
        meta = {"doc": r["doc"], "width": r["width"], "height": r["height"], "bytes": len(r["png_base64"])}
        if "path" in r:
            meta["saved_to"] = r["path"]
        note = self._reconnect_note()
        if note is not None:
            meta["reconnected"] = note
        return {
            "content": [
                {"type": "image", "data": r["png_base64"], "mimeType": "image/png"},
                {"type": "text", "text": json.dumps(meta, separators=(",", ":"))},
            ]
        }


def main() -> None:
    """MCP stdio loop: one JSON-RPC message per line."""
    server = McpServer()
    for line in sys.stdin:
        line = line.strip()
        if not line:
            continue
        try:
            msg = json.loads(line)
        except json.JSONDecodeError:
            continue  # unparseable lines get no response (id unknowable)
        response = server.handle(msg)
        if response is not None:
            sys.stdout.write(json.dumps(response, separators=(",", ":")) + "\n")
            sys.stdout.flush()


if __name__ == "__main__":
    main()
