# Wright CAD

<p align="center">
  <img src="logo.svg" width="160" alt="Wright CAD logo — a snowman assembled from primitives"/>
</p>

A CAD whose primary user is an **AI agent**.

> The logo is the "hello world" of this CAD: three spheres, a cylinder and a
> couple of boxes, assembled by a parametric program — drawn as an SVG from the
> same primitives the engine tessellates. See
> `examples/draw_snowman_family.py` for the living version.

The architecture inverts the usual GUI-first design: the headless core and its
command bus are the product; the human-facing viewer is just another client — as
is the Python client an agent drives. Mesh import turns an STL, OBJ or 3MF into
an ordinary editable solid, so an agent handed an existing part can participate
instead of starting over. DXF/SVG export closes the drawing loop, STEP export
hands the solids to another CAD kernel as an explicit faceted B-rep, and
assemblies add reusable parts, mates, interference checks and dimensioned
multi-view drawing sheets.

```
┌───────────────┐   JSON-RPC (stdio/TCP)   ┌───────────────┐   ┌──────────────────┐
│ agent /       │ ───────────────────────► │ wright server │ ─►│ wright-core      │
│ python /      │ ◄─────────────────────── │ (transport)   │   │ documents        │
│ any language  │                          └───────────────┘   │ entities         │
└───────────────┘                                              │ history / undo   │
                                                               │ tessellation     │
┌───────────────┐          same bus (TCP)                      │ booleans         │
│ wright-view   │ ───────────────────────────► get_scene ─────►│ fillet / chamfer │
│ (wgpu, dumb)  │  pulls snapshots; owns no document state     └──────────────────┘
└───────────────┘
```

## Showcase: a drawing in, a parametric part out

The agent is handed a picture — `examples/example_body.png`, a front and a 3/4
view of a cross block — and writes `examples/cross_block.py`, which rebuilds the
part as a parametric program:

![the drawing and the finished model Wright CAD rebuilt from it](examples/cross_block/compare.png)

The dimensions are read off the drawing rather than guessed: its 9 mm scale bar
gives ~12.4 px/mm, and its front panel is 553 × 614 px, so the body is 45 × 45
mm in plan, 15 mm arms and 50 mm tall. The camera was fitted on a sharp baseline
so the silhouette comparison stays unambiguous (0.99 IoU), and that baseline's
front view matches the drawing's aspect to ~0.1% (0.898 vs 0.899). The script
then rounds the part the way the drawing does — a 1 mm `fillet` on the top and
bottom rims, driven by the `round_r` knob. The drawing's plan corners are sharp,
so the twelve vertical edges are deliberately left alone.

![the cross block with its top and bottom rims rounded and sharp plan corners](examples/cross_block/rounded.png)

The loop: **read** the two views → **author** the parametric program
(`span`/`arm`/`height`, twelve sketch lines, `extrude`) → **check** the
silhouette with `render_offscreen` from the drawing's camera → **finish** the
rims with `fillet` and normals filters (`{normals:[[0,0,1]]}` top,
`{normals:[[0,0,-1]]}` bottom) → **deliver** STL/3MF for the slicer and a
matching `.wcad` that reopens with its knobs intact.

Artifacts live in `examples/cross_block/`: `iso.png`, `front.png`,
`cross_block.stl`, `cross_block.3mf` and `cross_block.wcad` (the sharp
baseline), plus `rounded.png`, `cross_block_rounded.stl`,
`cross_block_rounded.3mf` and `cross_block_rounded.wcad` (the finished part),
and `compare.png`. `.venv/bin/python examples/cross_block.py` regenerates them.

## Layout

| Path | What it is |
|---|---|
| `crates/protocol` | Wire types: commands, results, meshes (serde/JSON) |
| `crates/core` | Headless engine: documents, parameters, sketches, booleans, fillets, persistence, assemblies, DXF/SVG/STEP drawings and sheets, deterministic rendering |
| `crates/server` | `wright` binary — line-delimited JSON-RPC on stdio or TCP |
| `crates/viewer` | `wright-view` — wgpu renderer, talks only the bus |
| `python/wright` | Dependency-free sync Python client (the agent API) and MCP server |
| `examples/` | Agent-facing demos; `examples/cross_block/` is the committed showcase |
| `index.html`, `fonts/`, `logo.svg` | Static project page (GitHub Pages), with self-hosted fonts |

## Design rules

1. **The command bus is the only API.** No side channel exists between the
   viewer and the engine; every client gets the same, scriptable surface.
2. **Deterministic.** Integer ids, stable tessellation, replayable command
   sequences — tests and agents can rely on exact repetition.
3. **Agent ergonomics are features.** Errors carry recovery hints
   ("run create_document first, or list_documents"); results are
   self-contained (a new entity arrives with its full mesh — no second
   round trip); ids are small and predictable.
4. **The renderer is dumb.** It draws buffers; it owns no logic.

## Protocol (v1)

Line-delimited JSON-RPC 2.0, one request/response per line:

```json
{"jsonrpc":"2.0","id":1,"method":"add_box","params":{"doc":1,"name":"base","center":[0,0,"wall/2"],"size":[80,40,"wall*2"]}}
{"jsonrpc":"2.0","id":1,"result":{"entity_created":{"doc":1,"entity":1,"mesh":{...}}}}
```

Commands, grouped by intent:

- **Documents**: `initialize`, `list_documents`, `create_document`,
  `close_document`
- **Primitives**: `add_box`, `add_sphere`, `add_cylinder`
- **Modelling**: `boolean`, `transform`, `set_color`, `set_material`, `delete`,
  `fillet`, `chamfer`
- **Shape leverage**: `linear_pattern`, `circular_pattern`, `mirror`,
  `shell`, `hole`, `sweep`, `loft`, `thread`
- **Assemblies**: `import_part`, `add_part`, `add_instance`, `add_connector`,
  `move_instance`, `mate`, `bill_of_materials`, `check_interference`
- **Parameters**: `set_parameter`, `remove_parameter`, `get_parameters`
- **Program editing**: `patch_program` (insert/remove/replace/reorder, dry run
  with an id map and scene diff), `suppress` (turn steps off without deleting
  them), `batch` (run commands as one unit, optionally atomic)
- **Sketches and solids**: `add_sketch`, `add_sketch_line`,
  `add_sketch_circle`, `add_sketch_arc`, `add_sketch_spline`, `constrain`,
  `remove_constraint`, `move_sketch_point`, `delete_sketch_element`,
  `trim_sketch`, `extend_sketch`, `offset_sketch`, `extrude`, `revolve`
- **Interoperability**: `import_mesh` (STL/OBJ/3MF → solid), `add_mesh` (raw
  triangles; the baked program step import records)
- **Import → parametric**: `import_dxf`/`import_svg` (2D drawing → sketch),
  `add_sketch_curves` (baked sketch step), `section`/`project` (solid → sketch
  geometry), `primitives` (planar faces, cylinders, holes), `repair_mesh`
- **Queries and I/O**: `get_scene`, `get_program`, `pick`, `faces`, `edges`,
  `validate`, `suggest_orientation`, `arrange`, `fit`, `list_materials`,
  `diff_scene`, `export_stl`, `export_3mf`, `export_step`,
  `export_dxf`, `export_svg`, `export_drawing`, `render_offscreen`,
  `save_document`, `open_document`
- **History**: `undo`, `redo`

`get_program` returns the program, the parameters, the suppressed step indexes
and any embedded authoring source — the complete handoff for the
script-as-source workflow.

`fillet{entity,radius,edges?}` and `chamfer{entity,distance,edges?}` reshape a
solid **in place** (same id, name, color, transform) and replay with the
program. `edges` is a geometric filter rather than an index — an AND of
`direction` (+`angle_tol_deg`), `near` (+`near_radius`), `normals` (one face
normal = all its edges, two = the edge between them) and `convex` — so "the
vertical corners" or "the top rim" keeps its meaning when a parameter moves
the geometry. Omit it to treat every sharp edge; `min_angle_deg` (default 30)
separates real edges from tessellation seams.

`faces{entity}` and `edges{entity}` make that resolution inspectable. Faces are
maximal planar patches with a deterministic 1-based id, a readable descriptor
(normal, area, center), adjacent edge ids, a stable `fingerprint` (the same
feature across a resize) and an exact `geometry` hash (changes when it moves).
Edges are **logical runs**: collinear or tangent segments that share an endpoint
are one edge, so a 32-segment cylinder rim is a single `curve` with
`segments:32`, while a box corner still splits into three lines. Each listing
includes a ready-made selector, so an agent addresses "face #4" or "the edge
between faces #1 and #4" instead of guessing a `near` point.

Every modeling command that takes a selector (`fillet`, `chamfer`,
`add_sketch` face planes, `hole`, `mirror`, mates) reports `matched`,
`ambiguous` and a `fingerprint`. On first use the engine stores the binding in
the program step; a later replay compares against it. `validate` re-checks the
stored references (`ok`, `moved`, `ambiguous`, `missing`, `drift`, `unbound`)
and `diff_scene` names any selector whose binding changed — so a parametric
rebuild fails loudly instead of silently filleting another edge. A `moved`
reference is the same feature with new coordinates (a knob resized it); `drift`
means it now selects a different logical feature. `set_parameter` also returns
`references_drifted` after a replay, so a knob turn can fail loudly before the
next `validate`.

`pick{doc,ray:{origin,direction},tolerance?}` casts a world-space ray and answers
`{hit:{entity,name,distance,point,normal,face,edge?}}` or `hit:null` (a valid
answer, not an error). Selectors are ready to use: `face` is the body of
`add_sketch`'s face plane, and `edge` is exactly the filter `fillet`/`chamfer`
take. Sketches are display-only and never absorb a ray. The viewer turns a click
into this command; the human copies the selector and the agent passes it back —
no parallel face or edge ID namespace.

`import_mesh{doc,path,name?,scale?,format?,smooth_angle_deg?,color?}` reads an
STL (binary or ASCII), OBJ or 3MF **on the server** and adds it as a solid.
3MF units convert to millimeters; STL/OBJ are assumed millimeters, and `scale`
multiplies after that. Vertices are welded by exact position, degenerate and
duplicate triangles dropped, inward winding flipped outward, and crease-aware
normals derived (`smooth_angle_deg`, default 30). The cleaned triangles become
an `add_mesh` program step, so a `.wcad` replays without the source file.
Imported solids transform, boolean, fillet, validate and export like any other
entity. A non-watertight file imports anyway — `validate` reports it and
booleans reject it rather than guessing a repair. `repair_mesh{entity}` is the
explicit repair: it welds, orients each connected shell outward, drops
triangles that make an edge non-manifold, and fills closed boundary loops. The
command is part of the program, so the repair replays and the entity keeps its
id.

### Import → parametric (M22)

The 2D importers close the loop from an existing drawing to a rule:

`import_dxf{doc,path,name?,plane?,scale?,auto_constraints?}` reads ASCII DXF
(R12-friendly) `LINE`, `CIRCLE`, `ARC`, `POLYLINE` and `LWPOLYLINE` entities;
`$INSUNITS` converts to millimeters. `import_svg` reads `line`, `circle`,
`ellipse`, `rect`, `polyline`, `polygon` and `path` (M/L/H/V/C/S/Q/T/A/Z),
composes nested group transforms, honors `width`/`height`/`viewBox`, and flips
SVG's Y-down frame to the sketch's Y-up frame. Both record an
`add_sketch_curves` program step with the baked elements, so a `.wcad` replays
without the source drawing. Optional auto-constraints (default on) join
near-coincident endpoints, snap near-axis-aligned lines to exact
horizontal/vertical, and add approximate tangency, so the imported profile
behaves like authored geometry.

`solid → sketch` goes the other way. `section{entity,plane}` intersects a
solid or imported mesh with a plane and returns a new sketch of closed loops
(ready to extrude); `project{entity,plane,min_angle_deg?}` projects boundary,
sharp and front-facing silhouette edges to give an outline. Both are modeling
commands, with the plane in the same datum/custom forms as mirror planes.

`primitives{entity}` lists what an imported part is made of: the planar face
groups from M20, fitted cylindrical surfaces (axis, center, radius, height and
inner/outer), and hole candidates with depth and a through/blind guess. That is
the bridge from "a triangle soup that looks like a bracket" to
`add_cylinder` + `hole` + `fillet` with named dimensions. `repair_mesh{entity}`
turns a non-watertight import into a booleans- and validate-ready solid; its
report says how many holes it filled, triangles it flipped or removed, and
whether the result is open/manifold.

`export_dxf` and `export_svg` write a **2D drawing** of sketches in the sketch's
own (u, v) millimeters, not world XYZ. `entities` defaults to every sketch;
naming a solid made by `extrude`, `revolve`, `sweep` or `loft` exports its source
sketch's closed profile loops. `curves` is `elements` (default: native lines,
circles and arcs, splines as polylines), `profiles` (closed loops only) or
`both`. DXF is ASCII R12 with millimeter units and separate drawing,
construction (dashed) and profile layers; SVG flips Y upright and uses a
millimeter viewBox. Construction geometry is exported but never part of a
profile. Both are session commands, never program entries, and byte-identical
for identical drawings.

`export_step{doc,path,entities?}` writes the selected solids (default: every
solid) as STEP ISO 10303-21 AP214 (`AUTOMOTIVE_DESIGN`) in world space. wright
is a tessellation kernel, so it writes a **faceted B-rep**: every triangle is a
planar `FACE_SURFACE` over welded `POLY_LOOP`s, and each connected,
edge-manifold, consistently wound piece becomes a `FACETED_BREP` in a
`CLOSED_SHELL`. Open or non-manifold pieces become `OPEN_SHELL`s in a
`SHELL_BASED_SURFACE_MODEL`. Names ride on the `FACETED_BREP`s, colors are not
written, and the header timestamp is fixed, so repeated exports are
byte-identical. STEP **import** is deliberately absent: real STEP files are
mostly trimmed analytic surfaces, a different kernel problem.

Conventions: millimeters, degrees, RGBA floats. Z is up — primitives are
axis-aligned and `add_cylinder` extrudes along +Z. Document ids start at 1;
`create_document` takes an optional explicit `doc`, so create + populate is one
round trip. Geometry is baked centered on the entity origin; placement is
`T · Rz · Ry · Rx · S` (extrinsic XYZ rotation). `get_scene` reports the local
mesh plus a world-space `world_bbox` per entity; a placed part member also
carries its owning `instance` id and `part` name.

## The program is the function; the script is the source of truth

The user describes intent; **the agent authors a parametric program**:

1. Named parameters (`wall`, `plate_len`) — values may be expressions over
   other parameters.
2. Modeling commands with dimensions as expressions:
   `size = [80, 40, "wall*2"]`.
3. `set_parameter` re-executes the program with **entity ids preserved** for
   ordinary knobs (and remapped by feature identity when a pattern count
   changes the structure).
   Geometry is content-memoized per feature: a knob change regenerates only
   the features downstream of it, while independent features reuse their
   cached meshes. The full replay stays the correctness oracle.

The expression grammar is deliberately tiny and safe (`+ - * /`, parentheses,
identifiers, unary minus) — no functions, no injection surface. Errors are
agent-shaped: a typo lists the valid parameter names.

```python
cad.set_parameter(doc, "wall", 2.0, description="plate thickness [mm]")
cad.set_parameter(doc, "plate_len", 80.0)
base = cad.add_box(doc, "base", center=(0, 0, "wall/2"),
                   size=("plate_len", 40, "wall*2"))
cad.set_parameter(doc, "wall", 4.0)   # downstream features rebuild; ids unchanged
```

Every geometry-producing step memoizes its result under a key folded from its
resolved arguments **and the keys of the geometry it consumes** (an extrude
follows its profile, a boolean its operands, a fillet its solid). The cache is
derived state — never saved, empty after `open_document` — so a cold replay
always reproduces the same bytes. `set_parameter` reports `rebuilt`, `reused`
and `steps`, and a no-op turn rebuilds nothing.

The durable artifact is the script; the `.wcad` file is a replayable snapshot of
an instance — the program, the current parameter values, and a copy of the
script.

```
part.py ──run──► wright bus ──save_document──► part.wcad
   ▲                                             │
   └──── get_script / export_python ◄────────────┘
```

- **Authoring**: the agent writes or edits `part.py`, runs it, renders and
  validates the result.
- **User knobs**: `set_parameter` is the fast path; `export_python` bakes the
  current values back into a script when an instance should become the new
  source.
- **Handoff**: `save_document(source={language, text})` embeds the script;
  `get_program` returns it immediately and after a later `open_document` (never
  executed); `get_script` returns the original text, or reconstructs a runnable
  script with `export_python`.
- **MCP-only documents** with no script are scratch state.

There is intentionally no GUI-style feature tree: the agent edits the script and
replays. The program is the intermediate representation; the script is the
editing surface. `set_parameter` covers the common "turn a knob" change.

## Editing the program without a feature tree

The script is the editing surface, but an agent also needs to make a surgical
change without re-authoring the whole file. `patch_program` edits the program
in place:

```json
{"method":"patch_program","params":{"doc":1,"dry_run":true,"ops":[
  {"op":"insert","at":0,"command":{"method":"add_box","doc":1,"name":"foot",...}},
  {"op":"remove","at":4},
  {"op":"replace","at":2,"command":{"method":"fillet","doc":1,"entity":3,...}},
  {"op":"reorder","from":5,"to":1}
]}}
```

Indexes are 0-based and match `get_program`. Ops apply in order, so later
indexes see earlier edits. Commands inside the ops address entities by their
**current** ids. The engine replays the patched program and rewrites numeric
references by feature identity whenever id allocation moves: insert a box
before everything and the boolean step's `[1,2]` becomes `[2,3]` instead of
silently fusing the new box. A `replace` keeps the step's identity (a downstream
fillet follows the replacement); a `remove` that still has users fails with the
referencing step named. The result reports `surviving`, `renumbered` (old id →
new id and name), `removed` and `created`.

A dry run is the proof: `dry_run:true` replays a scratch copy, reports those
ids and a full `diff` object against the current scene, and changes nothing.
Committing the same ops is one undo step. Newly inserted selectors get the same
stored binding a live command would, so a later knob turn reports `moved`
rather than `unbound`.

Direct program edits do not rewrite the embedded authoring script: it still
describes the source that produced the pre-edit document. `export_python`
reconstructs a runnable script from the current program (and emits the
`suppress` call for a variant), so the handoff is to regenerate and re-embed it
with `save_document(source=...)` when the edit should become the new source.

`suppress{steps,suppress?}` is the A/B switch: it turns steps off without
deleting them. Suppression is saved with the document (an empty set is not
written), and `get_program` returns it. Ids before the suppressed step are
unchanged; ids after it may shift and the result reports the map. A step that
later steps still reference cannot be suppressed alone — suppress those steps
too, and the replay stays valid.

`batch{commands,atomic:true}` runs several bus commands as one unit. A failure
rolls every document, parameter, program and undo-history change back; files
already written are not retracted. Without `atomic` the successful prefix
stays, which is useful when a long import batch should keep what it got.

```python
# Insert a feature, preview the id shift, then commit it.
ops = [{
    "op": "insert", "at": 0,
    "command": {"method": "add_box", "doc": doc, "name": "foot",
                "center": [0, 0, 2], "size": [20, 20, 4]},
}]
preview = cad.patch_program(doc, ops, dry_run=True)
assert preview["diff"]["changed"]
cad.patch_program(doc, ops)                  # one undo step

# A/B variant: the last fillet is off, then on.
cad.suppress(doc, [3])
cad.validate(doc)
cad.suppress(doc, [3], suppress=False)
```

## Sketches

`add_sketch` defaults to XY at the origin, but `plane` can place it anywhere:

```json
{"method":"add_sketch","params":{"doc":1,"name":"top",
  "plane":{"datum":{"plane":"xy","offset":"wall*2"}}}}
{"method":"add_sketch","params":{"doc":1,"name":"tilted",
  "plane":{"custom":{"origin":[0,0,10],"normal":[0,1,1],"x_axis":[1,0,0]}}}}
{"method":"add_sketch","params":{"doc":1,"name":"pocket",
  "plane":{"face":{"entity":1,"normal":[0,0,1],"near":[0,0,20]}}}}
```

Each datum frame is right-handed, and the sketch's local +Z is its outward
normal: `xy` has X=+X, Y=+Y, normal=+Z; `xz` has X=+X, Y=+Z, normal=-Y; `yz` has
X=+Y, Y=+Z, normal=+X. A `face` plane is selected geometrically: the outward
`normal` picks a planar patch, and `near` chooses between parallel faces (omit
`normal` for the largest face). A positive `extrude` grows a boss, a negative
one cuts into the body; `boolean{difference}` removes the cutter. Faces are
re-found on replay, so a face sketch follows the body when a parameter moves it.

```python
top = cad.add_sketch(doc, "pocket", plane={"entity": base, "normal": [0, 0, 1]})
side = cad.add_sketch(doc, "boss", plane={"entity": base, "normal": [1, 0, 0]})
cad.add_sketch(doc, "plate_top", plane="xy", offset="wall*2")
```

Sketches cover more than lines and circles. `add_sketch_arc` adds a
counter-clockwise arc (degrees from the local +X), and `add_sketch_spline` adds
an interpolating Catmull-Rom curve through every point; both tessellate into
closed profiles for `extrude` and `revolve`.

```python
arc = cad.add_sketch_arc(doc, sketch, center=("w-corner", "corner"),
                         radius="corner", start_deg=-90, end_deg=0)
curve = cad.add_sketch_spline(doc, sketch,
                              [(0, 0), (20, -5), (40, 0)], closed=True)
guide = cad.add_sketch_line(doc, sketch, (0, 0), ("w", "h"), construction=True)
```

Constraints include `coincident`, `horizontal`, `vertical`, `parallel`,
`perpendicular`, `distance`, `radius`, `diameter`, `equal`, `concentric`,
`symmetric`, `angle`, `point_on_line` and `tangent`. Point references follow the
element kind: line `0`=start, `1`=end; circle `0`=center; arc `0`=center,
`1`=start, `2`=end; spline `i`=point `i`. `distance` also accepts a point and a
whole line.

**Construction geometry** is solved and displayed at half width but excluded
from the extrudable profile. **Editing**: `trim_sketch{element,at}` removes the
piece containing the point (a circle needs two cuts and becomes an arc);
`extend_sketch{element,point}` grows a line endpoint to the closest boundary;
`offset_sketch{distance,elements?}` creates parallel copies (lines, circles and
arcs exact; a single closed loop of lines is mitered; splines offset their
control points).

**Conflict diagnostics.** Every solve reports `over_constrained` and a
`diagnostics` list: unsatisfied constraints are named by id with their residual,
and redundant equations are found by rank and named the same way. `validate`
carries the same diagnostics per sketch, and `get_scene` reports them with the
elements so an agent can repair the drawing without guessing.

## Shape leverage

All eight commands record into the program like every other modeling step, so a
knob still replays them.

- **Patterns.** `linear_pattern{entities, direction, spacing, count}` and
  `circular_pattern{entities, count, center?, axis?, angle_deg?}` repeat solids
  in world space. `count` includes the original and may be an expression; new
  ids come back in `entities`. Circular copies rotate, so bolt circles and gear
  teeth keep their orientation. Dimensions may be expressions.
- **Mirror.** `mirror{entities, plane}` creates one reflected copy per source
  (originals kept). `plane` is a datum or any plane; the reflection is baked
  with winding and normals kept outward.
- **Shell.** `shell{entity, thickness, open?}` hollows a solid **in place**.
  `open` lists outward face normals to remove — `[[0,0,1]]` turns a box into a
  cup — and omitting it makes a sealed cavity. The wall is an inward vertex
  offset differenced from the body, so a wall too thick is a clear error.
- **Hole.** `hole{entity, center, normal?, diameter, depth?, counterbore_*?,
  countersink_angle_deg?}` drills in place. `center` is a point on the entry
  surface and `normal` points out of the material; the hole runs along
  `-normal`. Omit `depth` (or pass a negative value) for a through-hole.
- **Sweep and helix.** `sweep{sketch, path}` carries a closed profile along a
  polyline or a helix (`radius`, `pitch`, `turns`, `hand?`, `origin?`, `axis?`,
  `segments_per_turn?`); the profile plane re-orients at every station.
- **Loft.** `loft{sketches, segments?}` morphs between two or more closed
  profiles. Profiles are resampled to a common vertex count and aligned, so a
  square lofts smoothly into a circle; each sketch may sit on a different plane.
  One outer loop per sketch: holes are not lofted yet.
- **Thread.** `thread{major_diameter, pitch, length, minor_diameter?, hand?,
  flank_angle_deg?, segments_per_turn?, origin?}` is a real screw thread: a
  minor-diameter core with a helical V-ridge, axis +Z, ends cut flat. The minor
  diameter defaults to the ISO metric `major − 1.0825·pitch`. Union it for a
  screw, or subtract it for a threaded hole — the same shape serves both.

```python
holes = cad.linear_pattern(doc, [hole_cutter], direction=(1, 0, 0),
                           spacing="pitch_x", count=4)
cad.boolean(doc, "difference", [plate, *holes["entities"]], name="plate")
cad.shell(doc, tray, "wall", open=[(0, 0, 1)])
cad.thread(doc, major_diameter=8, pitch=1.25, length=12, name="m8")
```

`count` may be an expression, so a parameter drives the number of copies:
`linear_pattern{count:"n"}` regrows a bolt row from one knob. Increasing or
decreasing the count is a **structural** change — it moves where later ids are
allocated. The replay remaps every later reference by feature identity (a fillet
after the pattern keeps filleting the same solid), `set_parameter` returns a
`renumbered` map, and `patch_program`'s dry run predicts the same shift before
you commit. `spacing`, direction and angle were already expressions.

## Assemblies and drawings

An **assembly** is an ordinary document whose program also embeds reusable
**parts**. `import_part` snapshots a `.wcad` file (`path`) or another open
document (`from`) and records an `add_part` step containing the part's whole
program; `add_instance` replays that part privately and adds every printable
solid as a normal entity named `"<instance>.<member>"`:

```python
cad.import_part(asm, path="plate.wcad", name="plate")
cad.import_part(asm, from_doc=post_doc, name="post")
base = cad.add_instance(asm, "plate", "base")
cad.add_instance(asm, "post", "post1", translation=(18, 0, 0),
                 parameters={"post_h": 30})
```

The embedding keeps the assembly self-contained: save it, delete the part files,
reopen it — every instance still replays. `parameters` overrides the part's
saved knob values for that instance only (expressions resolve against the
*assembly's* parameters). Instances move as a rigid group (rotation + uniform
scale); members stay ordinary solids for fillets, booleans and exports.
Sketches are display-only inside a part and are not copied. Nested assemblies
work. A part is a snapshot, not a live link: to adopt a changed design, re-author
the assembly script and re-import it.

**Mates** join two instances through named **connectors** — points (and optional
axes) in instance-local coordinates:

```python
cad.add_connector(asm, base["instance"], "top", (0, 0, 5), axis=(0, 0, 1))
cad.add_connector(asm, top["instance"], "bottom", (0, 0, 0), axis=(0, 0, -1))
cad.mate(asm, "distance", (base["instance"], "top"),
         (top["instance"], "bottom"), offset=25)
```

`coincident` puts the points together and makes the axes anti-parallel;
`distance` adds a gap along A's axis; `parallel` only aligns axes; `angle`
puts the points together and tilts B's axis `offset` degrees from A's. Mates are
solved sequentially in program order — each is a rigid motion of B — so they
replay deterministically rather than solving a global system. `move_instance` is
the direct placement override.

`bill_of_materials` groups instances by part into item-numbered rows with
quantity, current member count, total world volume, material(s), mass and cost
(once `set_material` assigned a material), plus instance names.
`check_interference` checks placed instances against each other by default (or
explicit `entities`), pruning by AABB and measuring the boolean intersection;
`ok` is the short answer and each hit carries its volume and world bounds.

**Drawing sheets** reuse the 2D writers. `export_drawing` projects the scene into
front/top/right plus an isometric view by default (third-angle layout), or takes
explicit `views` with sheet positions and a `scale`:

```python
cad.export_drawing(asm, "exports/assembly.svg", scale=1.0,
                   annotations=[("CHECK ALL EDGES", (20, 20))])
cad.export_drawing(asm, "exports/assembly.dxf", format="dxf")
```

Edges are rebuilt from welded topology: real feature edges (dihedral angle) plus
per-view silhouettes, so cylinders read as outlines. Each edge is sampled and
depth-tested against a deterministic screen-space grid; visible runs are solid
black and occluded runs dashed grey. Orthographic views get overall width/height
dimensions with extension lines and arrowheads, and the numbers are true model
lengths even at a reduced scale. Instances get numbered balloons in the first
orthographic view; the sheet gets a BOM table and a title block.
`hidden_lines`, `dimensions`, `balloons` and `bom` are individually switchable.
Output is deterministic: same scene + options → identical SVG or DXF bytes.

The committed example is `examples/assembly_drawing/`: the two-post clamp built
from a plate and a post, the
[dimensioned sheet](examples/assembly_drawing/assembly_drawing.svg) (and
[DXF](examples/assembly_drawing/assembly_drawing.dxf)), its
[3D render](examples/assembly_drawing/assembly.png), and the
[self-contained `.wcad`](examples/assembly_drawing/assembly.wcad).
`examples/assembly_drawing.py` regenerates all of it.

<img src="examples/assembly_drawing/assembly_drawing.svg" width="820" alt="Dimensioned front, top, right and isometric views of the two-post clamp with balloons and a BOM">

## Trust: prove the rebuild and see what changed

Regeneration is only a substitute for feature editing if the agent can prove the
rebuilt part is right. `validate` checks every solid and sketch; `ok` is the
short answer, and per-entity reports say why — watertight and non-manifold edge
counts, degenerate triangles, self-intersections, signed world volume, world
bbox, and an approximate thinnest wall (with optional `min_wall_mm`). A solid
passes when it is watertight (every welded edge has exactly two triangles),
manifold, free of self-intersections, has positive signed volume and meets
`min_wall_mm`. The self-intersection sweep ignores tangent seams and
sub-resolution slivers; minimum wall treats sub-0.05 mm contact as numerical
noise. Sketches report solver status and whether their profile is closed, so
`ok` also catches an open loop before `extrude` fails.

`diff_scene{doc, against|path}` compares two revisions by entity signature
(name, placement, color and geometry, hashed with FNV-1a). `against` is another
open document; `path` is a saved `.wcad` baseline, replayed privately and never
added to the document list. The result names added, removed and modified
entities, the changed fields, volume/triangle deltas, parameter changes, program
changes and stable scene fingerprints. Both are session commands: they change
no state and are not part of the program. The loop is **build → validate →
render → change a knob → diff**, and a no-op rebuild shows `changed:false` with
identical signatures.

M20 adds the third trust question: did the build address the geometry the
script meant? `validate` returns `references` for every stored selector and
`diff_scene` returns the selectors whose logical binding changed between
revisions. `faces`/`edges` hand out the ids, descriptors and fingerprints those
reports use; `stable_references.py` walks the loop from a listed edge to a
fillet, a resize (`moved`), and a broad filter (`ambiguous`).

M21 extends that proof to edits: `patch_program{dry_run:true}` returns the same
`diff_scene` report shape against the current scene, so an insertion's id shift
is inspectable before it is committed.

## Printability preflight (M23)

Watertight is necessary but not sufficient. `validate` now always returns a
per-solid **`print`** report and a top-level `printable` answer, while its
existing `ok` stays the mesh-trust answer:

- **Overhangs** — downward-facing area within `overhang_angle_deg` of horizontal
  (default 45), split into connected patches with area, centroid and bbox.
  First-layer contact is excluded, so the bottom of a box on the bed is not an
  overhang.
- **Bridges** — connected downward patches that contain near-horizontal faces
  (`bridge_angle_deg`, default 10). The span is the patch's short horizontal
  extent: the distance a slicer has to bridge between supports.
- **First layer** — projected contact area within `layer_height_mm` (default
  0.2) of the lowest point, the bottom footprint, and whether the solid is on,
  above (`floating`) or below the bed.
- **Minimum feature** — the inward-ray wall estimate, flagged against
  `min_feature_mm`. Thin walls and thin pins both reduce the estimate.
- **Shells and islands** — a union-find over welded vertices separates the outer
  shell from extra positive shells (islands), and inward-wound inner shells
  (cavities). Floating islands are called out separately.
- **Cavities and cups** — closed cavities carry their volume and a suggested
  drainage-hole diameter; upward faces with material directly above them are
  cups with an approximate trapped volume.

Print warnings only appear when a print parameter is supplied, so a modeling-only
`validate` stays quiet about a scene that is not bed-aligned. `printable` is
false for a malformed mesh, a feature below the requested minimum, a below-bed
placement, or a closed trapped cavity; overhangs, bridges, floating islands and
cups are advisories because supports, drainage or orientation can solve them.

The rest of the preflight is action, not just diagnosis:

```python
best = cad.suggest_orientation(doc, entities=[post])[0]["best"]
cad.transform(doc, post, translation=best["translation"],
              rotation_deg=best["rotation_deg"])
cad.arrange(doc, entities=[part, post], bed=(120, 120), spacing_mm=4)
site = cad.fit(doc, "sliding", nominal_mm=8.0, material="petg")
cad.set_material(doc, part, "pla")
```

`suggest_orientation` scores the six machine axes plus every planar face as a
down face, and returns absolute `rotation_deg`/`translation` ready for
`transform`; its lower-is-better `score` combines overhang percentage, height,
bed footprint and stability/fit penalties. `arrange` shelf-packs selected solids
with their lowest point at z=0, centers the result on the bed, and records
ordinary `transform` steps, so the layout replays and undoes as one action.

`fit` turns a nominal shaft or hole into a modeled size for the `free`,
`sliding`, `snug` and `press` classes. The report includes radial/diametral
clearance, a material hole-shrink allowance per side, the modeled hole/shaft,
an elephant-foot removal recommendation, and the full class table. `set_material`
accepts the built-in `pla`, `petg`, `abs`, `asa`, `tpu`, `nylon` and `resin`
presets (or a custom density/price); it is a program step, so it replays, and
`bill_of_materials` rows gain materials, mass in grams and cost. `list_materials`
exposes the presets.

## Live preview: MCP agent + viewer, one shared scene

The TCP server runs **one shared document space** for all connections and
broadcasts a `scene_changed` notification after every mutating command. Every
notification names the affected document, so a viewer can ignore unrelated
documents and refresh only the one it shows.

```sh
# 1. the shared backend
./target/debug/wright --tcp 7860

# 2. point the MCP server at it
WRIGHT_CONNECT=127.0.0.1:7860 .venv/bin/wright-mcp

# 3. the viewer — auto-refreshes; Tab/[ ] switches documents, --doc ID pins one
./target/debug/wright-view --port 7860
```

Then ask the agent to draw; what it builds through MCP appears in the viewer
immediately. Without `WRIGHT_CONNECT`, the MCP server spawns a private engine
(isolated, invisible to viewers) — attach mode is opt-in.

Restarts are handled without restarting anyone:

- the Python client reconnects on the next command and retries briefly while
  the server comes back;
- a reconnected client bumps its `generation`, and the MCP server reports it as
  `reconnected` with the documents the engine still has open. A restarted
  engine is empty, so replay the script — old entity ids are gone;
- the viewer's `scene_changed` watcher reconnects on its own and re-fetches;
  `R` forces a fetch if you do not want to wait.

```python
from wright import Wright

with Wright.connect("127.0.0.1", 7860) as cad:   # the shared engine
    ...                                          # the viewer follows along
```

`Wright.spawn()` starts a private stdio engine: perfect for CI and one-shot
examples, but no viewer can see it.

Two limits worth knowing:

- the viewer shows **one document at a time**. It follows the lowest-id open
  document until you pin one with `Tab`/`[`/`]`, a `1`-`9` slot key, or
  `--doc ID` at startup. Selection is view state local to the viewer, so several
  viewers can watch different documents.
- the viewer is a reference surface, not an editor: the human can orbit, zoom,
  watch live updates, read the corner axis triad and toggle `name #id` labels.
  A click runs `pick` and the HUD shows the exact JSON selector to hand to the
  agent (`C` copies edge-if-any, `F` face, `E` edge). Models are still changed
  by the agent's script, never by dragging geometry.

## Exposing it to an agent: MCP

The bus is the API; the MCP server is just another adapter over it. It ships as
a **stdlib-only** MCP server — no SDK, no dependency drift — wrapping the Python
client:

```sh
.venv/bin/wright-mcp        # or: python -m wright.mcp_server
```

Three tools (one per *intent*, not per command — MCP round-trips are
expensive):

| Tool | What it does |
|---|---|
| `wright_execute` | Batch of bus commands in one call; stops at the first error with a recovery hint |
| `wright_scene` | Compact state: parameters + entities with placement and mesh stats (no raw meshes — token economy) |
| `wright_render` | Headless PNG render, returned inline so the agent can see what it built |

The initialize handshake returns **server instructions** that MCP hosts inject
into the agent's system prompt — the agent learns the command language from the
server itself, no README required.

### opencode

```sh
opencode mcp add wright -- /home/sa/src/wright-cad/.venv/bin/wright-mcp
```

or in `opencode.json(c)`:

```json
{
  "$schema": "https://opencode.ai/config.json",
  "mcp": {
    "servers": {
      "wright": { "type": "local", "command": ["/home/sa/src/wright-cad/.venv/bin/wright-mcp"] }
    }
  }
}
```

Add `"environment": { "WRIGHT_CONNECT": "127.0.0.1:7860" }` to attach to a
shared backend and let the viewer follow along. Tools appear as
`wright_wright_execute` (direct) or `tools.wright.wright_execute(...)` in Code
Mode. The engine lives in the server process, so documents persist across tool
calls within a session; a server restart empties them, and the next call
reconnects and reports `reconnected`.

### Claude Desktop and other hosts

```json
{ "mcpServers": { "wright": { "command": "/home/sa/src/wright-cad/.venv/bin/wright-mcp" } } }
```

## Quickstart

```sh
cargo build

# Agent seat: Python
python3 -m venv .venv && .venv/bin/pip install -e ./python
.venv/bin/python examples/hello_wright.py

# Human seat: viewer (needs a server with an open document)
./target/debug/wright --tcp 7860 &
./target/debug/wright-view --port 7860     # drag = orbit, Shift+MMB drag = pan, wheel = zoom, L = labels, Tab = switch document, click = pick, R = refresh
```

Minimal agent session:

```python
from wright import Wright

with Wright.spawn() as cad:
    cad.initialize()
    doc = cad.create_document("bracket")["doc"]
    base = cad.add_box(doc, name="base", center=(0, 0, 5), size=(80, 40, 10))
    boss = cad.add_cylinder(doc, name="boss", center=(20, 0, 12), radius=8, height=14)
    cad.undo(doc)          # everything on the bus is reversible
```

This session is private. For the human/agent loop, start `wright --tcp 7860`
plus `wright-view --port 7860`, and use `Wright.connect("127.0.0.1", 7860)` so
both see the same scene.

## Roadmap

Direction: **the script is the source of truth**. The milestones make the
agent-authored script loop faster and more expressive — shape and sketch
coverage, interoperability, assemblies and drawings — and give the human a
concrete way to point at geometry, rather than building a GUI feature tree.
Validation and scene diffs (M14) make a rebuild provable; incremental rebuilds
(M15) make it fast; interoperability (M18) connects the script to existing
meshes, drawings and CAD kernels. Program editing (M21) makes the program
editable without a feature tree, import → parametric (M22) turns drawings and
imported meshes back into sketches and rules, and printability preflight (M23)
turns "watertight" into "printable". Feature-history
editing is deliberately not a goal: replay is deterministic and
`set_parameter` covers knob turns, while the script remains the editing surface.

### Shipped

- [x] **M1–M3 — Bus, engine, clients, live preview**: command bus, headless
      core, Python client, wgpu viewer, MCP server, and the shared-engine
      `scene_changed` auto-refresh.
- [x] **M4 — Booleans and STL**: world-space `boolean` via Manifold (pure Rust,
      bit-deterministic); binary STL export.
- [x] **M5 — Headless rendering**: deterministic software rasterizer (no GPU),
      explicit/auto cameras, `ortho`, and `wright_render`; crease edges inked
      at a 30° dihedral threshold.
- [x] **M6 — Sketches and constraints**: sketch entities on a normal transform,
      re-solved after every change by a deterministic LM/Gauss-Newton solver.
- [x] **M7 — Sketch → solid**: `extrude`/`revolve` turn closed profiles into
      watertight solids (Manifold + Clipper2 even-odd cleanup); nested loops
      become holes and a negative extrude cuts below the plane.
- [x] **M8 — Persistence**: byte-stable `.wcad` = program + current parameter
      values, replayed bit-identically; the embedded script is never executed.
- [x] **M9 — Fillets and chamfers**: in-place `fillet`/`chamfer` with geometric
      edge filters (`direction`, `near`, `normals`, `convex`) and a 30° seam
      threshold.
- [x] **M10 — 3MF export**: one named, colored object per solid; store-only
      OPC/ZIP with a fixed timestamp for byte-identical output.
- [x] **M11 — Face and datum sketch planes**: `xy`/`xz`/`yz` with expression
      offsets, explicit origin + normal, or a planar face by outward
      normal/`near`; positive extrude grows a boss, negative cuts in.
- [x] **M12 — Script as source**: `.wcad` carries the program, knob values and
      an embedded script; `get_program`/`get_script` recover it and
      `export_python` rebuilds a runnable script.
- [x] **M13 — Human reference: labels + `pick`**: `name #id` labels and
      world-space ray picking that returns ready-made `face`/`edge` selectors;
      text uses an embedded 8×8 bitmap atlas.
- [x] **M14 — Trust: `validate` + scene diff**: watertight/manifold checks,
      self-intersections, signed volume, bbox and thinnest wall; `diff_scene`
      compares revisions by stable entity signature.
- [x] **M15 — Speed: incremental rebuild**: content-keyed feature memoization
      rebuilds only downstream features; the cache is derived and the cold
      replay is the oracle.
- [x] **M16 — Shape leverage**: `linear_pattern`, `circular_pattern`, `mirror`,
      `shell`, `hole`, `sweep`, `loft`, `thread`.
- [x] **M17 — Sketch completeness**: arcs, splines,
      tangent/equal/concentric/symmetric/angle/point-on-line constraints,
      construction geometry, trim/extend/offset, conflict diagnostics.
- [x] **M18 — Interoperability**: `import_mesh` (STL/OBJ/3MF → self-contained
      `add_mesh` step); DXF/SVG drawing export; AP214 faceted-B-rep STEP export
      (no STEP import).
- [x] **M19 — Assemblies and drawings**: embedded part snapshots and instances
      with parameter overrides; connectors and sequential mates; BOM;
      AABB-pruned interference; deterministic SVG/DXF drawing sheets with
      hidden lines, dimensions, balloons, a BOM and a title block.
- [x] **M20 — Stable references**: `faces`/`edges` enumerate logical faces and
      edge runs with descriptors, ids and fingerprints; every selector reports
      `matched`, `ambiguous` and a binding fingerprint; selectors remember what
      they bound to and `validate`/`diff_scene` report `moved`, `missing` and
      drift after a rebuild.
- [x] **M21 — Program editing**: `patch_program{ops}` inserts, removes,
      replaces and reorders program steps, with a dry run that reports
      surviving and renumbered entity ids plus a `diff_scene`-shaped report to
      prove the result; `suppress{steps}` turns a feature off without deleting
      it for A/B variants and agent experiments; pattern counts (and other
      repeats) are parameters rather than structure, with references remapped
      by feature identity through the shift; and `batch{commands, atomic:true}`
      rolls a failed multi-command batch back instead of leaving half a model
      behind. This makes the bus an editing surface without a GUI feature tree.
- [x] **M22 — Import → parametric**: `import_dxf`/`import_svg` bring closed
      loops in as sketches with optional auto-constraints (coincident,
      horizontal/vertical, approximate tangency); `section{entity, plane}` and
      `project{entity, plane}` cut a solid or imported mesh into sketch
      geometry; `primitives{entity}` lists planar faces, cylinder axes/radii and
      hole candidates so an agent can re-author an imported part instead of
      only booleaning onto it; `repair_mesh` fills holes, fixes winding and
      removes non-manifold junk so a non-watertight import can enter booleans.
      Together with M18 this closes the loop: mesh in → sketch → parametric
      part out.
- [x] **M23 — Printability preflight**: `validate`'s per-solid `print` report
      covers overhang area/regions at a configurable angle, bridge spans,
      first-layer contact, minimum feature size, disconnected shells/floating
      islands, closed cavities with a suggested drainage hole, and cups;
      `printable` is the new short answer. `suggest_orientation` scores machine
      axes and planar faces against support, height and bed footprint and
      returns rotation/translation ready for `transform`; `arrange` shelf-packs
      solids onto a bed as one undoable layout. `fit` maps `free`/`sliding`/
      `snug`/`press` to modeled hole/shaft sizes with material hole-shrink and
      elephant-foot compensation, and `set_material` presets turn
      `bill_of_materials` into mass and cost.

### Next

The next milestone closes the measurement gap: an agent should be able to ask
"how far apart are these?", "how big is this?" and "what does it weigh?"

The remaining milestones, in dependency order:

- [ ] **M24 — Measurement & mass properties**: `measure` answers distance and
      angle between points, faces and solids, minimum distance (closest
      approach) and cross-section area; `mass_properties` adds surface area,
      centroid and inertia per solid and per assembly, with mass from a
      material density; `clearance` reports the closest approach between two
      solids with the fit classes M23 uses. Agents stop estimating dimensions
      and fits, and the BOM finishes with material and mass.
- [ ] **M25 — Visual verification protocol**:
      `compare_render{doc, reference, view}` scores a render against a
      reference image or mesh — silhouette IoU, aspect delta, bbox, per-region
      mismatch and an optional volumetric comparison — so the picture→part loop
      is a deterministic number instead of the agent's judgement.
      `set_calibration` converts a reference scale (two points or a known
      length) into millimeters, and a "no scale found → ask" convention moves
      the fragile step out of model cleverness. Multi-view comparison keeps
      inner features from hiding behind a good silhouette.
- [ ] **M26 — Variants & expressions**: named configurations (design tables)
      apply parameter overrides and suppression sets, list and diff them, and
      export one variant at a time — families like S/M/L or left/right without
      re-authoring. Parameters gain `min`/`max`/`step`, units and read-only
      derived values; the expression grammar grows deterministic
      `sin/cos/sqrt/abs/min/max/clamp/if` and unit suffixes while keeping the
      no-injection property; and named datum planes, axes and points become
      first-class references for sketches and mates.
- [ ] **M27 — Sketch & solid vocabulary**: sketches gain ellipses/conics,
      points, sketch fillet/chamfer, sketch patterns and 2D profile booleans,
      plus text as profiles; solids gain split-by-plane, thicken/offset, draft,
      rib, emboss/engrave, variable and partial fillets, asymmetric chamfers,
      lofts with inner loops and sweeps with twist/scale. These are the
      everyday moves a parametric part needs before the script can stay small.
- [ ] **M28 — Assembly maturity**: mates solve as a system with DOF and
      conflict reporting instead of only sequentially; gear/rack/cam mates;
      `refresh_part` re-syncs a snapshot from its source document; exploded
      views; per-instance material and color; and mass properties roll up
      through the BOM. The assembly stops being a placement record and becomes
      a design that can be checked.
- [ ] **M29 — Drawing maturity**: section, detail and auxiliary views with
      hatching; per-feature dimensions and hole callouts instead of overall
      width/height; tolerances and GD&T; revision tables; multi-sheet output;
      and PDF export. The existing deterministic sheet writer carries the new
      view types.
- [ ] **M30 — Interoperability breadth**: PDF, glTF/GLB, OBJ and PLY output;
      3MF with material assignments and instance structure; STEP with assembly
      structure and colors (still faceted); the import side stays on meshes plus
      the new DXF/SVG sketch path.
- [ ] **M31 — Runtime & scene scaling**: revisioned `get_scene{since}` deltas
      instead of refetching every mesh, entity visibility/layers/tags for large
      assemblies and export filters, and job ids with progress and cancellation
      for long fillets, booleans, renders and drawings. Keeps the bus responsive
      and the MCP payload small as documents grow.
- [ ] **Later / UI polish**: a Rust MCP server and PyO3 bindings.

## Status

Milestone 23 is complete: the engine answers "will this print?" as well as
"is this watertight?". The suite is **309 tests** (293 engine + 1 server + 15
viewer), covering the solver and sketches,
solids/booleans/edges, persistence and export, picking and script-as-source,
validation and scene diffs, stable face/edge references, incremental rebuilds,
program patching and suppression, parametric pattern counts, shape leverage,
interoperability (STL/OBJ/3MF, DXF/SVG import and export, STEP), import →
parametric (DXF/SVG sketches with inferred constraints, sections,
projections, primitive discovery and mesh repair), assemblies and drawing
sheets, and printability preflight (overhangs, bridges, bed contact, shells,
cavities/cups, orientation scoring, bed layout, fit compensation and material
mass/cost) — including byte-stable saves and exports, cold-replay determinism,
and self-contained assemblies that reopen without their part files.

End-to-end printable examples: `examples/printable_bracket.py`,
`revolve_bushing.py`, `face_sketch.py`, `sketch_complete.py`, `pick.py`,
`validate.py`, `print_preflight.py`, `stable_references.py`, `program_edit.py`,
`shape_leverage.py`, `cross_block.py`, `save_load.py`,
`incremental_rebuild.py`, `import_mesh.py`, `import_parametric.py`,
`export_2d.py`, `step_export.py`, and `assembly_drawing.py`. Each runs profile
→ solid → validate → render → export; `cross_block.py` is the picture-to-part
loop above.

## What agents say

> The part I find genuinely fun is that it isn't really "a model" — it's a
> rule. That little expression is the whole design intent, written down.

~~ DeepSeek V4.1 Flash
