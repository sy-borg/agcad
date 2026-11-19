# agcad

<p align="center">
  <img src="logo.svg" width="160" alt="agcad logo — a snowman assembled from primitives"/>
</p>

A CAD whose primary user is an **AI agent**.

> The logo is the "hello world" of this CAD: three spheres, a cylinder and
> a couple of boxes, assembled by a parametric program — drawn as an SVG
> from the same primitives the engine tessellates. See
> `examples/draw_snowman_family.py` for the living version.

The architecture inverts the usual GUI-first design: the headless core and
its command bus are the product; the human-facing viewer is just another
client — as is the Python client an agent drives.

```
┌────────────┐   JSON-RPC (stdio/TCP)   ┌──────────────┐
│  agent /   │ ───────────────────────► │   agcad      │ ┌───────────────┐
│  python /  │ ◄─────────────────────── │  server      │ │   agcad-core  │
│  any lang  │                          │ (transport)  │ │ documents     │
└────────────┘                          └──────────────┘ │ entities      │
                                                             │ history/undo  │
┌────────────┐          same bus (TCP)                       │ tessellation  │
│ agcad-view │ ─────────────────────────────► get_scene ───► └───────────────┘
│ (wgpu, dumb)│  pulls snapshots; owns no document state
└────────────┘
```

## Layout

| Path | What it is |
|---|---|
| `crates/protocol` | Wire types: commands, results, meshes (serde/JSON) |
| `crates/core` | Headless engine: document model, deterministic primitives, undo |
| `crates/server` | `agcad` binary — line-delimited JSON-RPC on stdio or TCP |
| `crates/viewer` | `agcad-view` — wgpu renderer, talks only the bus |
| `python/agcad` | Dependency-free sync Python client (the agent API) |
| `examples/` | Agent-facing demos |

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

Commands: `initialize`, `list_documents`, `create_document`,
`close_document`, `add_box`, `add_sphere`, `add_cylinder`, `transform`,
`set_color`, `delete`, `set_parameter`, `remove_parameter`,
`get_parameters`, `undo`, `redo`, `get_scene`.

Conventions: millimeters, degrees, RGBA floats. Geometry is baked centered
on the entity origin; placement comes from `T · Rz · Ry · Rx · S`
(extrinsic XYZ rotation, applied X then Y then Z). Z is up.

## The parametric layer: the program *is* the function

The user describes intent; **the agent authors a parametric program**:

1. Named parameters (`wall`, `plate_len`) — values may themselves be
   expressions over other parameters.
2. Modeling commands with dimensions as expressions: `size = [80, 40,
   "wall*2"]`.
3. `set_parameter` replays the whole program: **entity ids are preserved**,
   geometry regenerates, and undo restores parameters + program + geometry
   atomically.

Expression grammar is deliberately tiny and safe (`+ - * /`, parentheses,
identifiers, unary minus) — no functions, no injection surface. Errors are
agent-shaped: a typo lists the valid parameter names.

```python
cad.set_parameter(doc, "wall", 2.0, description="plate thickness [mm]")
base = cad.add_box(doc, "base", center=(0, 0, "wall/2"), size=("plate", 40, "wall*2"))
cad.set_parameter(doc, "wall", 4.0)   # rebuilds everything, ids unchanged
```

This is the substrate for the target workflow: a vision model looks at a
picture, the agent writes the parametric program, renders it headless,
compares, and iterates — the user never touches a function.

## Live preview: MCP agent + viewer, one shared scene

The TCP server runs **one shared document space** for all connections, and
broadcasts a `scene_changed` notification after every mutating command.
The viewer listens and re-fetches by itself — no refresh key needed.

```sh
# 1. the shared backend
./target/debug/agcad --tcp 7860

# 2. point the MCP server at it (opencode config, `environment` field)
{
  "mcp": {
    "servers": {
      "agcad": {
        "type": "local",
        "command": ["/home/sa/src/agcad/.venv/bin/agcad-mcp"],
        "environment": { "AGCAD_CONNECT": "127.0.0.1:7860" }
      }
    }
  }
}

# 3. the viewer — it auto-refreshes when the agent draws
./target/debug/agcad-view --port 7860
```

Then ask the agent to draw; what it builds through MCP appears in the
viewer immediately. Without `AGCAD_CONNECT`, the MCP server spawns a
private engine (isolated, invisible to viewers) — attach mode is opt-in.

## Exposing it to an agent: MCP

The bus is the API; the MCP server is just another adapter over it
(like the viewer). Ships as a **stdlib-only** MCP server — no SDK, no
dependency drift — wrapping the Python client:

```sh
.venv/bin/agcad-mcp        # or: python -m agcad.mcp_server
```

Two tools (one per *intent*, not per command — MCP round-trips are expensive):

| Tool | What it does |
|---|---|
| `agcad_execute` | Batch of bus commands in one call; stops at the first error with a recovery hint |
| `agcad_scene` | Compact state: parameters + entities with placement and mesh stats (no raw meshes — token economy) |

The initialize handshake returns **server instructions** that MCP hosts
inject into the agent's system prompt — the agent learns the command
language from the server itself, no README required.

### opencode

```sh
opencode mcp add agcad -- /home/sa/src/agcad/.venv/bin/agcad-mcp
# or in opencode.json(c):
{
  "$schema": "https://opencode.ai/config.json",
  "mcp": {
    "servers": {
      "agcad": { "type": "local", "command": ["/home/sa/src/agcad/.venv/bin/agcad-mcp"] }
    }
  }
}
```

Tools then appear as `agcad_agcad_execute` (direct) or
`tools.agcad.agcad_execute(...)` in Code Mode. The engine lives in the
server process, so documents persist across tool calls within a session.

### Claude Desktop and other hosts

```json
{ "mcpServers": { "agcad": { "command": "/home/sa/src/agcad/.venv/bin/agcad-mcp" } } }
```

## Quickstart

```sh
cargo build

# Agent seat: Python
python3 -m venv .venv && .venv/bin/pip install -e ./python
.venv/bin/python examples/hello_agcad.py

# Human seat: viewer (needs a server with an open document)
./target/debug/agcad --tcp 7860 &
./target/debug/agcad-view --port 7860     # drag = orbit, wheel = zoom, R = refresh
```

Minimal agent session:

```python
from agcad import Agcad

with Agcad.spawn() as cad:
    cad.initialize()
    doc = cad.create_document("bracket")["doc"]
    base = cad.add_box(doc, name="base", center=(0, 0, 5), size=(80, 40, 10))
    boss = cad.add_cylinder(doc, name="boss", center=(20, 0, 12), radius=8, height=14)
    cad.undo(doc)          # everything on the bus is reversible
```

## Roadmap

- [x] Command bus + headless engine + Python client + wgpu viewer (M1)
- [x] Parametric layer: parameters, expressions, program replay (M2)
- [x] MCP server + shared-engine live preview: `scene_changed` broadcast,
      viewer auto-refresh, MCP attach mode (M3)
- [x] **Mesh booleans via Manifold** (`manifold-rust`, pure Rust, bit-deterministic):
      `boolean{op: union|difference|intersection}` in world space, result is a
      new entity that replays parametrically; plus `export_stl` (binary, world
      space, slicer-ready). 3MF when a project asks for it
- [x] **Headless `render_offscreen` (PNG) + camera control**: deterministic
      software rasterizer in core (no GPU needed — works over SSH/CI),
      auto-framed 3/4 camera by default, explicit eye/target/up/fov when
      wanted, and `ortho` for parallel front/top/side views; MCP tool
      `agcad_render` returns the image inline so the agent can see what it
      built and iterate
- [x] **Sketch/constraint layer**: sketches are entities (`add_sketch`,
      `add_sketch_line`, `add_sketch_circle`), placed in 3D by the normal
      transform; `constrain{kind: coincident|horizontal|vertical|parallel|
      perpendicular|distance|radius|diameter}` re-solves after every change
      with a deterministic LM/Gauss-Newton solver. Coordinates and values may
      be expressions, so sketches re-solve on parameter change. Snapshot
      reports `solved/residual/dof`; sketches render as ribbons and are
      skipped by STL export
- [x] **Sketch → solid**: `extrude{distance}` and `revolve{angle_deg, axis}`
      turn closed profiles into watertight printable solids (Manifold's
      extrusion/revolution constructors + Clipper2 even-odd profile cleanup,
      so nested loops become holes). Solids inherit the sketch placement,
      follow later sketch edits, replay parametrically, and report `volume`
      + `tris`; negative extrude goes below the sketch plane
- [x] **Document persistence**: `save_document` writes a `.agcad` file — the
      program plus the current parameter values, no geometry — and
      `open_document` replays it into a new document with bit-identical ids
      and meshes. Saves are byte-stable (save → open → save is a no-op diff),
      and the original script can ride along as provenance (never executed).
      `get_program` exposes the same data to any client; the Python client's
      `export_python` renders it back into a runnable script. The `.py` is
      the recipe; the `.agcad` is the instance — recipe plus the knobs as the
      user left them
- [ ] **Fillets and chamfers**: `fillet{edges, radius}` / `chamfer{edges,
      distance}` over an edge-selection query (edges named by the two adjacent
      faces or by a bounding-box filter, so an agent can target them without
      clicking). Rounded edges are the difference between a CAD model and a
      printable-looking part
- [ ] **3MF export**: multi-object builds, per-object color/material, unit and
      slicer metadata — STL loses all of it
- [ ] **Sketches on a face / datum planes**: place a sketch on an existing
      planar face (or an explicit plane) instead of always on XY, with the
      normal derived from the face so `extrude` cuts or adds material in the
      right direction
- [ ] **Shaded-with-edges rendering**: a thin dark outline where adjacent
      faces meet (and where a face silhouette crosses). Flat shading alone
      hides coplanar-depth steps — a solid's front view reads as a blank
      rectangle, and construction sketches peek out at the base. CAD drawings
      draw edges for exactly this reason
- [ ] Rust MCP server (single self-contained binary; the Python one is
      already adapter-clean)
- [ ] PyO3 bindings (native-speed path for hot loops, same API)
- [ ] Occlusion picking in the viewer

## Status

Milestone 7 complete: sketches → solids. 65 tests cover the solver
(rectangles from messy input, radius/diameter, parallel/perpendicular,
over-constraint reporting, bit-identical determinism), the engine path
(constraints through the bus, parametric re-solve, undo/redo, cascade
delete, STL sketch filtering), and solid construction (extrude volumes,
nested holes, negative extrusion, revolve around X/Y, partial sweeps,
profile validation, dependent rebuilds, cross-engine determinism) plus
the orthographic camera (parallel projection, depth-independent size), and
document persistence (save/open round-trips, byte-stable saves, doc-id
rebinding, corrupt/newer-file hints, parameters keeping their descriptions).

End-to-end printable examples: `examples/printable_bracket.py` (constrained
profile with holes → extrude → STL → render),
`examples/revolve_bushing.py`, `examples/cross_block.py` — a picture →
model reproduction: `examples/example_body.png` is read for its two views
(45 x 45 mm cross, 15 mm arms, 50 mm tall), rebuilt as a parametric plus
profile and extruded, then rendered from a camera fitted to the drawing —
and `examples/save_load.py` (save → open in a fresh engine → change a knob →
export back to Python and run it).

Also in the roadmap but not started: fillets/chamfers with edge selection,
3MF export, and sketches placed on existing faces or datum planes.
