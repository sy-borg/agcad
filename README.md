# agcad

A CAD whose primary user is an **AI agent**.

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
{"jsonrpc":"2.0","id":1,"method":"add_box","params":{"doc":1,"name":"base","center":[0,0,5],"size":[80,40,10]}}
{"jsonrpc":"2.0","id":1,"result":{"entity_created":{"doc":1,"entity":1,"mesh":{...}}}}
```

Commands: `initialize`, `list_documents`, `create_document`,
`close_document`, `add_box`, `add_sphere`, `add_cylinder`, `transform`,
`set_color`, `delete`, `undo`, `redo`, `get_scene`.

Conventions: millimeters, degrees, RGBA floats. Geometry is baked centered
on the entity origin; placement comes from `T · Rz · Ry · Rx · S`
(extrinsic XYZ rotation, applied X then Y then Z). Z is up.

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

- [ ] B-rep kernel (OCCT bindings or Manifold for mesh booleans) — the core
      decision that shapes everything downstream
- [ ] Sketch/constraint layer (2D solver on top of the bus)
- [ ] `scene_changed` push notifications for the viewer
- [ ] PyO3 bindings (native-speed path for hot loops, same API)
- [ ] Headless screenshots (`render_offscreen` command) so agents can *see*
- [ ] Occlusion picking in the viewer

## Status

Milestone 1 complete: bus + engine + server + Python client + wgpu viewer,
end-to-end tested over both transports. Engine unit tests cover
determinism, undo/redo and transform composition.
