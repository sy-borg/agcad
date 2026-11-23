# AGENTS.md

wright is a headless parametric CAD for 3D printing. The Rust engine and its
JSON-RPC command bus are the product; the Python client, MCP server and wgpu
viewer are clients. `README.md` is the authoritative user/protocol doc — read
it first; this file only records what is easy to get wrong.

Keep the `README.md` and `index.html` in synch with the code.

## Commands

- `cargo build` before running any Python example or live demo. They spawn
  `target/debug/wright`, and `cargo test` does **not** refresh that binary
  (verified by mtime) — `cargo test` only builds test artifacts.
- `cargo test` — full suite. One test: `cargo test <name>`. Engine tests are in
  `crates/core/src/lib.rs`; module tests live beside their code (e.g.
  `crates/core/src/fillet.rs`).
- Python: the venv is `.venv`
  (`python3 -m venv .venv && .venv/bin/pip install -e ./python`). Run examples
  with `.venv/bin/python examples/<name>.py`. Server lookup order is
  `$WRIGHT_SERVER`, then `wright` in `PATH`, then `target/{debug,release}/wright`.
- MCP: `.venv/bin/wright-mcp` (or `python -m wright.mcp_server`). With
  `WRIGHT_CONNECT=host:port` it attaches to a shared backend for live preview;
  otherwise it spawns a private engine. A running MCP process keeps its old
  Python code and binary — restart it after changes. Restarting the backend
  wipes its in-memory documents.
- Showcase: `.venv/bin/python examples/cross_block.py` (ImageMagick is needed
  for `compare.png`).
- Do not run `cargo fmt` repo-wide: the tree is not rustfmt-clean and it would
  reformat unrelated code. Clippy has pre-existing warnings. There is no CI,
  so neither is a gate.
- Keep `python/wright` stdlib-only (`pyproject.toml` has no dependencies).

## Layout and conventions

- `crates/protocol` wire types; `crates/core` documents, history, geometry,
  constraints, booleans, fillets, software rasterizer; `crates/server` JSON-RPC
  over stdio/TCP; `crates/viewer` wgpu client. The bus is the only API; the
  viewer and MCP own no document state. Viewer-side labels are display only;
  hit-testing and selector construction live in the engine's `pick` command,
  never in the viewer.
- Units are mm and degrees, Z is up, colors are `[r,g,b,a]` floats 0..1, and
  cylinders extrude along +Z. Document ids start at 1; `create_document`
  accepts an explicit unused id so create + populate can share one batch.
- `.wcad` stores the program plus current parameter values, never geometry
  (`open_document` replays it). An embedded Python source rides along as the
  canonical recipe but is never executed. Saves must be byte-stable
  (save → open → save is a no-op).
- `examples/cross_block.py` embeds its own source into the committed
  `examples/cross_block/*.wcad`. Editing the script therefore changes those
  files' recipe text; regenerate with the example — program and parameters
  should stay identical.
- `exports/` is gitignored. The artifacts under `examples/cross_block/` are
  committed and must stay byte-reproducible; `compare.png` needs
  `-strip -define png:exclude-chunks=date,time` (the script already does it).
- Rendering is a deterministic software rasterizer in core (no GPU); PNG bytes
  are part of the reproducibility promise.

## Source of truth

The client script is the durable, editable artifact; `.wcad` is a replayable
snapshot (program + current parameter values + embedded script). Agents edit the
script, run it, and save the instance with `source=...`. `get_program` returns
the embedded source and the Python client's `get_script` hands it back, so a
handoff can recover the recipe without executing it. MCP-only documents with no
script are scratch state.

Do not build a GUI-style feature tree or in-place feature editing: `set_parameter`
handles knob turns and the agent regenerates structure from the script. The
program is the intermediate representation, not the editing surface. If
script-less MCP documents become a first-class mode, add a small program-patch
API instead of a history UI.

## Determinism is a hard requirement

Same program + parameters must produce bit-identical ids, meshes and `.wcad`
bytes; tests and agents rely on exact replay.

- Use `BTreeMap`/`BTreeSet`, never `HashMap`, for anything that affects order.
- No randomness, clocks, hash iteration or locale-dependent formatting in
  geometry or serialization paths.
- Meshes are `f32`; the manifold kernel is `f64`. `boolean::from_manifold`
  casts to f32 and duplicates one vertex per triangle corner for crease
  normals; `to_manifold` welds by exact f32 bits and sets the import tolerance
  `max(5e-7 * bbox_diagonal, 3e-4)`, which the kernel uses as its epsilon. That
  tolerance is what keeps repeated booleans manifold on filleted/tessellated
  geometry — re-test rim fillets before changing it.
- Topology consumers (`edges::sharp_edges`) weld by exact f32 position bits.
  `world_bbox` is the tight world-space mesh bbox, not the rotated local bbox.
- The incremental-rebuild cache is derived state: feature meshes are keyed by
  content (exact bits folded through upstream geometry keys), never saved and
  never part of undo snapshots. Cached output must stay byte-identical to a
  cold replay; clearing the cache may only cost speed.
- `validate` and `diff_scene` are read-only session commands (never program
  entries). Entity signatures are FNV-1a over name/placement/color/geometry
  bits; scene signatures include ids, so diffs match entities by id. The
  self-intersection sweep and wall sampling are deterministic, and they
  intentionally ignore tangent seams, sub-resolution slivers and sub-0.05 mm
  contact — keep those tolerances when touching them or a filleted boolean
  starts reporting phantom defects.

## Testing and artifacts

- When fixing a bug, add a regression test and check that it fails without the
  fix.
- Fillet tests use `assert_closed` (every welded edge must have exactly two
  triangles) and `signed_volume`. Results may legitimately contain a few
  zero-area triangles on tangent seams; the engine tolerates them, and dropping
  them naively opens the mesh — leave them alone.
- After regenerating the showcase, `git status` should show only the files you
  meant to change; baseline renders and STL must not move unless the model did.
