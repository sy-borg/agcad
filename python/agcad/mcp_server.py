"""Minimal Model Context Protocol (MCP) server for agcad — stdlib only.

Speaks newline-delimited JSON-RPC 2.0 on stdio (the MCP stdio transport)
and forwards tool calls to the agcad command bus. Deliberately depends on
nothing but the `agcad` client package: the protocol surface we need
(initialize, tools/list, tools/call, ping) is small, and zero deps beats
SDK version drift.

Design rules:
- One tool per *intent*, not per command: `agcad_execute` runs a batch of
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

from .client import Agcad, AgcadError

SERVER_INFO = {"name": "agcad", "version": "0.2.0"}
DEFAULT_PROTOCOL_VERSION = "2025-06-18"
MAX_BATCH = 500

INSTRUCTIONS = """agcad is a headless parametric CAD for 3D printing.
A document holds a program: every modeling command is recorded, dimensions may be
expressions over named parameters ("wall*2"). set_parameter replays the program
deterministically (entity ids stay stable). undo/redo revert one command each and
restore parameters + program + geometry atomically.
Units: millimeters and degrees. Z is up. Colors are [r,g,b,a] floats 0..1.
Cylinders extrude along +Z. Document ids start at 1; create_document takes an
optional explicit `doc` id, so create-then-populate fits in one batch.
Workflow: create_document -> set_parameter(...) -> add primitives with expression
dimensions -> transform -> boolean ops for printable solids -> agcad_scene to
verify -> agcad_render to SEE the result -> export_stl for slicing. Sketches are
2D constraint-solved drawings placed in 3D by the entity transform: add_sketch ->
add_sketch_line/add_sketch_circle -> constrain(...) re-solves after every change ->
extrude/revolve turns a closed profile into a printable solid. Fillets and
chamfers reshape a solid in place (same entity id): fillet{doc,entity,radius,edges?}
or chamfer{doc,entity,distance,edges?}, where `edges` is a geometric filter
({direction}, {near,near_radius}, {normals}, {convex}) instead of an index — omit
it to take every sharp edge. Persist a model with
save_document -> .agcad (JSON program + current parameter values, replayed on
open_document); a Python client script is the authoring source, not the file.
Failed commands report a hint; fix the named issue and retry."""

EXECUTE_DESCRIPTION = """Batch-execute agcad commands. Each command is an object:
{"method": "<name>", ...params} or {"method": "<name>", "params": {...}}.
Commands: initialize{}, list_documents{}, create_document{name,doc?}
  (doc is an unused id >= 1; omit it to allocate the next one — pass it to
  create+populate in a single batch), close_document{doc},
set_parameter{doc,name,value(number|string expression),description?},
remove_parameter{doc,name}, get_parameters{doc},
add_box{doc,name,center:[x,y,z],size:[w,d,h],color?},
add_sphere{doc,name,center,radius,segments?,color?},
add_cylinder{doc,name,center,radius,height,segments?,color?} (extruded along
  +Z, spanning center.z ± height/2 — no rotation needed to stand it up),
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
add_sketch{doc,name} (empty 2D sketch; place it in 3D with transform),
add_sketch_line{doc,sketch,p1:[x,y],p2:[x,y]},
add_sketch_circle{doc,sketch,center:[x,y],radius},
constrain{doc,sketch,kind,refs,value?} (re-solves; kind is coincident,
  horizontal, vertical, parallel, perpendicular, distance, radius or diameter;
  refs are {"element":id,"point":0|1} for points or {"element":id} for whole
  elements — coincident/distance take 2 point refs, horizontal/vertical 1 line,
  parallel/perpendicular 2 lines, radius/diameter 1 circle + value;
  result reports solved/residual/dof),
remove_constraint{doc,sketch,constraint},
move_sketch_point{doc,sketch,element,point,to:[x,y]} (0 = line start / circle
  center, 1 = line end; steers the solver, then re-solves),
delete_sketch_element{doc,sketch,element} (constraints on it cascade away),
extrude{doc,sketch,distance,name?,color?} (closed profile → watertight solid along
  the sketch normal; negative distance extrudes the other way; nested loops become
  holes; the solid follows later sketch edits; returns volume + tris),
revolve{doc,sketch,angle_deg?,axis?:"x"|"y",segments?,name?,color?} (profile →
  solid of revolution around an in-plane axis, default 360°; profile must stay on
  one side of the axis),
save_document{doc,path,source?} (writes a .agcad JSON file: the program plus
  the current parameter values; geometry is rebuilt by replay, so the file stays
  small and diffable; source={language,text} is provenance only, never executed;
  save -> open -> save is a no-op diff; session command, changes no state),
open_document{path,name?} (loads a .agcad file as a NEW document and replays it;
  returns doc, name, counts; the stored name is used unless name overrides),
get_program{doc} (read-only program + parameters — what a save would write),
export_stl{doc,path,entities?} (binary STL, world space, absolute path — a
  slicer-ready file; not part of the program, changes no state; sketches are
  display-only and skipped),
render_offscreen{doc,width?,height?,eye?,target?,up?,fov_deg?,ortho?,background?,path?}
  (headless PNG; prefer the agcad_render tool, which returns the image inline.
  ortho:true gives a parallel projection for front/top/side views).
Numeric arguments accept expression strings over parameters. Commands run in order;
execution stops at the first error (reported with a recovery hint)."""

SCENE_DESCRIPTION = """Inspect an agcad document: parameters and entities with
placement, a world-space bounding box per entity (meshes are entity-local;
world_bbox already includes translation/rotation/scale), and vertex/triangle
counts — no raw meshes."""

RENDER_DESCRIPTION = """Render the document to a PNG image, returned inline as an
image content block — so you can SEE the scene and compare it against what the
user asked for, then iterate. Args: doc, width? (default 800), height? (600),
eye?/target?/up? as [x,y,z] camera vectors (omit eye for an auto-framed 3/4
view of the whole scene), fov_deg? (default 40), path? to also save the PNG.
Headless software render, deterministic; changes no state."""

TOOLS = [
    {
        "name": "agcad_execute",
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
        "name": "agcad_scene",
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
        "name": "agcad_render",
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
    payloads are replaced with a pointer to `agcad_render` (images belong
    in the vision loop, not in a text dump)."""
    if isinstance(obj, dict):
        out = {}
        for k, v in obj.items():
            if k == "mesh" and isinstance(v, dict):
                out[k] = mesh_stats(v)
            elif k == "png_base64" and isinstance(v, str):
                out[k] = f"<{len(v)} base64 chars — call agcad_render to get this as an image>"
            else:
                out[k] = summarize(v)
        return out
    if isinstance(obj, list):
        return [summarize(v) for v in obj]
    return obj


# --- the server ---------------------------------------------------------------


class McpServer:
    def __init__(self) -> None:
        self._cad: Agcad | None = None

    @property
    def cad(self) -> Agcad:
        # Spawn the engine lazily so `tools/list` works even without a binary.
        if self._cad is None:
            attach = os.environ.get("AGCAD_CONNECT")
            if attach:
                # Live mode: draw into a shared backend (e.g. the one a
                # viewer is watching) instead of a private engine.
                host, sep, port = attach.rpartition(":")
                if not sep:
                    host, port = "127.0.0.1", attach
                self._cad = Agcad.connect(host, int(port))
                print(f"agcad-mcp: attached to shared backend {host}:{port} (live preview)", file=sys.stderr)
            else:
                self._cad = Agcad.spawn()
        return self._cad

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
        if name == "agcad_execute":
            return self._tool_execute(args)
        if name == "agcad_scene":
            return self._tool_scene(args)
        if name == "agcad_render":
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
            except AgcadError as e:
                results.append({"step": i, "method": method, "error": str(e)})
                error = str(e)
                break
            except Exception as e:  # noqa: BLE001
                results.append({"step": i, "method": method, "error": f"{type(e).__name__}: {e}"})
                error = "unexpected failure, see last step"
                break

        payload: dict[str, Any] = {"executed": len(results), "results": results}
        if error is not None:
            payload["stopped_early"] = True
            payload["remaining"] = len(commands) - len(results)
            payload["hint"] = "fix the error in the last result, then re-send the remaining commands"
        return {"content": [{"type": "text", "text": json.dumps(payload, separators=(",", ":"))}]}

    def _tool_scene(self, args: dict) -> dict:
        try:
            doc = int(args["doc"])
        except (KeyError, TypeError, ValueError):
            return {"content": [{"type": "text", "text": "integer 'doc' is required"}], "isError": True}
        try:
            scene = self.cad.call("get_scene", {"doc": doc})
            params = self.cad.call("get_parameters", {"doc": doc})
        except AgcadError as e:
            return {"content": [{"type": "text", "text": str(e)}], "isError": True}

        entities = []
        for e in scene["entities"]:
            stats = mesh_stats(e["mesh"])
            entities.append(
                {
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
            )
        payload = {"doc": doc, "parameters": params["params"], "entities": entities}
        return {"content": [{"type": "text", "text": json.dumps(payload, separators=(",", ":"))}]}

    def _tool_render(self, args: dict) -> dict:
        params = {k: v for k, v in args.items() if v is not None}
        try:
            r = self.cad.call("render_offscreen", params)
        except AgcadError as e:
            return {"content": [{"type": "text", "text": str(e)}], "isError": True}
        meta = {"doc": r["doc"], "width": r["width"], "height": r["height"], "bytes": len(r["png_base64"])}
        if "path" in r:
            meta["saved_to"] = r["path"]
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
