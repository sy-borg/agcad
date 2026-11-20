"""agcad — Python client for the agcad CAD command bus.

The primary interface for AI agents. Plain, dependency-free, synchronous:

>>> from agcad import Agcad
>>> cad = Agcad.spawn()
>>> _ = cad.initialize()
>>> doc = cad.create_document("bracket")["doc"]
>>> box = cad.add_box(doc, name="base", center=(0, 0, 5), size=(10, 10, 10))
>>> box["entity"]
1

The server binary is found via ``$AGCAD_SERVER``, ``$PATH`` (``agcad``), or
the local cargo build (``target/debug/agcad`` / ``target/release/agcad``).
"""
from __future__ import annotations

import itertools
import json
import os
import shutil
import subprocess
import socket
import threading
from pathlib import Path
from typing import Any, Iterable

__all__ = ["Agcad", "AgcadError", "AgcadStdioTransport", "AgcadTcpTransport"]
__version__ = "0.1.0"


class AgcadError(RuntimeError):
    """The server rejected a command. ``message`` is agent-readable and
    usually contains a hint on how to recover."""


# ---------------------------------------------------------------------------
# transports: send(str) + recv(id) — one in-flight request per client, which
# is exactly what an agent loop wants. JSON-RPC allows pipelining later.
# ---------------------------------------------------------------------------


class _LineTransport:
    def __init__(self) -> None:
        self._cond = threading.Condition()
        self._responses: dict[int, dict] = {}
        self._closed = False

    def _start_reader(self, readline) -> None:
        def read_loop() -> None:
            try:
                while True:
                    line = readline()
                    if not line:
                        break  # EOF: server went away
                    try:
                        response = json.loads(line)
                    except json.JSONDecodeError:
                        continue
                    rid = response.get("id")
                    if isinstance(rid, int):
                        with self._cond:
                            self._responses[rid] = response
                            self._cond.notify_all()
            finally:
                # Signal waiters on EOF or read errors, so pending recv()
                # calls fail fast instead of running out their timeout.
                self.mark_closed()
        with self._cond:
            self._closed = False
        threading.Thread(target=read_loop, daemon=True).start()

    def send(self, line: str) -> None:
        raise NotImplementedError

    def recv(self, rid: int, timeout: float = 30.0) -> dict:
        with self._cond:
            while rid not in self._responses:
                if self._closed:
                    raise AgcadError("server connection closed")
                if not self._cond.wait(timeout):
                    raise AgcadError(f"timed out waiting for response to request {rid}")
            return self._responses.pop(rid)

    def mark_closed(self) -> None:
        with self._cond:
            self._closed = True
            self._cond.notify_all()


class AgcadStdioTransport(_LineTransport):
    """Runs the server as a subprocess, speaking over its stdio."""

    def __init__(self, argv: Iterable[str]) -> None:
        super().__init__()
        args = list(argv)
        self._proc = subprocess.Popen(
            args,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=None,  # banners/diagnostics go straight to the terminal
            text=True,
            bufsize=1,
        )
        assert self._proc.stdout and self._proc.stdin
        self._start_reader(self._proc.stdout.readline)
        self._stdin = self._proc.stdin

    def send(self, line: str) -> None:
        assert self._stdin
        try:
            self._stdin.write(line + "\n")
            self._stdin.flush()
        except (BrokenPipeError, ValueError):
            self.mark_closed()
            raise AgcadError("server process is not running") from None

    def close(self) -> None:
        if self._proc.poll() is None:
            try:
                self._stdin.close() if self._stdin else None
            except Exception:
                pass
            try:
                self._proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self._proc.kill()
        self.mark_closed()


class AgcadTcpTransport(_LineTransport):
    """Connects to a running ``agcad --tcp PORT`` server."""

    def __init__(self, host: str, port: int) -> None:
        super().__init__()
        # The connect timeout must NOT linger on the socket: it would make
        # the reader thread die the first time the connection idles longer
        # than the timeout (agent sessions idle for minutes), and every
        # response after that would rot in the buffer.
        self._sock = socket.create_connection((host, port), timeout=10)
        self._sock.settimeout(None)  # back to blocking mode
        self._file = self._sock.makefile("r", encoding="utf-8")
        self._start_reader(self._file.readline)

    def send(self, line: str) -> None:
        try:
            self._sock.sendall((line + "\n").encode())
        except OSError:
            self.mark_closed()
            raise AgcadError("tcp connection to server lost") from None

    def close(self) -> None:
        try:
            self._sock.close()
        except OSError:
            pass
        self.mark_closed()


def _find_server() -> list[str]:
    """Locate the agcad server binary."""
    env = os.environ.get("AGCAD_SERVER")
    if env:
        return env.split()
    which = shutil.which("agcad")
    if which:
        return [which]
    # Local cargo build, relative to this file: python/agcad/client.py
    root = Path(__file__).resolve().parents[2]
    for profile in ("debug", "release"):
        candidate = root / "target" / profile / "agcad"
        if candidate.is_file():
            return [str(candidate)]
    raise AgcadError(
        "agcad server not found. Build it (cargo build) or set AGCAD_SERVER."
    )


# ---------------------------------------------------------------------------


class Agcad:
    """Synchronous, agent-friendly client. Every method maps 1:1 to a
    command on the bus and raises :class:`AgcadError` on rejection.

    If the connection dies (server restart, network blip) the client
    reconnects transparently on the next call. Note: documents live in
    the server's memory, so after a restart the engine is empty — the
    reconnect just keeps the client usable (the agent sees a clean
    unknown-document error instead of permanent connection failures)."""

    def __init__(self, transport: _LineTransport, reconnect: tuple | None = None) -> None:
        self._t = transport
        self._ids = itertools.count(1)
        # How to build a fresh transport: ("tcp", host, port) or ("spawn", argv).
        self._reconnect = reconnect

    # -- constructors -------------------------------------------------------

    @classmethod
    def spawn(cls, argv: Iterable[str] | None = None) -> "Agcad":
        """Start a fresh server subprocess (stdio transport)."""
        args = list(argv) if argv is not None else _find_server()
        return cls(AgcadStdioTransport(args), reconnect=("spawn", args))

    @classmethod
    def connect(cls, host: str = "127.0.0.1", port: int = 7860) -> "Agcad":
        """Attach to a running server. TCP mode shares one document space
        across all connections; mutations are broadcast to the others."""
        return cls(AgcadTcpTransport(host, port), reconnect=("tcp", host, port))

    def _fresh_transport(self) -> _LineTransport:
        if self._reconnect is None:
            raise AgcadError("connection lost (no reconnect info available)")
        kind = self._reconnect
        if kind[0] == "tcp":
            return AgcadTcpTransport(kind[1], kind[2])
        return AgcadStdioTransport(kind[1])

    # -- plumbing -----------------------------------------------------------

    def call(self, method: str, params: dict[str, Any] | None = None) -> dict[str, Any]:
        """Public raw command invocation. Used by the MCP server and power
        users who want the full bus without typed wrappers."""
        return self._request(method, **(params or {}))

    @staticmethod
    def _is_connection_error(e: AgcadError) -> bool:
        text = str(e)
        return any(
            marker in text
            for marker in ("connection closed", "connection to server lost", "not running")
        )

    def _request(self, method: str, **params: Any) -> dict[str, Any]:
        try:
            return self._do_request(method, **params)
        except AgcadError as e:
            if self._is_connection_error(e):
                # Transport died (server restart, network blip): rebuild it
                # once and retry, so the client stays usable. A restarted
                # server has an empty engine — callers get honest
                # unknown-document errors instead of connection failures.
                self._t.close()
                self._t = self._fresh_transport()
                return self._do_request(method, **params)
            raise

    def _do_request(self, method: str, **params: Any) -> dict[str, Any]:
        rid = next(self._ids)
        self._t.send(json.dumps({"jsonrpc": "2.0", "id": rid, "method": method, "params": params}))
        response = self._t.recv(rid)
        if "error" in response:
            raise AgcadError(response["error"]["message"])
        result = response["result"]
        # Results are externally tagged: {"entity_created": {...}}. Unwrap
        # the single key so callers work with plain dicts.
        (tag, inner), = result.items()
        return {"result": tag, **inner}

    def close(self) -> None:
        self._t.close()

    def __enter__(self) -> "Agcad":
        return self

    def __exit__(self, *exc: Any) -> None:
        self.close()

    # -- session ------------------------------------------------------------

    def initialize(self) -> dict[str, Any]:
        """Handshake. Returns the protocol version and server name."""
        return self._request("initialize")

    def list_documents(self) -> list[dict[str, Any]]:
        return self._request("list_documents")["docs"]

    def create_document(self, name: str = "untitled", doc: int | None = None) -> dict[str, Any]:
        """Create a document.

        Omit ``doc`` to allocate the next id (ids start at 1). Pass an
        explicit unused id (≥ 1) when a single batch should create and then
        address the document: ``create_document("part", doc=10)`` followed by
        commands with ``doc=10``.
        """
        return self._request("create_document", name=name, doc=doc)

    def close_document(self, doc: int) -> dict[str, Any]:
        return self._request("close_document", doc=doc)

    # -- parameters (the agent-authored "function") ---------------------------

    def set_parameter(
        self,
        doc: int,
        name: str,
        value: float | str,
        description: str | None = None,
    ) -> dict[str, Any]:
        """Define or update a named parameter. ``value`` may be an
        expression over other parameters ("wall*2"). Changing a parameter
        replays the document program: entity ids are preserved, geometry
        regenerates."""
        return self._request("set_parameter", doc=doc, name=name, value=value, description=description)

    def remove_parameter(self, doc: int, name: str) -> dict[str, Any]:
        return self._request("remove_parameter", doc=doc, name=name)

    def get_parameters(self, doc: int) -> list[dict[str, Any]]:
        return self._request("get_parameters", doc=doc)["params"]

    # -- modeling -----------------------------------------------------------
    # Dimensions accept numbers or expression strings over document
    # parameters, e.g. size=("plate", 40, "wall*3").

    def add_box(
        self,
        doc: int,
        name: str = "box",
        center: tuple[float | str, float | str, float | str] = (0.0, 0.0, 0.0),
        size: tuple[float | str, float | str, float | str] = (10.0, 10.0, 10.0),
        color: tuple[float, float, float, float] | None = None,
    ) -> dict[str, Any]:
        return self._request("add_box", doc=doc, name=name, center=center, size=size, color=color)

    def add_sphere(
        self,
        doc: int,
        name: str = "sphere",
        center: tuple[float | str, float | str, float | str] = (0.0, 0.0, 0.0),
        radius: float | str = 5.0,
        segments: int | None = None,
        color: tuple[float, float, float, float] | None = None,
    ) -> dict[str, Any]:
        return self._request(
            "add_sphere", doc=doc, name=name, center=center, radius=radius, segments=segments, color=color
        )

    def add_cylinder(
        self,
        doc: int,
        name: str = "cylinder",
        center: tuple[float | str, float | str, float | str] = (0.0, 0.0, 0.0),
        radius: float | str = 5.0,
        height: float | str = 10.0,
        segments: int | None = None,
        color: tuple[float, float, float, float] | None = None,
    ) -> dict[str, Any]:
        """A cylinder centered at ``center``, extruded along **+Z**.

        It spans ``center.z ± height/2`` — no rotation needed to stand it up.
        Use ``transform`` if you need a different axis.
        """
        return self._request(
            "add_cylinder",
            doc=doc, name=name, center=center, radius=radius, height=height,
            segments=segments, color=color,
        )

    def boolean(
        self,
        doc: int,
        op: str,
        entities: list[int],
        name: str | None = None,
        color: tuple[float, float, float, float] | None = None,
    ) -> dict[str, Any]:
        """Union/difference/intersection of world-space solids → new entity.

        Sources are kept; delete them if you only want the fused result.
        For ``difference`` the first id is the base.
        """
        return self._request(
            "boolean", doc=doc, op=op, entities=entities, name=name, color=color
        )

    def fillet(
        self,
        doc: int,
        entity: int,
        radius: float | str,
        edges: dict[str, Any] | None = None,
        min_angle_deg: float | None = None,
    ) -> dict[str, Any]:
        """Round the sharp edges of one solid, in place.

        ``radius`` may be an expression over parameters, so the rounding
        follows a knob change. ``edges`` selects which edges to touch; omit it
        for every sharp edge. Filters are geometric (world space) and ANDed:

        * ``{"direction": [0, 0, 1]}`` — edges parallel to a vector, e.g. the
          vertical corners of an extrusion;
        * ``{"near": [x, y, z], "near_radius": 6}`` — edges whose midpoint is
          close to a point;
        * ``{"normals": [[0, 0, 1], [1, 0, 0]]}`` — the edge between two faces,
          given by their outward normals (either order);
        * ``{"convex": true}`` — only outside corners (``false`` for inside
          ones, which get filled instead of cut).

        ``min_angle_deg`` (default 30) is how sharp a face pair must be to
        count as an edge; tessellation seams of curved surfaces stay below it.
        Returns the new local mesh, the number of edges touched and the
        world-space volume.
        """
        return self._request(
            "fillet", doc=doc, entity=entity, radius=radius, edges=edges,
            min_angle_deg=min_angle_deg,
        )

    def chamfer(
        self,
        doc: int,
        entity: int,
        distance: float | str,
        edges: dict[str, Any] | None = None,
        min_angle_deg: float | None = None,
    ) -> dict[str, Any]:
        """Bevel the sharp edges of one solid with a flat face, in place.

        ``distance`` is the setback along each adjacent face (mm, may be an
        expression). Edge selection is the same as :meth:`fillet`; convex
        edges lose material, concave edges gain it.
        """
        return self._request(
            "chamfer", doc=doc, entity=entity, distance=distance, edges=edges,
            min_angle_deg=min_angle_deg,
        )

    def transform(
        self,
        doc: int,
        entity: int,
        translation: tuple[float | str, float | str, float | str] | None = None,
        rotation_deg: tuple[float | str, float | str, float | str] | None = None,
        scale: tuple[float | str, float | str, float | str] | None = None,
    ) -> dict[str, Any]:
        """Set any subset of the placement; omitted parts are kept."""
        return self._request(
            "transform", doc=doc, entity=entity,
            translation=translation, rotation_deg=rotation_deg, scale=scale,
        )

    def set_color(self, doc: int, entity: int, color: tuple[float, float, float, float]) -> dict[str, Any]:
        return self._request("set_color", doc=doc, entity=entity, color=color)

    def delete(self, doc: int, entity: int) -> dict[str, Any]:
        return self._request("delete", doc=doc, entity=entity)

    # -- sketches -----------------------------------------------------------
    # Sketches are 2D constraint-solved drawings placed in 3D by the entity
    # transform. Coordinates and constraint values may be expression
    # strings, so sketches re-solve when parameters change.

    def add_sketch(self, doc: int, name: str = "sketch") -> dict[str, Any]:
        """Create an empty sketch (default placement: XY plane at origin)."""
        return self._request("add_sketch", doc=doc, name=name)

    def add_sketch_line(
        self,
        doc: int,
        sketch: int,
        p1: tuple[float | str, float | str],
        p2: tuple[float | str, float | str],
    ) -> dict[str, Any]:
        """Append a line segment (sketch-local mm, expressions allowed)."""
        return self._request("add_sketch_line", doc=doc, sketch=sketch, p1=list(p1), p2=list(p2))

    def add_sketch_circle(
        self,
        doc: int,
        sketch: int,
        center: tuple[float | str, float | str],
        radius: float | str,
    ) -> dict[str, Any]:
        """Append a circle (sketch-local mm, expressions allowed)."""
        return self._request(
            "add_sketch_circle", doc=doc, sketch=sketch, center=list(center), radius=radius
        )

    def constrain(
        self,
        doc: int,
        sketch: int,
        kind: str,
        refs: list[dict[str, Any]],
        value: float | str | None = None,
    ) -> dict[str, Any]:
        """Add a constraint and re-solve.

        ``kind`` is one of ``coincident``, ``horizontal``, ``vertical``,
        ``parallel``, ``perpendicular``, ``distance``, ``radius``,
        ``diameter``. Each ref is ``{"element": id, "point": 0|1}`` for a
        point or ``{"element": id}`` for a whole element. The result
        reports ``solved``, ``residual`` and remaining ``dof``.
        """
        return self._request(
            "constrain", doc=doc, sketch=sketch, kind=kind, refs=refs, value=value
        )

    def remove_constraint(self, doc: int, sketch: int, constraint: int) -> dict[str, Any]:
        """Remove a constraint by id and re-solve."""
        return self._request("remove_constraint", doc=doc, sketch=sketch, constraint=constraint)

    def move_sketch_point(
        self,
        doc: int,
        sketch: int,
        element: int,
        point: int,
        to: tuple[float, float],
    ) -> dict[str, Any]:
        """Drag a point (0 = line start / circle center, 1 = line end),
        then re-solve."""
        return self._request(
            "move_sketch_point", doc=doc, sketch=sketch, element=element, point=point, to=list(to)
        )

    def delete_sketch_element(self, doc: int, sketch: int, element: int) -> dict[str, Any]:
        """Delete an element; constraints referencing it cascade away."""
        return self._request(
            "delete_sketch_element", doc=doc, sketch=sketch, element=element
        )

    # -- sketch → solid ------------------------------------------------------

    def extrude(
        self,
        doc: int,
        sketch: int,
        distance: float | str,
        name: str | None = None,
        color: tuple[float, float, float, float] | None = None,
    ) -> dict[str, Any]:
        """Extrude a closed sketch profile along the sketch normal.

        Holes are handled (nested loops alternate solid/hole). A negative
        distance extrudes the other way. The solid follows later sketch
        edits and replays parametrically. Returns ``volume`` and ``tris``.
        """
        return self._request(
            "extrude", doc=doc, sketch=sketch, distance=distance, name=name, color=color
        )

    def revolve(
        self,
        doc: int,
        sketch: int,
        angle_deg: float | str | None = None,
        axis: str | None = None,
        segments: int | None = None,
        name: str | None = None,
        color: tuple[float, float, float, float] | None = None,
    ) -> dict[str, Any]:
        """Revolve a closed sketch profile around an in-plane axis.

        ``axis`` is ``"x"`` (default) or ``"y"`` in sketch-local coordinates;
        ``angle_deg`` defaults to 360. The profile must stay on one side of
        the axis (touching is fine).
        """
        return self._request(
            "revolve", doc=doc, sketch=sketch, angle_deg=angle_deg, axis=axis,
            segments=segments, name=name, color=color,
        )

    def undo(self, doc: int) -> dict[str, Any]:
        return self._request("undo", doc=doc)

    def redo(self, doc: int) -> dict[str, Any]:
        return self._request("redo", doc=doc)

    def get_scene(self, doc: int, meshes: bool = True) -> dict[str, Any]:
        """Scene snapshot: entities with transforms, colors and meshes."""
        return self._request("get_scene", doc=doc)

    # -- persistence ---------------------------------------------------------

    def save_document(
        self,
        doc: int,
        path: str,
        source: dict[str, str] | None = None,
    ) -> dict[str, Any]:
        """Write the document to a ``.agcad`` file.

        The file is JSON holding the program and the current parameter
        values — not the geometry, which is rebuilt deterministically by
        replay on open. ``source`` may carry the original script as
        provenance (``{"language": "python", "text": ...}``); it is never
        executed. Saves are byte-stable: save → open → save is a no-op diff.
        """
        return self._request("save_document", doc=doc, path=path, source=source)

    def open_document(self, path: str, name: str | None = None) -> dict[str, Any]:
        """Open a ``.agcad`` file as a new document and replay its program.

        Returns ``doc`` (the new id), ``name``, and command/parameter/entity
        counts. The stored name is used unless ``name`` overrides it.
        """
        return self._request("open_document", path=path, name=name)

    def get_program(self, doc: int) -> dict[str, Any]:
        """Read-only view of the program and parameters.

        Returns ``name``, ``params`` and ``program`` (the modeling commands
        with expression arguments intact). This is exactly what
        :meth:`save_document` writes, available to any client.
        """
        return self._request("get_program", doc=doc)

    def export_python(self, doc: int, path: str | None = None) -> str:
        """Render the document's program as a runnable Python script.

        The engine stores a language-neutral program; this is its Python
        view, with typed client calls and every parameter value baked in.
        Re-running the script rebuilds the same model, so it also works as a
        readable, diffable save format. Writes ``path`` when given.

        (Generation lives here, not in the engine: the Python method names
        belong to this client, not to the core.)
        """
        snap = self.get_program(doc)
        name = snap["name"]
        params = snap.get("params", [])
        program = snap.get("program", [])

        out = [
            f'"""Generated from the agcad document {name!r}.',
            "",
            "Re-running this script rebuilds the model: it is an ordinary",
            "client, so edit it freely or keep it as the project's source.",
            '"""',
            "",
            "from agcad import Agcad",
            "",
            "",
            "def main() -> None:",
            "    cad = Agcad.spawn()",
            f'    doc = cad.create_document({name!r})["doc"]',
            "",
        ]
        if params:
            out.append("    # knobs — the user-facing parameters")
            for p in params:
                desc = p.get("description")
                if desc:
                    out.append(
                        f"    cad.set_parameter(doc, {p['name']!r}, {p['value']!r}, {desc!r})"
                    )
                else:
                    out.append(f"    cad.set_parameter(doc, {p['name']!r}, {p['value']!r})")
            out.append("")
        out.append("    # program")
        for cmd in program:
            method = cmd["method"]
            args = [
                (k, v)
                for k, v in cmd.items()
                if k not in ("method", "doc") and v is not None
            ]
            one_line = f"    cad.{method}(doc" + "".join(
                f", {k}={v!r}" for k, v in args
            ) + ")"
            if len(one_line) <= 96:
                out.append(one_line)
            else:
                out.append(f"    cad.{method}(")
                out.append("        doc,")
                out.extend(f"        {k}={v!r}," for k, v in args)
                out.append("    )")
        out += ["", "    cad.close()", "", "", 'if __name__ == "__main__":', "    main()"]

        text = "\n".join(out) + "\n"
        if path:
            with open(path, "w", encoding="utf-8") as f:
                f.write(text)
        return text

    def export_stl(self, doc: int, path: str, entities: list[int] | None = None) -> dict[str, Any]:
        """Write a binary STL (world space) — a session command."""
        return self._request("export_stl", doc=doc, path=path, entities=entities)

    def render_offscreen(
        self,
        doc: int,
        width: int | None = None,
        height: int | None = None,
        eye: tuple[float, float, float] | None = None,
        target: tuple[float, float, float] | None = None,
        up: tuple[float, float, float] | None = None,
        fov_deg: float | None = None,
        background: tuple[float, float, float, float] | None = None,
        path: str | None = None,
        ortho: bool | None = None,
    ) -> dict[str, Any]:
        """Headless software render to PNG (base64 in the result).

        Omit ``eye`` for an auto-framed 3/4 view of the whole scene. Set
        ``ortho=True`` for a parallel projection — the standard front/top/
        side CAD view, where sizes do not change with depth.
        Deterministic: same scene + camera → identical bytes."""
        return self._request(
            "render_offscreen", doc=doc, width=width, height=height, eye=eye,
            target=target, up=up, fov_deg=fov_deg, background=background, path=path,
            ortho=ortho,
        )
