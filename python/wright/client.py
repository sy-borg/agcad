"""Wright CAD — Python client for the wright command bus.

The primary interface for AI agents. Plain, dependency-free, synchronous:

>>> from wright import Wright
>>> cad = Wright.spawn()
>>> _ = cad.initialize()
>>> doc = cad.create_document("bracket")["doc"]
>>> box = cad.add_box(doc, name="base", center=(0, 0, 5), size=(10, 10, 10))
>>> box["entity"]
1

The server binary is found via ``$WRIGHT_SERVER``, ``$PATH`` (``wright``), or
the local cargo build (``target/debug/wright`` / ``target/release/wright``).
"""
from __future__ import annotations

import itertools
import json
import os
import shutil
import subprocess
import socket
import threading
import time
from pathlib import Path
from typing import Any, Iterable

__all__ = ["Wright", "WrightError", "WrightStdioTransport", "WrightTcpTransport"]
__version__ = "0.1.0"


class WrightError(RuntimeError):
    """The server rejected a command. ``message`` is agent-readable and
    usually contains a hint on how to recover."""


_DATUM_PLANES = ("xy", "xz", "yz")


def _mirror_plane_payload(plane: Any) -> dict:
    """Normalize the client's mirror `plane` sugar into the wire form.

    Accepted forms:
      * ``"xy" | "xz" | "yz"`` → a datum plane
      * ``{"plane": "xy", "offset": 5}`` → a datum plane
      * ``{"origin": [x, y, z], "normal": [x, y, z]}`` → any plane
      * ``{"datum": {...}}`` / ``{"custom": {...}}`` → wire form, passed through
    """
    if isinstance(plane, str):
        if plane not in _DATUM_PLANES:
            raise ValueError(f"mirror plane must be one of {_DATUM_PLANES} or a plane dict (got {plane!r})")
        return {"datum": {"plane": plane}}
    if not isinstance(plane, dict):
        raise TypeError("mirror plane must be a string, a dict, or a wire-form dict")
    if "datum" in plane or "custom" in plane:
        return dict(plane)
    if "plane" in plane:
        return {"datum": dict(plane)}
    if "origin" in plane:
        return {"custom": dict(plane)}
    raise ValueError(
        "mirror plane dict must contain 'plane' (datum), 'origin' + 'normal' (custom), "
        "or already be a {'datum': ...} / {'custom': ...} wire form"
    )


def _sweep_path_payload(path: Any) -> dict:
    """Normalize a sweep path into the wire form.

    * a list of ``[x, y, z]`` points → a polyline;
    * ``{"points": [...], "closed": false}`` → a polyline;
    * ``{"radius": ..., "pitch": ..., "turns": ...}`` → a helix around +Z;
    * ``{"polyline": {...}}`` / ``{"helix": {...}}`` → wire form, passed through.
    """
    if isinstance(path, (list, tuple)):
        return {"polyline": {"points": [list(p) for p in path]}}
    if not isinstance(path, dict):
        raise TypeError("sweep path must be a list of points or a dict")
    if "polyline" in path or "helix" in path:
        return dict(path)
    if "points" in path:
        return {"polyline": dict(path)}
    return {"helix": dict(path)}


def _sketch_plane_payload(plane: Any, offset: float | str | None) -> dict | None:
    """Normalize the client's `plane` sugar into the wire `SketchPlane`.

    Accepted forms:
      * ``None`` → the XY datum plane at the origin
      * ``"xy" | "xz" | "yz"`` → a datum plane (``offset`` shifts it along
        its normal)
      * ``{"plane": "xy", "offset": 5}`` → a datum plane
      * ``{"entity": 1, "normal": [0, 0, 1], "near": [...]}`` → a face
      * ``{"face": {...}}`` → a face (wire form, passed through)
      * ``{"origin": [...], "normal": [...], "x_axis": [...]}`` → a custom
        plane
      * ``{"datum": {...}}`` / ``{"custom": {...}}`` → wire form, passed
        through
    """
    if plane is None:
        if offset is None:
            return None
        plane = "xy"
    if isinstance(plane, str):
        if plane not in _DATUM_PLANES:
            raise ValueError(f"plane must be one of {_DATUM_PLANES}, a face/custom dict, or None (got {plane!r})")
        spec: dict[str, Any] = {"plane": plane}
        if offset is not None:
            spec["offset"] = offset
        return {"datum": spec}
    if not isinstance(plane, dict):
        raise TypeError("plane must be a string, a dict, or None")

    if offset is not None and "datum" not in plane and "plane" not in plane:
        raise ValueError("offset only applies to a datum plane (plane='xy'/'xz'/'yz' or {'plane': ...})")

    payload = dict(plane)
    if "datum" in payload:
        if offset is not None:
            payload["datum"] = {**payload["datum"], "offset": offset}
        return payload
    if "custom" in payload or "face" in payload:
        return payload
    if "plane" in payload:
        if offset is not None:
            payload["offset"] = offset
        return {"datum": payload}
    if "origin" in payload:
        return {"custom": payload}
    if "entity" in payload:
        return {"face": payload}
    raise ValueError(
        "plane dict must contain 'plane' (datum), 'origin' + 'normal' (custom), "
        "or 'entity' / 'face' (an existing solid's face)"
    )


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
                    raise WrightError("server connection closed")
                if not self._cond.wait(timeout):
                    raise WrightError(f"timed out waiting for response to request {rid}")
            return self._responses.pop(rid)

    def mark_closed(self) -> None:
        with self._cond:
            self._closed = True
            self._cond.notify_all()


class WrightStdioTransport(_LineTransport):
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
            raise WrightError("server process is not running") from None

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


class WrightTcpTransport(_LineTransport):
    """Connects to a running ``wright --tcp PORT`` server."""

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
            raise WrightError("tcp connection to server lost") from None

    def close(self) -> None:
        try:
            self._sock.close()
        except OSError:
            pass
        self.mark_closed()


def _find_server() -> list[str]:
    """Locate the wright server binary."""
    env = os.environ.get("WRIGHT_SERVER")
    if env:
        return env.split()
    which = shutil.which("wright")
    if which:
        return [which]
    # Local cargo build, relative to this file: python/wright/client.py
    root = Path(__file__).resolve().parents[2]
    for profile in ("debug", "release"):
        candidate = root / "target" / profile / "wright"
        if candidate.is_file():
            return [str(candidate)]
    raise WrightError(
        "wright server not found. Build it (cargo build) or set WRIGHT_SERVER."
    )


# Retry windows for opening a transport. A restarting server is unavailable
# for a moment, so a single `ConnectionRefused` must not surface as a permanent
# failure. These total ~1.15 s; a tool call stays responsive while covering a
# normal process restart.
_CONNECT_DELAYS = (0.0, 0.1, 0.3, 0.75)


def _open_with_retry(
    kind: tuple, delays: tuple[float, ...], what: str
) -> "_LineTransport":
    """Build a transport, retrying transient failures with a short backoff.

    ``kind`` is ``("tcp", host, port)`` or ``("spawn", argv)``. Failures are
    wrapped in :class:`WrightError` so callers (and the MCP boundary) get an
    agent-readable message instead of a bare ``ConnectionRefusedError``.
    """
    last: OSError | None = None
    for delay in delays:
        if delay:
            time.sleep(delay)
        try:
            if kind[0] == "tcp":
                return WrightTcpTransport(kind[1], kind[2])
            return WrightStdioTransport(kind[1])
        except OSError as e:
            last = e
    raise WrightError(
        f"could not {what} the wright server ({last}); "
        "if it is still starting, retry the command"
    ) from None


# ---------------------------------------------------------------------------


class Wright:
    """Synchronous, agent-friendly client. Every method maps 1:1 to a
    command on the bus and raises :class:`WrightError` on rejection.

    If the connection dies (server restart, network blip) the client
    reconnects on the next call, retrying briefly while the server comes
    back. Documents live in the server's memory, so after a restart the
    engine is empty — reconnection keeps the client usable (the agent sees
    a clean unknown-document error instead of permanent connection
    failures) and :attr:`generation` tells callers that the transport was
    rebuilt, so they can re-check ``list_documents`` and replay the script.
    """

    def __init__(self, transport: _LineTransport, reconnect: tuple | None = None) -> None:
        self._t = transport
        self._ids = itertools.count(1)
        # How to build a fresh transport: ("tcp", host, port) or ("spawn", argv).
        self._reconnect = reconnect
        # Counts successful transport rebuilds; see the `generation` property.
        self._generation = 0

    # -- constructors -------------------------------------------------------

    @classmethod
    def spawn(cls, argv: Iterable[str] | None = None) -> "Wright":
        """Start a fresh server subprocess (stdio transport)."""
        args = list(argv) if argv is not None else _find_server()
        return cls(
            _open_with_retry(("spawn", args), _CONNECT_DELAYS, "start"),
            reconnect=("spawn", args),
        )

    @classmethod
    def connect(cls, host: str = "127.0.0.1", port: int = 7860) -> "Wright":
        """Attach to a running server. TCP mode shares one document space
        across all connections; mutations are broadcast to the others."""
        return cls(
            _open_with_retry(("tcp", host, port), _CONNECT_DELAYS, "connect to"),
            reconnect=("tcp", host, port),
        )

    @property
    def generation(self) -> int:
        """Counts transport rebuilds. It moves when the client reconnected
        after a lost connection. Compare it between calls: a new generation
        means the server may have restarted (an empty in-memory document
        space), so re-check ``list_documents`` before using old ids."""
        return self._generation

    def _fresh_transport(self) -> _LineTransport:
        if self._reconnect is None:
            raise WrightError("connection lost (no reconnect info available)")
        return _open_with_retry(self._reconnect, _CONNECT_DELAYS, "reconnect to")

    # -- plumbing -----------------------------------------------------------

    def call(self, method: str, params: dict[str, Any] | None = None) -> dict[str, Any]:
        """Public raw command invocation. Used by the MCP server and power
        users who want the full bus without typed wrappers."""
        return self._request(method, **(params or {}))

    @staticmethod
    def _is_connection_error(e: WrightError) -> bool:
        text = str(e)
        return any(
            marker in text
            for marker in ("connection closed", "connection to server lost", "not running")
        )

    def _request(self, method: str, **params: Any) -> dict[str, Any]:
        try:
            return self._do_request(method, **params)
        except WrightError as e:
            if self._is_connection_error(e):
                # Transport died (server restart, network blip): rebuild it
                # once and retry, so the client stays usable. A restarted
                # server has an empty engine — callers get honest
                # unknown-document errors instead of connection failures.
                self._t.close()
                self._t = self._fresh_transport()
                self._generation += 1
                return self._do_request(method, **params)
            raise

    def _do_request(self, method: str, **params: Any) -> dict[str, Any]:
        rid = next(self._ids)
        self._t.send(json.dumps({"jsonrpc": "2.0", "id": rid, "method": method, "params": params}))
        response = self._t.recv(rid)
        if "error" in response:
            raise WrightError(response["error"]["message"])
        result = response["result"]
        # Results are externally tagged: {"entity_created": {...}}. Unwrap
        # the single key so callers work with plain dicts.
        (tag, inner), = result.items()
        return {"result": tag, **inner}

    def close(self) -> None:
        self._t.close()

    def __enter__(self) -> "Wright":
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
        re-executes the document program: entity ids are preserved and only
        geometry downstream of the change is regenerated. The result reports
        ``rebuilt`` and ``reused`` feature counts plus the number of program
        ``steps``."""
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

    def import_mesh(
        self,
        doc: int,
        path: str,
        name: str | None = None,
        scale: float | None = None,
        format: str | None = None,
        smooth_angle_deg: float | None = None,
        color: tuple[float, float, float, float] | None = None,
    ) -> dict[str, Any]:
        """Read an STL, OBJ or 3MF file on the **server** and add it as a solid.

        ``path`` is resolved on the server, so absolute paths are least
        surprising. 3MF's declared unit is converted to millimeters
        automatically; ``scale`` multiplies the result (e.g. 25.4 for an
        inch-unit file without metadata). ``format`` (``"stl"``, ``"obj"`` or
        ``"3mf"``) overrides detection from the extension and contents;
        ``name`` defaults to the file stem.

        The parsed triangles are recorded as an ``add_mesh`` program step, so
        the resulting ``.wcad`` replays without the source file. Vertices are
        welded, degenerate and duplicate triangles are dropped, an inward
        winding is flipped outward, and ``smooth_angle_deg`` (default 30)
        controls which face pairs share shading normals. A non-watertight
        import is still created — :meth:`validate` reports it, and booleans
        may reject it. Returns the cleaned vertex/triangle/dropped counts.
        """
        return self._request(
            "import_mesh",
            doc=doc,
            path=path,
            name=name,
            scale=scale,
            format=format,
            smooth_angle_deg=smooth_angle_deg,
            color=color,
        )

    def add_mesh(
        self,
        doc: int,
        name: str = "mesh",
        positions: list[tuple[float, float, float]] | list[list[float]] | None = None,
        indices: list[int] | list[list[int]] | None = None,
        smooth_angle_deg: float | None = None,
        color: tuple[float, float, float, float] | None = None,
    ) -> dict[str, Any]:
        """Add a solid from raw triangles (the program step import_mesh records).

        ``indices`` is accepted flat (three per triangle) or as a list of
        triples; it is flattened here. The same cleanup as :meth:`import_mesh`
        applies (welding, degenerate/duplicate removal, outward winding).
        """
        if positions is None or indices is None:
            raise TypeError("add_mesh needs positions and indices")
        flat: list[int] = []
        for item in indices:
            if isinstance(item, (list, tuple)):
                flat.extend(int(i) for i in item)
            else:
                flat.append(int(item))
        return self._request(
            "add_mesh",
            doc=doc,
            name=name,
            positions=[list(p) for p in positions],
            indices=flat,
            smooth_angle_deg=smooth_angle_deg,
            color=color,
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

    def add_sketch(
        self,
        doc: int,
        name: str = "sketch",
        plane: str | dict[str, Any] | None = None,
        offset: float | str | None = None,
    ) -> dict[str, Any]:
        """Create an empty sketch, optionally on a datum plane or a solid face.

        ``plane`` accepts:

        * ``None`` — the XY datum plane at the origin (the default);
        * ``"xy"``, ``"xz"``, ``"yz"`` — a standard datum plane. ``offset``
          shifts it along its normal, and may be an expression;
        * ``{"entity": id, "normal": [0, 0, 1]}`` — a planar face of an
          existing solid. The sketch normal is the face's outward normal, so
          ``extrude`` with a positive distance adds material in front of the
          face and a negative distance cuts into the body. Add
          ``"near": [x, y, z]`` to choose between parallel faces with the
          same normal; the sketch origin is ``near`` projected onto the
          chosen face, or the face center when omitted;
        * ``{"origin": [x, y, z], "normal": [x, y, z], "x_axis": [...]}`` —
          an explicit plane (``x_axis`` is optional and projected in-plane).

        The datum frames are right-handed: ``xy`` has X=+X, Y=+Y, normal=+Z;
        ``xz`` has X=+X, Y=+Z, normal=-Y; ``yz`` has X=+Y, Y=+Z, normal=+X.
        A face or custom plane takes world +X projected onto the plane as its
        local X (world +Y when the plane is nearly perpendicular to X). The
        resolved frame is reported as ``sketch.plane`` in ``get_scene``.
        """
        return self._request(
            "add_sketch",
            doc=doc,
            name=name,
            plane=_sketch_plane_payload(plane, offset),
        )

    def add_sketch_line(
        self,
        doc: int,
        sketch: int,
        p1: tuple[float | str, float | str],
        p2: tuple[float | str, float | str],
        construction: bool = False,
    ) -> dict[str, Any]:
        """Append a line segment (sketch-local mm, expressions allowed).

        ``construction=True`` draws a guide that participates in constraints
        and display but is excluded from the extrudable profile.
        """
        return self._request(
            "add_sketch_line",
            doc=doc,
            sketch=sketch,
            p1=list(p1),
            p2=list(p2),
            construction=construction,
        )

    def add_sketch_circle(
        self,
        doc: int,
        sketch: int,
        center: tuple[float | str, float | str],
        radius: float | str,
        construction: bool = False,
    ) -> dict[str, Any]:
        """Append a circle (sketch-local mm, expressions allowed)."""
        return self._request(
            "add_sketch_circle",
            doc=doc,
            sketch=sketch,
            center=list(center),
            radius=radius,
            construction=construction,
        )

    def add_sketch_arc(
        self,
        doc: int,
        sketch: int,
        center: tuple[float | str, float | str],
        radius: float | str,
        start_deg: float | str,
        end_deg: float | str,
        construction: bool = False,
    ) -> dict[str, Any]:
        """Append a counter-clockwise arc from ``start_deg`` to ``end_deg``.

        Angles are degrees from the sketch's local +X. A sweep of 360° is a
        full circle, so use :meth:`add_sketch_circle` for that. ``construction``
        excludes the arc from the extrudable profile.
        """
        return self._request(
            "add_sketch_arc",
            doc=doc,
            sketch=sketch,
            center=list(center),
            radius=radius,
            start_deg=start_deg,
            end_deg=end_deg,
            construction=construction,
        )

    def add_sketch_spline(
        self,
        doc: int,
        sketch: int,
        points: list[tuple[float | str, float | str]],
        closed: bool = False,
        construction: bool = False,
    ) -> dict[str, Any]:
        """Append an interpolating spline through ``points``.

        The curve passes through every point (Catmull-Rom). An open spline
        needs at least 2 points, a closed one at least 3; open endpoints
        connect to other elements like a line.
        """
        return self._request(
            "add_sketch_spline",
            doc=doc,
            sketch=sketch,
            points=[list(p) for p in points],
            closed=closed,
            construction=construction,
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
        ``diameter``, ``equal``, ``concentric``, ``symmetric``, ``angle``,
        ``point_on_line`` or ``tangent``. Each ref is
        ``{"element": id, "point": 0|1|...}`` for a point or ``{"element": id}``
        for a whole element. The result reports ``solved``, ``residual``,
        ``dof`` and ``diagnostics``; ``over_constrained`` names redundant
        constraints by id.
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
        """Drag a point then re-solve.

        Point refs are line 0=start/1=end, circle 0=center, arc
        0=center/1=start/2=end, spline ``i``=control point ``i``.
        """
        return self._request(
            "move_sketch_point", doc=doc, sketch=sketch, element=element, point=point, to=list(to)
        )

    def delete_sketch_element(self, doc: int, sketch: int, element: int) -> dict[str, Any]:
        """Delete an element; constraints referencing it cascade away."""
        return self._request(
            "delete_sketch_element", doc=doc, sketch=sketch, element=element
        )

    def trim_sketch(
        self, doc: int, sketch: int, element: int, at: tuple[float, float]
    ) -> dict[str, Any]:
        """Trim the piece of ``element`` containing ``at`` (sketch-local).

        The piece is bounded by the nearest intersections with other elements;
        circles need two cuts and become arcs. Returns ``created``/``removed``/
        ``modified`` element ids plus the solver report.
        """
        return self._request(
            "trim_sketch", doc=doc, sketch=sketch, element=element, at=list(at)
        )

    def extend_sketch(
        self, doc: int, sketch: int, element: int, point: int
    ) -> dict[str, Any]:
        """Extend one line endpoint (0=start/1=end) to its nearest boundary."""
        return self._request(
            "extend_sketch", doc=doc, sketch=sketch, element=element, point=point
        )

    def offset_sketch(
        self,
        doc: int,
        sketch: int,
        distance: float | str,
        elements: list[int] | None = None,
        construction: bool | None = None,
    ) -> dict[str, Any]:
        """Offset curves, creating new elements.

        Positive distances move to the left of each element's direction; a
        single closed loop of lines is mitered into a proper outline and
        positive moves outward. ``elements`` restricts the selection (default:
        every printable element). Lines, circles, arcs and splines are
        supported; spline offsets move the control points.
        """
        return self._request(
            "offset_sketch",
            doc=doc,
            sketch=sketch,
            elements=elements,
            distance=distance,
            construction=construction,
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
        distance extrudes the other way — on a sketch placed on a face, the
        normal is the face's outward normal, so positive adds material in
        front of the face and negative cuts into the body (combine with
        ``boolean("difference", ...)`` to remove it). The solid follows
        later sketch edits and replays parametrically. Returns ``volume``
        and ``tris``.
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

    # -- shape leverage -------------------------------------------------------

    def linear_pattern(
        self,
        doc: int,
        entities: list[int],
        direction: tuple[float | str, float | str, float | str],
        spacing: float | str,
        count: int,
        name: str | None = None,
        color: tuple[float, float, float, float] | None = None,
    ) -> dict[str, Any]:
        """Repeat solids along a line (`count` includes the original).

        Creates ``count - 1`` copies per source and returns their ids in
        ``entities``. The copies are new solids, so a pattern of hole cutters
        is one ``boolean("difference", ...)`` away from a perforated part.
        ``spacing`` may be an expression. Sketches are display-only: extrude
        first.
        """
        return self._request(
            "linear_pattern", doc=doc, entities=list(entities), direction=direction,
            spacing=spacing, count=count, name=name, color=color,
        )

    def circular_pattern(
        self,
        doc: int,
        entities: list[int],
        count: int,
        center: tuple[float | str, float | str, float | str] = (0.0, 0.0, 0.0),
        axis: tuple[float, float, float] = (0.0, 0.0, 1.0),
        angle_deg: float | str | None = 360.0,
        name: str | None = None,
        color: tuple[float, float, float, float] | None = None,
    ) -> dict[str, Any]:
        """Repeat solids around an axis (`count` includes the original).

        Copies are spread evenly over ``angle_deg`` (step = angle / count) and
        rotate with the pattern, so gear teeth and bolt circles keep their
        orientation. ``center`` and ``angle_deg`` may be expressions.
        """
        return self._request(
            "circular_pattern", doc=doc, entities=list(entities), center=center,
            axis=list(axis), angle_deg=angle_deg, count=count, name=name, color=color,
        )

    def mirror(
        self,
        doc: int,
        entities: list[int],
        plane: str | dict[str, Any],
        name: str | None = None,
        color: tuple[float, float, float, float] | None = None,
    ) -> dict[str, Any]:
        """Mirror solids across a plane, creating one copy per source.

        ``plane`` is ``"xy"``/``"xz"``/``"yz"``, ``{"plane": "xy", "offset":
        5}`` or ``{"origin": [...], "normal": [...]}``. The originals are
        kept. The reflection bakes the mesh, so winding and normals stay
        outward.
        """
        return self._request(
            "mirror", doc=doc, entities=list(entities),
            plane=_mirror_plane_payload(plane), name=name, color=color,
        )

    def shell(
        self,
        doc: int,
        entity: int,
        thickness: float | str,
        open: list[tuple[float, float, float]] | None = None,
        angle_tol_deg: float | None = None,
    ) -> dict[str, Any]:
        """Hollow a solid in place with a wall of ``thickness``.

        ``open`` lists outward face normals to remove, so ``[(0, 0, 1)]``
        turns a closed box into a cup; omit it for a sealed hollow body. The
        thickness may be an expression. Returns the new volume, triangle count
        and local mesh; the entity keeps its id.
        """
        return self._request(
            "shell", doc=doc, entity=entity, thickness=thickness,
            open=[list(n) for n in open] if open else [], angle_tol_deg=angle_tol_deg,
        )

    def hole(
        self,
        doc: int,
        entity: int,
        center: tuple[float | str, float | str, float | str],
        diameter: float | str,
        normal: tuple[float, float, float] = (0.0, 0.0, 1.0),
        depth: float | str | None = None,
        counterbore_diameter: float | str | None = None,
        counterbore_depth: float | str | None = None,
        countersink_angle_deg: float | str | None = None,
        segments: int | None = None,
    ) -> dict[str, Any]:
        """Drill a hole into a solid in place, with an optional screw seat.

        ``center`` is a point on the entry surface and ``normal`` points out of
        the material (the hole runs along ``-normal``). Omit ``depth`` for a
        through-hole (or pass a negative value). Give ``counterbore_diameter``
        plus ``counterbore_depth`` for a cylindrical recess, or
        ``countersink_angle_deg`` (included angle, e.g. 90) for a conical one.
        Returns the new volume, triangle count and local mesh.
        """
        return self._request(
            "hole", doc=doc, entity=entity, center=center, normal=list(normal),
            diameter=diameter, depth=depth,
            counterbore_diameter=counterbore_diameter,
            counterbore_depth=counterbore_depth,
            countersink_angle_deg=countersink_angle_deg, segments=segments,
        )

    def sweep(
        self,
        doc: int,
        sketch: int,
        path: Any,
        segments: int | None = None,
        name: str | None = None,
        color: tuple[float, float, float, float] | None = None,
    ) -> dict[str, Any]:
        """Sweep a closed sketch profile along a 3D path.

        ``path`` is a list of world-space points, ``{"points": [...],
        "closed": false}``, or a helix ``{"radius": 12, "pitch": 8, "turns":
        3, "hand": "right"}``. The profile is re-oriented at every station,
        so the sketch supplies the cross-section and its in-plane X, while the
        path supplies the placement. A helix path makes a spring.
        """
        return self._request(
            "sweep", doc=doc, sketch=sketch, path=_sweep_path_payload(path),
            segments=segments, name=name, color=color,
        )

    def loft(
        self,
        doc: int,
        sketches: list[int],
        segments: int | None = None,
        name: str | None = None,
        color: tuple[float, float, float, float] | None = None,
    ) -> dict[str, Any]:
        """Loft a solid through two or more closed sketch profiles.

        Each sketch must have one closed outer loop (holes are not lofted).
        Profiles are resampled to a common vertex count and aligned, so a
        square lofts smoothly into a circle. Profiles may sit on different
        planes; place them with ``add_sketch(plane=..., offset=...)``.
        """
        return self._request(
            "loft", doc=doc, sketches=list(sketches), segments=segments,
            name=name, color=color,
        )

    def thread(
        self,
        doc: int,
        major_diameter: float | str,
        pitch: float | str,
        length: float | str,
        minor_diameter: float | str | None = None,
        hand: str = "right",
        flank_angle_deg: float | str | None = None,
        segments_per_turn: int | None = None,
        origin: tuple[float | str, float | str, float | str] | None = None,
        name: str | None = None,
        color: tuple[float, float, float, float] | None = None,
    ) -> dict[str, Any]:
        """A real screw thread as a standalone solid (axis +Z).

        The minor diameter defaults to the ISO metric ``major - 1.0825 *
        pitch``; ``flank_angle_deg`` is the included angle (default 60).
        ``hand`` is ``"right"`` or ``"left"``. Union the result with a boss
        for a screw, or subtract it from a body for a threaded hole.
        """
        if hand not in ("right", "left"):
            raise ValueError('hand must be "right" or "left"')
        return self._request(
            "thread", doc=doc, major_diameter=major_diameter,
            minor_diameter=minor_diameter, pitch=pitch, length=length,
            hand=hand, flank_angle_deg=flank_angle_deg,
            segments_per_turn=segments_per_turn, origin=origin,
            name=name, color=color,
        )

    def undo(self, doc: int) -> dict[str, Any]:
        return self._request("undo", doc=doc)

    def redo(self, doc: int) -> dict[str, Any]:
        return self._request("redo", doc=doc)

    def get_scene(self, doc: int, meshes: bool = True) -> dict[str, Any]:
        """Scene snapshot: entities with transforms, colors and meshes."""
        return self._request("get_scene", doc=doc)

    # -- trust ---------------------------------------------------------------

    def validate(
        self,
        doc: int,
        entities: list[int] | None = None,
        min_wall_mm: float | None = None,
    ) -> dict[str, Any]:
        """Check that the document is printable.

        For every solid this reports watertightness (open and non-manifold
        edges), degenerate triangles, self-intersections, signed world
        volume, world bbox and an approximate thinnest wall; for every
        sketch it reports solver status and whether the profile is closed.

        ``ok`` is the short answer: all solids are closed, manifold,
        self-intersection free, have positive volume and are at least
        ``min_wall_mm`` thick when that threshold is given; all sketches are
        solved and (when non-empty) closed. ``entities`` restricts the check.

        A session command: it changes no state and is not part of the
        program. The wall estimate casts one inward ray per sampled triangle,
        so it is a fast sanity check rather than a medial-axis measurement.
        """
        return self._request(
            "validate", doc=doc, entities=entities, min_wall_mm=min_wall_mm
        )

    def diff_scene(
        self,
        doc: int,
        against: int | None = None,
        path: str | None = None,
    ) -> dict[str, Any]:
        """Compare two revisions by entity signature.

        The baseline is either another open document (``against``) or a saved
        ``.wcad`` file (``path``); pass exactly one. The result classifies
        entities as added, removed, modified (with the changed fields and
        volume/triangle deltas) or unchanged, and reports parameter and
        program changes. ``scene_signature``/``against_signature`` are stable
        content fingerprints.

        Reading a file baseline is read-only: it is replayed privately and
        never inserted into the document list. A session command — it
        changes no state and is not part of the program.
        """
        return self._request("diff_scene", doc=doc, against=against, path=path)

    # -- persistence ---------------------------------------------------------

    def save_document(
        self,
        doc: int,
        path: str,
        source: dict[str, str] | None = None,
    ) -> dict[str, Any]:
        """Write the document to a ``.wcad`` file.

        The file is JSON holding the program and the current parameter
        values — not the geometry, which is rebuilt deterministically by
        replay on open. ``source`` carries the original script as the
        canonical recipe (``{"language": "python", "text": ...}``); it is
        remembered on the document and never executed. Saves are byte-stable:
        save → open → save is a no-op diff.
        """
        return self._request("save_document", doc=doc, path=path, source=source)

    def open_document(self, path: str, name: str | None = None) -> dict[str, Any]:
        """Open a ``.wcad`` file as a new document and replay its program.

        Returns ``doc`` (the new id), ``name``, and command/parameter/entity
        counts. The stored name is used unless ``name`` overrides it. Any
        embedded recipe is available through :meth:`get_program` or
        :meth:`get_script`.
        """
        return self._request("open_document", path=path, name=name)

    def get_program(self, doc: int) -> dict[str, Any]:
        """Read-only view of the program, parameters and embedded source.

        Returns ``name``, ``params``, ``program`` (the modeling commands with
        expression arguments intact) and, when the document was saved with
        one, ``source`` — the authoring script that rode along in the
        ``.wcad`` snapshot. This is exactly what :meth:`save_document`
        writes, available to any client.

        The script on disk is the source of truth. ``source`` is how a
        handoff recovers it without executing anything; use :meth:`get_script`
        if you just want the text.
        """
        return self._request("get_program", doc=doc)

    def get_script(self, doc: int, path: str | None = None) -> str:
        """Return the canonical Python script for a document.

        If the document was saved with an embedded source, that text is
        returned unchanged — the script is the source of truth. Otherwise the
        program is rendered back into a runnable client script with
        :meth:`export_python`. Writes ``path`` when given.
        """
        snap = self.get_program(doc)
        source = snap.get("source")
        if source and source.get("language") == "python":
            text = source.get("text", "")
        else:
            text = self.export_python(doc)
        if path:
            with open(path, "w", encoding="utf-8") as f:
                f.write(text)
        return text

    def export_python(self, doc: int, path: str | None = None) -> str:
        """Render the document's program as a runnable Python script.

        The engine stores a language-neutral program; this is its Python
        view, with typed client calls and every parameter value baked in.
        Re-running the script rebuilds the same model, so it also works as a
        readable, diffable save format. Writes ``path`` when given.

        This is the reconstruction path when no embedded source exists. If
        the document carries the authoring script, :meth:`get_script` returns
        that original text instead. (Generation lives here, not in the
        engine: the Python method names belong to this client, not to the
        core.)
        """
        snap = self.get_program(doc)
        name = snap["name"]
        params = snap.get("params", [])
        program = snap.get("program", [])

        out = [
            f'"""Generated from the wright document {name!r}.',
            "",
            "Re-running this script rebuilds the model: it is an ordinary",
            "client, so edit it freely or keep it as the project's source.",
            '"""',
            "",
            "from wright import Wright",
            "",
            "",
            "def main() -> None:",
            "    cad = Wright.spawn()",
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

    def export_3mf(self, doc: int, path: str, entities: list[int] | None = None) -> dict[str, Any]:
        """Write a 3MF package (world space, multi-object with names and
        per-object colors) — a session command."""
        return self._request("export_3mf", doc=doc, path=path, entities=entities)

    def export_dxf(
        self,
        doc: int,
        path: str,
        entities: list[int] | None = None,
        curves: str = "elements",
    ) -> dict[str, Any]:
        """Write a 2D ASCII DXF (R12) of sketches — a session command.

        Coordinates are **sketch-local** (u, v) millimeters, not world XYZ.
        ``entities`` defaults to every sketch; name an extrude/revolve/sweep/
        loft solid to export its source sketch's closed profile instead.
        ``curves`` is ``"elements"`` (solved lines/circles/arcs/splines),
        ``"profiles"`` (closed loops only, e.g. for a cutter) or ``"both"``.
        """
        return self._request(
            "export_dxf", doc=doc, path=path, entities=entities, curves=curves
        )

    def export_svg(
        self,
        doc: int,
        path: str,
        entities: list[int] | None = None,
        curves: str = "elements",
    ) -> dict[str, Any]:
        """Write an SVG of sketches (same selection as :meth:`export_dxf`).

        The viewBox is in millimeters, construction geometry is dashed and
        profile loops form their own group — a session command.
        """
        return self._request(
            "export_svg", doc=doc, path=path, entities=entities, curves=curves
        )

    def pick(
        self,
        doc: int,
        origin: tuple[float, float, float],
        direction: tuple[float, float, float],
        tolerance: float | None = None,
    ) -> dict[str, Any]:
        """Cast a world-space ray and return the nearest solid hit.

        ``hit`` is ``None`` when the ray misses (a valid answer, not an
        error). A hit carries its entity/name/point/normal plus two
        ready-made selectors: ``face`` (pass it as ``add_sketch(plane=...)``)
        and, when the hit is within ``tolerance`` mm of a sharp edge,
        ``edge`` (pass it to ``fillet``/``chamfer`` as ``edges=...``).
        Sketches are display-only and never absorb a ray. Omit
        ``tolerance`` for 1% of the picked entity's bbox diagonal."""
        return self._request(
            "pick", doc=doc,
            ray={"origin": list(origin), "direction": list(direction)},
            tolerance=tolerance,
        )

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
