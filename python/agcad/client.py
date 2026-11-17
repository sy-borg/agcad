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
            while True:
                try:
                    line = readline()
                except Exception:
                    break
                if not line:
                    break
                try:
                    response = json.loads(line)
                except json.JSONDecodeError:
                    continue
                rid = response.get("id")
                if isinstance(rid, int):
                    with self._cond:
                        self._responses[rid] = response
                        self._cond.notify_all()
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
        self._sock = socket.create_connection((host, port), timeout=10)
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
    command on the bus and raises :class:`AgcadError` on rejection."""

    def __init__(self, transport: _LineTransport) -> None:
        self._t = transport
        self._ids = itertools.count(1)

    # -- constructors -------------------------------------------------------

    @classmethod
    def spawn(cls, argv: Iterable[str] | None = None) -> "Agcad":
        """Start a fresh server subprocess (stdio transport)."""
        return cls(AgcadStdioTransport(argv if argv is not None else _find_server()))

    @classmethod
    def connect(cls, host: str = "127.0.0.1", port: int = 7860) -> "Agcad":
        """Attach to a running server. Note: each connection gets its own
        isolated document set."""
        return cls(AgcadTcpTransport(host, port))

    # -- plumbing -----------------------------------------------------------

    def call(self, method: str, params: dict[str, Any] | None = None) -> dict[str, Any]:
        """Public raw command invocation. Used by the MCP server and power
        users who want the full bus without typed wrappers."""
        return self._request(method, **(params or {}))

    def _request(self, method: str, **params: Any) -> dict[str, Any]:
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

    def create_document(self, name: str = "untitled") -> dict[str, Any]:
        return self._request("create_document", name=name)

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
        return self._request(
            "add_cylinder",
            doc=doc, name=name, center=center, radius=radius, height=height,
            segments=segments, color=color,
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

    def undo(self, doc: int) -> dict[str, Any]:
        return self._request("undo", doc=doc)

    def redo(self, doc: int) -> dict[str, Any]:
        return self._request("redo", doc=doc)

    def get_scene(self, doc: int, meshes: bool = True) -> dict[str, Any]:
        """Scene snapshot: entities with transforms, colors and meshes."""
        return self._request("get_scene", doc=doc)
