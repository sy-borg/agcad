#!/usr/bin/env python3
"""Drive the agcad MCP server exactly like an MCP host would:
initialize handshake, notifications/initialized, tools/list, tools/call."""
import json
import os
import subprocess
import sys
from pathlib import Path

root = Path(__file__).resolve().parents[1]
env = {**os.environ, "PYTHONPATH": str(root / "python")}
proc = subprocess.Popen(
    [sys.executable, "-m", "agcad.mcp_server"],
    stdin=subprocess.PIPE,
    stdout=subprocess.PIPE,
    stderr=None,  # banners visible in the terminal
    text=True,
    bufsize=1,
    env=env,
    cwd=root,
)


def send(msg: dict) -> None:
    proc.stdin.write(json.dumps(msg) + "\n")
    proc.stdin.flush()


def recv() -> dict:
    return json.loads(proc.stdout.readline())


try:
    # --- handshake --------------------------------------------------------
    send({"jsonrpc": "2.0", "id": 1, "method": "initialize", "params": {
        "protocolVersion": "2025-06-18", "capabilities": {},
        "clientInfo": {"name": "smoke", "version": "0"}}})
    init = recv()["result"]
    assert init["serverInfo"]["name"] == "agcad"
    assert "instructions" in init and "parametric" in init["instructions"]
    print("initialize: ok | protocol", init["protocolVersion"])

    send({"jsonrpc": "2.0", "method": "notifications/initialized"})  # no response expected
    send({"jsonrpc": "2.0", "id": 2, "method": "ping"})
    assert recv()["result"] == {}
    print("ping: ok")

    send({"jsonrpc": "2.0", "id": 3, "method": "tools/list"})
    tools = recv()["result"]["tools"]
    names = [t["name"] for t in tools]
    assert names == ["agcad_execute", "agcad_scene"], names
    print("tools/list: ok", names)

    # --- batch modeling through agcad_execute ------------------------------
    batch = [
        {"method": "create_document", "name": "mcp-bracket"},
        {"method": "set_parameter", "params": {"doc": 1, "name": "wall", "value": 2.0}},
        {"method": "add_box", "params": {"doc": 1, "name": "plate",
            "center": [0, 0, "wall/2"], "size": ["wall*30", 40, "wall"], "color": [0.7, 0.7, 0.75, 1]}},
        {"method": "add_cylinder", "params": {"doc": 1, "name": "boss",
            "center": [10, 0, "wall + 5"], "radius": "wall*2.5", "height": 10}},
    ]
    send({"jsonrpc": "2.0", "id": 4, "method": "tools/call",
          "params": {"name": "agcad_execute", "arguments": {"commands": batch}}})
    call = recv()["result"]
    assert not call.get("isError", False), call
    payload = json.loads(call["content"][0]["text"])
    assert payload["executed"] == 4 and "stopped_early" not in payload
    plate = payload["results"][2]["result"]
    assert plate["result"] == "entity_created" and plate["entity"] == 1, plate
    assert plate["mesh"]["bbox"][1][2] == 1.0, plate  # z max = wall/2 (plate centered)
    assert "positions" not in json.dumps(plate), "meshes must be summarized, not shipped"
    print("agcad_execute: ok |", payload["executed"], "commands, meshes summarized")

    # --- scene inspection ---------------------------------------------------
    send({"jsonrpc": "2.0", "id": 5, "method": "tools/call",
          "params": {"name": "agcad_scene", "arguments": {"doc": 1}}})
    scene = json.loads(recv()["result"]["content"][0]["text"])
    assert len(scene["entities"]) == 2
    assert scene["parameters"][0]["name"] == "wall" and scene["parameters"][0]["value"] == 2.0
    print("agcad_scene: ok |", len(scene["entities"]), "entities, wall =", scene["parameters"][0]["value"])

    # --- parameter change through MCP --------------------------------------
    send({"jsonrpc": "2.0", "id": 6, "method": "tools/call",
          "params": {"name": "agcad_execute",
                     "arguments": {"commands": [{"method": "set_parameter",
                                                 "params": {"doc": 1, "name": "wall", "value": 4.0}}]}}})
    recv()
    send({"jsonrpc": "2.0", "id": 7, "method": "tools/call",
          "params": {"name": "agcad_scene", "arguments": {"doc": 1}}})
    scene = json.loads(recv()["result"]["content"][0]["text"])
    plate = next(e for e in scene["entities"] if e["name"] == "plate")
    assert plate["id"] == 1, plate
    assert plate["translation"][2] == 2.0 and plate["bbox"][1][2] == 2.0, plate  # wall/2, thickness 4.0
    print("parametric rebuild via MCP: ok | plate id", plate["id"], "thickness", plate["bbox"][1][2])

    # --- agent-shaped error --------------------------------------------------
    send({"jsonrpc": "2.0", "id": 8, "method": "tools/call",
          "params": {"name": "agcad_execute", "arguments": {"commands": [
              {"method": "add_box", "params": {"doc": 1, "name": "x", "center": [0, 0, 0],
                                               "size": ["walll", 1, 1]}}]}}})
    err_payload = json.loads(recv()["result"]["content"][0]["text"])
    assert err_payload["stopped_early"] and "available parameters" in err_payload["hint"] + json.dumps(err_payload["results"])
    print("error path: ok |", err_payload["results"][-1]["error"][:60], "...")

    print("MCP SMOKE OK")
finally:
    proc.stdin.close()
    proc.wait(timeout=10)
