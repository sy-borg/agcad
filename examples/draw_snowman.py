#!/usr/bin/env python3
"""Draw the agcad snowman — the same parametric program an MCP agent runs,
replayed against a running server so the viewer can show it.

Usage:
  .venv/bin/python examples/draw_snowman.py            # spawn own server
  .venv/bin/python examples/draw_snowman.py --port 7860  # your backend
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "python"))

from agcad import Agcad  # noqa: E402

SNOW = [0.95, 0.96, 0.98, 1.0]
DARK = [0.12, 0.12, 0.15, 1.0]
CARROT = [0.92, 0.5, 0.08, 1.0]
RED = [0.85, 0.18, 0.15, 1.0]
WOOD = [0.45, 0.3, 0.18, 1.0]

# The program — one batch, exactly as an agent would author it. Dimensions
# are expressions over three parameters; change them and the snowman
# resculpts itself with identical entity ids.
COMMANDS = [
    {"method": "set_parameter", "name": "base_r", "value": 30.0, "description": "bottom snowball radius [mm]"},
    {"method": "set_parameter", "name": "mid_r", "value": 22.0, "description": "middle snowball radius [mm]"},
    {"method": "set_parameter", "name": "head_r", "value": 15.0, "description": "head radius [mm]"},
    {"method": "add_box", "name": "ground", "center": [0, 0, -1], "size": ["base_r*5", "base_r*5", 2], "color": [0.85, 0.9, 0.96, 1.0]},
    {"method": "add_sphere", "name": "body", "center": [0, 0, "base_r"], "radius": "base_r", "color": SNOW},
    {"method": "add_sphere", "name": "torso", "center": [0, 0, "base_r*2 + mid_r - 10"], "radius": "mid_r", "color": SNOW},
    {"method": "add_sphere", "name": "head", "center": [0, 0, "base_r*2 + mid_r*2 + head_r - 18"], "radius": "head_r", "color": SNOW},
    {"method": "add_cylinder", "name": "nose", "center": [0, "head_r - 2", "base_r*2 + mid_r*2 + head_r - 19"], "radius": 2.2, "height": 18, "color": CARROT},
    {"method": "add_sphere", "name": "eye_left", "center": [-4.5, "head_r*0.9", "base_r*2 + mid_r*2 + head_r - 14"], "radius": 1.8, "color": DARK},
    {"method": "add_sphere", "name": "eye_right", "center": [4.5, "head_r*0.9", "base_r*2 + mid_r*2 + head_r - 14"], "radius": 1.8, "color": DARK},
    {"method": "add_sphere", "name": "button_a", "center": [0, "mid_r*0.94", "base_r*2 + mid_r - 18"], "radius": 1.6, "color": DARK},
    {"method": "add_sphere", "name": "button_b", "center": [0, "mid_r*0.99", "base_r*2 + mid_r - 6"], "radius": 1.6, "color": DARK},
    {"method": "add_box", "name": "arm_left", "center": ["-mid_r*0.8 - 10", 0, "base_r*2 + mid_r"], "size": [24, 3, 3], "color": WOOD},
    {"method": "add_box", "name": "arm_right", "center": ["mid_r*0.8 + 10", 0, "base_r*2 + mid_r"], "size": [24, 3, 3], "color": WOOD},
    {"method": "add_box", "name": "scarf", "center": [0, 0, "base_r*2 + 2*mid_r - 12"], "size": [16, "mid_r*1.6", 6], "color": RED},
    {"method": "add_cylinder", "name": "hat_brim", "center": [0, 0, "base_r*2 + mid_r*2 + 2*head_r - 19"], "radius": "head_r + 4", "height": 2, "color": DARK},
    {"method": "add_cylinder", "name": "hat_crown", "center": [0, 0, "base_r*2 + mid_r*2 + 2*head_r - 12"], "radius": "head_r - 4", "height": 12, "color": DARK},
    {"method": "transform", "entity": 10, "rotation_deg": [0, 0, 20]},
    {"method": "transform", "entity": 11, "rotation_deg": [0, 0, -20]},
    # The nose is a +Z cylinder; tilt it forward along +Y.
    {"method": "transform", "entity": 5, "rotation_deg": [-90, 0, 0]},
]


def build(cad: Agcad, doc: int) -> None:
    for cmd in COMMANDS:
        method = cmd["method"]
        params = {k: v for k, v in cmd.items() if k != "method"}
        if method != "create_document":
            params["doc"] = doc
        cad.call(method, params)


def main() -> None:
    port = None
    args = sys.argv[1:]
    if "--port" in args:
        port = int(args[args.index("--port") + 1])

    if port is not None:
        cad = Agcad.connect(port=port)
        print(f"connected to server on :{port}")
    else:
        cad = Agcad.spawn()

    doc = cad.create_document("snowman")["doc"]
    build(cad, doc)
    scene = cad.get_scene(doc)
    names = [e["name"] for e in scene["entities"]]
    print(f"doc {doc}: {len(names)} entities -> {', '.join(names)}")
    if port is not None:
        print("press R in the viewer to refresh")
    cad.close()


if __name__ == "__main__":
    main()
