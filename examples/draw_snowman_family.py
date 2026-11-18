#!/usr/bin/env python3
"""Draw a family of three snowmen — child, classic, tall — on one ice plate.

Every snowman is the same 13-solid program, parametrized by four knobs
(``<prefix>_ox`` position and three radii). Nothing is hard-coded: the hat
brim is ``head_r + 4``, the eyes sit at ``head_r*0.9``, the arms tilt
about Z. Change a knob and that snowman resculpts, entity ids stable.

Usage:
  .venv/bin/python examples/draw_snowman_family.py              # own server
  .venv/bin/python examples/draw_snowman_family.py --port 7860  # shared backend
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "python"))

from agcad import Agcad  # noqa: E402

SNOW = [0.95, 0.96, 0.98, 1.0]
DARK = [0.12, 0.12, 0.15, 1.0]
CARROT = [0.92, 0.5, 0.08, 1.0]
WOOD = [0.45, 0.3, 0.18, 1.0]

FAMILY = [
    # (prefix, x offset, base_r, mid_r, head_r, scarf color)
    ("kid",   -90, 20, 15, 10, [0.85, 0.18, 0.15, 1.0]),
    ("adult",   0, 30, 22, 15, [0.15, 0.55, 0.25, 1.0]),
    ("tall",   95, 33, 24, 16, [0.15, 0.35, 0.75, 1.0]),
]


def snowman_solid_params(p: str, ox, base, mid, head, scarf) -> list[dict]:
    """The 13 modeling commands of one snowman, doc-agnostic."""
    b, m, h, x = f"{p}_base_r", f"{p}_mid_r", f"{p}_head_r", f"{p}_ox"
    top = f"{b}*2 + {m}*2 + {h}"
    return [
        {"method": "add_sphere",   "name": f"{p}_body",      "center": [x, 0, b], "radius": b, "color": SNOW},
        {"method": "add_sphere",   "name": f"{p}_torso",     "center": [x, 0, f"{b}*2 + {m} - 10"], "radius": m, "color": SNOW},
        {"method": "add_sphere",   "name": f"{p}_head",      "center": [x, 0, f"{top} - 18"], "radius": h, "color": SNOW},
        {"method": "add_cylinder", "name": f"{p}_nose",      "center": [x, f"{h} - 2", f"{top} - 19"], "radius": 2.2, "height": 18, "color": CARROT},
        {"method": "add_sphere",   "name": f"{p}_eye_left",  "center": [f"{x} - 4.5", f"{h}*0.9", f"{top} - 14"], "radius": 1.8, "color": DARK},
        {"method": "add_sphere",   "name": f"{p}_eye_right", "center": [f"{x} + 4.5", f"{h}*0.9", f"{top} - 14"], "radius": 1.8, "color": DARK},
        {"method": "add_sphere",   "name": f"{p}_button_a",  "center": [x, f"{m}*0.94", f"{b}*2 + {m} - 18"], "radius": 1.6, "color": DARK},
        {"method": "add_sphere",   "name": f"{p}_button_b",  "center": [x, f"{m}*0.99", f"{b}*2 + {m} - 6"], "radius": 1.6, "color": DARK},
        {"method": "add_box",      "name": f"{p}_arm_left",  "center": [f"{x} - {m}*0.8 - 10", 0, f"{b}*2 + {m}"], "size": [24, 3, 3], "color": WOOD},
        {"method": "add_box",      "name": f"{p}_arm_right", "center": [f"{x} + {m}*0.8 + 10", 0, f"{b}*2 + {m}"], "size": [24, 3, 3], "color": WOOD},
        {"method": "add_box",      "name": f"{p}_scarf",     "center": [x, 0, f"{b}*2 + 2*{m} - 12"], "size": [16, f"{m}*1.6", 6], "color": scarf},
        {"method": "add_cylinder", "name": f"{p}_hat_brim",  "center": [x, 0, f"{top} - 19"], "radius": f"{h} + 4", "height": 2, "color": DARK},
        {"method": "add_cylinder", "name": f"{p}_hat_crown", "center": [x, 0, f"{top} - 12"], "radius": f"{h} - 4", "height": 12, "color": DARK},
    ]


def main() -> None:
    args = sys.argv[1:]
    port = int(args[args.index("--port") + 1]) if "--port" in args else None
    cad = Agcad.connect(port=port) if port is not None else Agcad.spawn()
    print(f"{'connected to server on :' + str(port) if port else 'spawned private server'}")

    doc = cad.create_document("snowman_family")["doc"]

    # One big ice plate for the whole family.
    cad.add_box(doc, name="ground", center=(0, 0, -1), size=(440, 240, 2), color=(0.85, 0.9, 0.96, 1.0))

    arm_ids = {}
    for p, ox, base, mid, head, scarf in FAMILY:
        cad.set_parameter(doc, f"{p}_ox", ox, description=f"{p} x position [mm]")
        cad.set_parameter(doc, f"{p}_base_r", base, description=f"{p} bottom ball radius [mm]")
        cad.set_parameter(doc, f"{p}_mid_r", mid, description=f"{p} middle ball radius [mm]")
        cad.set_parameter(doc, f"{p}_head_r", head, description=f"{p} head radius [mm]")
        for cmd in snowman_solid_params(p, ox, base, mid, head, scarf):
            method, params = cmd["method"], {k: v for k, v in cmd.items() if k != "method"}
            result = cad.call(method, {"doc": doc, **params})
            if "arm" in params["name"]:
                arm_ids[params["name"]] = result["entity"]
            if "hat" in params["name"]:
                # cylinders are Y-axis (nose convention) — stand hats up
                cad.transform(doc, result["entity"], rotation_deg=(90, 0, 0))

    # Tilt the arms ±20° about Z, from the recorded ids.
    for p, *_ in FAMILY:
        cad.transform(doc, arm_ids[f"{p}_arm_left"], rotation_deg=(0, 0, 20))
        cad.transform(doc, arm_ids[f"{p}_arm_right"], rotation_deg=(0, 0, -20))

    scene = cad.get_scene(doc)
    entities = scene["entities"]
    tris = sum(len(e["mesh"]["indices"]) // 3 for e in entities)
    print(f"doc {doc}: {len(entities)} entities, {tris} triangles")
    print("  ", ", ".join(e["name"] for e in entities if "eye" in e["name"] or e["name"] == "ground"))
    if port is not None:
        print("the viewer should refresh itself; press R if it doesn't")
    cad.close()


if __name__ == "__main__":
    main()
