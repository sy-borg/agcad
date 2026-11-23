#!/usr/bin/env python3
"""Parametric wright: the agent authors the function, the user tunes it.

The document keeps a program (every modeling command, with dimensions as
expressions over named parameters). Changing a parameter replays the
program: identical entity ids, regenerated geometry. Undo restores
parameters + program + geometry atomically.

In the intended workflow the *agent* writes this program from a natural
language description (or a picture), and the user only turns knobs.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "python"))

from wright import Wright  # noqa: E402

STEEL = (0.75, 0.76, 0.80, 1.0)
RED = (0.85, 0.25, 0.20, 1.0)


def bbox_z(mesh: dict) -> float:
    zs = [p[2] for p in mesh["positions"]]
    return max(zs) - min(zs)


def main() -> None:
    with Wright.spawn() as cad:
        cad.initialize()
        doc = cad.create_document("parametric-bracket")["doc"]

        # --- the agent authors the parametric program ----------------------
        cad.set_parameter(doc, "wall", 2.0, description="plate thickness [mm]")
        cad.set_parameter(doc, "plate_len", 80.0, description="base plate length [mm]")
        cad.set_parameter(doc, "boss_r", 8.0, description="boss radius [mm]")

        base = cad.add_box(
            doc, name="base",
            center=(0, 0, "wall/2"),
            size=("plate_len", 40, "wall"),
            color=STEEL,
        )
        boss = cad.add_cylinder(
            doc, name="boss",
            center=("plate_len*0.25", 0, "wall + 7"),
            radius="boss_r", height=14,
            color=STEEL,
        )
        pin = cad.add_sphere(
            doc, name="pin",
            center=("-plate_len*0.3", 0, "wall + 5"),
            radius="boss_r*0.75",
            color=RED,
        )

        print(f"program: base(base_id={base['entity']}), boss(id={boss['entity']}), pin(id={pin['entity']})")
        print(f"wall=2.0  -> base thickness {bbox_z(base['mesh']):.1f} mm")

        # --- the user (or agent) turns a knob; ids are stable ---------------
        cad.set_parameter(doc, "wall", 4.0)
        scene = cad.get_scene(doc)
        base_now = next(e for e in scene["entities"] if e["name"] == "base")
        assert base_now["id"] == base["entity"], "ids must survive parameter changes"
        print(f"wall=4.0  -> base thickness {bbox_z(base_now['mesh']):.1f} mm (same id {base_now['id']})")

        # Derived parameters work too: boss_r becomes a fraction of the plate.
        cad.set_parameter(doc, "boss_r", "plate_len*0.125")
        scene = cad.get_scene(doc)
        boss_now = next(e for e in scene["entities"] if e["name"] == "boss")
        xs = [p[0] for p in boss_now["mesh"]["positions"]]
        assert max(xs) - min(xs) > 18.0, "boss radius must track plate_len*0.125"

        params = cad.get_parameters(doc)
        print("parameters:", [(p["name"], p["value"]) for p in params])

        # Error ergonomics: a typo lists the valid parameter names.
        try:
            cad.add_box(doc, name="oops", center=(0, 0, 0), size=("plate_lenn", 1, 1))
        except Exception as err:
            print("error as designed:", err)

        print("PARAMETRIC OK")


if __name__ == "__main__":
    main()
