"""M16 shape leverage: patterns, mirror, shell, holes, sweep, loft, thread.

One document, several printable demonstrations laid out side by side:

* a shelled tray (open top) whose wall is a knob,
* a plate with a linear pattern of through-holes,
* a flange with a circular pattern of bolt holes,
* a pair of ribs made with `mirror`,
* a helical sweep (a spring),
* a square-to-circle loft,
* a real M8 screw thread.

Every feature is a normal program step, so the script is still the source and
`set_parameter` replays the lot. The script renders the scene, validates it and
saves a `.wcad` with its own source embedded.

Usage: .venv/bin/python examples/shape_leverage.py
"""

import os
import sys

sys.path.insert(0, "python")  # noqa: E402

from wright import Wright  # noqa: E402

OUT = "exports"


def main() -> None:
    os.makedirs(OUT, exist_ok=True)
    with Wright.spawn() as cad:
        cad.initialize()
        doc = cad.create_document("shape_leverage")["doc"]

        cad.set_parameter(doc, "wall", 3.0, "tray wall [mm]")
        cad.set_parameter(doc, "hole_d", 6.0, "bolt clearance diameter [mm]")
        cad.set_parameter(doc, "bolt_r", 18.0, "bolt circle radius [mm]")
        cad.set_parameter(doc, "pitch", 1.25, "thread pitch [mm]")

        # --- a shelled tray with an open top -----------------------------
        tray = cad.add_box(doc, "tray", center=(-70, 0, 12.5), size=(70, 50, 25))
        cad.shell(doc, tray["entity"], "wall", open=[(0, 0, 1)])

        # --- plate: linear pattern of through-holes ----------------------
        plate = cad.add_box(doc, "plate", center=(70, 0, 8), size=(70, 40, 8))
        plate_src = plate["entity"]
        cutter = cad.add_cylinder(
            doc, "hole", center=(52, 0, 8), radius=("hole_d/2"), height=30
        )
        holes = cad.linear_pattern(
            doc, [cutter["entity"]], direction=(1, 0, 0), spacing=12, count=4,
            name="hole",
        )
        plate = cad.boolean(
            doc, "difference", [plate_src, cutter["entity"], *holes["entities"]],
            name="plate",
        )
        for eid in [plate_src, cutter["entity"]]:
            cad.delete(doc, eid)

        # --- flange: circular pattern of bolt holes ----------------------
        flange = cad.add_cylinder(doc, "flange", center=(0, 80, 4), radius=28, height=8)
        bolt = cad.add_cylinder(
            doc, "bolt_hole", center=("bolt_r", 80, 4), radius=("hole_d/2"), height=30
        )
        bolts = cad.circular_pattern(
            doc, [bolt["entity"]], count=6, center=(0, 80, 0), angle_deg=360,
            name="bolt_hole",
        )
        cad.boolean(
            doc, "difference",
            [flange["entity"], bolt["entity"], *bolts["entities"]],
            name="flange",
        )
        for eid in [flange["entity"], bolt["entity"]]:
            cad.delete(doc, eid)

        # --- mirror: two ribs across the part's centreline ---------------
        rib = cad.add_box(doc, "rib", center=(70, 22, 8), size=(40, 4, 8))
        other = cad.mirror(doc, [rib["entity"]], plane="xz", name="rib")
        merged = cad.boolean(
            doc, "union", [plate["entity"], rib["entity"], other["entities"][0]],
            name="plate",
        )
        cad.delete(doc, plate["entity"])
        cad.delete(doc, rib["entity"])
        plate = merged

        # --- sweep: a helical spring -------------------------------------
        wire = cad.add_sketch(doc, "wire")
        cad.add_sketch_circle(doc, wire["entity"], center=(0, 0), radius=1.5)
        cad.sweep(
            doc, wire["entity"],
            path={
                "origin": (0, -80, 0),
                "radius": 15,
                "pitch": 10,
                "turns": 2.5,
                "segments_per_turn": 16,
            },
            name="spring",
        )

        # --- loft: square to circle --------------------------------------
        lower = cad.add_sketch(doc, "square")
        for p1, p2 in [
            ((70, -90), (90, -90)),
            ((90, -90), (90, -70)),
            ((90, -70), (70, -70)),
            ((70, -70), (70, -90)),
        ]:
            cad.add_sketch_line(doc, lower["entity"], p1=p1, p2=p2)
        upper = cad.add_sketch(
            doc, "circle", plane={"origin": (80, -80, 30), "normal": (0, 0, 1)}
        )
        cad.add_sketch_circle(doc, upper["entity"], center=(0, 0), radius=6)
        cad.loft(doc, [lower["entity"], upper["entity"]], segments=32, name="loft")

        # --- thread: an M8 stud ------------------------------------------
        cad.thread(
            doc, major_diameter=8.0, pitch="pitch", length=12.0,
            segments_per_turn=32, origin=(0, -140, 0), name="m8_stud",
        )

        # --- prove and see it --------------------------------------------
        report = cad.validate(doc, min_wall_mm=0.1)
        print(
            f"validate: ok={report['ok']} solids={report['totals']['solids']} "
            f"tris={report['totals']['tris']} "
            f"volume={report['totals']['volume']:.1f} mm^3"
        )
        for solid in report["solids"]:
            print(
                f"  {solid['entity']:>2} {solid['name']:<10} {solid['kind']:<9} "
                f"volume {solid['volume']:>9.2f}  tris {solid['tris']:>5}  "
                f"{'ok' if solid['ok'] else 'FAIL'}"
            )

        cad.render_offscreen(
            doc, width=1200, height=700, path=os.path.join(OUT, "shape_leverage.png")
        )
        cad.save_document(
            doc,
            os.path.join(OUT, "shape_leverage.wcad"),
            source={"language": "python", "text": open(__file__, encoding="utf-8").read()},
        )
        print(f"wrote {OUT}/shape_leverage.png and {OUT}/shape_leverage.wcad")


if __name__ == "__main__":
    main()
