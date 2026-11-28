#!/usr/bin/env python3
"""Live-preview demo: attach to the viewer's engine and rebuild in place.

Start the shared backend and viewer first:

    ./target/debug/wright --tcp 7860
    ./target/debug/wright-view --port 7860

Then run this script. `Wright.open()` finds the running server (explicit
endpoint, WRIGHT_CONNECT, or the project session descriptor), and
`cad.document("live_demo", reset=True)` clears/reuses the same document id so
the viewer follows the rebuild without a Tab press.

Force an isolated run with `--private` or `WRIGHT_PRIVATE=1`.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "python"))

from wright import Wright  # noqa: E402

STEEL = (0.72, 0.75, 0.80, 1.0)
RED = (0.85, 0.25, 0.20, 1.0)


def main() -> None:
    private = True if "--private" in sys.argv[1:] else None
    cad = Wright.open(private=private)
    print(f"engine: {'shared (viewer attached)' if cad.shared else 'private'}")

    doc = cad.document("live_demo", reset=True)
    print(f"document: #{doc} live_demo")

    # The script owns the program; every dimension is a knob.
    for name, value, desc in [
        ("plate_len", 90.0, "base length [mm]"),
        ("plate_w", 46.0, "base width [mm]"),
        ("wall", 5.0, "base thickness [mm]"),
        ("boss_r", 11.0, "boss radius [mm]"),
        ("boss_h", 16.0, "boss height [mm]"),
        ("hole_d", 6.5, "mounting hole diameter [mm]"),
        ("bore_d", 11.0, "boss bore diameter [mm]"),
        ("mount_span", 60.0, "mounting hole spacing [mm]"),
    ]:
        cad.set_parameter(doc, name, value, description=desc)

    # Base plate with a soft top edge.
    base = cad.add_box(
        doc,
        name="base",
        center=(0, 0, "wall/2"),
        size=("plate_len", "plate_w", "wall"),
        color=STEEL,
    )["entity"]
    cad.chamfer(doc, base, 1.2, edges={"normals": [[0, 0, 1]]})

    # Central boss, fused to the plate.
    boss = cad.add_cylinder(
        doc,
        name="boss",
        center=(0, 0, "wall + boss_h/2"),
        radius="boss_r",
        height="boss_h",
        color=STEEL,
    )["entity"]
    body = cad.boolean(doc, "union", [base, boss], name="body", color=STEEL)["entity"]
    cad.delete(doc, base)
    cad.delete(doc, boss)

    # Two mounting holes and one bore, cut in two booleans.
    cutters = []
    for side, x in (("left", "-mount_span/2"), ("right", "mount_span/2")):
        cutters.append(
            cad.add_cylinder(
                doc,
                name=f"mount_{side}",
                center=(x, 0, "wall/2"),
                radius="hole_d/2",
                height="wall*4",
            )["entity"]
        )
    old = body
    body = cad.boolean(doc, "difference", [old, *cutters], name="body", color=STEEL)["entity"]
    cad.delete(doc, old)
    for cutter in cutters:
        cad.delete(doc, cutter)

    bore = cad.add_cylinder(
        doc,
        name="bore",
        center=(0, 0, "wall + boss_h/2"),
        radius="bore_d/2",
        height="boss_h*2",
    )["entity"]
    old = body
    body = cad.boolean(doc, "difference", [old, bore], name="body", color=STEEL)["entity"]
    cad.delete(doc, old)
    cad.delete(doc, bore)

    # Mounting holes sit outside the boss; the bore goes through its centre.
    # A separate red pin makes the live update obvious.
    cad.add_sphere(
        doc,
        name="pin",
        center=(0, "-plate_w*0.32", "wall + 4.0"),
        radius=4.5,
        color=RED,
    )

    report = cad.validate(doc, entities=[body])
    solid = report["solids"][0]
    print(
        f"body: {solid['tris']} tris, volume {solid['volume']:.0f} mm3, "
        f"watertight={solid['watertight']}"
    )
    print(f"entities: {[e['name'] for e in cad.get_scene(doc, meshes=False)['entities']]}")
    if cad.shared:
        print("viewer: the same document should now show the rebuilt bracket")
    cad.close()


if __name__ == "__main__":
    main()
