"""Sketch completeness (M17): arcs, splines, the wider constraint set,
construction geometry, trim/extend/offset and conflict diagnostics.

Usage: .venv/bin/python examples/sketch_complete.py [out.png]

The part is a parametric rounded plate:

* four lines and four tangent arcs form a rounded rectangle, driven by
  ``width``/``height``/``corner``;
* a closed spline becomes a kidney-shaped hole (nested loops are holes);
* a construction diagonal guides the drawing but is excluded from the solid;
* the example deliberately over-constrains the sketch, prints the diagnostic
  that names the redundant equation, then removes it;
* a second sketch demonstrates trim, extend and a mitered offset.

Everything is recorded in the program, so changing a parameter re-solves the
sketch and rebuilds the extrusion.
"""

import base64
import sys

sys.path.insert(0, "python")  # noqa: E402

from wright import Wright  # noqa: E402


def pt(element: int, point: int) -> dict:
    return {"element": element, "point": point}


def whole(element: int) -> dict:
    return {"element": element}


def main() -> None:
    out = sys.argv[1] if len(sys.argv) > 1 else "exports/sketch_complete.png"

    cad = Wright.spawn()
    doc = cad.create_document("rounded plate")["doc"]
    for name, value, desc in [
        ("width", 64.0, "overall width"),
        ("height", 40.0, "overall height"),
        ("corner", 10.0, "corner radius"),
        ("thickness", 5.0, "plate thickness"),
        ("hole_w", 24.0, "kidney hole width"),
        ("hole_h", 12.0, "kidney hole height"),
    ]:
        cad.set_parameter(doc, name, value, desc)

    sketch = cad.add_sketch(doc, "outline")["entity"]

    # --- rounded rectangle from four lines and four tangent arcs -----------
    bottom = cad.add_sketch_line(doc, sketch, ("corner", 0.0), ("width-corner", 0.0))["element"]
    br = cad.add_sketch_arc(
        doc, sketch, ("width-corner", "corner"), "corner", -90.0, 0.0
    )["element"]
    right = cad.add_sketch_line(doc, sketch, ("width", "corner"), ("width", "height-corner"))["element"]
    tr = cad.add_sketch_arc(
        doc, sketch, ("width-corner", "height-corner"), "corner", 0.0, 90.0
    )["element"]
    top = cad.add_sketch_line(doc, sketch, ("width-corner", "height"), ("corner", "height"))["element"]
    tl = cad.add_sketch_arc(doc, sketch, ("corner", "height-corner"), "corner", 90.0, 180.0)["element"]
    left = cad.add_sketch_line(doc, sketch, (0.0, "height-corner"), (0.0, "corner"))["element"]
    bl = cad.add_sketch_arc(doc, sketch, ("corner", "corner"), "corner", 180.0, 270.0)["element"]

    # The endpoints are placed by expressions (corner, width-corner, …), so
    # the outline stays closed when a parameter moves; the constraints below
    # encode the *shape* rules: square sides, matching corners and tangency.
    for line, arc in [(bottom, br), (right, tr), (top, tl), (left, bl)]:
        cad.constrain(doc, sketch, "tangent", [whole(line), whole(arc)])
    cad.constrain(doc, sketch, "equal", [whole(br), whole(tr)])
    cad.constrain(doc, sketch, "equal", [whole(tr), whole(tl)])
    cad.constrain(doc, sketch, "equal", [whole(tl), whole(bl)])

    cad.constrain(doc, sketch, "horizontal", [whole(bottom)])
    cad.constrain(doc, sketch, "horizontal", [whole(top)])
    cad.constrain(doc, sketch, "vertical", [whole(right)])
    cad.constrain(doc, sketch, "vertical", [whole(left)])
    cad.constrain(doc, sketch, "distance", [pt(bottom, 0), pt(bottom, 1)], "width-2*corner")
    cad.constrain(doc, sketch, "distance", [pt(right, 0), pt(right, 1)], "height-2*corner")
    cad.constrain(doc, sketch, "radius", [whole(br)], "corner")

    # A construction diagonal is drawn and solved but never extruded.
    cad.add_sketch_line(doc, sketch, (0.0, 0.0), ("width", "height"), construction=True)

    # --- a closed spline hole ----------------------------------------------
    cx, cy = "width/2", "height/2"
    kidney = cad.add_sketch_spline(
        doc,
        sketch,
        [
            (f"{cx}-hole_w/2", f"{cy}-hole_h/2"),
            (f"{cx}-hole_w/6", f"{cy}-hole_h"),
            (f"{cx}+hole_w/6", f"{cy}-hole_h"),
            (f"{cx}+hole_w/2", f"{cy}-hole_h/2"),
            (f"{cx}+hole_w/6", f"{cy}+hole_h"),
            (f"{cx}-hole_w/6", f"{cy}+hole_h"),
        ],
        closed=True,
    )["element"]
    print(f"spline hole {kidney}")

    # --- demonstrate conflict diagnostics ----------------------------------
    duplicate = cad.constrain(
        doc, sketch, "distance", [pt(bottom, 0), pt(bottom, 1)], "width-2*corner"
    )
    print(
        f"duplicate constraint {duplicate['constraint']}: "
        f"over_constrained={duplicate['over_constrained']}"
    )
    for line in duplicate["diagnostics"]:
        print("  diagnostic:", line)
    cad.remove_constraint(doc, sketch, duplicate["constraint"])

    snap = next(e for e in cad.get_scene(doc)["entities"] if e["id"] == sketch)["sketch"]
    print(
        f"outline: {len(snap['elements'])} elements, {len(snap['constraints'])} constraints, "
        f"solved={snap['solved']} dof={snap['dof']} over={snap['over_constrained']}"
    )
    assert snap["solved"], snap["diagnostics"]

    plate = cad.extrude(doc, sketch, "thickness", name="plate")
    print(f"plate volume {plate['volume']:.1f} mm³, {plate['tris']} triangles")

    # --- trim / extend / offset on a second document -----------------------
    doodles = cad.create_document("edit demos")["doc"]
    edit = cad.add_sketch(doodles, "trim + extend")["entity"]
    blade = cad.add_sketch_line(doodles, edit, (0.0, 0.0), (30.0, 0.0))["element"]
    cad.add_sketch_line(doodles, edit, (10.0, -8.0), (10.0, 8.0))
    cad.add_sketch_line(doodles, edit, (20.0, -8.0), (20.0, 8.0))
    trimmed = cad.trim_sketch(doodles, edit, blade, (5.0, 0.0))
    print(f"trim: created={trimmed['created']} removed={trimmed['removed']} modified={trimmed['modified']}")

    stem = cad.add_sketch_line(doodles, edit, (30.0, 0.0), (34.0, 0.0))["element"]
    cad.add_sketch_line(doodles, edit, (38.0, -8.0), (38.0, 8.0))
    extended = cad.extend_sketch(doodles, edit, stem, 1)
    print(f"extend: modified={extended['modified']}")

    box = cad.add_sketch(doodles, "offset demo")["entity"]
    box_lines = [
        cad.add_sketch_line(doodles, box, p1, p2)["element"]
        for p1, p2 in [
            ((0.0, 0.0), (16.0, 0.0)),
            ((16.0, 0.0), (16.0, 10.0)),
            ((16.0, 10.0), (0.0, 10.0)),
            ((0.0, 10.0), (0.0, 0.0)),
        ]
    ]
    offset = cad.offset_sketch(doodles, box, 2.0)
    print(f"offset: {len(offset['created'])} mitered lines from {len(box_lines)}")

    report = cad.validate(doc)
    assert report["ok"], report.get("warnings")
    print(f"validate: ok={report['ok']} solids={len(report['solids'])} sketches={len(report['sketches'])}")

    r = cad.render_offscreen(doc, width=1000, height=700, path=out)
    png = base64.b64decode(r["png_base64"])
    assert png[:8] == bytes([0x89, 0x50, 0x4E, 0x47, 0x0D, 0x0A, 0x1A, 0x0A])
    print(f"rendered -> {r['path']} ({len(png)} bytes)")

    # Parametric proof: widen the plate and watch the profile and solid follow.
    before = report["totals"]["volume"]
    cad.set_parameter(doc, "width", 84.0)
    after = cad.validate(doc)
    print(f"width 64 -> 84: plate volume {before:.1f} -> {after['totals']['volume']:.1f} mm³")

    cad.close()


if __name__ == "__main__":
    main()
