"""A parametric bracket profile drawn with the sketch/constraint layer.

Usage: .venv/bin/python examples/sketch_bracket.py [out.png]

Spawns its own engine, builds a rectangle from four *messy* lines, and lets
the constraint solver square it up. Every dimension is an expression over
parameters, so changing `width`, `height` or `hole` re-solves the sketch on
replay — the drawing is parametric, not baked.

Then it renders the sketch with the headless rasterizer so you can see the
solved result.
"""

import base64
import sys

sys.path.insert(0, "python")  # noqa: E402

from agcad import Agcad  # noqa: E402


def main() -> None:
    out = sys.argv[1] if len(sys.argv) > 1 else "exports/sketch_bracket.png"

    cad = Agcad.spawn()
    doc = cad.create_document("bracket profile")["doc"]

    for name, value, desc in [
        ("width", 80.0, "overall width"),
        ("height", 50.0, "overall height"),
        ("hole", 4.0, "mounting hole diameter"),
        ("margin", 10.0, "hole inset from the left/bottom edges"),
    ]:
        cad.set_parameter(doc, name, value, desc)

    sketch = cad.add_sketch(doc, "profile")["entity"]

    # Deliberately sloppy corners: the solver has to clean them up.
    lines = [
        cad.add_sketch_line(doc, sketch, (0.0, 0.0), ("width", 3.0))["element"],
        cad.add_sketch_line(doc, sketch, ("width", 3.0), ("width+3", "height"))["element"],
        cad.add_sketch_line(doc, sketch, ("width+3", "height"), (-2.0, "height-2"))["element"],
        cad.add_sketch_line(doc, sketch, (-2.0, "height-2"), (0.0, 0.0))["element"],
    ]
    bottom, right, top, left = lines

    def pt(element: int, point: int) -> dict:
        return {"element": element, "point": point}

    def whole(element: int) -> dict:
        return {"element": element}

    # Close the loop.
    for a, b in [(bottom, right), (right, top), (top, left), (left, bottom)]:
        cad.constrain(doc, sketch, "coincident", [pt(a, 1), pt(b, 0)])
    # Square it up.
    cad.constrain(doc, sketch, "horizontal", [whole(bottom)])
    cad.constrain(doc, sketch, "horizontal", [whole(top)])
    cad.constrain(doc, sketch, "vertical", [whole(right)])
    cad.constrain(doc, sketch, "vertical", [whole(left)])
    # Drive the size from parameters.
    w = cad.constrain(doc, sketch, "distance", [pt(bottom, 0), pt(bottom, 1)], "width")
    h = cad.constrain(doc, sketch, "distance", [pt(right, 0), pt(right, 1)], "height")
    print(f"width  constraint -> solved={w['solved']} dof={w['dof']} residual={w['residual']:.2e}")
    print(f"height constraint -> solved={h['solved']} dof={h['dof']} residual={h['residual']:.2e}")

    # Two mounting holes, positioned by expressions and sized by a
    # diameter constraint.
    for cx, cy in [("margin", "margin"), ("width-margin", "height-margin")]:
        hole = cad.add_sketch_circle(doc, sketch, (cx, cy), "hole/2")["element"]
        r = cad.constrain(doc, sketch, "diameter", [whole(hole)], "hole")
        print(f"hole   constraint -> solved={r['solved']} residual={r['residual']:.2e}")

    scene = cad.get_scene(doc)
    snap = next(e for e in scene["entities"] if e["id"] == sketch)["sketch"]
    print(f"sketch: {len(snap['elements'])} elements, {len(snap['constraints'])} constraints, "
          f"solved={snap['solved']} dof={snap['dof']}")

    r = cad.render_offscreen(doc, width=900, height=650, path=out)
    assert r["result"] == "rendered", r
    png = base64.b64decode(r["png_base64"])
    assert png[:8] == bytes([0x89, 0x50, 0x4E, 0x47, 0x0D, 0x0A, 0x1A, 0x0A])
    print(f"rendered -> {r['path']} ({len(png)} bytes)")

    # Parametric proof: change width and watch the sketch re-solve.
    before = next(p for p in snap["elements"] if p["type"] == "line" and p["id"] == bottom)
    cad.set_parameter(doc, "width", 120.0)
    after_snap = next(e for e in cad.get_scene(doc)["entities"] if e["id"] == sketch)["sketch"]
    after = next(p for p in after_snap["elements"] if p["type"] == "line" and p["id"] == bottom)
    print(f"width 80 -> bottom line {before['p1']}..{before['p2']}")
    print(f"width 120 -> bottom line {after['p1']}..{after['p2']}")

    cad.close()


if __name__ == "__main__":
    main()
