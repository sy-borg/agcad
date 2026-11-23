"""A printable bracket: constrained sketch profile → extruded solid → STL + 3MF.

Usage: .venv/bin/python examples/printable_bracket.py [out.png]

The whole chain is parametric: the rectangle is driven by `width`/`height`
distance constraints, the holes by a `diameter` constraint, and the
extrusion height by `thickness`. Change a parameter and the sketch re-solves,
the solid rebuilds and the STL re-exports.

This is the loop wright exists for: agent authors the program, user turns
the knobs, the slicer gets a watertight part.
"""

import base64
import os
import sys

sys.path.insert(0, "python")  # noqa: E402

from wright import Wright  # noqa: E402


def z_extent(mesh: dict) -> float:
    zs = [p[2] for p in mesh["positions"]]
    return max(zs) - min(zs)


def main() -> None:
    out = sys.argv[1] if len(sys.argv) > 1 else "exports/bracket.png"
    stl = os.path.abspath("exports/bracket.stl")
    mf = os.path.abspath("exports/bracket.3mf")
    os.makedirs(os.path.dirname(stl), exist_ok=True)

    cad = Wright.spawn()
    doc = cad.create_document("bracket")["doc"]

    for name, value, desc in [
        ("width", 80.0, "overall width"),
        ("height", 50.0, "overall height"),
        ("thickness", 6.0, "plate thickness"),
        ("hole", 4.0, "mounting hole diameter"),
        ("margin", 12.0, "hole inset from the edges"),
    ]:
        cad.set_parameter(doc, name, value, desc)

    sketch = cad.add_sketch(doc, "profile")["entity"]

    # Four sloppy lines, then let the solver square them up.
    ids = [
        cad.add_sketch_line(doc, sketch, (0.0, 0.0), ("width", 2.0))["element"],
        cad.add_sketch_line(doc, sketch, ("width", 2.0), ("width+2", "height"))["element"],
        cad.add_sketch_line(doc, sketch, ("width+2", "height"), (-2.0, "height-2"))["element"],
        cad.add_sketch_line(doc, sketch, (-2.0, "height-2"), (0.0, 0.0))["element"],
    ]
    bottom, right, top, left = ids

    def pt(element: int, point: int) -> dict:
        return {"element": element, "point": point}

    def whole(element: int) -> dict:
        return {"element": element}

    for a, b in [(bottom, right), (right, top), (top, left), (left, bottom)]:
        cad.constrain(doc, sketch, "coincident", [pt(a, 1), pt(b, 0)])
    cad.constrain(doc, sketch, "horizontal", [whole(bottom)])
    cad.constrain(doc, sketch, "horizontal", [whole(top)])
    cad.constrain(doc, sketch, "vertical", [whole(right)])
    cad.constrain(doc, sketch, "vertical", [whole(left)])
    cad.constrain(doc, sketch, "distance", [pt(bottom, 0), pt(bottom, 1)], "width")
    cad.constrain(doc, sketch, "distance", [pt(right, 0), pt(right, 1)], "height")

    # Two through-holes: circles sized by a diameter constraint, placed by
    # parameter expressions (nested loops become holes in the solid).
    for cx, cy in [("margin", "margin"), ("width-margin", "height-margin")]:
        hole = cad.add_sketch_circle(doc, sketch, (cx, cy), "hole/2")["element"]
        cad.constrain(doc, sketch, "diameter", [whole(hole)], "hole")

    snap = next(e for e in cad.get_scene(doc)["entities"] if e["id"] == sketch)["sketch"]
    print(f"sketch: {len(snap['elements'])} elements, {len(snap['constraints'])} constraints, "
          f"solved={snap['solved']} dof={snap['dof']}")

    solid = cad.extrude(doc, sketch, "thickness", name="bracket")
    print(f"extrude: entity {solid['entity']}, volume {solid['volume']:.1f} mm³, {solid['tris']} tris")

    exp = cad.export_stl(doc, stl)
    print(f"stl: {exp['path']} ({exp['tris']} tris, {exp['bytes']} bytes)")

    exp = cad.export_3mf(doc, mf)
    print(f"3mf: {exp['path']} ({exp['objects']} objects, {exp['tris']} tris, {exp['bytes']} bytes)")

    r = cad.render_offscreen(doc, width=900, height=650, path=out)
    png = base64.b64decode(r["png_base64"])
    assert png[:8] == bytes([0x89, 0x50, 0x4E, 0x47, 0x0D, 0x0A, 0x1A, 0x0A])
    print(f"rendered -> {r['path']} ({len(png)} bytes)")

    # Turn a knob: the solid rebuilds from the re-solved sketch and the STL
    # is rewritten from the new geometry.
    cad.set_parameter(doc, "thickness", 10.0)
    cad.set_parameter(doc, "width", 100.0)
    scene = cad.get_scene(doc)
    bracket = next(e for e in scene["entities"] if e["id"] == solid["entity"])
    print(f"thickness 6 -> 10: solid z extent now {z_extent(bracket['mesh']):.2f} mm")
    exp = cad.export_stl(doc, stl)
    print(f"stl re-exported: {exp['tris']} tris, {exp['bytes']} bytes")
    exp = cad.export_3mf(doc, mf)
    print(f"3mf re-exported: {exp['objects']} objects, {exp['tris']} tris, {exp['bytes']} bytes")

    cad.close()


if __name__ == "__main__":
    main()
