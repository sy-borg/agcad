"""2D export: the same sketch as an editable DXF drawing and a cutter SVG.

Usage: .venv/bin/python examples/export_2d.py

A sketch is inherently 2D: its elements live in the sketch plane's (u, v)
frame and the entity transform only places that frame in 3D. `export_dxf` and
`export_svg` therefore keep the sketch-local millimeters instead of projecting
to world XYZ — a sketch on a vertical wall would otherwise collapse to a line.

Three selections are shown:
  * the whole drawing (lines, arcs, holes, construction guide),
  * only closed profile loops (`curves="profiles"`) — what a laser cutter wants,
  * a solid's source profile by naming the solid, not the sketch.

Profiles are the exact closed loops an extrude consumes, so the DXF profile and
the SVG outline describe the same plate the STL does.
"""

import os
import sys
from pathlib import Path

sys.path.insert(0, "python")  # noqa: E402

from wright import Wright  # noqa: E402


def main() -> None:
    os.makedirs("exports", exist_ok=True)
    drawing_dxf = os.path.abspath("exports/slot_plate_drawing.dxf")
    profile_dxf = os.path.abspath("exports/slot_plate_profile.dxf")
    outline_svg = os.path.abspath("exports/slot_plate_outline.svg")
    wcad = os.path.abspath("exports/slot_plate.wcad")

    cad = Wright.spawn()
    doc = cad.create_document("slot plate")["doc"]
    cad.set_parameter(doc, "length", 40.0, description="slot length [mm]")
    cad.set_parameter(doc, "radius", 5.0, description="slot end radius [mm]")
    cad.set_parameter(doc, "thickness", 4.0, description="plate thickness [mm]")

    # A slot: two straight sides closed by two semicircular ends.
    sketch = cad.add_sketch(doc, "slot")["entity"]
    cad.add_sketch_line(doc, sketch, ("length/2", "radius"), ("-length/2", "radius"))
    cad.add_sketch_arc(doc, sketch, ("-length/2", 0), "radius", 90, 270)
    cad.add_sketch_line(doc, sketch, ("-length/2", "-radius"), ("length/2", "-radius"))
    cad.add_sketch_arc(doc, sketch, ("length/2", 0), "radius", 270, 90)
    # Two mounting holes and a construction centreline (not part of the profile).
    cad.add_sketch_circle(doc, sketch, ("-length/4", 0), 2.5)
    cad.add_sketch_circle(doc, sketch, ("length/4", 0), 2.5)
    cad.add_sketch_line(doc, sketch, (-30, 0), (30, 0), construction=True)

    solid = cad.extrude(doc, sketch, "thickness", name="slot plate")["entity"]

    # 1. The full drawing: native DXF LINE/ARC/CIRCLE entities plus the
    #    construction guide on its own dashed layer.
    got = cad.export_dxf(doc, drawing_dxf, curves="elements")
    print(f"drawing DXF: {got['curves']} curves -> {got['path']} ({got['bytes']} bytes)")

    # 2. Profiles only: three closed polylines (slot + two holes).
    got = cad.export_dxf(doc, profile_dxf, curves="profiles")
    print(f"profile DXF: {got['curves']} closed loops -> {got['path']}")

    # 3. Name the solid instead of the sketch; the engine follows the source
    #    sketch and writes the same closed loops on a layer named after the part.
    got = cad.export_svg(doc, outline_svg, entities=[solid], curves="profiles")
    print(f"outline SVG: {got['curves']} loops from solid {solid} -> {got['path']}")

    trust = cad.validate(doc)
    print(f"validate: ok={trust['ok']}, volume={trust['totals']['volume']:.1f} mm3")

    text = Path(drawing_dxf).read_text(encoding="utf-8")
    print(
        "DXF layout:",
        f"{text.count(chr(10) + 'LINE' + chr(10))} LINE,",
        f"{text.count(chr(10) + 'ARC' + chr(10))} ARC,",
        f"{text.count(chr(10) + 'CIRCLE' + chr(10))} CIRCLE,",
        f"layers {sorted({line for line in text.splitlines() if line.startswith('slot_')})}",
    )
    svg = Path(outline_svg).read_text(encoding="utf-8")
    print(
        f"SVG: {svg.count('<path ')} closed paths, "
        f"viewBox {svg.split('viewBox=\"')[1].split('\"')[0]} mm"
    )

    cad.save_document(
        doc, wcad,
        source={"language": "python", "text": Path(__file__).read_text(encoding="utf-8")},
    )
    print(f"saved {wcad}")
    cad.close()


if __name__ == "__main__":
    main()
