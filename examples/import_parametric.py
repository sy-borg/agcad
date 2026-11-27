"""Import → parametric: drawing/mesh in, a rule out (M22).

Usage: .venv/bin/python examples/import_parametric.py [out.png]

The loop this example proves:

* a DXF drawing becomes a constraint-solved sketch (`import_dxf`), which
  extrudes into a solid;
* the solid cuts and projects back into new sketches (`section`, `project`);
* `primitives` recovers the planar faces, the inner cylinder and the hole so
  the part can be re-authored with named dimensions;
* a deliberately broken STL imports and `repair_mesh` closes it, after which
  `validate` says watertight;
* the sketch exports to SVG and imports back, extruding to the same volume;
* save/open replays the baked `add_sketch_curves` steps without the sources.
"""

import base64
import os
import struct
import sys
from pathlib import Path

sys.path.insert(0, "python")  # noqa: E402

from wright import Wright  # noqa: E402

DXF = """0
SECTION
2
HEADER
9
$INSUNITS
70
4
0
ENDSEC
0
SECTION
2
ENTITIES
0
LINE
10
0
20
0
11
80
21
0
0
LINE
10
80
20
0
11
80
21
40
0
LINE
10
80
20
40
11
0
21
40
0
LINE
10
0
20
40
11
0
21
0
0
CIRCLE
10
20
20
20
40
6
0
ENDSEC
0
EOF
"""


def bbox_size(entity: dict) -> list[float]:
    lo, hi = entity["world_bbox"]["min"], entity["world_bbox"]["max"]
    return [round(hi[i] - lo[i], 3) for i in range(3)]


def break_stl(path: str, broken: str) -> tuple[int, int]:
    """Drop the last triangle from a binary STL, leaving a real hole."""
    with open(path, "rb") as f:
        data = f.read()
    count = struct.unpack_from("<I", data, 80)[0]
    assert 84 + count * 50 <= len(data), "not a binary STL"
    with open(broken, "wb") as f:
        f.write(data[:80])
        f.write(struct.pack("<I", count - 1))
        f.write(data[84 : 84 + (count - 1) * 50])
    return count, count - 1


def main() -> None:
    out = sys.argv[1] if len(sys.argv) > 1 else "exports/import_parametric.png"
    exports = os.path.abspath("exports")
    os.makedirs(exports, exist_ok=True)
    dxf_path = os.path.join(exports, "import_plate.dxf")
    svg_path = os.path.join(exports, "import_plate.svg")
    wcad_path = os.path.join(exports, "import_parametric.wcad")
    Path(dxf_path).write_text(DXF, encoding="utf-8")

    cad = Wright.spawn()

    # 1. DXF → sketch → solid.
    doc = cad.create_document("dxf plate")["doc"]
    got = cad.import_dxf(doc, dxf_path, name="plate outline", auto_constraints=True)
    print(
        f"import_dxf: sketch {got['sketch']}, {got['curves']} curves → "
        f"{got['elements']} elements, {got['constraints']} constraints, "
        f"{got['closed_loops']} closed loop(s)"
    )
    plate = cad.extrude(doc, got["sketch"], distance=4)["entity"]
    print(f"  extrude: entity {plate}, volume {cad.validate(doc)['totals']['volume']:.2f} mm³")

    # 2. The imported drawing behaves like authored geometry: turning a knob
    #    re-solves the inferred constraints, it does not just scale a mesh.
    scene = cad.get_scene(doc)
    sketch = next(e for e in scene["entities"] if e["id"] == got["sketch"])
    print(
        f"  sketch solver: solved={sketch['sketch']['solved']} "
        f"dof={sketch['sketch']['dof']} "
        f"over_constrained={sketch['sketch']['over_constrained']}"
    )

    # 3. primitives: what is this part made of?
    report = cad.primitives(doc, plate)
    print(
        f"primitives: {len(report['planar_faces'])} planar faces, "
        f"{len(report['cylinders'])} cylinder(s), {len(report['holes'])} hole(s)"
    )
    for hole in report["holes"]:
        print(
            f"  {hole['descriptor']} (through={hole['through']}, "
            f"depth={hole['depth']:.2f} mm)"
        )

    # 4. section and project turn the solid back into sketch geometry.
    cut = cad.section(doc, plate, name="mid section", plane={"plane": "xy", "offset": 2})
    print(
        f"section: sketch {cut['sketch']}, {cut['elements']} elements, "
        f"{cut['closed_loops']} closed loop(s) (ready to extrude)"
    )
    cut_solid = cad.extrude(doc, cut["sketch"], distance=3)["entity"]
    outline = cad.project(doc, plate, name="top outline", plane={"plane": "xy", "offset": 30})
    print(
        f"project: sketch {outline['sketch']}, {outline['elements']} edges, "
        f"{outline['closed_loops']} closed loop(s)"
    )

    # 5. A broken mesh: export the plate, drop a triangle, import, repair.
    broken_stl = os.path.join(exports, "import_broken.stl")
    good_stl = os.path.join(exports, "import_good.stl")
    cad.export_stl(doc, good_stl, entities=[plate])
    before, after = break_stl(good_stl, broken_stl)
    repair_doc = cad.create_document("repair")["doc"]
    broken = cad.import_mesh(repair_doc, broken_stl, name="broken plate")
    bad = cad.validate(repair_doc, entities=[broken["entity"]])
    print(
        f"broken import: {before} → {after} triangles, "
        f"watertight={bad['solids'][0]['watertight']} "
        f"({bad['solids'][0]['open_edges']} open edges)"
    )
    fixed = cad.repair_mesh(repair_doc, broken["entity"])
    good = cad.validate(repair_doc, entities=[broken["entity"]])
    print(
        f"repair_mesh: filled {fixed['report']['holes_filled']} hole(s), "
        f"flipped {fixed['report']['flipped']}, removed "
        f"{fixed['report']['triangles_removed']} triangle(s); "
        f"watertight={good['solids'][0]['watertight']}"
    )

    # 6. SVG round trip: export the imported sketch, import it back, extrude.
    cad.export_svg(doc, svg_path, entities=[got["sketch"]], curves="profiles")
    svg_doc = cad.create_document("svg round trip")["doc"]
    back = cad.import_svg(svg_doc, svg_path, name="from SVG")
    back_solid = cad.extrude(svg_doc, back["sketch"], distance=4)["entity"]
    original = cad.validate(doc, entities=[plate])["solids"][0]["volume"]
    round_trip = cad.validate(svg_doc, entities=[back_solid])["solids"][0]["volume"]
    print(
        f"svg round trip: {back['elements']} elements, {back['closed_loops']} loop(s), "
        f"volume {round_trip:.2f} vs {original:.2f} mm³"
    )

    # 7. Save/open: the source DXF/SVG/STL files are no longer needed.
    cad.save_document(
        doc, wcad_path,
        source={"language": "python", "text": Path(__file__).read_text(encoding="utf-8")},
    )
    os.remove(dxf_path)
    os.remove(svg_path)
    os.remove(good_stl)
    os.remove(broken_stl)
    reopened = cad.open_document(wcad_path, name="reopened without sources")["doc"]
    methods = [cmd["method"] for cmd in cad.get_program(reopened)["program"]]
    trust = cad.validate(reopened)
    print(
        f"reopened: {len(methods)} steps ({methods[0]} first), "
        f"validate ok={trust['ok']}"
    )

    r = cad.render_offscreen(reopened, width=1000, height=700, path=out)
    png = base64.b64decode(r["png_base64"])
    assert png[:8] == bytes([0x89, 0x50, 0x4E, 0x47, 0x0D, 0x0A, 0x1A, 0x0A])
    print(f"rendered -> {r['path']} ({len(png)} bytes)")
    plate_entity = next(e for e in cad.get_scene(reopened)["entities"] if e["id"] == plate)
    print(f"part sizes: plate {bbox_size(plate_entity)} mm")

    cad.close()


if __name__ == "__main__":
    main()
