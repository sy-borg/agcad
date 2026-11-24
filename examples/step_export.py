"""STEP export: a faceted B-rep with an explicit solid/surface decision.

Usage: .venv/bin/python examples/step_export.py

wright is a tessellation kernel, so `export_step` does not pretend a
tessellated hole is an analytic cylinder. It writes an AP214 **faceted B-rep**:
every triangle becomes a planar `FACE_SURFACE` over a `POLY_LOOP`, coincident
vertices are welded and shared, and each closed edge-manifold piece becomes a
`FACETED_BREP`. The file is real B-rep topology that another CAD kernel can
measure, section and boolean, just without analytic surfaces.

The same command is honest about meshes that are not solids:

  * a watertight part becomes a `FACETED_BREP` (`solids` in the result),
  * an open sheet becomes an `OPEN_SHELL` in a
    `SHELL_BASED_SURFACE_MODEL` (`surfaces`), never a fake solid.

Incoming STEP is deliberately unsupported: real STEP files are mostly trimmed
analytic surfaces, and turning those into a mesh is a different kernel problem.
Use `import_mesh` (STL/OBJ/3MF) for geometry coming the other way.
"""

import os
import sys
from pathlib import Path

sys.path.insert(0, "python")  # noqa: E402

from wright import Wright  # noqa: E402


def main() -> None:
    os.makedirs("exports", exist_ok=True)
    part_step = os.path.abspath("exports/bracket.step")
    repeat_step = os.path.abspath("exports/bracket_repeat.step")
    sheet_step = os.path.abspath("exports/open_sheet.step")
    wcad = os.path.abspath("exports/step_bracket.wcad")

    cad = Wright.spawn()
    doc = cad.create_document("step bracket")["doc"]
    cad.set_parameter(doc, "width", 60.0, description="plate width [mm]")
    cad.set_parameter(doc, "depth", 30.0, description="plate depth [mm]")
    cad.set_parameter(doc, "thickness", 6.0, description="plate thickness [mm]")

    base = cad.add_box(
        doc, "base", center=(0, 0, "thickness/2"), size=("width", "depth", "thickness")
    )["entity"]
    hole = cad.add_cylinder(
        doc,
        "bore",
        center=(0, 0, "thickness/2"),
        radius=6,
        height="thickness*2",
        segments=48,
    )["entity"]
    bracket = cad.boolean(doc, "difference", [base, hole], name="bracket")["entity"]
    # Move it so the STEP file visibly carries world-space coordinates, not
    # the entity-local mesh.
    cad.transform(doc, bracket, translation=(100, 0, 0))

    # 1. The real part: one closed shell -> one faceted B-rep solid. Export
    #    only the finished boolean; its operands still exist in the program.
    got = cad.export_step(doc, part_step, entities=[bracket])
    print(
        f"step: {got['entities']} entity -> {got['solids']} solid(s), "
        f"{got['surfaces']} surface(s), {got['faces']} faces, "
        f"{got['bytes']} bytes -> {got['path']}"
    )
    text = Path(part_step).read_text(encoding="utf-8")
    print(
        "  entities:",
        f"{text.count('FACETED_BREP(')} faceted brep,",
        f"{text.count('FACE_SURFACE(')} planar faces,",
        f"{text.count('CARTESIAN_POINT(')} welded points,",
        f"{text.count('CYLINDRICAL_SURFACE(')} analytic cylinders",
    )
    assert text.count("FACETED_BREP(") == 1
    assert text.count("CYLINDRICAL_SURFACE(") == 0
    # World-space: the plate spans x = 100 +/- width/2.
    assert "(130.," in text and "(70.," in text

    # 2. Determinism: the header timestamp is fixed, so identical geometry
    #    produces identical bytes.
    cad.export_step(doc, repeat_step, entities=[bracket])
    assert Path(part_step).read_bytes() == Path(repeat_step).read_bytes()
    print("  byte-identical on a second export (fixed STEP timestamp)")

    # 3. An open sheet has no closed shell, so it is written as surfaces and
    #    reported honestly instead of being mislabelled a solid.
    sheet = cad.create_document("open sheet")["doc"]
    cad.add_mesh(
        sheet,
        "sheet",
        positions=[[0, 0, 0], [20, 0, 0], [20, 10, 0], [0, 10, 0]],
        indices=[0, 1, 2, 0, 2, 3],
    )
    got = cad.export_step(sheet, sheet_step)
    print(
        f"open mesh: {got['solids']} solid(s), {got['surfaces']} surface(s), "
        f"{got['faces']} faces -> {got['path']}"
    )
    sheet_text = Path(sheet_step).read_text(encoding="utf-8")
    assert got["solids"] == 0 and got["surfaces"] == 1
    assert "SHELL_BASED_SURFACE_MODEL" in sheet_text

    trust = cad.validate(doc, entities=[bracket])
    print(
        f"validate bracket: ok={trust['ok']}, "
        f"volume={trust['totals']['volume']:.1f} mm3"
    )

    cad.save_document(
        doc,
        wcad,
        source={"language": "python", "text": Path(__file__).read_text(encoding="utf-8")},
    )
    print(f"saved {wcad}")
    cad.close()


if __name__ == "__main__":
    main()
