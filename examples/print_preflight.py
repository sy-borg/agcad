"""Printability preflight: from "watertight" to "this prints" (M23).

Usage: .venv/bin/python examples/print_preflight.py

This walks the questions an agent should answer before handing a model to a
slicer:

* ``validate`` grows a ``print`` report — overhang area at a configurable
  angle, bridge spans, first-layer contact, minimum feature, disconnected
  shells/floating islands, closed cavities with a suggested drainage hole, and
  cups — plus a top-level ``printable`` answer.
* ``suggest_orientation`` scores stable build orientations from the part's
  planar faces and the machine axes, returning absolute rotation/translation
  that can be applied with one ``transform``.
* ``arrange`` packs several solids onto the bed, lowest point at z=0, and
  records the moves as ordinary transforms so the layout undoes as one action.
* ``fit`` turns a nominal shaft/hole into a modeled size for free/sliding/
  snug/press fits, including material hole-shrink and an elephant-foot offset.
* ``set_material`` + ``bill_of_materials`` report mass and cost.
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "python"))

from wright import Wright  # noqa: E402


def main() -> None:
    with Wright.spawn() as cad:
        doc = cad.create_document("print preflight")["doc"]

        # A plate with a horizontal tunnel: its ceiling is a bridge, and the
        # flat bottom is first-layer contact.
        base = cad.add_box(
            doc, name="base", center=(0, 0, 4), size=(60, 40, 8)
        )["entity"]
        cutter = cad.add_cylinder(
            doc, name="tunnel", center=(0, 0, 4), radius=2.5, height=80
        )["entity"]
        cad.transform(doc, cutter, rotation_deg=(0, 90, 0))
        part = cad.boolean(doc, "difference", [base, cutter], name="bracket")["entity"]
        cad.delete(doc, base)
        cad.delete(doc, cutter)

        # A tall post demonstrates orientation scoring.
        post = cad.add_box(
            doc, name="post", center=(0, 0, 15), size=(10, 20, 30)
        )["entity"]

        report = cad.validate(
            doc,
            entities=[part],
            min_wall_mm=1.0,
            overhang_angle_deg=45,
            layer_height_mm=0.2,
            min_feature_mm=1.0,
        )
        solid = report["solids"][0]
        p = solid["print"]
        assert report["ok"] and report["printable"], report.get("warnings")
        print(
            f"validate: ok={report['ok']} printable={report['printable']} "
            f"contact={p['first_layer']['contact_area']:.0f} mm² "
            f"overhang={p['overhang_area']:.1f} mm² "
            f"bridge_span={p['max_bridge_span']:.1f} mm "
            f"shells={len(p['shells'])} cavities={len(p.get('cavities', []))}"
        )

        orientation = cad.suggest_orientation(doc, entities=[post], bed=(120, 120))
        best = orientation[0]["best"]
        print(
            f"orientation: {best['label']} height={best['height']:.0f} mm "
            f"footprint={best['footprint']:.0f} mm² score={best['score']:.2f}"
        )
        cad.transform(
            doc,
            post,
            translation=best["translation"],
            rotation_deg=best["rotation_deg"],
        )

        layout = cad.arrange(doc, entities=[part, post], bed=(120, 120), spacing_mm=4)
        print(
            f"arrange: fits={layout['fits']} parts={len(layout['placements'])} "
            f"bed={layout['bed']} used={layout['used_bbox']}"
        )

        fit = cad.fit(doc, "sliding", nominal_mm=8.0, material="petg")
        print(
            f"fit: sliding 8 mm shaft -> hole {fit['report']['modeled_hole_mm']:.2f} mm "
            f"(shrink {fit['report']['hole_shrink_mm']:.2f} mm/side, "
            f"elephant foot {fit['report']['elephant_foot_mm']:.2f} mm)"
        )

        # Materials feed the bill of materials. Parts are reused through an
        # assembly, which is where BOM rows (and their mass/cost) live.
        part_doc = cad.create_document("material part")["doc"]
        cube = cad.add_box(
            part_doc, name="cube", center=(0, 0, 5), size=(10, 10, 10)
        )["entity"]
        cad.set_material(part_doc, cube, "pla")
        asm = cad.create_document("materials")["doc"]
        cad.import_part(asm, from_doc=part_doc, name="cube")
        cad.add_instance(asm, "cube", "cube_1")
        row = cad.bill_of_materials(asm)[0]
        print(
            f"bom: {row['part']} x{row['quantity']} {row['material']} "
            f"volume={row['volume']:.0f} mm³ mass={row['mass_g']:.2f} g "
            f"cost=${row['cost']:.4f}"
        )
        print("print preflight complete")


if __name__ == "__main__":
    main()
