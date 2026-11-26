"""M19 assemblies and drawings: a bolted two-post clamp, dimensioned.

Two reusable parts are authored once, then placed several times:

* ``plate`` — a 60 x 30 x 5 mm plate with two clearance holes,
* ``post`` — a 9 mm round post, 30 mm long.

The assembly imports the plate from a saved ``.wcad`` file and the post from
an open document, placing four instances. Connectors and a ``distance`` mate
seat the top plate on the posts, the BOM groups the instances, and
``check_interference`` proves the posts clear their holes. Finally
``export_drawing`` writes a dimensioned SVG and DXF sheet with balloons and a
BOM table.

The important property is that the assembly is *self-contained*: both part
programs are embedded in the assembly program, so the source files can be
deleted and ``assembly.wcad`` still reopens with every instance intact.

Usage: .venv/bin/python examples/assembly_drawing.py
"""

import os
import sys
from pathlib import Path

sys.path.insert(0, "python")  # noqa: E402

from wright import Wright  # noqa: E402

OUT = "examples/assembly_drawing"


def build_plate(cad: Wright) -> tuple[int, list[int]]:
    doc = cad.create_document("plate")["doc"]
    cad.set_parameter(doc, "w", 60.0, "plate width [mm]")
    cad.set_parameter(doc, "d", 30.0, "plate depth [mm]")
    cad.set_parameter(doc, "t", 5.0, "plate thickness [mm]")
    cad.set_parameter(doc, "hole_d", 10.0, "post clearance diameter [mm]")
    cad.set_parameter(doc, "span", 36.0, "hole center spacing [mm]")

    plate = cad.add_box(
        doc, "body", center=(0, 0, "t/2"), size=("w", "d", "t")
    )["entity"]
    cutters = []
    for x, label in ((-18.0, "left"), (18.0, "right")):
        cutters.append(
            cad.add_cylinder(
                doc,
                f"hole_{label}",
                center=(x, 0, "t/2"),
                radius="hole_d/2",
                height="t*3",
            )["entity"]
        )
    cad.boolean(doc, "difference", [plate, *cutters], name="plate")
    for eid in [plate, *cutters]:
        cad.delete(doc, eid)
    solids = [e["id"] for e in cad.get_scene(doc, meshes=False)["entities"]]
    return doc, solids


def build_post(cad: Wright) -> int:
    doc = cad.create_document("post")["doc"]
    cad.set_parameter(doc, "post_d", 9.0, "post diameter [mm]")
    cad.set_parameter(doc, "post_h", 30.0, "post length [mm]")
    cad.add_cylinder(
        doc, "post", center=(0, 0, "post_h/2"), radius="post_d/2", height="post_h"
    )
    return doc


def main() -> None:
    os.makedirs(OUT, exist_ok=True)
    source = Path(__file__).read_text(encoding="utf-8")
    plate_wcad = os.path.abspath(f"{OUT}/m19_plate.wcad")

    with Wright.spawn() as cad:
        cad.initialize()

        # 1. Parts are ordinary documents. Save one, and keep the other open:
        #    import_part accepts either source.
        plate_doc, _ = build_plate(cad)
        cad.save_document(plate_doc, plate_wcad)
        post_doc = build_post(cad)

        # 2. Assembly: import both parts, then place them.
        asm = cad.create_document("two-post clamp")["doc"]
        print("plate:", cad.import_part(asm, path=plate_wcad, name="plate"))
        print("post:", cad.import_part(asm, from_doc=post_doc, name="post"))

        base = cad.add_instance(asm, "plate", "base")
        for i, x in enumerate((-18.0, 18.0), start=1):
            cad.add_instance(asm, "post", f"post{i}", translation=(x, 0, 0))

        # The top plate starts far above and is pulled onto the posts by a
        # distance mate: base top (z=5) + 25 mm = the 30 mm post length.
        cad.add_connector(asm, base["instance"], "top", (0, 0, "5"), axis=(0, 0, 1))
        top = cad.add_instance(asm, "plate", "top", translation=(0, 0, 60))
        cad.add_connector(asm, top["instance"], "bottom", (0, 0, 0), axis=(0, 0, -1))
        cad.mate(
            asm,
            "distance",
            (base["instance"], "top"),
            (top["instance"], "bottom"),
            offset=25,
        )

        # 3. Assembly-level checks.
        for row in cad.bill_of_materials(asm):
            print(
                f"BOM {row['item']}: {row['quantity']} x {row['part']} "
                f"({row['entities']} solids, {row['volume']:.0f} mm3)"
            )
        print("interference:", cad.check_interference(asm))

        # 4. A drawing sheet with dimensions, balloons, BOM and title block.
        svg = os.path.abspath(f"{OUT}/assembly_drawing.svg")
        dxf = os.path.abspath(f"{OUT}/assembly_drawing.dxf")
        print("svg:", cad.export_drawing(asm, svg))
        print("dxf:", cad.export_drawing(asm, dxf, format="dxf"))
        png = os.path.abspath(f"{OUT}/assembly.png")
        cad.render_offscreen(asm, path=png, width=1000, height=700)

        # 5. Save the assembly with its recipe. The embedded part programs
        #    make this file independent of plate_wcad and post_doc.
        saved = os.path.abspath(f"{OUT}/assembly.wcad")
        print(
            "save:",
            cad.save_document(
                asm, saved, source={"language": "python", "text": source}
            ),
        )
        os.remove(plate_wcad)
        reopened = cad.open_document(saved, name="reopened clamp")["doc"]
        names = [e["name"] for e in cad.get_scene(reopened, meshes=False)["entities"]]
        print("reopened solids:", names)
        print(f"wrote {svg}, {dxf}, {png} and {saved}")


if __name__ == "__main__":
    main()
