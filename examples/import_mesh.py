"""Mesh import round trip: STL and 3MF in, a printable solid out.

Usage: .venv/bin/python examples/import_mesh.py [out.png]

The interesting property is that import is *data*, not a file reference.
`import_mesh` parses the file and records a normal `add_mesh` program step with
the baked triangles, so the saved `.wcad` opens after the source STL/3MF is
gone and the part can be transformed, booleaned, validated and re-exported
like any other solid.
"""

import base64
import os
import sys
from pathlib import Path

sys.path.insert(0, "python")  # noqa: E402

from wright import Wright  # noqa: E402


def bbox_extent(entity: dict) -> list[float]:
    lo, hi = entity["world_bbox"]["min"], entity["world_bbox"]["max"]
    return [hi[i] - lo[i] for i in range(3)]


def main() -> None:
    out = sys.argv[1] if len(sys.argv) > 1 else "exports/import_mesh.png"
    stl = os.path.abspath("exports/import_source.stl")
    mf = os.path.abspath("exports/import_source.3mf")
    wcad = os.path.abspath("exports/import_roundtrip.wcad")
    os.makedirs(os.path.dirname(stl), exist_ok=True)

    cad = Wright.spawn()

    # 1. Author a small part with one source of truth: the script.
    src = cad.create_document("import source")["doc"]
    base = cad.add_box(src, "base", center=(0, 0, 10), size=(60, 40, 20))["entity"]
    cutter = cad.add_cylinder(src, "hole", center=(0, 0, 10), radius=6, height=30,
                              segments=48)["entity"]
    part = cad.boolean(src, "difference", [base, cutter], name="bracket")["entity"]
    # Export only the finished solid: source operands are kept by `boolean`,
    # and exporting the whole document would put overlapping solids in one
    # STL. A mesh file is one part, so drop the operands first.
    cad.delete(src, cutter)
    cad.delete(src, base)
    cad.export_stl(src, stl, entities=[part])
    cad.export_3mf(src, mf, entities=[part])
    print(f"wrote {stl} and {mf}")

    # 2. Import the STL into a fresh document and keep modeling on it.
    doc = cad.create_document("stl import")["doc"]
    got = cad.import_mesh(doc, stl, name="bracket from STL", smooth_angle_deg=30)
    print(
        f"import_mesh: entity {got['entity']}, {got['format']} "
        f"{got['vertices']} verts / {got['triangles']} tris "
        f"({got['dropped']} dropped, {got['bytes']} bytes)"
    )
    scene = cad.get_scene(doc)
    imported = next(e for e in scene["entities"] if e["id"] == got["entity"])
    print(f"  world size {bbox_extent(imported)} mm")
    trust = cad.validate(doc)
    print(f"  validate: ok={trust['ok']} (welded STL holes stay watertight)")

    # The import is a normal solid: fuse a boss onto it.
    cad.add_box(doc, "boss", center=(0, 0, 25), size=(20, 20, 10))
    fused = cad.boolean(doc, "union", [got["entity"], 2], name="bracket + boss")
    print(f"  union: entity {fused['entity']}, {len(fused['mesh']['indices']) // 3} tris")

    # 3. Import the 3MF (units and multiple objects come from the package).
    doc3 = cad.create_document("3mf import")["doc"]
    got3 = cad.import_mesh(doc3, mf, name="bracket from 3MF")
    trust3 = cad.validate(doc3)
    print(
        f"3mf import: entity {got3['entity']}, {got3['triangles']} tris, "
        f"validate ok={trust3['ok']}"
    )
    r = cad.render_offscreen(doc3, width=900, height=650, path=out)
    png = base64.b64decode(r["png_base64"])
    assert png[:8] == bytes([0x89, 0x50, 0x4E, 0x47, 0x0D, 0x0A, 0x1A, 0x0A])
    print(f"rendered -> {r['path']} ({len(png)} bytes)")

    # 4. Prove the save is self-contained: bake the program, delete the source
    #    files, then reopen. The program says add_mesh, not import_mesh.
    before = cad.get_program(doc)["program"]
    print(f"program step: {[cmd['method'] for cmd in before]}")
    cad.save_document(
        doc, wcad,
        source={"language": "python", "text": Path(__file__).read_text(encoding="utf-8")},
    )
    os.remove(stl)
    os.remove(mf)
    reopened = cad.open_document(wcad, name="reopened without the files")["doc"]
    program = cad.get_program(reopened)["program"]
    methods = [cmd["method"] for cmd in program]
    assert methods[0] == "add_mesh", methods
    trust = cad.validate(reopened)
    print(f"reopened: {len(program)} steps, first is {methods[0]}, validate ok={trust['ok']}")

    cad.close()


if __name__ == "__main__":
    main()
