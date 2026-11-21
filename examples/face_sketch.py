"""Sketches on faces: cut a pocket and grow a boss on a parametric block.

Usage: .venv/bin/python examples/face_sketch.py [out.png]

The base block is driven by ``length``/``width``/``height``. A sketch on its
TOP face (normal +Z) cuts a blind pocket with a negative extrude, and a sketch
on its +X face (normal +X) grows a cylindrical boss with a positive extrude.
Because both planes are derived from the faces, changing ``height`` or
``length`` carries the features with the body — the program replays, the
faces are found again, and the sketch frames follow.

The point of the example is the workflow: a face sketch's local +Z is the
face's OUTWARD normal. Positive extrude = material in front of the face;
negative extrude = into the body (combine with ``boolean difference``).
"""

import base64
import os
import sys
from pathlib import Path

sys.path.insert(0, "python")  # noqa: E402

from agcad import Agcad  # noqa: E402


def mesh_volume(mesh: dict) -> float:
    """Signed volume of a closed triangle mesh (mm³)."""
    p, idx = mesh["positions"], mesh["indices"]
    v = 0.0
    for i in range(0, len(idx), 3):
        a, b, c = (p[idx[i + k]] for k in range(3))
        v += a[0] * (b[1] * c[2] - b[2] * c[1])
        v -= a[1] * (b[0] * c[2] - b[2] * c[0])
        v += a[2] * (b[0] * c[1] - b[1] * c[0])
    return v / 6.0


def main() -> None:
    out = sys.argv[1] if len(sys.argv) > 1 else "exports/face_sketch.png"
    stl = os.path.abspath("exports/face_sketch.stl")
    os.makedirs(os.path.dirname(stl), exist_ok=True)

    cad = Agcad.spawn()
    doc = cad.create_document("face sketch")["doc"]

    for name, value, desc in [
        ("length", 80.0, "block length along X"),
        ("width", 50.0, "block width along Y"),
        ("height", 18.0, "block height along Z"),
        ("pocket_d", 30.0, "pocket diameter"),
        ("pocket_depth", 8.0, "pocket depth into the top face"),
        ("boss_d", 16.0, "boss diameter"),
        ("boss_h", 12.0, "boss height off the +X face"),
    ]:
        cad.set_parameter(doc, name, value, desc)

    base = cad.add_box(
        doc, "base", center=(0, 0, "height/2"), size=("length", "width", "height")
    )["entity"]

    # --- a pocket on the top face -------------------------------------------
    pocket_sketch = cad.add_sketch(
        doc,
        "pocket",
        plane={"entity": base, "normal": [0, 0, 1]},
    )["entity"]
    cad.add_sketch_circle(doc, pocket_sketch, (0, 0), "pocket_d/2")
    pocket = cad.extrude(doc, pocket_sketch, "-pocket_depth", name="pocket cutter")

    # --- a boss on the +X face ----------------------------------------------
    # On this face local X is world +Y and local Y is world +Z (the frame the
    # engine derives from the outward normal); a circle at the local origin
    # is centered on the face.
    boss_sketch = cad.add_sketch(
        doc,
        "boss",
        plane={"entity": base, "normal": [1, 0, 0]},
    )["entity"]
    cad.add_sketch_circle(doc, boss_sketch, (0, 0), "boss_d/2")
    boss = cad.extrude(doc, boss_sketch, "boss_h", name="boss")

    pocketed = cad.boolean(doc, "difference", [base, pocket["entity"]], name="pocketed")
    final = cad.boolean(doc, "union", [pocketed["entity"], boss["entity"]], name="finished")

    # The intermediate solids are scaffolding; the final boolean replays from
    # its operands, so deleting them keeps the program reproducible.
    for eid in (base, pocket["entity"], boss["entity"], pocketed["entity"]):
        cad.delete(doc, eid)

    scene = cad.get_scene(doc)
    for name, eid in [("pocket sketch", pocket_sketch), ("boss sketch", boss_sketch)]:
        f = next(e for e in scene["entities"] if e["id"] == eid)["sketch"]["plane"]
        print(
            f"{name}: origin {tuple(round(v, 3) for v in f['origin'])} "
            f"normal {tuple(round(v, 3) for v in f['normal'])} "
            f"x {tuple(round(v, 3) for v in f['x_axis'])}"
        )

    # Hide the construction sketches so the render shows the finished solid.
    # The booleans already consumed the solids, so this is safe on replay.
    for eid in (pocket_sketch, boss_sketch):
        cad.delete(doc, eid)

    scene = cad.get_scene(doc)
    solid = next(e for e in scene["entities"] if e["id"] == final["entity"])
    bb = solid["world_bbox"]
    volume = mesh_volume(solid["mesh"])
    expected = 80 * 50 * 18 - 3.14159265 * 15**2 * 8 + 3.14159265 * 8**2 * 12
    print(
        "finished: bbox "
        f"x {bb['min'][0]:.2f}..{bb['max'][0]:.2f} "
        f"y {bb['min'][1]:.2f}..{bb['max'][1]:.2f} "
        f"z {bb['min'][2]:.2f}..{bb['max'][2]:.2f}, "
        f"volume {volume:.1f} mm³ (pocket -{3.14159265 * 15**2 * 8:.0f}, "
        f"boss +{3.14159265 * 8**2 * 12:.0f})"
    )
    assert abs(volume - expected) / expected < 0.02, f"volume {volume} vs {expected}"

    exp = cad.export_stl(doc, stl)
    print(f"stl: {exp['path']} ({exp['tris']} tris, {exp['bytes']} bytes)")

    r = cad.render_offscreen(doc, width=1000, height=720, path=out)
    png = base64.b64decode(r["png_base64"])
    assert png[:8] == bytes([0x89, 0x50, 0x4E, 0x47, 0x0D, 0x0A, 0x1A, 0x0A])
    print(f"rendered -> {r['path']} ({len(png)} bytes)")

    # Prove the face references are parametric: grow the block and export
    # again. The pocket stays 8 mm deep from the new top; the boss stays on
    # the new +X face.
    cad.set_parameter(doc, "height", 30.0)
    cad.set_parameter(doc, "length", 110.0)
    scene = cad.get_scene(doc)
    solid = next(e for e in scene["entities"] if e["id"] == final["entity"])
    bb = solid["world_bbox"]
    print(
        "height 18 -> 30, length 80 -> 110: bbox "
        f"x {bb['min'][0]:.2f}..{bb['max'][0]:.2f} "
        f"z {bb['min'][2]:.2f}..{bb['max'][2]:.2f}"
    )
    assert abs(bb["max"][0] - (55.0 + 12.0)) < 1e-4, "boss follows the +X face"
    assert abs(bb["max"][2] - 30.0) < 1e-4, "block grows in Z"
    exp = cad.export_stl(doc, stl)
    print(f"stl re-exported: {exp['tris']} tris, {exp['bytes']} bytes")

    # The script is the source of truth; the .agcad is a replayable instance
    # carrying the current knob values and a copy of this script.
    source_text = Path(__file__).read_text(encoding="utf-8")
    agcad = os.path.abspath("exports/face_sketch.agcad")
    saved = cad.save_document(
        doc, agcad, source={"language": "python", "text": source_text}
    )
    assert cad.get_script(doc) == source_text, "the embedded source must round-trip"
    print(
        f"agcad: {saved['path']} ({saved['bytes']} bytes, "
        f"{saved['commands']} commands, source embedded)"
    )

    cad.close()


if __name__ == "__main__":
    main()
