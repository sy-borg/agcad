"""Reproduce examples/example_body.png from its two views.

The reference drawing shows a plus-shaped prism: two 45 x 15 mm bars crossed
and extruded 50 mm, with ~1 mm fillets on the edges. The left panel is a
front view (45 mm wide, 50 mm tall); the right panel is a 3/4 view.

The plus profile is drawn as twelve parametric lines and extruded, so the
shape is driven by `span`, `arm` and `height` — change one and the sketch
re-solves and the solid rebuilds.

Fillets are not in agcad yet (see the roadmap), so the edges here are sharp;
everything else follows the drawing's proportions.

Writes the committed showcase into ``examples/cross_block/``: ``iso.png``,
``front.png``, ``cross_block.stl``, and (when ImageMagick is installed) the
stacked ``compare.png`` used in the README.

Usage: .venv/bin/python examples/cross_block.py
"""

import base64
import os
import shutil
import subprocess
import sys
import tempfile

sys.path.insert(0, "python")  # noqa: E402

from agcad import Agcad  # noqa: E402

# Close to the reference's light green; the lit faces come out near it.
GREEN = (0.85, 1.0, 0.83, 1.0)

# Measured off the drawing: its 9 mm scale bar gives ~12.4 px/mm, and the
# front panel is 553 x 614 px, so the body is 45 x 45 mm in plan and 50 tall.
SPAN, ARM, HEIGHT = 45.0, 15.0, 50.0

REFERENCE = os.path.join("examples", "example_body.png")
OUT_DIR = os.path.join("examples", "cross_block")


def compose_showcase(reference: str, iso_png: str, out_path: str) -> None:
    """Stack the drawing above the render, with labels.

    This is presentation only, so it is optional: without ImageMagick the
    model and the renders are still produced.
    """
    magick = shutil.which("magick")
    if not magick or not os.path.exists(reference):
        print("showcase: ImageMagick or the reference drawing missing, skipping compare.png")
        return
    tmp = tempfile.mkdtemp(prefix="agcad_showcase_")
    ref = os.path.join(tmp, "ref.png")
    iso = os.path.join(tmp, "iso.png")
    ref_l = os.path.join(tmp, "ref_l.png")
    iso_l = os.path.join(tmp, "iso_l.png")

    def run(*args: str) -> None:
        subprocess.run([magick, *args], check=True)

    run(reference, "-resize", "1200x", ref)
    run(
        iso_png, "-fuzz", "3%", "-trim", "+repage", "-bordercolor", "white",
        "-border", "30", "-resize", "x600", "-background", "white",
        "-gravity", "center", "-extent", "1200x600", iso,
    )
    for src, dst, text in [
        (ref, ref_l, "the drawing (examples/example_body.png)"),
        (iso, iso_l, "the model agcad rebuilt from it (examples/cross_block.py)"),
    ]:
        run(
            src, "-background", "white", "-gravity", "north", "-splice", "0x46",
            "-pointsize", "28", "-fill", "black", "-annotate", "+0+8", text, dst,
        )
    # Strip metadata / timestamps: re-running must produce identical bytes,
    # so the committed showcase is reproducible and the tree stays clean.
    run(
        ref_l, iso_l, "-background", "white", "-gravity", "center", "-append",
        "-strip", "-define", "png:exclude-chunks=date,time", out_path,
    )
    shutil.rmtree(tmp, ignore_errors=True)
    print(f"showcase -> {out_path}")


def main() -> None:
    os.makedirs(OUT_DIR, exist_ok=True)
    stl = os.path.abspath(os.path.join(OUT_DIR, "cross_block.stl"))
    iso_png = os.path.join(OUT_DIR, "iso.png")
    front_png = os.path.join(OUT_DIR, "front.png")
    compare_png = os.path.join(OUT_DIR, "compare.png")

    cad = Agcad.spawn()
    doc = cad.create_document("cross_block")["doc"]

    for name, value, desc in [
        ("span", SPAN, "overall width of the cross"),
        ("arm", ARM, "width of each arm"),
        ("height", HEIGHT, "extrusion height"),
    ]:
        cad.set_parameter(doc, name, value, desc)

    sketch = cad.add_sketch(doc, "profile")["entity"]

    # Plus outline, counter-clockwise, starting at the right arm's lower
    # corner. Expressions keep it parametric: the solver re-evaluates them
    # whenever a parameter changes.
    a, b = "span/2", "arm/2"
    na, nb = "-span/2", "-arm/2"
    pts = [
        (a, nb), (a, b), (b, b), (b, a),
        (nb, a), (nb, b), (na, b), (na, nb),
        (nb, nb), (nb, na), (b, na), (b, nb),
    ]
    for i in range(len(pts)):
        cad.add_sketch_line(doc, sketch, pts[i], pts[(i + 1) % len(pts)])

    snap = next(e for e in cad.get_scene(doc)["entities"] if e["id"] == sketch)["sketch"]
    print(f"profile: {len(snap['elements'])} lines, solved={snap['solved']}, dof={snap['dof']}")

    solid = cad.extrude(doc, sketch, "height", name="cross_block", color=GREEN)

    # The sketch stays in the document (the solid rebuilds from it), and its
    # display ribbons sit on the base plane. Tint them like the part so they
    # don't draw a blue outline around the bottom edge.
    cad.set_color(doc, sketch, GREEN)

    expected = (SPAN * ARM + SPAN * ARM - ARM * ARM) * HEIGHT
    print(f"extrude: entity {solid['entity']}, volume {solid['volume']:.0f} mm³ "
          f"(expected {expected:.0f}), {solid['tris']} tris")

    mesh = next(e for e in cad.get_scene(doc)["entities"] if e["id"] == solid["entity"])["mesh"]
    xs = [p[0] for p in mesh["positions"]]
    zs = [p[2] for p in mesh["positions"]]
    print(f"extents: {max(xs) - min(xs):.1f} x {max(zs) - min(zs):.1f} mm")

    exp = cad.export_stl(doc, stl)
    print(f"stl: {exp['path']} ({exp['tris']} tris, {exp['bytes']} bytes)")

    # 3/4 view. Camera fitted to the reference drawing's silhouette: the
    # projection of the model is compared with the drawing and the camera
    # searched for the best overlap (azimuth 30°, elevation 32° → 0.99 IoU).
    # Orthographic, like a CAD view: parallel edges stay parallel and size
    # does not change with depth.
    png = cad.render_offscreen(
        doc, width=900, height=900,
        eye=(83.7, 48.3, 85.4), target=(0.0, 0.0, 25.0), up=(0.0, 0.0, 1.0),
        fov_deg=40.0, ortho=True, background=(1.0, 1.0, 1.0, 1.0), path=iso_png,
    )
    assert base64.b64decode(png["png_base64"])[:8] == bytes([0x89, 0x50, 0x4E, 0x47, 0x0D, 0x0A, 0x1A, 0x0A])
    print(f"rendered 3/4  -> {png['path']}")

    # Front view along -Y, orthographic, to match the left panel of the
    # reference drawing.
    png = cad.render_offscreen(
        doc, width=640, height=720,
        eye=(0.0, -120.0, 25.0), target=(0.0, 0.0, 25.0), up=(0.0, 0.0, 1.0),
        fov_deg=40.0, ortho=True, background=(1.0, 1.0, 1.0, 1.0), path=front_png,
    )
    print(f"rendered front -> {png['path']}")

    compose_showcase(REFERENCE, iso_png, compare_png)

    # Turn a knob: the solid rebuilds from the re-solved sketch.
    cad.set_parameter(doc, "height", 30.0)
    mesh = next(e for e in cad.get_scene(doc)["entities"] if e["id"] == solid["entity"])["mesh"]
    zs = [p[2] for p in mesh["positions"]]
    print(f"height 50 -> 30: solid z extent now {max(zs) - min(zs):.1f} mm")
    cad.set_parameter(doc, "height", 50.0)

    # Persist the instance (program + knob values). The script above is the
    # recipe; this is the same model with the knobs as they stand, and an
    # embedded copy of the recipe as provenance.
    with open(__file__, encoding="utf-8") as f:
        source = {"language": "python", "text": f.read()}
    doc_path = os.path.abspath(os.path.join(OUT_DIR, "cross_block.agcad"))
    saved = cad.save_document(doc, doc_path, source=source)
    print(f"saved -> {saved['path']} ({saved['bytes']} bytes, {saved['params']} params)")

    cad.close()


if __name__ == "__main__":
    main()
