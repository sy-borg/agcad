"""Save a model to a file, open it again, and export it back to Python.

Two layers, one document, one source of truth:

- ``.py`` is the *source of truth*: the script an agent edits and re-runs.
  A copy is embedded in the ``.agcad`` as provenance (never executed), and
  ``get_program`` / ``get_script`` hand it back on a later open.
- ``.agcad`` is the *instance*: the program plus the current parameter
  values, as JSON. Geometry is not stored; opening replays the program
  deterministically, so the file stays small and diffable, and a user's
  knob turns survive.

``export_python`` reconstructs a runnable script from the program when no
embedded source exists, and bakes the current knob values back into a script
when the instance needs to become the new source.

Usage: .venv/bin/python examples/save_load.py
"""

import json
import os
import runpy
import sys
from pathlib import Path

sys.path.insert(0, "python")  # noqa: E402

from agcad import Agcad  # noqa: E402

PLATE = (0.72, 0.80, 0.72, 1.0)


def build(cad: Agcad) -> tuple[int, int]:
    """A plate with a bore, driven by four parameters."""
    doc = cad.create_document("plate")["doc"]
    for name, value, desc in [
        ("length", 60.0, "plate length"),
        ("width", 40.0, "plate width"),
        ("thickness", 8.0, "plate thickness"),
        ("hole", 14.0, "bore diameter"),
    ]:
        cad.set_parameter(doc, name, value, desc)

    plate = cad.add_box(
        doc,
        "plate",
        center=(0.0, 0.0, "thickness/2"),
        size=("length", "width", "thickness"),
        color=PLATE,
    )["entity"]
    # The bore is a +Z cylinder, already upright: it cuts through the plate.
    bore = cad.add_cylinder(
        doc,
        "bore",
        center=(0.0, 0.0, "thickness/2"),
        radius="hole/2",
        height="thickness*3",
        segments=64,
        color=PLATE,
    )["entity"]
    part = cad.boolean(doc, "difference", [plate, bore], name="plate_with_hole")["entity"]
    cad.delete(doc, plate)
    cad.delete(doc, bore)
    return doc, part


def scene_signature(cad: Agcad, doc: int) -> str:
    """Ids, placement and every mesh bit — the replay contract."""
    keep = ("id", "name", "translation", "rotation_deg", "scale", "transform", "color", "mesh")
    ents = cad.get_scene(doc)["entities"]
    return json.dumps([{k: e[k] for k in keep} for e in ents], sort_keys=True)


def world_extents(cad: Agcad, doc: int) -> list[float]:
    """Axis-aligned size of the whole scene, in world space."""
    lo = [float("inf")] * 3
    hi = [float("-inf")] * 3
    for e in cad.get_scene(doc)["entities"]:
        m = e["transform"]  # column-major 4x4
        for p in e["mesh"]["positions"]:
            w = [m[0][r] * p[0] + m[1][r] * p[1] + m[2][r] * p[2] + m[3][r] for r in range(3)]
            for i in range(3):
                lo[i] = min(lo[i], w[i])
                hi[i] = max(hi[i], w[i])
    return [round(hi[i] - lo[i], 3) for i in range(3)]


def main() -> None:
    out_dir = "exports"
    os.makedirs(out_dir, exist_ok=True)
    doc_path = os.path.abspath(os.path.join(out_dir, "plate.agcad"))
    py_path = os.path.abspath(os.path.join(out_dir, "plate.py"))
    png_path = os.path.join(out_dir, "plate_reopened.png")

    # --- author ------------------------------------------------------------
    cad_a = Agcad.spawn()
    doc_a, _ = build(cad_a)
    signature = scene_signature(cad_a, doc_a)
    print(f"built: {world_extents(cad_a, doc_a)} mm")

    # --- save: program + knobs (+ the recipe, as provenance) ---------------
    source = {"language": "python", "text": Path(__file__).read_text(encoding="utf-8")}
    saved = cad_a.save_document(doc_a, doc_path, source=source)
    print(
        f"saved: {os.path.basename(saved['path'])} ({saved['bytes']} bytes, "
        f"{saved['commands']} commands, {saved['params']} parameters)"
    )
    data = json.loads(Path(doc_path).read_text(encoding="utf-8"))
    print(f"       keys {sorted(data)}")
    print(f"       'thickness' = {data['parameters']['thickness']['value']!r}")
    print(f"       first command = {data['program'][0]}")
    cad_a.close()

    # --- open in a *fresh* engine: geometry must come back bit-identical ---
    cad_b = Agcad.spawn()
    opened = cad_b.open_document(doc_path)
    print(
        f"opened: doc {opened['doc']} '{opened['name']}' — "
        f"{opened['entities']} entities, {opened['commands']} commands, "
        f"{opened['params']} parameters"
    )
    assert scene_signature(cad_b, opened["doc"]) == signature, "replay drifted"
    print("        geometry matches the original exactly")

    # The script rides along as the canonical recipe and comes back out.
    recovered = cad_b.get_script(opened["doc"])
    assert recovered == source["text"], "the embedded source must round-trip"
    assert cad_b.get_program(opened["doc"])["source"]["text"] == source["text"]
    print("        embedded source recovered (the script is the source of truth)")

    # --- the knobs still work after a round trip ---------------------------
    cad_b.set_parameter(opened["doc"], "thickness", 12.0)
    print(f"thickness 8 -> 12: {world_extents(cad_b, opened['doc'])} mm")
    assert abs(world_extents(cad_b, opened["doc"])[2] - 12.0) < 1e-6

    png = cad_b.render_offscreen(
        opened["doc"], width=800, height=600,
        background=(1.0, 1.0, 1.0, 1.0), path=png_path,
    )
    print(f"render: {png['path']}")

    # --- export the stored program back to a runnable script ---------------
    text = cad_b.export_python(opened["doc"], py_path)
    print(f"python: {os.path.basename(py_path)} ({len(text.splitlines())} lines)")
    cad_b.close()

    # The generated script is a normal client: run it end to end. Any bad
    # command would raise and make this fail.
    runpy.run_path(py_path, run_name="__main__")
    print("        generated script runs and rebuilds the model")


if __name__ == "__main__":
    main()
