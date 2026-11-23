"""A mouse click without a mouse: `pick` returns the selector a human copies.

Usage: .venv/bin/python examples/pick.py [out.png]

The viewer does not invent a face or edge ID namespace. It sends a
world-space ray to `pick`, and the engine answers with selectors that are
already the wire shape of the commands that consume them:

  face -> add_sketch{plane: {...}}   {entity, normal, near}
  edge -> fillet/chamfer{edges: {...}}   {direction, near, near_radius}

Here a top-face pick places the boss sketch, and an edge pick survives a
parameter change before it is used for a fillet. The same JSON is what the
viewer HUD shows and copies.
"""

import os
import sys

sys.path.insert(0, "python")  # noqa: E402

from wright import Wright  # noqa: E402


def main() -> None:
    out = sys.argv[1] if len(sys.argv) > 1 else "exports/pick.png"
    os.makedirs(os.path.dirname(out) or ".", exist_ok=True)

    cad = Wright.spawn()
    doc = cad.create_document("pick demo")["doc"]
    for name, value, desc in [
        ("length", 60.0, "block length along X"),
        ("width", 40.0, "block width along Y"),
        ("height", 20.0, "block height along Z"),
    ]:
        cad.set_parameter(doc, name, value, desc)

    base = cad.add_box(
        doc, "base", center=(0, 0, "height/2"), size=("length", "width", "height")
    )["entity"]

    # --- a face click: put a boss on whatever face is under the ray --------
    top = cad.pick(doc, origin=(0, 0, 200), direction=(0, 0, -1))["hit"]
    assert top and top["entity"] == base
    print(f"picked {top['name']} #{top['entity']} at {top['point']}")
    print(f"  face selector: {top['face']}")

    boss_sketch = cad.add_sketch(doc, "boss", plane=top["face"])["entity"]
    for p1, p2 in [
        ((-6, -4), (6, -4)),
        ((6, -4), (6, 4)),
        ((6, 4), (-6, 4)),
        ((-6, 4), (-6, -4)),
    ]:
        cad.add_sketch_line(doc, boss_sketch, p1=p1, p2=p2)
    boss = cad.extrude(doc, boss_sketch, 6.0, name="boss")["entity"]

    # --- an edge click: pick the base's +X top rim -------------------------
    # 0.5 mm inside the rim is still a face hit, but close enough to snap.
    edge_hit = cad.pick(
        doc, origin=(29.5, 0, 200), direction=(0, 0, -1), tolerance=1.0
    )["hit"]
    assert edge_hit and edge_hit["entity"] == base and edge_hit.get("edge")
    edge = edge_hit["edge"]
    print(f"  edge selector: {edge}")

    # The selector is geometric, not an index: it still finds the rim after
    # a knob moves it. (The boss follows the top face through the same replay.)
    cad.set_parameter(doc, "height", 24.0)
    rounded = cad.fillet(doc, base, 2.0, edges=edge)
    print(f"  filleted {rounded['edges']} edge(s) after height 20 -> 24")
    assert rounded["edges"] == 1, "the old click should still name one edge"

    png = cad.render_offscreen(
        doc, width=900, height=650, path=out, background=(1.0, 1.0, 1.0, 1.0)
    )
    print(f"render: {png['path']}")
    scene = cad.get_scene(doc)
    bbox = next(e["world_bbox"] for e in scene["entities"] if e["id"] == boss)
    print(f"boss #{boss} world bbox: {bbox}")
    cad.close()


if __name__ == "__main__":
    main()
