"""A revolved bushing: profile → solid of revolution.

Usage: .venv/bin/python examples/revolve_bushing.py [out.png]

Shows the `revolve` half of the sketch→solid story: a stepped profile above
the sketch's X axis becomes a bushing with a through-bore. Every dimension
is parameter-driven, so changing a value re-solves the sketch and rebuilds
the solid.
"""

import base64
import sys

sys.path.insert(0, "python")  # noqa: E402

from agcad import Agcad  # noqa: E402


def main() -> None:
    out = sys.argv[1] if len(sys.argv) > 1 else "exports/bushing.png"

    cad = Agcad.spawn()
    doc = cad.create_document("bushing")["doc"]
    for name, value, desc in [
        ("length", 24.0, "overall length along the axis"),
        ("flange", 20.0, "flange outer radius"),
        ("body", 12.0, "body outer radius"),
        ("bore", 6.0, "through-bore radius"),
        ("flange_len", 6.0, "flange length"),
    ]:
        cad.set_parameter(doc, name, value, desc)

    sketch = cad.add_sketch(doc, "profile")["entity"]

    # Closed L-shaped profile in the sketch plane: x runs along the axis,
    # y is the radius. It sits above the X axis, so nothing crosses it.
    pts = [
        (0.0, "bore"),
        (0.0, "flange"),
        ("flange_len", "flange"),
        ("flange_len", "body"),
        ("length", "body"),
        ("length", "bore"),
    ]
    for i in range(len(pts)):
        cad.add_sketch_line(doc, sketch, pts[i], pts[(i + 1) % len(pts)])

    r = cad.revolve(doc, sketch, segments=128, name="bushing")
    print(f"revolve: entity {r['entity']}, volume {r['volume']:.1f} mm³, {r['tris']} tris")

    scene = cad.get_scene(doc)
    mesh = next(e for e in scene["entities"] if e["id"] == r["entity"])["mesh"]
    xs = [p[0] for p in mesh["positions"]]
    rs = [max(abs(p[1]), abs(p[2])) for p in mesh["positions"]]
    print(f"axis extent {max(xs) - min(xs):.2f} mm, max radius {max(rs):.2f} mm")

    png = cad.render_offscreen(doc, width=800, height=600, path=out)
    assert base64.b64decode(png["png_base64"])[:8] == bytes([0x89, 0x50, 0x4E, 0x47, 0x0D, 0x0A, 0x1A, 0x0A])
    print(f"rendered -> {png['path']}")

    cad.close()


if __name__ == "__main__":
    main()
