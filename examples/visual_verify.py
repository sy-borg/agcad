"""Visual verification: score a picture against a part, not by eye (M25).

Usage: .venv/bin/python examples/visual_verify.py

``compare_render`` turns the picture→part loop into a deterministic number:
the candidate is rendered as a silhouette and compared with a reference PNG
or mesh by bbox-normalized IoU, aspect delta and per-region mismatch.

The demo starts from a reference part with a through-hole, builds a candidate
without it, and shows the center region reporting the missing material even
though the overall silhouette is still good. Drilling the hole closes the gap.
Then:

* ``set_calibration`` uses the known 80 mm base width (two pixel points) so
  the report also carries millimeters instead of scale-free pixels.
* A multi-view comparison checks front/top/right references, so an inner
  feature cannot hide behind one good silhouette.
* A mesh (STL) reference adds a volume and world-bbox comparison.
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "python"))

from wright import Wright  # noqa: E402

OUT = Path("exports/visual_verify")


def bracket(cad: Wright, name: str, hole: bool) -> tuple[int, int]:
    doc = cad.create_document(name)["doc"]
    plate = cad.add_box(doc, name="plate", center=(0, 0, 5), size=(80, 50, 10))["entity"]
    boss = cad.add_cylinder(doc, name="boss", center=(0, 0, 15), radius=10, height=10)["entity"]
    part = cad.boolean(doc, "union", [plate, boss], name="bracket")["entity"]
    cad.delete(doc, plate)
    cad.delete(doc, boss)
    if hole:
        cad.hole(doc, part, center=(0, 0, 20), diameter=8)
    return doc, part


def render_view(cad: Wright, doc: int, view: str, path: Path) -> str:
    eye, up = {
        "front": ((0, -220, 10), (0, 0, 1)),
        "top": ((0, 0, 220), (0, 1, 0)),
        "right": ((220, 0, 10), (0, 0, 1)),
    }[view]
    cad.render_offscreen(
        doc, width=420, height=420, eye=eye, target=(0, 0, 10), up=up,
        ortho=True, path=str(path),
    )
    return str(path)


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    with Wright.spawn() as cad:
        # The target has a through-hole; the first candidate does not.
        ref_doc, _ = bracket(cad, "reference", hole=True)
        cand_doc, candidate = bracket(cad, "candidate", hole=False)
        top = render_view(cad, ref_doc, "top", OUT / "reference_top.png")

        before = cad.compare_render(cand_doc, reference=top, view="top", size=420)
        center = next(r for r in before["views"][0]["regions"] if r["name"] == "center")
        print(
            f"missing hole: iou={before['iou']:.3f} passed={before['passed']} "
            f"center mismatch={center['mismatch']:.3f}"
        )
        assert center["mismatch"] > 0.05, "the region report should see the missing hole"
        assert before["needs_calibration"], "an image reference has no scale yet"

        # The base is 80 mm wide: two silhouette corners are the known length.
        bbox = before["views"][0]["reference_bbox"]
        cad.set_calibration(
            cand_doc,
            reference=top,
            two_points=(
                (bbox["x"], bbox["y"]),
                (bbox["x"] + bbox["width"], bbox["y"]),
            ),
            length_mm=80.0,
        )
        scaled = cad.compare_render(cand_doc, reference=top, view="top", size=420)
        size = scaled["views"][0]["reference_size_mm"]
        print(
            f"calibrated: reference {size['width']:.2f} x {size['height']:.2f} mm, "
            f"delta={scaled['views'][0]['size_delta_mm']}"
        )
        assert not scaled["needs_calibration"]

        # Drill the hole; the same view should now match the reference.
        cad.hole(cand_doc, candidate, center=(0, 0, 20), diameter=8)
        after = cad.compare_render(cand_doc, reference=top, view="top", size=420)
        center = next(r for r in after["views"][0]["regions"] if r["name"] == "center")
        print(f"after drill: iou={after['iou']:.3f} center mismatch={center['mismatch']:.3f}")
        assert after["iou"] > before["iou"]
        assert after["iou"] > 0.99 and after["passed"]

        # Multi-view: every reference is compared in its own view.
        front = render_view(cad, ref_doc, "front", OUT / "reference_front.png")
        right = render_view(cad, ref_doc, "right", OUT / "reference_right.png")
        multi = cad.compare_render(
            cand_doc,
            views=[
                {"view": "front", "reference": front},
                {"view": "top", "reference": top},
                {"view": "right", "reference": right},
            ],
            size=360,
        )
        for view in multi["views"]:
            print(
                f"  {view['view']:5s} iou={view['iou']:.3f} "
                f"aspect delta={view['aspect_delta']:.4f}"
            )
        print(
            f"multi-view: mean iou={multi['iou']:.3f} worst={multi['worst_view']} "
            f"passed={multi['passed']}"
        )
        assert multi["passed"] and multi["views"][0]["reference_kind"] == "image"

        # A mesh reference carries its own scale: volume and bbox come along.
        stl = OUT / "reference.stl"
        cad.export_stl(ref_doc, str(stl))
        mesh = cad.compare_render(cand_doc, reference=str(stl), view="front", size=360)
        volume = mesh["volumes"][0]
        print(
            f"mesh reference: volume ratio={volume['ratio']:.4f} "
            f"delta={volume['delta_mm3']:.1f} mm³"
        )
        assert abs(volume["ratio"] - 1.0) < 1e-6

        print("visual verification: all checks passed")


if __name__ == "__main__":
    main()
