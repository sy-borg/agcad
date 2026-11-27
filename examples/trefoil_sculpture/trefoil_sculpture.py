#!/usr/bin/env python3
"""Trefoil knot desk sculpture.

A trefoil (2,3)-torus knot swept from a circular cord, resting on a chamfered
stone plinth. The cord radius is a parameter, so the piece stays editable.
"""
import math

from wright import Wright

BRONZE = (0.78, 0.43, 0.18, 1.0)
STONE = (0.89, 0.87, 0.82, 1.0)


def trefoil(sxy=6.8, sz=10.0, samples=480):
    """Return a closed trefoil knot as a polyline of 3D points."""
    pts = []
    for i in range(samples):
        t = 2.0 * math.pi * i / samples
        pts.append((
            (math.sin(t) + 2.0 * math.sin(2.0 * t)) * sxy,
            (math.cos(t) - 2.0 * math.cos(2.0 * t)) * sxy,
            -math.sin(3.0 * t) * sz,
        ))
    return pts


def main() -> None:
    with Wright.spawn() as cad:
        doc = cad.create_document("trefoil-sculpture")["doc"]
        cad.set_parameter(doc, "tube_r", 1.9, description="cord radius (mm)")

        # Profile circle lives inside the plinth (z=4) so it never shows in renders.
        profile = cad.add_sketch(
            doc,
            "cord profile",
            plane={"custom": {"origin": (0, 0, 4), "normal": (0, 0, 1), "x_axis": (1, 0, 0)}},
        )["entity"]
        cad.add_sketch_circle(doc, profile, center=(0, 0), radius="tube_r")

        knot = cad.sweep(
            doc,
            profile,
            path={"polyline": {"points": trefoil(), "closed": True}},
            segments=24,
            name="trefoil",
            color=BRONZE,
        )["entity"]
        # Lift so the lowest point of the cord just kisses the plinth top (z=8).
        cad.transform(doc, knot, translation=(0, 0, 19.9), rotation_deg=(0, 0, 12))

        plinth = cad.add_cylinder(
            doc, name="plinth", center=(0, 0, 4), radius=26, height=8, segments=96, color=STONE
        )["entity"]
        cad.chamfer(doc, plinth, 1.5, edges={"normals": [[0, 0, 1]]})
        cad.chamfer(doc, plinth, 0.6, edges={"normals": [[0, 0, -1]]})

        print(cad.validate(doc)["totals"])
        cad.save_document(doc, "trefoil_sculpture.wcad")


if __name__ == "__main__":
    main()
