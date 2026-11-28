"""Measurement and mass properties (M24).

Usage: .venv/bin/python examples/measure_mass.py

Answers the three questions an agent should never estimate from a render:

* ``measure`` — point-to-point distance, face-to-face angle, solid-to-solid
  closest approach and a plane cross-section. Every report carries the closest
  points, so the number is actionable.
* ``mass_properties`` — surface area, centroid and inertia per solid and for
  the assembly, with mass from a material density.
* ``clearance`` — the closest approach between two solids classified against
  the same free/sliding/snug/press table ``fit`` uses.
"""

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "python"))

from wright import Wright  # noqa: E402


def main() -> None:
    with Wright.spawn() as cad:
        doc = cad.create_document("measure and mass")["doc"]

        plate = cad.add_box(
            doc, name="plate", center=(0, 0, 5), size=(60, 40, 10)
        )["entity"]
        boss = cad.add_cylinder(
            doc, name="boss", center=(20, 0, 17), radius=8, height=14
        )["entity"]
        part = cad.boolean(doc, "union", [plate, boss], name="bracket")["entity"]
        cad.delete(doc, plate)
        cad.delete(doc, boss)

        # Distance is exact between points and reports the delta components.
        width = cad.measure(doc, [-30, -20, 10], [30, -20, 10])
        print(
            f"point distance: {width['distance']:.3f} mm "
            f"delta={width['delta']} axes={width['angles_to_axes']}"
        )

        # Parallel faces give both the closest approach and the plane gap; a
        # perpendicular pair gives the acute plane angle and the normal angle.
        top_faces = cad.measure(
            doc,
            {"face": {"entity": part, "normal": [0, 0, 1], "near": [0, 0, 10]}},
            {"face": {"entity": part, "normal": [0, 0, 1], "near": [20, 0, 24]}},
        )
        side_vs_top = cad.measure(
            doc,
            {"face": {"entity": part, "normal": [1, 0, 0]}},
            {"face": {"entity": part, "normal": [0, 0, 1]}},
        )
        print(
            f"faces: plane gap {top_faces['plane_distance']:.3f} mm, "
            f"side/top angle {side_vs_top['angle_deg']:.1f}° "
            f"(normals {side_vs_top['normal_angle_deg']:.1f}°)"
        )

        # A section through the plate, below the boss: area and perimeter.
        section = cad.measure(doc, part, plane={"plane": "xy", "offset": 3})
        cut = section["section"]
        print(
            f"section @ z=3: area {cut['area']:.1f} mm² "
            f"perimeter {cut['perimeter']:.1f} mm loops {cut['loops']}"
        )

        # Mass properties use the PLA preset; the assembly roll-up is the
        # whole bracket, and inertia is about the centroid in g·mm².
        mass = cad.mass_properties(doc, material="pla")
        solid = mass["solids"][0]
        assembly = mass["assembly"]
        expected = (
            (60 * 40 * 10 + math.pi * 8 * 8 * 14) * 1.24 / 1000
        )
        assert abs(solid["mass_g"] - expected) < 0.05, (solid["mass_g"], expected)
        print(
            f"mass: volume {solid['volume']:.0f} mm³ "
            f"area {solid['surface_area']:.1f} mm² "
            f"centroid {[round(v, 2) for v in solid['centroid']]} "
            f"mass {solid['mass_g']:.2f} g"
        )
        print(
            f"assembly: {assembly['solids']} solid(s) "
            f"mass {assembly['mass_g']:.2f} g "
            f"COM {[round(v, 2) for v in assembly['center_of_mass']]} "
            f"Ixx {assembly['inertia'][0][0]:.1f} g·mm²"
        )

        # Clearance is the fit question, answered from the geometry. A 0.3 mm
        # radial gap is a sliding fit in PLA (0.20 class + 0.05 shrink).
        pin = cad.add_box(doc, name="pin", center=(100, 0, 5), size=(4, 4, 10))[
            "entity"
        ]
        hole = cad.add_box(
            doc, name="hole block", center=(104.3, 0, 5), size=(4, 4, 10)
        )["entity"]
        gap = cad.clearance(doc, pin, hole, class_="sliding", material="pla")
        print(
            f"clearance: {gap['distance']:.3f} mm -> nearest "
            f"{gap['nearest_class']}, sliding required "
            f"{gap['fit']['required_radial_mm']:.2f} mm, fits={gap['fit']['fits']}"
        )
        print("measurement and mass properties complete")


if __name__ == "__main__":
    main()
