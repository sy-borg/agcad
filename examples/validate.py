"""Trust loop: validate a build, then diff a knob change against it.

Usage: .venv/bin/python examples/validate.py

The agent-authored workflow is: build, prove the result is printable, turn a
knob, and say exactly what changed. This example does both halves of M14:

* ``validate`` checks every solid for watertightness, non-manifold edges,
  self-intersections, volume/bbox and an approximate thinnest wall, and every
  sketch for solver status and a closed profile. ``ok`` is the short answer;
  ``warnings`` explain failures in agent-readable terms.
* ``diff_scene`` compares the live document with a saved baseline by entity
  signature. It classifies entities as added/removed/modified, names the
  changed fields, and reports parameter and program changes.

A thin plate is deliberately used: at 4 mm it passes a 2 mm wall threshold,
and at 0.8 mm the same program fails it with the measured location.
"""

import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "python"))

from agcad import Agcad  # noqa: E402


def main() -> None:
    baseline = os.path.abspath("exports/validate_baseline.agcad")
    os.makedirs(os.path.dirname(baseline), exist_ok=True)

    with Agcad.spawn() as cad:
        doc = cad.create_document("validate demo")["doc"]
        cad.set_parameter(doc, "plate", 20.0, "plate side [mm]")
        cad.set_parameter(doc, "thickness", 4.0, "plate thickness [mm]")

        plate = cad.add_box(
            doc,
            name="plate",
            center=(0, 0, "thickness/2"),
            size=("plate", "plate", "thickness"),
        )["entity"]

        report = cad.validate(doc, min_wall_mm=2.0)
        solid = report["solids"][0]
        assert report["ok"], report.get("warnings")
        print(
            f"validate: ok={report['ok']} watertight={solid['watertight']} "
            f"self_intersections={solid['self_intersections']} "
            f"volume={solid['volume']:.0f} mm³ wall={solid['min_wall']:.2f} mm"
        )

        # Save this revision as the baseline, then turn the thickness knob.
        cad.save_document(doc, baseline)
        cad.set_parameter(doc, "thickness", 0.8)

        thin = cad.validate(doc, entities=[plate], min_wall_mm=2.0)
        solid = thin["solids"][0]
        assert not thin["ok"] and solid["thin"]
        print(
            f"thin plate: ok={thin['ok']} wall={solid['min_wall']:.2f} mm "
            f"at {tuple(round(v, 2) for v in solid['min_wall_at'])}"
        )
        for warning in thin.get("warnings", []):
            print(f"  warning: {warning}")

        diff = cad.diff_scene(doc, path=baseline)
        change = diff["modified"][0]
        param = diff["parameters"][0]
        assert diff["changed"] and change["changed"]
        print(
            f"diff: changed={diff['changed']} modified={len(diff['modified'])} "
            f"unchanged={diff['unchanged']} fields={change['changed']}"
        )
        print(
            f"  {param['name']}: {param['from']} -> {param['to']} mm; "
            f"volume {change['from']['volume']:.0f} -> {change['to']['volume']:.0f} mm³"
        )
        print(
            f"  signatures {change['from']['signature']} -> "
            f"{change['to']['signature']}"
        )

        # A no-op rebuild is visibly a no-op: ids, signatures and parameters
        # all come back identical.
        cad.set_parameter(doc, "thickness", 4.0)
        same = cad.diff_scene(doc, path=baseline)
        assert not same["changed"] and same["scene_signature"] == same["against_signature"]
        print(f"reverted: changed={same['changed']} unchanged={same['unchanged']}")


if __name__ == "__main__":
    main()
