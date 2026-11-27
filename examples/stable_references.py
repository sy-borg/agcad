"""Stable references: enumerate faces/edges, bind a selector, and see drift.

Usage: .venv/bin/python examples/stable_references.py

M20 makes the reference behind a fillet (or a face sketch, hole, mirror or
mate) inspectable instead of implicit:

* ``faces`` lists the planar faces of a solid with a readable descriptor, a
  stable ``fingerprint`` (same feature across a resize) and an exact
  ``geometry`` hash (changes when the face moves).
* ``edges`` groups tessellated runs: a 32-segment cylinder rim is one ``curve``
  edge with ``segments: 32``, not 32 candidates.
* Every selector used by a modeling command reports ``matched``,
  ``ambiguous`` and a ``fingerprint``. On first use the engine stores the
  binding in the program; ``validate`` later labels the reference ``ok``,
  ``moved``, ``missing`` or ``drift``.

The script builds a block, rounds one vertical edge by its listed selector,
resizes the block (same edge: moved, not drift) and finally checks that the
ambiguity of a broad direction filter is visible.
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "python"))

from wright import Wright  # noqa: E402


def main() -> None:
    with Wright.spawn() as cad:
        doc = cad.create_document("stable references")["doc"]
        cad.set_parameter(doc, "width", 30.0, "block width [mm]")
        cad.set_parameter(doc, "height", 20.0, "block height [mm]")
        block = cad.add_box(
            doc,
            name="block",
            center=(0, 0, 0),
            size=("width", 20.0, "height"),
        )["entity"]

        faces = cad.faces(doc, block)
        print(f"faces: {len(faces)}")
        for face in faces[:3]:
            print(f"  {face['descriptor']}")
            print(f"    fp={face['fingerprint']} geom={face['geometry']}")

        edges = cad.edges(doc, block)
        print(f"edges: {len(edges)} logical edge(s)")
        vertical = next(
            edge for edge in edges if edge["direction"] and abs(edge["direction"][2]) > 0.9
        )
        print(f"  picked {vertical['descriptor']}")
        print(f"    faces={vertical['faces']} fp={vertical['fingerprint']}")

        # The listed selector is ready to use. The result reports what it
        # bound to, including a set fingerprint and ambiguity.
        filleted = cad.fillet(
            doc, block, 2.0, edges=vertical["selector"]
        )
        report = filleted["reference"]
        print(
            f"fillet: {filleted['edges']} segment(s), reference matched={report['matched']} "
            f"ambiguous={report['ambiguous']} fp={report['fingerprint']}"
        )
        assert report["matched"] == 1 and not report["ambiguous"]

        status = _reference(cad.validate(doc), "fillet")
        print(f"validate after fillet: {status['status']} ({status['actual_descriptor']})")
        assert status["status"] == "ok"

        # A knob turn resizes the block. The selector still names the same
        # logical edge, so it reports "moved", not "drift".
        cad.set_parameter(doc, "width", 34.0)
        status = _reference(cad.validate(doc), "fillet")
        print(f"width 30 -> 34: {status['status']} ({status['actual_descriptor']})")
        assert status["status"] == "moved"

        # A broad filter is honest about matching several logical edges.
        broad = cad.fillet(doc, block, 1.0, edges={"direction": [0, 0, 1]})
        report = broad["reference"]
        print(
            f"direction filter: matched={report['matched']} "
            f"ambiguous={report['ambiguous']} groups={report['groups']}"
        )
        assert report["matched"] > 1 and report["ambiguous"]


def _reference(validation: dict, method: str) -> dict:
    return next(ref for ref in validation["references"] if ref["method"] == method)


if __name__ == "__main__":
    main()
