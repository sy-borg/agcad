"""Show what a knob turn actually rebuilds.

M15 memoizes each geometry feature by content, so `set_parameter` regenerates
only the branch whose inputs moved. The result carries `rebuilt`, `reused` and
`steps`, which this script prints for three cases:

* a no-op turn (same value): nothing rebuilds,
* a turn that reaches one independent branch: one fillet rebuilds,
* a cold `open_document`: every feature rebuilds, for comparison.

Usage: .venv/bin/python examples/incremental_rebuild.py
"""

import os
import shutil
import sys
import tempfile
import time

sys.path.insert(0, "python")  # noqa: E402

from wright import Wright  # noqa: E402

BRANCHES = 6


def main() -> None:
    with Wright.spawn() as cad:
        cad.initialize()
        doc = cad.create_document("incremental")["doc"]

        # Six independent boxes, each with its own fillet radius. They share
        # nothing, so changing one radius must leave the other five alone.
        for i in range(BRANCHES):
            cad.set_parameter(doc, f"r{i}", 2.0, f"fillet radius of branch {i} [mm]")
        for i in range(BRANCHES):
            box = cad.add_box(doc, f"box{i}", center=(i * 25, 0, 0), size=(18, 18, 18))
            cad.fillet(doc, box["entity"], f"r{i}")

        def turn(name: str, value: float) -> dict:
            started = time.perf_counter()
            result = cad.set_parameter(doc, name, value)
            elapsed = (time.perf_counter() - started) * 1000.0
            print(
                f"{name} = {value:g}: rebuilt {result['rebuilt']}, "
                f"reused {result['reused']}, steps {result['steps']} "
                f"({elapsed:.1f} ms)"
            )
            return result

        turn("r0", 2.0)          # same value: every feature is reused
        turn("r3", 2.5)          # one branch: one fillet rebuilds
        turn("r3", 2.0)          # back to the original shape: one rebuild again

        # A cold replay has no memo: all six boxes and six fillets rebuild.
        tmp = tempfile.mkdtemp(prefix="wright_incremental_")
        path = os.path.join(tmp, "branch.wcad")
        cad.save_document(doc, path)
        started = time.perf_counter()
        opened = cad.open_document(path)
        elapsed = (time.perf_counter() - started) * 1000.0
        print(
            f"cold open: {opened['entities']} entities, "
            f"{opened['commands']} commands ({elapsed:.1f} ms)"
        )
        cad.close_document(opened["doc"])
        shutil.rmtree(tmp, ignore_errors=True)


if __name__ == "__main__":
    main()
