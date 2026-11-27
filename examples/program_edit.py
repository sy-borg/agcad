"""Program editing without a feature tree (M21).

Usage: .venv/bin/python examples/program_edit.py

The script is the source of truth, but an agent also needs surgical edits:
insert a step, turn a feature off for a variant, drive a repeat count from a
knob, and run a batch that either lands whole or not at all.

* ``patch_program`` inserts a step at the front. The dry run reports the id
  shift (every later entity renumbers) and a scene diff; the commit rewrites
  the program's numeric references so the boolean/fillet steps still target
  the same features. It is one undo step.
* ``suppress`` turns the fillet off without deleting the step, then back on.
* ``set_parameter`` drives the pattern count. A structural change renumbers
  downstream entities and reports the map, while the fillet follows the plate
  by feature identity.
* ``batch{atomic:true}`` rolls a failing multi-command batch back.
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "python"))

from wright import Wright, WrightError  # noqa: E402


def ids(cad: "Wright", doc: int) -> list[int]:
    return [entity["id"] for entity in cad.get_scene(doc)["entities"]]


def fillet_step(program: list[dict]) -> tuple[int, int]:
    for step, command in enumerate(program):
        if command.get("method") == "fillet":
            return step, command["entity"]
    raise AssertionError("no fillet step")


def main() -> None:
    with Wright.spawn() as cad:
        doc = cad.create_document("program editing")["doc"]
        cad.set_parameter(doc, "fins", 3, "number of cooling fins")

        base = cad.add_box(doc, name="base", center=(0, 0, 5), size=(80, 40, 10))["entity"]
        fins = cad.linear_pattern(
            doc,
            entities=[base],
            direction=(0, 1, 0),
            spacing=12,
            count="fins",
        )["entities"]
        plate = cad.add_box(doc, name="plate", center=(0, 0, 16), size=(30, 30, 4))["entity"]
        cad.fillet(doc, plate, radius=1.0)
        print(f"build: base={base} fins={fins} plate={plate}")

        before_ids = ids(cad, doc)
        before_steps = len(cad.get_program(doc)["program"])

        # Dry run: insert a tab before everything and prove the shift.
        tab = {
            "method": "add_box",
            "doc": doc,
            "name": "tab",
            "center": [45, 0, 2],
            "size": [10, 10, 4],
        }
        preview = cad.patch_program(
            doc, [{"op": "insert", "at": 0, "command": tab}], dry_run=True
        )
        assert preview["dry_run"] and preview["diff"]["changed"]
        assert ids(cad, doc) == before_ids and len(cad.get_program(doc)["program"]) == before_steps
        shift = {change["from"]: change["to"] for change in preview["renumbered"]}
        print(
            f"dry run: surviving={preview['surviving']} created={preview['created']} "
            f"renumbered={shift} diff={preview['diff']['changed']}"
        )
        assert preview["created"] == [1] and shift[plate] == plate + 1

        # Commit the same edit; the id map is now real.
        cad.patch_program(doc, [{"op": "insert", "at": 0, "command": tab}])
        moved_plate = plate + 1
        program = cad.get_program(doc)["program"]
        step, entity = fillet_step(program)
        assert entity == moved_plate, (entity, moved_plate)
        print(f"patched: {len(program)} steps, fillet follows plate -> {entity}")

        # A/B variant: the fillet is off, then on. The step is never deleted.
        cad.suppress(doc, [step])
        assert cad.validate(doc)["ok"]
        print(f"suppress: fillet step {step} off, validate=ok")
        cad.suppress(doc, [step], suppress=False)
        assert cad.validate(doc)["ok"]
        print(f"restore: fillet step {step} on, validate=ok")

        # The pattern count is a knob. Fewer copies renumber downstream ids.
        turned = cad.set_parameter(doc, "fins", 2)
        print(f"fins=2: renumbered={[(c['from'], c['to']) for c in turned['renumbered']]}")
        turned = cad.set_parameter(doc, "fins", 3)
        print(f"fins=3: renumbered={[(c['from'], c['to']) for c in turned['renumbered']]}")
        step, entity = fillet_step(cad.get_program(doc)["program"])
        assert entity == moved_plate
        assert cad.validate(doc)["ok"]

        # An atomic batch leaves no half model behind.
        before_ids = ids(cad, doc)
        before_steps = len(cad.get_program(doc)["program"])
        try:
            cad.batch(
                [
                    {
                        "method": "add_box", "doc": doc, "name": "ok",
                        "center": [0, 0, 40], "size": [5, 5, 5],
                    },
                    {
                        "method": "add_box", "doc": doc, "name": "bad",
                        "center": [0, 0, 50], "size": [0, 1, 1],
                    },
                ],
                atomic=True,
            )
        except WrightError as error:
            print(f"atomic batch: rolled back ({str(error).splitlines()[0]})")
        else:
            raise AssertionError("a zero-size box must fail")
        assert ids(cad, doc) == before_ids
        assert len(cad.get_program(doc)["program"]) == before_steps
        print(f"unchanged after failure: {len(before_ids)} entities, {before_steps} steps")


if __name__ == "__main__":
    main()
