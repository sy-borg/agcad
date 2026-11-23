#!/usr/bin/env python3
"""First smoke test of the wright command bus, from the agent's seat."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "python"))

from wright import Wright  # noqa: E402

STEEL = (0.75, 0.76, 0.80, 1.0)
RED = (0.85, 0.25, 0.20, 1.0)


def main() -> None:
    with Wright.spawn() as cad:
        handshake = cad.initialize()
        print("handshake:", handshake)

        doc = cad.create_document("bracket")["doc"]
        print("document:", doc)

        base = cad.add_box(doc, name="base", center=(0, 0, 5), size=(80, 40, 10), color=STEEL)
        print("base:", base["entity"], "| verts:", len(base["mesh"]["positions"]))

        boss = cad.add_cylinder(doc, name="boss", center=(20, 0, 12), radius=8, height=14, color=STEEL)
        print("boss:", boss["entity"])

        pin = cad.add_sphere(doc, name="pin", center=(-25, 0, 15), radius=6, color=RED)
        cad.transform(doc, pin["entity"], rotation_deg=(0, 0, 45), scale=(1.0, 1.0, 0.5))

        scene = cad.get_scene(doc)
        print(f"scene: {len(scene['entities'])} entities")
        for e in scene["entities"]:
            print(f"  {e['id']}: {e['name']:<8} at {e['translation']}")

        # Undo semantics: each undo rolls back exactly one command.
        cad.undo(doc)  # undoes the transform -> still 3 entities
        scene = cad.get_scene(doc)
        assert len(scene["entities"]) == 3
        assert scene["entities"][2]["rotation_deg"] == [0.0, 0.0, 0.0], "transform was not undone"
        cad.undo(doc)  # undoes add_sphere -> 2 entities
        scene = cad.get_scene(doc)
        assert len(scene["entities"]) == 2, "undo failed"
        cad.redo(doc)  # re-adds the sphere with the SAME id (determinism)
        scene = cad.get_scene(doc)
        assert scene["entities"][2]["id"] == pin["entity"], "redo must restore identical ids"
        print("undo/redo: ok")

        docs = cad.list_documents()
        print("documents:", docs)
        print("ALL OK")


if __name__ == "__main__":
    main()
