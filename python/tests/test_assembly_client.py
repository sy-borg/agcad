"""Payload-shape regressions for the M19 client methods.

No server is needed: a fake transport records the JSON line each call sends,
so the tests pin the wire names and the tuple/dict sugar.

    .venv/bin/python -m unittest discover -s python/tests
"""
import json
import unittest

from wright import Wright


class _RecordingTransport:
    def __init__(self) -> None:
        self.sent: list[str] = []
        self.closed = False

    def send(self, line: str) -> None:
        self.sent.append(line)

    def recv(self, rid: int, timeout: float = 30.0) -> dict:
        return {"jsonrpc": "2.0", "id": rid, "result": {"ok": {}}}

    def close(self) -> None:
        self.closed = True


class AssemblyClientTests(unittest.TestCase):
    def setUp(self) -> None:
        self.transport = _RecordingTransport()
        self.cad = Wright(self.transport)

    def last(self) -> dict:
        return json.loads(self.transport.sent[-1])

    def test_import_part_uses_the_reserved_from_key(self) -> None:
        self.cad.import_part(2, from_doc=1, name="plate")
        msg = self.last()
        self.assertEqual(msg["method"], "import_part")
        self.assertEqual(msg["params"]["from"], 1)
        self.assertIsNone(msg["params"]["path"])
        self.assertEqual(msg["params"]["name"], "plate")

    def test_mate_accepts_pairs_and_dicts(self) -> None:
        self.cad.mate(3, "distance", (1, "top"), {"instance": 2, "connector": "bottom"}, offset=5)
        params = self.last()["params"]
        self.assertEqual(params["kind"], "distance")
        self.assertEqual(params["a"], {"instance": 1, "connector": "top"})
        self.assertEqual(params["b"], {"instance": 2, "connector": "bottom"})
        self.assertEqual(params["offset"], 5)

    def test_export_drawing_normalizes_views_and_annotations(self) -> None:
        self.cad.export_drawing(
            4,
            "/tmp/out.svg",
            views=["front", {"kind": "iso", "at": [10, 20]}],
            annotations=[("NOTE", (5, 6)), {"text": "B", "at": [7, 8]}],
            scale=2,
        )
        params = self.last()["params"]
        self.assertEqual(params["format"], "svg")
        self.assertEqual(
            params["views"],
            [{"view": "front"}, {"view": "iso", "at": [10, 20]}],
        )
        self.assertEqual(
            params["annotations"],
            [{"text": "NOTE", "at": [5, 6]}, {"text": "B", "at": [7, 8]}],
        )
        self.assertEqual(params["scale"], 2)


if __name__ == "__main__":
    unittest.main()
