"""Payload-shape regressions for the M25 visual-verification client methods.

No server is needed: a fake transport records the JSON line each call sends,
so the tests pin the wire names and the calibration/view sugar.

    .venv/bin/python -m unittest discover -s python/tests
"""
import json
import unittest

from wright import Wright


class _RecordingTransport:
    def __init__(self) -> None:
        self.sent: list[str] = []
        self.result: dict = {"ok": {}}

    def send(self, line: str) -> None:
        self.sent.append(line)

    def recv(self, rid: int, timeout: float = 30.0) -> dict:
        return {"jsonrpc": "2.0", "id": rid, "result": self.result}

    def close(self) -> None:
        pass


class VisualClientTests(unittest.TestCase):
    def setUp(self) -> None:
        self.transport = _RecordingTransport()
        self.cad = Wright(self.transport)

    def last(self) -> dict:
        return json.loads(self.transport.sent[-1])

    def test_two_points_sugar_becomes_a_and_b(self) -> None:
        self.cad.set_calibration(
            1, reference="ref.png", two_points=((10, 20), (110, 20)), length_mm=50.0
        )
        self.assertEqual(self.last()["method"], "set_calibration")
        params = self.last()["params"]
        self.assertEqual(params["a"], [10, 20])
        self.assertEqual(params["b"], [110, 20])
        self.assertEqual(params["length_mm"], 50.0)
        self.assertEqual(params["reference"], "ref.png")
        self.assertFalse(params["clear"])

    def test_two_points_and_a_are_mutually_exclusive(self) -> None:
        with self.assertRaises(ValueError):
            self.cad.set_calibration(1, two_points=((0, 0), (1, 1)), a=(0, 0))

    def test_string_views_become_compare_view_objects(self) -> None:
        self.cad.compare_render(
            3,
            reference="ref.png",
            views=["front", {"view": "top", "reference": "top.png"}],
            size=256,
            regions=4,
            min_iou=0.9,
        )
        self.assertEqual(self.last()["method"], "compare_render")
        params = self.last()["params"]
        self.assertEqual(params["views"][0], {"view": "front"})
        self.assertEqual(params["views"][1]["reference"], "top.png")
        self.assertEqual(params["reference"], "ref.png")
        self.assertEqual(params["size"], 256)
        self.assertEqual(params["regions"], 4)
        self.assertEqual(params["min_iou"], 0.9)


if __name__ == "__main__":
    unittest.main()
