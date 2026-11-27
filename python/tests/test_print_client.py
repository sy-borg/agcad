"""Payload-shape regressions for the M23 printability client methods.

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
        self.result: dict = {"ok": {}}

    def send(self, line: str) -> None:
        self.sent.append(line)

    def recv(self, rid: int, timeout: float = 30.0) -> dict:
        return {"jsonrpc": "2.0", "id": rid, "result": self.result}

    def close(self) -> None:
        pass


class PrintClientTests(unittest.TestCase):
    def setUp(self) -> None:
        self.transport = _RecordingTransport()
        self.cad = Wright(self.transport)

    def last(self) -> dict:
        return json.loads(self.transport.sent[-1])

    def test_validate_forwards_print_parameters(self) -> None:
        self.cad.validate(
            1,
            overhang_angle_deg=50,
            layer_height_mm=0.16,
            min_feature_mm=0.8,
            bridge_angle_deg=15,
        )
        params = self.last()["params"]
        self.assertEqual(self.last()["method"], "validate")
        self.assertEqual(params["overhang_angle_deg"], 50)
        self.assertEqual(params["layer_height_mm"], 0.16)
        self.assertEqual(params["min_feature_mm"], 0.8)
        self.assertEqual(params["bridge_angle_deg"], 15)

    def test_set_material_and_presets_use_the_wire_names(self) -> None:
        self.cad.set_material(2, 7, "petg")
        params = self.last()["params"]
        self.assertEqual(params["material"], "petg")
        self.assertIsNone(params["density_g_cm3"])
        self.transport.result = {"materials": {"materials": []}}
        self.cad.list_materials()
        self.assertEqual(self.last()["method"], "list_materials")

    def test_orientation_and_arrange_expand_the_bed_tuple(self) -> None:
        self.transport.result = {"orientation_suggested": {"reports": []}}
        self.cad.suggest_orientation(1, entities=[3], bed=(120, 80), limit=4)
        params = self.last()["params"]
        self.assertEqual(params["bed_width_mm"], 120)
        self.assertEqual(params["bed_height_mm"], 80)
        self.assertEqual(params["limit"], 4)
        self.cad.arrange(1, bed=(120, 80), spacing_mm=3)
        params = self.last()["params"]
        self.assertEqual(params["spacing_mm"], 3)
        self.assertEqual(params["center"], True)

    def test_fit_reserves_the_class_keyword(self) -> None:
        self.cad.fit(1, "sliding", nominal_mm=8, material="petg")
        params = self.last()["params"]
        self.assertEqual(params["class"], "sliding")
        self.assertEqual(params["nominal_mm"], 8)
        self.assertEqual(params["material"], "petg")


if __name__ == "__main__":
    unittest.main()
