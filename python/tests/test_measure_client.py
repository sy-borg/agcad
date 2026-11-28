"""Payload-shape regressions for the M24 measurement client methods.

No server is needed: a fake transport records the JSON line each call sends,
so the tests pin the wire names and the target/plane sugar.

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


class MeasureClientTests(unittest.TestCase):
    def setUp(self) -> None:
        self.transport = _RecordingTransport()
        self.cad = Wright(self.transport)

    def last(self) -> dict:
        return json.loads(self.transport.sent[-1])

    def test_points_solids_and_a_datum_plane_are_normalized(self) -> None:
        self.cad.measure(1, [0, 0, 0], (10, 0, 0), plane="xy")
        params = self.last()["params"]
        self.assertEqual(self.last()["method"], "measure")
        self.assertEqual(params["a"], {"kind": "point", "point": [0, 0, 0]})
        self.assertEqual(params["b"], {"kind": "point", "point": [10, 0, 0]})
        self.assertEqual(params["plane"], {"datum": {"plane": "xy"}})

        self.cad.measure(1, 4, {"solid": 5})
        params = self.last()["params"]
        self.assertEqual(params["a"], {"kind": "solid", "entity": 4})
        self.assertEqual(params["b"], {"kind": "solid", "entity": 5})

    def test_face_and_custom_plane_sugar(self) -> None:
        self.cad.measure(
            1,
            3,
            {"face": {"entity": 3, "normal": [0, 0, 1], "near": [0, 0, 10]}},
            plane={"origin": [0, 0, 5], "normal": [0, 0, 1]},
        )
        params = self.last()["params"]
        self.assertEqual(params["b"]["kind"], "face")
        self.assertEqual(params["b"]["entity"], 3)
        self.assertEqual(params["b"]["normal"], [0, 0, 1])
        self.assertEqual(params["plane"]["custom"]["origin"], [0, 0, 5])

    def test_mass_properties_and_clearance_wire_names(self) -> None:
        self.cad.mass_properties(1, entities=[2, 3], material="pla", density_g_cm3=1.2)
        params = self.last()["params"]
        self.assertEqual(self.last()["method"], "mass_properties")
        self.assertEqual(params["entities"], [2, 3])
        self.assertEqual(params["material"], "pla")
        self.assertEqual(params["density_g_cm3"], 1.2)

        self.cad.clearance(1, 2, 3, class_="snug", material="petg", tolerance=0.01)
        params = self.last()["params"]
        self.assertEqual(self.last()["method"], "clearance")
        self.assertEqual(params["class"], "snug")
        self.assertEqual(params["material"], "petg")
        self.assertEqual(params["tolerance"], 0.01)


if __name__ == "__main__":
    unittest.main()
