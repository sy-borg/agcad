"""Payload-shape regressions for the M21 program-editing client methods.

No server is needed: a fake transport records the JSON line each call sends.

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


class _ProgramTransport(_RecordingTransport):
    """Answers get_program so export_python can be tested offline."""

    def recv(self, rid: int, timeout: float = 30.0) -> dict:
        return {
            "jsonrpc": "2.0",
            "id": rid,
            "result": {
                "program": {
                    "doc": 1,
                    "name": "variant",
                    "params": [],
                    "program": [
                        {"method": "add_box", "doc": 1, "name": "base"},
                        {"method": "fillet", "doc": 1, "entity": 1},
                    ],
                    "suppressed": [1],
                }
            },
        }


class ProgramEditingClientTests(unittest.TestCase):
    def setUp(self) -> None:
        self.transport = _RecordingTransport()
        self.cad = Wright(self.transport)

    def last(self) -> dict:
        return json.loads(self.transport.sent[-1])

    def test_patch_program_forwards_ops_and_dry_run(self) -> None:
        ops = [
            {"op": "insert", "at": 0, "command": {"method": "add_box", "doc": 1}},
            {"op": "reorder", "from": 2, "to": 0},
        ]
        self.cad.patch_program(1, ops, dry_run=True)
        msg = self.last()
        self.assertEqual(msg["method"], "patch_program")
        self.assertEqual(msg["params"]["doc"], 1)
        self.assertEqual(msg["params"]["ops"], ops)
        self.assertTrue(msg["params"]["dry_run"])

    def test_suppress_defaults_to_turning_steps_off(self) -> None:
        self.cad.suppress(3, [1, 2])
        msg = self.last()
        self.assertEqual(msg["method"], "suppress")
        self.assertEqual(msg["params"]["doc"], 3)
        self.assertEqual(msg["params"]["steps"], [1, 2])
        self.assertTrue(msg["params"]["suppress"])

        self.cad.suppress(3, [1], suppress=False)
        self.assertFalse(self.last()["params"]["suppress"])

    def test_batch_is_atomic_by_default(self) -> None:
        commands = [{"method": "add_box", "doc": 1, "name": "a"}]
        self.cad.batch(commands)
        msg = self.last()
        self.assertEqual(msg["method"], "batch")
        self.assertEqual(msg["params"]["commands"], commands)
        self.assertTrue(msg["params"]["atomic"])

        self.cad.batch(commands, atomic=False)
        self.assertFalse(self.last()["params"]["atomic"])

    def test_export_python_records_suppression(self) -> None:
        cad = Wright(_ProgramTransport())
        text = cad.export_python(1)
        self.assertIn("cad.add_box(doc, name='base')", text)
        self.assertIn("cad.suppress(doc, [1])", text)


if __name__ == "__main__":
    unittest.main()
