"""Reconnect regressions for the Python client.

Run with the repo's venv, from the repo root::

    .venv/bin/python -m unittest discover -s python/tests

(or from ``python/``: ``python -m unittest discover -s tests``).

No server is needed: fake transports exercise the rebuild path in
``Wright._request`` and the retry wrapper.
"""
import unittest
from unittest import mock

from wright import Wright, WrightError
from wright import client as client_mod


class _BoomTransport:
    """Fails the moment a request is sent, like a restarted server."""

    def __init__(self) -> None:
        self.closed = False

    def send(self, line: str) -> None:
        raise WrightError("tcp connection to server lost")

    def recv(self, rid: int, timeout: float = 30.0) -> dict:
        raise AssertionError("recv must not be reached after send failed")

    def close(self) -> None:
        self.closed = True


class _OkTransport:
    def __init__(self) -> None:
        self.sent: list[str] = []
        self.closed = False

    def send(self, line: str) -> None:
        self.sent.append(line)

    def recv(self, rid: int, timeout: float = 30.0) -> dict:
        return {"jsonrpc": "2.0", "id": rid, "result": {"ok": {}}}

    def close(self) -> None:
        self.closed = True


class _RebuildingWright(Wright):
    def __init__(self, first, rest) -> None:
        super().__init__(first, reconnect=("spawn", ["unused"]))
        self.rest = list(rest)
        self.rebuilds = 0

    def _fresh_transport(self):
        self.rebuilds += 1
        return self.rest.pop(0)


class _UnreachableWright(Wright):
    def __init__(self, first) -> None:
        super().__init__(first, reconnect=("spawn", ["unused"]))

    def _fresh_transport(self):
        raise WrightError(
            "could not reconnect to the wright server (Connection refused); "
            "if it is still starting, retry the command"
        )


class ClientReconnectTest(unittest.TestCase):
    def test_connection_loss_rebuilds_once_and_bumps_generation(self) -> None:
        dead = _BoomTransport()
        live = _OkTransport()
        cad = _RebuildingWright(dead, [live])

        self.assertEqual(cad.generation, 0)
        self.assertEqual(cad.call("ping"), {"result": "ok"})
        self.assertEqual(cad.generation, 1)
        self.assertEqual(cad.rebuilds, 1)
        self.assertTrue(dead.closed, "the dead transport must be closed")
        self.assertEqual(len(live.sent), 1, "the request is retried once")

        # A healthy call does not bump the generation.
        cad.call("ping")
        self.assertEqual(cad.generation, 1)

    def test_failed_rebuild_surfaces_an_actionable_error(self) -> None:
        dead = _BoomTransport()
        cad = _UnreachableWright(dead)

        with self.assertRaises(WrightError) as ctx:
            cad.call("ping")
        self.assertIn("retry the command", str(ctx.exception))
        self.assertEqual(cad.generation, 0, "a failed rebuild is not a reconnect")

    def test_open_with_retry_wraps_oserror_after_all_attempts(self) -> None:
        delays = (0.0, 0.0, 0.0)
        with mock.patch.object(
            client_mod, "WrightTcpTransport", side_effect=ConnectionRefusedError(111, "refused")
        ) as transport:
            with self.assertRaises(WrightError) as ctx:
                client_mod._open_with_retry(("tcp", "127.0.0.1", 1), delays, "connect to")
        self.assertEqual(transport.call_count, len(delays))
        self.assertIn("could not connect to the wright server", str(ctx.exception))
        self.assertIn("retry the command", str(ctx.exception))


if __name__ == "__main__":
    unittest.main()
