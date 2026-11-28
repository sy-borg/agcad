"""Tests for `Wright.open()` discovery and the document helper.

No server or real socket is needed: discovery is exercised with mocks, and
`document()` uses a fake transport that answers the few bus commands it needs.
"""
import json
import os
import tempfile
import unittest
from unittest import mock

from wright import Wright, WrightError
from wright import client as client_mod


class _FakeTransport:
    def __init__(self, docs: list[dict]) -> None:
        self.docs = docs
        self.methods: list[str] = []

    def send(self, line: str) -> None:
        self.methods.append(json.loads(line)["method"])

    def recv(self, rid: int, timeout: float = 30.0) -> dict:
        method = self.methods[-1]
        if method == "list_documents":
            result = {"documents": {"docs": self.docs}}
        elif method == "clear_document":
            result = {"cleared": {"doc": self.docs[0]["doc"]}}
        elif method == "create_document":
            result = {"document_created": {"doc": 9}}
        else:
            raise AssertionError(f"unexpected method {method}")
        return {"jsonrpc": "2.0", "id": rid, "result": result}

    def close(self) -> None:
        pass


class OpenDiscoveryTest(unittest.TestCase):
    def test_private_forces_spawn(self) -> None:
        sentinel = object()
        with mock.patch.object(Wright, "spawn", return_value=sentinel) as spawn:
            self.assertIs(Wright.open(private=True), sentinel)
        spawn.assert_called_once_with()

    def test_private_env_forces_spawn(self) -> None:
        sentinel = object()
        with mock.patch.dict(os.environ, {"WRIGHT_PRIVATE": "1"}, clear=False), \
                mock.patch.object(Wright, "spawn", return_value=sentinel) as spawn:
            self.assertIs(Wright.open(), sentinel)
        spawn.assert_called_once_with()

    def test_wright_connect_env_parses_host_and_port(self) -> None:
        sentinel = object()
        with mock.patch.dict(
            os.environ, {"WRIGHT_CONNECT": "0.0.0.0:9000", "WRIGHT_PRIVATE": ""}, clear=False
        ), mock.patch.object(Wright, "connect", return_value=sentinel) as connect:
            self.assertIs(Wright.open(), sentinel)
        connect.assert_called_once_with("0.0.0.0", 9000)

    def test_explicit_endpoint_does_not_fall_back_to_spawn(self) -> None:
        with mock.patch.object(
            Wright, "connect", side_effect=WrightError("nope")
        ) as connect, mock.patch.object(Wright, "spawn") as spawn:
            with self.assertRaises(WrightError):
                Wright.open("127.0.0.1", 7999)
        connect.assert_called_once_with("127.0.0.1", 7999)
        spawn.assert_not_called()

    def test_explicit_endpoint_overrides_private_env(self) -> None:
        sentinel = object()
        with mock.patch.dict(os.environ, {"WRIGHT_PRIVATE": "1"}, clear=False), \
                mock.patch.object(Wright, "connect", return_value=sentinel) as connect, \
                mock.patch.object(Wright, "spawn") as spawn:
            self.assertIs(Wright.open(port=7999), sentinel)
        connect.assert_called_once_with("127.0.0.1", 7999)
        spawn.assert_not_called()

    def test_session_descriptor_is_found_from_a_subdirectory(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = os.path.join(tmp, "project")
            child = os.path.join(root, "examples")
            os.makedirs(child)
            sessions = os.path.join(tmp, "sessions")

            sentinel = object()
            old = os.getcwd()
            try:
                os.chdir(child)
                with mock.patch.dict(
                    os.environ,
                    {"WRIGHT_SESSION_DIR": sessions, "WRIGHT_CONNECT": "",
                     "WRIGHT_PRIVATE": ""},
                    clear=False,
                ):
                    session = client_mod._session_path(root)
                    session.parent.mkdir(parents=True, exist_ok=True)
                    session.write_text(
                        json.dumps({"host": "127.0.0.1", "port": 9123}),
                        encoding="utf-8",
                    )
                    with mock.patch.object(
                        Wright, "connect", return_value=sentinel
                    ) as connect:
                        self.assertIs(Wright.open(), sentinel)
                    connect.assert_called_once_with("127.0.0.1", 9123)
            finally:
                os.chdir(old)

    def test_no_backend_falls_back_to_spawn(self) -> None:
        with tempfile.TemporaryDirectory() as tmp, mock.patch.dict(
            os.environ,
            {"WRIGHT_SESSION_DIR": tmp, "WRIGHT_CONNECT": "", "WRIGHT_PRIVATE": ""},
            clear=False,
        ), mock.patch.object(
            client_mod, "_open_with_retry", side_effect=WrightError("refused")
        ), mock.patch.object(Wright, "spawn") as spawn:
            sentinel = object()
            spawn.return_value = sentinel
            self.assertIs(Wright.open(), sentinel)
        spawn.assert_called_once_with()

    def test_endpoint_forms(self) -> None:
        self.assertEqual(client_mod._parse_endpoint("7860"), ("127.0.0.1", 7860))
        self.assertEqual(client_mod._parse_endpoint(":7860"), ("127.0.0.1", 7860))
        self.assertEqual(client_mod._parse_endpoint("host:9000"), ("host", 9000))
        self.assertEqual(client_mod._parse_endpoint("[::1]:9000"), ("::1", 9000))

    def test_session_key_matches_the_rust_server(self) -> None:
        # Keep in lockstep with crates/server/src/session.rs.
        self.assertEqual(client_mod._session_key("C:\\project"), "1b78181f91d7ce21")
        self.assertEqual(
            client_mod._session_key("/home/sa/src/wright-cad"), "ca9f27a9e426936c"
        )


class DocumentHelperTest(unittest.TestCase):
    def test_existing_name_is_reused_and_reset(self) -> None:
        transport = _FakeTransport([{"doc": 7, "name": "bracket", "entity_count": 2}])
        cad = Wright(transport)
        self.assertEqual(cad.document("bracket", reset=True), 7)
        self.assertEqual(transport.methods, ["list_documents", "clear_document"])

    def test_existing_name_is_reused_without_reset(self) -> None:
        transport = _FakeTransport([{"doc": 7, "name": "bracket", "entity_count": 2}])
        cad = Wright(transport)
        self.assertEqual(cad.document("bracket"), 7)
        self.assertEqual(transport.methods, ["list_documents"])

    def test_missing_name_is_created(self) -> None:
        transport = _FakeTransport([])
        cad = Wright(transport)
        self.assertEqual(cad.document("bracket", doc=9), 9)
        self.assertEqual(transport.methods, ["list_documents", "create_document"])


if __name__ == "__main__":
    unittest.main()
