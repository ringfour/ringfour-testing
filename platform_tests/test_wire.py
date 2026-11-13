"""Fast unit tests: URI parsing, JSON-RPC framing, and a real client/server
roundtrip over both transports — no ROS, no sim.

The in-process server uses the agent's actual server + handlers modules
(they import no rclpy by design), so this covers the same code paths the
sim-backed exit test exercises, minus ROS.
"""

from __future__ import annotations

import asyncio
import hashlib
import threading

import pytest

from ringfour import wire
from ringfour.errors import ConnectionFailed, ProtocolMismatch, RingfourError
from ringfour.transport import Connection
from ringfour.uri import parse_uri
from ringfour.client import Robot

from ringfour_agent.bootstrap import build_agent
from ringfour_agent.profile import load_profile

from .conftest import STUB_PROFILE, _free_tcp_port

# stub.yaml declares one tip-less pointing group, so `groups` is in
# the report even though the stub has no arm (6.1).
STUB_CAPABILITIES = {"mobility", "groups", "cameras", "lidar", "status"}


# -- uri ----------------------------------------------------------------------

def test_parse_unix_uri():
    """Parse unix uri

    Categories: sdk
    What it checks: Verifies that parse unix uri. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that framing, handshakes, error handling, timeouts, and binary payloads work with controlled inputs.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the wire transport code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    parsed = parse_uri("unix:///run/ringfour.sock")
    assert parsed.kind == "unix"
    assert parsed.path == "/run/ringfour.sock"


def test_parse_ws_uri_with_default_port():
    """Parse ws uri with default port

    Categories: sdk
    What it checks: Verifies that parse ws uri with default port. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that framing, handshakes, error handling, timeouts, and binary payloads work with controlled inputs.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the wire transport code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    parsed = parse_uri("ws://robot.local")
    assert parsed.kind == "tcp"
    assert parsed.host == "robot.local"
    assert parsed.port == 10101
    assert parse_uri("ws://10.0.0.5:9000").port == 9000


@pytest.mark.parametrize("bad", ["http://x", "unix://", "ws://", "nonsense"])
def test_parse_bad_uri_raises(bad):
    """Parse bad uri raises

    Categories: sdk
    What it checks: Verifies that parse bad uri raises. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that framing, handshakes, error handling, timeouts, and binary payloads work with controlled inputs.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the wire transport code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    with pytest.raises(ConnectionFailed):
        parse_uri(bad)


# -- framing --------------------------------------------------------------------

def test_request_response_roundtrip():
    """Request response roundtrip

    Categories: sdk
    What it checks: Verifies that request response roundtrip. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that framing, handshakes, error handling, timeouts, and binary payloads work with controlled inputs.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the wire transport code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    msg = wire.parse_message(wire.request(7, "ping", None))
    assert msg == {"jsonrpc": "2.0", "id": 7, "method": "ping"}
    msg = wire.parse_message(wire.response(7, {"ok": True}))
    assert msg["result"] == {"ok": True}
    msg = wire.parse_message(
        wire.error_response(7, wire.ERR_APPLICATION, "boom", {"type": "X"})
    )
    assert msg["error"]["code"] == wire.ERR_APPLICATION


def test_payload_frame_roundtrip():
    """Payload frame roundtrip

    Categories: sdk
    What it checks: Verifies that payload frame roundtrip. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that framing, handshakes, error handling, timeouts, and binary payloads work with controlled inputs.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the wire transport code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    frame = wire.encode_payload_frame(42, b"hello")
    assert wire.decode_payload_frame(frame) == (42, b"hello")


def test_payload_frame_too_short():
    """Payload frame too short

    Categories: sdk
    What it checks: Verifies that payload frame too short. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that framing, handshakes, error handling, timeouts, and binary payloads work with controlled inputs.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the wire transport code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    with pytest.raises(ValueError):
        wire.decode_payload_frame(b"abc")


def test_parse_message_rejects_non_jsonrpc():
    """Parse message rejects non jsonrpc

    Categories: sdk
    What it checks: Verifies that parse message rejects non jsonrpc. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that framing, handshakes, error handling, timeouts, and binary payloads work with controlled inputs.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the wire transport code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    with pytest.raises(ValueError):
        wire.parse_message('{"hello": "world"}')


# -- client/server roundtrip (in-process, both transports) ----------------------

@pytest.fixture()
def local_agent(tmp_path):
    """The real agent stack (server + handlers + audit + safety) on a
    background loop. No ROS."""
    profile = load_profile(STUB_PROFILE)
    unix_path = str(tmp_path / "agent.sock")
    port = _free_tcp_port()
    server, ctx = build_agent(
        profile,
        unix_path=unix_path,
        host="127.0.0.1",
        port=port,
        audit_dir=tmp_path / "audit",
     allow_unverified_stops=True)
    loop = asyncio.new_event_loop()
    thread = threading.Thread(target=loop.run_forever, daemon=True)
    thread.start()
    asyncio.run_coroutine_threadsafe(server.start(), loop).result(5)
    yield {"unix": f"unix://{unix_path}", "tcp": f"ws://127.0.0.1:{port}",
           "ctx": ctx, "audit_dir": tmp_path / "audit"}
    asyncio.run_coroutine_threadsafe(server.close(), loop).result(5)
    loop.call_soon_threadsafe(loop.stop)
    thread.join(timeout=2)
    ctx.audit.close()


@pytest.mark.parametrize("transport", ["unix", "tcp"])
def test_handshake_and_capability_report(local_agent, transport):
    """Handshake and capability report

    Categories: sdk
    What it checks: Verifies that handshake and capability report. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that framing, handshakes, error handling, timeouts, and binary payloads work with controlled inputs.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the wire transport code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    with Robot(local_agent[transport]) as robot:
        assert robot.name == "stub-sim"
        assert robot.capabilities == STUB_CAPABILITIES
        assert robot.has("mobility")
        assert not robot.has("slam")
        assert robot.profile.model == "base101 stub (phase 1)"
        assert robot.ping() < 1.0


def test_protocol_mismatch_rejected(local_agent):
    """Protocol mismatch rejected

    Categories: sdk
    What it checks: Verifies that protocol mismatch rejected. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that framing, handshakes, error handling, timeouts, and binary payloads work with controlled inputs.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the wire transport code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    conn = Connection(local_agent["tcp"])
    try:
        with pytest.raises(ProtocolMismatch):
            conn.call("hello", {"protocol": 999, "client_version": "x"})
    finally:
        conn.close()


def test_call_before_hello_rejected(local_agent):
    """Call before hello rejected

    Categories: sdk
    What it checks: Verifies that call before hello rejected. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that framing, handshakes, error handling, timeouts, and binary payloads work with controlled inputs.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the wire transport code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    conn = Connection(local_agent["tcp"])
    try:
        with pytest.raises(RingfourError):
            conn.call("ping")
    finally:
        conn.close()


def test_unknown_method_rejected(local_agent):
    """Unknown method rejected

    Categories: sdk
    What it checks: Verifies that unknown method rejected. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that framing, handshakes, error handling, timeouts, and binary payloads work with controlled inputs.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the wire transport code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    conn = Connection(local_agent["tcp"])
    try:
        conn.call("hello", {"protocol": 1, "client_version": "test"})
        with pytest.raises(RingfourError) as excinfo:
            conn.call("no.such_method", {})
        assert excinfo.value.call == "no.such_method"
        assert excinfo.value.robot == "stub-sim"
    finally:
        conn.close()


def test_binary_payload_channel(local_agent):
    """Binary payload channel

    Categories: sdk
    What it checks: Verifies that binary payload channel. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that framing, handshakes, error handling, timeouts, and binary payloads work with controlled inputs.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the wire transport code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    conn = Connection(local_agent["tcp"])
    try:
        conn.call("hello", {"protocol": 1, "client_version": "test"})
        result = conn.call("debug.send_payload", {"size": 100_000})
        kind, meta, data = conn.get_payload(result["payload_id"])
        assert kind == "debug/random"
        assert meta == {"size": 100_000}
        assert len(data) == 100_000
        assert hashlib.sha256(data).hexdigest() == result["sha256"]
    finally:
        conn.close()


def test_binary_payload_larger_than_websockets_default(local_agent):
    # websockets' default max_size is 1 MiB; an oversized frame aborts the
    # connection. Regression for the 4 MiB demo_04 failure: our transports
    # must agree on MAX_MESSAGE_BYTES.
    """Binary payload larger than websockets default

    Categories: sdk
    What it checks: Verifies that binary payload larger than websockets default. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that framing, handshakes, error handling, timeouts, and binary payloads work with controlled inputs.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the wire transport code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    size = 4 * 1024 * 1024
    conn = Connection(local_agent["tcp"])
    try:
        conn.call("hello", {"protocol": 1, "client_version": "test"})
        result = conn.call("debug.send_payload", {"size": size})
        _, _, data = conn.get_payload(result["payload_id"], timeout=15)
        assert len(data) == size
        assert hashlib.sha256(data).hexdigest() == result["sha256"]
    finally:
        conn.close()


def test_connect_refused_raises():
    """Connect refused raises

    Categories: sdk
    What it checks: Verifies that connect refused raises. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that framing, handshakes, error handling, timeouts, and binary payloads work with controlled inputs.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the wire transport code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    with pytest.raises(ConnectionFailed):
        Robot(f"ws://127.0.0.1:{_free_tcp_port()}", timeout=2.0)
