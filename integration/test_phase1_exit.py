"""Connection and handshake contracts for any running agent."""
import pytest
from ringfour import Robot
from ringfour.enums import AuditKind, Outcome
from ringfour.errors import ProtocolMismatch
from ringfour.models.protocol import Welcome
from ringfour.transport import Connection

pytestmark = pytest.mark.sim


def test_connect_to_configured_agent(robot, welcome, agent_full):
    """Connect to the selected robot

    Categories: connection,profile,safety
    What it checks: Opens a client connection, compares identity and capabilities with the collected handshake, checks a nonnegative ping round trip, validates handshake/group consistency and typed audit results when advertised, and tries a separate TCP endpoint when configured.
    Why it matters: All other checks depend on reaching the intended agent reliably.
    If it fails: Check the agent address, service health, and loaded profile. If identity changes during a run, keep the agent configuration stable.
    Profile fields: name;model;version
    Scenario fields: None.
    Robot effects: Opens a connection and pings.
    """
    assert robot.name == welcome.profile.name
    assert robot.profile == welcome.profile
    assert robot.capabilities == frozenset(welcome.capabilities)
    assert robot.ping() >= 0
    if agent_full.external_tcp_uri and agent_full.tcp_uri != agent_full.unix_uri:
        with Robot(agent_full.tcp_uri) as network:
            assert network.profile == welcome.profile
            assert network.ping() >= 0
    if robot.has("audit"):
        events = robot.audit.query(kind=AuditKind.RESULT, limit=50)
        assert events
        assert any(event.outcome is Outcome.SUCCESS for event in events)
        for event in events:
            assert event.kind is AuditKind.RESULT
            assert event.outcome is None or isinstance(event.outcome, Outcome)
    assert welcome.protocol == 1
    assert welcome.profile.name and welcome.profile.model
    assert len(welcome.capabilities) == len(set(welcome.capabilities))
    assert set(welcome.profile.capabilities) == set(welcome.capabilities)
    assert set(robot.groups) == set(welcome.groups) if robot.has("groups") else not welcome.groups
    assert set(welcome.instances.get("groups", ())) == set(welcome.groups)
    assert set(welcome.instances.get("arms", ())) == {n for n,g in welcome.groups.items() if g.kind == "arm"}


def test_connect_tcp(agent_full, welcome):
    """Connect over WebSocket

    Categories: connection
    What it checks: Connects through the supplied TCP endpoint and checks the loaded identity and a ping round trip. Skips when only a Unix endpoint is supplied.
    Why it matters: Verifies the network transport the operator configured.
    If it fails: Check the WebSocket host/port, exposed container port, firewall, and agent listener. Supply a TCP address to exercise this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Opens a network connection and pings.
    """
    if not agent_full.external_tcp_uri:
        pytest.skip("NOT CONFIGURED: no TCP endpoint supplied")
    with Robot(agent_full.tcp_uri) as robot:
        assert robot.profile == welcome.profile
        assert robot.ping() >= 0


def test_protocol_mismatch_rejected(agent_full):
    """Reject an incompatible protocol version

    Categories: connection,safety
    What it checks: Sends a handshake with deliberately unsupported protocol version 999 and expects a protocol-mismatch error.
    Why it matters: Incompatible clients must be refused before commands are accepted.
    If it fails: Verify that client and agent implement the same protocol and that the agent rejects unsupported versions. Do not change the robot profile to accept version 999.
    Profile fields: protocol
    Scenario fields: None.
    Robot effects: Opens a connection and sends an invalid handshake.
    """
    conn = Connection(agent_full.unix_uri)
    try:
        with pytest.raises(ProtocolMismatch):
            conn.call("hello", {"protocol": 999, "client_version": "conformance"})
    finally:
        conn.close()


def test_handshake_is_internally_consistent(welcome):
    """Robot identity and capability report agree

    Categories: profile,connection
    What it checks: Checks protocol 1, nonempty name/model, unique capabilities, matching profile capabilities, and consistency between group metadata and named arm/group instances.
    Why it matters: A contradictory handshake can make clients select the wrong interfaces.
    If it fails: Check profile identity and group kinds; inspect the agent's handshake serialization if its own lists disagree.
    Profile fields: name;model;protocol;groups.*.kind
    Scenario fields: None.
    Robot effects: Inspects the handshake.
    """
    assert isinstance(welcome, Welcome)
    assert welcome.protocol == 1
    assert welcome.profile.name and welcome.profile.model
    assert len(welcome.capabilities) == len(set(welcome.capabilities))
    assert set(welcome.profile.capabilities) == set(welcome.capabilities)
    assert set(welcome.instances.get("groups", ())) == set(welcome.groups)
    assert set(welcome.instances.get("arms", ())) == {n for n,g in welcome.groups.items() if g.kind == "arm"}


def test_advertised_capability_has_a_known_contract(capability_name, request):
    # This registry describes suite coverage, never a robot's anatomy.
    # Future capabilities cannot silently disappear behind a green run.
    """Every advertised capability has test coverage

    Categories: profile
    What it checks: Checks whether this suite has a contract for each advertised capability. A future capability with no shared test is explicitly untested.
    Why it matters: Passing existing checks must not imply that an unimplemented interface was validated.
    If it fails: Add the missing capability's test/backend contract. Removing a real capability merely to turn the report green hides coverage gaps.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Inspects capability coverage.
    """
    contracts = {"mobility", "groups", "sensors", "actions", "cameras", "lidar",
                 "range_sensors", "imu", "force_torque", "environment", "slam",
                 "teleop", "watches", "places", "audit", "status", "power"}
    if capability_name not in contracts:
        from conformance import skip_coverage
        skip_coverage(request, "not_configured", f"no shared test contract yet for capability {capability_name}")


def test_binary_payload_channel(agent_full, debug_enabled):
    """Binary data arrives intact

    Categories: connection
    What it checks: Requests 250,000 debug bytes, reads the payload, and checks its size, type, and SHA-256 checksum.
    Why it matters: Large payloads must survive the transport without corruption.
    If it fails: Check that the agent provides debug APIs and that debug checks have not been explicitly disabled in Test settings. Inspect transport framing and the debug payload handler.
    Profile fields: None.
    Scenario fields: debug
    Robot effects: Transfers synthetic bytes; debug checks are enabled by default.
    """
    import hashlib
    conn = Connection(agent_full.unix_uri)
    try:
        conn.call("hello", {"protocol": 1, "client_version": "conformance"})
        result = conn.call("debug.send_payload", {"size": 250000})
        kind,metadata,data = conn.get_payload(result["payload_id"])
        assert kind == "debug/random" and len(data) == 250000
        assert hashlib.sha256(data).hexdigest() == result["sha256"]
    finally:
        conn.close()
