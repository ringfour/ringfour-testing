"""Profile claims, live state, multiple clients, and optional fault injection."""
import math
import time

import pytest
from ringfour import Robot
from ringfour.enums import AuditKind, Outcome
from ringfour.errors import SafetyViolation
from ringfour.transport import Connection
from conformance import expected_capabilities

pytestmark = pytest.mark.sim


def test_capabilities_reflect_profile_yaml(robot, profile_data, request):
    """Loaded robot matches the selected YAML

    Categories: profile
    What it checks: Compares live name, model, version, capability sections, camera names, and group names with the selected independent YAML.
    Why it matters: Editing a file is insufficient if the agent is still running another configuration.
    If it fails: Choose the YAML actually loaded by the agent. After valid edits, reload the agent using your stack's workflow and inspect again.
    Profile fields: name;model;version;groups;cameras
    Scenario fields: None.
    Robot effects: Compares live metadata with a file.
    """
    if profile_data is None:
        from conformance import skip_coverage
        skip_coverage(request, "not_configured", "no independent YAML supplied; set RINGFOUR_PROFILE")
    assert robot.name == profile_data["name"]
    assert robot.profile.model == profile_data["model"]
    assert robot.profile.version == str(profile_data["version"])
    assert robot.capabilities == expected_capabilities(profile_data)
    contracts = {"mobility", "groups", "sensors", "actions", "cameras", "lidar",
                 "range_sensors", "imu", "force_torque", "environment", "slam",
                 "teleop", "watches", "places", "audit", "status", "power"}
    missing = set(robot.capabilities) - contracts
    if missing:
        from conformance import skip_coverage
        skip_coverage(request, "not_configured", f"No shared operation tests for advertised capabilities: {sorted(missing)}")
    if robot.has("groups"):
        assert set(robot.groups) == set(profile_data.get("groups", {}))
    if robot.has("cameras"):
        assert set(robot.cameras) == set(profile_data.get("cameras", {}))


@pytest.mark.requires("mobility")
def test_pose_and_velocity_are_finite(robot):
    """Position and velocity readings are usable

    Categories: movement
    What it checks: Checks the live pose frame, finite 3D position and linear/angular velocities, Boolean movement state, and finite nonnegative distance traveled.
    Why it matters: Movement results cannot be trusted without usable state feedback.
    If it fails: Check odometry publication, its units, timestamps, and configured frame aliases.
    Profile fields: mobility.odom_topic;frames
    Scenario fields: None.
    Robot effects: Reads pose and velocity.
    """
    pose, velocity = robot.get_pose(), robot.get_velocity()
    assert pose.frame
    assert all(math.isfinite(v) for v in (pose.x, pose.y, pose.z or 0))
    assert all(math.isfinite(v) for vector in (velocity.linear, velocity.angular) for v in (vector.x, vector.y, vector.z))
    assert isinstance(robot.get_is_moving(), bool)
    assert math.isfinite(robot.get_distance_traveled()) and robot.get_distance_traveled() >= 0


@pytest.mark.requires("audit", "teleop")
@pytest.mark.motion
def test_rejected_call_lands_in_audit_log_with_reason(robot, debug_enabled, motion_enabled):
    """Emergency-stop rejection is recorded

    Categories: safety
    What it checks: Through debug APIs, engages emergency stop, requests a teleop session, expects an estop safety rejection, checks its audit entry, and restores the original stop state.
    Why it matters: A safety refusal needs a traceable reason for operators.
    If it fails: Enable the explicit debug scenario, check audit configuration and the safety handler, and preserve the robot's original stop state.
    Profile fields: audit;teleop
    Scenario fields: None.
    Robot effects: Temporarily changes debug emergency-stop state and restores it.
    """
    connection = robot._robot.connection
    call = lambda method, params: robot._runner.run(connection.call(method, params))
    was_engaged = robot.get_estop_engaged()
    call("debug.set_estop", {"engaged": True})
    try:
        with pytest.raises(SafetyViolation) as exc:
            call("teleop.start", {"watchdog": .1})
        assert exc.value.reason == "estop"
        events = robot.audit.query(kind=AuditKind.SAFETY)
        assert any(e.outcome is Outcome.REJECTED and e.detail.get("reason") == "estop" for e in events)
    finally:
        call("debug.set_estop", {"engaged": was_engaged})


def test_multi_client_reads(agent_full, welcome):
    """Two clients can read the same robot

    Categories: connection
    What it checks: Connects two clients to the same agent and checks identity and ping from both.
    Why it matters: Read-only tools should coexist with other clients.
    If it fails: Check agent connection/session handling and listener availability. This check does not test simultaneous command ownership.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Opens two read clients.
    """
    with Robot(agent_full.unix_uri) as first, Robot(agent_full.unix_uri) as second:
        for robot in (first, second):
            assert robot.profile == welcome.profile
            assert robot.ping() >= 0
        assert first.description_sha == second.description_sha


@pytest.mark.requires("audit")
def test_task_lifecycle_over_wire(robot, debug_enabled):
    """Task start and completion are recorded

    Categories: connection
    What it checks: Starts a short synthetic debug task and waits up to ten seconds for started and successful-completion audit entries.
    Why it matters: Clients need to distinguish a task being accepted from it actually completing.
    If it fails: Enable debug test plumbing, check audit configuration, and inspect task lifecycle handling if completion is missing.
    Profile fields: audit
    Scenario fields: debug
    Robot effects: Runs a synthetic debug task; requires opt-in.
    """
    result = robot._runner.run(robot._robot.connection.call("debug.run_task", {"duration": .5,"ticks": 5}))
    deadline = time.monotonic()+10
    events = []
    while time.monotonic()<deadline:
        events = robot.audit.query(task_id=result["task_id"])
        if any(e.outcome is Outcome.SUCCESS and e.detail.get("state")=="succeeded" for e in events):
            break
        time.sleep(.1)
    assert any(e.kind is AuditKind.COMMAND and e.detail.get("event")=="started" for e in events)
    assert any(e.outcome is Outcome.SUCCESS and e.detail.get("state")=="succeeded" for e in events)
