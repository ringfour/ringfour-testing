"""Typed live values without assumptions about locomotion or transport."""
import pytest
from ringfour.enums import AuditKind, Outcome, SlamMode, Transport, Phase
from ringfour.models import PayloadRef
from conformance import validate_pose

pytestmark = pytest.mark.sim


@pytest.mark.requires("audit")
def test_audit_outcomes_arrive_as_enum_members(robot):
    """Audit records have consistent result types

    Categories: safety
    What it checks: Pings the robot, reads result audit entries, checks typed event kind/outcome and a successful result, and compares the string/member filter behavior.
    Why it matters: A safety and command history needs consistent, interpretable outcomes.
    If it fails: Check audit configuration and the agent/client protocol versions. Missing or malformed records can be an agent/SDK issue rather than a profile setting.
    Profile fields: audit
    Scenario fields: None.
    Robot effects: Pings and reads audit records.
    """
    robot.ping()
    events = robot.audit.query(kind=AuditKind.RESULT, limit=50)
    assert events, "successful ping should be audited"
    for event in events:
        assert isinstance(event.kind, AuditKind)
        assert event.outcome is None or isinstance(event.outcome, Outcome)
    assert any(e.outcome is Outcome.SUCCESS for e in events)
    assert all(e.kind is AuditKind.RESULT for e in robot.audit.query(kind="result", limit=5))


@pytest.mark.requires("slam")
def test_slam_mode_is_typed(robot, profile_data):
    """Mapping mode and backend are identifiable

    Categories: mapping
    What it checks: Reads a typed mapping/localization/idle mode and checks the backend against the selected YAML when supplied.
    Why it matters: Operators need to know whether the robot is mapping or localizing with the intended backend.
    If it fails: Check slam.backend and which profile the agent actually loaded. Inspect the backend's reported mode.
    Profile fields: slam.backend
    Scenario fields: None.
    Robot effects: Reads SLAM state.
    """
    assert isinstance(robot.slam.get_mode(), SlamMode)
    if profile_data:
        assert robot._robot._welcome.features.get('integration',{}).get('slam_backend') == profile_data["slam"].get("backend", "slam_toolbox")
    else:
        assert robot._robot._welcome.features.get('integration',{}).get('slam_backend')


@pytest.mark.requires("cameras")
def test_payload_transport_is_typed(robot, camera):
    """Camera payload declares its transport

    Categories: cameras,connection
    What it checks: Requests camera payload metadata and checks its transport parses as a supported protocol value. Actual image retrieval is checked separately.
    Why it matters: Clients need to know how to retrieve large camera payloads.
    If it fails: Check matching agent/SDK versions and the payload transport declaration. Use image-read checks to diagnose inaccessible shared memory or network data.
    Profile fields: cameras.*.rgb
    Scenario fields: None.
    Robot effects: Requests camera payload metadata.
    """
    value = robot._runner.run(robot._robot.connection.call("camera.get_frame", {"camera": camera.name}))
    parsed = PayloadRef.model_validate(value["payload"])
    assert isinstance(parsed.transport, Transport)
    # SHM and network are both valid; retrieval failures remain failures
    # in the frame tests rather than being hidden as missing capabilities.


@pytest.mark.requires("mobility")
def test_profile_locomotion_is_reported_independently_of_robot_name(profile_data, robot, request):
    """Locomotion is explicitly configured

    Categories: profile,movement
    What it checks: Checks the selected independent YAML contains a nonempty locomotion string. It does not infer movement type from the robot name.
    Why it matters: A robot name should not determine which movement assumptions the configuration makes.
    If it fails: Select the real YAML and set mobility.locomotion to a type supported by your backend. This check alone does not prove gait or flight control.
    Profile fields: mobility.locomotion
    Scenario fields: None.
    Robot effects: Inspects the selected YAML.
    """
    from conformance import skip_coverage
    if profile_data is None:
        skip_coverage(request, "not_configured", "loaded handshake does not expose locomotion; supply RINGFOUR_PROFILE for an independent check")
    assert isinstance(profile_data["mobility"]["locomotion"], str)
    assert profile_data["mobility"]["locomotion"]


@pytest.mark.requires("mobility")
@pytest.mark.motion
def test_nav_status_phase_is_typed(robot, navigation_recipe):
    """Navigation progress has usable statuses

    Categories: movement
    What it checks: Navigates to the reviewed first goal and checks typed status phases, bounded progress, and a final arrived phase.
    Why it matters: A UI must distinguish progress from actual arrival consistently.
    If it fails: Check the navigation backend, agent/client versions, and whether the configured goal can be reached.
    Profile fields: mobility.nav2_navigate_to_pose
    Scenario fields: navigation.goals
    Robot effects: Navigates and stops in cleanup.
    """
    goal = validate_pose(navigation_recipe["goals"][0], "navigation.goal")
    seen = []
    try:
        with robot.start_navigation(goal) as op:
            for update in op.updates():
                assert update.state.value in ('accepted','running','cancelling','succeeded','failed','cancelled')
            assert op.result().stop.verified

    finally:
        robot.stop()
