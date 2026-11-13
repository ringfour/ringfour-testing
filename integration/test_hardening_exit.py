"""Portable hardening integration checks; robot eligibility comes from settings."""
import pytest

from ringfour import Controller, JointTrajectory
from ringfour.errors import SafetyViolation
from conformance import skip_coverage

pytestmark = [pytest.mark.integration, pytest.mark.sim, pytest.mark.requires("groups"), pytest.mark.motion]


def test_inhibit_rejects_all_group_motion(request, session_group, welcome, debug_enabled):
    """A02: inhibited group motion is rejected before dispatch

    Categories: manipulation,safety
    What it checks: Opens an eligible reviewed group, engages the simulated inhibit, attempts trajectory, hold, tool and additional acquisition RPCs, and verifies rejection. Restores the original inhibit state and controller scope.
    Why it matters: A motion method omitted from central admission could move while other methods report the robot is inhibited.
    If it fails: Inspect agent method descriptors and the production dispatcher; do not loosen a robot profile tolerance.
    Profile fields: groups.*.controllers
    Scenario fields: groups.*.probe;debug
    Robot effects: Switches one reviewed controller scope and temporarily engages a simulated inhibit; submits no accepted motion.
    """
    if not welcome.features.get("control_session_ids"):
        skip_coverage(request, "not_configured", "selected agent lacks hardening session-ID admission")
    group = session_group
    robot = group._owner
    def call(method, params=None):
        return robot._runner.run(robot._robot.connection.call(method, params))
    original = call("status.get").get("estop", False)
    if original:
        pytest.skip("robot already inhibited; preserve existing state")
    with group.control() as control:
        scope = {"group": group.name, "session_id": control._aio._session_id}
        commands = {
            "group.start_trajectory": {**scope, "trajectory": JointTrajectory.from_waypoints([group.configuration(group.get_joint_positions())], step_duration=1.).model_dump(mode="json")},
            "group.hold": scope,
            "group.start_tool": {"group": group.name, "position": 0.},
            "group.control_start": {"group": group.name, "role": str(Controller.TRAJECTORY)},
        }
        try:
            call("debug.set_estop", {"engaged": True})
            for method, params in commands.items():
                with pytest.raises(SafetyViolation) as rejected:
                    call(method, params)
                assert rejected.value.reason == "estop"
        finally:
            call("debug.set_estop", {"engaged": original})
