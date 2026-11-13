"""Phase 3 unit tests: spatial math and the teleop manager (FakeRos) —
no ROS, no sim. The sim-backed teleop/TF behavior lives in
test_phase3_exit.py.
"""

from __future__ import annotations

import asyncio
import math

import pytest

from ringfour.enums import AuditKind, Outcome
from ringfour.errors import FrameMismatch
from ringfour.models import Point, Pose, Quaternion, Transform, Vec3

from ringfour_agent.audit import AuditLog
from ringfour_agent.profile import load_profile
from ringfour_agent.safety import MotionLock
from ringfour_agent.server import RpcError
from ringfour_agent.teleop import TeleopManager

from .conftest import SDK_DIR, PROFILES_DIR

FULL_PROFILE = PROFILES_DIR / "arm.yaml"


# -- spatial math ----------------------------------------------------------------

def test_quaternion_euler_roundtrip():
    """Quaternion euler roundtrip

    Categories: sdk
    What it checks: Verifies that quaternion euler roundtrip. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that velocity clamping, watchdogs, frame resolution, and motion ownership work without hardware.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the teleop control code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    for roll, pitch, yaw in [(0, 0, 0), (0.3, -0.2, 1.1), (-1.0, 0.4, -2.5)]:
        q = Quaternion.from_euler(roll, pitch, yaw)
        r2, p2, y2 = q.to_euler()
        assert math.isclose(r2, roll, abs_tol=1e-9)
        assert math.isclose(p2, pitch, abs_tol=1e-9)
        assert math.isclose(y2, yaw, abs_tol=1e-9)


def test_pose_yaw_shorthand():
    """Pose yaw shorthand

    Categories: sdk
    What it checks: Verifies that pose yaw shorthand. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that velocity clamping, watchdogs, frame resolution, and motion ownership work without hardware.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the teleop control code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    pose = Pose(x=1, y=2, yaw=math.pi / 2, frame="map")
    assert math.isclose(pose.yaw, math.pi / 2, abs_tol=1e-9)
    with pytest.raises(Exception):
        Pose(x=0, y=0, yaw=1.0, q=Quaternion(), frame="map")


def test_pose_distance_and_bearing():
    """Pose distance and bearing

    Categories: sdk
    What it checks: Verifies that pose distance and bearing. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that velocity clamping, watchdogs, frame resolution, and motion ownership work without hardware.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the teleop control code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    a = Pose(x=0, y=0, frame="map")
    b = Pose(x=3, y=4, frame="map")
    assert math.isclose(a.distance_to(b), 5.0)
    assert math.isclose(a.angle_to(b), math.atan2(4, 3))
    c = Pose(x=0, y=0, frame="odom")
    with pytest.raises(FrameMismatch):
        a.distance_to(c)


def test_pose_shifted_in_own_frame():
    # Facing +y (yaw 90deg): "forward 1m" moves +y, dyaw adds up.
    """Pose shifted in own frame

    Categories: sdk
    What it checks: Verifies that pose shifted in own frame. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that velocity clamping, watchdogs, frame resolution, and motion ownership work without hardware.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the teleop control code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    pose = Pose(x=1, y=1, yaw=math.pi / 2, frame="map")
    moved = pose.shifted(dx=1.0, dyaw=math.pi / 2)
    assert math.isclose(moved.x, 1.0, abs_tol=1e-9)
    assert math.isclose(moved.y, 2.0, abs_tol=1e-9)
    assert math.isclose(moved.yaw, math.pi, abs_tol=1e-9)
    assert moved.frame == "map"


def test_transform_apply():
    # child frame sits at (1, 0), rotated 90deg: child (1, 0) -> parent (1, 1)
    """Transform apply

    Categories: sdk
    What it checks: Verifies that transform apply. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that velocity clamping, watchdogs, frame resolution, and motion ownership work without hardware.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the teleop control code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    t = Transform(
        translation=Vec3(x=1.0, y=0.0, z=0.0),
        rotation=Quaternion.from_euler(0, 0, math.pi / 2),
        parent="map", child="base_link",
    )
    moved = t.apply(Point(x=1.0, y=0.0, frame="base_link"))
    assert math.isclose(moved.x, 1.0, abs_tol=1e-9)
    assert math.isclose(moved.y, 1.0, abs_tol=1e-9)
    assert moved.frame == "map"
    with pytest.raises(FrameMismatch):
        t.apply(Point(x=0, y=0, frame="map"))


# -- teleop manager (FakeRos) -------------------------------------------------------


class FakeRos:
    """Records published twists; that's all the teleop manager needs."""

    def __init__(self) -> None:
        self.published: list[tuple[float, float, float]] = []

    def publish_twist(self, linear, angular, lateral=0.0):
        self.published.append((linear, angular, lateral))

    def publish_zero(self):
        self.published.append((0.0, 0.0, 0.0))


@pytest.fixture()
def teleop(tmp_path):
    profile = load_profile(FULL_PROFILE)
    audit = AuditLog("testbot", None, directory=tmp_path)
    ros = FakeRos()
    manager = TeleopManager(MotionLock(), audit, ros, profile, allow_unverified_stops=True)
    yield manager, ros, audit
    audit.close()


def test_drive_clamped_to_profile_limits(teleop):
    """Drive clamped to profile limits

    Categories: sdk
    What it checks: Verifies that drive clamped to profile limits. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that velocity clamping, watchdogs, frame resolution, and motion ownership work without hardware.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the teleop control code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    manager, ros, _ = teleop

    async def scenario():
        granted = manager.start(client_id=1, requested_watchdog=0.5)
        assert granted == 0.5
        # base101_arm: max_linear 0.5, max_angular 1.5
        manager.drive(1, linear=2.0, angular=-9.0)
        manager.end(1)

    asyncio.run(scenario())
    assert ros.published[0] == (0.5, -1.5, 0.0)
    assert ros.published[-1] == (0.0, 0.0, 0.0)  # end() zeroes


def test_watchdog_clamped_to_profile_max(teleop):
    """Watchdog clamped to profile max

    Categories: sdk
    What it checks: Verifies that watchdog clamped to profile max. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that velocity clamping, watchdogs, frame resolution, and motion ownership work without hardware.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the teleop control code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    manager, _, _ = teleop

    async def scenario():
        # base101_arm teleop.max_watchdog is 1.0; ask for more, get 1.0
        granted = manager.start(client_id=1, requested_watchdog=10.0)
        manager.end(1)
        return granted

    assert asyncio.run(scenario()) == 1.0


def test_lateral_rejected_on_differential(teleop):
    """Lateral rejected on differential

    Categories: sdk
    What it checks: Verifies that lateral rejected on differential. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that velocity clamping, watchdogs, frame resolution, and motion ownership work without hardware.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the teleop control code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    manager, _, _ = teleop

    async def scenario():
        manager.start(client_id=1, requested_watchdog=0.5)
        with pytest.raises(RpcError) as excinfo:
            manager.drive(1, linear=0.1, angular=0.0, lateral=0.2)
        assert excinfo.value.error_type == "CapabilityNotSupported"
        manager.end(1)

    asyncio.run(scenario())


def test_second_client_cannot_start(teleop):
    """Second client cannot start

    Categories: sdk
    What it checks: Verifies that second client cannot start. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that velocity clamping, watchdogs, frame resolution, and motion ownership work without hardware.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the teleop control code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    manager, _, _ = teleop

    async def scenario():
        manager.start(client_id=1, requested_watchdog=0.5)
        with pytest.raises(RpcError) as excinfo:
            manager.start(client_id=2, requested_watchdog=0.5)
        assert excinfo.value.error_type == "SafetyViolation"
        manager.end(1)
        # after release, client 2 can start
        manager.start(client_id=2, requested_watchdog=0.5)
        manager.end(2)

    asyncio.run(scenario())


def test_drive_without_session_rejected(teleop):
    """Drive without session rejected

    Categories: sdk
    What it checks: Verifies that drive without session rejected. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that velocity clamping, watchdogs, frame resolution, and motion ownership work without hardware.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the teleop control code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    manager, _, _ = teleop

    async def scenario():
        with pytest.raises(RpcError) as excinfo:
            manager.drive(7, linear=0.1, angular=0.0)
        assert excinfo.value.error_type == "SafetyViolation"

    asyncio.run(scenario())


def test_watchdog_halts_on_silence(teleop):
    """Watchdog halts on silence

    Categories: sdk
    What it checks: Verifies that watchdog halts on silence. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that velocity clamping, watchdogs, frame resolution, and motion ownership work without hardware.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the teleop control code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    manager, ros, audit = teleop

    async def scenario():
        manager.start(client_id=1, requested_watchdog=0.2)
        manager.drive(1, linear=0.3, angular=0.0)
        await asyncio.sleep(0.5)  # silence > watchdog
        manager.end(1)

    asyncio.run(scenario())
    assert (0.0, 0.0, 0.0) in ros.published, "watchdog must zero velocity"
    halts = [e for e in audit.query(kind="safety")
             if e.detail.get("reason") == "teleop watchdog"]
    assert halts, "watchdog halt must be audited"


def test_disconnect_ends_session_and_zeroes(teleop):
    """Disconnect ends session and zeroes

    Categories: sdk
    What it checks: Verifies that disconnect ends session and zeroes. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that velocity clamping, watchdogs, frame resolution, and motion ownership work without hardware.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the teleop control code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    manager, ros, audit = teleop

    async def scenario():
        manager.start(client_id=1, requested_watchdog=0.5)
        manager.drive(1, linear=0.3, angular=0.0)
        manager.on_disconnect(1)
        # lock must be free again
        manager.start(client_id=2, requested_watchdog=0.5)
        manager.end(2)

    asyncio.run(scenario())
    assert ros.published[-1] == (0.0, 0.0, 0.0)
    # The reason moved out of the free-text outcome and into detail
    # (Phase 6 enum migration); outcome is now filterable.
    events = audit.query(kind=AuditKind.SAFETY)
    assert any(e.detail.get("reason") == "disconnect"
               and e.outcome is Outcome.SUCCESS for e in events)
