"""Phase 2 unit tests: profile validation, capability sugar, task
lifecycle, audit log, safety layer, motion lock, and the aio client —
no ROS, no sim.

Sim-backed equivalents live in test_phase2_exit.py; these cover the same
code paths fast, plus the failure modes that are awkward to stage in sim.
"""

from __future__ import annotations

import asyncio
import json
import threading

import pytest
import yaml

from ringfour import Pose, Robot, aio
from ringfour.enums import AuditKind, Outcome
from ringfour.errors import (
    AmbiguousCapability,
    RingfourError,
    CapabilityNotSupported,
    ExecutionFailed,
    SafetyViolation,
)
from ringfour_agent.access import AccessPolicy
from ringfour_agent.models.profile import RobotProfile

from ringfour_agent.audit import AuditLog
from ringfour_agent.bootstrap import build_agent
from ringfour_agent.profile import ProfileError, load_profile
from ringfour_agent.safety import MotionLock

from .conftest import SDK_DIR, STUB_PROFILE, _free_tcp_port, PROFILES_DIR

FULL_PROFILE = PROFILES_DIR / "arm.yaml"
BASE_PROFILE = PROFILES_DIR / "base.yaml"

# base101_arm.yaml (chassis + one mod101 arm). Retargeted from the retired
# base101_full.yaml, which also declared `joint_groups` (the tower, gone with
# the restructure) and `exploration` (removed: explore_lite is not installed
# or launched, and Phase 5 moved frontier finding to user-land Python).
# `groups` (not `manipulation`) since 6.1 merged the two sections.
FULL_CAPABILITIES = {
    "mobility", "groups", "sensors", "actions", "cameras", "lidar",
    "imu", "slam", "teleop", "watches", "places", "audit", "status",
}


# -- profile schema -------------------------------------------------------------

def test_full_profile_loads_with_expected_capabilities():
    """Full profile loads with expected capabilities

    Categories: sdk
    What it checks: Verifies that full profile loads with expected capabilities. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that handshakes, schema rules, task ownership, and audit data agree over the wire.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the connection, profiles, and audit code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    profile = load_profile(FULL_PROFILE)
    assert set(profile.info.capabilities) == FULL_CAPABILITIES
    assert profile.instances["arms"] == ("arm",)
    assert profile.instances["cameras"] == ("base", "arm_wrist")
    # The arm is a group of kind ARM, and `arms` is the reported subset.
    assert profile.instances["groups"] == ("arm",)
    # Retired sections must not come back as instance keys.
    assert "joint_groups" not in profile.instances
    assert "manipulation" not in profile.info.capabilities


def test_base_profile_has_no_groups():
    """Base profile has no groups

    Categories: sdk
    What it checks: Verifies that base profile has no groups. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that handshakes, schema rules, task ownership, and audit data agree over the wire.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the connection, profiles, and audit code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    profile = load_profile(BASE_PROFILE)
    assert "groups" not in profile.info.capabilities
    assert "arms" not in profile.instances


def test_profile_rejects_unknown_top_level_section(tmp_path):
    """Profile rejects unknown top level section

    Categories: sdk
    What it checks: Verifies that profile rejects unknown top level section. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that handshakes, schema rules, task ownership, and audit data agree over the wire.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the connection, profiles, and audit code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    data = yaml.safe_load(STUB_PROFILE.read_text())
    data["mobilty"] = data.pop("mobility")  # typo must not pass silently
    bad = tmp_path / "bad.yaml"
    bad.write_text(yaml.safe_dump(data))
    with pytest.raises(ProfileError):
        load_profile(bad)


def test_profile_rejects_wrong_protocol(tmp_path):
    """Profile rejects wrong protocol

    Categories: sdk
    What it checks: Verifies that profile rejects wrong protocol. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that handshakes, schema rules, task ownership, and audit data agree over the wire.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the connection, profiles, and audit code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    data = yaml.safe_load(STUB_PROFILE.read_text())
    data["protocol"] = 99
    bad = tmp_path / "bad.yaml"
    bad.write_text(yaml.safe_dump(data))
    with pytest.raises(ProfileError):
        load_profile(bad)


def test_profile_requires_frames(tmp_path):
    """Profile requires frames

    Categories: sdk
    What it checks: Verifies that profile requires frames. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that handshakes, schema rules, task ownership, and audit data agree over the wire.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the connection, profiles, and audit code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    data = yaml.safe_load(STUB_PROFILE.read_text())
    del data["frames"]
    bad = tmp_path / "bad.yaml"
    bad.write_text(yaml.safe_dump(data))
    with pytest.raises(ProfileError):
        load_profile(bad)


def test_profile_extra_keys_inside_sections_are_kept():
    # Sections tolerate config for future phases (extra="allow").
    """Profile extra keys inside sections are kept

    Categories: sdk
    What it checks: Verifies that profile extra keys inside sections are kept. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that handshakes, schema rules, task ownership, and audit data agree over the wire.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the connection, profiles, and audit code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    spec = RobotProfile.model_validate({
        "name": "x", "model": "y", "protocol": 1,
        "frames": {"map": "map", "odom": "odom", "base": "base_link"},
        "lidar": {"scan": "/scan", "future_option": 42},
    })
    assert spec.lidar.future_option == 42  # type: ignore[attr-defined]


# -- motion lock ------------------------------------------------------------------

def test_motion_lock_single_writer():
    """Motion lock single writer

    Categories: sdk
    What it checks: Verifies that motion lock single writer. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that handshakes, schema rules, task ownership, and audit data agree over the wire.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the connection, profiles, and audit code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    lock = MotionLock()
    assert lock.acquire(1)
    assert lock.acquire(1), "re-acquire by holder is fine"
    assert not lock.acquire(2), "second client must not take a held lock"
    assert lock.held_by_other(2)
    assert not lock.held_by_other(1)
    assert not lock.release(2), "only the holder releases"
    assert lock.release(1)
    assert lock.acquire(2), "released lock is takeable"


# -- audit log ---------------------------------------------------------------------

def test_audit_ring_query_filters(tmp_path):
    """Audit ring query filters

    Categories: sdk
    What it checks: Verifies that audit ring query filters. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that handshakes, schema rules, task ownership, and audit data agree over the wire.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the connection, profiles, and audit code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    audit = AuditLog("testbot", None, directory=tmp_path)
    audit.record("command", client=1, call="ping")
    audit.record("safety", client=1, call="mobility.navigate_to",
                 outcome="rejected", detail={"reason": "estop"})
    audit.record("result", client=1, call="ping", outcome="success",
                 task_id="t1")
    assert len(audit.query()) == 3
    safety = audit.query(kind="safety")
    assert len(safety) == 1
    assert safety[0].detail["reason"] == "estop"
    assert audit.query(task_id="t1")[0].kind == "result"
    cutoff = safety[0].stamp
    assert all(e.stamp >= cutoff for e in audit.query(since=cutoff))
    audit.close()


def test_audit_jsonl_persisted(tmp_path):
    """Audit jsonl persisted

    Categories: sdk
    What it checks: Verifies that audit jsonl persisted. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that handshakes, schema rules, task ownership, and audit data agree over the wire.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the connection, profiles, and audit code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    audit = AuditLog("testbot", None, directory=tmp_path)
    audit.record(AuditKind.ESTOP, outcome=Outcome.SUCCESS,
                 detail={"engaged": True})
    audit.close()
    line = json.loads((tmp_path / "audit.jsonl").read_text().splitlines()[-1])
    # The enum members ARE their wire values: no adapter serialized these.
    assert line["kind"] == "estop"
    assert line["outcome"] == "success"
    assert line["detail"] == {"engaged": True}


def test_audit_rotation(tmp_path):
    """Audit rotation

    Categories: sdk
    What it checks: Verifies that audit rotation. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that handshakes, schema rules, task ownership, and audit data agree over the wire.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the connection, profiles, and audit code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    audit = AuditLog("testbot", None, directory=tmp_path)
    audit._max_file_bytes = 2_000  # force tiny rotation threshold
    for index in range(50):
        audit.record("command", call=f"call_{index}", detail={"pad": "x" * 80})
    audit.close()
    assert (tmp_path / "audit.previous.jsonl").exists()
    assert (tmp_path / "audit.jsonl").stat().st_size <= 4_000


# -- full stack over the wire (sync + aio) -------------------------------------------


@pytest.fixture()
def full_agent(tmp_path):
    """The real agent stack with the flagship profile. No ROS."""
    profile = load_profile(FULL_PROFILE)
    port = _free_tcp_port()
    server, ctx = build_agent(
        profile,
        unix_path=str(tmp_path / "agent.sock"),
        host="127.0.0.1",
        port=port,
        audit_dir=tmp_path / "audit",
        access_policy=AccessPolicy(trusted_tcp=True),
     allow_unverified_stops=False)
    loop = asyncio.new_event_loop()
    thread = threading.Thread(target=loop.run_forever, daemon=True)
    thread.start()
    asyncio.run_coroutine_threadsafe(server.start(), loop).result(5)
    yield f"ws://127.0.0.1:{port}"
    asyncio.run_coroutine_threadsafe(server.close(), loop).result(5)
    loop.call_soon_threadsafe(loop.stop)
    thread.join(timeout=2)
    ctx.audit.close()


def test_capability_surface_and_sugar(full_agent):
    """Capability surface and sugar

    Categories: sdk
    What it checks: Verifies that capability surface and sugar. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that handshakes, schema rules, task ownership, and audit data agree over the wire.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the connection, profiles, and audit code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    with Robot(full_agent) as robot:
        assert robot.capabilities == FULL_CAPABILITIES
        assert set(robot.arms) == {"arm"}
        # base101_arm has exactly ONE arm, so the singular sugar resolves
        # instead of raising — that is spec §8's contract. The retired
        # base101_full had left+right, which is why this asserted
        # AmbiguousCapability before the retarget.
        assert robot.arm.name == "arm"
        # Two cameras (base, arm_wrist), so `robot.camera` is still ambiguous.
        with pytest.raises(AmbiguousCapability):
            _ = robot.camera
        assert robot.arms["arm"].name == "arm"


def test_capability_not_supported_on_stub(local_agent_uri):
    """Capability not supported on stub

    Categories: sdk
    What it checks: Verifies that capability not supported on stub. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that handshakes, schema rules, task ownership, and audit data agree over the wire.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the connection, profiles, and audit code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    with Robot(local_agent_uri) as robot:
        assert not robot.has("slam")
        with pytest.raises(CapabilityNotSupported):
            _ = robot.slam
        # stub has exactly one camera: singular sugar resolves
        assert robot.camera.name == "base"


@pytest.fixture()
def local_agent_uri(tmp_path):
    """Stub-profile agent (single camera, no arms) for sugar tests."""
    profile = load_profile(STUB_PROFILE)
    port = _free_tcp_port()
    server, ctx = build_agent(
        profile, unix_path=None, host="127.0.0.1", port=port,
        audit_dir=tmp_path / "audit",
        access_policy=AccessPolicy(trusted_tcp=True),
     allow_unverified_stops=False)
    loop = asyncio.new_event_loop()
    thread = threading.Thread(target=loop.run_forever, daemon=True)
    thread.start()
    asyncio.run_coroutine_threadsafe(server.start(), loop).result(5)
    yield f"ws://127.0.0.1:{port}"
    asyncio.run_coroutine_threadsafe(server.close(), loop).result(5)
    loop.call_soon_threadsafe(loop.stop)
    thread.join(timeout=2)
    ctx.audit.close()


def test_task_lifecycle_success(full_agent):
    """Task lifecycle success

    Categories: sdk
    What it checks: Verifies that task lifecycle success. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that handshakes, schema rules, task ownership, and audit data agree over the wire.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the connection, profiles, and audit code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    with Robot(full_agent) as robot:
        result = robot._runner.run(
            robot._robot.connection.call(
                "debug.run_task", {"duration": 0.3, "ticks": 3})
        )
        # consume via the public Task path: navigate_to_async-style handle
        # is exercised below; here verify the raw flow finished
        assert "task_id" in result


def test_navigate_to_without_ros_fails_cleanly(full_agent):
    # No ROS in unit tests: the handler refuses before starting a task
    # (Phase 5 made navigate_to real; sim coverage is in phase5_exit).
    """Navigate to without ros fails cleanly

    Categories: sdk
    What it checks: Verifies that navigate to without ros fails cleanly. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that handshakes, schema rules, task ownership, and audit data agree over the wire.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the connection, profiles, and audit code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    with Robot(full_agent) as robot:
        with pytest.raises(RingfourError, match="without a ROS interface"):
            robot.navigate_to(Pose(x=1.0, y=0.0, frame="map"))


def test_estop_rejects_navigate_and_is_audited(full_agent):
    """Estop rejects navigate and is audited

    Categories: sdk
    What it checks: Verifies that estop rejects navigate and is audited. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that handshakes, schema rules, task ownership, and audit data agree over the wire.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the connection, profiles, and audit code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    with Robot(full_agent) as robot:
        conn = robot._robot.connection
        robot._runner.run(conn.call("debug.set_estop", {"engaged": True}))
        with pytest.raises(SafetyViolation) as excinfo:
            robot.navigate_to(Pose(x=1.0, frame="map"))
        assert excinfo.value.reason == "estop"
        robot._runner.run(conn.call("debug.set_estop", {"engaged": False}))
        safety_events = robot.audit.query(kind="safety")
        assert safety_events, "rejection must appear in the audit log"
        assert safety_events[-1].detail["reason"] == "estop"
        assert safety_events[-1].call == "mobility.navigate_to"


def test_audit_tail_sees_live_events(full_agent):
    # tail() only delivers events recorded after subscription, so the
    # consumer must be subscribed before the prodder generates events.
    """Audit tail sees live events

    Categories: sdk
    What it checks: Verifies that audit tail sees live events. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that handshakes, schema rules, task ownership, and audit data agree over the wire.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the connection, profiles, and audit code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    async def scenario():
        async with await aio.Robot.connect(full_agent) as watcher, \
                   await aio.Robot.connect(full_agent) as prodder:
            seen: list[str] = []

            async def consume():
                async for event in watcher.audit.tail():
                    # subscription is active once the generator runs at all;
                    # the first event proves the stream works end to end.
                    seen.append(event.call or event.kind)
                    if "ping" in seen:
                        return

            consumer = asyncio.ensure_future(consume())
            # audit.subscribe is the first thing tail() awaits; once the
            # watcher's own subscribe command shows up in the audit query,
            # the subscription flag is set on the agent.
            while not any(e.call == "audit.subscribe"
                          for e in await prodder.audit.query()):
                await asyncio.sleep(0.02)
            await prodder.ping()
            await asyncio.wait_for(consumer, timeout=5.0)
            assert "ping" in seen

    asyncio.run(scenario())


def test_aio_client_end_to_end(full_agent):
    """Aio client end to end

    Categories: sdk
    What it checks: Verifies that aio client end to end. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that handshakes, schema rules, task ownership, and audit data agree over the wire.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the connection, profiles, and audit code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    async def scenario():
        async with await aio.Robot.connect(full_agent) as robot:
            assert robot.capabilities == FULL_CAPABILITIES
            assert (await robot.ping()) < 1.0
            # task lifecycle through the public async API
            task = await robot._tasks.start_task(
                "debug.run_task", {"duration": 0.3, "ticks": 3})
            result = await task.wait(5.0)
            assert result == {"ticks": 3}
            assert task.state == "succeeded"
            # deliberate failure maps to the right exception
            task = await robot._tasks.start_task(
                "debug.run_task", {"duration": 0.2, "ticks": 2, "fail": True})
            with pytest.raises(ExecutionFailed):
                await task.wait(5.0)
            # cancellation
            task = await robot._tasks.start_task(
                "debug.run_task", {"duration": 30, "ticks": 30})
            await task.cancel()
            assert (await task.wait(5.0)) is None
            assert task.state == "cancelled"

    asyncio.run(scenario())


def test_task_cancel_via_sync_api(full_agent):
    # A long-running task cancelled through the public Task handle
    # (navigate_to became real in Phase 5; debug.run_task stands in).
    """Task cancel via sync api

    Categories: sdk
    What it checks: Verifies that task cancel via sync api. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that handshakes, schema rules, task ownership, and audit data agree over the wire.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the connection, profiles, and audit code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    from ringfour.operations import Operation
    from ringfour.aio.operations import Operation as AioOperation
    from ringfour.errors import OperationCancelled
    with Robot(full_agent) as robot:
        aio_task = robot._runner.run(robot._robot._tasks.start_task(
            "debug.run_task", {"duration": 30.0, "ticks": 300}))
        task=Operation(robot._runner,AioOperation(aio_task,lambda i,r:r,lambda t:None,resource="debug"))
        task.cancel()
        with pytest.raises(OperationCancelled):task.result(5.)
        assert task.snapshot.state.value=="cancelled"
        assert task.close().criterion=="not-dispatched"


def test_multi_client_concurrent_reads(full_agent):
    """Multi client concurrent reads

    Categories: sdk
    What it checks: Verifies that multi client concurrent reads. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that handshakes, schema rules, task ownership, and audit data agree over the wire.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the connection, profiles, and audit code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    robots = [Robot(full_agent) for _ in range(3)]
    try:
        for robot in robots:
            assert robot.ping() < 1.0
            assert robot.capabilities == FULL_CAPABILITIES
    finally:
        for robot in robots:
            robot.close()
