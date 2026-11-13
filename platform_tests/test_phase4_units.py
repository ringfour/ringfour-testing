"""Phase 4 unit tests: shm payload transport, frame cache + deprojection
math, cloud voxelization, watch registry semantics, and the full
watch/halt/event loop over the wire — no ROS, no sim.

Sim-backed equivalents live in test_phase4_exit.py.
"""

from __future__ import annotations

import asyncio
import os
import threading
import time

import numpy as np
import pytest
import yaml

from ringfour import Robot
from ringfour.errors import FrameExpired, RingfourError
from ringfour.enums import Controller, GroupKind
from ringfour.models import CameraIntrinsics, Transform
from ringfour.models.sensing import PayloadRef
from ringfour.payloads import fetch_array

from ringfour_agent.access import AccessPolicy
from ringfour_agent.bootstrap import build_agent
from ringfour_agent.profile import load_profile
from ringfour_agent.sensing import FrameCache, FrameRecord, deproject, make_cloud
from ringfour_agent.server import RpcError
from ringfour_agent.shm import ShmStore
from ringfour_agent.watches import WatchRegistry

from .conftest import SDK_DIR, _free_tcp_port, PROFILES_DIR

FULL_PROFILE = PROFILES_DIR / "arm.yaml"
BASE_PROFILE = PROFILES_DIR / "base.yaml"

K = CameraIntrinsics(fx=100.0, fy=100.0, cx=320.0, cy=240.0,
                     width=640, height=480)


# -- profile additions ---------------------------------------------------------

def test_profile_phase4_fields():
    """Profile phase4 fields

    Categories: sdk
    What it checks: Verifies that profile phase4 fields. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that payload retrieval, image/depth geometry, streams, and robot-side watches behave consistently.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the camera data and safety watches code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    profile = load_profile(FULL_PROFILE)
    base = profile.spec.cameras["base"]
    assert base.depth == "/base_camera/depth_image"
    assert base.frame_ttl == 30.0          # ruling: 30 s default
    # optical: true is REQUIRED on base101_arm — the sim and the real
    # realsense/depthai drivers all stamp camera_optical_frame. Without it
    # the agent re-applies the optical->link rotation and deproject/get_cloud
    # come out 90 degrees wrong with no error (current-state.md §3).
    assert base.optical is True
    arm = profile.spec.groups["arm"]
    assert arm.kind is GroupKind.ARM
    assert arm.tip == "arm_wrist_flange"
    # No joints: in the profile any more - they are derived from the
    # controller's claimed interfaces at startup (6.1). The profile field
    # survives only as an override.
    assert arm.joints == ()
    assert arm.controllers[Controller.STREAM] == "arm_controller"
    assert "watches" in profile.info.capabilities
    assert profile.spec.joint_states == "/joint_states"


def test_profile_frame_ttl_override(tmp_path):
    """Profile frame ttl override

    Categories: sdk
    What it checks: Verifies that profile frame ttl override. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that payload retrieval, image/depth geometry, streams, and robot-side watches behave consistently.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the camera data and safety watches code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    data = yaml.safe_load(BASE_PROFILE.read_text())
    data["cameras"]["base"]["frame_ttl"] = 5.0
    path = tmp_path / "p.yaml"
    path.write_text(yaml.safe_dump(data))
    assert load_profile(path).spec.cameras["base"].frame_ttl == 5.0


# -- shm transport ----------------------------------------------------------------

def test_shm_roundtrip_zero_copy():
    """Shm roundtrip zero copy

    Categories: sdk
    What it checks: Verifies that shm roundtrip zero copy. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that payload retrieval, image/depth geometry, streams, and robot-side watches behave consistently.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the camera data and safety watches code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    store = ShmStore("unit-test")
    try:
        sent = np.arange(640 * 480 * 3, dtype=np.uint8).reshape(480, 640, 3)
        ref = store.put(sent)
        assert ref.transport == "shm"
        got = fetch_array(ref)
        assert got.shape == (480, 640, 3) and got.dtype == np.uint8
        assert not got.flags.writeable            # read-only view
        assert np.array_equal(got, sent)
        # POSIX semantics: the mapping outlives the unlink.
        store.close()
        assert not os.path.exists(f"/dev/shm/{ref.name}")
        assert int(got[100, 100, 0]) == int(sent[100, 100, 0])
    finally:
        store.close()


def test_shm_expired_payload_raises_frame_expired():
    """Shm expired payload raises frame expired

    Categories: sdk
    What it checks: Verifies that shm expired payload raises frame expired. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that payload retrieval, image/depth geometry, streams, and robot-side watches behave consistently.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the camera data and safety watches code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    store = ShmStore("unit-test")
    ref = store.put(np.zeros(8, dtype=np.float32), ttl=0.0)
    store.put(np.zeros(8, dtype=np.float32))  # any put sweeps expired
    with pytest.raises(FrameExpired):
        fetch_array(ref)
    store.close()


def test_shm_budget_evicts_oldest():
    """Shm budget evicts oldest

    Categories: sdk
    What it checks: Verifies that shm budget evicts oldest. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that payload retrieval, image/depth geometry, streams, and robot-side watches behave consistently.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the camera data and safety watches code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    store = ShmStore("unit-test", budget=4096)
    first = store.put(np.zeros(2048, dtype=np.uint8))
    store.put(np.zeros(2048, dtype=np.uint8))
    store.put(np.zeros(2048, dtype=np.uint8))  # pushes total over budget
    with pytest.raises(FrameExpired):
        fetch_array(first)
    store.close()


def test_fetch_rejects_path_tricks():
    """Fetch rejects path tricks

    Categories: sdk
    What it checks: Verifies that fetch rejects path tricks. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that payload retrieval, image/depth geometry, streams, and robot-side watches behave consistently.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the camera data and safety watches code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    ref = PayloadRef(name="../etc/passwd", dtype="uint8", shape=(4,), size=4)
    with pytest.raises(RingfourError):
        fetch_array(ref)


# -- frame cache + deprojection ----------------------------------------------------

def _record(**kwargs) -> FrameRecord:
    depth = kwargs.pop("depth", np.full((480, 640), 2.0, dtype=np.float32))
    return FrameRecord(
        camera=kwargs.pop("camera", "base"),
        stamp=kwargs.pop("stamp", 1.0),
        tf_frame=kwargs.pop("tf_frame", "base_camera"),
        optical=kwargs.pop("optical", False),
        intrinsics=kwargs.pop("intrinsics", K),
        depth=depth,
        rgb=kwargs.pop("rgb", None),
        transforms=kwargs.pop("transforms", {}),
    )


def test_frame_cache_ttl_expiry():
    """Frame cache ttl expiry

    Categories: sdk
    What it checks: Verifies that frame cache ttl expiry. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that payload retrieval, image/depth geometry, streams, and robot-side watches behave consistently.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the camera data and safety watches code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    cache = FrameCache()
    fid = cache.store(_record(), ttl=0.05)
    assert cache.get(fid).camera == "base"
    time.sleep(0.08)
    with pytest.raises(RpcError) as err:
        cache.get(fid)
    assert err.value.error_type == "FrameExpired"


def test_deproject_center_pixel_optical_and_link():
    """Deproject center pixel optical and link

    Categories: sdk
    What it checks: Verifies that deproject center pixel optical and link. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that payload retrieval, image/depth geometry, streams, and robot-side watches behave consistently.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the camera data and safety watches code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    optical = deproject(_record(optical=True), (320, 240), None)
    assert optical == pytest.approx(
        {"x": 0.0, "y": 0.0, "z": 2.0, "frame": "base_camera"})
    # Same pixel through a physical-link camera frame: z(out) -> x(forward)
    link = deproject(_record(optical=False), (320, 240), None)
    assert link == pytest.approx(
        {"x": 2.0, "y": 0.0, "z": 0.0, "frame": "base_camera"})
    # Pixel right of center is +x optical -> -y in the link frame
    right = deproject(_record(optical=False), (420, 240), None)
    assert right["y"] == pytest.approx(-2.0)
    assert right["x"] == pytest.approx(2.0)


def test_deproject_into_snapshotted_frame():
    """Deproject into snapshotted frame

    Categories: sdk
    What it checks: Verifies that deproject into snapshotted frame. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that payload retrieval, image/depth geometry, streams, and robot-side watches behave consistently.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the camera data and safety watches code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    t = Transform.model_validate({
        "translation": {"x": 1.0, "y": 0.5, "z": 0.0},
        "rotation": {"x": 0, "y": 0, "z": 0, "w": 1},
        "parent": "map", "child": "base_camera",
    })
    record = _record(transforms={"map": t})
    point = deproject(record, (320, 240), "map")
    assert point == pytest.approx(
        {"x": 3.0, "y": 0.5, "z": 0.0, "frame": "map"})
    with pytest.raises(RpcError):
        deproject(record, (320, 240), "odom")  # not snapshotted


def test_deproject_failure_modes():
    """Deproject failure modes

    Categories: sdk
    What it checks: Verifies that deproject failure modes. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that payload retrieval, image/depth geometry, streams, and robot-side watches behave consistently.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the camera data and safety watches code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    with pytest.raises(RpcError):  # out of bounds
        deproject(_record(), (1000, 10), None)
    holed = np.full((480, 640), 2.0, dtype=np.float32)
    holed[240, 320] = 0.0
    with pytest.raises(RpcError):  # no return at pixel
        deproject(_record(depth=holed), (320, 240), None)
    with pytest.raises(RpcError):  # no depth stream at all
        deproject(_record(depth=None), (320, 240), None)


# -- clouds -----------------------------------------------------------------------

def test_make_cloud_voxel_budget_and_colors():
    """Make cloud voxel budget and colors

    Categories: sdk
    What it checks: Verifies that make cloud voxel budget and colors. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that payload retrieval, image/depth geometry, streams, and robot-side watches behave consistently.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the camera data and safety watches code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    depth = np.full((480, 640), 1.0, dtype=np.float32)
    rgb = np.full((480, 640, 3), 37, dtype=np.uint8)
    fine_pts, _ = make_cloud(depth, None, K, voxel=0.005, optical=True)
    coarse_pts, colors = make_cloud(depth, rgb, K, voxel=0.1, optical=True)
    assert fine_pts.dtype == np.float32 and fine_pts.shape[1] == 3
    assert coarse_pts.shape[0] < fine_pts.shape[0]   # voxel is a size budget
    assert colors is not None and colors.shape == coarse_pts.shape
    assert colors.dtype == np.uint8 and int(colors[0, 0]) == 37
    # plane at 1 m: every point's optical z is 1
    assert np.allclose(coarse_pts[:, 2], 1.0, atol=1e-5)
    with pytest.raises(RpcError):
        make_cloud(depth, None, K, voxel=0.0, optical=True)


# -- watch registry ----------------------------------------------------------------

def _eval(reg, value, fired, t):
    reg.evaluate(lambda _path: value, lambda w, v: fired.append((w.id, v)),
                 now_mono=t, now_wall=1000.0 + t)


def test_watch_latches_and_records_edges():
    """Watch latches and records edges

    Categories: sdk
    What it checks: Verifies that watch latches and records edges. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that payload retrieval, image/depth geometry, streams, and robot-side watches behave consistently.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the camera data and safety watches code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    reg = WatchRegistry()
    fired: list = []
    wid = reg.start(owner=1, path="battery.level", above=None, below=15.0,
                    outside=None, stop=False, debounce=0.0, lifetime=None)
    _eval(reg, 50.0, fired, t=0.0)
    assert reg.poll(wid, 1)["triggered"] is False
    _eval(reg, 10.0, fired, t=1.0)         # edge 1
    _eval(reg, 9.0, fired, t=1.1)          # same episode, no new edge
    _eval(reg, 50.0, fired, t=2.0)         # recovers; stays latched
    _eval(reg, 5.0, fired, t=3.0)          # edge 2
    state = reg.poll(wid, 1)
    assert state["triggered"] is True       # latched through recovery
    assert len(state["events"]) == 2
    assert state["triggered_at"] == pytest.approx(1001.0)
    assert [v for _id, v in fired] == [10.0, 5.0]


def test_watch_debounce_requires_holding():
    """Watch debounce requires holding

    Categories: sdk
    What it checks: Verifies that watch debounce requires holding. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that payload retrieval, image/depth geometry, streams, and robot-side watches behave consistently.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the camera data and safety watches code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    reg = WatchRegistry()
    fired: list = []
    wid = reg.start(owner=1, path="p", above=2.0, below=None, outside=None,
                    stop=False, debounce=0.5, lifetime=None)
    _eval(reg, 3.0, fired, t=0.0)
    _eval(reg, 1.0, fired, t=0.3)           # dipped before debounce elapsed
    _eval(reg, 3.0, fired, t=0.4)
    _eval(reg, 3.0, fired, t=0.8)           # held only 0.4 s
    assert fired == []
    _eval(reg, 3.0, fired, t=1.0)           # held 0.6 s -> fires
    assert len(fired) == 1
    assert reg.poll(wid, 1)["triggered"] is True


def test_watch_outside_band():
    """Watch outside band

    Categories: sdk
    What it checks: Verifies that watch outside band. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that payload retrieval, image/depth geometry, streams, and robot-side watches behave consistently.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the camera data and safety watches code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    reg = WatchRegistry()
    fired: list = []
    reg.start(owner=1, path="p", above=None, below=None,
              outside=(-1.0, 1.0), stop=False, debounce=0.0, lifetime=None)
    _eval(reg, 0.5, fired, t=0.0)
    assert fired == []
    _eval(reg, 1.5, fired, t=1.0)
    assert len(fired) == 1


def test_watch_validation_and_ownership():
    """Watch validation and ownership

    Categories: sdk
    What it checks: Verifies that watch validation and ownership. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that payload retrieval, image/depth geometry, streams, and robot-side watches behave consistently.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the camera data and safety watches code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    reg = WatchRegistry()
    with pytest.raises(RpcError):
        reg.start(owner=1, path="p", above=1.0, below=2.0, outside=None,
                  stop=False, debounce=0.0, lifetime=None)
    with pytest.raises(RpcError):
        reg.start(owner=1, path="p", above=None, below=None, outside=None,
                  stop=False, debounce=0.0, lifetime=None)
    wid = reg.start(owner=1, path="p", above=1.0, below=None, outside=None,
                    stop=False, debounce=0.0, lifetime=None)
    with pytest.raises(RpcError):
        reg.poll(wid, owner=2)  # not the owner


def test_disconnect_keeps_stop_watches_for_lifetime():
    """Disconnect keeps stop watches for lifetime

    Categories: sdk
    What it checks: Verifies that disconnect keeps stop watches for lifetime. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that payload retrieval, image/depth geometry, streams, and robot-side watches behave consistently.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the camera data and safety watches code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    reg = WatchRegistry()
    fired: list = []
    plain = reg.start(owner=1, path="p", above=1.0, below=None, outside=None,
                      stop=False, debounce=0.0, lifetime=None)
    armed = reg.start(owner=1, path="p", above=1.0, below=None, outside=None,
                      stop=True, debounce=0.0, lifetime=2.0)
    reg.on_disconnect(1)
    base = time.monotonic()
    _eval(reg, 5.0, fired, t=base + 1.0)    # inside lifetime: still firing
    assert [wid for wid, _v in fired] == [armed]
    with pytest.raises(RpcError):
        reg.poll(plain, 1)                   # plain watch died on disconnect
    fired.clear()
    _eval(reg, 5.0, fired, t=base + 3.0)    # past lifetime: removed
    assert fired == []


# -- the full loop over the wire (agent stack, no ROS) ------------------------------

@pytest.fixture()
def watch_agent(tmp_path):
    """Real agent stack, no ROS, with an injectable battery value."""
    profile = load_profile(FULL_PROFILE)
    port = _free_tcp_port()
    server, ctx = build_agent(
        profile, unix_path=None, host="127.0.0.1", port=port,
        audit_dir=tmp_path / "audit",
        access_policy=AccessPolicy(trusted_tcp=True),
     allow_unverified_stops=True)
    holder = {"battery.level": None}
    ctx.watch_paths = {"battery.level": lambda: holder["battery.level"]}
    loop = asyncio.new_event_loop()
    thread = threading.Thread(target=loop.run_forever, daemon=True)
    thread.start()
    asyncio.run_coroutine_threadsafe(server.start(), loop).result(5)
    yield f"ws://127.0.0.1:{port}", holder, ctx
    asyncio.run_coroutine_threadsafe(server.close(), loop).result(5)
    loop.call_soon_threadsafe(loop.stop)
    thread.join(timeout=2)
    ctx.audit.close()


def test_watch_unwatchable_path_rejected(watch_agent):
    """Watch unwatchable path rejected

    Categories: sdk
    What it checks: Verifies that watch unwatchable path rejected. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that payload retrieval, image/depth geometry, streams, and robot-side watches behave consistently.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the camera data and safety watches code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    uri, _holder, _ctx = watch_agent
    with Robot(uri) as robot:
        with pytest.raises(RingfourError, match="not watchable"):
            robot._watch_path("flux.capacitor", below=1.0).__enter__()



def test_estop_event_reaches_subscriber(watch_agent):
    """Estop event reaches subscriber

    Categories: sdk
    What it checks: Verifies that estop event reaches subscriber. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that payload retrieval, image/depth geometry, streams, and robot-side watches behave consistently.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the camera data and safety watches code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    uri, _holder, _ctx = watch_agent
    with Robot(uri) as robot, Robot(uri) as observer:
        events = observer.events()
        robot._runner.run(robot._robot.connection.call(
            "debug.set_estop", {"engaged": True}))
        event = next(events)
        assert event.kind == "estop" and event.data["engaged"] is True
        robot._runner.run(robot._robot.connection.call(
            "debug.set_estop", {"engaged": False}))
        events.close()
        assert robot.get_estop_engaged() is False


def test_status_get_without_ros(watch_agent):
    """Status get without ros

    Categories: sdk
    What it checks: Verifies that status get without ros. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that payload retrieval, image/depth geometry, streams, and robot-side watches behave consistently.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the camera data and safety watches code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    uri, _holder, _ctx = watch_agent
    with Robot(uri) as robot:
        assert robot.get_battery() is None   # no ROS: no battery data
        assert robot.get_estop_engaged() is False
