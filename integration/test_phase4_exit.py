"""Every advertised camera, depth stream, lidar, and joint group."""
import math
import numpy as np
import pytest
from ringfour.errors import FrameExpired
from ringfour.enums import Transport
from ringfour.models import PayloadRef

pytestmark = pytest.mark.sim


@pytest.mark.requires("cameras")
def test_camera_frame_matches_its_intrinsics(camera, robot):
    """Camera image matches its calibration

    Categories: cameras
    What it checks: Validates the camera payload transport, fetches its RGB image, and checks uint8 RGB shape against its reported width/height, positive focal lengths, timestamp, and capture ID.
    Why it matters: Calibration must describe the actual image dimensions before vision or deprojection can work.
    If it fails: Check RGB and camera-info topics and the driver's published calibration. Correct the calibration or binding rather than forcing an expected resolution.
    Profile fields: cameras.*.rgb;cameras.*.info
    Scenario fields: None.
    Robot effects: Reads RGB images and calibration.
    """
    raw = robot._runner.run(robot._robot.connection.call("camera.get_frame", {"camera": camera.name}))
    assert isinstance(PayloadRef.model_validate(raw["payload"]).transport, Transport)
    frame = camera.get_frame()
    k = camera.get_intrinsics()
    assert frame.image.dtype == np.uint8
    assert frame.image.shape == (k.height, k.width, 3)
    assert k.width > 0 and k.height > 0 and k.fx > 0 and k.fy > 0
    assert frame.stamp >= 0
    assert frame.id


@pytest.mark.requires("cameras")
def test_camera_stream_yields_fresh_frames(camera):
    """Camera stream supplies new frames

    Categories: cameras
    What it checks: Collects three streamed frames at a requested 5 Hz and checks strictly increasing timestamps.
    Why it matters: Repeated or out-of-order captures can invalidate tracking and robot perception.
    If it fails: Check the image publisher, timestamps, stream delivery, and simulation/time synchronization. A silent stream fails its wall-clock budget.
    Profile fields: cameras.*.rgb
    Scenario fields: None.
    Robot effects: Streams camera images.
    """
    stamps = []
    stream = camera.stream(fps=5)
    try:
        for frame in stream:
            stamps.append(frame.stamp)
            if len(stamps) == 3:
                break
    finally:
        stream.close()
    assert len(stamps) >= 2
    assert all(b > a for a,b in zip(stamps, stamps[1:]))


@pytest.mark.requires("cameras")
def test_synced_depth_deproject_and_cloud(camera, robot, request):
    """RGB and depth can produce 3D points

    Categories: cameras
    What it checks: For each depth camera, checks matching image/depth dimensions, float32 depth, a shared capture ID, valid depth returns, finite deprojected points, cloud format/frame, and rejection of an expired frame ID.
    Why it matters: RGB-D pairing and frame conventions determine whether pixels map to meaningful 3D locations.
    If it fails: Check depth/RGB/info topics, alignment, frame, optical convention, and driver returns. The test does not reposition the robot to manufacture depth data.
    Profile fields: cameras.*.depth;cameras.*.rgb;cameras.*.info;cameras.*.frame;cameras.*.optical
    Scenario fields: None.
    Robot effects: Reads RGB-D data and computes points/clouds.
    """
    info = camera.get_info()
    if not info.get("has_depth"):
        from conformance import skip_coverage
        skip_coverage(request, "not_applicable", f"camera {camera.name} has no depth stream")
    frame, depth = camera.get_synced()
    assert depth.depth.shape == frame.image.shape[:2]
    assert depth.depth.dtype == np.float32
    assert frame.id == depth.id
    valid = np.argwhere(np.isfinite(depth.depth) & (depth.depth > 0))
    assert len(valid), "Advertised depth stream has no valid returns; the test does not rotate the robot to manufacture them"
    v,u = valid[len(valid)//2]
    point = camera.deproject((int(u),int(v)), frame=frame)
    assert point.frame == info["frame"]
    assert all(math.isfinite(v) for v in (point.x,point.y,point.z))
    cloud = camera.get_cloud(voxel=.05)
    assert cloud.points.dtype == np.float32
    assert cloud.points.ndim == 2 and cloud.points.shape[1] == 3
    assert cloud.frame == info["frame"]
    if cloud.colors is not None:
        assert cloud.colors.shape == cloud.points.shape
        assert cloud.colors.dtype == np.uint8
    with pytest.raises(FrameExpired):
        camera.deproject((0,0), frame="f999999999")


@pytest.mark.requires("lidar")
def test_lidar_scan_and_stream(robot):
    """Lidar scan geometry is consistent

    Categories: sensors
    What it checks: Checks nonempty float32 rays, angular spacing/count consistency, finite angle bounds, and delivery of two streamed scans. Individual invalid/no-return rays may be NaN or infinity.
    Why it matters: Consumers must be able to interpret the scan's geometry correctly.
    If it fails: Check the scan topic, publisher, angle_min/max/increment, and message shape. Signal-quality requirements need an independent sensor scenario.
    Profile fields: lidar.scan
    Scenario fields: None.
    Robot effects: Reads and streams lidar scans.
    """
    scan = robot.lidar.scan()
    rays = len(scan.ranges)
    assert rays > 0 and scan.angle_increment > 0
    assert rays == pytest.approx((scan.angle_max-scan.angle_min)/scan.angle_increment+1, abs=1.5)
    assert scan.ranges.dtype == np.float32
    assert scan.angle_max >= scan.angle_min
    # Empty/invalid individual returns may be inf or NaN; geometry and
    # signal-quality expectations need an independent sensor scenario.
    assert math.isfinite(scan.angle_min) and math.isfinite(scan.angle_max)
    stream = robot.lidar.stream(hz=5)
    try:
        for i,sweep in enumerate(stream):
            assert sweep.ranges.ndim == 1 and len(sweep.ranges) > 0
            if i == 1:
                break
    finally:
        stream.close()


@pytest.mark.requires("groups")
def test_group_joint_states_with_urdf_limits(group, scenarios, description):
    """Reported joints stay within model limits

    Categories: manipulation,profile
    What it checks: For each group, compares resolved joint names, live positions and joint bounds across SDK state, live model and local kinematics, checks finite positions and ordered bounds, and verifies position lies within reported limits using the configured per-joint allowance (default 0.00001 joint units).
    Why it matters: A model/controller mismatch or miscalibration can invalidate every motion target.
    If it fails: Joint limits come from the live robot description, not ordinary profile YAML. Check the URDF, encoder calibration, controller/joint order, and physical state. Configure a tolerance only from independently measured compliance or sensor uncertainty.
    Profile fields: groups.*.joints
    Scenario fields: tolerances.joint_limits
    Robot effects: Reads joint state and reported model limits.
    """
    from ringfour import kinematics as kin
    joints, positions = group.get_joints(), group.get_joint_positions()
    assert kin.limits(group) == {name: info.limits for name, info in joints.items()}
    assert set(joints) == set(positions) == set(group.joint_names)
    assert joints, "Advertised group has no resolved joints"
    for name, joint in joints.items():
        assert math.isfinite(joint.position), name
        if name in description.joints:
            assert joint.limits == description.joints[name].limits, name
        if joint.limits is not None:
            low,high = joint.limits
            assert low <= high, name
            tolerance = scenarios.get("tolerances", {}).get("joint_limits", {}).get(name, 1e-5)
            assert low-tolerance <= joint.position <= high+tolerance, f"{name} outside limits (tolerance {tolerance})"


@pytest.mark.requires("teleop", "mobility", "watches", "audit")
@pytest.mark.motion
def test_stop_watch_halts_motion_without_client_polling(robot, teleop_recipe, agent_full, tmp_path, request):
    """Persistent guard stops a client that ignores it

    Categories: safety,movement
    What it checks: A separate driver enters the configured guard before motion, then keeps sending commands without polling it. Checks a matching safety audit entry and that the robot stays stopped.
    Why it matters: The stop condition must be enforced on the robot side even when the client is unresponsive or buggy.
    If it fails: Check the watch path/threshold, input signal, audit configuration, and bridge-side halt handling. The condition must become true during the reviewed scenario.
    Profile fields: watches;audit;teleop
    Scenario fields: teleop.watch_stop;teleop.command;teleop.stop_speed;teleop.halt_timeout
    Robot effects: Moves the robot and deliberately triggers a persistent guard.
    """
    import json
    import os
    import subprocess
    import sys
    import time
    from pathlib import Path
    from conformance import SDK_DIR, skip_coverage
    from test_phase3_exit import speed
    recipe = teleop_recipe
    condition = recipe.get("watch_stop")
    if not condition:
        skip_coverage(request, "not_configured", "teleop.watch_stop must identify a condition that becomes true after configured motion")
    helper = Path(__file__).parent / "helpers" / "drive_ignoring_watch.py"
    log = tmp_path / "watch-probe.log"
    with log.open("w") as stream:
        process = subprocess.Popen([sys.executable,str(helper),agent_full.unix_uri,json.dumps(recipe)],
                                   env=dict(os.environ,PYTHONPATH=str(SDK_DIR)),stdout=stream,stderr=subprocess.STDOUT)
    try:
        deadline = time.monotonic()+recipe["duration"]+recipe["halt_timeout"]
        marker = None
        while marker is None:
            marker = next((line for line in log.read_text().splitlines() if line.startswith("GUARD ACTIVE ")),None)
            if marker is not None: break
            assert process.poll() is None, log.read_text()
            assert time.monotonic()<deadline, "watch helper did not reach moving/armed state"
            time.sleep(.05)
        watch_id = marker.rsplit(" ",1)[1]
        deadline = time.monotonic()+recipe["halt_timeout"]
        halts = []
        while time.monotonic()<deadline:
            halts = [e for e in robot.audit.query(kind="safety",limit=50)
                     if e.detail.get("guard_id")==watch_id and e.detail.get("event")=="tripped"]
            if halts and speed(robot)<=recipe["stop_speed"]:
                break
            time.sleep(.05)
        assert halts and speed(robot)<=recipe["stop_speed"], "bridge did not halt on configured watch"
        assert process.poll() is None, "the client must still be alive and ignoring its watch"
        time.sleep(recipe["watchdog"]*2)
        assert speed(robot)<=recipe["stop_speed"], "ignored halt allowed later commands to move the robot"
    finally:
        if process.poll() is None:
            process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=5)
            # The helper bounds its orphan guard lifetime independently.
            time.sleep(.6)
        robot.stop()
