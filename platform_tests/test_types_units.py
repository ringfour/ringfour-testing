"""Types-reference units (ringfour-types.md): every type constructs and
round-trips, spatial inverses behave, and the latest-value sensor family
(range/imu/mag/wrench/environment/diagnostics) works over the wire with
a fake RosInterface — no ROS, no sim.
"""

from __future__ import annotations

import asyncio
import math
import threading

import numpy as np
import pytest

from ringfour import (
    BatteryState,
    DiagnosticsReport,
    Frontier,
    GeoPose,
    ImuReading,
    MapInfo,
    OccupancyGrid,
    Point,
    Polygon,
    Pose,
    Quaternion,
    RangeReading,
    Robot,
    SubsystemStatus,
    Transform,
    Vec3,
)

from ringfour.models.status import NavStatus, TrajStatus

from ringfour_agent.bootstrap import build_agent
from ringfour_agent.profile import load_profile
from ringfour_agent.state_paths import build_path_table

from .conftest import AGENT_SRC_DIR, _free_tcp_port

SENSOR_PROFILE = """
name: sensors-test
model: fake robot with every sensor section
version: "0"
protocol: 1
frames: {map: map, odom: odom, base: base_link}
groups:
  left:
    kind: arm
    joints: [j1, j2]
    controllers: {trajectory: /left_arm_controller}
range_sensors:
  front_left: {topic: /range/front_left, role: cliff}
  front_right: {topic: /range/front_right, role: cliff}
imu:
  topic: /imu/data
  mag_topic: /imu/mag
force_torque:
  left: {topic: /left_arm_wrench}
environment:
  temperature: /env/temperature
status:
  battery: /battery_state
  diagnostics: /diagnostics
watches: {}
"""


class FakeRos:
    """The reader surface the sensor handlers and watch paths consume."""

    def battery_state(self):
        return {"level": 76.0, "voltage": 24.1, "current": -1.2,
                "temperature": None, "is_charging": False,
                "power_supply_status": "discharging", "stamp": 5.0}

    def range_reading(self, name):
        return {"range": 0.42, "min_range": 0.02, "max_range": 4.0,
                "field_of_view": 0.26, "radiation_type": "ultrasonic",
                "stamp": 5.0, "frame": f"{name}_link"}

    def imu_reading(self):
        return {"orientation": None,
                "angular_velocity": {"x": 0.0, "y": 0.0, "z": 0.3},
                "linear_acceleration": {"x": 0.1, "y": 0.0, "z": 9.81},
                "stamp": 5.0, "frame": "imu_link"}

    def mag_reading(self):
        return {"magnetic_field": {"x": 2e-5, "y": 0.0, "z": -4e-5},
                "stamp": 5.0, "frame": "imu_link"}

    def wrench_reading(self, arm):
        return {"force": {"x": 0.0, "y": 0.0, "z": 11.5},
                "torque": {"x": 0.0, "y": 0.2, "z": 0.0},
                "stamp": 5.0, "frame": f"{arm}_wrist"}

    def environment_reading(self, kind):
        return {"temperature": 21.5, "variance": 0.0,
                "stamp": 5.0, "frame": "env_link"}

    def diagnostics_report(self):
        return {"stamp": 5.0, "subsystems": [
            {"name": "lidar", "level": "ok", "message": "", "values": {}},
            {"name": "battery", "level": "warn",
             "message": "low", "values": {"level": "12"}},
        ]}

    def joint_snapshot(self):
        return {"j1": (0.5, 0.0, 1.0), "j2": (-0.25, 0.0, 2.0)}

    def joint_limits(self):
        return {"j1": (-1.0, 1.0), "j2": None}

    def get_state(self):
        raise RuntimeError("no odometry in this fake")


# -- pure type behavior --------------------------------------------------------

def test_quaternion_and_transform_inverse_roundtrip():
    """Quaternion and transform inverse roundtrip

    Categories: sdk
    What it checks: Verifies that quaternion and transform inverse roundtrip. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that spatial types, frame conversions, and generated signatures preserve their documented meanings.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the sdk data types code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    q = Quaternion.from_euler(0.2, -0.1, 1.3)
    qi = q.inverse()
    x, y, z = qi.rotate(*q.rotate(1.0, 2.0, 3.0))
    assert (x, y, z) == pytest.approx((1.0, 2.0, 3.0))

    t = Transform(translation=Vec3(x=1, y=2, z=3), rotation=q,
                  parent="map", child="base")
    p = Point(x=0.5, y=-0.5, z=0.25, frame="base")
    back = t.inverse().apply(t.apply(p))
    assert (back.x, back.y, back.z) == pytest.approx((0.5, -0.5, 0.25))
    assert t.inverse().parent == "base" and t.inverse().child == "map"


def test_occupancy_grid_queries():
    """Occupancy grid queries

    Categories: sdk
    What it checks: Verifies that occupancy grid queries. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that spatial types, frame conversions, and generated signatures preserve their documented meanings.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the sdk data types code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    data = np.zeros((10, 10), dtype=np.int8)
    data[5, 7] = 100
    data[0, 0] = -1
    grid = OccupancyGrid(width=10, height=10, resolution=0.5,
                         origin=Pose(x=-2.0, y=-2.0, frame="map"),
                         data=data)
    assert grid.cost_at(-2.0 + 7 * 0.5, -2.0 + 5 * 0.5) == 100
    assert not grid.is_free(-2.0, -2.0)        # unknown != free
    assert grid.is_free(0.0, 0.0)
    with pytest.raises(ValueError):
        grid.cost_at(100.0, 0.0)
    assert grid.to_numpy() is data


def test_types_construct_per_reference():
    """Types construct per reference

    Categories: sdk
    What it checks: Verifies that types construct per reference. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that spatial types, frame conversions, and generated signatures preserve their documented meanings.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the sdk data types code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    Polygon(points=((0, 0), (1, 0), (1, 1)), frame="map")
    Polygon(points=((44.4270, 26.1020), (44.4280, 26.1020)))  # geo
    GeoPose(lat=44.43, lon=26.10)
    NavStatus(phase="moving", progress=0.4,
              pose=Pose(x=1, y=2, frame="map"),
              remaining_distance=3.2, waypoint_index=1, stamp=1.0)
    TrajStatus(phase="planning", progress=0.0, joint_positions={"j1": 0.1})
    Frontier(pose=Pose(x=5, y=5, frame="map"), area=12.0, distance=4.2)
    MapInfo(name="kitchen", created_at="2026-06-12T00:00:00",
            resolution=0.05, dimensions=(20.0, 15.0))
    report = DiagnosticsReport(stamp=1.0, subsystems=(
        SubsystemStatus(name="lidar", level="ok"),
        SubsystemStatus(name="arm", level="error", message="overheat"),
    ))
    assert report.check() is False


# -- profile sections -----------------------------------------------------------

def test_sensor_profile_capabilities_and_instances(tmp_path):
    """Sensor profile capabilities and instances

    Categories: sdk
    What it checks: Verifies that sensor profile capabilities and instances. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that spatial types, frame conversions, and generated signatures preserve their documented meanings.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the sdk data types code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    path = tmp_path / "sensors.yaml"
    path.write_text(SENSOR_PROFILE)
    profile = load_profile(path)
    for capability in ("range_sensors", "imu", "force_torque",
                       "environment", "status", "watches"):
        assert capability in profile.info.capabilities
    assert profile.instances["range_sensors"] == ("front_left",
                                                  "front_right")
    assert profile.instances["force_torque"] == ("left",)


# -- the wire, with a fake RosInterface ---------------------------------------------

@pytest.fixture()
def sensor_agent(tmp_path):
    path = tmp_path / "sensors.yaml"
    path.write_text(SENSOR_PROFILE)
    profile = load_profile(path)
    port = _free_tcp_port()
    server, ctx = build_agent(
        profile, unix_path=None, host="127.0.0.1", port=port,
        audit_dir=tmp_path / "audit", ros=FakeRos(),
     allow_unverified_stops=True)
    loop = asyncio.new_event_loop()
    thread = threading.Thread(target=loop.run_forever, daemon=True)
    thread.start()
    asyncio.run_coroutine_threadsafe(server.start(), loop).result(5)
    yield f"ws://127.0.0.1:{port}", ctx
    asyncio.run_coroutine_threadsafe(server.close(), loop).result(5)
    loop.call_soon_threadsafe(loop.stop)
    thread.join(timeout=2)
    ctx.audit.close()


def test_sensor_readings_over_the_wire(sensor_agent):
    """Sensor readings over the wire

    Categories: sdk
    What it checks: Verifies that sensor readings over the wire. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that spatial types, frame conversions, and generated signatures preserve their documented meanings.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the sdk data types code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    uri, _ctx = sensor_agent
    with Robot(uri) as robot:
        battery = robot.get_battery()
        assert isinstance(battery, BatteryState)
        assert battery.voltage == 24.1
        assert battery.power_supply_status == "discharging"

        assert robot.range_sensors == ("front_left", "front_right")
        reading = robot.range("front_left")
        assert isinstance(reading, RangeReading)
        assert reading.range == 0.42
        assert reading.radiation_type == "ultrasonic"

        imu = robot.imu()
        assert isinstance(imu, ImuReading)
        assert imu.orientation is None
        assert imu.linear_acceleration.z == pytest.approx(9.81)
        assert robot.magnetometer().magnetic_field.x == pytest.approx(2e-5)

        wrench = robot.arms["left"].wrench()
        assert wrench.force.z == 11.5 and wrench.frame == "left_wrist"

        assert robot.temperature().temperature == 21.5

        report = robot.diagnostics()
        assert isinstance(report, DiagnosticsReport)
        assert len(report.subsystems) == 2
        assert report.check()  # ok + warn: no error/stale

        positions = robot.arms["left"].get_joint_positions()
        assert positions == {"j1": 0.5, "j2": -0.25}
        assert robot.arms["left"].get_joints()["j1"].limits == (-1.0, 1.0)


def test_unknown_sensor_names_rejected(sensor_agent):
    """Unknown sensor names rejected

    Categories: sdk
    What it checks: Verifies that unknown sensor names rejected. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that spatial types, frame conversions, and generated signatures preserve their documented meanings.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the sdk data types code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    uri, _ctx = sensor_agent
    from ringfour.errors import RingfourError
    with Robot(uri) as robot:
        with pytest.raises(RingfourError):
            robot.range("nonexistent")


def test_watch_paths_cover_new_sensors(sensor_agent):
    """Watch paths cover new sensors

    Categories: sdk
    What it checks: Verifies that watch paths cover new sensors. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that spatial types, frame conversions, and generated signatures preserve their documented meanings.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the sdk data types code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    _uri, ctx = sensor_agent
    table = build_path_table(ctx)
    assert table["range.front_left.range"]() == 0.42
    assert table["imu.angular_velocity.z"]() == pytest.approx(0.3)
    assert table["arms.left.wrench.force.z"]() == 11.5
    assert table["environment.temperature"]() == 21.5
    assert table["battery.voltage"]() == 24.1
    assert table["arms.left.effort.j1"]() == 1.0
    # pose paths exist but resolve None while the fake has no odometry
    assert "pose.x" not in table or table["pose.x"]() is None


def test_ros_interface_init_wires_critical_attributes():
    """Ros interface init wires critical attributes

    Categories: sdk
    What it checks: ros.py imports rclpy so we can't instantiate it here — but we CAN
    assert structurally that __init__ still assigns the attributes the
    rest of the agent depends on. Guards against edits that accidentally
    split __init__ (it happened: methods pasted mid-constructor orphaned
    the TF buffer and cmd_vel publisher into dead code). Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that spatial types, frame conversions, and generated signatures preserve their documented meanings.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the sdk data types code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.

    Technical notes:
    ros.py imports rclpy so we can't instantiate it here — but we CAN
    assert structurally that __init__ still assigns the attributes the
    rest of the agent depends on. Guards against edits that accidentally
    split __init__ (it happened: methods pasted mid-constructor orphaned
    the TF buffer and cmd_vel publisher into dead code).
    """
    import ast
    from pathlib import Path

    # The agent is robot-agnostic and lives in its own workspace outside
    # robots/ (see conftest.AGENT_SRC_DIR).
    # From conftest rather than rebuilt here, so a directory rename
    # breaks one line instead of every file that reaches for the agent.
    source = (AGENT_SRC_DIR / "ringfour_agent" / "ros.py").read_text()
    tree = ast.parse(source)
    init = next(
        node
        for cls in ast.walk(tree)
        if isinstance(cls, ast.ClassDef) and cls.name == "RosInterface"
        for node in cls.body
        if isinstance(node, ast.FunctionDef) and node.name == "__init__"
    )
    assigned = set()
    for node in ast.walk(init):
        targets = []
        if isinstance(node, ast.Assign):
            targets = node.targets
        elif isinstance(node, ast.AnnAssign):
            targets = [node.target]
        assigned.update(t.attr for t in targets
                        if isinstance(t, ast.Attribute))
    for required in ("_tf_buffer", "_tf_listener", "_cmd_vel_pub",
                     "_latest", "_cameras", "_lock"):
        assert required in assigned, f"__init__ no longer sets {required}"


def test_laserscan_and_pointcloud_carry_new_fields():
    """Laserscan and pointcloud carry new fields

    Categories: sdk
    What it checks: Verifies that laserscan and pointcloud carry new fields. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that spatial types, frame conversions, and generated signatures preserve their documented meanings.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the sdk data types code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    from ringfour import LaserScan, PointCloud
    scan = LaserScan(ranges=np.ones(4, np.float32), angle_min=-1.0,
                     angle_max=1.0, angle_increment=0.5, stamp=1.0,
                     frame="lidar", range_min=0.1, range_max=12.0,
                     intensities=np.zeros(4, np.float32))
    assert scan.range_max == 12.0 and scan.intensities is not None
    cloud = PointCloud(points=np.zeros((3, 3), np.float32), colors=None,
                       stamp=1.0, frame="cam",
                       intensities=np.ones(3, np.float32))
    assert cloud.intensities.shape == (3,)
    assert math.isfinite(cloud.stamp)
