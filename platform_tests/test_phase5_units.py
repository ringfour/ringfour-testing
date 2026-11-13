"""Phase 5 unit tests: NavRoute semantics, the SLAM mode machine,
places storage, and user-land frontier exploration — no ROS, no sim.

Sim-backed equivalents live in test_phase5_exit.py.
"""

from __future__ import annotations

import asyncio
import sys
from pathlib import Path

import struct
import numpy as np
import pytest
import yaml

from ringfour import Frontier, Polygon, Pose
from ringfour.data import OccupancyGrid
from ringfour.errors import NavigationFailed, RingfourError
from ringfour.errors import TimeoutError as RcTimeout

from ringfour_agent.server import RpcError
from ringfour_agent.places import PlaceStore
from ringfour_agent.slam import SCRATCH_MAP, SlamManager


# -- places -----------------------------------------------------------------------------

def test_places_scoped_to_active_map(slam):
    """Places scoped to active map

    Categories: sdk
    What it checks: Verifies that places scoped to active map. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that navigation, map operations, places, cancellation, and unsupported capabilities follow the protocol.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the navigation and mapping code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    places = PlaceStore(slam)
    pose = {"x": 1.0, "y": 2.0, "frame": "map"}
    places.save("dock", pose)
    assert places.list() == ["dock"]
    assert places.get("dock")["x"] == 1.0

    # places saved on the scratch map migrate with the first save()
    slam.save("kitchen")
    assert places.list() == ["dock"]
    saved = yaml.safe_load(
        (slam.map_dir / "kitchen" / "places.yaml").read_text())
    assert "dock" in saved

    # a different map sees a different (empty) set
    slam.active_map = "elsewhere"
    assert places.list() == []
    with pytest.raises(RpcError) as err:
        places.get("dock")
    assert err.value.error_type == "UnknownPlace"

    slam.active_map = "kitchen"
    places.delete("dock")
    assert places.list() == []


# -- SLAM mode machine (fake ros) -----------------------------------------------------

class FakeSlamRos:
    def __init__(self) -> None:
        self.calls: list = []
        self.correction: tuple | None = (0.0, 0.0, 0.0)

    def slam_pause_toggle(self):
        self.calls.append("pause")
        return True

    def slam_reset(self, pause=False):
        self.calls.append("reset")

    def slam_serialize(self, filename):
        self.calls.append(("serialize", filename))
        Path(filename + ".posegraph").write_bytes(b"fake")
        Path(filename + ".data").write_bytes(b"fake")

    def slam_deserialize(self, filename, match_type, x=0.0, y=0.0,
                         theta=0.0, timeout=20.0):
        self.calls.append(("deserialize", filename, match_type))

    def get_state(self):
        return {"pose": {"x": 1.0, "y": 2.0,
                         "q": {"x": 0, "y": 0, "z": 0, "w": 1},
                         "frame": "map"}}

    def map_grid(self, wait=1.0):
        # A whole grid, not just the header: _export_grid renders the PNG
        # itself now (it used to shell out to map_saver_cli, which simply
        # failed under test), so it needs `data` and `origin` too.
        return {"width": 10, "height": 10, "resolution": 0.05,
                "origin": {"x": -0.25, "y": -0.25, "z": 0.0,
                           "q": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                           "frame": "map"},
                "stamp": 0.0,
                "data": np.full((10, 10), -1, dtype=np.int8)}

    def map_odom_correction(self):
        return self.correction


@pytest.fixture()
def slam(tmp_path):
    return SlamManager(FakeSlamRos(), str(tmp_path / "maps"))


def test_slam_mode_transitions(slam):
    """Slam mode transitions

    Categories: sdk
    What it checks: Verifies that slam mode transitions. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that navigation, map operations, places, cancellation, and unsupported capabilities follow the protocol.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the navigation and mapping code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    assert slam.mode == "mapping"
    assert slam.stop() == "idle"
    assert slam._ros.calls == ["pause"]
    assert slam.start() == "mapping"
    assert slam._ros.calls == ["pause", "pause"]


def test_slam_save_load_localize_cycle(slam):
    """Slam save load localize cycle

    Categories: sdk
    What it checks: Verifies that slam save load localize cycle. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that navigation, map operations, places, cancellation, and unsupported capabilities follow the protocol.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the navigation and mapping code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    info = slam.save("kitchen")
    assert info["name"] == "kitchen"
    assert slam.active_map == "kitchen"
    assert slam.saved_maps() == ["kitchen"]
    assert slam.map_info("kitchen")["backend"] == "slam_toolbox"

    slam.localize()  # against the active map, at the fake current pose
    assert slam.mode == "localization"
    deserialize = [c for c in slam._ros.calls
                   if c and c[0] == "deserialize"][-1]
    assert deserialize[2] == 3  # LOCALIZE_AT_POSE

    # start() from localization resets to a fresh scratch map
    assert slam.start() == "mapping"
    assert "reset" in slam._ros.calls
    assert slam.active_map == SCRATCH_MAP

    slam.load("kitchen")
    assert slam.mode == "mapping" and slam.active_map == "kitchen"


def test_slam_save_writes_map_server_grid(slam, tmp_path):
    """Slam save writes map server grid

    Categories: sdk
    What it checks: save() writes the grid.png + grid.yaml pair map_server loads.

    The agent renders these itself; it used to shell out to
    `ros2 run nav2_map_server map_saver_cli`, which cost 274 MB of Nav2 in
    the release image to draw a message the agent already holds. The
    pixel rules were checked against the real CLI's output. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that navigation, map operations, places, cancellation, and unsupported capabilities follow the protocol.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the navigation and mapping code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.

    Technical notes:
    save() writes the grid.png + grid.yaml pair map_server loads.

    The agent renders these itself; it used to shell out to
    `ros2 run nav2_map_server map_saver_cli`, which cost 274 MB of Nav2 in
    the release image to draw a message the agent already holds. The
    pixel rules were checked against the real CLI's output.
    """
    info = slam.save("kitchen")
    target = slam.map_dir / "kitchen"

    png = (target / "grid.png").read_bytes()
    assert png[:8] == b"\x89PNG\r\n\x1a\n"
    # 8-bit greyscale, and the OccupancyGrid's dimensions
    width, height, depth, colour = struct.unpack(">IIBB", png[16:26])
    assert (width, height, depth, colour) == (10, 10, 8, 0)

    meta = yaml.safe_load((target / "grid.yaml").read_text())
    assert meta["image"] == "grid.png" and meta["mode"] == "trinary"
    assert meta["resolution"] == 0.05
    assert meta["origin"] == [-0.25, -0.25, 0.0]
    # 0.196, not 0.25: grey (205) reads back as 0.19608, and a higher
    # threshold would load every unknown cell as free space.
    assert meta["free_thresh"] == 0.196 and meta["occupied_thresh"] == 0.65

    assert info["resolution"] == 0.05
    assert info["dimensions"] == [0.5, 0.5]


def test_slam_save_survives_a_missing_grid(slam, monkeypatch):
    """Slam save survives a missing grid

    Categories: sdk
    What it checks: No /map yet is not a failed save: the pose graph still lands. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that navigation, map operations, places, cancellation, and unsupported capabilities follow the protocol.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the navigation and mapping code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.

    Technical notes:
    No /map yet is not a failed save: the pose graph still lands.
    """
    monkeypatch.setattr(slam._ros, "map_grid",
                        lambda wait=1.0: (_ for _ in ()).throw(
                            RuntimeError("no map")))
    info = slam.save("early")
    assert info["name"] == "early" and info["dimensions"] == [0.0, 0.0]
    assert not (slam.map_dir / "early" / "grid.png").exists()
    assert (slam.map_dir / "early" / "map_info.yaml").exists()


def test_slam_guards(slam):
    """Slam guards

    Categories: sdk
    What it checks: Verifies that slam guards. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that navigation, map operations, places, cancellation, and unsupported capabilities follow the protocol.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the navigation and mapping code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    with pytest.raises(RpcError):
        slam.localize()                 # nothing saved yet
    with pytest.raises(RpcError):
        slam.save("../evil")
    with pytest.raises(RpcError):
        slam.save("_unsaved")
    slam.save("a")
    with pytest.raises(RpcError):
        slam.delete("a")                # active map is protected
    with pytest.raises(RpcError):
        slam.map_info("ghost")


def test_slam_quality_heuristic(slam):
    """Slam quality heuristic

    Categories: sdk
    What it checks: Verifies that slam quality heuristic. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that navigation, map operations, places, cancellation, and unsupported capabilities follow the protocol.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the navigation and mapping code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    ros = slam._ros
    for _ in range(5):
        slam.sample_quality()
    assert slam.quality == 1.0 and not slam.is_lost
    ros.correction = (1.0, 0.0, 0.0)    # a relocalization jump
    slam.sample_quality()
    assert slam.quality < 0.5
    ros.correction = None               # TF gone entirely
    slam.sample_quality()
    assert slam.quality == 0.0 and slam.is_lost
    ros.correction = (1.0, 0.0, 0.0)
    for _ in range(40):                 # stability slowly restores trust
        slam.sample_quality()
    assert slam.quality > 0.8


# -- goal sugar over the wire ------------------------------------------------------------

def test_goal_to_wire_shapes():
    """Goal to wire shapes

    Categories: sdk
    What it checks: Verifies that goal to wire shapes. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that navigation, map operations, places, cancellation, and unsupported capabilities follow the protocol.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the navigation and mapping code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    from ringfour.aio.autonomy import goal_to_wire
    with pytest.raises(TypeError):goal_to_wire("counter")
    wire = goal_to_wire(Pose(x=1.0, y=2.0, frame="map"))
    assert wire["x"] == 1.0 and wire["frame"] == "map"
    with pytest.raises(TypeError):
        goal_to_wire({"x": 1.0})  # raw dicts are not goals anymore
