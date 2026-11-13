"""Mapping reads and explicitly configured navigation journeys."""
import math
import time
import uuid
import numpy as np
import pytest
from ringfour import SlamMode, Pose
from ringfour.enums import Phase, OperationState
from ringfour.errors import NavigationFailed
from conformance import validate_pose
from conformance import skip_coverage

pytestmark = pytest.mark.sim


@pytest.mark.requires("slam")
def test_map_grid_is_live(robot, profile_data):
    """Mapping produces a valid occupancy grid

    Categories: mapping
    What it checks: Checks typed SLAM mode and backend against the effective profile, reads a nonempty 2D int8 grid, checks positive finite resolution, and verifies occupancy values are between -1 and 100.
    Why it matters: Mapping consumers need a coherent grid; no particular map resolution or room size is assumed.
    If it fails: Check the SLAM backend and map publisher. Inspect malformed occupancy data or absent mapping output.
    Profile fields: slam.backend
    Scenario fields: None.
    Robot effects: Reads the current map.
    """
    assert isinstance(robot.slam.get_mode(), SlamMode)
    assert robot._robot._welcome.features.get('integration',{}).get('slam_backend')
    if profile_data:
        assert "slam" in profile_data, "Selected profile omits advertised SLAM; choose the effective profile"
        assert robot._robot._welcome.features.get('integration',{}).get('slam_backend') == profile_data["slam"].get("backend", "slam_toolbox")
    grid = robot.get_map()
    assert math.isfinite(grid.resolution) and grid.resolution > 0
    assert grid.data.dtype == np.int8 and grid.data.ndim == 2 and grid.data.size > 0
    assert np.all((grid.data >= -1) & (grid.data <= 100))


@pytest.mark.requires("mobility")
@pytest.mark.motion
def test_navigate_to_configured_goal(robot, navigation_recipe):
    """Robot reaches a reviewed navigation goal

    Categories: movement
    What it checks: Verifies the configured goal uses the live pose frame and requires observable displacement, navigates to it, and checks final distance against scenario tolerance.
    Why it matters: A completed navigation task must correspond to real movement to the requested location.
    If it fails: Check navigation action binding, frame aliases, localization, controller/costmap configuration, and the goal's actual reachability. Use an independently measured arrival tolerance.
    Profile fields: mobility.nav2_navigate_to_pose;frames
    Scenario fields: navigation.goals;navigation.timeout;navigation.tolerance;navigation.min_distance
    Robot effects: Navigates and stops in cleanup.
    """
    r = navigation_recipe
    goal = validate_pose(r["goals"][0], "navigation.goal")
    start = robot.get_pose()
    assert start.frame == goal.frame, "scenario goals must use the live pose frame"
    assert start.distance_to(goal) >= r.get("min_distance", .05), "goal must exercise actual motion"
    try:
        with robot.start_route([goal]) as op:
            result=op.result(r.get('timeout',120))
            assert len(result.arrivals)==1 and result.arrivals[0].stop.verified
        assert robot.get_pose().distance_to(goal) <= r.get("tolerance", .1)
        assert start.distance_to(robot.get_pose()) >= r.get("min_distance", .05)
    finally:
        robot.stop()


@pytest.mark.requires("mobility")
@pytest.mark.motion
def test_navigation_cancellation_halts(robot, navigation_recipe):
    """Cancelling navigation stops the robot

    Categories: movement,safety
    What it checks: Starts navigation to the furthest reviewed goal, waits for actual movement, cancels through stop(), and checks task cancellation and halt before the configured deadline.
    Why it matters: Operators need cancellation to stop the physical robot, not just update a task label.
    If it fails: Check action cancellation, controller stop handling, odometry, the reviewed goal, and braking time.
    Profile fields: mobility.nav2_navigate_to_pose;mobility.odom_topic
    Scenario fields: navigation.goals;navigation.halt_timeout
    Robot effects: Navigates, cancels, and stops.
    """
    r = navigation_recipe
    start = robot.get_pose()
    goals = [validate_pose(item, "navigation.goal") for item in r["goals"]]
    goal = max(goals, key=start.distance_to)
    task = robot.start_navigation(goal)
    try:
        deadline = time.monotonic() + r.get("halt_timeout", 10)
        while not robot.get_is_moving() and time.monotonic() < deadline:
            time.sleep(.05)
        assert robot.get_is_moving(), "goal did not produce observable motion before cancellation"
        assert robot.stop() >= 1
        from ringfour.errors import OperationCancelled
        with pytest.raises(OperationCancelled):task.result(r.get("halt_timeout",10))
        deadline = time.monotonic() + r.get("halt_timeout", 10)
        stopped_since = None
        while True:
            state = robot.get_mobility_state()
            now = time.monotonic()
            if state.is_moving:
                stopped_since = None
            else:
                stopped_since = now if stopped_since is None else stopped_since
                if now-stopped_since >= .2: break
            assert now < deadline, f"Navigation cancellation did not settle: {state.velocity}"
            time.sleep(.05)
    finally:
        robot.stop()


@pytest.mark.requires("mobility")
@pytest.mark.motion
def test_navigation_failure_carries_reason(robot, navigation_recipe, request):
    """Unreachable navigation goal gives a reason

    Categories: movement
    What it checks: Requests an independently known unreachable goal and expects a navigation failure with a nonempty reason.
    Why it matters: A mission needs to distinguish a rejected/unreachable route from successful arrival.
    If it fails: Supply a genuinely unreachable goal for this environment and inspect the navigation backend's failure translation. Do not replace it with an arbitrary far coordinate.
    Profile fields: mobility.nav2_navigate_to_pose
    Scenario fields: navigation.unreachable_goals;navigation.timeout
    Robot effects: Submits a navigation goal and stops in cleanup.
    """
    goals = navigation_recipe.get("unreachable_goals")
    if not goals:
        skip_coverage(request, "not_configured", "no independently known unreachable goal")
    try:
        with pytest.raises(NavigationFailed) as err:
            robot.navigate_to(validate_pose(goals[0], "unreachable_goal"), timeout=navigation_recipe.get("timeout", 60))
        assert err.value.reason
    finally:
        robot.stop()


@pytest.mark.requires("slam", "places", "mobility")
@pytest.mark.motion
def test_map_save_localize_and_go_journey(robot, navigation_recipe, scenarios, request):
    """Saved map can localize and navigate

    Categories: mapping,movement
    What it checks: Starting in mapping mode, creates a unique place/map, saves it, localizes against it, navigates by place name, checks arrival, then removes its artifacts and resumes mapping while retaining the pose graph and existing places.
    Why it matters: Saving a map is useful only if it can be used for localization and navigation.
    If it fails: Check SLAM save/localization services, map storage, navigation action binding, and the reviewed goal. Enable this scenario explicitly because it changes mapping state.
    Profile fields: slam.backend;slam.map_dir;places;mobility.nav2_navigate_to_pose
    Scenario fields: mapping.save_localize;navigation.goals;navigation.tolerance
    Robot effects: Saves a temporary map/place, changes localization mode, moves, then cleans up.
    """
    if not scenarios.get("mapping", {}).get("save_localize"):
        skip_coverage(request, "not_configured", "map save/localization changes require mapping.save_localize")
    # Restoring arbitrary preexisting localization needs its original map
    # identity, so this scenario explicitly starts in mapping mode.
    assert robot.slam.get_mode() is SlamMode.MAPPING, "save/localize scenario requires mapping as its initial state"
    initial = robot.slam.get_state()
    if not initial.get("supports_preserve_graph"):
        skip_coverage(request, "not_configured", "agent cannot restore mapping without clearing the existing graph; update the agent before this test")
    name = "conformance-" + uuid.uuid4().hex[:12]
    place = name + "-goal"
    goal = validate_pose(navigation_recipe["goals"][0], "journey.goal")
    before_places = set(robot.places.list())
    saved = False
    try:
        robot.places.save(place, goal)
        info = robot.slam.save(name)
        saved = True
        assert info.name == name and name in robot.slam.get_saved_maps()
        assert before_places <= set(robot.places.list())
        robot.slam.localize(name=name)
        assert robot.slam.get_mode() is SlamMode.LOCALIZATION
        assert robot.slam.wait_for_localization(timeout=30) >= 0
        robot.navigate_to(place, timeout=navigation_recipe.get("timeout", 120))
        assert robot.get_pose().distance_to(goal) <= navigation_recipe.get("tolerance", .1)
    finally:
        robot.stop()
        # Remove only artifacts created by this test.
        if place in robot.places.list():
            robot.places.delete(place)
        if initial["active_map"]:
            robot.slam.load(initial["active_map"])
        else:
            robot.slam.start(preserve_graph=True)
        # save() copies place namespaces: remove our unique name from
        # the original namespace as well as the temporary one.
        if place in robot.places.list():
            robot.places.delete(place)
        assert place not in robot.places.list(), "Map cleanup left its temporary place"
        assert before_places <= set(robot.places.list()), "Map cleanup lost preexisting places"
        if saved:
            robot.slam.delete(name)


@pytest.mark.requires("places")
def test_existing_places_are_readable(robot):
    """Saved places can be listed and read

    Categories: mapping
    What it checks: Lists existing place names, checks uniqueness, and reads each position/frame with finite coordinates.
    Why it matters: Named destinations must remain usable after storage or map changes.
    If it fails: Check place storage, active map, frame aliases, and persisted coordinates. This check does not create or delete places.
    Profile fields: places;frames
    Scenario fields: None.
    Robot effects: Reads saved places.
    """
    names = robot.places.list()
    assert len(names)==len(set(names))
    for name in names:
        pose = robot.places[name]
        assert pose.frame
        assert all(math.isfinite(v) for v in (pose.x,pose.y,pose.z or 0))


@pytest.mark.requires("places")
def test_places_save_resolve_delete(robot, profile_data, request):
    """Saved places can be created, resolved and deleted

    Categories: mapping
    What it checks: Saves a unique temporary coordinate label in the configured map frame, verifies its pose through the SDK, then deletes it and checks existing place names are preserved. Does not navigate there.
    Why it matters: Listing existing places alone cannot prove the SDK can write or remove them.
    If it fails: Check places storage permissions, active map namespace, profile frames, and SDK/agent place operations.
    Profile fields: places;frames.map
    Scenario fields: None.
    Robot effects: Creates and deletes one uniquely named place; sends no motion commands.
    """
    if profile_data is None:
        skip_coverage(request, "not_configured", "Place round trip needs an independent configured map frame")
    original = set(robot.places.list())
    name = "conformance-places-" + uuid.uuid4().hex
    pose = Pose(x=0, y=0, z=0, frame=profile_data["frames"]["map"])
    try:
        robot.places.save(name, pose)
        assert name in robot.places.list()
        actual = robot.places[name]
        assert actual.frame == pose.frame and actual.distance_to(pose) < 1e-9
    finally:
        if name in robot.places.list():
            robot.places.delete(name)
    assert name not in robot.places.list()
    assert original <= set(robot.places.list())
