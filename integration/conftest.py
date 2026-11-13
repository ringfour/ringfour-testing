"""Live conformance harness. Connect to any running agent; own no robot stack."""
from __future__ import annotations

import os
import sys
import time
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
SDK_DIR = REPO_ROOT / "ringfour-sdk"
for extra in (str(SDK_DIR), str(Path(__file__).parent)):
    if extra not in sys.path:
        sys.path.insert(0, extra)

from conformance import (AgentEndpoints, discover, expected_capabilities,
                         external_endpoints, instance_names, read_yaml,
                         validate_scenarios, validate_vector, skip_coverage)

# Kept as an alias for endpoint-routing callers.
_external_endpoints = external_endpoints


def pytest_addoption(parser):
    parser.addoption("--robot-suite", choices=("compatibility", "qualification", "diagnostics", "regression", "all"), default="compatibility",
                     help="Live robot suite; explicitly named individual checks are always retained")


def pytest_collection_modifyitems(config, items):
    from dashboard.suites import suite_for
    chosen = config.getoption("robot_suite")
    keep, deselected = [], []
    for item in items:
        is_live = Path(item.path).parent == Path(__file__).parent
        function = getattr(item, "originalname", None) or item.name.split("[", 1)[0]
        explicit = any("::" + function in argument for argument in config.args)
        if is_live:
            item.add_marker(pytest.mark.integration)
        if not is_live or chosen == "all" or explicit or suite_for(function, "robot") == chosen:
            keep.append(item)
        else:
            deselected.append(item)
    if deselected:
        config.hook.pytest_deselected(items=deselected)
        items[:] = keep


def pytest_configure(config):
    config.addinivalue_line("markers", "integration: explicitly selected live robot or simulator integration")
    config.addinivalue_line("markers", "hardware: explicitly selected deployment with operator opt-in")
    config.addinivalue_line("markers", "sim: live agent conformance (the suite launches no simulator)")
    config.addinivalue_line("markers", "requires(*capabilities): capabilities required by this contract check")
    config.addinivalue_line("markers", "movement_profile(name): selectable movement scenario")
    config.addinivalue_line("markers", "motion: requires explicit motion opt-in and a matching scenario")
    path=os.environ.get('RINGFOUR_CONFORMANCE')
    if path:
        data=read_yaml(path)
        root=data.get('artifacts',{}).get('snapshot_dir')
        if root:
            os.environ['RINGFOUR_SNAPSHOTS']=os.pathsep.join([str(Path(root).expanduser().resolve()),os.environ.get('RINGFOUR_SNAPSHOTS','')]).rstrip(os.pathsep)
        if data.get('runtime',{}).get('test_timeout'):
            os.environ['RINGFOUR_TEST_TIMEOUT']=str(data['runtime']['test_timeout'])
    if config.option.xmlpath is None:
        directory = REPO_ROOT / "ringfour-testing" / "test-results"
        directory.mkdir(exist_ok=True)
        config.option.xmlpath = str(directory / f"run-{time.strftime('%Y%m%d-%H%M%S')}.xml")


def target(config):
    if not hasattr(config, "_conformance_welcome"):
        endpoints = external_endpoints()
        config._conformance_welcome = discover(endpoints.unix_uri) if endpoints else None
    return config._conformance_welcome


def pytest_generate_tests(metafunc):
    types = {"group_name": "groups", "camera_name": "cameras", "range_name": "range_sensors",
             "wrench_name": "force_torque", "sensor_name": "sensors", "action_name": "actions",
             "power_name": "power_lanes", "capability_name": "capabilities"}
    for fixture, kind in types.items():
        if fixture not in metafunc.fixturenames:
            continue
        welcome = target(metafunc.config)
        names = instance_names(welcome, kind) if welcome else []
        metafunc.parametrize(fixture, names or [pytest.param(None, marks=pytest.mark.skip(
            reason=f"NOT APPLICABLE: no advertised {kind} instances"))])



@pytest.fixture(scope="session")
def agent_full():
    endpoints = external_endpoints()
    if endpoints is None:
        pytest.fail("Set RINGFOUR_AGENT_TCP=ws://host:port or RINGFOUR_AGENT_UNIX=/path; start your robot stack separately.")
    return endpoints


@pytest.fixture(scope="session")
def welcome(request, agent_full):
    return target(request.config) or discover(agent_full.unix_uri)


@pytest.fixture(scope="session")
def profile_data():
    path = os.environ.get("RINGFOUR_PROFILE")
    return read_yaml(path) if path else None


@pytest.fixture(scope="session")
def scenarios(welcome, profile_data):
    path = os.environ.get("RINGFOUR_CONFORMANCE")
    try:
        from movement import default_scenarios
        return validate_scenarios(read_yaml(path) if path else default_scenarios(welcome, profile_data), welcome)
    except ValueError as exc:
        pytest.fail(str(exc))


@pytest.fixture
def robot(request, agent_full, welcome):
    from ringfour import Robot
    for mark in request.node.iter_markers("requires"):
        missing = set(mark.args) - set(welcome.capabilities)
        if missing:
            skip_coverage(request, "not_applicable", f"agent does not advertise {sorted(missing)}")
    with Robot(agent_full.unix_uri) as value:
        yield value


@pytest.fixture
def group(robot, group_name):
    return robot.groups[group_name]


@pytest.fixture
def camera(robot, camera_name):
    return robot.cameras[camera_name]


@pytest.fixture
def description(request, robot, welcome):
    if "group_name" in request.fixturenames:
        name = request.getfixturevalue("group_name")
        if name in welcome.group_description_shas:
            shape = robot.refresh_description(group=name)
            assert shape.sha == welcome.group_description_shas[name], "group description SHA differs from handshake"
            return shape
    shape = robot.description
    if shape is None:
        # A declared articulated group must be backed by a description.
        if robot.has("groups") and any(name not in welcome.group_description_shas for name in robot.group_infos):
            pytest.fail("Advertised groups have no live robot description")
        skip_coverage(request, "not_applicable", "agent publishes no robot description")
    return shape


@pytest.fixture
def tip_group(request, group, description):
    if not group.tip:
        skip_coverage(request, "not_applicable", f"group {group.name} declares no tip")
    return group


@pytest.fixture
def motion_enabled(request, welcome, robot):
    if os.environ.get("RINGFOUR_TEST_MOTION") != "1":
        skip_coverage(request, "disabled", "motion tests are disabled; set RINGFOUR_TEST_MOTION=1 to enable configured scenarios")
    access = welcome.features.get("access")
    if access is not None and not {"operate", "stop"}.issubset(access.get("permissions", [])):
        skip_coverage(request, "not_configured", "selected connection lacks operator/stop permissions; use an authorized deployment connection")
    if robot.get_estop_engaged():
        skip_coverage(request, "not_configured", "selected agent is already inhibited; inspect and explicitly recover/reset before motion qualification")


@pytest.fixture
def group_recipe(request, group, scenarios):
    recipe = scenarios.get("groups", {}).get(group.name)
    return dict(recipe or {})


@pytest.fixture
def group_motion(request, motion_enabled, group, group_recipe, description):
    if "trajectory" not in robot_controllers(group):
        skip_coverage(request, "not_applicable", f"group {group.name} has no trajectory controller")
    policy = group_recipe.get("probe", {})
    mode = policy.get("mode")
    if mode in ("read", "hold"):
        skip_coverage(request, "disabled", f"{group.name}: {mode} policy excludes trajectory movement")
    if mode == "small":
        if policy.get("reviewed") is not True:
            skip_coverage(request, "not_configured", f"{group.name}: confirm group eligibility for generated movement")
        from group_recipes import generate_probe
        try:
            generated = generate_probe(group.joint_names, group.get_joint_positions(),
                                       description.joints, policy, group_recipe.get("tolerance", .001),
                                       request.getfixturevalue("scenarios").get("tolerances",{}).get("joint_limits",{}))
        except ValueError as exc:
            skip_coverage(request, "not_configured", str(exc))
        group_recipe = dict(group_recipe, waypoints=generated["waypoints"],
                            tolerance=group_recipe.get("tolerance", .001))
        generated["description_sha"] = request.getfixturevalue("welcome").description_sha
        request.node.user_properties.append(("generated_group_recipe", __import__('json').dumps(generated)))
    if not group_recipe.get("waypoints"):
        skip_coverage(request, "not_configured", f"{group.name}: select Small movements or supply named poses in Test settings")
    values = [validate_vector(q, group.joint_names, group.name) for q in group_recipe["waypoints"]]
    for q in values:
        for joint, value in zip(group.joint_names, q):
            limits = description.joints[joint].limits
            if limits and not limits[0] <= value <= limits[1]:
                pytest.fail(f"Scenario target for {joint} is outside its live limits {limits}")
    home = group.get_joint_positions()
    if not all(__import__('math').isfinite(v) for v in home.values()):
        pytest.fail("Cannot restore a group with invalid initial joint state")
    for joint, value in home.items():
        limits = description.joints[joint].limits
        if limits and not limits[0] <= value <= limits[1]:
            allowance = request.getfixturevalue("scenarios").get("tolerances", {}).get("joint_limits", {}).get(joint, 1e-5)
            if value < limits[0]-allowance or value > limits[1]+allowance:
                pytest.fail(f"Cannot restore {joint}: its initial position is outside live limits {limits}")
            # Never send an out-of-range restore command, even when measured
            # simulation compliance is within the declared acceptance band.
            home[joint] = max(limits[0], min(limits[1], value))
    # Select a reviewed first target that differs from the current posture.
    tolerance = group_recipe.get("tolerance", .01)
    for index, target in enumerate(values):
        if any(abs(home[j]-v) > tolerance for j,v in zip(group.joint_names,target)):
            values = values[index:] + values[:index]
            break
    else:
        pytest.fail("No configured waypoint exercises motion from the current posture")
    yield group, values, group_recipe
    # Restore only the group exercised by this scenario, within a session.
    from ringfour import Controller
    with group.control(Controller.TRAJECTORY) as session:
        from ringfour import JointTrajectory
        trajectory=JointTrajectory.from_waypoints([group.configuration(home)],step_duration=group_recipe.get('dt',2.))
        session.run(trajectory,timeout=group_recipe.get('restore_timeout',30.))
        import time
        deadline = time.monotonic()+group_recipe.get("restore_timeout",max(10.,group_recipe.get("dt",2.)+2.))
        while True:
            measured = group.get_joint_positions()
            if all(abs(measured[j]-home[j]) <= group_recipe.get("tolerance",.01) for j in group.joint_names):
                break
            assert time.monotonic()<deadline, f"Group {group.name} did not restore its initial joints: target={home}, measured={measured}"
            time.sleep(.05)


def robot_controllers(group):
    return group._owner.integration.group_controllers(group.name)


@pytest.fixture
def teleop_recipe(request, robot, scenarios, motion_enabled):
    recipe = scenarios.get("teleop")
    if recipe is None:
        braking = scenarios.get("movement_profiles", {}).get("braking")
        if braking:
            recipe = {key: braking[key] for key in ("duration", "watchdog", "min_distance", "stop_speed", "halt_timeout")}
            recipe["command"] = {"linear": braking["linear"], "angular": 0., "lateral": 0.}
    if recipe is None:
        skip_coverage(request, "not_configured", "configure a movement command and expected response in Test settings")
    return recipe


@pytest.fixture
def navigation_recipe(request, robot, scenarios, profile_data, record_property):
    if profile_data is not None and not (profile_data.get("mobility") or {}).get("nav2_navigate_to_pose"):
        skip_coverage(request, "not_applicable", "profile has no navigation action binding; mobility alone does not advertise navigation")
    request.getfixturevalue("motion_enabled")
    recipe = scenarios.get("navigation")
    if not recipe or not recipe.get("goals"):
        if recipe and recipe.get("auto"):
            if not robot.has("slam"):
                skip_coverage(request, "not_configured", "Automatic navigation goals need an advertised occupancy map or reviewed manual goals")
            from navigation import live_map_goals
            try:
                recipe = {**recipe, "goals":live_map_goals(robot,recipe)}
            except ValueError as exc:
                skip_coverage(request, "not_configured", str(exc))
            record_property("navigation.goal_source", "connected known-free live map cells")
            record_property("navigation.clearance_m", recipe.get("clearance",1))
        else:
            skip_coverage(request, "not_configured", "Configure map-based navigation or supply reviewed goals")
    from conformance import validate_pose
    # Reusing one endpoint across a suite can turn a cancellation test into
    # an already-arrived no-op. Choose the furthest reviewed endpoint anew.
    recipe = dict(recipe)
    current = robot.get_pose()
    recipe["goals"] = sorted(recipe["goals"], key=lambda g:current.distance_to(validate_pose(g,"navigation.goal")), reverse=True)
    record_property("navigation.goals", __import__("json").dumps(recipe["goals"]))
    return recipe


@pytest.fixture
def debug_enabled(request, scenarios, welcome):
    if scenarios.get("debug", True) is not True:
        skip_coverage(request, "disabled", "debug checks were explicitly disabled in test settings")
    access = welcome.features.get("access")
    if access is not None and not {"configure", "stop", "reset-inhibit"}.issubset(access.get("permissions", [])):
        skip_coverage(request, "not_configured", "selected connection lacks permissions for simulated inhibit/fault checks")


@pytest.fixture
def session_group(request, group):
    if "trajectory" not in robot_controllers(group):
        skip_coverage(request, "not_applicable", f"{group.name} has no trajectory controller")
    request.getfixturevalue("motion_enabled")
    group_recipe = request.getfixturevalue("group_recipe")
    policy = group_recipe.get("probe", {})
    if policy.get("mode") == "read":
        skip_coverage(request, "disabled", f"{group.name}: read-only group policy")
    if policy.get("mode") in ("hold", "small"):
        if policy.get("reviewed") is not True:
            skip_coverage(request, "not_configured", f"{group.name}: confirm controller-session eligibility")
    elif not group_recipe.get("allow_sessions") and not group_recipe.get("waypoints"):
        skip_coverage(request, "not_configured", f"{group.name}: select Hold position or Small movements in Test settings")
    return group


@pytest.fixture
def tool_case(request, group):
    tool = group.tool
    if tool is None or tool.joint is None or tool.open_at is None or tool.closed_at is None:
        skip_coverage(request, "not_applicable", f"{group.name} has no position-controlled tool")
    request.getfixturevalue("motion_enabled")
    group_recipe = request.getfixturevalue("group_recipe")
    description = request.getfixturevalue("description")
    if group_recipe.get("probe", {}).get("mode") in ("read", "hold"):
        skip_coverage(request, "disabled", f"{group.name}: selected group policy excludes tool movement")
    recipe = group_recipe.get("tool")
    if recipe is None:
        skip_coverage(request, "not_configured", f"tool motion not enabled for {group.name}")
    limits = description.joints[tool.joint].limits if tool.joint in description.joints else None
    if limits:
        values = [tool.open_at, tool.closed_at]
        if "blocked_target" in recipe:
            values.append(recipe["blocked_target"])
        assert all(limits[0] <= v <= limits[1] for v in values), "Tool targets exceed live joint limits"
    home = tool.get_width()
    assert __import__('math').isfinite(home), "Cannot restore a tool without a finite initial position"
    if limits:
        assert limits[0] <= home <= limits[1], "Cannot restore a tool whose initial position exceeds live limits"
    yield tool, recipe
    tool.set(home, timeout=recipe.get("timeout",30), execution_timeout=recipe.get("timeout",30))
    import time
    deadline = time.monotonic()+recipe.get("timeout",15)
    while abs(tool.get_width()-home)>recipe.get("tolerance",.01):
        assert time.monotonic()<deadline, "Tool did not restore its initial position"
        time.sleep(.05)


@pytest.fixture
def kin_backend(request, tip_group, description):
    import importlib.util
    if importlib.util.find_spec("roboticstoolbox") is None:
        skip_coverage(request, "not_configured", "optional ringfour[kinematics] backend is not installed")
    # Moving mounts are supported by the current SDK. Use one measured
    # snapshot rather than rejecting any chain larger than command ownership.
    return request.getfixturevalue("kinematic_state")


class MeasuredPositions(dict):
    stamp = None


@pytest.fixture
def kinematic_state(request, group, description):
    from ringfour.errors import CapabilityNotSupported
    import math
    external = {j.name for j in description.joints.values() if j.moves and j.name not in group.joint_names}
    if external:
        try:
            snapshot = group.get_kinematic_snapshot()
            state = snapshot.positions
        except CapabilityNotSupported:
            skip_coverage(request, "not_configured",
                          "running agent lacks measured external-joint snapshots required by this SDK; rebuild/reload the agent before testing kinematics")
    else:
        state = group.get_joint_positions()
    required = external | set(group.joint_names)
    assert not required - state.keys(), f"Snapshot lacks measured joints: {sorted(required-state.keys())}"
    assert all(math.isfinite(float(v)) for v in state.values()), "Snapshot has non-finite joint positions"
    result = MeasuredPositions(state)
    result.stamp = snapshot.stamp if external else None
    return result


@pytest.fixture(autouse=True)
def bounded_live_test(request):
    # A silent sensor stream cannot leave the test runner pending forever.
    # Linux/POSIX is the host environment of this repository.
    if "robot" not in request.fixturenames:
        yield
        return
    import signal
    seconds = float(os.environ.get("RINGFOUR_TEST_TIMEOUT", "180"))
    if not __import__('math').isfinite(seconds) or seconds <= 0:
        pytest.fail("RINGFOUR_TEST_TIMEOUT must be finite and positive")
    previous = signal.getsignal(signal.SIGALRM)
    def expired(signum, frame):
        pytest.fail(f"Live test exceeded {seconds:g}s wall-clock budget")
    signal.signal(signal.SIGALRM, expired)
    signal.setitimer(signal.ITIMER_REAL, seconds)
    try:
        yield
    finally:
        signal.setitimer(signal.ITIMER_REAL, 0)
        signal.signal(signal.SIGALRM, previous)


@pytest.fixture
def movement_profile_recipe(request, scenarios, motion_enabled, record_property):
    name = request.node.get_closest_marker("movement_profile").args[0]
    recipe = scenarios.get("movement_profiles", {}).get(name)
    if recipe is None:
        skip_coverage(request, "not_configured", f"select and configure {name.replace('_', ' ')} in Test settings")
    from movement import effective_movement_recipe, PRECISION_FIELDS
    factor = scenarios.get("movement_tolerance", 1)
    effective = effective_movement_recipe(name, recipe, factor)
    record_property("movement.tolerance_multiplier", factor)
    for field in PRECISION_FIELDS.get(name, ()):
        record_property("movement.effective." + field, effective[field])
    return effective
