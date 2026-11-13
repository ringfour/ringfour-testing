"""Group discovery across arms, legs, lifts, heads, and arbitrary joint names."""
import math
import time
import pytest
from ringfour.enums import Controller, GroupKind
from ringfour.errors import AmbiguousCapability

pytestmark = [pytest.mark.sim, pytest.mark.requires("groups")]


def test_capability_report_lists_groups(robot, welcome):
    """All declared joint groups are exposed

    Categories: profile,manipulation
    What it checks: Compares the client's group names with the handshake and verifies retired capability names are not advertised.
    Why it matters: Missing or stale group declarations make operators select the wrong structures.
    If it fails: Check groups declarations, loaded profile version, and capability reporting. Migrate retired manipulation/joint_groups sections.
    Profile fields: groups
    Scenario fields: None.
    Robot effects: Inspects group metadata.
    """
    assert set(robot.groups) == set(welcome.groups)
    assert not robot.has("manipulation") and not robot.has("joint_groups")


def test_arms_sugar_resolves_to_the_same_handles(robot, welcome):
    """Arm shortcuts resolve without ambiguity

    Categories: manipulation
    What it checks: Checks that arm-kind groups appear in the arm subset and share handles with the full group list. A single-arm shortcut must work only for exactly one arm.
    Why it matters: Multi-arm robots must not silently select one arm through an ambiguous shortcut.
    If it fails: Check each group's kind. Use named groups on multi-arm robots; an SDK identity/ambiguity failure is not fixed by renaming physical arms.
    Profile fields: groups.*.kind
    Scenario fields: None.
    Robot effects: Inspects client handles.
    """
    names = {n for n,g in welcome.groups.items() if g.kind is GroupKind.ARM}
    assert set(robot.arms) == names
    for name in names:
        assert robot.groups[name] is robot.arms[name]
    if len(names) == 1:
        assert robot.arm is robot.arms[next(iter(names))]
    else:
        with pytest.raises(AmbiguousCapability):
            _ = robot.arm


def test_group_identity_and_controller_roles(group, welcome):
    """Group metadata matches its controller roles

    Categories: manipulation,profile
    What it checks: For each group, compares kind, tip, canonical joint order, unique joint names, and nonempty controller-role names with the handshake.
    Why it matters: Commands depend on stable identity, ordering, and the intended controller role.
    If it fails: Check group kind/tip, controller role mappings, and controller-reported joint ordering.
    Profile fields: groups.*.kind;groups.*.tip;groups.*.controllers;groups.*.joints
    Scenario fields: None.
    Robot effects: Inspects group metadata.
    """
    info = welcome.groups[group.name]
    expected = info.tool
    tool = group.tool
    if expected is None:
        assert tool is None
    else:
        assert tool is not None
        assert (tool.type, tool.joint, tool.open_at, tool.closed_at) == (expected.type, expected.joint, expected.open, expected.closed)
    assert group.kind is info.kind
    assert group.tip == info.tip
    assert group.joint_names == info.joints
    assert len(info.joints) == len(set(info.joints))
    assert group._owner.group_infos[group.name].control_modes == info.control_modes
    for role, controller in group._owner.integration.group_controllers(group.name).items():
        assert role in {str(value) for value in Controller}
        assert controller


def test_joint_names_match_live_description(group, description, profile_data):
    """Profile joint/controller bindings match the model

    Categories: manipulation,profile,description
    What it checks: Checks resolved joints exist in the description or hardware declarations, explicit YAML joint order matches live order, and YAML controller names match resolved roles.
    Why it matters: A syntactically valid group can still point at the wrong hardware or controller.
    If it fails: Check explicit joint ordering, controller names, and live URDF/ros2_control declarations. Omit joints only when controller-derived ordering is intended.
    Profile fields: groups.*.joints;groups.*.controllers
    Scenario fields: None.
    Robot effects: Reads bindings and model declarations.
    """
    assert group.joint_names
    for name in group.joint_names:
        assert name in description.joints or name in description.integration['hardware_joints'], name
    if profile_data is not None:
        declared = profile_data["groups"][group.name].get("joints")
        if declared:
            assert tuple(declared) == group.joint_names
        controllers = profile_data["groups"][group.name].get("controllers", {})
        for role,name in controllers.items():
            assert group._owner.integration.group_controllers(group.name)[role] == name.lstrip('/'), role


@pytest.mark.requires("watches")
def test_joint_watch_paths_follow_resolved_joints(group):
    """A group joint can be monitored

    Categories: manipulation,safety
    What it checks: Creates a position watch for the group's first resolved joint with a deliberately unreachable upper threshold and waits for finite measured feedback and checks it stays untriggered.
    Why it matters: A misspelled or unresolved joint should not create a watch that can never observe the intended actuator.
    If it fails: Check resolved joint names, group declarations, watch support, and the live joint-state source. This check does not test the stop action.
    Profile fields: groups.*.joints;watches;joint_states
    Scenario fields: None.
    Robot effects: Creates and removes a non-stopping watch.
    """
    joint = group.joint_names[0]
    with group.watch_position(joint, above=1e9) as watch:
        assert not watch.get_triggered()
        deadline = time.monotonic() + 3
        while watch.get_value() is None and time.monotonic() < deadline:
            time.sleep(.05)
        assert watch.get_value() is not None and math.isfinite(watch.get_value()), "Registered watch never received measured joint feedback"



def test_tool_metadata_matches_declared_group(group, welcome):
    """Fitted tool matches the declared metadata

    Categories: manipulation,profile
    What it checks: Compares tool existence, type, joint, and open/closed endpoint values with the handshake for every group.
    Why it matters: A tool command should address the mechanism the robot actually declares.
    If it fails: Check groups.<name>.tool type, joint, and measured open/closed values. Metadata agreement does not prove physical travel; use the motion check for that.
    Profile fields: groups.*.tool.type;groups.*.tool.joint;groups.*.tool.open;groups.*.tool.closed
    Scenario fields: None.
    Robot effects: Inspects tool metadata.
    """
    tool, expected = group.tool, welcome.groups[group.name].tool
    if expected is None:
        assert tool is None
    else:
        assert tool is not None
        assert (tool.type,tool.joint,tool.open_at,tool.closed_at) == (expected.type,expected.joint,expected.open,expected.closed)
