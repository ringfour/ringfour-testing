"""Unit tests for the `groups:` profile section and its resolution (6.1).

Two halves:

* the **schema** (`ringfour.models.profile.GroupConfig`) — what a profile
  may say, and which self-contradictions are rejected at parse time;
* the **resolution** (`ringfour_agent.groups`) — what the robot says, and
  which mismatches between the two are fatal at startup.

The second half runs against fakes shaped like the real answers, which were
read off the live base101 stack on 2026-08-24: an unloaded trajectory
controller and a loaded position twin. The tool joint here is deliberately
declared to ros2_control ONLY - base101 also models it, but the schema
must accept a gripper that is commandable without being modelled.
"""

from __future__ import annotations

import pytest
import yaml
from pydantic import ValidationError

from ringfour.enums import Controller, GroupKind
from ringfour_agent.models.profile import GroupConfig, RobotProfile

from ringfour.description import RobotDescription
from ringfour_agent.groups import GroupValidationError, resolve_groups


# -- fixtures shaped like the live robot ----------------------------------------

# Trimmed to the shape that matters: the arm chain, a fixed joint partway
# along it, a link off to the side that is NOT downstream of the arm, and
# a tool joint declared to ros2_control but not modelled - the harder of
# the two cases the tool check has to accept.
URDF = """<?xml version="1.0"?>
<robot name="base101">
  <link name="base_link"/>
  <link name="top_plate"/>
  <link name="arm_base_link"/>
  <link name="arm_shoulder"/>
  <link name="arm_elbow"/>
  <link name="arm_wrist_flange"/>
  <link name="camera_link"/>
  <joint name="plate" type="fixed">
    <parent link="base_link"/><child link="top_plate"/>
  </joint>
  <joint name="arm_base_mount" type="fixed">
    <parent link="top_plate"/><child link="arm_base_link"/>
  </joint>
  <joint name="arm_joint_base" type="revolute">
    <parent link="arm_base_link"/><child link="arm_shoulder"/>
    <limit lower="-3.14" upper="3.14"/>
  </joint>
  <joint name="arm_joint_elbow" type="revolute">
    <parent link="arm_shoulder"/><child link="arm_elbow"/>
    <limit lower="-2.0" upper="2.0"/>
  </joint>
  <joint name="arm_joint_wrist_roll" type="revolute">
    <parent link="arm_elbow"/><child link="arm_wrist_flange"/>
    <limit lower="-3.14" upper="3.14"/>
  </joint>
  <joint name="camera_mount" type="fixed">
    <parent link="base_link"/><child link="camera_link"/>
  </joint>
  <ros2_control name="arm_mod101_sim_system" type="system">
    <joint name="arm_joint_base"/>
    <joint name="arm_joint_elbow"/>
    <joint name="arm_joint_wrist_roll"/>
  </ros2_control>
  <ros2_control name="arm_mod101_tool_jaws_system" type="system">
    <joint name="arm_6"/>
  </ros2_control>
</robot>
"""

ARM_JOINTS = ("arm_joint_base", "arm_joint_elbow", "arm_joint_wrist_roll")

PROFILE = """
name: groups-test
model: fake base101
version: "0"
protocol: 1
frames: {map: map, odom: odom, base: base_link}
groups:
  arm:
    kind: arm
    tip: arm_wrist_flange
    dof_mask: [1, 1, 1, 1, 0, 1]
    controllers:
      trajectory: /arm_trajectory_controller
      stream: /arm_controller
      tool: /gripper_controller
    tool: {type: jaws, joint: arm_6}
"""


class FakeRos:
    """The controller_manager and description answers, as base101 gives them.

    Note what is deliberately absent: ``arm_trajectory_controller`` is
    DECLARED but not loaded, so it contributes no claimed interfaces. That
    is the normal state of the robot, not an edge case (plan 6.0b).
    """

    def __init__(self, *, loaded=None, declared=None, urdf=URDF):
        self._loaded = loaded if loaded is not None else {
            "arm_controller": {
                "state": "active",
                "type": "position_controllers/JointGroupPositionController",
                "joints": ARM_JOINTS,
            },
            "gripper_controller": {
                "state": "active",
                "type": "position_controllers/JointGroupPositionController",
                "joints": ("arm_6",),
            },
        }
        self._declared = frozenset(declared) if declared is not None else (
            frozenset({"arm_controller", "arm_trajectory_controller",
                       "gripper_controller", "gripper_trajectory_controller"}))
        self._urdf = urdf

    def loaded_controllers(self):
        return self._loaded

    def declared_controllers(self):
        return self._declared

    def description(self):
        return self._urdf


def profile_with(**group_overrides) -> RobotProfile:
    """The base profile, with the arm group's keys overridden."""
    data = yaml.safe_load(PROFILE)
    data["groups"]["arm"].update(group_overrides)
    return RobotProfile.model_validate(data)


# -- schema ---------------------------------------------------------------------

def test_group_needs_a_trajectory_controller():
    """Group needs a trajectory controller

    Categories: sdk
    What it checks: Verifies that group needs a trajectory controller. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that group roles, joint resolution, tools, and schema validation agree.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the group configuration code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    with pytest.raises(ValidationError, match="must include 'trajectory'"):
        GroupConfig.model_validate(
            {"kind": "arm", "controllers": {"stream": "/arm_controller"}})


def test_controller_names_are_stored_without_the_leading_slash():
    """Controller names are stored without the leading slash

    Categories: sdk
    What it checks: Profiles write /arm_controller; ros2_control says arm_controller.

    Both are accepted and normalised here so no caller has to strip. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that group roles, joint resolution, tools, and schema validation agree.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the group configuration code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.

    Technical notes:
    Profiles write /arm_controller; ros2_control says arm_controller.

    Both are accepted and normalised here so no caller has to strip.
    """
    group = GroupConfig.model_validate(
        {"kind": "linear",
         "controllers": {"trajectory": "/tower_controller"}})
    assert group.controllers[Controller.TRAJECTORY] == "tower_controller"

    unslashed = GroupConfig.model_validate(
        {"kind": "linear",
         "controllers": {"trajectory": "tower_controller"}})
    assert unslashed.controllers == group.controllers


def test_controller_roles_are_enum_members():
    """Controller roles are enum members

    Categories: sdk
    What it checks: Verifies that controller roles are enum members. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that group roles, joint resolution, tools, and schema validation agree.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the group configuration code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    group = profile_with().groups["arm"]
    assert set(group.controllers) == {
        Controller.TRAJECTORY, Controller.STREAM, Controller.TOOL}
    assert group.controllers[Controller.TOOL] == "gripper_controller"
    # controller_names is role-ordered and deduplicated.
    assert group.controller_names == (
        "arm_trajectory_controller", "arm_controller", "gripper_controller")


def test_kind_is_an_enum_member_and_a_string():
    """Kind is an enum member and a string

    Categories: sdk
    What it checks: Verifies that kind is an enum member and a string. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that group roles, joint resolution, tools, and schema validation agree.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the group configuration code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    group = profile_with().groups["arm"]
    assert group.kind is GroupKind.ARM
    assert group.kind == "arm"


def test_dof_mask_must_be_six_binary_flags():
    """Dof mask must be six binary flags

    Categories: sdk
    What it checks: Verifies that dof mask must be six binary flags. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that group roles, joint resolution, tools, and schema validation agree.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the group configuration code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    with pytest.raises(ValidationError, match="6 entries"):
        profile_with(dof_mask=[1, 1, 1])
    with pytest.raises(ValidationError, match="must be 0 or 1"):
        profile_with(dof_mask=[1, 1, 1, 1, 0, 2])


def test_pointing_group_needs_a_pointing_frame():
    """Pointing group needs a pointing frame

    Categories: sdk
    What it checks: Verifies that pointing group needs a pointing frame. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that group roles, joint resolution, tools, and schema validation agree.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the group configuration code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    with pytest.raises(ValidationError, match="needs pointing_frame"):
        GroupConfig.model_validate(
            {"kind": "pointing",
             "controllers": {"trajectory": "/head_controller"}})


def test_pointing_frame_is_rejected_on_other_kinds():
    """Pointing frame is rejected on other kinds

    Categories: sdk
    What it checks: Verifies that pointing frame is rejected on other kinds. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that group roles, joint resolution, tools, and schema validation agree.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the group configuration code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    with pytest.raises(ValidationError, match="only meaningful on"):
        profile_with(pointing_frame="arm_wrist_flange")


def test_tool_and_tool_controller_come_as_a_pair():
    """Tool and tool controller come as a pair

    Categories: sdk
    What it checks: Either half alone is a half-declared gripper. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that group roles, joint resolution, tools, and schema validation agree.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the group configuration code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.

    Technical notes:
    Either half alone is a half-declared gripper.
    """
    with pytest.raises(ValidationError, match="without controllers.tool"):
        GroupConfig.model_validate(
            {"kind": "arm",
             "controllers": {"trajectory": "/arm_trajectory_controller"},
             "tool": {"type": "jaws", "joint": "arm_6"}})
    with pytest.raises(ValidationError, match="without a tool: section"):
        GroupConfig.model_validate(
            {"kind": "arm",
             "controllers": {"trajectory": "/arm_trajectory_controller",
                             "tool": "/gripper_controller"}})


def test_workspace_box_axes_and_ordering():
    """Workspace box axes and ordering

    Categories: sdk
    What it checks: Verifies that workspace box axes and ordering. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that group roles, joint resolution, tools, and schema validation agree.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the group configuration code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    good = profile_with(workspace={"box": {"x": [-0.1, 0.5], "z": [0.2, 1.1]}})
    assert good.groups["arm"].workspace.box["x"] == (-0.1, 0.5)
    with pytest.raises(ValidationError, match="is not one of x, y, z"):
        profile_with(workspace={"box": {"w": [0.0, 1.0]}})
    with pytest.raises(ValidationError, match="inverted"):
        profile_with(workspace={"box": {"x": [0.5, -0.1]}})


def test_retired_sections_are_not_silently_accepted():
    """Retired sections are not silently accepted

    Categories: sdk
    What it checks: `manipulation:` must not survive as an extra key. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that group roles, joint resolution, tools, and schema validation agree.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the group configuration code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.

    Technical notes:
    `manipulation:` must not survive as an extra key.
    """
    data = yaml.safe_load(PROFILE)
    data["manipulation"] = {"arms": {"arm": {}}}
    with pytest.raises(ValidationError):
        RobotProfile.model_validate(data)


def test_arms_is_the_arm_kind_subset_of_groups():
    """Arms is the arm kind subset of groups

    Categories: sdk
    What it checks: Verifies that arms is the arm kind subset of groups. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that group roles, joint resolution, tools, and schema validation agree.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the group configuration code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    data = yaml.safe_load(PROFILE)
    data["groups"]["head"] = {
        "kind": "pointing",
        "pointing_frame": "camera_link",
        "controllers": {"trajectory": "/head_controller"},
    }
    profile = RobotProfile.model_validate(data)
    assert profile.instances["groups"] == ("arm", "head")
    assert profile.instances["arms"] == ("arm",)
    assert profile.group_names(GroupKind.POINTING) == ("head",)
    assert "groups" in profile.capabilities


# -- description parsing --------------------------------------------------------

def test_description_ignores_ros2_control_joints():
    """Description ignores ros2 control joints

    Categories: sdk
    What it checks: `<ros2_control>` blocks contain <joint> too, with the same names.

    They have no parent/child, so an iter() walk would either mix them
    into the kinematic tree or (as here) have to guess. findall() keeps
    the two namespaces apart, which is what makes hardware_joints a
    meaningful separate set. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that group roles, joint resolution, tools, and schema validation agree.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the group configuration code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.

    Technical notes:
    `<ros2_control>` blocks contain <joint> too, with the same names.

    They have no parent/child, so an iter() walk would either mix them
    into the kinematic tree or (as here) have to guess. findall() keeps
    the two namespaces apart, which is what makes hardware_joints a
    meaningful separate set.
    """
    description = RobotDescription.parse(URDF)
    assert "arm_6" not in description.joints
    assert "arm_6" in description.integration['hardware_joints']
    assert "arm_6" not in description.links


def test_chain_to_tip_includes_fixed_joints_root_first():
    """Chain to tip includes fixed joints root first

    Categories: sdk
    What it checks: Verifies that chain to tip includes fixed joints root first. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that group roles, joint resolution, tools, and schema validation agree.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the group configuration code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    chain = RobotDescription.parse(URDF).chain_to("arm_wrist_flange")
    assert chain == ("plate", "arm_base_mount", "arm_joint_base",
                     "arm_joint_elbow", "arm_joint_wrist_roll")


def test_chain_to_unknown_link_raises():
    """Chain to unknown link raises

    Categories: sdk
    What it checks: Verifies that chain to unknown link raises. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that group roles, joint resolution, tools, and schema validation agree.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the group configuration code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    with pytest.raises(KeyError):
        RobotDescription.parse(URDF).chain_to("nope")


# -- resolution against the robot -----------------------------------------------

def test_joints_come_from_the_loaded_controller_not_the_profile():
    """Joints come from the loaded controller not the profile

    Categories: sdk
    What it checks: The profile declares no joints; the position controller does.

    This is the normal base101 state: the trajectory controller is the
    stated authority but is unloaded, so its exclusive twin answers. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that group roles, joint resolution, tools, and schema validation agree.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the group configuration code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.

    Technical notes:
    The profile declares no joints; the position controller does.

    This is the normal base101 state: the trajectory controller is the
    stated authority but is unloaded, so its exclusive twin answers.
    """
    groups = resolve_groups(profile_with(), FakeRos())
    arm = groups["arm"]
    assert arm.joints == ARM_JOINTS
    assert arm.joints_from == "arm_controller (stream)"
    # The tool joint is tracked separately - a different controller
    # commands it, and it is not in the kinematic chain.
    assert arm.tool_joint == "arm_6"
    assert arm.all_joints == ARM_JOINTS + ("arm_6",)


def test_trajectory_controller_wins_when_it_is_loaded():
    """Trajectory controller wins when it is loaded

    Categories: sdk
    What it checks: Verifies that trajectory controller wins when it is loaded. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that group roles, joint resolution, tools, and schema validation agree.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the group configuration code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    ros = FakeRos(loaded={
        "arm_trajectory_controller": {
            "state": "active",
            "type": "joint_trajectory_controller/JointTrajectoryController",
            "joints": ARM_JOINTS,
        },
        "arm_controller": {
            "state": "inactive", "type": "x",
            "joints": tuple(reversed(ARM_JOINTS)),
        },
        "gripper_controller": {"state": "active", "type": "x",
                               "joints": ("arm_6",)},
    })
    arm = resolve_groups(profile_with(), ros)["arm"]
    assert arm.joints == ARM_JOINTS
    assert arm.joints_from == "arm_trajectory_controller (trajectory)"


def test_profile_joints_override_the_controller():
    """Profile joints override the controller

    Categories: sdk
    What it checks: Verifies that profile joints override the controller. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that group roles, joint resolution, tools, and schema validation agree.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the group configuration code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    override = ("arm_joint_wrist_roll", "arm_joint_base", "arm_joint_elbow")
    arm = resolve_groups(profile_with(joints=list(override)), FakeRos())["arm"]
    assert arm.joints == override
    assert arm.joints_from == "profile"


def test_declared_but_unloaded_controller_is_accepted():
    """Declared but unloaded controller is accepted

    Categories: sdk
    What it checks: The whole point of 6.0b: `load_controller` needs a declaration, not
    a running controller. Rejecting these would reject every base101. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that group roles, joint resolution, tools, and schema validation agree.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the group configuration code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.

    Technical notes:
    The whole point of 6.0b: `load_controller` needs a declaration, not
    a running controller. Rejecting these would reject every base101.
    """
    ros = FakeRos()
    assert "arm_trajectory_controller" not in ros.loaded_controllers()
    assert "arm_trajectory_controller" in ros.declared_controllers()
    assert resolve_groups(profile_with(), ros)   # no raise


def test_unknown_controller_is_fatal_and_names_itself():
    """Unknown controller is fatal and names itself

    Categories: sdk
    What it checks: wait=0: at boot this is indistinguishable from a spawner that has
    not run yet, so the real agent retries first (see
    test_controllers_that_appear_late_are_waited_for). Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that group roles, joint resolution, tools, and schema validation agree.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the group configuration code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.

    Technical notes:
    wait=0: at boot this is indistinguishable from a spawner that has
    not run yet, so the real agent retries first (see
    test_controllers_that_appear_late_are_waited_for).
    """
    ros = FakeRos(declared={"arm_controller", "gripper_controller"})
    with pytest.raises(GroupValidationError) as exc:
        resolve_groups(profile_with(), ros, wait=0)
    assert "arm_trajectory_controller" in str(exc.value)
    assert "groups.arm.controllers.trajectory" in str(exc.value)


def test_tip_must_be_a_link():
    """Tip must be a link

    Categories: sdk
    What it checks: Verifies that tip must be a link. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that group roles, joint resolution, tools, and schema validation agree.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the group configuration code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    with pytest.raises(GroupValidationError, match="not a link"):
        resolve_groups(profile_with(tip="arm_wrist_flang"), FakeRos())


def test_tip_must_be_downstream_of_the_joints():
    """Tip must be downstream of the joints

    Categories: sdk
    What it checks: camera_link is a real link, on a branch the arm does not move.

    Declaring it as the tip would produce an ee_pose that never changes -
    the failure this check exists to make impossible. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that group roles, joint resolution, tools, and schema validation agree.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the group configuration code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.

    Technical notes:
    camera_link is a real link, on a branch the arm does not move.

    Declaring it as the tip would produce an ee_pose that never changes -
    the failure this check exists to make impossible.
    """
    with pytest.raises(GroupValidationError, match="not downstream"):
        resolve_groups(profile_with(tip="camera_link"), FakeRos())


def test_no_tip_is_allowed_and_is_the_ik_gate():
    """No tip is allowed and is the ik gate

    Categories: sdk
    What it checks: Verifies that no tip is allowed and is the ik gate. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that group roles, joint resolution, tools, and schema validation agree.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the group configuration code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    data = yaml.safe_load(PROFILE)
    del data["groups"]["arm"]["tip"]
    groups = resolve_groups(RobotProfile.model_validate(data), FakeRos())
    assert groups["arm"].tip is None


def test_tool_joint_must_be_known_to_ros2_control():
    """Tool joint must be known to ros2 control

    Categories: sdk
    What it checks: Verifies that tool joint must be known to ros2 control. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that group roles, joint resolution, tools, and schema validation agree.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the group configuration code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    profile = profile_with(tool={"type": "jaws", "joint": "arm_7"})
    with pytest.raises(GroupValidationError, match="not declared to"):
        resolve_groups(profile, FakeRos())


def test_a_modelled_tool_joint_is_also_accepted():
    """A modelled tool joint is also accepted

    Categories: sdk
    What it checks: base101's real shape: arm_6 is a revolute joint in the tree AND
    declared to ros2_control. Either alone is enough; both must pass. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that group roles, joint resolution, tools, and schema validation agree.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the group configuration code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.

    Technical notes:
    base101's real shape: arm_6 is a revolute joint in the tree AND
    declared to ros2_control. Either alone is enough; both must pass.
    """
    modelled = URDF.replace(
        '<link name="camera_link"/>',
        '<link name="camera_link"/>\n  <link name="arm_jaws_moving"/>'
    ).replace(
        '<joint name="camera_mount" type="fixed">',
        '<joint name="arm_6" type="revolute">\n'
        '    <parent link="arm_wrist_flange"/>'
        '<child link="arm_jaws_moving"/>\n'
        '    <limit lower="0.0" upper="2.14"/>\n'
        '  </joint>\n'
        '  <joint name="camera_mount" type="fixed">')
    description = RobotDescription.parse(modelled)
    assert "arm_6" in description.joints
    assert "arm_6" in description.integration['hardware_joints']
    arm = resolve_groups(profile_with(), FakeRos(urdf=modelled))["arm"]
    assert arm.tool_joint == "arm_6"
    # It is modelled, but it is still NOT one of the group's joints: a
    # different controller commands it.
    assert "arm_6" not in arm.joints


def test_undeducible_joints_are_fatal():
    """Undeducible joints are fatal

    Categories: sdk
    What it checks: No loaded controller and no override: better to refuse than to
    serve a group whose joint list is empty. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that group roles, joint resolution, tools, and schema validation agree.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the group configuration code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.

    Technical notes:
    No loaded controller and no override: better to refuse than to
    serve a group whose joint list is empty.
    """
    ros = FakeRos(loaded={"gripper_controller": {
        "state": "active", "type": "x", "joints": ("arm_6",)}})
    with pytest.raises(GroupValidationError, match="cannot derive joint"):
        resolve_groups(profile_with(), ros, wait=0)


class LateRos(FakeRos):
    """A controller_manager that is up but whose spawners have not run.

    This is the state a cold `docker compose up` is in for the first few
    seconds, and it is what caught the original bug: `list_controllers`
    answers with an EMPTY list, which by content is indistinguishable
    from a profile naming controllers that will never exist.
    """

    def __init__(self, ready_after: int, **kwargs):
        super().__init__(**kwargs)
        self._ready_after = ready_after
        self.calls = 0

    def loaded_controllers(self):
        self.calls += 1
        if self.calls <= self._ready_after:
            return {}
        return super().loaded_controllers()

    def declared_controllers(self):
        if self.calls <= self._ready_after:
            return frozenset()
        return super().declared_controllers()


def test_controllers_that_appear_late_are_waited_for():
    """Controllers that appear late are waited for

    Categories: sdk
    What it checks: The agent and its stack start together; the agent asks first. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that group roles, joint resolution, tools, and schema validation agree.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the group configuration code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.

    Technical notes:
    The agent and its stack start together; the agent asks first.
    """
    ros = LateRos(ready_after=2)
    groups = resolve_groups(profile_with(), ros, wait=10, poll=0.01)
    assert groups["arm"].joints == ARM_JOINTS
    assert ros.calls > 2, "should have had to ask more than once"


def test_an_empty_controller_manager_is_never_silently_accepted():
    """An empty controller manager is never silently accepted

    Categories: sdk
    What it checks: The bug this replaced: an empty list was read as "cannot answer",
    so the group resolved with zero joints and the agent booted fine -
    then raised on the first joint read. Deferred failure is the one thing
    this module exists to prevent. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that group roles, joint resolution, tools, and schema validation agree.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the group configuration code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.

    Technical notes:
    The bug this replaced: an empty list was read as "cannot answer",
    so the group resolved with zero joints and the agent booted fine -
    then raised on the first joint read. Deferred failure is the one thing
    this module exists to prevent.
    """
    ros = LateRos(ready_after=10_000)     # never comes up
    with pytest.raises(GroupValidationError) as exc:
        resolve_groups(profile_with(), ros, wait=0)
    assert "lists no controllers" in str(exc.value)
    assert "not a slow boot" in str(exc.value)


def test_resolution_without_ros_trusts_the_profile():
    """Resolution without ros trusts the profile

    Categories: sdk
    What it checks: Unit tests and ROS-less runs: no live checks, no crash. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that group roles, joint resolution, tools, and schema validation agree.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the group configuration code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.

    Technical notes:
    Unit tests and ROS-less runs: no live checks, no crash.
    """
    groups = resolve_groups(profile_with(joints=list(ARM_JOINTS)), None)
    assert groups["arm"].joints == ARM_JOINTS
    assert groups["arm"].joints_from == "profile"


def test_a_fake_that_answers_nothing_downgrades_rather_than_crashing():
    """A fake that answers nothing downgrades rather than crashing

    Categories: sdk
    What it checks: Verifies that a fake that answers nothing downgrades rather than crashing. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that group roles, joint resolution, tools, and schema validation agree.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the group configuration code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    class Empty:
        pass

    groups = resolve_groups(profile_with(joints=list(ARM_JOINTS)), Empty())
    assert groups["arm"].joints == ARM_JOINTS


def test_no_groups_section_resolves_to_nothing():
    """No groups section resolves to nothing

    Categories: sdk
    What it checks: Verifies that no groups section resolves to nothing. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that group roles, joint resolution, tools, and schema validation agree.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the group configuration code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    data = yaml.safe_load(PROFILE)
    del data["groups"]
    profile = RobotProfile.model_validate(data)
    assert resolve_groups(profile, FakeRos()) == {}
    assert "groups" not in profile.capabilities
