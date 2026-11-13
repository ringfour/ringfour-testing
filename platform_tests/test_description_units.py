"""Unit tests for the description parser and its transport (6.3).

Three layers, all offline:

* `ringfour.description` — the parser both sides share;
* the profile rule that there is never a file path (spec 26.2);
* the wire — handshake payload, `model.description`, `model.changed` —
  driven through an in-process agent with a fake ROS side.

The URDF below is small but keeps every shape that has bitten us: a fixed
mount partway down the arm, a branch that is NOT downstream of the arm
(the base camera), a continuous joint with no limits, a wrist camera that
rides on part of the arm but not all of it, and `<ros2_control>` blocks
whose `<joint>` elements share names with real joints and have no
parent/child.
"""

from __future__ import annotations
from types import SimpleNamespace

import asyncio
import threading

import pytest
import yaml
from pydantic import ValidationError

from ringfour import Robot
from ringfour.description import RobotDescription, sha_of
from ringfour_agent.models.profile import GroupConfig, RobotProfile

from ringfour_agent.bootstrap import build_agent
from ringfour_agent.model import run_model_watcher
from ringfour_agent.profile import load_profile

from .conftest import _free_tcp_port

URDF = """<?xml version="1.0"?>
<robot name="testbot">
  <link name="base_link"/>
  <link name="camera_link"/>
  <link name="arm_base"/>
  <link name="arm_upper"/>
  <link name="arm_wrist"/>
  <link name="arm_flange"/>
  <link name="wrist_camera"/>
  <link name="left_wheel"/>
  <joint name="camera_mount" type="fixed">
    <parent link="base_link"/><child link="camera_link"/>
    <origin xyz="0.20 0 0.15" rpy="0 0 0"/>
  </joint>
  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/><child link="left_wheel"/>
    <axis xyz="0 1 0"/>
  </joint>
  <joint name="arm_mount" type="fixed">
    <parent link="base_link"/><child link="arm_base"/>
    <origin xyz="0 0 0.19"/>
  </joint>
  <joint name="j_shoulder" type="revolute">
    <parent link="arm_base"/><child link="arm_upper"/>
    <origin xyz="0 0 0.05"/><axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="8.0" velocity="2.0"/>
  </joint>
  <joint name="j_elbow" type="revolute">
    <parent link="arm_upper"/><child link="arm_wrist"/>
    <origin xyz="0.15 0 0"/><axis xyz="0 1 0"/>
    <limit lower="0.0" upper="3.14"/>
  </joint>
  <joint name="wrist_cam_mount" type="fixed">
    <parent link="arm_wrist"/><child link="wrist_camera"/>
    <origin xyz="0.02 0 0.03"/>
  </joint>
  <joint name="j_roll" type="revolute">
    <parent link="arm_wrist"/><child link="arm_flange"/>
    <origin xyz="0.10 0 0"/><axis xyz="1 0 0"/>
    <limit lower="-3.14" upper="3.14"/>
  </joint>
  <ros2_control name="arm_system" type="system">
    <joint name="j_shoulder"/>
    <joint name="j_elbow"/>
    <joint name="j_roll"/>
  </ros2_control>
  <ros2_control name="tool_system" type="system">
    <joint name="tool_1"/>
  </ros2_control>
</robot>
"""

ARM = ("j_shoulder", "j_elbow", "j_roll")


@pytest.fixture(scope="module")
def shape() -> RobotDescription:
    return RobotDescription.parse(URDF)


# -- the parser -----------------------------------------------------------------

def test_links_and_joints(shape):
    """Links and joints

    Categories: sdk
    What it checks: Verifies that links and joints. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that model parsing, hashing, limits, and rebuild notifications remain consistent.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the robot model parsing code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    assert shape.name == "testbot"
    assert len(shape.links) == 8
    assert shape.root == "base_link"
    # 7 kinematic joints; the ros2_control ones are not among them.
    assert len(shape.joints) == 7
    assert "tool_1" not in shape.joints


def test_ros2_control_joints_are_kept_apart(shape):
    """Ros2 control joints are kept apart

    Categories: sdk
    What it checks: Same names, no parent/child. An iter() walk mixes them in. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that model parsing, hashing, limits, and rebuild notifications remain consistent.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the robot model parsing code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.

    Technical notes:
    Same names, no parent/child. An iter() walk mixes them in.
    """
    assert shape.integration['hardware_joints'] == frozenset(
        {"j_shoulder", "j_elbow", "j_roll", "tool_1"})
    assert "tool_1" in shape.integration['hardware_joints']
    assert "tool_1" not in shape.links


def test_dof_counts_only_moving_joints(shape):
    # 3 arm revolutes + 1 continuous wheel; the 3 fixed mounts do not count.
    """Dof counts only moving joints

    Categories: sdk
    What it checks: Verifies that dof counts only moving joints. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that model parsing, hashing, limits, and rebuild notifications remain consistent.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the robot model parsing code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    assert shape.dof == 4
    assert shape.joints["arm_mount"].moves is False
    assert shape.joints["left_wheel_joint"].moves is True


def test_continuous_joint_has_no_limits(shape):
    """Continuous joint has no limits

    Categories: sdk
    What it checks: A continuous joint is unbounded by definition — reporting (0, 0)
    or a half-bound would make a reachability check lie. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that model parsing, hashing, limits, and rebuild notifications remain consistent.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the robot model parsing code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.

    Technical notes:
    A continuous joint is unbounded by definition — reporting (0, 0)
    or a half-bound would make a reachability check lie.
    """
    assert shape.joints["left_wheel_joint"].limits is None
    assert shape.joints["j_elbow"].limits == (0.0, 3.14)


def test_origins_and_axes_are_parsed(shape):
    """Origins and axes are parsed

    Categories: sdk
    What it checks: Verifies that origins and axes are parsed. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that model parsing, hashing, limits, and rebuild notifications remain consistent.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the robot model parsing code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    mount = shape.joints["arm_mount"]
    assert mount.origin_xyz == (0.0, 0.0, 0.19)
    assert mount.origin_rpy == (0.0, 0.0, 0.0)
    assert mount.axis is None            # fixed joints have no axis
    assert shape.joints["j_elbow"].axis == (0.0, 1.0, 0.0)
    assert shape.joints["j_shoulder"].effort == 8.0
    assert shape.joints["j_shoulder"].velocity == 2.0


def test_chain_includes_fixed_joints_root_first(shape):
    """Chain includes fixed joints root first

    Categories: sdk
    What it checks: Verifies that chain includes fixed joints root first. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that model parsing, hashing, limits, and rebuild notifications remain consistent.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the robot model parsing code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    assert shape.chain_to("arm_flange") == (
        "arm_mount", "j_shoulder", "j_elbow", "j_roll")
    # The fixed mount carries the deck offset; dropping it loses geometry.
    assert shape.moving_chain_to("arm_flange") == ARM


def test_chain_to_unknown_link_raises(shape):
    """Chain to unknown link raises

    Categories: sdk
    What it checks: Verifies that chain to unknown link raises. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that model parsing, hashing, limits, and rebuild notifications remain consistent.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the robot model parsing code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    with pytest.raises(KeyError):
        shape.chain_to("nope")


def test_parent_and_children(shape):
    """Parent and children

    Categories: sdk
    What it checks: Verifies that parent and children. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that model parsing, hashing, limits, and rebuild notifications remain consistent.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the robot model parsing code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    assert shape.parent_joint("base_link") is None
    assert shape.parent_joint("arm_flange").name == "j_roll"
    assert set(shape.children_of("base_link")) == {
        "camera_link", "left_wheel", "arm_base"}


def test_is_downstream_is_the_tip_check(shape):
    """Is downstream is the tip check

    Categories: sdk
    What it checks: Verifies that is downstream is the tip check. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that model parsing, hashing, limits, and rebuild notifications remain consistent.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the robot model parsing code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    assert shape.is_downstream("arm_flange", ARM)
    # A real link on a branch the arm does not move. This is the failure
    # the check exists to prevent: it parses, and its pose never changes.
    assert not shape.is_downstream("camera_link", ARM)
    # The wrist camera rides on two of the three joints, not all of them.
    assert not shape.is_downstream("wrist_camera", ARM)
    assert shape.is_downstream("wrist_camera", ("j_shoulder", "j_elbow"))
    assert not shape.is_downstream("does_not_exist", ARM)


def test_subchain_starts_at_the_groups_first_joint(shape):
    """Subchain starts at the groups first joint

    Categories: sdk
    What it checks: Verifies that subchain starts at the groups first joint. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that model parsing, hashing, limits, and rebuild notifications remain consistent.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the robot model parsing code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    sliced = shape.subchain(ARM, "arm_flange")
    assert [j.name for j in sliced] == ["j_shoulder", "j_elbow", "j_roll"]
    # A group mounted further down keeps the fixed joints between its own
    # joints, because those are the offsets between actuators.
    sliced = shape.subchain(("j_shoulder",), "wrist_camera")
    assert [j.name for j in sliced] == [
        "j_shoulder", "j_elbow", "wrist_cam_mount"]


def test_subchain_rejects_a_joint_off_the_chain(shape):
    """Subchain rejects a joint off the chain

    Categories: sdk
    What it checks: Verifies that subchain rejects a joint off the chain. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that model parsing, hashing, limits, and rebuild notifications remain consistent.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the robot model parsing code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    with pytest.raises(ValueError, match="do not lie on the chain"):
        shape.subchain(("left_wheel_joint",), "arm_flange")


def test_limits_of(shape):
    """Limits of

    Categories: sdk
    What it checks: Verifies that limits of. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that model parsing, hashing, limits, and rebuild notifications remain consistent.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the robot model parsing code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    assert shape.limits_of(ARM) == {
        "j_shoulder": (-1.57, 1.57),
        "j_elbow": (0.0, 3.14),
        "j_roll": (-3.14, 3.14),
    }


def test_malformed_xml_raises_valueerror():
    """Malformed xml raises valueerror

    Categories: sdk
    What it checks: Verifies that malformed xml raises valueerror. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that model parsing, hashing, limits, and rebuild notifications remain consistent.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the robot model parsing code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    with pytest.raises(ValueError, match="not valid XML"):
        RobotDescription.parse("<robot><link name='a'></robot>")


def test_a_cycle_raises_rather_than_hanging():
    """A cycle raises rather than hanging

    Categories: sdk
    What it checks: Runs at agent startup, where a hang is worse than a crash. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that model parsing, hashing, limits, and rebuild notifications remain consistent.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the robot model parsing code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.

    Technical notes:
    Runs at agent startup, where a hang is worse than a crash.
    """
    cyclic = """<?xml version="1.0"?>
    <robot name="loop">
      <link name="a"/><link name="b"/>
      <joint name="ab" type="fixed"><parent link="a"/><child link="b"/></joint>
      <joint name="ba" type="fixed"><parent link="b"/><child link="a"/></joint>
    </robot>"""
    with pytest.raises(ValueError, match="cycle"):
        RobotDescription.parse(cyclic).chain_to("a")


def test_sha_is_of_the_exact_bytes(shape):
    """Sha is of the exact bytes

    Categories: sdk
    What it checks: Verifies that sha is of the exact bytes. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that model parsing, hashing, limits, and rebuild notifications remain consistent.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the robot model parsing code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    assert shape.sha == sha_of(URDF)
    assert sha_of(URDF) != sha_of(URDF + " ")   # whitespace is a change
    assert len(shape.sha) == 64


# -- no file paths, anywhere ----------------------------------------------------

BASE = {"name": "x", "model": "y", "protocol": 1,
        "frames": {"map": "map", "odom": "odom", "base": "base_link"}}


@pytest.mark.parametrize("key", ["urdf", "urdf_path", "urdf_file",
                                 "description_file", "srdf"])
def test_top_level_description_file_is_rejected(key):
    """Top level description file is rejected

    Categories: sdk
    What it checks: Verifies that top level description file is rejected. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that model parsing, hashing, limits, and rebuild notifications remain consistent.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the robot model parsing code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    with pytest.raises(ValidationError):
        RobotProfile.model_validate({**BASE, key: "/tmp/robot.urdf"})


@pytest.mark.parametrize("key", ["urdf", "urdf_path", "srdf"])
def test_description_file_in_a_group_is_rejected(key):
    """Description file in a group is rejected

    Categories: sdk
    What it checks: The loophole worth closing: sections allow extra keys so they can
    carry config for unbuilt phases, and a group is exactly where someone
    porting an arm would drop a URDF path. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that model parsing, hashing, limits, and rebuild notifications remain consistent.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the robot model parsing code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.

    Technical notes:
    The loophole worth closing: sections allow extra keys so they can
    carry config for unbuilt phases, and a group is exactly where someone
    porting an arm would drop a URDF path.
    """
    with pytest.raises(ValidationError, match="never points at a description"):
        GroupConfig.model_validate({
            "kind": "arm",
            "controllers": {"trajectory": "/arm_controller"},
            key: "/tmp/arm.urdf",
        })


def test_description_file_in_any_section_is_rejected():
    """Description file in any section is rejected

    Categories: sdk
    What it checks: Verifies that description file in any section is rejected. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that model parsing, hashing, limits, and rebuild notifications remain consistent.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the robot model parsing code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    with pytest.raises(ValidationError, match="never points at a description"):
        RobotProfile.model_validate({
            **BASE,
            "mobility": {"locomotion": "differential", "urdf": "/x.urdf"},
        })


def test_description_topic_is_still_allowed():
    """Description topic is still allowed

    Categories: sdk
    What it checks: The override is a live topic, not a file — the rule is about
    provenance, not about configurability. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that model parsing, hashing, limits, and rebuild notifications remain consistent.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the robot model parsing code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.

    Technical notes:
    The override is a live topic, not a file — the rule is about
    provenance, not about configurability.
    """
    group = GroupConfig.model_validate({
        "kind": "arm",
        "controllers": {"trajectory": "/arm_controller"},
        "description_topic": "/left/robot_description",
    })
    assert group.description_topic == "/left/robot_description"


# -- the wire -------------------------------------------------------------------

PROFILE = """
name: description-test
model: fake testbot
version: "0"
protocol: 1
frames: {map: map, odom: odom, base: base_link}
groups:
  arm:
    kind: arm
    tip: arm_flange
    controllers: {trajectory: /arm_trajectory_controller,
                  stream: /arm_controller}
"""


class FakeRos:
    """The description-serving surface, and nothing else."""

    def __init__(self, urdf: str | None = URDF):
        self._urdf = urdf
        self.group_urdfs: dict[str, str] = {}

    # -- what groups.py and the handshake need
    def loaded_controllers(self):
        return {"arm_controller": {"state": "active", "type": "x",
                                   "joints": ARM}}

    def declared_controllers(self):
        return frozenset({"arm_controller", "arm_trajectory_controller"})

    def description(self, wait: float = 0.0):
        return self._urdf

    def description_sha(self):
        return None if self._urdf is None else sha_of(self._urdf)

    def group_description(self, group: str):
        return self.group_urdfs.get(group)

    def group_description_topics(self):
        return {}

    def joint_snapshot(self):
        return {name: (0.0, 0.0, 0.0) for name in ARM}

    def joint_limits(self):
        if self._urdf is None:
            return {}
        return RobotDescription.parse(self._urdf).limits_of(ARM)


def _agent(tmp_path, ros, profile: str = PROFILE):
    path = tmp_path / "profile.yaml"
    path.write_text(profile)
    port = _free_tcp_port()
    server, ctx = build_agent(
        load_profile(path), unix_path=None, host="127.0.0.1", port=port,
        audit_dir=tmp_path / "audit", ros=ros,
     allow_unverified_stops=True)
    loop = asyncio.new_event_loop()
    thread = threading.Thread(target=loop.run_forever, daemon=True)
    thread.start()
    asyncio.run_coroutine_threadsafe(server.start(), loop).result(5)
    return f"ws://127.0.0.1:{port}", ctx, server, loop, thread


@pytest.fixture()
def agent_with_description(tmp_path):
    uri, ctx, server, loop, thread = _agent(tmp_path, FakeRos())
    yield uri, ctx
    asyncio.run_coroutine_threadsafe(server.close(), loop).result(5)
    loop.call_soon_threadsafe(loop.stop)
    thread.join(timeout=2)
    ctx.audit.close()


def test_handshake_carries_the_description(agent_with_description):
    """Handshake carries the description

    Categories: sdk
    What it checks: The point of 6.3: no round trip, no ROS on the client. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that model parsing, hashing, limits, and rebuild notifications remain consistent.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the robot model parsing code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.

    Technical notes:
    The point of 6.3: no round trip, no ROS on the client.
    """
    uri, _ctx = agent_with_description
    with Robot(uri) as robot:
        assert robot.description_sha == sha_of(URDF)
        shape = robot.description
        assert shape is not None
        assert shape.name == "testbot"
        assert shape.moving_chain_to("arm_flange") == ARM
        # Cached: the second read is the same object, not a reparse.
        assert robot.description is shape


def test_client_slices_the_declared_group_subchain(agent_with_description):
    """Client slices the declared group subchain

    Categories: sdk
    What it checks: 6.3's exit criterion, on the shape the client was handed. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that model parsing, hashing, limits, and rebuild notifications remain consistent.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the robot model parsing code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.

    Technical notes:
    6.3's exit criterion, on the shape the client was handed.
    """
    uri, _ctx = agent_with_description
    with Robot(uri) as robot:
        shape = robot.description
        joints = tuple(robot.arms["arm"].get_joints())
        assert joints == ARM
        sliced = shape.subchain(joints, "arm_flange")
        assert [j.name for j in sliced] == list(ARM)


def test_model_description_rpc_returns_the_composed_description(
        agent_with_description):
    """Model description rpc returns the composed description

    Categories: sdk
    What it checks: Verifies that model description rpc returns the composed description. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that model parsing, hashing, limits, and rebuild notifications remain consistent.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the robot model parsing code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    uri, _ctx = agent_with_description
    with Robot(uri) as robot:
        again = robot.refresh_description()
        assert again.sha == sha_of(URDF)
        assert robot.description_sha == sha_of(URDF)


def test_model_description_for_a_group_falls_back(agent_with_description):
    """Model description for a group falls back

    Categories: sdk
    What it checks: A group with no topic override answers with the composed
    description, so a caller can ask uniformly. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that model parsing, hashing, limits, and rebuild notifications remain consistent.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the robot model parsing code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.

    Technical notes:
    A group with no topic override answers with the composed
    description, so a caller can ask uniformly.
    """
    uri, _ctx = agent_with_description
    with Robot(uri) as robot:
        assert robot.refresh_description("arm").sha == sha_of(URDF)


def test_model_description_rejects_an_unknown_group(agent_with_description):
    """Model description rejects an unknown group

    Categories: sdk
    What it checks: Verifies that model description rejects an unknown group. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that model parsing, hashing, limits, and rebuild notifications remain consistent.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the robot model parsing code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    from ringfour.errors import RingfourError

    uri, _ctx = agent_with_description
    with Robot(uri) as robot:
        with pytest.raises(RingfourError, match="unknown group"):
            robot.refresh_description("head")


# The same profile without a tip. A group that declares one and has no
# description is a promise the agent cannot check, so it refuses to boot
# (see test_a_declared_tip_needs_a_description). A tip-less group is the
# case where "no description yet" is legitimately just a state.
TIPLESS_PROFILE = PROFILE.replace("    tip: arm_flange\n", "")


def test_no_publisher_yet_is_a_state_not_an_error(tmp_path):
    """No publisher yet is a state not an error

    Categories: sdk
    What it checks: A client connecting before robot_state_publisher gets None, and
    the handshake still succeeds. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that model parsing, hashing, limits, and rebuild notifications remain consistent.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the robot model parsing code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.

    Technical notes:
    A client connecting before robot_state_publisher gets None, and
    the handshake still succeeds.
    """
    uri, ctx, server, loop, thread = _agent(
        tmp_path, FakeRos(urdf=None), profile=TIPLESS_PROFILE)
    try:
        with Robot(uri) as robot:
            assert robot.description is None
            assert robot.description_sha is None
    finally:
        asyncio.run_coroutine_threadsafe(server.close(), loop).result(5)
        loop.call_soon_threadsafe(loop.stop)
        thread.join(timeout=2)
        ctx.audit.close()


def test_a_declared_tip_needs_a_description(tmp_path):
    """A declared tip needs a description

    Categories: sdk
    What it checks: The other half of the rule: a group that says where its end
    effector is must be checkable, and spec 26.2 says the description is
    always there. Refusing to boot beats an ee_pose nobody validated. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that model parsing, hashing, limits, and rebuild notifications remain consistent.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the robot model parsing code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.

    Technical notes:
    The other half of the rule: a group that says where its end
    effector is must be checkable, and spec 26.2 says the description is
    always there. Refusing to boot beats an ee_pose nobody validated.
    """
    from ringfour_agent.groups import GroupValidationError, resolve_groups

    spec = RobotProfile.model_validate(yaml.safe_load(PROFILE))
    with pytest.raises(GroupValidationError, match="no robot description"):
        resolve_groups(spec, FakeRos(urdf=None), wait=0)


# -- model.changed --------------------------------------------------------------

class _Hub:
    def __init__(self):
        self.events = []

    def emit(self, kind, data=None):
        self.events.append((kind, data or {}))


class _Ctx:
    def __init__(self, ros):
        self.ros = ros
        self.events = _Hub()
        from ringfour_agent.safety import AgentState, MotionLock
        self.state=AgentState();self.lock=MotionLock();self.control=None;self.groups={}
        self.teleop=None;self.actions=None
        self.tasks=SimpleNamespace(cancel_running=lambda:None)
        async def cancel_all():pass
        self.tasks.cancel_all=cancel_all
        self.audit=SimpleNamespace(record=lambda *a,**kw:None)


def test_model_watcher_announces_a_rebuilt_description():
    """Model watcher announces a rebuilt description

    Categories: sdk
    What it checks: Verifies that model watcher announces a rebuilt description. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that model parsing, hashing, limits, and rebuild notifications remain consistent.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the robot model parsing code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    ros = FakeRos()
    ctx = _Ctx(ros)

    async def drive():
        task = asyncio.ensure_future(run_model_watcher(ctx, interval=0.01))
        await asyncio.sleep(0.05)
        assert ctx.events.events == [], "the baseline must not be announced"
        ros._urdf = URDF.replace('name="testbot"', 'name="testbot2"')
        await asyncio.sleep(0.1)
        task.cancel()

    asyncio.run(drive())
    assert [kind for kind, _ in ctx.events.events] == ["model.changed"]
    _, data = ctx.events.events[0]
    assert data["sha"] == sha_of(ros._urdf)
    assert data["previous"] == sha_of(URDF)


def test_model_watcher_announces_a_late_publisher():
    """Model watcher announces a late publisher

    Categories: sdk
    What it checks: A client that connected before robot_state_publisher got
    description: null, and must be told when it shows up. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that model parsing, hashing, limits, and rebuild notifications remain consistent.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the robot model parsing code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.

    Technical notes:
    A client that connected before robot_state_publisher got
    description: null, and must be told when it shows up.
    """
    ros = FakeRos(urdf=None)
    ctx = _Ctx(ros)

    async def drive():
        task = asyncio.ensure_future(run_model_watcher(ctx, interval=0.01))
        await asyncio.sleep(0.05)
        ros._urdf = URDF
        await asyncio.sleep(0.1)
        task.cancel()

    asyncio.run(drive())
    assert [kind for kind, _ in ctx.events.events] == ["model.changed"]
    assert ctx.events.events[0][1]["previous"] is None


def test_model_watcher_is_quiet_when_nothing_changes():
    """Model watcher is quiet when nothing changes

    Categories: sdk
    What it checks: Verifies that model watcher is quiet when nothing changes. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that model parsing, hashing, limits, and rebuild notifications remain consistent.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the robot model parsing code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    ctx = _Ctx(FakeRos())

    async def drive():
        task = asyncio.ensure_future(run_model_watcher(ctx, interval=0.01))
        await asyncio.sleep(0.1)
        task.cancel()

    asyncio.run(drive())
    assert ctx.events.events == []


def test_model_watcher_exits_cleanly_without_ros():
    """Model watcher exits cleanly without ros

    Categories: sdk
    What it checks: Verifies that model watcher exits cleanly without ros. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that model parsing, hashing, limits, and rebuild notifications remain consistent.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the robot model parsing code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    asyncio.run(run_model_watcher(_Ctx(None)))


# -- per-group description topics -----------------------------------------------

GROUP_TOPIC_PROFILE = yaml.safe_load(PROFILE)
GROUP_TOPIC_PROFILE["groups"]["arm"]["description_topic"] = \
    "/left/robot_description"


def test_a_group_is_validated_against_its_own_topic(tmp_path):
    """A group is validated against its own topic

    Categories: sdk
    What it checks: On a namespaced launch the default topic does not contain this
    arm's links at all, so falling back to it would reject every tip. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that model parsing, hashing, limits, and rebuild notifications remain consistent.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the robot model parsing code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.

    Technical notes:
    On a namespaced launch the default topic does not contain this
    arm's links at all, so falling back to it would reject every tip.
    """
    from ringfour_agent.groups import resolve_groups

    spec = RobotProfile.model_validate(GROUP_TOPIC_PROFILE)
    ros = FakeRos(urdf="<robot name='chassis'><link name='base_link'/></robot>")
    ros.group_urdfs["arm"] = URDF          # the arm lives on its own topic
    groups = resolve_groups(spec, ros)
    assert groups["arm"].tip == "arm_flange"


def test_a_group_topic_with_no_publisher_is_fatal(tmp_path):
    """A group topic with no publisher is fatal

    Categories: sdk
    What it checks: Verifies that a group topic with no publisher is fatal. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that model parsing, hashing, limits, and rebuild notifications remain consistent.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the robot model parsing code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    from ringfour_agent.groups import GroupValidationError, resolve_groups

    spec = RobotProfile.model_validate(GROUP_TOPIC_PROFILE)
    ros = FakeRos()                        # nothing on /left/robot_description
    with pytest.raises(GroupValidationError, match="nothing has been "
                       "published"):
        resolve_groups(spec, ros, wait=0)
