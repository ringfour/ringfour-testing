"""Unit tests for `ringfour.kinematics` (6.5), offline.

Two halves that must not be confused:

* the **core promise** — `import ringfour` and everything in Phases 1-5
  works with no solver installed. Tested by making the backend
  unimportable, not by uninstalling it.
* the **maths** — FK against transforms computed by hand, IK round-trips,
  determinism, and the failure modes. These need the backend and skip
  without it.

The fixture arm is deliberately awkward in the ways real robots are: a
fixed deck mount above the first joint (so base-frame answers are wrong if
you slice it off), a joint whose axis is **-y** (so the axis-alignment
path is exercised, not just +z), and a fixed tool offset below the last
joint (so the tip is not the last actuator).
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from ringfour.description import RobotDescription
from ringfour.enums import GroupKind
from ringfour.errors import CapabilityNotSupported, Unreachable
from ringfour.models import GroupInfo, Pose

kin = pytest.importorskip(
    "ringfour.kinematics",
    reason="needs the optional backend: pip install 'ringfour[kinematics]'")
pytest.importorskip(
    "roboticstoolbox",
    reason="needs the optional backend: pip install 'ringfour[kinematics]'")


# A planar 3R arm with a 0.2 m deck mount, unit links, and a 0.1 m tool.
# Every number below is chosen so FK is checkable on paper.
URDF = """<?xml version="1.0"?>
<robot name="fixture">
  <link name="base_link"/>
  <link name="deck"/>
  <link name="l1"/>
  <link name="l2"/>
  <link name="l3"/>
  <link name="tip"/>
  <link name="stray"/>
  <joint name="deck_mount" type="fixed">
    <parent link="base_link"/><child link="deck"/>
    <origin xyz="0 0 0.2"/>
  </joint>
  <joint name="j1" type="revolute">
    <parent link="deck"/><child link="l1"/>
    <origin xyz="0 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-3.0" upper="3.0"/>
  </joint>
  <joint name="j2" type="revolute">
    <parent link="l1"/><child link="l2"/>
    <origin xyz="1 0 0"/><axis xyz="0 -1 0"/>
    <limit lower="-2.0" upper="2.0"/>
  </joint>
  <joint name="j3" type="revolute">
    <parent link="l2"/><child link="l3"/>
    <origin xyz="1 0 0"/><axis xyz="0 -1 0"/>
    <limit lower="-2.0" upper="2.0"/>
  </joint>
  <joint name="tool_mount" type="fixed">
    <parent link="l3"/><child link="tip"/>
    <origin xyz="0.1 0 0"/>
  </joint>
  <joint name="stray_mount" type="fixed">
    <parent link="base_link"/><child link="stray"/>
    <origin xyz="0 0.5 0"/>
  </joint>
</robot>
"""

JOINTS = ("j1", "j2", "j3")


class FakeRobot:
    """The two things a group handle's owner has to answer."""

    def __init__(self, info: GroupInfo, urdf: str = URDF):
        self._info = info
        self.name = "kinematics-fixture"
        self.description = RobotDescription.parse(urdf)

    def group_info(self, name: str) -> GroupInfo:
        return self._info


class FakeGroup:
    """A group handle: a name, an owner, and live joint positions."""

    def get_joint_positions(self): return self.joint_positions
    def get_kinematic_positions(self): return self.kinematic_positions

    def __init__(self, info: GroupInfo, positions=None, urdf: str = URDF,
                 name: str = "arm"):
        self.name = name
        self._owner = FakeRobot(info, urdf)
        self.joint_positions = dict(
            zip(info.joints, positions or [0.0] * len(info.joints)))


def group(tip: str | None = "tip", mask=(1, 0, 1, 0, 1, 0),
          kind=GroupKind.ARM, joints=JOINTS, positions=None,
          pointing_frame=None, urdf: str = URDF) -> FakeGroup:
    info = GroupInfo(kind=kind, tip=tip, dof_mask=mask, joints=joints,
                     pointing_frame=pointing_frame)
    return FakeGroup(info, positions, urdf)


# -- FK, against transforms worked out by hand -----------------------------------

def test_fk_at_zero_includes_the_deck_mount():
    """Fk at zero includes the deck mount

    Categories: sdk
    What it checks: The whole chain from the ROOT: 1 + 1 + 0.1 along x, 0.2 up.

    Slicing the chain at the group's first joint (which is what
    `subchain` does, correctly, for a different question) drops the deck
    and silently puts the tip 0.2 m low. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that known numerical fixtures validate pose, Jacobian, IK, and collision behavior.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the kinematics calculations code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.

    Technical notes:
    The whole chain from the ROOT: 1 + 1 + 0.1 along x, 0.2 up.

    Slicing the chain at the group's first joint (which is what
    `subchain` does, correctly, for a different question) drops the deck
    and silently puts the tip 0.2 m low.
    """
    pose = kin.fk(group(), q=[0.0, 0.0, 0.0])
    assert pose.x == pytest.approx(2.1)
    assert pose.y == pytest.approx(0.0, abs=1e-12)
    assert pose.z == pytest.approx(0.2)
    assert pose.frame == "base_link"


def test_fk_rotates_about_the_base_joint():
    """Fk rotates about the base joint

    Categories: sdk
    What it checks: Verifies that fk rotates about the base joint. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that known numerical fixtures validate pose, Jacobian, IK, and collision behavior.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the kinematics calculations code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    pose = kin.fk(group(), q=[math.pi / 2, 0.0, 0.0])
    assert pose.x == pytest.approx(0.0, abs=1e-9)
    assert pose.y == pytest.approx(2.1)
    assert pose.z == pytest.approx(0.2)


def test_fk_handles_a_negative_axis():
    """Fk handles a negative axis

    Categories: sdk
    What it checks: j2's axis is -y, so +q2 lifts the arm. Getting the sign wrong here
    is invisible at q=0 and wrong everywhere else. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that known numerical fixtures validate pose, Jacobian, IK, and collision behavior.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the kinematics calculations code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.

    Technical notes:
    j2's axis is -y, so +q2 lifts the arm. Getting the sign wrong here
    is invisible at q=0 and wrong everywhere else.
    """
    pose = kin.fk(group(), q=[0.0, math.pi / 2, 0.0])
    # The 1 m upper link stands up; the rest (1 + 0.1) goes with it.
    assert pose.x == pytest.approx(1.0)
    assert pose.z == pytest.approx(0.2 + 1.1)


def test_fk_defaults_to_live_joint_positions():
    """Fk defaults to live joint positions

    Categories: sdk
    What it checks: Verifies that fk defaults to live joint positions. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that known numerical fixtures validate pose, Jacobian, IK, and collision behavior.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the kinematics calculations code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    handle = group(positions=[0.0, math.pi / 2, 0.0])
    assert kin.fk(handle).z == pytest.approx(kin.fk(
        handle, q=[0.0, math.pi / 2, 0.0]).z)


def test_fk_orientation_is_not_yaw_only():
    """Fk orientation is not yaw only

    Categories: sdk
    What it checks: `Pose.yaw` is a navigation convenience. Building a Pose from it
    discards roll and pitch, which for an arm means FK produces a pose IK
    can never hit. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that known numerical fixtures validate pose, Jacobian, IK, and collision behavior.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the kinematics calculations code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.

    Technical notes:
    `Pose.yaw` is a navigation convenience. Building a Pose from it
    discards roll and pitch, which for an arm means FK produces a pose IK
    can never hit.
    """
    pose = kin.fk(group(), q=[0.0, math.pi / 2, 0.0])
    roll, pitch, yaw = pose.q.to_euler()
    assert pitch == pytest.approx(-math.pi / 2)
    assert (roll, yaw) == pytest.approx((0.0, 0.0), abs=1e-9)


def test_fk_to_a_named_link():
    """Fk to a named link

    Categories: sdk
    What it checks: Solving to l3 drops the 0.1 m tool offset. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that known numerical fixtures validate pose, Jacobian, IK, and collision behavior.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the kinematics calculations code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.

    Technical notes:
    Solving to l3 drops the 0.1 m tool offset.
    """
    assert kin.fk(group(), q=[0, 0, 0], link="l3").x == pytest.approx(2.0)


def test_fk_rejects_a_wrong_length_q():
    """Fk rejects a wrong length q

    Categories: sdk
    What it checks: Verifies that fk rejects a wrong length q. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that known numerical fixtures validate pose, Jacobian, IK, and collision behavior.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the kinematics calculations code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    with pytest.raises(ValueError, match="has 2 values"):
        kin.fk(group(), q=[0.0, 0.0])


# -- IK ---------------------------------------------------------------------------

def test_ik_round_trips_fk():
    """Ik round trips fk

    Categories: sdk
    What it checks: Verifies that ik round trips fk. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that known numerical fixtures validate pose, Jacobian, IK, and collision behavior.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the kinematics calculations code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    handle = group()
    q0 = [0.4, 0.6, -0.3]
    target = kin.fk(handle, q=q0)
    solved = kin.ik(handle, target, seed=q0)
    back = kin.fk(handle, q=solved)
    assert (back.x, back.y, back.z) == pytest.approx(
        (target.x, target.y, target.z), abs=1e-6)


def test_ik_is_deterministic():
    """Ik is deterministic

    Categories: sdk
    What it checks: The contract that separates this from a planner. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that known numerical fixtures validate pose, Jacobian, IK, and collision behavior.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the kinematics calculations code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.

    Technical notes:
    The contract that separates this from a planner.
    """
    handle = group()
    target = kin.fk(handle, q=[0.4, 0.6, -0.3])
    solutions = {tuple(round(v, 12) for v in
                       kin.ik(handle, target, seed=[0.4, 0.6, -0.3]).vector(JOINTS))
                 for _ in range(50)}
    assert len(solutions) == 1


def test_ik_seed_defaults_to_the_live_configuration():
    """Ik seed defaults to the live configuration

    Categories: sdk
    What it checks: So successive solves stay in one branch and the arm does not flip
    configuration mid-mission. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that known numerical fixtures validate pose, Jacobian, IK, and collision behavior.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the kinematics calculations code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.

    Technical notes:
    So successive solves stay in one branch and the arm does not flip
    configuration mid-mission.
    """
    q0 = [0.4, 0.6, -0.3]
    handle = group(positions=q0)
    target = kin.fk(handle, q=q0)
    assert kin.ik(handle, target) == kin.ik(handle, target, seed=q0)


def test_ik_raises_rather_than_returning_none():
    """Ik raises rather than returning none

    Categories: sdk
    What it checks: A membership test over waypoints cannot say WHICH pose failed. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that known numerical fixtures validate pose, Jacobian, IK, and collision behavior.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the kinematics calculations code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.

    Technical notes:
    A membership test over waypoints cannot say WHICH pose failed.
    """
    with pytest.raises(Unreachable) as exc:
        kin.ik(group(), Pose(x=50.0, y=0.0, z=0.0, frame="base_link"))
    message = str(exc.value)
    assert "arm" in message and "tip" in message
    assert "50.000" in message


def test_reachable_is_ik_without_the_raise():
    """Reachable is ik without the raise

    Categories: sdk
    What it checks: Verifies that reachable is ik without the raise. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that known numerical fixtures validate pose, Jacobian, IK, and collision behavior.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the kinematics calculations code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    handle = group()
    assert kin.reachable(handle, kin.fk(handle, q=[0.2, 0.3, 0.1]))
    assert not kin.reachable(
        handle, Pose(x=50.0, y=0.0, z=0.0, frame="base_link"))


def test_ik_needs_a_tip():
    """Ik needs a tip

    Categories: sdk
    What it checks: Verifies that ik needs a tip. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that known numerical fixtures validate pose, Jacobian, IK, and collision behavior.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the kinematics calculations code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    with pytest.raises(CapabilityNotSupported, match="declares no tip"):
        kin.ik(group(tip=None), Pose(x=1.0, y=0.0, frame="base_link"))


def test_ik_mask_must_have_six_entries():
    """Ik mask must have six entries

    Categories: sdk
    What it checks: Verifies that ik mask must have six entries. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that known numerical fixtures validate pose, Jacobian, IK, and collision behavior.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the kinematics calculations code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    handle = group()
    with pytest.raises(ValueError, match="6 entries"):
        kin.ik(handle, kin.fk(handle, q=[0, 0, 0]), mask=[1, 1, 1])


# -- things that work without a tip, and without solving --------------------------

def test_limits_and_links_need_no_solver_state():
    """Limits and links need no solver state

    Categories: sdk
    What it checks: Verifies that limits and links need no solver state. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that known numerical fixtures validate pose, Jacobian, IK, and collision behavior.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the kinematics calculations code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    assert kin.limits(group()) == {
        "j1": (-3.0, 3.0), "j2": (-2.0, 2.0), "j3": (-2.0, 2.0)}
    # links() reports the span the GROUP owns: from its first joint down,
    # so the deck mount above it is not included. That is the opposite of
    # what fk() needs, which is why fk builds from the root instead.
    chain = kin.links(group())
    assert [link.joint for link in chain] == [
        "j1", "j2", "j3", "tool_mount"]
    assert [link.moves for link in chain] == [True, True, True, False]


def test_limits_works_without_a_tip():
    """Limits works without a tip

    Categories: sdk
    What it checks: A lift or a head has limits but nothing to reach with. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that known numerical fixtures validate pose, Jacobian, IK, and collision behavior.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the kinematics calculations code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.

    Technical notes:
    A lift or a head has limits but nothing to reach with.
    """
    assert kin.limits(group(tip=None)) == kin.limits(group())


def test_reach_is_the_summed_extension():
    # 1 + 1 + 0.1 of link, plus the 0.2 deck riser.
    """Reach is the summed extension

    Categories: sdk
    What it checks: Verifies that reach is the summed extension. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that known numerical fixtures validate pose, Jacobian, IK, and collision behavior.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the kinematics calculations code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    assert kin.reach(group()) == pytest.approx(2.3)


def test_jacobian_shape():
    """Jacobian shape

    Categories: sdk
    What it checks: Verifies that jacobian shape. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that known numerical fixtures validate pose, Jacobian, IK, and collision behavior.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the kinematics calculations code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    assert kin.jacobian(group(), q=[0.1, 0.2, 0.3]).shape == (6, 3)


def test_manipulability_masks_to_the_controllable_dof():
    """Manipulability masks to the controllable dof

    Categories: sdk
    What it checks: A 3-joint arm has a 6x3 Jacobian, so the textbook sqrt(det(J Jt))
    is 0.0 everywhere — a number that looks like a permanent singularity
    and means nothing. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that known numerical fixtures validate pose, Jacobian, IK, and collision behavior.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the kinematics calculations code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.

    Technical notes:
    A 3-joint arm has a 6x3 Jacobian, so the textbook sqrt(det(J Jt))
    is 0.0 everywhere — a number that looks like a permanent singularity
    and means nothing.
    """
    handle = group()
    assert kin.manipulability(handle, q=[0.3, 0.7, -0.4]) > 0.0
    J = kin.jacobian(handle, q=[0.3, 0.7, -0.4])
    assert float(np.linalg.det(J @ J.T)) == pytest.approx(0.0, abs=1e-9)


def test_manipulability_is_zero_at_a_singularity():
    """Manipulability is zero at a singularity

    Categories: sdk
    What it checks: Fully extended: the arm cannot move along its own length. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that known numerical fixtures validate pose, Jacobian, IK, and collision behavior.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the kinematics calculations code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.

    Technical notes:
    Fully extended: the arm cannot move along its own length.
    """
    assert kin.manipulability(group(), q=[0.0, 0.0, 0.0]) == pytest.approx(
        0.0, abs=1e-9)


def test_manipulability_rejects_more_dof_than_joints():
    """Manipulability rejects more dof than joints

    Categories: sdk
    What it checks: Verifies that manipulability rejects more dof than joints. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that known numerical fixtures validate pose, Jacobian, IK, and collision behavior.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the kinematics calculations code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    with pytest.raises(ValueError, match="manipulability is undefined"):
        kin.manipulability(group(mask=(1, 1, 1, 1, 1, 1)), q=[0, 0, 0])


# -- pointing groups ---------------------------------------------------------------

POINTING_URDF = """<?xml version="1.0"?>
<robot name="head">
  <link name="base_link"/><link name="pan_link"/><link name="tilt_link"/>
  <joint name="pan" type="revolute">
    <parent link="base_link"/><child link="pan_link"/>
    <axis xyz="0 0 1"/><limit lower="-1.5" upper="1.5"/>
  </joint>
  <joint name="tilt" type="revolute">
    <parent link="pan_link"/><child link="tilt_link"/>
    <axis xyz="0 1 0"/><limit lower="-0.5" upper="0.5"/>
  </joint>
</robot>
"""


def head(**kwargs):
    return group(tip=None, kind=GroupKind.POINTING, joints=("pan", "tilt"),
                 pointing_frame="tilt_link", urdf=POINTING_URDF, **kwargs)


def test_point_at_aims_a_body_frame_along_x():
    """Point at aims a body frame along x

    Categories: sdk
    What it checks: `tilt_link` here is a BODY frame, so its boresight is +X.

    The 45 degrees is the easy half. The point of the test is `axis="x"`:
    a body frame and an optical frame differ by a quarter turn, and
    point_at cannot guess which one it was handed. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that known numerical fixtures validate pose, Jacobian, IK, and collision behavior.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the kinematics calculations code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.

    Technical notes:
    `tilt_link` here is a BODY frame, so its boresight is +X.

    The 45 degrees is the easy half. The point of the test is `axis="x"`:
    a body frame and an optical frame differ by a quarter turn, and
    point_at cannot guess which one it was handed.
    """
    pan, tilt = kin.point_at(head(), (1.0, 1.0, 0.0), axis="x")
    assert pan == pytest.approx(math.pi / 4, abs=1e-3)
    assert tilt == pytest.approx(0.0, abs=1e-3)


def test_point_at_aims_an_optical_frame_along_z():
    """Point at aims an optical frame along z

    Categories: sdk
    What it checks: The same head, same target, default axis: +Z is the boresight, so
    the answer is a quarter turn away from the body-frame one — and the
    tilt joint's +-0.5 rad cannot get there, so it refuses rather than
    returning a plausible-looking wrong angle. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that known numerical fixtures validate pose, Jacobian, IK, and collision behavior.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the kinematics calculations code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.

    Technical notes:
    The same head, same target, default axis: +Z is the boresight, so
    the answer is a quarter turn away from the body-frame one — and the
    tilt joint's +-0.5 rad cannot get there, so it refuses rather than
    returning a plausible-looking wrong angle.
    """
    with pytest.raises(Unreachable, match="cannot be aimed"):
        kin.point_at(head(), (1.0, 1.0, 0.0))


def test_point_at_best_effort_returns_the_closest_aim():
    """Point at best effort returns the closest aim

    Categories: sdk
    What it checks: `strict=False` is for a tracker: a head that cannot centre
    something should turn as far as it can, not refuse and watch it
    leave. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that known numerical fixtures validate pose, Jacobian, IK, and collision behavior.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the kinematics calculations code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.

    Technical notes:
    `strict=False` is for a tracker: a head that cannot centre
    something should turn as far as it can, not refuse and watch it
    leave.
    """
    pan, tilt = kin.point_at(head(), (1.0, 1.0, 0.0), strict=False)
    assert -1.5 <= pan <= 1.5
    assert -0.5 <= tilt <= 0.5


def test_point_at_rejects_an_unknown_axis():
    """Point at rejects an unknown axis

    Categories: sdk
    What it checks: Verifies that point at rejects an unknown axis. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that known numerical fixtures validate pose, Jacobian, IK, and collision behavior.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the kinematics calculations code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    with pytest.raises(ValueError, match="axis must be"):
        kin.point_at(head(), (1.0, 0.0, 0.0), axis="w")


def test_point_at_clamps_to_the_joint_limits():
    """Point at clamps to the joint limits

    Categories: sdk
    What it checks: Aiming behind the robot with a +-1.5 rad pan: clamped, not wrapped
    to a value the joint cannot reach. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that known numerical fixtures validate pose, Jacobian, IK, and collision behavior.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the kinematics calculations code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.

    Technical notes:
    Aiming behind the robot with a +-1.5 rad pan: clamped, not wrapped
    to a value the joint cannot reach.
    """
    pan, _tilt = kin.point_at(head(), (-1.0, 0.0, 0.0), axis="x",
                              strict=False)
    # Either stop is equally close to a target directly behind, so the
    # test is that it went to A stop — not that it sat at zero calling
    # itself aimed, which is what an antipodal solution looks like.
    assert abs(pan) == pytest.approx(1.5, abs=1e-3)


def test_point_at_rejects_an_arm():
    """Point at rejects an arm

    Categories: sdk
    What it checks: Verifies that point at rejects an arm. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that known numerical fixtures validate pose, Jacobian, IK, and collision behavior.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the kinematics calculations code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    with pytest.raises(CapabilityNotSupported, match="kind pointing"):
        kin.point_at(group(), (1.0, 0.0, 0.0))


# -- guards -------------------------------------------------------------------------

def test_a_bare_object_is_not_a_group():
    """A bare object is not a group

    Categories: sdk
    What it checks: Verifies that a bare object is not a group. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that known numerical fixtures validate pose, Jacobian, IK, and collision behavior.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the kinematics calculations code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    class NotAGroup:
        name = "nope"

    with pytest.raises(TypeError, match="not a group handle"):
        kin.fk(NotAGroup())


def test_group_joints_must_follow_chain_order():
    """Group joints follow their model chain

    Categories: sdk
    What it checks: Rejects a controller group whose declared joints are out of order along the robot model chain.
    Why it matters: Joint ordering and measured mount state must produce consistent kinematics.
    If it fails: Inspect the SDK kinematics implementation and synthetic fixtures. This is a software regression, not a live robot profile adjustment.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    handle = group(joints=("j2", "j1", "j3"))
    with pytest.raises(ValueError, match="do not match the chain"):
        kin.ik(handle, Pose(x=1.0, y=0.0, z=0.2, frame="base_link"))


def test_no_description_is_a_readable_error():
    """No description is a readable error

    Categories: sdk
    What it checks: Verifies that no description is a readable error. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that known numerical fixtures validate pose, Jacobian, IK, and collision behavior.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the kinematics calculations code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    handle = group()
    handle._owner.description = None
    with pytest.raises(CapabilityNotSupported, match="no robot description"):
        kin.fk(handle, q=[0, 0, 0])


# -- the dependency promise (spec 1.6) ---------------------------------------------

def test_the_backend_is_never_imported_at_module_scope():
    """The backend is never imported at module scope

    Categories: sdk
    What it checks: `pip install ringfour` with no extras must import and run.

    Checked by reading the module rather than by uninstalling: the import
    has to be inside a function, and a future edit that hoists it to the
    top would break every user who never asked for kinematics. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that known numerical fixtures validate pose, Jacobian, IK, and collision behavior.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the kinematics calculations code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.

    Technical notes:
    `pip install ringfour` with no extras must import and run.

    Checked by reading the module rather than by uninstalling: the import
    has to be inside a function, and a future edit that hoists it to the
    top would break every user who never asked for kinematics.
    """
    import inspect

    source = inspect.getsource(kin).split("def _backend")[0]
    assert "import roboticstoolbox" not in source
    assert "from spatialmath" not in source
    assert "import spatialmath" not in source


def test_a_missing_backend_says_what_to_install(monkeypatch):
    """A missing backend says what to install

    Categories: sdk
    What it checks: The error a user without the extra actually sees. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that known numerical fixtures validate pose, Jacobian, IK, and collision behavior.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the kinematics calculations code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.

    Technical notes:
    The error a user without the extra actually sees.
    """
    import builtins

    real_import = builtins.__import__

    def blocked(name, *args, **kwargs):
        if name.split(".")[0] in ("roboticstoolbox", "spatialmath"):
            raise ImportError(f"No module named {name!r}")
        return real_import(name, *args, **kwargs)

    monkeypatch.setattr(builtins, "__import__", blocked)
    kin._model_for.cache_clear()
    with pytest.raises(CapabilityNotSupported) as exc:
        kin.ik(group(), Pose(x=1.0, y=0.0, z=0.2, frame="base_link"))
    assert "ringfour[kinematics]" in str(exc.value)


def test_fk_and_limits_work_with_no_backend_at_all(monkeypatch):
    """Fk and limits work with no backend at all

    Categories: sdk
    What it checks: Forward kinematics is a product of 4x4s, not a solve.

    Making it native means `robot.description` plus `kin.fk` answers
    "where would the gripper be" for a user who never installed the
    extra — and it is what lets `link=` name a link above the group's
    first joint, which a chain built for the group cannot reach. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that known numerical fixtures validate pose, Jacobian, IK, and collision behavior.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the kinematics calculations code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.

    Technical notes:
    Forward kinematics is a product of 4x4s, not a solve.

    Making it native means `robot.description` plus `kin.fk` answers
    "where would the gripper be" for a user who never installed the
    extra — and it is what lets `link=` name a link above the group's
    first joint, which a chain built for the group cannot reach.
    """
    import builtins

    real_import = builtins.__import__

    def blocked(name, *args, **kwargs):
        if name.split(".")[0] in ("roboticstoolbox", "spatialmath"):
            raise ImportError(f"No module named {name!r}")
        return real_import(name, *args, **kwargs)

    monkeypatch.setattr(builtins, "__import__", blocked)
    assert kin.limits(group())["j1"] == (-3.0, 3.0)
    assert len(kin.links(group())) == 4
    assert kin.fk(group(), q=[0.0, 0.0, 0.0]).x == pytest.approx(2.1)
    assert kin.fk(group(), q=[0, 0, 0], link="deck").z == pytest.approx(0.2)


@pytest.mark.parametrize("joint_type,axis,value", [
    ("prismatic", "0 0 1", 0.4),
    ("revolute", "0 0 1", 0.6),
])
def test_independent_mount_fk_ik_and_cache(joint_type, axis, value):
    """Kinematics follows a moving mount

    Categories: sdk
    What it checks: Checks forward and inverse kinematics with measured prismatic or revolute mounting joints, including cache updates and invalid state rejection.
    Why it matters: Joint ordering and measured mount state must produce consistent kinematics.
    If it fails: Inspect the SDK kinematics implementation and synthetic fixtures. This is a software regression, not a live robot profile adjustment.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    urdf = URDF.replace('name="deck_mount" type="fixed"',
                        f'name="deck_mount" type="{joint_type}"').replace(
        '<origin xyz="0 0 0.2"/>',
        f'<origin xyz="0 0 0.2"/><axis xyz="{axis}"/>'
        '<limit lower="-1" upper="1"/>')
    handle = group(urdf=urdf, mask=(1, 1, 1, 0, 0, 0))
    q = [0.4, 0.6, -0.3]
    state = {"deck_mount": value, **dict(zip(JOINTS, q))}
    handle.kinematic_positions = state
    pose = kin.fk(handle, q=q)
    baseline = kin.fk(handle, q=q, state={"deck_mount": 0.0})
    if joint_type == "prismatic":
        assert pose.z == pytest.approx(baseline.z + value)
    else:
        assert pose.x == pytest.approx(math.cos(value)*baseline.x - math.sin(value)*baseline.y)
        assert pose.y == pytest.approx(math.sin(value)*baseline.x + math.cos(value)*baseline.y)
    solved = kin.ik(handle, pose, seed=q, state=state)
    assert len(solved.joints) == len(JOINTS)
    assert solved.vector(JOINTS) == pytest.approx(q, abs=1e-5)
    assert handle.kinematic_positions == state
    handle.kinematic_positions = {**state, "deck_mount": 0.0}
    changed = kin._chain_of(handle)[3]
    fixed = kin._chain_of(handle, state=state)[3]
    assert changed is not fixed
    assert kin.ik(handle, pose, seed=q, state=state) == solved
    assert kin.jacobian(handle, q=q).shape == (6, 3)
    with pytest.raises(ValueError, match="missing measured"):
        kin.ik(handle, pose, state={})
    with pytest.raises(ValueError, match="finite"):
        kin.fk(handle, state={"deck_mount": float("nan")})


def test_partial_group_freezes_downstream_joint():
    """Partial groups hold other joints at measured positions

    Categories: sdk
    What it checks: Checks forward and inverse kinematics for a group that controls only part of the chain while another joint stays at its measured position.
    Why it matters: Joint ordering and measured mount state must produce consistent kinematics.
    If it fails: Inspect the SDK kinematics implementation and synthetic fixtures. This is a software regression, not a live robot profile adjustment.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    handle = group(joints=("j1", "j2"), mask=(1, 0, 1, 0, 0, 0))
    state = {"j3": 0.3}
    pose = kin.fk(handle, q=[0.2, 0.4], state=state)
    solved = kin.ik(handle, pose, seed=[0.2, 0.4], state=state)
    assert solved.vector(("j1", "j2")) == pytest.approx([0.2, 0.4], abs=1e-5)


def test_tipless_ik_rejects_before_requesting_external_snapshot():
    """Tip-less IK rejects before requesting joint snapshots

    Categories: sdk
    What it checks: A group on an independent moving mount with no tip rejects IK with its no-tip explanation without requesting external-joint snapshots.
    Why it matters: An older agent's missing snapshot support must not obscure the interface error for a joint-space-only group.
    If it fails: Check the order of tip validation and state acquisition in SDK IK. No live profile adjustment is needed.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    from ringfour.errors import CapabilityNotSupported
    class NoSnapshotGroup(FakeGroup):
        @property
        def kinematic_positions(self):
            pytest.fail('A no-tip request attempted to read an external snapshot')
    urdf=URDF.replace('name="deck_mount" type="fixed"','name="deck_mount" type="prismatic"').replace(
        '<origin xyz="0 0 0.2"/>','<origin xyz="0 0 0.2"/><axis xyz="0 0 1"/><limit lower="0" upper="1"/>')
    handle=NoSnapshotGroup(GroupInfo(kind=GroupKind.ARM,joints=JOINTS,tip=None),urdf=urdf)
    with pytest.raises(CapabilityNotSupported,match='no tip'):
        kin.ik(handle,Pose(x=0,y=0,frame=handle._owner.description.root))
