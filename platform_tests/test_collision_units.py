"""Unit tests for `ringfour.collision`, offline.

Two halves: the intersection test, checked against overlaps worked out on
paper, and the allowed-collision matrix, checked against a URDF built to
contain each case it has to recognise.

The fixture robot is deliberately shaped like a CAD export: one physical
part split into several links joined by FIXED joints, with collision boxes
that interpenetrate slightly — because that is what makes a naive
adjacency rule report 72% of configurations as colliding.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from ringfour.collision import CollisionModel, _overlap, _rpy, link_poses
from ringfour.description import RobotDescription

I = np.eye(3)
HALF = np.array([0.1, 0.1, 0.1])


# -- the separating-axis test ----------------------------------------------------

@pytest.mark.parametrize("dx,depth", [
    (0.0, 0.2),        # coincident: full 2 x half
    (0.15, 0.05),
    (0.199, 0.001),
    (0.2, 0.0),        # exactly touching is not colliding
    (0.201, 0.0),
    (1.0, 0.0),
])
def test_axis_aligned_boxes(dx, depth):
    """Axis aligned boxes

    Categories: sdk
    What it checks: The case a degenerate cross-product axis silently broke: two
    PARALLEL boxes. Every edge-edge axis is undefined for them, and using
    it anyway drove the reported depth to ~1e-10 — so boxes at the same
    centre read as clear. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that geometry transforms, exclusions, and intersection calculations are consistent.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the collision geometry code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.

    Technical notes:
    The case a degenerate cross-product axis silently broke: two
    PARALLEL boxes. Every edge-edge axis is undefined for them, and using
    it anyway drove the reported depth to ~1e-10 — so boxes at the same
    centre read as clear.
    """
    assert _overlap(np.zeros(3), I, HALF,
                    np.array([dx, 0.0, 0.0]), I, HALF) == pytest.approx(
                        depth, abs=1e-9)


def test_rotated_box_reaches_further_by_its_corner():
    """Rotated box reaches further by its corner

    Categories: sdk
    What it checks: A 45-degree box reaches sqrt(2)*half along x, so contact happens
    further out than for an aligned one. This is the case the edge-edge
    axes exist for. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that geometry transforms, exclusions, and intersection calculations are consistent.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the collision geometry code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.

    Technical notes:
    A 45-degree box reaches sqrt(2)*half along x, so contact happens
    further out than for an aligned one. This is the case the edge-edge
    axes exist for.
    """
    rotated = _rpy((0.0, 0.0, math.pi / 4))
    reach = 0.1 * math.sqrt(2) + 0.1          # 0.2414
    assert _overlap(np.zeros(3), I, HALF,
                    np.array([reach - 0.002, 0, 0]), rotated, HALF) > 0.0
    assert _overlap(np.zeros(3), I, HALF,
                    np.array([reach + 0.002, 0, 0]), rotated, HALF) == 0.0


def test_overlap_is_symmetric():
    """Overlap is symmetric

    Categories: sdk
    What it checks: Verifies that overlap is symmetric. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that geometry transforms, exclusions, and intersection calculations are consistent.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the collision geometry code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    a_centre, b_centre = np.zeros(3), np.array([0.12, 0.03, 0.0])
    rotated = _rpy((0.3, 0.0, 0.7))
    forward = _overlap(a_centre, I, HALF, b_centre, rotated, HALF)
    backward = _overlap(b_centre, rotated, HALF, a_centre, I, HALF)
    assert forward == pytest.approx(backward, abs=1e-9)


def test_separation_on_any_axis_is_enough():
    """Separation on any axis is enough

    Categories: sdk
    What it checks: Overlapping in x and y but clear in z is not a collision. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that geometry transforms, exclusions, and intersection calculations are consistent.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the collision geometry code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.

    Technical notes:
    Overlapping in x and y but clear in z is not a collision.
    """
    assert _overlap(np.zeros(3), I, HALF,
                    np.array([0.05, 0.05, 0.5]), I, HALF) == 0.0


# -- the fixture robot -----------------------------------------------------------

# base -> [fixed] bracket -> [fixed] servo   <- one rigid body, boxes overlap
#      -> [revolute j1] upper -> [fixed] pad <- second body, overlaps the first
#      -> [revolute j2] far                  <- can swing into `post`
# post is a fixed obstacle bolted to base, like a lidar mount.
URDF = """<?xml version="1.0"?>
<robot name="fixture">
  <link name="base">
    <collision><geometry><box size="0.2 0.2 0.05"/></geometry></collision>
  </link>
  <link name="bracket">
    <collision><origin xyz="0 0 0.02"/>
      <geometry><box size="0.06 0.06 0.06"/></geometry></collision>
  </link>
  <link name="servo">
    <collision><geometry><box size="0.05 0.05 0.05"/></geometry></collision>
  </link>
  <link name="upper">
    <collision><origin xyz="0.1 0 0"/>
      <geometry><box size="0.2 0.03 0.03"/></geometry></collision>
  </link>
  <link name="pad">
    <collision><geometry><box size="0.04 0.04 0.04"/></geometry></collision>
  </link>
  <link name="post">
    <collision><geometry><cylinder radius="0.03" length="0.3"/></geometry>
    </collision>
  </link>
  <link name="frame_only"/>
  <joint name="f_bracket" type="fixed">
    <parent link="base"/><child link="bracket"/><origin xyz="0 0 0.03"/>
  </joint>
  <joint name="f_servo" type="fixed">
    <parent link="bracket"/><child link="servo"/><origin xyz="0.01 0 0.02"/>
  </joint>
  <joint name="j1" type="revolute">
    <parent link="servo"/><child link="upper"/>
    <origin xyz="0 0 0.03"/><axis xyz="0 1 0"/>
    <limit lower="-3.14" upper="3.14"/>
  </joint>
  <joint name="f_pad" type="fixed">
    <parent link="upper"/><child link="pad"/><origin xyz="0.02 0 0"/>
  </joint>
  <joint name="j2" type="revolute">
    <parent link="upper"/><child link="far"/>
    <origin xyz="0.2 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14"/>
  </joint>
  <joint name="f_post" type="fixed">
    <parent link="base"/><child link="post"/><origin xyz="0.18 0 0.15"/>
  </joint>
  <joint name="f_frame" type="fixed">
    <parent link="base"/><child link="frame_only"/>
  </joint>
  <link name="far">
    <collision><origin xyz="0.05 0 0"/>
      <geometry><box size="0.1 0.03 0.03"/></geometry></collision>
  </link>
</robot>
"""

JOINTS = ("j1", "j2")


@pytest.fixture(scope="module")
def description() -> RobotDescription:
    return RobotDescription.parse(URDF)


@pytest.fixture(scope="module")
def model(description) -> CollisionModel:
    return CollisionModel.build(description, JOINTS)


# -- parsing ---------------------------------------------------------------------

def test_primitives_become_half_extents(description):
    """Primitives become half extents

    Categories: sdk
    What it checks: Verifies that primitives become half extents. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that geometry transforms, exclusions, and intersection calculations are consistent.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the collision geometry code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    base = description.collisions["base"][0]
    assert base.kind == "box"
    assert base.half == (0.1, 0.1, 0.025)
    post = description.collisions["post"][0]
    assert post.kind == "cylinder"
    # A cylinder is reduced to the box containing it: radius, radius,
    # half-length. Conservative, and it keeps one intersection test.
    assert post.half == (0.03, 0.03, 0.15)


def test_a_link_with_no_collision_is_absent(description):
    """A link with no collision is absent

    Categories: sdk
    What it checks: Pure frames have no extent and must not be invented. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that geometry transforms, exclusions, and intersection calculations are consistent.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the collision geometry code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.

    Technical notes:
    Pure frames have no extent and must not be invented.
    """
    assert "frame_only" not in description.collisions


# -- the allowed-collision matrix ------------------------------------------------

def test_links_joined_by_fixed_joints_are_one_body(description):
    """Links joined by fixed joints are one body

    Categories: sdk
    What it checks: Verifies that links joined by fixed joints are one body. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that geometry transforms, exclusions, and intersection calculations are consistent.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the collision geometry code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    bodies = CollisionModel._bodies(description)
    assert bodies["base"] == bodies["bracket"] == bodies["servo"]
    assert bodies["upper"] == bodies["pad"]
    assert bodies["base"] != bodies["upper"]


def test_same_body_pairs_are_never_tested(model):
    """Same body pairs are never tested

    Categories: sdk
    What it checks: base/bracket/servo interpenetrate by construction and cannot move
    relative to each other. Testing them answers the same question
    forever. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that geometry transforms, exclusions, and intersection calculations are consistent.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the collision geometry code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.

    Technical notes:
    base/bracket/servo interpenetrate by construction and cannot move
    relative to each other. Testing them answers the same question
    forever.
    """
    pairs = {frozenset(pair) for pair in model.pairs}
    assert frozenset(("base", "bracket")) not in pairs
    assert frozenset(("bracket", "servo")) not in pairs
    assert frozenset(("upper", "pad")) not in pairs


def test_bodies_across_one_moving_joint_are_not_tested(model):
    """Bodies across one moving joint are not tested

    Categories: sdk
    What it checks: The two halves of a joint assembly are built touching. This is
    MoveIt's "adjacent" rule generalised to a URDF that splits parts
    across fixed joints — without it, base101 reported 72% of random
    configurations as colliding on one such pair alone. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that geometry transforms, exclusions, and intersection calculations are consistent.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the collision geometry code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.

    Technical notes:
    The two halves of a joint assembly are built touching. This is
    MoveIt's "adjacent" rule generalised to a URDF that splits parts
    across fixed joints — without it, base101 reported 72% of random
    configurations as colliding on one such pair alone.
    """
    pairs = {frozenset(pair) for pair in model.pairs}
    assert frozenset(("servo", "upper")) not in pairs
    assert frozenset(("base", "upper")) not in pairs      # same bodies
    assert frozenset(("upper", "far")) not in pairs       # across j2


def test_distant_pairs_are_tested(model):
    """Distant pairs are tested

    Categories: sdk
    What it checks: `far` is two moving joints from the base and can genuinely swing
    into the post. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that geometry transforms, exclusions, and intersection calculations are consistent.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the collision geometry code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.

    Technical notes:
    `far` is two moving joints from the base and can genuinely swing
    into the post.
    """
    pairs = {frozenset(pair) for pair in model.pairs}
    assert frozenset(("far", "post")) in pairs
    assert frozenset(("far", "base")) in pairs


def test_static_pairs_are_not_tested(model):
    """Static pairs are not tested

    Categories: sdk
    What it checks: post and base cannot move relative to each other. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that geometry transforms, exclusions, and intersection calculations are consistent.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the collision geometry code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.

    Technical notes:
    post and base cannot move relative to each other.
    """
    pairs = {frozenset(pair) for pair in model.pairs}
    assert frozenset(("post", "base")) not in pairs


# -- checking --------------------------------------------------------------------

def _poses(description, **q):
    return link_poses(description, q, description.collisions)


def test_a_clear_configuration_is_clear(description, model):
    """A clear configuration is clear

    Categories: sdk
    What it checks: Verifies that a clear configuration is clear. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that geometry transforms, exclusions, and intersection calculations are consistent.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the collision geometry code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    assert model.check(_poses(description, j1=0.0, j2=0.0)) == ()


def test_swinging_into_the_post_is_caught(description, model):
    """Swinging into the post is caught

    Categories: sdk
    What it checks: `far` starts along +x at the same height as the post; folding j1
    up and round brings it onto the post. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that geometry transforms, exclusions, and intersection calculations are consistent.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the collision geometry code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.

    Technical notes:
    `far` starts along +x at the same height as the post; folding j1
    up and round brings it onto the post.
    """
    found = [
        q for q in np.linspace(-math.pi, math.pi, 60)
        if model.check(_poses(description, j1=float(q), j2=0.0))
    ]
    assert found, "the arm can reach the post; the model should say so"


def test_reported_pairs_name_both_links(description, model):
    """Reported pairs name both links

    Categories: sdk
    What it checks: Verifies that reported pairs name both links. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that geometry transforms, exclusions, and intersection calculations are consistent.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the collision geometry code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    for angle in np.linspace(-math.pi, math.pi, 60):
        hits = model.check(_poses(description, j1=float(angle), j2=0.0))
        if hits:
            assert hits[0].a and hits[0].b and hits[0].a != hits[0].b
            assert hits[0].depth > 0.0
            assert "<->" in str(hits[0])
            return
    pytest.fail("expected at least one collision across the sweep")


def test_deepest_first(description, model):
    """Deepest first

    Categories: sdk
    What it checks: Verifies that deepest first. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that geometry transforms, exclusions, and intersection calculations are consistent.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the collision geometry code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    for angle in np.linspace(-math.pi, math.pi, 120):
        hits = model.check(_poses(description, j1=float(angle), j2=0.0))
        if len(hits) > 1:
            depths = [hit.depth for hit in hits]
            assert depths == sorted(depths, reverse=True)
            return


# -- what it cannot see ----------------------------------------------------------

MESH_URDF = URDF.replace(
    '<collision><origin xyz="0.05 0 0"/>\n'
    '      <geometry><box size="0.1 0.03 0.03"/></geometry></collision>',
    '<collision><geometry><mesh filename="package://x/far.stl"/></geometry>'
    '</collision>')


def test_a_mesh_link_is_reported_not_silently_ignored():
    """A mesh link is reported not silently ignored

    Categories: sdk
    What it checks: The file lives on the robot; a client holding a description it
    received over a socket cannot open `package://`. On base101 this is
    all three jaw links — the part most likely to hit something — so it
    has to be visible rather than absent. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that geometry transforms, exclusions, and intersection calculations are consistent.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the collision geometry code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.

    Technical notes:
    The file lives on the robot; a client holding a description it
    received over a socket cannot open `package://`. On base101 this is
    all three jaw links — the part most likely to hit something — so it
    has to be visible rather than absent.
    """
    description = RobotDescription.parse(MESH_URDF)
    model = CollisionModel.build(description, JOINTS)
    assert "far" in model.unmodelled
    assert "far" not in model.links


def test_determinism(description):
    """Determinism

    Categories: sdk
    What it checks: `ik` consults this and `ik`'s contract is determinism, so the
    sampling pass that derives the matrix must be seeded. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that geometry transforms, exclusions, and intersection calculations are consistent.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the collision geometry code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.

    Technical notes:
    `ik` consults this and `ik`'s contract is determinism, so the
    sampling pass that derives the matrix must be seeded.
    """
    first = CollisionModel.build(description, JOINTS)
    second = CollisionModel.build(description, JOINTS)
    assert first.pairs == second.pairs
    assert first.always_touching == second.always_touching
