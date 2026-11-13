"""Kinematics and collision checks derived from each group's live structure."""
import math
import time
import numpy as np
import pytest
from ringfour import Pose
from ringfour import kinematics as kin
from ringfour.errors import CapabilityNotSupported, Unreachable
from conformance import validate_vector
from conformance import skip_coverage

pytestmark = [pytest.mark.sim, pytest.mark.requires("groups")]


@pytest.fixture(autouse=True)
def available_group_description_backend(request, welcome, group_name):
    sha = welcome.group_description_shas.get(group_name)
    if sha is not None and sha != welcome.description_sha:
        skip_coverage(request, "not_configured", f"SDK kinematics does not yet consume {group_name}'s separate description; group description wire checks still run")


def distance(a,b):
    return math.dist((a.x,a.y,a.z or 0),(b.x,b.y,b.z or 0))


def positions(group):
    q = group.get_joint_positions()
    return [q[n] for n in group.joint_names]


def in_mount_frame(group, description, link=None, state=None):
    """Compare the GROUP's contribution, independent of moving ancestors.

    Use the same measured snapshot for root-to-mount and root-to-link.
    Removing the mounting transform leaves the group-local geometry and
    avoids both moving-ancestor offsets and races between separate reads.
    """
    from ringfour.models import Quaternion
    mount = description.joints[group.joint_names[0]].parent
    state = group.get_kinematic_positions() if state is None else state
    q = [state[j] for j in group.joint_names]
    origin = kin.fk(group,q=q,link=mount,state=state)
    tip = kin.fk(group,q=q,link=link or group.tip,state=state)
    inverse = Quaternion(x=-origin.q.x,y=-origin.q.y,z=-origin.q.z,w=origin.q.w)
    x,y,z = inverse.rotate(tip.x-origin.x,tip.y-origin.y,(tip.z or 0)-(origin.z or 0))
    return Pose(x=x,y=y,z=z,q=inverse.multiply(tip.q),frame=mount)


def aligned_tf_state(group, description, request):
    from ringfour.errors import FrameExpired
    mount = description.joints[group.joint_names[0]].parent
    transform = group._owner.tf.lookup(mount, group.tip)
    if transform.stamp is None or transform.stamp <= 0:
        skip_coverage(request, "not_configured", "live FK/TF comparison needs timestamped TF and recorded joint feedback; update the agent")
    try:
        snapshot = group.kinematic_snapshot_at(transform.stamp)
    except (CapabilityNotSupported, FrameExpired) as exc:
        skip_coverage(request, "not_configured", str(exc))
    return snapshot.positions, snapshot.stamp


def test_fk_agrees_with_tf_tree(tip_group, description, scenarios, kinematic_state, request):
    """Calculated tool position agrees with live transforms

    Categories: kinematics,manipulation
    What it checks: Calculates the group's tip pose in its mounting frame and compares position and quaternion orientation with TF. Uses configured FK tolerances, defaulting to 0.00001.
    Why it matters: This independently checks model geometry, joint axes, offsets, and actual joint feedback without confusing a moving mount with arm motion.
    If it fails: Check tip, joint order, URDF axes/origins, TF timing, and encoder state. Use tolerances only for measured uncertainty; inspect the mounted group's own frame.
    Profile fields: groups.*.tip;groups.*.joints
    Scenario fields: tolerances.fk_position;tolerances.fk_quaternion
    Robot effects: Computes kinematics from read-only state.
    """
    state, stamp = aligned_tf_state(tip_group,description,request)
    group = tip_group
    computed = in_mount_frame(group,description,state=state)
    actual = group._owner.tf.pose_of(group.tip, computed.frame, stamp=stamp)
    assert computed.frame == actual.frame
    tolerances = scenarios.get("tolerances", {})
    assert distance(computed,actual) < tolerances.get("fk_position", 1e-5)
    # q and -q encode the same rotation; do not compare Euler angles at
    # their wrap boundary or at a gimbal singularity.
    a,b = computed.q,actual.q
    assert abs(a.x*b.x+a.y*b.y+a.z*b.z+a.w*b.w) == pytest.approx(1, abs=tolerances.get("fk_quaternion", 1e-5))


def test_fk_intermediate_links_agree(tip_group, description, scenarios, kinematic_state, request):
    """Intermediate joint geometry agrees with TF

    Categories: kinematics,description
    What it checks: Compares calculated and live positions for every link in the group's sliced chain, using the mounting frame and configured position tolerance.
    Why it matters: A missing fixed offset can be hidden at the tip but visible inside the chain.
    If it fails: Check intermediate URDF origins/axes and joint order. Inspect the named link in the failure details.
    Profile fields: groups.*.tip;groups.*.joints
    Scenario fields: tolerances.fk_position
    Robot effects: Reads model/state and computes poses.
    """
    state, stamp = aligned_tf_state(tip_group,description,request)
    group = tip_group
    for joint in description.subchain(group.joint_names, group.tip):
        computed = in_mount_frame(group,description,link=joint.child,state=state)
        actual = group._owner.tf.pose_of(joint.child, computed.frame, stamp=stamp)
        assert distance(computed,actual) < scenarios.get("tolerances", {}).get("fk_position", 1e-5), joint.child


def solver_configuration(group, state):
    """Choose a hypothetical in-limit configuration; never command it.

    A live joint-limit violation is tested separately. It must not make an
    inverse-solver round trip ask for a mathematically out-of-bounds solution.
    """
    result = []
    for joint in group.joint_names:
        value = state[joint]
        bounds = kin.limits(group)[joint]
        if bounds:
            margin = min(.05, (bounds[1]-bounds[0])*.02)
            value = max(bounds[0]+margin, min(bounds[1]-margin,value))
        result.append(value)
    return result


def test_ik_round_trips_real_chain(tip_group, kin_backend):
    """Inverse kinematics reproduces a known pose

    Categories: kinematics
    What it checks: Computes a pose from current joint values, solves back using those values as seed with collision rejection disabled, and checks the resulting position within 0.00001 metres.
    Why it matters: The solver must reproduce a configuration the same chain can already represent.
    If it fails: Check group joints, tip, DOF mask, live limits, and optional IK backend. Independently articulated mounts are currently untested by this backend.
    Profile fields: groups.*.joints;groups.*.tip;groups.*.dof_mask
    Scenario fields: None.
    Robot effects: Runs a numerical solve; does not command joints.
    """
    q = solver_configuration(tip_group,kin_backend)
    target = kin.fk(tip_group,q=q,state=kin_backend)
    answer = kin.ik(tip_group,target,seed=q,avoid_collisions=False,state=kin_backend)
    assert distance(kin.fk(tip_group,q=answer,state=kin_backend),target) < 1e-5


def test_ik_is_deterministic(tip_group, kin_backend):
    """The same IK request returns the same solution

    Categories: kinematics
    What it checks: Solves the same known pose five times with the same seed and collision checking disabled; compares joint vectors rounded to ten decimal places.
    Why it matters: Hidden random retries or branch changes make planning behavior hard to reproduce.
    If it fails: Check backend version and solver determinism. This is usually a solver regression rather than a profile adjustment.
    Profile fields: groups.*.dof_mask
    Scenario fields: None.
    Robot effects: Runs numerical solves only.
    """
    q = solver_configuration(tip_group,kin_backend)
    target = kin.fk(tip_group,q=q,state=kin_backend)
    solutions = {tuple(round(v,10) for v in kin.ik(tip_group,target,seed=q,avoid_collisions=False,state=kin_backend)) for _ in range(5)}
    assert len(solutions) == 1


def test_profile_dof_mask_is_consistent(tip_group, kin_backend):
    """Requested Cartesian freedoms fit the group

    Categories: kinematics,profile
    What it checks: Checks a six-entry binary DOF mask with a positive constrained count; constrained targets may be restricted to a reachable manifold, plus a finite 6-by-joint-count Jacobian.
    Why it matters: A solver cannot independently satisfy more Cartesian constraints than the group can control.
    If it fails: Set dof_mask according to independently measured controllable freedoms. Verify joint axes and the actual chain; do not disable freedoms merely to hide a faulty model.
    Profile fields: groups.*.dof_mask;groups.*.joints
    Scenario fields: None.
    Robot effects: Computes the Jacobian; does not move.
    """
    group = tip_group
    flags = group._owner.group_info(group.name).dof_mask or (1,)*6
    assert len(flags) == 6 and set(flags) <= {0,1}
    assert 0 < sum(flags) <= 6
    J = kin.jacobian(group,state=kin_backend)
    assert J.shape == (6,len(group.joint_names))
    assert np.isfinite(J).all()
    for value in (kin.reach(group,state=kin_backend), kin.manipulability(group,state=kin_backend)):
        assert math.isfinite(value) and value >= 0


def test_unreachable_names_group_and_tip(tip_group, description, kin_backend):
    """Unreachable IK target gives useful context

    Categories: kinematics
    What it checks: Builds an unreachable target from the chain's own reach bound, expects an Unreachable error naming the group and tip, and checks reachable() agrees.
    Why it matters: A failed solve should identify what cannot reach the target rather than silently returning a configuration.
    If it fails: Check tip/joint configuration and solver error translation. A fixed desktop-arm size is not assumed.
    Profile fields: groups.*.tip;groups.*.joints
    Scenario fields: None.
    Robot effects: Runs numerical reachability checks only.
    """
    group = tip_group
    extension = kin.reach(group,state=kin_backend)
    # A bound derived from the chain, not a metre-scale desktop arm.
    target = kin.fk(group,state=kin_backend)
    far = Pose(x=target.x+max(1,extension)*100, y=target.y, z=target.z, frame=description.root)
    with pytest.raises(Unreachable) as exc:
        kin.ik(group,far,avoid_collisions=False,state=kin_backend)
    assert group.name in str(exc.value) and group.tip in str(exc.value)
    assert not kin.reachable(group,far,avoid_collisions=False,state=kin_backend)


def test_limits_come_from_live_description(group, description):
    """Kinematics uses the same joint bounds as the agent

    Categories: kinematics,profile
    What it checks: Compares limits from the kinematics parser, live description, and agent joint-state call for every controlled joint.
    Why it matters: A solver using different bounds could return targets the controller cannot accept.
    If it fails: Check model subscriptions and joint resolution. Fix limits in the live URDF/controller configuration rather than adding unrelated YAML limits.
    Profile fields: groups.*.joints;groups.*.description_topic
    Scenario fields: None.
    Robot effects: Reads and compares joint bounds.
    """
    assert kin.limits(group) == {name: description.joints[name].limits for name in group.joint_names}
    assert kin.limits(group) == {name: info.limits for name,info in group.get_joints().items()}


def test_reach_is_finite(tip_group, kin_backend):
    """Chain reach can be calculated

    Categories: kinematics
    What it checks: Calculates the chain's maximum-extension upper bound and checks it is finite and nonnegative.
    Why it matters: A usable reach bound should come from the model, not a guessed robot size.
    If it fails: Check model transforms and the solver's supported chain topology. No fixed minimum arm size is required.
    Profile fields: groups.*.tip;groups.*.joints
    Scenario fields: None.
    Robot effects: Computes a geometric bound.
    """
    value = kin.reach(tip_group,state=kin_backend)
    assert math.isfinite(value) and value >= 0


def test_manipulability_is_finite(tip_group, kin_backend):
    """Manipulability can be evaluated

    Categories: kinematics
    What it checks: Calculates manipulability at the current configuration and checks a finite nonnegative result; zero is allowed at a singularity.
    Why it matters: A valid singularity metric must respect the freedoms the group can control.
    If it fails: Check DOF mask and joint axes. A singular current pose is valid; a malformed or overconstrained mask is not.
    Profile fields: groups.*.dof_mask
    Scenario fields: None.
    Robot effects: Computes a metric without moving.
    """
    value = kin.manipulability(tip_group,state=kin_backend)
    assert math.isfinite(value) and value >= 0  # singular configurations legitimately yield zero


def test_group_without_tip_refuses_ik(group, description, request):
    """A tip-less group does not pretend to support IK

    Categories: kinematics,profile
    What it checks: For a real group without a tip, expects IK to reject the request with a no-tip explanation while joint-limit lookup still works.
    Why it matters: Joint-space lifts or heads should remain usable without inventing an end-effector target.
    If it fails: Leave tip absent when the group has no Cartesian endpoint. Inspect client capability gating if the rejection or limit lookup is wrong.
    Profile fields: groups.*.tip
    Scenario fields: None.
    Robot effects: Checks API gating and reads limits.
    """
    if group.tip:
        skip_coverage(request, "not_applicable", "group declares a tip")
    with pytest.raises(CapabilityNotSupported, match="no tip"):
        kin.ik(group,Pose(x=0,y=0,frame=description.root))
    assert set(kin.limits(group)) == set(group.joint_names)


def test_collision_model_covers_reported_geometry(group, description):
    """Collision model reports what geometry it can see

    Categories: kinematics,safety
    What it checks: Checks modelled/unmodelled links exist in the description, unresolved meshes are explicitly reported without a snapshot, and collision pairs refer to distinct modelled links.
    Why it matters: Invisible mesh geometry must not be mistaken for checked empty space.
    If it fails: Check URDF collision geometry and record a matching snapshot when mesh volumes are needed. Missing geometry is not fixed by renaming a group.
    Profile fields: groups.*.description_topic
    Scenario fields: None.
    Robot effects: Builds a collision model; does not move.
    """
    model = kin.collision_model(group)
    assert set(model.links) <= set(description.links)
    assert set(model.unmodelled) <= set(description.links)
    expected_unmodelled = {name for name,shapes in description.collisions.items() if any(not s.modelled for s in shapes)}
    if kin.snapshot(group) is None:
        assert set(model.unmodelled) == expected_unmodelled
    for a,b in model.pairs:
        assert a != b and a in model.links and b in model.links


def test_snapshot_matches_live_robot(group, request):
    """Recorded geometry matches the current model

    Categories: kinematics
    What it checks: Checks an available snapshot is keyed to the live description SHA and rejects an unrelated SHA. Missing recording is untested.
    Why it matters: A geometry recording from an older robot must not silently be reused.
    If it fails: Record a snapshot for the current description and verify its storage/SHA. Re-record after geometry changes.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Reads recorded geometry.
    """
    recorded = kin.snapshot(group)
    if recorded is None:
        skip_coverage(request, "not_configured", "no recorded snapshot for this description")
    assert recorded.sha == group._owner.description_sha
    assert recorded.matches(group._owner.description_sha)
    assert not recorded.matches("0"*64)


def test_reach_map_is_internally_consistent(group, request, description):
    """Recorded reach map answers consistent queries

    Categories: kinematics
    What it checks: Checks positive resolution and occupied voxels, verifies the densest occupied point is reachable, and rejects a point derived outside the grid.
    Why it matters: Precomputed reachability should be useful and geometrically consistent.
    If it fails: Record a reach map for the current description and inspect its counts, resolution, and bounds. Missing recording is untested.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Reads and queries a recorded reach map.
    """
    if not group.tip:
        skip_coverage(request, "not_applicable", "joint-space group has no Cartesian reach map")
    external = set(description.moving_chain_to(group.tip)) - set(group.joint_names)
    if external:
        skip_coverage(request, "not_applicable", "fixed reach maps do not apply to a group on independently moving joints")
    found = kin.reach_map(group)
    if found is None:
        skip_coverage(request, "not_configured", "no reach map for this description")
    assert found.resolution > 0 and found.reachable_voxels > 0
    points,counts = found.occupied()
    point = tuple(points[counts.argmax()])
    assert found.can_reach(point) and found.density(point) == pytest.approx(1)
    outside = tuple(o+(n+10)*found.resolution for o,n in zip(found.origin,found.counts.shape))
    assert found.index_of(outside) is None and not found.can_reach(outside)


def test_rigid_bodies_are_excluded_from_collision_pairs(group, description):
    """Rigidly connected parts are not checked against themselves

    Categories: kinematics,safety
    What it checks: Checks collision pairs exclude links belonging to the same rigid body after fixed joints are collapsed.
    Why it matters: Treating a bolted assembly as self-collision creates false collision reports.
    If it fails: Check fixed-joint relationships and collision-model body grouping. This is typically a geometry/model issue.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Computes collision pairs.
    """
    model = kin.collision_model(group)
    bodies = model._bodies(description)
    assert all(bodies[a] != bodies[b] for a,b in model.pairs)


def test_known_clear_configurations(group, group_recipe):
    """Reviewed clear poses are collision-free

    Categories: kinematics,safety
    What it checks: Evaluates each independently validated clear joint configuration and expects both collision APIs to report no collision.
    Why it matters: Known physical examples provide independent evidence beyond internal model consistency.
    If it fails: Check the recorded clear configurations, model origins/collision volumes, and snapshot freshness. Use real measured examples, not arbitrary zero vectors.
    Profile fields: None.
    Scenario fields: groups.*.clear_configurations
    Robot effects: Computes collisions at hypothetical joint values; does not move.
    """
    vectors = group_recipe.get("clear_configurations")
    if not vectors:
        pytest.skip("NOT CONFIGURED: no independently validated clear configurations")
    for vector in vectors:
        q = validate_vector(vector,group.joint_names,group.name)
        assert kin.collisions(group,q=q,state=group_recipe.get("clear_state")) == ()
        assert not kin.in_collision(group,q=q,state=group_recipe.get("clear_state"))


def test_known_collision_configurations(group, group_recipe):
    """Reviewed colliding poses are detected

    Categories: kinematics,safety
    What it checks: Evaluates each independently known colliding configuration and expects nonempty collisions and true in_collision().
    Why it matters: A model must detect contact examples it claims to cover.
    If it fails: Check the named joint maps, collision geometry, excluded pairs, and snapshot. Supply independently verified examples.
    Profile fields: None.
    Scenario fields: groups.*.collision_configurations
    Robot effects: Computes hypothetical collisions; does not command contact.
    """
    vectors = group_recipe.get("collision_configurations")
    if not vectors:
        pytest.skip("NOT CONFIGURED: no independently validated colliding configurations")
    for vector in vectors:
        q = validate_vector(vector,group.joint_names,group.name)
        assert kin.collisions(group,q=q,state=group_recipe.get("collision_state"))
        assert kin.in_collision(group,q=q,state=group_recipe.get("collision_state"))


def test_ik_refuses_colliding_configuration(tip_group, group_recipe, kin_backend):
    """IK rejects a solution that intersects the robot

    Categories: kinematics,safety
    What it checks: Uses a known collision fixture, verifies it collides, solves its pose with rejection disabled, then expects the default solver to reject it as going through itself.
    Why it matters: Collision-aware reachability must not return an intersecting configuration as safe.
    If it fails: Check the collision fixture/model and default rejection behavior. This test never physically drives into the collision.
    Profile fields: None.
    Scenario fields: groups.*.collision_configurations
    Robot effects: Runs numerical solves and collision checks only.
    """
    vectors = group_recipe.get("collision_configurations")
    if not vectors:
        pytest.skip("NOT CONFIGURED: no known collision fixture")
    group = tip_group
    kin_backend = group_recipe.get("collision_state", kin_backend)
    q = validate_vector(vectors[0],group.joint_names,group.name)
    assert kin.in_collision(group,q=q,state=kin_backend), "independent collision fixture must actually collide"
    target = kin.fk(group,q=q,state=kin_backend)
    loose = kin.ik(group,target,seed=q,avoid_collisions=False,state=kin_backend)
    assert kin.in_collision(group,q=loose,state=kin_backend)
    with pytest.raises(Unreachable,match="through itself"):
        kin.ik(group,target,seed=q,state=kin_backend)


def test_collision_checking_meets_configured_budget(group, group_recipe):
    """Collision queries fit the reviewed time budget

    Categories: kinematics
    What it checks: Warms the model, runs five collision queries at current joint values, and compares average time with this robot's configured budget.
    Why it matters: Planning loops need a measured performance budget appropriate to their geometry and hardware.
    If it fails: Profile model complexity and compute performance. Set max_check_seconds from the mission's actual budget, not a universal desktop-arm number.
    Profile fields: None.
    Scenario fields: groups.*.max_check_seconds
    Robot effects: Benchmarks computations; does not move.
    """
    budget = group_recipe.get("max_check_seconds")
    if budget is None:
        pytest.skip("NOT CONFIGURED: no collision performance budget for this morphology")
    q = positions(group)
    kin.collisions(group,q=q)
    start = time.monotonic()
    for _ in range(5):
        kin.collisions(group,q=q)
    assert (time.monotonic()-start)/5 < budget
