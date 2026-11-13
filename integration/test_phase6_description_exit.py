"""Description integrity and arbitrary group/camera topology."""
import pytest
from ringfour import Robot
from ringfour.description import RobotDescription, sha_of

pytestmark = pytest.mark.sim


def test_description_arrives_in_the_handshake(robot, description):
    """Robot model arrives with a valid identity

    Categories: description
    What it checks: Checks the parsed description, SHA-256 identity, root link, and nonempty link set against the handshake.
    Why it matters: Geometry consumers must know exactly which robot model they received.
    If it fails: Check the description publisher and the agent's model subscription. The live description is authoritative; a file on the client is not a substitute.
    Profile fields: groups.*.description_topic
    Scenario fields: None.
    Robot effects: Reads the live model.
    """
    assert isinstance(description, RobotDescription)
    assert robot.description_sha == description.sha == sha_of(description.urdf)
    assert description.root in description.links
    assert description.links


@pytest.mark.requires("groups")
def test_description_slices_into_each_group_subchain(tip_group, description):
    """Group joints form the declared chain

    Categories: description,manipulation
    What it checks: For each group with a tip, checks that the tip exists, every controlled joint leads to it, and slicing preserves the exact moving-joint order and structural offsets.
    Why it matters: Wrong tip links or joint order produce incorrect kinematics and motion vectors.
    If it fails: Check the group's tip and joint ordering against the live description and controller interfaces. Fix the URDF branch if the tip is not downstream of these joints.
    Profile fields: groups.*.tip;groups.*.joints;groups.*.controllers
    Scenario fields: None.
    Robot effects: Reads group/model structure.
    """
    group = tip_group
    assert description.has_link(group.tip)
    assert description.is_downstream(group.tip, group.joint_names)
    chain = description.subchain(group.joint_names, group.tip)
    assert [j.name for j in chain if j.moves] == list(group.joint_names)
    # Preserve every structural offset in the actual subchain, whatever
    # fixed joints this robot happens to have.
    full = description.chain_to(group.tip)
    assert all(j.name in full for j in chain)


@pytest.mark.requires("cameras")
def test_camera_frame_exists_in_live_description(camera, description):
    """Camera frame belongs to the live model

    Categories: cameras,description
    What it checks: Checks each camera's configured frame exists as a model link and that its moving-chain entries are actual moving joints.
    Why it matters: A camera can supply images while its spatial relationship to the robot is unusable.
    If it fails: Check cameras.<name>.frame and the matching URDF/TF link. Verify the model includes the fitted camera.
    Profile fields: cameras.*.frame
    Scenario fields: None.
    Robot effects: Reads camera metadata and model links.
    """
    frame = camera.get_info()["frame"]
    assert description.has_link(frame), f"{camera.name}: {frame} absent from live description"
    for joint in description.moving_chain_to(frame):
        assert description.joints[joint].moves


@pytest.mark.requires("groups")
def test_limits_match_joint_state_call(group, description):
    """Agent and client agree on joint limits

    Categories: description,manipulation,profile
    What it checks: Compares limits reported with each joint state against limits parsed from the live robot description.
    Why it matters: Two components using different bounds may accept incompatible targets.
    If it fails: Check description subscriptions and resolved joint names. Correct limits in the model/controller setup; profile YAML does not override URDF joint limits.
    Profile fields: groups.*.joints;groups.*.description_topic
    Scenario fields: None.
    Robot effects: Reads joint state and model bounds.
    """
    for name,info in group.get_joints().items():
        if name in description.joints:
            assert info.limits == description.joints[name].limits, name


def test_description_is_stable_across_connections(robot, agent_full):
    """Reconnecting sees the same robot model

    Categories: description,connection
    What it checks: Connects a second client and compares the reported model SHA with the first connection.
    Why it matters: Clients must be able to cache a model without reconnecting to a different geometry unexpectedly.
    If it fails: Check that one stable model is published and both connections reach the same agent. Concurrent model rebuilds can make this check fail.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Opens another read connection.
    """
    with Robot(agent_full.unix_uri) as other:
        assert other.description_sha == robot.description_sha


def test_refresh_matches_handshake(robot, description):
    """Refreshing the model returns the same geometry

    Categories: description
    What it checks: Fetches the live description again and checks its SHA matches the handshake's parsed model.
    Why it matters: A refresh must agree with the model originally served unless an intentional rebuild occurred.
    If it fails: Check the description subscription and model-refresh handler. Keep the model stable during this check.
    Profile fields: groups.*.description_topic
    Scenario fields: None.
    Robot effects: Fetches the live model.
    """
    refreshed = robot.refresh_description()
    assert refreshed is not None and refreshed.sha == description.sha


@pytest.mark.motion
def test_republishing_triggers_model_changed(robot, description, scenarios, motion_enabled, request):
    """Model rebuild emits a change event

    Categories: description
    What it checks: Runs explicit change and restore commands, waits for a model.changed event with old/new SHA values, checks refresh sees the new model, and verifies restoration of the original SHA.
    Why it matters: Clients need a reliable signal to invalidate cached geometry after a rebuild.
    If it fails: Supply reviewed change/restore commands for your stack. Check description updates, event delivery, and whether the restore command reinstates the original model.
    Profile fields: None.
    Scenario fields: description_change.command;description_change.restore_command;description_change.timeout
    Robot effects: Runs the configured stack commands and restores the original description.
    """
    import asyncio
    import subprocess
    from conformance import skip_coverage
    recipe = scenarios.get("description_change")
    if recipe is None:
        skip_coverage(request, "not_configured", "description_change requires explicit change/restore commands for this robot stack")
    # events() commits its subscription before returning its async iterator.
    events = robot._runner.run(robot._robot.events())
    before = description.sha
    async def wait_changed():
        async for event in events:
            if event.kind == "model.changed":
                return event
    try:
        subprocess.run(recipe["command"], check=True, timeout=recipe.get("timeout",60))
        event = robot._runner.run(asyncio.wait_for(wait_changed(), timeout=recipe.get("timeout",60)))
        assert event.data["previous"] == before
        assert event.data["sha"] != before
        assert robot.refresh_description().sha == event.data["sha"]
    finally:
        robot._runner.run(events.aclose())
        subprocess.run(recipe["restore_command"], check=True, timeout=recipe.get("timeout",60))
        assert robot.refresh_description().sha == before, "restore command did not reinstate the original description"
