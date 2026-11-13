"""Configured motion contracts for every group, with arbitrary joint count."""
import time
import pytest
from ringfour import Controller, Robot, JointTrajectory
from ringfour.enums import AuditKind, Phase, OperationState
from ringfour.errors import CapabilityNotSupported, RingfourError, SafetyViolation
from conformance import skip_coverage

pytestmark = [pytest.mark.sim, pytest.mark.requires("groups"), pytest.mark.motion]


def positions(group):
    values = group.get_joint_positions()
    return [values[name] for name in group.joint_names]


def trajectory(group,waypoints,recipe):
    return JointTrajectory.from_waypoints([group.configuration(dict(zip(group.joint_names,row))) for row in waypoints],
                                        step_duration=recipe.get('dt',2.))


def test_follow_waypoints_and_converge(group_motion):
    """Joint group reaches its configured trajectory

    Categories: manipulation
    What it checks: Checks the first waypoint requires observable motion, follows all reviewed named-joint targets, verifies controller identity and status/progress, and checks actual movement and final joints against tolerance; verifies restoration of initial joints.
    Why it matters: Controller success alone does not prove that each joint arrived.
    If it fails: Check trajectory-controller mapping, joint order, live limits, actuator feedback, waypoint feasibility, duration, and independently measured tolerance.
    Profile fields: groups.*.controllers.trajectory;groups.*.joints
    Scenario fields: groups.*.waypoints;groups.*.dt;groups.*.settle;groups.*.tolerance
    Robot effects: Moves the group and restores its initial joints.
    """
    group, waypoints, recipe = group_motion
    assert any(abs(a-b) > recipe.get("tolerance", .01) for a,b in zip(positions(group),waypoints[0])), "first waypoint must exercise motion"
    initial = positions(group)
    moved = state_changed = False
    seen = []
    progress = []
    moved_joints = set()
    with group.control() as session:
        with session.start_trajectory(trajectory(group,waypoints,recipe)) as op:
            for update in op.updates():
                status=update.progress
                if status is not None and status.positions is not None:
                    measured={j.name:j.position for j in status.positions}
                    moved |= any(abs(measured[j]-value)>recipe.get('tolerance',.01) for j,value in zip(group.joint_names,initial))
                    moved_joints.update(j for j,value in zip(group.joint_names,initial) if abs(measured[j]-value)>recipe.get('tolerance',.01))
                state_changed |= any(abs(value-before)>recipe.get('tolerance',.01) for value,before in zip(positions(group),initial))
            result=op.result()
            assert result.stop.verified
        if recipe.get('probe',{}).get('mode')=='small':
            assert moved_joints==set(group.joint_names), f'Generated probes did not produce observed motion for {set(group.joint_names)-moved_joints}'
        assert moved, 'Trajectory completed without observable measured joint motion'
        assert state_changed, 'SDK joint-state reads did not track the observed trajectory motion'
        time.sleep(recipe.get('settle',.5))
        assert positions(group)==pytest.approx(waypoints[-1],abs=recipe.get('tolerance',.01))


def test_progress_climbs_monotonically(group_motion):
    """Trajectory progress advances to completion

    Categories: manipulation
    What it checks: Follows reviewed waypoints, checks reported elapsed-time progress never decreases, and requires a verified terminal result; restores initial joints.
    Why it matters: A progress display must distinguish elapsed-time estimates from measured arrival.
    If it fails: Check trajectory feedback/timing, agent/client status conversion, and the configured waypoint duration.
    Profile fields: groups.*.controllers.trajectory
    Scenario fields: groups.*.waypoints;groups.*.dt
    Robot effects: Moves the group and restores its initial joints.
    """
    group, waypoints, recipe = group_motion
    values=[]
    with group.control() as session, session.start_trajectory(trajectory(group,waypoints,recipe)) as op:
        for update in op.updates():
            if update.progress and update.progress.elapsed_fraction is not None:values.append(update.progress.elapsed_fraction)
        assert op.result().stop.verified
    assert values==sorted(values)


def test_break_cancels_trajectory(group_motion):
    """Leaving the control scope cancels unfinished trajectory work

    Categories: manipulation,safety
    What it checks: Breaks during observable mid-trajectory progress, records stopped joints, waits the configured settling time, and checks joints stay within tolerance; restores initial joints.
    Why it matters: The explicit control scope must clean up unfinished motion; breaking iteration only stops observation.
    If it fails: Use a sufficiently long reviewed trajectory and check cancellation, braking/hold behavior, settling time, and measured tolerance.
    Profile fields: groups.*.controllers.trajectory
    Scenario fields: groups.*.waypoints;groups.*.dt;groups.*.settle;groups.*.tolerance
    Robot effects: Moves, cancels, and restores the group.
    """
    group, waypoints, recipe = group_motion
    broken=False
    with group.control() as session:
        with session.start_trajectory(trajectory(group,waypoints,recipe)) as op:
            for update in op.updates():
                if update.progress and update.progress.elapsed_fraction is not None and .05<update.progress.elapsed_fraction<.95:
                    broken=True;break
        assert op.snapshot.state is OperationState.CANCELLED
        stopped=positions(group)
        time.sleep(recipe.get('settle',.5))
        assert positions(group)==pytest.approx(stopped,abs=recipe.get('tolerance',.01))
    assert broken, 'Trajectory must expose mid-run progress to exercise cancellation'


def test_second_client_is_refused_while_lock_is_held(session_group, agent_full):
    """Only one client controls a group at a time

    Categories: manipulation,safety
    What it checks: Holds a trajectory session, expects a second client to be refused with a typed ownership reason, then checks it can acquire the released session.
    Why it matters: Two command writers must not fight for the same actuators.
    If it fails: Check controller-role mappings and motion-lock/session release handling. Explicitly enable controller-switch checks for this group.
    Profile fields: groups.*.controllers.trajectory
    Scenario fields: groups.*.allow_sessions
    Robot effects: Switches/acquires controller sessions; sends no trajectory.
    """
    group = session_group
    with Robot(agent_full.unix_uri) as second:
        with group.control(Controller.TRAJECTORY):
            with pytest.raises(SafetyViolation) as exc:
                with second.groups[group.name].control(Controller.TRAJECTORY):
                    pass
            assert exc.value.reason == "motion lock held"
        with second.groups[group.name].control(Controller.TRAJECTORY) as released:
            assert released.integration['controller']


def test_nothing_moves_outside_session(session_group):
    """A trajectory request requires a control session

    Categories: manipulation,safety
    What it checks: Sends a follow request containing current joint values without acquiring control and expects a no-control-session error.
    Why it matters: The motion lock must not be bypassable through a raw command.
    If it fails: Inspect the group's command/session gate. Changing profile names cannot repair a missing ownership check.
    Profile fields: groups.*.controllers.trajectory
    Scenario fields: groups.*.allow_sessions
    Robot effects: Submits a deliberately unauthorized trajectory request using current values.
    """
    group = session_group
    with pytest.raises(RingfourError, match="no control session"):
        group._owner._runner.run(group._owner._robot.connection.call("group.start_trajectory", {
            "group": group.name, "session_id": "not-an-acquired-session", "trajectory": JointTrajectory.from_waypoints([group.configuration(group.get_joint_positions())], step_duration=1.).model_dump(mode="json")}))


def test_session_restores_controllers_on_exit(session_group, record_property):
    """Controller sessions reopen consistently

    Categories: manipulation
    What it checks: Opens and closes two trajectory sessions, checks the controller and transition lists match, and verifies new audit events when advertised. This indirectly checks session restoration; the SDK does not expose actual ROS controller states.
    Why it matters: A test or mission must not leave a different controller configuration after exiting.
    If it fails: Check trajectory/stream role bindings, controller lifecycle and resource claims, and restoration handling.
    Profile fields: groups.*.controllers
    Scenario fields: groups.*.allow_sessions
    Robot effects: Switches controller sessions twice.
    """
    group = session_group
    expected = group._owner.integration.group_controllers(group.name)[Controller.TRAJECTORY]
    transitions = []
    before_audit = group._owner.audit.query(kind=AuditKind.COMMAND, limit=100) if group._owner.has("audit") else []
    for _ in range(2):
        with group.control(Controller.TRAJECTORY) as session:
            assert session.integration['controller'] == expected
            assert all(isinstance(n,str) and n for n in session.integration['loaded'] + session.integration['deactivated'])
            transitions.append((session.integration['loaded'],session.integration['deactivated']))
    assert transitions[0] == transitions[1], "controller session transitions differ between identical acquisitions"
    record_property("verification.controller_restoration", "Indirect: matching session transitions; ROS controller state is not exposed by SDK")
    if group._owner.has("audit"):
        events = group._owner.audit.query(kind=AuditKind.COMMAND, limit=100)
        # Only new events can provide evidence for these acquisitions.
        recent = [e for e in events if e not in before_audit]
        opens = [e for e in recent if e.call == "group.control" and e.detail.get("group") == group.name]
        closes = [e for e in recent if e.call == "group.control_stop" and e.detail.get("group") == group.name]
        assert len(opens) == 2 and len(closes) == 2
        for event in opens:
            assert event.kind is AuditKind.COMMAND
            assert event.detail["controller"] == expected
            assert event.detail["role"] == "trajectory"
    # Loaded/deactivated lists depend on runtime controller state. They
    # must not be asserted against one robot's boot configuration.


@pytest.mark.requires("audit")
def test_switch_is_audited(session_group):
    """Controller changes are recorded

    Categories: manipulation,safety
    What it checks: Opens/closes a trajectory session and checks audit records identify the actual group, role, controller, displaced controllers, and session close.
    Why it matters: Operators need to know which physical controller took over and when it was released.
    If it fails: Check audit configuration and resolved role/controller names, then inspect session audit handling.
    Profile fields: groups.*.controllers;audit
    Scenario fields: groups.*.allow_sessions
    Robot effects: Switches a controller session and reads its audit.
    """
    group = session_group
    with group.control(Controller.TRAJECTORY) as session:
        expected = session.integration['controller']
        displaced = list(session.integration['deactivated'])
    events = group._owner.audit.query(kind=AuditKind.COMMAND, limit=100)
    opens = [e for e in events if e.call == "group.control" and e.detail.get("group") == group.name]
    assert opens
    assert opens[-1].detail["controller"] == expected
    assert opens[-1].detail["role"] == "trajectory"
    assert opens[-1].detail["deactivated"] == displaced
    assert any(e.call == "group.control_stop" and e.detail.get("group") == group.name for e in events)


def test_hold_commands_current_position(session_group, group_recipe):
    """Hold keeps the group at its current joints

    Categories: manipulation
    What it checks: Records current joints, requests hold in a trajectory session, checks returned targets match, then checks settled joints remain within the configured tolerance.
    Why it matters: A hold command must preserve real position rather than choose a default pose.
    If it fails: Check controller hold behavior, joint feedback, group ordering, and independently measured settling/tolerance.
    Profile fields: groups.*.controllers.trajectory;groups.*.joints
    Scenario fields: groups.*.allow_sessions;groups.*.settle;groups.*.tolerance
    Robot effects: Commands a hold at the current joint positions.
    """
    group = session_group
    with group.control(Controller.TRAJECTORY) as session:
        # Controller takeover can change the measured posture. Hold promises
        # the position at invocation, not a sample from before takeover.
        before = group.get_joint_positions()
        held = session.hold()
        assert set(held) == set(group.joint_names)
        assert held == pytest.approx(before, abs=group_recipe.get("tolerance", .01))
        time.sleep(group_recipe.get("settle", .5))
        assert group.get_joint_positions() == pytest.approx(held, abs=group_recipe.get("tolerance", .01))


def test_tool_opens_and_closes(tool_case):
    """Tool physically reaches both declared endpoints

    Categories: manipulation
    What it checks: Closes then opens the declared position-controlled tool, checks measured position at each endpoint and a distinct physical change, then restores its original position.
    Why it matters: Correct metadata is insufficient when a gripper never actually moves or endpoint meanings are reversed.
    If it fails: Check tool joint/controller, measured open/closed values, live limits, direction convention, and settling/tolerance. Do not infer endpoints from another robot's gripper.
    Profile fields: groups.*.tool.joint;groups.*.tool.open;groups.*.tool.closed;groups.*.controllers.tool
    Scenario fields: groups.*.tool.settle;groups.*.tool.tolerance;groups.*.tool.timeout
    Robot effects: Operates the tool and restores its initial position.
    """
    tool, recipe = tool_case
    tolerance = recipe.get("tolerance", .01)
    def await_endpoint(target):
        deadline = time.monotonic() + recipe.get("timeout",15)
        while abs(tool.get_width()-target) > tolerance:
            assert time.monotonic() < deadline, f"Tool did not reach {target} before the configured deadline (measured {tool.get_width()})"
            time.sleep(.05)
    assert abs(tool.open_at-tool.closed_at) > tolerance, "endpoints must be observably distinct"
    tool.set(tool.closed_at, timeout=recipe.get("timeout",30), execution_timeout=recipe.get("timeout",30))
    await_endpoint(tool.closed_at)
    closed = tool.get_width()
    assert closed == pytest.approx(tool.closed_at, abs=tolerance)
    tool.set(tool.open_at, timeout=recipe.get("timeout",30), execution_timeout=recipe.get("timeout",30))
    await_endpoint(tool.open_at)
    opened = tool.get_width()
    assert opened == pytest.approx(tool.open_at, abs=tolerance)
    assert abs(opened-closed) > tolerance, "test requires actual movement between endpoints"


def test_is_holding_distinguishes_known_obstruction(tool_case, request):
    """Tool reports inferred holding when movement stalls

    Categories: manipulation
    What it checks: Opens the tool, requests a bounded target, and verifies that actual movement followed by a stalled position is reported as inferred holding. Uses a reviewed physical obstruction or an explicit simulator actuator-stall fixture; this does not measure grip force.
    Why it matters: Reaching a target and gripping an object are different outcomes.
    If it fails: Provide a known contact setup, check feedback and holding inference, and keep the target inside declared joint limits. No mechanical overtravel is manufactured.
    Profile fields: groups.*.tool;groups.*.controllers.tool
    Scenario fields: groups.*.tool.blocked_target;groups.*.tool.stall_command;groups.*.tool.timeout;groups.*.tool.tolerance
    Robot effects: Operates the tool; an explicitly configured simulator fixture may hold its controller at an intermediate position. Restores the original tool position.
    """
    tool,recipe = tool_case
    target = recipe.get("blocked_target")
    if target is None:
        skip_coverage(request, "not_configured", "no known obstruction scenario; test never drives past a mechanical hard stop")
    import subprocess
    fault = None
    import tempfile
    log = tempfile.TemporaryFile(mode="w+t")
    try:
        tool.open()
        deadline=time.monotonic()+recipe.get("timeout",15)
        while abs(tool.get_width()-tool.open_at)>recipe.get("tolerance",.01):
            assert time.monotonic()<deadline, "Tool did not open before obstruction check"
            time.sleep(.05)
        tool.set(target)
        if recipe.get("stall_command"):
            fault=subprocess.Popen(recipe["stall_command"],start_new_session=True,stdout=log,stderr=subprocess.STDOUT)
            deadline=time.monotonic()+recipe.get("timeout",15)
            while True:
                log.seek(0)
                if "STALL FIXTURE READY" in log.read(): break
                assert fault.poll() is None, "Stall fixture exited before readiness"
                assert time.monotonic()<deadline, "Stall fixture did not report readiness"
                time.sleep(.05)
            request.node.user_properties.append(("fixture", "explicit simulated actuator stall; not a physical grasp"))
        deadline=time.monotonic()+recipe.get("timeout",15)
        while True:
            state=tool.state()
            if state["moved"] is True and state["is_holding"] is True:
                break
            assert time.monotonic()<deadline, f"No inferred stall after movement: {state}"
            time.sleep(.05)
        assert abs(state["position"]-target)>recipe.get("tolerance",.01)
    finally:
        if fault is not None:
            # Stop the remote publisher before restoring tool position;
            # killing only the local docker CLI leaves its ROS child running.
            subprocess.run(recipe["stall_restore_command"], check=True, timeout=5)
            try:
                fault.wait(timeout=5)
            except subprocess.TimeoutExpired:
                import os,signal
                os.killpg(fault.pid,signal.SIGTERM)
                fault.wait(timeout=3)
                pytest.fail("Stall fixture did not stop; tool restoration cannot be trusted")
        log.close()


def test_missing_controller_role_says_what_is_available(session_group, request):
    """Unavailable controller roles give a clear error

    Categories: manipulation,profile
    What it checks: Chooses a role this group does not declare, requests it, and expects a capability/session error naming that role. Groups declaring every role are not applicable.
    Why it matters: An unsupported control mode should be rejected clearly before commands are sent.
    If it fails: Check role mappings for the controllers that actually exist. Do not add a nonexistent controller just to make every role available.
    Profile fields: groups.*.controllers
    Scenario fields: groups.*.allow_sessions
    Robot effects: Requests an undeclared controller role; no trajectory is sent.
    """
    group = session_group
    roles = group._owner.integration.group_controllers(group.name)
    missing = [r for r in Controller if r not in roles]
    if not missing:
        skip_coverage(request, "not_applicable", "group declares every known controller role")
    with pytest.raises((CapabilityNotSupported, RingfourError)) as exc:
        with group.control(missing[0]):
            pass
    assert missing[0].value in str(exc.value).lower()
