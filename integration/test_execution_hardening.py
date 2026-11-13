"""Portable live execution faults, selected through reviewed per-group recipes."""
import json
import os
from pathlib import Path
import queue
import subprocess
import sys
import threading
import time
import uuid

import pytest
from ringfour import JointTrajectory, Robot
from ringfour.errors import (ConnectionLost, OperationCancelled, PermissionDenied,
                             ResourceBusy, RingfourError, SafetyViolation, WaitTimeout)
from ringfour.transport import Connection
from conformance import SDK_DIR, skip_coverage

pytestmark = [pytest.mark.integration, pytest.mark.sim]
TERMINAL = {'succeeded', 'failed', 'cancelled'}


def rpc(robot, method, params=None, timeout=15):
    return robot._runner.run(robot._robot.connection.call(method, params, timeout=timeout))


def planned(group_motion, duration=None):
    group, waypoints, recipe = group_motion
    values = dict(zip(group.joint_names, waypoints[0]))
    return group, JointTrajectory.from_waypoints([group.configuration(values)],
        step_duration=duration or recipe.get('dt', 2.)), recipe, values


def reacquire(group, deadline):
    while True:
        try:
            with group.control():
                return
        except SafetyViolation as exc:
            if exc.reason != 'motion lock held':
                raise  # Quarantine/inhibit is a failure, never a retry to bypass it.
        except ResourceBusy:
            pass
        assert time.monotonic() < deadline, 'revoked owner did not finish cleanup/release'
        time.sleep(.05)


def fault_enabled(request, recipe, name):
    config = recipe.get('hardening', {}).get(name)
    if not config:
        skip_coverage(request, 'not_configured', f'group hardening.{name} needs explicit reviewed eligibility')
    return config


def joint_target(group, values, tolerance):
    home = group.get_joint_positions()
    differences = {n: abs(values[n]-home[n]) for n in group.joint_names}
    name = max(differences, key=differences.get)
    assert differences[name] > 2*tolerance, 'configured first target cannot exercise the fault'
    return name, home


@pytest.mark.motion
@pytest.mark.requires('groups')
def test_operation_wait_and_observer_exit_do_not_cancel(group_motion, record_property):
    """A result wait and a closed observer do not cancel accepted work.

    Categories: manipulation,safety
    What it checks: Expires an observer wait and closes its update iterator while a reviewed trajectory remains active, then explicitly cancels and requires verified stopping.
    Why it matters: A UI or observer lifetime must not silently become execution ownership.
    If it fails: Inspect the execution, authority and measured-cleanup evidence. Do not relax robot tolerances to turn a contract failure into success.
    Profile fields: groups.*.controllers.trajectory
    Scenario fields: groups.*.waypoints;groups.*.probe;groups.*.dt
    Robot effects: Moves one reviewed group and restores its initial measured configuration.
    """
    group, trajectory, recipe, _ = planned(group_motion)
    with group.control() as control:
        with control.start_trajectory(trajectory) as operation:
            with pytest.raises(WaitTimeout):
                operation.result(timeout=min(.02, recipe.get('dt', 2.)/20))
            assert operation.get_status().state.value not in TERMINAL
            observer = operation.updates()
            next(observer)
            observer.close()
            assert operation.get_status().state.value not in TERMINAL
            receipt = operation.cancel()
            stop = operation.close()
            assert stop.verified
            with pytest.raises(OperationCancelled):
                operation.result()
            record_property('cancel_receipt', json.dumps(receipt.model_dump(mode='json')))
            record_property('measured_stop', json.dumps(stop.model_dump(mode='json')))


@pytest.mark.motion
@pytest.mark.requires('groups')
def test_joint_guard_rejects_unsafe_entry_and_latches(group_motion, welcome, request, record_property):
    """A discovered joint guard rejects unsafe entry and stops crossing motion.

    Categories: manipulation,safety
    What it checks: Builds a guard from discovered joint metadata, rejects unsafe entry, crosses a reviewed midpoint, requires stopping and latched inhibition, then removes the guard without resuming old work.
    Why it matters: Persistent interlocks must act independently of the observer and prevent further covered motion.
    If it fails: Inspect the execution, authority and measured-cleanup evidence. Do not relax robot tolerances to turn a contract failure into success.
    Profile fields: groups.*.controllers.trajectory
    Scenario fields: groups.*.waypoints;groups.*.probe;groups.*.tolerance
    Robot effects: Moves a reviewed group, trips and removes its guard, then restores initial joints.
    """
    if not welcome.features.get('persistent_guards'):
        skip_coverage(request, 'not_configured', 'agent does not advertise persistent guards')
    group, trajectory, recipe, values = planned(group_motion)
    robot = group._owner
    name, home = joint_target(group, values, recipe.get('tolerance', .01))
    signal = robot.signals['arms'][group.name]['joint_positions'][name]
    age = welcome.features.get('group_feedback_policy', {}).get(group.name, {}).get('max_age', .5)
    direction = 1 if values[name] > home[name] else -1
    unsafe = signal.above(home[name]-recipe.get('tolerance', .01))
    with pytest.raises(SafetyViolation):
        with robot.guard(unsafe):
            pytest.fail('unsafe guard entered its body')
    threshold = (home[name]+values[name])/2
    condition = signal.above(threshold, max_age=age) if direction > 0 else signal.below(threshold, max_age=age)
    with robot.guard(condition) as guard:
        with group.control() as control:
            operation = control.start_trajectory(trajectory)
            deadline = time.monotonic()+recipe.get('restore_timeout', 30.)
            while not guard.get_status().tripped:
                assert time.monotonic() < deadline, 'guard never observed its configured threshold crossing'
                time.sleep(.025)
            stop = operation.close()
            assert stop is not None and stop.verified
            with pytest.raises(SafetyViolation) as denied:
                with group.control():
                    pass
            assert denied.value.reason == 'guard'
            record_property('guard_status', json.dumps(guard.get_status().model_dump(mode='json')))
    assert operation.get_status().state.value in TERMINAL  # Removal never resumes old work.


def test_healthy_lease_survives_slow_observation(robot, welcome, request, record_property):
    """An ordinary TF wait cannot starve a healthy connection's lease renewals.

    Categories: connection,safety
    What it checks: Waits for a unique nonexistent TF frame longer than the advertised lease, requires an ordinary TF timeout, and verifies the same connection still works.
    Why it matters: An ordinary slow read must not starve automatic authority renewal.
    If it fails: Inspect the execution, authority and measured-cleanup evidence. Do not relax robot tolerances to turn a contract failure into success.
    Profile fields: safety.lease
    Scenario fields: None.
    Robot effects: Read-only TF wait and heartbeat messages; sends no motion.
    """
    lease = robot._robot._welcome.features.get('connection_lease')
    if not lease:
        skip_coverage(request, 'not_configured', 'selected connection has no finite operator lease')
    timeout = lease['lifetime']+2*lease['renew_interval']
    started = time.monotonic()
    from ringfour.errors import TimeoutError
    with pytest.raises(TimeoutError):
        robot.tf.wait_for('__ringfour_absent_'+uuid.uuid4().hex, timeout=timeout)
    assert not robot._robot.connection.closed
    robot.ping()
    record_property('slow_observation_seconds', time.monotonic()-started)


@pytest.mark.motion
@pytest.mark.requires('groups')
def test_group_lease_expiry_stops_a_live_nonrenewing_connection(group_motion, agent_full, request, record_property):
    """A live socket cannot continue motion after its authority lease expires.

    Categories: connection,manipulation,safety
    What it checks: Uses a separate live connection with no renewer, observes a reviewed trajectory moving across lease expiry, requires verified release and refusal to revive the expired token.
    Why it matters: Socket liveness and fresh telemetry are not evidence of valid control authority.
    If it fails: Inspect the execution, authority and measured-cleanup evidence. Do not relax robot tolerances to turn a contract failure into success.
    Profile fields: safety.lease;groups.*.stop_verification
    Scenario fields: groups.*.hardening.lease_expiry;groups.*.waypoints;groups.*.probe;groups.*.restore_timeout
    Robot effects: Moves one reviewed group, deliberately expires its owner, and restores its initial joints.
    """
    group, _, recipe, values = planned(group_motion)
    fault_enabled(request, recipe, 'lease_expiry')
    raw = Connection(agent_full.unix_uri, timeout=15)
    try:
        welcome = raw.call('hello', {'protocol':1, 'client_version':'lease-conformance'})
        lease = welcome['features'].get('connection_lease')
        if not lease:
            skip_coverage(request, 'not_configured', 'agent did not grant a finite operator lease')
        name, home = joint_target(group, values, recipe.get('tolerance', .01))
        duration = max(recipe.get('dt', 2.), lease['lifetime']*1.5)
        _, trajectory, _, _ = planned(group_motion, duration)
        session = raw.call('group.control_start', {'group':group.name, 'role':'trajectory'})
        raw.call('lease.renew', {'lease_id':lease['id'], 'epoch':lease['epoch']})
        started = time.monotonic()
        accepted = raw.call('group.start_trajectory', {'group':group.name, 'session_id':session['session_id'],
                                                      'trajectory':trajectory.model_dump(mode='json')})
        observed = False
        while time.monotonic()-started < lease['lifetime']:
            observed |= abs(group.get_joint_positions()[name]-home[name]) > recipe.get('tolerance', .01)
            time.sleep(.05)
        reacquire(group, started+lease['lifetime']+recipe.get('restore_timeout', 30.))
        assert observed, 'lease fault never exercised measured movement'
        with pytest.raises((ConnectionLost, SafetyViolation)):
            raw.call('lease.renew', {'lease_id':lease['id'], 'epoch':lease['epoch']})
        record_property('expired_lease', json.dumps(lease))
        record_property('accepted_operation', json.dumps(accepted))
    finally:
        raw.close()


@pytest.mark.motion
@pytest.mark.requires('groups')
def test_killed_group_client_stops_and_releases(group_motion, agent_full, request, record_property):
    """Killing a group owner triggers verified stopping and eventual release.

    Categories: connection,manipulation,safety
    What it checks: Starts an independent reviewed trajectory owner, confirms measured movement, kills that process and requires verified stopping/release before another owner acquires.
    Why it matters: Client death must initiate cleanup rather than leave detached group motion.
    If it fails: Inspect the execution, authority and measured-cleanup evidence. Do not relax robot tolerances to turn a contract failure into success.
    Profile fields: safety.stop_verification;groups.*.controllers.trajectory
    Scenario fields: groups.*.hardening.client_death;groups.*.waypoints;groups.*.probe;groups.*.restore_timeout
    Robot effects: Moves one reviewed group and kills only its test driver; restores initial joints.
    """
    group, _, recipe, values = planned(group_motion)
    fault_enabled(request, recipe, 'client_death')
    name, home = joint_target(group, values, recipe.get('tolerance', .01))
    duration = max(recipe.get('dt', 2.)*3, 10.)
    _, trajectory, _, _ = planned(group_motion, duration)
    helper = Path(__file__).parent/'helpers'/'group_until_killed.py'
    env = dict(os.environ, PYTHONPATH=os.pathsep.join([str(SDK_DIR), os.environ.get('PYTHONPATH','')]))
    process = subprocess.Popen([sys.executable, str(helper), agent_full.unix_uri, group.name,
        json.dumps(trajectory.model_dump(mode='json'))], env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    lines = queue.Queue()
    threading.Thread(target=lambda: lines.put(process.stdout.readline()), daemon=True).start()
    try:
        assert lines.get(timeout=recipe.get('restore_timeout',30.)), 'child exited before accepting work'
        deadline = time.monotonic()+recipe.get('restore_timeout',30.)
        while abs(group.get_joint_positions()[name]-home[name]) <= recipe.get('tolerance',.01):
            assert process.poll() is None, 'child exited before measured movement'
            assert time.monotonic()<deadline, 'child never produced measured movement'
            time.sleep(.05)
        killed = time.monotonic()
        process.kill(); process.wait(timeout=5)
        reacquire(group, killed+recipe.get('restore_timeout',30.))
        record_property('kill_to_verified_release_seconds', time.monotonic()-killed)
    finally:
        if process.poll() is None:
            process.kill(); process.wait(timeout=5)
        process.stdout.close(); process.stderr.close()


@pytest.mark.motion
@pytest.mark.requires('groups')
def test_feedback_loss_requires_quarantine_and_local_recovery(group_motion, debug_enabled, request, record_property):
    """Lost feedback fails cleanup and cannot be erased by reconnect or reset.

    Categories: manipulation,safety
    What it checks: Runs reviewed feedback fault and restoration commands, verifies failed cleanup and retained quarantine, refuses reset, executes the explicit local recovery command and preserves the failed operation outcome.
    Why it matters: Restored feedback or reconnect must not erase a failed stopping contract.
    If it fails: Inspect the execution, authority and measured-cleanup evidence. Do not relax robot tolerances to turn a contract failure into success.
    Profile fields: safety.stop_verification;groups.*.stop_verification
    Scenario fields: groups.*.hardening.feedback_loss;debug;groups.*.waypoints;groups.*.probe
    Robot effects: Moves one reviewed group, temporarily disables configured feedback, latches inhibition and performs explicitly configured local recovery; restores feedback and initial joints.
    """
    group, trajectory, recipe, values = planned(group_motion)
    config = fault_enabled(request, recipe, 'feedback_loss')
    if config.get('reviewed') is not True:
        skip_coverage(request,'not_configured','feedback-loss commands require explicit review')
    robot = group._owner
    name, home = joint_target(group, values, recipe.get('tolerance',.01))
    timeout = config.get('timeout', 30.)
    attempted = False
    primary = None
    operation = None
    try:
        with group.control() as control:
            operation = control.start_trajectory(trajectory)
            deadline = time.monotonic()+timeout
            while abs(group.get_joint_positions()[name]-home[name]) <= recipe.get('tolerance',.01):
                assert time.monotonic()<deadline, 'feedback fault never exercised measured motion'
                time.sleep(.025)
            attempted = True  # Restore even if the command applies an effect then loses its reply.
            subprocess.run(config['command'], check=True, capture_output=True, text=True, timeout=timeout)
            age = robot._robot._welcome.features.get('group_feedback_policy',{}).get(group.name,{}).get('max_age',.5)
            time.sleep(age+.1)
            assert group.get_joint_measurement(require_valid=False).validity != 'valid'
            operation.cancel()
            operation.close()
            pytest.fail('missing feedback fabricated verified cleanup')
    except RingfourError as exc:
        primary = exc
    finally:
        if attempted:
            subprocess.run(config['restore_command'], check=True, capture_output=True, text=True, timeout=timeout)
    assert primary is not None and operation is not None
    status = rpc(robot,'task.status',{'task_id':operation.id})
    record_property('feedback_loss_terminal', json.dumps(status))
    record_property('feedback_loss_error', str(primary))
    assert status['state']=='failed', {'terminal': status, 'cleanup_error': str(primary)}
    with pytest.raises(SafetyViolation) as denied:
        with group.control():
            pass
    assert denied.value.reason=='quarantine'
    try:
        rpc(robot,'debug.set_estop',{'engaged':True})
    except RingfourError:
        pass  # The inhibit latches even when retained quarantine makes halt report failure.
    assert robot.get_estop_engaged()
    with pytest.raises(SafetyViolation):
        rpc(robot,'debug.set_estop',{'engaged':False})
    command = [value.replace('{resource}',group.name) for value in config['recovery_command']]
    completed = subprocess.run(command,check=True,capture_output=True,text=True,timeout=timeout)
    recovery = json.loads(completed.stdout)
    assert recovery['recovered'] and recovery['inhibit_engaged'] and recovery['stop_verification']['verified']
    assert robot.get_estop_engaged()
    assert rpc(robot,'task.status',{'task_id':operation.id})['state']=='failed'
    rpc(robot,'debug.set_estop',{'engaged':False})
    record_property('retained_failed_operation',json.dumps(status))
    record_property('local_recovery',json.dumps(recovery))


def test_unverified_network_identity_cannot_recover(agent_full, welcome, request):
    """An unverified network principal cannot claim quarantine recovery authority.

    Categories: connection,safety
    What it checks: Inspects an explicitly supplied network peer and verifies an unverified principal lacks recovery permission and receives PermissionDenied before recovery dispatch.
    Why it matters: Operator network access must not imply authenticated quarantine recovery authority.
    If it fails: Inspect the execution, authority and measured-cleanup evidence. Do not relax robot tolerances to turn a contract failure into success.
    Profile fields: None.
    Scenario fields: connection.tcp_uri
    Robot effects: Reads permissions and submits an unauthorized recovery request; performs no recovery or motion.
    """
    if not agent_full.tcp_uri.startswith(('ws://','wss://')):
        skip_coverage(request,'not_configured','an explicit TCP endpoint is required')
    with Robot(agent_full.tcp_uri) as network:
        access = network._robot._welcome.features.get('access')
        if access is None:
            skip_coverage(request,'not_configured','agent does not report connection permissions')
        if access['authenticated']:
            skip_coverage(request,'not_applicable','selected network connection has authenticated identity')
        assert 'recover' not in access['permissions']
        resource = 'mobility' if network.has('mobility') else next(iter(network.groups),'unavailable') if network.has('groups') else 'unavailable'
        with pytest.raises(PermissionDenied):
            rpc(network,'safety.recover',{'resource':resource})


@pytest.mark.motion
@pytest.mark.requires('groups')
def test_lost_motion_reply_reconciles_without_duplicate_dispatch(group_motion, monkeypatch, request, record_property):
    """A lost motion reply reconciles to the original accepted operation.

    Categories: manipulation,connection,safety
    What it checks: Drops only one trajectory acceptance reply in the client, observes SubmissionUnknown, queries the original command, retries it twice and rejects a conflicting payload; also drops terminal notifications and queries the authoritative result.
    Why it matters: Lost replies and lost events must never cause duplicate motion or fabricated completion.
    If it fails: Inspect the command ledger, operation identity and adapter audit; never retry with a fresh ID.
    Profile fields: safety.lease;groups.*.controllers.trajectory
    Scenario fields: groups.*.waypoints;groups.*.probe;groups.*.dt
    Robot effects: Executes one reviewed group trajectory and restores its initial posture; injects client-side message loss only.
    """
    import asyncio
    from ringfour import SubmissionUnknown, CommandConflict
    group, trajectory, recipe, _ = planned(group_motion)
    robot = group._owner
    if not robot._robot._welcome.features.get('commands'):
        skip_coverage(request,'not_configured','agent must advertise command reconciliation')
    connection = robot._robot.connection
    original = connection._handle_text
    dropped = []
    def receive(text):
        message = json.loads(text)
        if 'result' in message and isinstance(message['result'],dict) and 'task_id' in message['result'] and not dropped:
            dropped.append(message)
            return
        if message.get('method') == 'task.update' and message.get('params',{}).get('state') in TERMINAL:
            return  # Terminal evidence must remain queryable without notifications.
        original(text)
    with group.control() as control:
        params = {'group':group.name, 'session_id':control._aio._session_id,
                  'trajectory':trajectory.model_dump(mode='json')}
        key = robot.commands.new()
        with monkeypatch.context() as patch:
            patch.setattr(connection, '_handle_text', receive)
            with pytest.raises(SubmissionUnknown) as unknown:
                robot.commands.retry(key,'group.start_trajectory',params,timeout=.15)
            assert unknown.value.details['command_id']==key and dropped
            accepted = robot.commands.get(key)
            assert accepted.operation_id == dropped[0]['result']['task_id']
            async def repeat():
                return await asyncio.gather(*[robot._robot.commands.retry(key,'group.start_trajectory',params) for _ in range(2)])
            replies = robot._runner.run(repeat())
            assert replies == [accepted.result, accepted.result]
            changed = {**params, 'trajectory': {**params['trajectory'], 'times': [trajectory.times[0]+1]}}
            with pytest.raises(CommandConflict):robot.commands.retry(key,'group.start_trajectory',changed)
            deadline = time.monotonic()+recipe.get('restore_timeout',30)
            while True:
                result = robot.operations.get(accepted.operation_id)
                if result.state=='terminal':break
                assert time.monotonic()<deadline,'operation never became terminal'
                time.sleep(.025)
            assert result.snapshot.state=='succeeded'
            assert result.snapshot.result['stop_verification']['verified']
            audit = rpc(robot,'audit.query',{'task_id':accepted.operation_id,'limit':100})['events']
            assert sum(event['detail'].get('event')=='started' for event in audit)==1
            record_property('command_record',accepted.model_dump_json())
            record_property('queried_terminal',result.model_dump_json())


@pytest.mark.motion
def test_agent_restart_retains_results_and_rejects_old_epoch(agent_full, scenarios, motion_enabled, request, record_property):
    """Restart retains terminal evidence and rejects old-epoch execution.

    Categories: connection,safety
    What it checks: Runs an explicitly reviewed local identity probe, gracefully restarts only the agent, then verifies durable terminal evidence and old-epoch retry refusal from a new authenticated connection.
    Why it matters: Restart must preserve recorded outcomes without replaying accepted work.
    If it fails: Inspect the state volume, principal identity, shutdown marker and command epoch; never replace the safety volume to obtain a pass.
    Profile fields: safety.lease
    Scenario fields: lifecycle.reviewed;lifecycle.probe_command;lifecycle.restart_command;lifecycle.timeout;debug
    Robot effects: Completes a synthetic non-motion task and restarts the selected agent through reviewed argv; does not restart the robot simulator.
    """
    config = scenarios.get('lifecycle')
    if not config or config.get('reviewed') is not True:
        skip_coverage(request,'not_configured','reviewed authenticated probe and agent restart argv are required')
    timeout = config.get('timeout',90)
    prepared = subprocess.run([*config['probe_command'],'prepare'],check=True,capture_output=True,text=True,timeout=timeout)
    prior = json.loads(prepared.stdout)
    subprocess.run(config['restart_command'],check=True,capture_output=True,text=True,timeout=timeout)
    deadline = time.monotonic()+timeout
    while True:
        checked = subprocess.run([*config['probe_command'],'verify'],input=json.dumps(prior),capture_output=True,text=True,timeout=timeout)
        if checked.returncode==0:break
        # Only startup connection failure is retryable. A semantic failure must be retained.
        if 'ConnectionFailed' not in checked.stderr or time.monotonic()>=deadline:
            raise AssertionError(checked.stderr)
        time.sleep(.1)
    outcome = json.loads(checked.stdout)
    assert outcome['terminal_retained'] and outcome['old_epoch_rejected']
    record_property('restart_terminal_probe',json.dumps(outcome))


@pytest.mark.motion
@pytest.mark.requires('teleop','mobility')
def test_killed_mobility_client_verifies_stop_and_releases(robot, agent_full, scenarios, motion_enabled, request, record_property):
    """Killed drive client releases ownership only after verified body rest.

    Categories: movement,connection,safety
    What it checks: Uses reviewed input isolation and a six-axis observed command, kills the driver, waits for verified ownership release and inspects the agent's measured stop report.
    Why it matters: A killed client must not leave motion active or make missing stopping evidence look successful.
    If it fails: Preserve quarantine and examine the actual stop-verification residuals; do not increase thresholds to hide the failure.
    Profile fields: safety.stop_verification;safety.lease;teleop.max_watchdog
    Scenario fields: mobility_fault.reviewed;mobility_fault.command;mobility_fault.setup_command;mobility_fault.restore_command;mobility_fault.timeout;mobility_fault.watchdog
    Robot effects: Drives the reviewed body command, kills its client, verifies stop and restores the input selector; does not automatically recover a failed stop.
    """
    config = scenarios.get('mobility_fault')
    if not config or config.get('reviewed') is not True:
        skip_coverage(request,'not_configured','reviewed mobility fault and input isolation fixture required')
    policy = robot._robot._welcome.features.get('body_stop_policy')
    if not policy:
        skip_coverage(request,'not_configured','explicit measured body-stop policy required')
    timeout = config.get('timeout',10)
    process = None
    attempted = False
    def moving():
        value = robot.get_velocity()
        linear = max(abs(getattr(value.linear,axis)) for axis in 'xyz')
        angular = max(abs(getattr(value.angular,axis)) for axis in 'xyz')
        return linear > policy['linear_velocity_limit'] or angular > policy['angular_velocity_limit']
    try:
        attempted=True
        subprocess.run(config['setup_command'],check=True,capture_output=True,text=True,timeout=timeout)
        helper=Path(__file__).parent/'helpers'/'drive_forever.py'
        process=subprocess.Popen([sys.executable,str(helper),agent_full.unix_uri,str(config['watchdog']),json.dumps(config['command'])],
            env=dict(os.environ,PYTHONPATH=str(SDK_DIR)),stdout=subprocess.DEVNULL,stderr=subprocess.PIPE,text=True)
        deadline=time.monotonic()+timeout
        while not moving():
            assert process.poll() is None,'drive helper exited before observed motion'
            assert time.monotonic()<deadline,'reviewed command produced no measured motion'
            time.sleep(.025)
        killed=time.monotonic();process.kill();process.wait(timeout=5)
        deadline=killed+timeout
        while True:
            try:
                rpc(robot,'teleop.start',{'watchdog':config['watchdog']})
                report=rpc(robot,'teleop.end')['stop_verification']
                break
            except (ResourceBusy,SafetyViolation) as exc:
                if isinstance(exc,SafetyViolation) and exc.reason!='motion lock held':raise
                assert time.monotonic()<deadline,'verified ownership release exceeded fixture deadline'
                time.sleep(.025)
        assert report['verified'] and not moving()
        record_property('kill_to_verified_release_seconds',time.monotonic()-killed)
        record_property('measured_body_stop',json.dumps(report))
    finally:
        if process is not None:
            if process.poll() is None:process.kill()
            process.wait(timeout=5)
            process.stderr.close()
        try:robot.stop()
        finally:
            if attempted:subprocess.run(config['restore_command'],check=True,capture_output=True,text=True,timeout=timeout)
