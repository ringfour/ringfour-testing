"""A02/A03/A09 deterministic access, resource and measured-stop evidence."""
import asyncio
import json
import os
from dataclasses import replace
from types import SimpleNamespace

import pytest
from .conftest import command_id
from ringfour_agent.access import AccessPolicy
from ringfour_agent.bootstrap import build_agent
from ringfour_agent.models.profile import JointStopPolicy
from ringfour_agent.profile import load_profile
from ringfour_agent.safety import MotionLock
from ringfour_agent.server import RpcError
from ringfour_agent.stopping import verify_joint_stop
from .conftest import STUB_PROFILE, _free_tcp_port
from .test_hardening_contract import environment

pytestmark = pytest.mark.contract


class Clock:
    now = 0.
    def read(self): return self.now
    def sleep(self, seconds): self.now += seconds


class StopAdapter:
    def __init__(self, clock, fault=None): self.clock, self.fault = clock, fault
    def joint_type(self, name):
        if self.fault == 'unknown-units': return None
        return 'prismatic' if name == 'slide' else 'revolute'
    def stop_snapshot(self, names):
        now = self.clock.now
        source = 0. if self.fault in ('frozen', 'paused-source') else now
        received = 0. if self.fault == 'frozen' else now
        result = {'positions': {n: now if self.fault == 'zero-velocity-moving-position' else 0. for n in names},
                  'velocities': {n: .2 if self.fault == 'moving' else 0. for n in names},
                  'source_stamps': {n: source for n in names}, 'received': {n: received for n in names},
                  'source_now': source if self.fault == 'paused-source' else now, 'receive_now': now}
        if self.fault == 'missing': result['positions'].pop(names[0])
        if self.fault == 'stale': result['source_stamps'][names[0]] = now - 2
        if self.fault == 'future': result['source_stamps'][names[0]] = now + 2
        if self.fault == 'nonfinite-position': result['positions'][names[0]] = float('nan')
        if self.fault == 'nonfinite-velocity': result['velocities'][names[0]] = float('nan')
        return result


POLICY = JointStopPolicy(angular_velocity_limit=.01, linear_velocity_limit=.001, dwell=.2, timeout=.8)


@pytest.mark.parametrize('fault', ['missing', 'stale', 'future', 'nonfinite-position', 'nonfinite-velocity',
                                  'frozen', 'paused-source', 'moving', 'zero-velocity-moving-position', 'unknown-units'])
def test_stop_requires_progressive_complete_stationary_evidence(fault):
    """A09: incomplete evidence never verifies a stop

    Categories: sdk
    What it checks: Injects missing, stale, future, nonfinite, frozen or moving joint feedback under a deterministic clock and explicit angular/linear criteria.
    Why it matters: Controller cancellation and cached zero velocities do not prove measured rest.
    If it fails: Repair shared stop verification and provenance checks.
    Profile fields: groups.*.stop_verification
    Scenario fields: None.
    Robot effects: Offline cache adapter and fake clock only.
    """
    clock = Clock()
    with pytest.raises(RpcError) as error:
        verify_joint_stop(StopAdapter(clock, fault), ['hinge', 'slide'], POLICY, .5, clock=clock.read, sleep=clock.sleep)
    assert error.value.error_type == 'StopUnverified'
    assert clock.now <= POLICY.timeout


def test_stop_verification_keeps_units_separate():
    """A09: mixed joints use explicit native velocity units

    Categories: sdk
    What it checks: Verifies progressive stationary feedback for revolute and prismatic joints and inspects separate thresholds, units, dwell and sample evidence.
    Why it matters: A radian threshold cannot silently become a metre threshold.
    If it fails: Repair unit resolution and measured stop reporting.
    Profile fields: groups.*.stop_verification
    Scenario fields: None.
    Robot effects: Offline cache adapter and fake clock only.
    """
    clock = Clock()
    report = verify_joint_stop(StopAdapter(clock), ['hinge', 'slide'], POLICY, .5, clock=clock.read, sleep=clock.sleep)
    assert report['verified'] and report['samples'] >= 2 and report['elapsed'] >= POLICY.dwell
    assert report['units'] == {'hinge': 'rad/s', 'slide': 'm/s'}
    assert report['velocity_limits'] == {'hinge': .01, 'slide': .001}
    assert report['physical_stop'] == 'uncommissioned'


@pytest.mark.parametrize('first,second', [
    ({'resources': ('adapter:mobility',)}, {'resources': ('adapter:mobility',)}),
    ({'resources': ('joint:hinge',)}, {'resources': ('joint:hinge',)}),
    ({'exclusive': True}, {'resources': ('joint:hinge',)}),
    ({'resources': ('joint:hinge',)}, {'exclusive': True}),
])
def test_conflicting_acquisitions_preserve_original_owner(first, second):
    """A03: shared and opaque footprints cannot overlap

    Categories: sdk
    What it checks: Attempts conflicting mobility, joint and opaque vendor acquisitions by one owner; rejection leaves its first child valid, while later acquisition succeeds after release.
    Why it matters: Same-client commands must not silently preempt physical resources.
    If it fails: Repair atomic acquisition resource accounting.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline lock accounting only.
    """
    lock = MotionLock()
    assert lock.acquire(1, 'first', **first)
    with pytest.raises(RpcError) as error: lock.acquire(1, 'second', **second)
    assert error.value.error_type == 'ResourceBusy'
    assert lock._acquisitions == {'first'} and lock.owner == 1
    lock.release(1, 'first')
    assert lock.acquire(1, 'second', **second)
    lock.release_all(1)
    assert not lock._resources and not lock._exclusive


@pytest.mark.protocol
@pytest.mark.parametrize('transport,trusted,method,params,allowed', [
    ('tcp', False, 'teleop.start', {}, False),
    ('tcp', False, 'ping', {}, True),
    ('tcp', True, 'teleop.start', {}, True),
    ('tcp', True, 'safety.recover', {'resource': 'x'}, False),
    ('unix', False, 'safety.recover', {'resource': 'x'}, True),
    ('unix-other', False, 'safety.recover', {'resource': 'x'}, False),
    ('tcp', False, 'debug.set_estop', {'engaged': False}, False),
])
def test_connection_permissions_are_enforced_over_protocol(tmp_path, transport, trusted, method, params, allowed):
    """A02/A09: transport policy cannot be bypassed with RPC parameters

    Categories: sdk
    What it checks: Exercises real TCP and Unix listeners under read-only, explicitly trusted and kernel-peer policies; records any denied handler dispatch.
    Why it matters: Permission labels must reject unauthorized motion and recovery before adapter work.
    If it fails: Repair connection identity, permission resolution and admission.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Isolated sockets and recording handler only.
    """
    from websockets.asyncio.client import connect, unix_connect
    async def exercise():
        path, port = str(tmp_path / 'agent.sock'), _free_tcp_port()
        policy = AccessPolicy(local_uid=os.getuid() + (1 if transport == 'unix-other' else 0), trusted_tcp=trusted)
        server, ctx = build_agent(load_profile(STUB_PROFILE), unix_path=path, host='127.0.0.1', port=port,
                                  audit_dir=tmp_path, access_policy=policy)
        calls = []
        async def recording(session, params):
            calls.append(session.principal)
            return {}
        server._registry[method] = recording
        await server.start()
        try:
            connector = connect(f'ws://127.0.0.1:{port}') if transport == 'tcp' else unix_connect(path)
            async with connector as socket:
                await socket.send(json.dumps({'command_id': command_id(ctx), 'jsonrpc': '2.0', 'id': 0, 'method': 'hello',
                                              'params': {'protocol': 1, 'client_version': 'test'}}))
                welcome = json.loads(await socket.recv())['result']
                access = welcome['features']['access']
                assert access['authenticated'] == (transport != 'tcp')
                if transport != 'tcp': assert access['principal'] == f'unix:uid:{os.getuid()}'
                await socket.send(json.dumps({'command_id': command_id(ctx), 'jsonrpc': '2.0', 'id': 1, 'method': method, 'params': params}))
                response = json.loads(await socket.recv())
                if allowed:
                    assert 'result' in response and len(calls) == 1
                else:
                    assert response['error']['data']['type'] == 'PermissionDenied' and calls == []
        finally:
            await server.close()
            ctx.audit.close()
    asyncio.run(exercise())


def prepared_recovery(environment, monkeypatch, fault=None):
    manager, lock, state, adapter, audit = environment
    clock = Clock()
    cache = StopAdapter(clock, fault)
    adapter.stop_snapshot = cache.stop_snapshot
    adapter.joint_type = cache.joint_type
    manager._groups['x'] = replace(manager._groups['x'], config=manager._groups['x'].config.model_copy(update={'stop_verification': POLICY}))
    from ringfour_agent import stopping
    original = stopping.verify_joint_stop
    monkeypatch.setattr(stopping, 'verify_joint_stop', lambda *args: original(*args, clock=clock.read, sleep=clock.sleep))
    state.estop_engaged = True
    lock.quarantine['x'] = {'expected_controllers': {'x_idle': 'active', 'x_trajectory': None}}
    return manager, lock, state, adapter, cache


@pytest.mark.parametrize('failure', ['moving', 'stale', 'controllers', 'not-inhibited', 'active-owner', 'missing-policy', 'cancel'])
def test_recovery_never_clears_unverified_quarantine(environment, monkeypatch, failure):
    """A09: recovery cannot erase missing safety evidence

    Categories: sdk
    What it checks: Attempts explicit recovery under moving/stale feedback, wrong controller states, missing policy, active acquisitions, cancellation rejection and released inhibit.
    Why it matters: Reconnect or a reset request cannot substitute for reverified state.
    If it fails: Repair recovery prerequisites and retained quarantine.
    Profile fields: groups.*.stop_verification
    Scenario fields: None.
    Robot effects: Offline recording adapter and fake clock only.
    """
    manager, lock, state, adapter, cache = prepared_recovery(environment, monkeypatch,
                                                           failure if failure in ('moving', 'stale') else None)
    if failure == 'controllers': adapter.states['x_idle']['state'] = 'inactive'
    if failure == 'not-inhibited': state.estop_engaged = False
    if failure == 'active-owner':
        # Represents an earlier child retained while another child quarantined.
        lock.owner = 1; lock._acquisitions.add('earlier')
    if failure == 'missing-policy':
        manager._groups['x'] = replace(manager._groups['x'], config=manager._groups['x'].config.model_copy(update={'stop_verification': None}))
    if failure == 'cancel': adapter.fail = 'cancel'
    with pytest.raises(RpcError): manager.recover('x')
    assert 'x' in lock.quarantine
    adapter.fail = None
    lock.release_all(1)


def test_recovery_clears_only_verified_resource_and_keeps_inhibit(environment, monkeypatch):
    """A09: authorized recovery preserves other quarantine and inhibit

    Categories: sdk
    What it checks: Rechecks cancellation, controller checkpoint and progressive measured rest, clears exactly one resource and keeps the inhibit engaged.
    Why it matters: Recovery must not silently enable motion or erase unrelated faults.
    If it fails: Repair recovery verification and resource-specific clearing.
    Profile fields: groups.*.stop_verification
    Scenario fields: None.
    Robot effects: Offline recording adapter and fake clock only.
    """
    manager, lock, state, adapter, cache = prepared_recovery(environment, monkeypatch)
    lock.quarantine['other'] = {'reason': 'independent failure'}
    report = manager.recover('x')
    assert report['recovered'] and report['stop_verification']['verified']
    assert state.estop_engaged and lock.quarantine == {'other': {'reason': 'independent failure'}}
    assert adapter.calls.count(('cancel',)) == 1
    with pytest.raises(RpcError): lock.acquire(2, 'later')


@pytest.mark.parametrize('fault', [None, 'moving'])
def test_configured_cleanup_verifies_rest_and_quarantines_failure(environment, monkeypatch, fault):
    """A09: configured scope cleanup requires measured rest

    Categories: sdk
    What it checks: Closes a group with a configured stop criterion under stationary and moving feedback; controller restoration alone cannot conceal failed stopping.
    Why it matters: Verified cleanup must include the configured measured criterion.
    If it fails: Repair control teardown and stop evidence propagation.
    Profile fields: groups.*.stop_verification
    Scenario fields: None.
    Robot effects: Offline recording adapter and fake clock only.
    """
    from ringfour import Controller
    manager, lock, state, adapter, cache = prepared_recovery(environment, monkeypatch, fault)
    lock.quarantine.clear(); state.estop_engaged = False
    current = manager.start(1, 'x', Controller.TRAJECTORY)
    if fault:
        with pytest.raises(RpcError) as failure: manager.end('x')
        report = failure.value.extra['cleanup']
        assert not report['confirmed'] and 'x' in lock.quarantine
        assert any(s['step'] == 'verify measured joint stop' and s['status'] == 'failed' for s in report['steps'])
        cache.fault = None; state.estop_engaged = True
        assert manager.recover('x')['recovered']
        # Explicit recovery does not rewrite an earlier failed cleanup result.
        with pytest.raises(RpcError, match='previous cleanup failed'):
            manager.end('x', client_id=1, session_id=current.id)
    else:
        report = manager.end('x')
        assert report['confirmed'] and report['stop_verification']['verified']
    assert lock.owner is None and adapter.states['x_idle']['state'] == 'active'


def test_raw_controller_footprint_cannot_bypass_group_ownership(environment):
    """A02/A04: raw controller names still require matching resources

    Categories: sdk
    What it checks: Rejects a raw controller claiming another group's joint and restores its attempted controller changes.
    Why it matters: Raw naming must not bypass ownership and physical resource admission.
    If it fails: Repair raw controller footprint checks and rollback.
    Profile fields: groups.*.controllers
    Scenario fields: None.
    Robot effects: Offline recording adapter only.
    """
    manager, lock, state, adapter, audit = environment
    before = {k: dict(v) for k, v in adapter.states.items()}
    # Recording adapter assigns a slide joint to this non-x controller.
    with pytest.raises(RpcError, match='claims do not match'):
        manager.start(1, 'x', None, 'opaque_controller')
    assert adapter.states == before and lock.owner is None


@pytest.mark.parametrize('method,params', [('slam.start', {}), ('slam.stop', {}),
    ('slam.load', {'name': 'map'}), ('slam.localize', {}), ('slam.relocalize', {})])
def test_localization_configuration_cannot_preempt_mobility(tmp_path, method, params):
    """A02/A03: configuration writes respect mobility dependencies

    Categories: sdk
    What it checks: Holds a mobility acquisition, attempts map/localization lifecycle changes by the same owner and verifies zero handler dispatch and preserved ownership.
    Why it matters: Configuration can invalidate running navigation even without submitting motion itself.
    If it fails: Repair descriptor dependencies and transient configuration reservations.
    Profile fields: slam
    Scenario fields: None.
    Robot effects: Offline dispatcher and recording handler only.
    """
    from ringfour_agent.server import Session
    async def exercise():
        server, ctx = build_agent(load_profile(STUB_PROFILE), unix_path=str(tmp_path / 'agent.sock'), audit_dir=tmp_path)
        owner = Session(None); owner.handshaken = True
        ctx.lock.acquire(owner.client_id, 'navigation', resources=('adapter:mobility',))
        calls = []
        async def handler(session, params): calls.append(params); return {}
        server._registry[method] = handler
        response = json.loads(await server._dispatch(owner, json.dumps({'command_id': command_id(ctx), 'jsonrpc': '2.0', 'id': 1, 'method': method, 'params': params})))
        assert response['error']['data']['type'] == 'ResourceBusy'
        assert calls == [] and ctx.lock._acquisitions == {'navigation'}
        ctx.lock.release_all(owner.client_id)
        response = json.loads(await server._dispatch(owner, json.dumps({'command_id': command_id(ctx), 'jsonrpc': '2.0', 'id': 2, 'method': method, 'params': params})))
        assert 'result' in response and len(calls) == 1
        assert ctx.lock.owner is None and not ctx.lock._resources
        ctx.audit.close()
    asyncio.run(exercise())


def test_restoration_cannot_hide_new_measured_motion(environment, monkeypatch):
    """A09: restoration is followed by a second measured stop check

    Categories: sdk
    What it checks: Supplies stationary feedback before restoration, then makes the restored controller move; verifies cleanup remains failed and quarantined.
    Why it matters: Restoring an old controller can apply an old target after cancellation.
    If it fails: Repair final teardown postcondition checks.
    Profile fields: groups.*.stop_verification
    Scenario fields: None.
    Robot effects: Offline recording adapter and fake clock only.
    """
    from ringfour import Controller
    manager, lock, state, adapter, cache = prepared_recovery(environment, monkeypatch)
    lock.quarantine.clear(); state.estop_engaged = False
    manager.start(1, 'x', Controller.TRAJECTORY)
    original = adapter.switch_controllers
    def switch(activate, deactivate):
        original(activate, deactivate)
        if 'x_idle' in activate: cache.fault = 'moving'
    adapter.switch_controllers = switch
    with pytest.raises(RpcError) as error: manager.end('x')
    report = error.value.extra['cleanup']
    assert report['pre_restore_stop_verification']['verified']
    assert not report['confirmed'] and 'x' in lock.quarantine
    assert any(s['step'] == 'verify restored measured joint stop' and s['status'] == 'failed' for s in report['steps'])


@pytest.mark.protocol
def test_cancellation_with_unverified_measured_stop_is_failed_over_protocol(environment, monkeypatch, tmp_path):
    """A06/A09: cancellation request cannot fabricate verified cancellation

    Categories: sdk
    What it checks: Cancels an accepted trajectory over a real Unix connection, acknowledges controller cancellation but supplies moving joint feedback, then queries a retained failed StopUnverified result.
    Why it matters: A cancellation acknowledgement alone does not satisfy a configured stop criterion.
    If it fails: Repair task cancellation verification and retained terminal evidence.
    Profile fields: groups.*.stop_verification
    Scenario fields: None.
    Robot effects: Isolated protocol and recording adapter with fake clock only.
    """
    import threading
    from ringfour import aio
    from ringfour_agent.handlers import build_registry
    async def exercise():
        manager, lock, state, adapter, cache = prepared_recovery(environment, monkeypatch, 'moving')
        lock.quarantine.clear(); state.estop_engaged = False
        server, ctx = build_agent(load_profile(STUB_PROFILE), unix_path=str(tmp_path / 'cancel.sock'), audit_dir=tmp_path / 'wire')
        ctx.control, ctx.groups, ctx.lock, ctx.state, ctx.safety, ctx.ros = manager, manager._groups, lock, state, manager._safety, adapter
        adapter.joint_snapshot = lambda: {'hinge': (0., 0., 0.), 'slide': (0., 0., 0.)}
        adapter.joint_limits = lambda: {'hinge': (-1., 1.), 'slide': (0., 1.)}
        adapter.description_sha=lambda:'fixture-revision'
        dispatched = threading.Event()
        adapter.follow_trajectory = lambda *args: dispatched.set()
        adapter.traj_wait = lambda timeout: None
        adapter.traj_feedback = lambda: {}
        server._registry = build_registry(ctx)
        await server.start()
        try:
            robot = await aio.Robot.connect('unix://' + str(tmp_path / 'cancel.sock'))
            try:
                scope = await robot.connection.call('group.control_start', {'group': 'x', 'role': 'trajectory'})
                task = await robot.connection.call('group.start_trajectory', {'group':'x','session_id':scope['session_id'],'trajectory':{'configurations':[{'robot':robot.name,'group':'x','model_revision':'fixture-revision','joints':[{'name':'hinge','position':.1,'unit':'rad','lower':-1.,'upper':1.}]}],'times':[.2]}})
                assert await asyncio.to_thread(dispatched.wait, 2)
                await robot.connection.call('task.cancel', {'task_id': task['task_id']})
                await ctx.tasks.drain_resource(scope['session_id'])
                result = await robot.connection.call('task.status', {'task_id': task['task_id']})
                assert result['state'] == 'failed' and result['error']['type'] == 'StopUnverified'
                assert 'x' in lock.quarantine
            finally:
                cache.fault = None
                await robot.close()
        finally:
            await server.close()
            ctx.audit.close()
    asyncio.run(exercise())


@pytest.mark.protocol
def test_local_recovery_rpc_reverifies_adapter_and_keeps_inhibit(environment, monkeypatch, tmp_path):
    """A09: actual authorized recovery works through the protocol

    Categories: sdk
    What it checks: Uses a real Unix principal and production recovery handler to reverify a synthetic controller checkpoint and measured rest; verifies inhibit remains engaged.
    Why it matters: Recovery permission and state verification must compose on the actual wire path.
    If it fails: Repair recovery handler wiring, access policy or verification.
    Profile fields: groups.*.stop_verification
    Scenario fields: None.
    Robot effects: Isolated Unix protocol and recording adapter with fake clock only.
    """
    from ringfour import aio
    from ringfour_agent.handlers import build_registry
    async def exercise():
        manager, lock, state, adapter, cache = prepared_recovery(environment, monkeypatch)
        server, ctx = build_agent(load_profile(STUB_PROFILE), unix_path=str(tmp_path / 'recover.sock'), audit_dir=tmp_path / 'wire')
        ctx.control, ctx.groups, ctx.lock, ctx.state, ctx.safety, ctx.ros = manager, manager._groups, lock, state, manager._safety, adapter
        server._registry = build_registry(ctx)
        await server.start()
        try:
            robot = await aio.Robot.connect('unix://' + str(tmp_path / 'recover.sock'))
            try:
                result = await robot.connection.call('safety.recover', {'resource': 'x'})
                assert result['recovered'] and result['stop_verification']['verified']
                assert state.estop_engaged and not lock.quarantine
                events = ctx.audit.query()
                assert any(e.call == 'safety.recover' and e.detail.get('recovery', {}).get('recovered') for e in events)
            finally: await robot.close()
        finally:
            await server.close()
            ctx.audit.close()
    asyncio.run(exercise())


@pytest.mark.parametrize('moving', [False, True])
def test_tool_recovery_verifies_tool_joint_and_preserves_group_quarantine(environment, monkeypatch, moving):
    """Tool recovery checks its own measured resource

    Categories: sdk,safety
    What it checks: Rechecks the tool controller and only the tool joint under stationary/moving feedback; successful tool recovery preserves arm quarantine and the inhibit.
    Why it matters: A stationary arm does not prove its independently actuated tool has stopped.
    If it fails: Repair tool recovery resource selection and measured-stop verification.
    Profile fields: groups.*.tool;groups.*.stop_verification
    Scenario fields: None.
    Robot effects: Offline recording adapter and fake clock only.
    """
    manager, lock, state, adapter, cache = prepared_recovery(environment, monkeypatch, 'moving' if moving else None)
    manager._groups['x'] = replace(manager._groups['x'], tool_joint='jaw')
    adapter.states['tool_controller'] = {'state': 'active', 'joints': ('jaw',)}
    lock.quarantine['tool:x'] = {'expected_controllers': {'tool_controller': 'active'}}
    observed = []
    def snapshot(names):
        assert names == ('jaw',)
        observed.append(names)
        return cache.stop_snapshot(names)
    adapter.stop_snapshot = snapshot
    if moving:
        with pytest.raises(RpcError): manager.recover('tool:x')
        assert 'tool:x' in lock.quarantine
    else:
        result = manager.recover('tool:x')
        assert result['recovered'] and result['stop_verification']['verified']
        assert 'tool:x' not in lock.quarantine
    assert observed and 'x' in lock.quarantine and state.estop_engaged
