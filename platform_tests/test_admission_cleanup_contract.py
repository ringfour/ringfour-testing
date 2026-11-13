"""A01/A02/A09 admission and non-group cleanup fault evidence.

Synthetic inputs and recording adapters only. No robot connection or ROS.
"""
import asyncio
import json
from types import SimpleNamespace

import pytest
from .conftest import command_id
from ringfour_agent.admission import DESCRIPTORS
from ringfour_agent.bootstrap import build_agent
from ringfour_agent.handlers import build_registry
from ringfour_agent.profile import load_profile
from ringfour_agent.server import Session, RpcError
from ringfour_agent.teleop import TeleopManager
from .conftest import STUB_PROFILE

pytestmark = pytest.mark.contract


@pytest.mark.protocol
@pytest.mark.parametrize('method,params', [
    ('teleop.drive', {'linear': True}),
    ('teleop.drive', {'linear': '0.2'}),
    ('teleop.drive', {'linear': float('nan')}),
    ('teleop.start', {'watchdog': 0}),
    ('group.control_start', {'group': ''}),
    ('group.start_trajectory', {'group': 'x', 'session_id': 's', 'waypoints': [[True]]}),
    ('group.start_trajectory', {'group': 'x', 'session_id': 's', 'waypoints': []}),
    ('group.hold', {'group': 'x'}),
    ('group.start_tool', {'group': 'x', 'position': '0.1'}),
    ('mobility.navigate_to', {'goal': {'x': float('inf')}}),
    ('mobility.navigate_to', {'goal': {'frame': 'map', 'x': True}}),
    ('mobility.navigate_to', {'goal': {'frame': 'map', 'x': '1'}}),
    ('mobility.navigate_to', {'goal': {'frame': 'map', 'q': {'w': 0}}}),
    ('mobility.navigate_to', {'goal': {'place': 'destination', 'x': 1}}),
    ('action.invoke', {'action': 'move', 'arguments': []}),
    ('teleop.drive', {'linear': .1, 'typo': .2}),
    ('teleop.drive', []),
    ('teleop.drive', None),
])
def test_invalid_motion_is_rejected_before_handler(tmp_path, method, params):
    """A01/A02: malformed motion has zero handler dispatches

    Categories: sdk
    What it checks: Sends malformed effectful parameters through the production wire dispatcher and records handler invocations.
    Why it matters: Coercion, nonfinite geometry and misspelled parameters must not reach actuators.
    If it fails: Repair the admission schema or dispatcher; robot profile changes cannot fix it.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline recording handler only; no ROS or robot.
    """
    async def exercise():
        server, ctx = build_agent(load_profile(STUB_PROFILE), unix_path=str(tmp_path / 'unused.sock'), audit_dir=tmp_path, allow_unverified_stops=True)
        calls = []
        async def handler(session, params):
            calls.append(params)
            return {}
        server._registry[method] = handler
        from websockets.asyncio.client import unix_connect
        await server.start()
        try:
            async with unix_connect(str(tmp_path / 'unused.sock')) as socket:
                await socket.send(json.dumps({'command_id': command_id(ctx), 'jsonrpc': '2.0', 'id': 0, 'method': 'hello',
                                             'params': {'protocol': 1, 'client_version': 'test'}}))
                assert 'result' in json.loads(await socket.recv())
                await socket.send(json.dumps({'command_id': command_id(ctx), 'jsonrpc': '2.0', 'id': 1,
                                              'method': method, 'params': params}))
                response = json.loads(await socket.recv())
                assert 'error' in response
                assert calls == []
        finally:
            await server.close()
            ctx.audit.close()
    asyncio.run(exercise())


def test_descriptors_resolve_resources_and_truthful_completion():
    """A01: admission declarations identify logical dependencies

    Categories: sdk
    What it checks: Resolves group, task and vendor resource declarations and checks that dispatch-only completion stays unverified.
    Why it matters: Resource and completion declarations must not imply physical success.
    If it fails: Repair the method descriptor inventory.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline declarations only.
    """
    assert DESCRIPTORS['group.start_trajectory'].resources({'group': 'arbitrary'}) == ('robot.motion', 'group:arbitrary')
    assert DESCRIPTORS['task.cancel'].resources({'task_id': 'generation'}) == ('task:generation',)
    assert 'measured' in DESCRIPTORS['group.start_tool'].completion
    assert 'unverified' in DESCRIPTORS['mobility.navigate_to'].completion


@pytest.mark.parametrize('path', ['stop', 'end', 'disconnect', 'watchdog', 'shutdown'])
def test_teleop_stop_failure_quarantines_without_success(tmp_path, path):
    """A09: failed teleop stop survives owner release

    Categories: sdk
    What it checks: Fails zero-command publication during stop, end, disconnect, watchdog and shutdown; verifies quarantine blocks later acquisitions and no success audit is emitted.
    Why it matters: A lost stop command must not silently release the robot for new motion.
    If it fails: Repair teleop cleanup and quarantine handling.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline failing adapter only.
    """
    async def exercise():
        server, ctx = build_agent(load_profile(STUB_PROFILE), unix_path=str(tmp_path / 'unused.sock'), audit_dir=tmp_path, allow_unverified_stops=True)
        attempts = []
        def zero():
            attempts.append('zero')
            raise RuntimeError('injected publication failure')
        manager = TeleopManager(ctx.lock, ctx.audit, SimpleNamespace(publish_zero=zero),
                                ctx.profile, ctx.safety, allow_unverified_stops=True)
        ctx.teleop = manager
        manager.start(1, .05)
        # A second child must survive failed teleop teardown.
        ctx.lock.acquire(1, 'other-child')
        if path == 'watchdog':
            active = manager._sessions[1]
            active.last_command -= 1
            await asyncio.wait_for(active.watchdog_task, 1)
        elif path == 'shutdown':
            await server.close()
        else:
            with pytest.raises(RpcError) as error:
                {'stop': manager.stop, 'end': manager.end,
                 'disconnect': manager.on_disconnect}[path](1)
            assert error.value.error_type == 'CleanupFailed'
        assert attempts
        assert 'mobility' in ctx.lock.quarantine
        assert 'other-child' in ctx.lock._acquisitions
        ctx.lock.release_all(1)
        with pytest.raises(RpcError, match='inhibited'):
            ctx.lock.acquire(2, 'new')
        if 1 in manager._sessions:
            with pytest.raises(RpcError): manager.end(1)
        events = ctx.audit.query()
        assert not any(e.outcome == 'success' and e.detail.get('reason') in ('disconnect', 'teleop watchdog') for e in events)
        ctx.audit.close()
    asyncio.run(exercise())


@pytest.mark.parametrize('failure', ['cancel', 'zero', 'both'])
def test_mobility_stop_attempts_both_paths_and_quarantines(tmp_path, failure):
    """A09: mobility stop retains all independent failures

    Categories: sdk
    What it checks: Fails cancellation and zero publication separately and together; verifies both paths are attempted and quarantine persists after release.
    Why it matters: Failure of one stop path must not suppress the other or fabricate success.
    If it fails: Repair navigation stop cleanup and report propagation.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline failing adapter only.
    """
    async def exercise():
        server, ctx = build_agent(load_profile(STUB_PROFILE), unix_path=str(tmp_path / 'unused.sock'), audit_dir=tmp_path, allow_unverified_stops=True)
        calls = []
        def cancel():
            calls.append('cancel')
            if failure in ('cancel', 'both'): raise RuntimeError('cancel failed')
        def zero():
            calls.append('zero')
            if failure in ('zero', 'both'): raise RuntimeError('zero failed')
        ctx.ros = SimpleNamespace(nav_cancel=cancel, publish_zero=zero)
        handler = build_registry(ctx)['mobility.stop']
        with pytest.raises(RpcError) as error:
            await handler(Session(None), {})
        assert error.value.error_type == 'CleanupFailed'
        assert calls == ['cancel', 'zero']
        assert len(error.value.extra['cleanup']['failures']) == (2 if failure == 'both' else 1)
        assert ctx.lock.quarantine['mobility']['physical_stop'] == 'unverified'
        ctx.audit.close()
    asyncio.run(exercise())


@pytest.mark.parametrize('phase', ['submission', 'execution'])
def test_navigation_cancel_failure_is_terminal_and_quarantined(tmp_path, phase):
    """A09: navigation cancellation failure cannot become cancellation success

    Categories: sdk
    What it checks: Cancels navigation while goal submission is in flight and during execution, then fails the adapter cancellation acknowledgement.
    Why it matters: Reply timing must not bypass cleanup quarantine or fabricate cancellation.
    If it fails: Repair navigation submission shielding and cancellation reports.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline recording adapter with controlled thread events only.
    """
    import threading
    async def exercise():
        server, ctx = build_agent(load_profile(STUB_PROFILE), unix_path=str(tmp_path / 'unused.sock'), audit_dir=tmp_path, allow_unverified_stops=True)
        entered, release, waiting = threading.Event(), threading.Event(), threading.Event()
        calls = []
        def start(goal):
            calls.append('dispatch')
            entered.set()
            assert release.wait(2)
        def poll(timeout):
            waiting.set()
            return None
        def cancel():
            calls.append('cancel')
            raise RuntimeError('injected cancellation rejection')
        ctx.ros = SimpleNamespace(clear_costmaps=lambda: None, nav_start=start,
                                  nav_wait=poll, nav_cancel=cancel, nav_feedback=lambda: None)
        class Socket:
            async def send(self, value): pass
        owner = Session(Socket())
        handler = build_registry(ctx)['mobility.navigate_to']
        try:
            response = await handler(owner, {'goal': {'x': 0, 'y': 0, 'z': 0,
                  'q': {'x': 0, 'y': 0, 'z': 0, 'w': 1}, 'frame': 'map'}})
            assert await asyncio.to_thread(entered.wait, 2)
            if phase == 'execution':
                release.set()
                assert await asyncio.to_thread(waiting.wait, 2)
            await ctx.tasks.cancel(response['task_id'], owner)
            release.set()
            await ctx.tasks.drain_resource('mobility')
            result = ctx.tasks.status(response['task_id'], owner)
            assert result['state'] == 'failed'
            assert result['error']['type'] == 'CleanupFailed'
            assert calls == ['dispatch', 'cancel']
            assert ctx.lock.owner is None
            with pytest.raises(RpcError, match='inhibited'):
                ctx.lock.acquire(owner.client_id + 1, 'later')
        finally:
            release.set()
            await ctx.tasks.cancel_all()
            ctx.audit.close()
    asyncio.run(exercise())


@pytest.mark.protocol
@pytest.mark.parametrize('method,params', [
    ('teleop.start', {'watchdog': .5}),
    ('teleop.drive', {'linear': .1}),
    ('group.control_start', {'group': 'arbitrary', 'role': 'trajectory'}),
    ('group.start_trajectory', {'group':'arbitrary','session_id':'s','trajectory':{'configurations':[],'times':[]}}),
    ('group.hold', {'group': 'arbitrary', 'session_id': 's'}),
    ('group.start_tool', {'group': 'arbitrary', 'position': .1}),
    ('mobility.navigate_to', {'goal': {'frame': 'map', 'x': 1}}),
])
def test_estop_rejects_every_motion_family_over_protocol(tmp_path, method, params):
    """A02: each motion family rejects before actuator dispatch

    Categories: sdk
    What it checks: Sends valid motion requests under inhibit through a real Unix WebSocket listener; a recording handler detects any forbidden dispatch.
    Why it matters: Navigation, teleop, scopes, trajectories, hold and tools share one central gate.
    If it fails: Repair central admission rather than tuning robot profiles.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Isolated protocol and recording handler only.
    """
    from websockets.asyncio.client import unix_connect
    async def exercise():
        server, ctx = build_agent(load_profile(STUB_PROFILE), unix_path=str(tmp_path / 'gate.sock'), audit_dir=tmp_path, allow_unverified_stops=True)
        calls = []
        async def dispatch(session, params):
            calls.append(params)
            return {}
        server._registry[method] = dispatch
        ctx.state.estop_engaged = True
        await server.start()
        try:
            async with unix_connect(str(tmp_path / 'gate.sock')) as socket:
                await socket.send(json.dumps({'command_id': command_id(ctx), 'jsonrpc': '2.0', 'id': 0, 'method': 'hello',
                                             'params': {'protocol': 1, 'client_version': 'test'}}))
                assert 'result' in json.loads(await socket.recv())
                await socket.send(json.dumps({'command_id': command_id(ctx), 'jsonrpc': '2.0', 'id': 1, 'method': method, 'params': params}))
                response = json.loads(await socket.recv())
                assert response['error']['data']['reason'] == 'estop'
                assert calls == []
        finally:
            await server.close()
            ctx.audit.close()
    asyncio.run(exercise())


def test_teleop_bootstrap_shares_dispatch_safety_layer(tmp_path):
    """A02: assembled teleop uses the central execution gate

    Categories: sdk
    What it checks: Boots a synthetic mobility profile with an adapter and verifies teleop rechecks the same safety state at drive dispatch.
    Why it matters: Offline managers and live bootstrap must enforce the same inhibit.
    If it fails: Repair bootstrap wiring and execution-time admission.
    Profile fields: teleop;mobility
    Scenario fields: None.
    Robot effects: Offline recording adapter only.
    """
    async def exercise():
        path = tmp_path / 'profile.yaml'
        path.write_text("name: synthetic\nmodel: synthetic\nversion: '1'\nprotocol: 1\nframes: {map: map, odom: odom, base: base_link}\nmobility: {locomotion: differential}\nteleop: {max_watchdog: 1}\n")
        calls = []
        ros = SimpleNamespace(publish_twist=lambda *args: calls.append(args), publish_zero=lambda: None)
        server, ctx = build_agent(load_profile(path), unix_path=str(tmp_path / 'unused.sock'), audit_dir=tmp_path, ros=ros, allow_unverified_stops=True)
        assert ctx.teleop._safety is ctx.safety
        ctx.teleop.start(1, .5)
        ctx.state.estop_engaged = True
        with pytest.raises(RpcError, match='e-stop'):
            ctx.teleop.drive(1, .1, 0)
        assert calls == []
        await server.close()
        ctx.audit.close()
    asyncio.run(exercise())


def test_empty_map_reports_unavailable_before_payload_serialization(tmp_path):
    """Absent map data is explicit rather than an internal serialization error

    Categories: sdk
    What it checks: Supplies an empty occupancy grid and verifies typed unavailable evidence before shared memory allocation.
    Why it matters: An empty environment may not yield a SLAM map; empty data must not masquerade as a usable map.
    If it fails: Repair map availability validation.
    Profile fields: slam
    Scenario fields: None.
    Robot effects: Offline synthetic grid only.
    """
    import numpy as np
    async def exercise():
        server, ctx = build_agent(load_profile(STUB_PROFILE), unix_path=str(tmp_path / 'unused.sock'), audit_dir=tmp_path, allow_unverified_stops=True)
        ctx.ros = SimpleNamespace(map_grid=lambda: {'width': 0, 'height': 0, 'data': np.zeros((0, 0), dtype=np.int8)})
        from ringfour_agent.profile import Profile
        ctx.profile = Profile(spec=ctx.profile.spec, info=ctx.profile.info.model_copy(update={'capabilities': [*ctx.profile.info.capabilities, 'slam']}))
        with pytest.raises(RpcError) as error:
            await build_registry(ctx)['slam.get_map'](Session(None), {})
        assert error.value.error_type == 'CompletionUnverified'
        assert error.value.extra == {'reason': 'map unavailable', 'width': 0, 'height': 0}
        assert ctx.shm._live == {}
        await server.close()
        ctx.audit.close()
    asyncio.run(exercise())


@pytest.mark.parametrize("value", [0., -1., True, float("inf"), "30"])
def test_tool_execution_budget_requires_finite_positive_number(value):
    """Tool travel deadline is an explicit finite budget

    Categories: sdk,safety
    What it checks: Rejects zero, negative, boolean, infinite and string tool execution deadlines before dispatch.
    Why it matters: Target travel and measured stopping need separate bounded deadlines on every robot.
    If it fails: Repair tool admission; do not relax native stop thresholds.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline admission validation only.
    """
    from ringfour_agent.admission import DESCRIPTORS
    from ringfour_agent.server import RpcError
    with pytest.raises(RpcError):
        DESCRIPTORS["group.start_tool"].schema.validate({"group": "x", "position": .2, "execution_timeout": value})
