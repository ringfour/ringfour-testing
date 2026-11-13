"""Strict milestone A contracts; no legacy acknowledgement-only fixtures."""
import asyncio
import json
from dataclasses import replace
from types import SimpleNamespace
import pytest
from .conftest import command_id
from ringfour.enums import Controller
from ringfour_agent.bootstrap import build_agent
from ringfour_agent.models.profile import ActionConfig, MobilityConfig
from ringfour_agent.profile import load_profile
from ringfour_agent.server import RpcError, Session
from ringfour_agent.stopping import require_mobility_stop, verify_mobility_stop, verify_navigation_arrival
from ringfour_agent.teleop import TeleopManager
from ringfour_agent.vendor import VendorActions
from .conftest import STUB_PROFILE
from .test_hardening_contract import environment
from .test_stop_recovery_contract import Clock, StopAdapter, POLICY

pytestmark = pytest.mark.contract


class BodyAdapter:
    def __init__(self, clock, fault=None):
        self.clock, self.fault, self.calls = clock, fault, []
    def mobility_stop_snapshot(self):
        now = self.clock.now
        result = dict(position=(0.,0.,0.), orientation=(0.,0.,0.,1.), linear=(0.,0.,0.), angular=(0.,0.,0.),
                      source_stamp=now, received=now, source_now=now, receive_now=now)
        if self.fault == 'vertical': result['linear'] = (0.,0.,.1)
        if self.fault == 'roll': result['angular'] = (.1,0.,0.)
        if self.fault == 'moving-position': result['position'] = (0.,0.,now)
        if self.fault == 'moving-orientation':
            import math
            result['orientation'] = (math.sin(now/2),0.,0.,math.cos(now/2))
        if self.fault == 'frozen': result.update(source_stamp=0., received=0.)
        if self.fault == 'paused-source': result.update(source_stamp=0., source_now=0.)
        if self.fault == 'stale': result['source_stamp'] = now-2
        if self.fault == 'future': result['source_stamp'] = now+2
        if self.fault == 'nonfinite': result['angular'] = (float('nan'),0.,0.)
        if self.fault == 'missing': result.pop('angular')
        if self.fault == 'bad-quaternion': result['orientation'] = (0.,0.,0.,0.)
        if self.fault == 'quaternion-sign': result['orientation'] = (0.,0.,0.,-1. if int(now*100)%2 else 1.)
        return result
    def publish_zero(self): self.calls.append('zero')
    def publish_twist(self, *args): self.calls.append(('drive', args))
    def vendor_ready(self, name): return True
    def vendor_publish(self, name, payload): self.calls.append((name, payload))
    def navigation_pose_snapshot(self, frame):
        self.calls.append(('pose', frame))
        return self.mobility_stop_snapshot()


def configured(profile):
    return replace(profile, spec=profile.spec.model_copy(update={'safety': profile.spec.safety.model_copy(update={'stop_verification': POLICY})}))


@pytest.mark.parametrize('fault', ['vertical','roll','moving-position','moving-orientation','frozen','paused-source','stale','future','nonfinite','missing','bad-quaternion'])
def test_body_stop_never_accepts_incomplete_or_moving_3d_evidence(fault):
    """Body stop needs progressive complete 3D evidence

    Categories: sdk
    What it checks: Injects vertical or roll motion, fabricated zero twist, frozen clocks and incomplete body feedback into strict stop verification.
    Why it matters: The same stopping contract must cover legged, flying and wheeled robots.
    If it fails: Fix shared measured-stop verification; never hide missing axes or provenance.
    Profile fields: safety.stop_verification
    Scenario fields: None.
    Robot effects: Offline adapter and deterministic clock only.
    """
    clock = Clock()
    with pytest.raises(RpcError) as exc:
        verify_mobility_stop(BodyAdapter(clock, fault), POLICY, .5, clock=clock.read, sleep=clock.sleep)
    assert exc.value.error_type == 'StopUnverified'
    assert clock.now <= POLICY.timeout


@pytest.mark.parametrize('fault', [None, 'quaternion-sign'])
def test_stationary_body_stop_reports_measured_dwell(fault):
    """Measured body rest preserves native units and provenance

    Categories: sdk
    What it checks: Verifies stationary 3D samples under a fake clock, including equivalent quaternion signs.
    Why it matters: Cache reads and quaternion sign changes must not invent movement or stopping evidence.
    If it fails: Repair dwell or orientation math.
    Profile fields: safety.stop_verification
    Scenario fields: None.
    Robot effects: Offline only.
    """
    clock = Clock()
    report = verify_mobility_stop(BodyAdapter(clock, fault), POLICY, .5, clock=clock.read, sleep=clock.sleep)
    assert report['verified'] and report['samples'] > 1 and report['elapsed'] >= POLICY.dwell
    assert report['physical_stop'] == 'uncommissioned'


@pytest.mark.parametrize('limit', ['angular_velocity_limit', 'linear_velocity_limit'])
def test_body_stop_requires_both_native_unit_limits(limit):
    """Body stop cannot reuse angular limits for linear motion

    Categories: sdk
    What it checks: Removes each native-unit policy threshold before admission.
    Why it matters: Morphology does not determine safe measured velocity limits.
    If it fails: Repair policy admission.
    Profile fields: safety.stop_verification
    Scenario fields: None.
    Robot effects: Offline only.
    """
    with pytest.raises(RpcError):
        require_mobility_stop(BodyAdapter(Clock()), POLICY.model_copy(update={limit: None}), .5)


@pytest.mark.parametrize('inhibited', [False, True])
def test_strict_group_rejects_missing_stop_contract_before_switch(environment, inhibited):
    """Strict group admission precedes every controller effect

    Categories: sdk
    What it checks: Opens an unconfigured group through the normal strict manager under safe and inhibited state.
    Why it matters: A profile omission must reject before any controller switch or ownership leak.
    If it fails: Repair the production admission boundary.
    Profile fields: safety.stop_verification, groups.*.stop_verification
    Scenario fields: None.
    Robot effects: Offline recording controllers only.
    """
    manager, lock, state, adapter, _ = environment
    manager.allow_unverified_stops = False
    state.estop_engaged = inhibited
    with pytest.raises(RpcError) as exc: manager.start(1, 'x', Controller.TRAJECTORY)
    assert exc.value.error_type == ('SafetyViolation' if inhibited else 'StopUnverified')
    assert adapter.calls == [] and lock.owner is None


def test_strict_group_inherits_shared_policy_and_rechecks_feedback(environment, monkeypatch):
    """A shared contract applies to each model-resolved joint group

    Categories: sdk
    What it checks: Acquires with shared policy, then removes required velocity evidence before dispatch.
    Why it matters: Admission at scope entry cannot authorize later stale or partial commands.
    If it fails: Repair dispatch-time feedback checks.
    Profile fields: safety.stop_verification
    Scenario fields: None.
    Robot effects: Offline only.
    """
    manager, lock, _, adapter, _ = environment
    manager.allow_unverified_stops = False
    manager._stop_policy = POLICY
    clock = Clock(); cache = StopAdapter(clock)
    adapter.stop_snapshot, adapter.joint_type = cache.stop_snapshot, cache.joint_type
    import ringfour_agent.stopping as stopping
    original = stopping.verify_joint_stop
    monkeypatch.setattr(stopping, 'verify_joint_stop', lambda a,n,p,m: original(a,n,p,m,clock=clock.read,sleep=clock.sleep))
    opened = manager.start(1, 'x', Controller.TRAJECTORY)
    cache.fault = 'nonfinite-velocity'
    calls = []
    with pytest.raises(RpcError): manager.dispatch(opened, lambda: calls.append('motion'))
    assert not calls
    cache.fault = None
    report = manager.end('x', client_id=1, session_id=opened.id)
    assert report['confirmed'] and report['stop_verification']['verified'] and lock.owner is None


@pytest.mark.parametrize('path', ['stop','end','disconnect','watchdog','shutdown'])
@pytest.mark.parametrize('fault', [None, 'vertical'])
def test_strict_teleop_cleanup_verifies_or_quarantines(tmp_path, monkeypatch, path, fault):
    """Every teleop lifetime boundary uses the same measured contract

    Categories: sdk
    What it checks: Ends, stops, disconnects, expires or shuts down an admitted teleop session with stationary or continuing vertical motion.
    Why it matters: Publishing zero alone cannot produce successful cleanup on any robot morphology.
    If it fails: Repair shared stopping, ownership retention or quarantine.
    Profile fields: safety.stop_verification
    Scenario fields: None.
    Robot effects: Offline fake clock and adapter only.
    """
    async def exercise():
        server, ctx = build_agent(configured(load_profile(STUB_PROFILE)), unix_path=str(tmp_path/'unused'), audit_dir=tmp_path)
        clock = Clock(); adapter = BodyAdapter(clock)
        import ringfour_agent.stopping as stopping
        original = stopping.verify_mobility_stop
        monkeypatch.setattr(stopping, 'verify_mobility_stop', lambda a,p,m: original(a,p,m,clock=clock.read,sleep=clock.sleep))
        manager = TeleopManager(ctx.lock, ctx.audit, adapter, ctx.profile, ctx.safety)
        manager.start(1, .05); adapter.fault = fault
        try:
            if path == 'watchdog':
                session = manager._sessions[1]
                session.last_command -= 1
                await asyncio.sleep(.05)
                assert session.halted_by_watchdog
            else:
                call = {'stop': lambda: manager.stop(1), 'end': lambda: manager.end(1),
                        'disconnect': lambda: manager.on_disconnect(1), 'shutdown': manager.end_all}[path]
                if fault:
                    with pytest.raises(RpcError): await asyncio.to_thread(call)
                else: await asyncio.to_thread(call)
            assert bool(ctx.lock.quarantine) == bool(fault)
            assert adapter.calls == ['zero']
            if fault:
                with pytest.raises(RpcError): manager.drive(1, .1, 0.)
                ctx.lock.release_all(1)
                with pytest.raises(RpcError): manager.start(2, .05)
        finally:
            adapter.fault = None
            try: manager.end_all()
            except RpcError: pass
            ctx.audit.close(); ctx.shm.close()
    asyncio.run(exercise())


@pytest.mark.parametrize('fault', ['missing-policy','missing-stop-payload','unknown-resource',None,'vertical'])
def test_vendor_motion_requires_stop_contract_and_cleans_up(tmp_path, monkeypatch, fault):
    """Opaque vendor motion needs an explicit stop payload and observable resource

    Categories: sdk
    What it checks: Validates a vendor motion contract before publication and verifies its explicit stop on release, including continued motion.
    Why it matters: Arbitrary topic publication cannot bypass robot-wide motion ownership or invent a stop command.
    If it fails: Repair vendor admission or lifetime cleanup.
    Profile fields: actions.*.motion_resource, actions.*.stop_arguments, safety.stop_verification
    Scenario fields: None.
    Robot effects: Offline publisher and body feedback only.
    """
    profile = configured(load_profile(STUB_PROFILE))
    if fault == 'missing-policy': profile = load_profile(STUB_PROFILE)
    action = ActionConfig(topic='/vendor', type='std_msgs/Float64', is_motion=True, motion_resource='mobility',
                          stop_arguments={'data': 0.}, parameters={'data': {'type':'float'}})
    if fault == 'missing-stop-payload': action = action.model_copy(update={'stop_arguments': None})
    if fault == 'unknown-resource': action = action.model_copy(update={'motion_resource': 'mystery'})
    server, ctx = build_agent(profile, unix_path=str(tmp_path/'unused'), audit_dir=tmp_path)
    clock = Clock(); adapter = BodyAdapter(clock)
    import ringfour_agent.stopping as stopping
    original = stopping.verify_mobility_stop
    monkeypatch.setattr(stopping, 'verify_mobility_stop', lambda a,p,m: original(a,p,m,clock=clock.read,sleep=clock.sleep))
    actions = VendorActions(adapter, {'move': action}, ctx.audit, ctx.safety, ctx.lock, profile=profile)
    try:
        if fault in ('missing-policy','missing-stop-payload','unknown-resource'):
            with pytest.raises(RpcError): actions.invoke('move', {'data': 1.}, 1)
            assert not adapter.calls and ctx.lock.owner is None
        else:
            receipt = actions.invoke('move', {'data': 1.}, 1)
            assert receipt['completion'] == 'unverified' and ctx.lock.owner == 1
            adapter.fault = fault
            if fault:
                with pytest.raises(RpcError): actions.release(1)
                assert 'action:move' in ctx.lock.quarantine
            else: actions.release(1)
            assert adapter.calls == [('move', {'data': 1.}), ('move', {'data': 0.})]
            assert ctx.lock.owner is None
    finally:
        ctx.audit.close(); ctx.shm.close()


@pytest.mark.parametrize('fault', [None,'vertical','moving-position','stale','missing','bad-quaternion'])
def test_navigation_terminal_success_requires_fresh_full_pose(fault):
    """Navigation controller success is checked against measured 3D pose

    Categories: sdk
    What it checks: Compares framed measured pose with the declared goal and native metre/radian tolerances, rejecting incomplete or stale evidence.
    Why it matters: A controller acknowledgement cannot conceal false arrival or vertical drift.
    If it fails: Repair navigation completion verification.
    Profile fields: mobility.arrival_position_tolerance, mobility.arrival_orientation_tolerance
    Scenario fields: None.
    Robot effects: Offline only.
    """
    clock = Clock(); clock.now = 1.
    adapter = BodyAdapter(clock, fault)
    goal = {'frame':'map', 'x':0.,'y':0.,'z':0., 'q': {'x':0.,'y':0.,'z':0.,'w':1.}}
    if fault in ('moving-position','stale','bad-quaternion'):
        with pytest.raises(RpcError) as exc: verify_navigation_arrival(adapter, goal, MobilityConfig(locomotion='differential'), .5)
        assert exc.value.error_type == 'CompletionUnverified'
    else:
        # Arrival validates position/orientation; measured rest is a separate
        # mandatory operation step, tested above for vertical/missing velocity.
        assert verify_navigation_arrival(adapter, goal, MobilityConfig(locomotion='differential'), .5)['verified']


@pytest.mark.parametrize('fault', ['missing-policy','missing-velocity','publish','stop-unverified',None])
def test_strict_tool_owns_full_completion_and_failure_cleanup(environment, tmp_path, monkeypatch, fault):
    """Tool positioning retains its joint until completion and cleanup

    Categories: sdk
    What it checks: Commands a tool through production RPC, injecting missing contract, feedback and publication errors, and checks retained ownership during verification.
    Why it matters: A tool is motion; releasing its acquisition immediately after publish permits conflicting commands.
    If it fails: Repair tool admission, completion evidence and holding cleanup.
    Profile fields: groups.*.tool, safety.stop_verification
    Scenario fields: None.
    Robot effects: Offline recording adapter only.
    """
    async def exercise():
        manager, lock, state, adapter, audit = environment
        manager.allow_unverified_stops = False
        manager._stop_policy = None if fault == 'missing-policy' else POLICY
        from ringfour_agent.models.profile import ToolConfig
        group = manager._groups['x']
        config = group.config.model_copy(update={'controllers': {**group.config.controllers, Controller.TOOL:'tool_controller'},
                                                'tool': ToolConfig(type='test', joint='jaw')})
        manager._groups['x'] = replace(group, config=config, tool_joint='jaw')
        clock = Clock(); cache = StopAdapter(clock, 'nonfinite-velocity' if fault == 'missing-velocity' else None)
        adapter.stop_snapshot, adapter.joint_type = cache.stop_snapshot, cache.joint_type
        adapter.verified_joint_snapshot = lambda n,m: {j:(0.,0.,0.) for j in n}
        adapter.joint_limits = lambda: {'jaw':(-1.,1.)}
        def command(controller, names, positions):
            adapter.calls.append(('tool', tuple(positions)))
            assert lock.owner == 1 and any(t.startswith('tool:x:') for t in lock._acquisitions)
            if fault == 'publish': raise RuntimeError('publication reply lost')
        adapter.states['tool_controller']={'state':'active','joints':('jaw',)}
        adapter.command_positions = command
        import ringfour_agent.stopping as stopping
        original = stopping.verify_joint_stop
        def verify(a,n,p,m,**kwargs):
            assert lock.owner == 1
            if fault == 'stop-unverified': cache.fault = 'moving'
            return original(a,n,p,m,clock=clock.read,sleep=clock.sleep,**kwargs)
        monkeypatch.setattr(stopping,'verify_joint_stop',verify)
        server,ctx = build_agent(configured(load_profile(STUB_PROFILE)), unix_path=str(tmp_path/'tool'), audit_dir=tmp_path/'wire')
        ctx.ros,ctx.control,ctx.groups,ctx.lock,ctx.safety,ctx.state = adapter,manager,manager._groups,lock,manager._safety,state
        session = Session(SimpleNamespace()); session.client_id=1; session.handshaken=True
        try:
            reply = json.loads(await server._dispatch(session,json.dumps({'command_id': command_id(ctx), 'jsonrpc':'2.0','id':1,'method':'group.start_tool','params':{'group':'x','position':0.}})))
            assert 'result' in reply, reply
            record=ctx.tasks._tasks[reply['result']['task_id']]
            await record.aio_task
            await asyncio.sleep(0)
            if fault: assert record.state=='failed', record.update
            else: assert record.state=='succeeded' and record.update.result['position']==0.
            assert lock.owner is None
            assert bool(lock.quarantine) == (fault in ('publish','stop-unverified'))
            if fault in ('missing-policy','missing-velocity'): assert adapter.calls == []
            if fault in ('publish','stop-unverified'): assert len([c for c in adapter.calls if c[0]=='tool']) == 2
        finally: ctx.audit.close();ctx.shm.close()
    asyncio.run(exercise())


@pytest.mark.parametrize('fault', ['missing-policy','missing-feedback','lost-acceptance','wait-error','false-arrival','stop-unverified',None])
def test_strict_navigation_rpc_retains_truthful_terminal_outcome(tmp_path, monkeypatch, fault):
    """Navigation retains stopping evidence after submission uncertainty

    Categories: sdk
    What it checks: Uses production dispatch and task lifecycle with missing contracts, lost acceptance, wait failure, false arrival and continuing motion.
    Why it matters: Releasing navigation ownership must not conceal an accepted goal or falsely report arrival.
    If it fails: Repair submission cleanup, measured stopping and task terminal outcomes.
    Profile fields: safety.stop_verification, mobility.arrival_position_tolerance
    Scenario fields: None.
    Robot effects: Offline protocol dispatcher and fake adapter only.
    """
    async def exercise():
        profile=load_profile(STUB_PROFILE) if fault=='missing-policy' else configured(load_profile(STUB_PROFILE))
        server,ctx=build_agent(profile,unix_path=str(tmp_path/'nav'),audit_dir=tmp_path)
        clock=Clock();adapter=BodyAdapter(clock,'missing' if fault=='missing-feedback' else None)
        def start(goal):
            adapter.calls.append('start')
            assert ctx.lock.owner==1
            if fault=='lost-acceptance': raise RuntimeError('accepted then lost response')
        def wait(timeout):
            if fault=='wait-error': raise RuntimeError('result channel lost')
            if fault=='stop-unverified': adapter.fault='vertical'
            return {'status':'succeeded','error_code':0,'error_msg':''}
        adapter.nav_start=start;adapter.nav_wait=wait
        adapter.clear_costmaps=lambda: None
        adapter.nav_cancel=lambda:adapter.calls.append('cancel')
        ctx.ros=adapter
        import ringfour_agent.stopping as stopping
        original=stopping.verify_mobility_stop
        monkeypatch.setattr(stopping,'verify_mobility_stop',lambda a,p,m:original(a,p,m,clock=clock.read,sleep=clock.sleep))
        session=Session(SimpleNamespace(send=lambda x: asyncio.sleep(0)));session.client_id=1;session.handshaken=True
        goal={'frame':'map','x':1. if fault=='false-arrival' else 0.,'y':0.,'z':0.,'q':{'x':0.,'y':0.,'z':0.,'w':1.}}
        try:
            reply=json.loads(await server._dispatch(session,json.dumps({'command_id': command_id(ctx), 'jsonrpc':'2.0','id':1,'method':'mobility.navigate_to','params':{'goal':goal}})))
            if fault in ('missing-policy','missing-feedback'):
                assert 'error' in reply and not adapter.calls
            else:
                assert 'result' in reply, reply
                record=ctx.tasks._tasks[reply['result']['task_id']]
                await record.aio_task
                if fault: assert record.state=='failed'
                else: assert record.state=='succeeded' and record.update.result['arrival_verification']['verified']
                if fault in ('lost-acceptance','wait-error'): assert 'cancel' in adapter.calls and 'zero' in adapter.calls
                if fault=='stop-unverified': assert 'mobility' in ctx.lock.quarantine
            await asyncio.sleep(0)
            assert ctx.lock.owner is None
        finally:ctx.audit.close();ctx.shm.close()
    asyncio.run(exercise())


@pytest.mark.parametrize('path', ['terminal','ack-only','reject','unknown','late-acceptance','lost-result'])
def test_action_handoff_requires_terminal_result_and_stops_late_acceptance(path):
    """Uncertain action acceptance cannot escape scoped stopping

    Categories: sdk
    What it checks: Uses real production action handoff with controllable futures for rejection, acknowledgement-only cancellation, late acceptance and lost terminal result.
    Why it matters: An acceptance reply arriving after timeout must still receive cancellation, while quarantine remains until separately verified.
    If it fails: Repair shared action submission handoff rather than changing robot timing tolerances.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline futures only.
    """
    from concurrent.futures import Future
    from threading import Event
    from ringfour_agent.action_submission import ActionSubmission
    accepted, terminal, cancelled = Future(), Future(), Event()
    class Handle:
        accepted=True
        def get_result_async(self):return terminal
        def cancel_goal_async(self):
            cancelled.set(); response=Future();response.set_result(SimpleNamespace(goals_canceling=[self]));return response
    handoff=ActionSubmission(accepted,lambda h,r:None)
    if path in ('unknown','late-acceptance'):
        with pytest.raises(RpcError):handoff.wait(.001)
        with pytest.raises(RpcError):handoff.stop(.001)
        if path=='late-acceptance':
            terminal.set_result(SimpleNamespace(status='canceled'))
            accepted.set_result(Handle())
            assert cancelled.wait(1.)
    else:
        accepted.set_result(None if path=='reject' else Handle())
        if path=='terminal':terminal.set_result(SimpleNamespace(status='canceled'))
        if path=='lost-result':terminal.set_exception(RuntimeError('result channel unavailable'))
        if path in ('ack-only','lost-result'):
            with pytest.raises((RpcError,RuntimeError)):handoff.stop(.001)
        else:handoff.stop(.001)
        assert cancelled.is_set()==(path!='reject')


def test_concurrent_teleop_end_retains_ownership_until_watchdog_stop_finishes(tmp_path, monkeypatch):
    """Disconnect cannot release a watchdog's in-flight stop

    Categories: sdk
    What it checks: Blocks measured verification on an explicit barrier, overlaps watchdog and end, and inspects ownership and event-loop responsiveness.
    Why it matters: Worker cancellation does not cancel a running stop verifier; teardown must join it before permitting another owner.
    If it fails: Repair stop coalescing and acquisition release ordering.
    Profile fields: safety.stop_verification
    Scenario fields: None.
    Robot effects: Offline barriers and fake adapter only.
    """
    async def exercise():
        from threading import Event
        server,ctx=build_agent(configured(load_profile(STUB_PROFILE)),unix_path=str(tmp_path/'race'),audit_dir=tmp_path)
        entered,finish=Event(),Event();adapter=BodyAdapter(Clock())
        def verify(*args):
            entered.set()
            assert finish.wait(2.)
            return {'verified':True,'physical_stop':'uncommissioned'}
        import ringfour_agent.stopping as stopping
        monkeypatch.setattr(stopping,'verify_mobility_stop',verify)
        manager=TeleopManager(ctx.lock,ctx.audit,adapter,ctx.profile,ctx.safety)
        manager.start(1,.05);manager._sessions[1].last_command-=1
        try:
            assert await asyncio.to_thread(entered.wait,1.)
            ending=asyncio.create_task(asyncio.to_thread(manager.end,1))
            await asyncio.sleep(.02)
            assert not ending.done() and ctx.lock.owner==1
            with pytest.raises(RpcError):manager.start(2,.05)
            # Monitoring event loop has continued while both workers wait.
            finish.set();await ending
            assert ctx.lock.owner is None and adapter.calls==['zero']
        finally:
            finish.set();manager.end_all();ctx.audit.close();ctx.shm.close()
    asyncio.run(exercise())


def test_strict_disjoint_scope_cleanup_cannot_cancel_other_group(environment, monkeypatch):
    """Closing an idle child cannot cancel another child's trajectory

    Categories: sdk
    What it checks: Acquires two strict disjoint model-resolved groups, marks one trajectory running, closes the other and checks counted ownership and cancellation targeting.
    Why it matters: A shared ROS adapter must not let one child teardown stop or release unrelated work.
    If it fails: Repair resource targeting and counted acquisitions.
    Profile fields: safety.stop_verification
    Scenario fields: None.
    Robot effects: Offline model-resolved recording adapter only.
    """
    manager,lock,_,adapter,_=environment
    manager.allow_unverified_stops=False;manager._stop_policy=POLICY
    clock=Clock();cache=StopAdapter(clock)
    adapter.stop_snapshot,adapter.joint_type=cache.stop_snapshot,cache.joint_type
    import ringfour_agent.stopping as stopping
    original=stopping.verify_joint_stop
    monkeypatch.setattr(stopping,'verify_joint_stop',lambda a,n,p,m:original(a,n,p,m,clock=clock.read,sleep=clock.sleep))
    x=manager.start(1,'x',Controller.TRAJECTORY);y=manager.start(1,'y',Controller.TRAJECTORY)
    x.running=True
    report=manager.end('y',client_id=1,session_id=y.id)
    assert report['confirmed'] and x.running and x.id in lock._acquisitions and y.id not in lock._acquisitions
    assert not any(c[0]=='cancel' for c in adapter.calls)
    assert not lock.acquire(2,'competing')
    manager.end('x',client_id=1,session_id=x.id)
    assert lock.owner is None and sum(c[0]=='cancel' for c in adapter.calls)==1


def test_inhibit_during_controller_preparation_prevents_activation(environment, monkeypatch):
    """Controller preparation cannot outlive motion eligibility

    Categories: sdk
    What it checks: Engages inhibit after a slow preparation step and asserts no activation, rollback of that acquisition and no ownership leak.
    Why it matters: An admission check made before adapter waiting cannot authorize a later controller switch.
    If it fails: Repair execution-time eligibility checks.
    Profile fields: safety.stop_verification
    Scenario fields: None.
    Robot effects: Offline recording adapter only.
    """
    manager,lock,state,adapter,_=environment
    manager.allow_unverified_stops=False;manager._stop_policy=POLICY
    clock=Clock();cache=StopAdapter(clock)
    adapter.stop_snapshot,adapter.joint_type=cache.stop_snapshot,cache.joint_type
    import ringfour_agent.stopping as stopping
    original=stopping.verify_joint_stop
    monkeypatch.setattr(stopping,'verify_joint_stop',lambda a,n,p,m:original(a,n,p,m,clock=clock.read,sleep=clock.sleep))
    configure=adapter.configure_controller
    def revoke(name): configure(name);state.estop_engaged=True
    adapter.configure_controller=revoke
    with pytest.raises(RpcError) as exc:manager.start(1,'x',Controller.TRAJECTORY)
    assert exc.value.error_type=='SafetyViolation'
    assert not any(c[0]=='switch' for c in adapter.calls)
    assert lock.owner is None and set(adapter.states)=={'x_idle','y_idle'}
