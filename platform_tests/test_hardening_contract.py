"""Deterministic acceptance evidence for the production admission/lifecycle path.

These cases use a recording adapter, never a connected robot. They establish
only the acceptance behaviors named in each test, not complete v1 conformance.
"""
import asyncio
import json
from types import SimpleNamespace

import pytest

from ringfour.enums import Controller
from ringfour.errors import CleanupFailed, error_from_wire
from ringfour.models import TaskUpdate
from ringfour.aio.tasks import Task
from ringfour_agent.admission import DESCRIPTORS, validate_registry
from ringfour_agent.audit import AuditLog
from ringfour_agent.control import ControlManager
from ringfour_agent.feedback import check_freshness
from ringfour_agent.groups import GroupRuntime
from ringfour_agent.models.profile import GroupConfig
from ringfour_agent.safety import AgentState, MotionLock, SafetyLayer
from ringfour_agent.server import RpcError, Session
from ringfour_agent.tasks import TaskManager

pytestmark = pytest.mark.contract


class RecordingAdapter:
    def __init__(self):
        self.calls = []
        self.fail = None
        self.states = {"x_idle": {"state": "active", "joints": ("hinge",)},
                       "y_idle": {"state": "active", "joints": ("slide",)}}

    def loaded_controllers(self):
        self.calls.append(("readback",))
        if self.fail == "readback": raise RuntimeError("readback stalled")
        return {k: dict(v) for k,v in self.states.items()}

    def load_controller(self, name):
        self.calls.append(("load", name))
        if self.fail == "load": raise RuntimeError("load rejected")
        self.states[name] = {"state": "unconfigured", "joints": ("hinge",) if name.startswith("x") else ("slide",)}

    def configure_controller(self, name):
        self.calls.append(("configure", name))
        if self.fail == "configure": raise RuntimeError("configure rejected")
        self.states[name]["state"] = "inactive"

    def switch_controllers(self, activate, deactivate):
        self.calls.append(("switch", tuple(activate), tuple(deactivate)))
        if self.fail == "restore" and any(n.endswith("idle") for n in activate):
            raise RuntimeError("restoration rejected")
        for name in deactivate: self.states[name]["state"] = "inactive"
        for name in activate: self.states[name]["state"] = "active"

    def unload_controller(self, name):
        self.calls.append(("unload", name))
        if self.fail == "unload": raise RuntimeError("unload rejected")
        del self.states[name]

    def traj_cancel(self):
        self.calls.append(("cancel",))
        if self.fail == "cancel": raise RuntimeError("cancel rejected")


@pytest.fixture
def environment(tmp_path):
    audit = AuditLog("synthetic", None, directory=tmp_path)
    lock = MotionLock()
    state = AgentState()
    safety = SafetyLayer(state, lock)
    adapter = RecordingAdapter()
    groups = {n: GroupRuntime(n, GroupConfig(kind="arm", controllers={"trajectory": n+"_trajectory"}),
                              (joint,), None, "synthetic") for n,joint in [("x","hinge"),("y","slide")]}
    manager = ControlManager(lock, audit, adapter, groups, safety, allow_unverified_stops=True)
    yield manager,lock,state,adapter,audit
    adapter.fail = None
    manager.end_all()
    audit.close()


def test_unclassified_rpc_rejected():
    """Unclassified rpc rejected

    Categories: sdk
    What it checks: Exercises unclassified rpc rejected through production agent components with a recording adapter and explicit fault inputs.
    Why it matters: Rejects forbidden dispatches and preserves ownership, evidence and terminal outcomes under failures.
    If it fails: Inspect admission or lifecycle code; changing a robot profile is not a repair.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline synthetic adapter only; no ROS or connected robot.
    """
    validate_registry({"group.hold": object()})
    with pytest.raises(ValueError, match="Unclassified"):
        validate_registry({"vendor.unknown_move": object()})
    assert DESCRIPTORS["group.start_tool"].effect == "motion"


@pytest.mark.parametrize("method", ["mobility.navigate_to", "teleop.start", "teleop.drive", "group.control_start", "group.start_trajectory", "group.hold", "group.start_tool"])
def test_inhibit_covers_all_motion_paths(environment, method):
    """Inhibit covers all motion paths

    Categories: sdk
    What it checks: Exercises inhibit covers all motion paths through production agent components with a recording adapter and explicit fault inputs.
    Why it matters: Rejects forbidden dispatches and preserves ownership, evidence and terminal outcomes under failures.
    If it fails: Inspect admission or lifecycle code; changing a robot profile is not a repair.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline synthetic adapter only; no ROS or connected robot.
    """
    manager,lock,state,adapter,_ = environment
    state.estop_engaged = True
    with pytest.raises(RpcError, match="e-stop"):
        manager._safety.check(1,method)
    assert not adapter.calls
    manager._safety.check(2,"group.stop")


def test_child_release_retains_other_acquisition(environment):
    """Child release retains other acquisition

    Categories: sdk
    What it checks: Exercises child release retains other acquisition through production agent components with a recording adapter and explicit fault inputs.
    Why it matters: Rejects forbidden dispatches and preserves ownership, evidence and terminal outcomes under failures.
    If it fails: Inspect admission or lifecycle code; changing a robot profile is not a repair.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline synthetic adapter only; no ROS or connected robot.
    """
    manager,lock,_,adapter,_ = environment
    x = manager.start(1,"x",Controller.TRAJECTORY)
    y = manager.start(1,"y",Controller.TRAJECTORY)
    manager.end("x")
    assert lock.owner == 1
    assert manager.session("y",1,y.id) is y
    with pytest.raises(RpcError): manager.start(2,"x",Controller.TRAJECTORY)
    manager.end("y")
    assert lock.owner is None
    manager.start(2,"x",Controller.TRAJECTORY)


def test_partial_acquisition_failure_preserves_existing_owner(environment):
    """Partial acquisition failure preserves existing owner

    Categories: sdk
    What it checks: Exercises partial acquisition failure preserves existing owner through production agent components with a recording adapter and explicit fault inputs.
    Why it matters: Rejects forbidden dispatches and preserves ownership, evidence and terminal outcomes under failures.
    If it fails: Inspect admission or lifecycle code; changing a robot profile is not a repair.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline synthetic adapter only; no ROS or connected robot.
    """
    manager,lock,_,adapter,_ = environment
    y = manager.start(1,"y",Controller.TRAJECTORY)
    adapter.fail = "configure"
    with pytest.raises(RpcError): manager.start(1,"x",Controller.TRAJECTORY)
    assert lock.owner == 1
    assert manager.session("y",1,y.id) is y
    assert "x_trajectory" not in adapter.states
    adapter.fail = None


@pytest.mark.parametrize("failure", ["restore", "unload", "cancel", "readback"])
def test_cleanup_failure_quarantines_across_release(environment, failure):
    """Cleanup failure quarantines across release

    Categories: sdk
    What it checks: Exercises cleanup failure quarantines across release through production agent components with a recording adapter and explicit fault inputs.
    Why it matters: Rejects forbidden dispatches and preserves ownership, evidence and terminal outcomes under failures.
    If it fails: Inspect admission or lifecycle code; changing a robot profile is not a repair.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline synthetic adapter only; no ROS or connected robot.
    """
    manager,lock,_,adapter,_ = environment
    session = manager.start(1,"x",Controller.TRAJECTORY)
    session.running = failure == "cancel"
    adapter.fail = failure
    with pytest.raises(RpcError) as caught: manager.end("x")
    assert caught.value.error_type == "CleanupFailed"
    assert not caught.value.extra["cleanup"]["confirmed"]
    assert "x" in lock.quarantine
    assert lock.owner is None
    lock.release_all(1)
    with pytest.raises(RpcError): manager.start(2,"y",Controller.TRAJECTORY)
    manager._safety.check(2,"group.stop")


def test_stale_session_cannot_command_reopened_group(environment):
    """Stale session cannot command reopened group

    Categories: sdk
    What it checks: Exercises stale session cannot command reopened group through production agent components with a recording adapter and explicit fault inputs.
    Why it matters: Rejects forbidden dispatches and preserves ownership, evidence and terminal outcomes under failures.
    If it fails: Inspect admission or lifecycle code; changing a robot profile is not a repair.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline synthetic adapter only; no ROS or connected robot.
    """
    manager,_,_,_,_ = environment
    old = manager.start(1,"x",Controller.TRAJECTORY)
    manager.end("x")
    current = manager.start(1,"x",Controller.TRAJECTORY)
    assert old.id != current.id
    with pytest.raises(RpcError, match="stale"):
        manager.session("x",1,old.id)


def test_dispatch_rechecks_inhibit_and_generation(environment):
    """Dispatch rechecks inhibit and generation

    Categories: sdk
    What it checks: Exercises dispatch rechecks inhibit and generation through production agent components with a recording adapter and explicit fault inputs.
    Why it matters: Rejects forbidden dispatches and preserves ownership, evidence and terminal outcomes under failures.
    If it fails: Inspect admission or lifecycle code; changing a robot profile is not a repair.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline synthetic adapter only; no ROS or connected robot.
    """
    manager,_,state,adapter,_ = environment
    current = manager.start(1,"x",Controller.TRAJECTORY)
    state.estop_engaged = True
    dispatches = []
    with pytest.raises(RpcError): manager.dispatch(current, lambda: dispatches.append(1))
    assert dispatches == []
    state.estop_engaged = False
    manager.end("x")
    with pytest.raises(RpcError): manager.dispatch(current, lambda: dispatches.append(1))
    assert dispatches == []


def test_shared_adapter_rejects_overlapping_trajectories(environment):
    """Shared adapter rejects overlapping trajectories

    Categories: sdk
    What it checks: Exercises shared adapter rejects overlapping trajectories through production agent components with a recording adapter and explicit fault inputs.
    Why it matters: Rejects forbidden dispatches and preserves ownership, evidence and terminal outcomes under failures.
    If it fails: Inspect admission or lifecycle code; changing a robot profile is not a repair.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline synthetic adapter only; no ROS or connected robot.
    """
    manager,_,_,_,_ = environment
    x = manager.start(1,"x",Controller.TRAJECTORY)
    y = manager.start(1,"y",Controller.TRAJECTORY)
    manager.reserve_trajectory("x",1,x.id)
    with pytest.raises(RpcError) as caught: manager.reserve_trajectory("y",1,y.id)
    assert caught.value.error_type == "ResourceBusy"


@pytest.mark.parametrize("source,received,source_now,receive_now", [(0.,9.9,10.,10.),(9.9,0.,10.,10.),(11.,9.9,10.,10.),(float("nan"),9.9,10.,10.),(None,9.9,10.,10.),(10.,9.,10.,10.)])
def test_joint_freshness_checks_both_clocks(source, received, source_now, receive_now):
    """Joint freshness checks both clocks

    Categories: sdk
    What it checks: Exercises joint freshness checks both clocks through production agent components with a recording adapter and explicit fault inputs.
    Why it matters: Rejects forbidden dispatches and preserves ownership, evidence and terminal outcomes under failures.
    If it fails: Inspect admission or lifecycle code; changing a robot profile is not a repair.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline synthetic adapter only; no ROS or connected robot.
    """
    with pytest.raises(ValueError):
        check_freshness(["hinge"], {"hinge":source}, {"hinge":received}, source_now,receive_now,.5)
    check_freshness(["hinge"], {"hinge":9.9}, {"hinge":9.9}, 10.,10.,.5)


class Socket:
    def __init__(self, drop=False): self.messages=[]; self.drop=drop
    async def send(self, message):
        if self.drop: raise ConnectionError("dropped")
        self.messages.append(json.loads(message))


def test_task_owner_terminal_retention_and_expiry(environment):
    """Task owner terminal retention and expiry

    Categories: sdk
    What it checks: Exercises task owner terminal retention and expiry through production agent components with a recording adapter and explicit fault inputs.
    Why it matters: Rejects forbidden dispatches and preserves ownership, evidence and terminal outcomes under failures.
    If it fails: Inspect admission or lifecycle code; changing a robot profile is not a repair.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline synthetic adapter only; no ROS or connected robot.
    """
    async def exercise():
        _,_,_,_,audit = environment
        now=[0.]
        tasks=TaskManager(audit,clock=lambda:now[0],retention=10.)
        owner=Session(Socket(drop=True)); other=Session(Socket())
        async def body(handle): return {"evidence":"measured"}
        task_id=tasks.start("test",owner,body)
        with pytest.raises(RpcError): await tasks.cancel(task_id,other)
        await tasks._tasks[task_id].aio_task
        result=tasks.status(task_id,owner)
        assert result["state"] == "succeeded" and result["result"]["evidence"] == "measured"
        await tasks.cancel(task_id,owner)
        queried=tasks.status(task_id,owner)
        assert queried["report_age"] >= result["report_age"]
        assert {k:v for k,v in queried.items() if k != "report_age"} == {k:v for k,v in result.items() if k != "report_age"}
        with pytest.raises(RpcError): tasks.status(task_id,other)
        now[0]=10.
        with pytest.raises(RpcError,match="expired"): tasks.status(task_id,owner)
    asyncio.run(exercise())


def test_cancellation_before_first_tick_produces_terminal_record(environment):
    """Cancellation before first tick produces terminal record

    Categories: sdk
    What it checks: Exercises cancellation before first tick produces terminal record through production agent components with a recording adapter and explicit fault inputs.
    Why it matters: Rejects forbidden dispatches and preserves ownership, evidence and terminal outcomes under failures.
    If it fails: Inspect admission or lifecycle code; changing a robot profile is not a repair.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline synthetic adapter only; no ROS or connected robot.
    """
    async def exercise():
        _,_,_,_,audit=environment
        tasks=TaskManager(audit); owner=Session(Socket()); dispatches=[]
        async def body(handle): dispatches.append(1); return {}
        task_id=tasks.start("test",owner,body)
        await tasks.cancel(task_id,owner)
        await tasks._tasks[task_id].aio_task
        assert tasks.status(task_id,owner)["state"] == "cancelled"
        assert dispatches == []
    asyncio.run(exercise())


def test_resource_cancellation_does_not_cancel_unrelated_task(environment):
    """Resource cancellation does not cancel unrelated task

    Categories: sdk
    What it checks: Exercises resource cancellation does not cancel unrelated task through production agent components with a recording adapter and explicit fault inputs.
    Why it matters: Rejects forbidden dispatches and preserves ownership, evidence and terminal outcomes under failures.
    If it fails: Inspect admission or lifecycle code; changing a robot profile is not a repair.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline synthetic adapter only; no ROS or connected robot.
    """
    async def exercise():
        _,_,_,_,audit=environment
        tasks=TaskManager(audit); owner=Session(Socket()); started=asyncio.Event()
        async def body(handle): started.set(); await asyncio.Event().wait()
        x=tasks.start("group.start_trajectory",owner,body,resource="x")
        y=tasks.start("group.start_trajectory",owner,body,resource="y")
        await started.wait()
        assert tasks.cancel_resource("x") == 1
        await tasks.drain_resource("x")
        assert tasks.status(x,owner)["state"] == "cancelled"
        assert tasks.status(y,owner)["state"] == "running"
        await tasks.cancel_all()
    asyncio.run(exercise())


def test_wire_errors_preserve_evidence_and_unknown_type():
    """Wire errors preserve evidence and unknown type

    Categories: sdk
    What it checks: Exercises wire errors preserve evidence and unknown type through production agent components with a recording adapter and explicit fault inputs.
    Why it matters: Rejects forbidden dispatches and preserves ownership, evidence and terminal outcomes under failures.
    If it fails: Inspect admission or lifecycle code; changing a robot profile is not a repair.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline synthetic adapter only; no ROS or connected robot.
    """
    evidence={"type":"CleanupFailed","cleanup":{"steps":[{"status":"failed"}]},"residual":{"hinge":.4,"slide":.02}}
    error=error_from_wire("cleanup failed",evidence)
    assert isinstance(error,CleanupFailed) and error.details == evidence
    unknown=error_from_wire("adapter failed",{"type":"VendorSpecificFailure","native_code":17})
    assert unknown.wire_type == "VendorSpecificFailure" and unknown.details["native_code"] == 17


def test_late_updates_cannot_rewrite_terminal_task():
    """Late updates cannot rewrite terminal task

    Categories: sdk
    What it checks: Exercises late updates cannot rewrite terminal task through production agent components with a recording adapter and explicit fault inputs.
    Why it matters: Rejects forbidden dispatches and preserves ownership, evidence and terminal outcomes under failures.
    If it fails: Inspect admission or lifecycle code; changing a robot profile is not a repair.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline synthetic adapter only; no ROS or connected robot.
    """
    async def exercise():
        task=Task(SimpleNamespace(),"test")
        task._apply(TaskUpdate(task_id="test",state="succeeded",result={"x":1}))
        task._apply(TaskUpdate(task_id="test",state="cancelled"))
        task._apply(TaskUpdate(task_id="test",state="running"))
        assert task.state == "succeeded" and task.result == {"x":1}
    asyncio.run(exercise())

@pytest.mark.protocol
@pytest.mark.parametrize('fault', ['missing', 'nonfinite', 'stale', 'residual', None])
def test_protocol_arrival_requires_complete_fresh_evidence(environment, tmp_path, fault):
    """A07/A08/A18: arrival evidence survives the actual protocol

    Categories: sdk
    What it checks: Sends a trajectory through a real Unix WebSocket client and production dispatcher; missing, stale, non-finite and out-of-tolerance feedback cannot produce success. Queries the retained terminal result.
    Why it matters: Controller success alone is not measured joint arrival.
    If it fails: Inspect agent completion checks and wire evidence preservation.
    Profile fields: groups.*.feedback_max_age;groups.*.goal_tolerance
    Scenario fields: None.
    Robot effects: Isolated protocol and recording adapter only; no ROS.
    """
    from ringfour import aio
    from ringfour_agent.bootstrap import build_agent
    from ringfour_agent.handlers import build_registry
    from ringfour_agent.profile import Profile, load_profile
    from .conftest import STUB_PROFILE
    async def exercise():
        manager,lock,state,adapter,audit = environment
        profile=load_profile(STUB_PROFILE)
        profile=Profile(spec=profile.spec.model_copy(update={'groups':{n:g.config.model_copy(update={'joints':g.joints}) for n,g in manager._groups.items()}}),info=profile.info)
        server,ctx=build_agent(profile,unix_path=str(tmp_path/'protocol.sock'),audit_dir=tmp_path/'wire-audit', allow_unverified_stops=True)
        # Production manager, dispatcher, serializer and client; only the physical adapter is replaced.
        ctx.groups=manager._groups;ctx.control=manager;ctx.lock=lock;ctx.state=state;ctx.safety=manager._safety;ctx.ros=adapter
        snapshot={'hinge':(0.,0.,0.),'slide':(0.,0.,0.)}
        adapter.joint_snapshot=lambda:dict(snapshot)
        adapter.joint_limits=lambda:{'hinge':(-1.,1.),'slide':(0.,1.)}
        adapter.description_sha=lambda:'fixture-revision'
        adapter.joint_type=lambda name:'revolute' if name=='hinge' else 'prismatic'
        def feedback(names,age):
            if fault=='stale':raise ValueError('source is stale')
            result=dict(snapshot)
            if fault=='missing':result.pop('hinge',None)
            if fault=='nonfinite':result['hinge']=(float('nan'),0.,0.)
            if fault=='residual':result['hinge']=(.8,0.,0.)
            return result
        adapter.verified_joint_snapshot=feedback
        def follow(controller,joints,points):
            adapter.calls.append(('dispatch',controller))
            for name,value in zip(joints,points[-1][0]):snapshot[name]=(value,0.,0.)
        adapter.follow_trajectory=follow
        adapter.traj_wait=lambda poll:{'status':'succeeded'}
        server._registry=build_registry(ctx)
        await server.start()
        try:
            robot=await aio.Robot.connect('unix://'+str(tmp_path/'protocol.sock'))
            try:
                async with robot.groups['x'].control(Controller.TRAJECTORY) as control:
                    from ringfour.models import JointValue,JointConfiguration,JointTrajectory
                    config=JointConfiguration(robot=robot.name,group='x',model_revision=adapter.description_sha(),
                        joints=(JointValue(name='hinge',position=.2,unit='rad',lower=-1.,upper=1.),))
                    trajectory=JointTrajectory.from_waypoints([config],step_duration=.1)
                    task=await robot._tasks.start_task('group.start_trajectory',{'group':'x','session_id':control._session_id,'trajectory':trajectory.model_dump(mode='json')})
                    if fault:
                        from ringfour.errors import ExecutionFailed
                        with pytest.raises(ExecutionFailed):
                            await task.wait()
                        update=await task.get_status()
                        assert update.state=='failed'
                        if fault=='residual':assert update.error.data['residual']['hinge']==pytest.approx(.6)
                    else:
                        await task.wait()
                        update=await task.get_status()
                        assert update.state=='succeeded' and update.result['arrived'] is True
                assert sum(c[0]=='dispatch' for c in adapter.calls)==1
            finally:await robot.close()
        finally:
            await server.close();ctx.audit.close()
    asyncio.run(exercise())

@pytest.mark.parametrize('facade', ['sync', 'async'])
def test_retained_iterator_break_does_not_cancel_until_scope_exit(facade):
    """A05/A06/A20: scope defines cancellation in both facades

    Categories: sdk
    What it checks: Retains an iterator after one update and break; no cancellation occurs until explicit scope exit. Uses the production sync trajectory facade and async scope cleanup.
    Why it matters: Garbage collection must not determine robot execution lifetime.
    If it fails: Inspect iterator finalization and explicit cleanup.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline synthetic operation only.
    """
    from ringfour.aio.operations import Operation as AsyncOperation
    from ringfour.operations import Operation
    from ringfour.aio.tasks import Task
    from ringfour.models import TaskUpdate
    from ringfour.runner import Runner
    class Connection:
        async def call(self,method,params):
            task._apply(TaskUpdate(task_id='fixture',state='cancelled',cleanup={
                'verified':True,'criterion':'not-dispatched','physical_stop':'unverified'}))
            return {'accepted':True,'already_terminal':False}
    task=Task(Connection(),'fixture')
    operation=AsyncOperation(task,lambda i,r:r,lambda t:None,resource='fixture')
    if facade=='sync':
        runner=Runner()
        try:
            with Operation(runner,operation) as op:
                retained=op.updates()
                for update in retained:break
                assert not task.done
            assert task.done
            retained.close()
        finally:runner.stop()
    else:
        async def exercise():
            async with operation:
                retained=operation.updates()
                await anext(retained)
                assert not task.done
            assert task.done
            await retained.aclose()
        asyncio.run(exercise())



def test_cleanup_failure_preserves_body_exception():
    """A21: mission and cleanup failures remain inspectable

    Categories: sdk
    What it checks: Raises a body failure and a cleanup failure, then inspects the original exception and attached cleanup report. Without a body failure, cleanup raises normally.
    Why it matters: A cleanup exception must not discard the original mission failure.
    If it fails: Inspect shared scope exit handling.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline synthetic cleanup only.
    """
    from ringfour.lifecycle import exit_scope
    async def exercise():
        body=ValueError('mission failed')
        async def close():raise CleanupFailed('restore failed',details={'restore':'failed'})
        await exit_scope(close,(ValueError,body,None))
        assert isinstance(body.cleanup_error,CleanupFailed)
        assert body.cleanup_error.details['restore']=='failed'
        assert body.__notes__
        with pytest.raises(CleanupFailed):await exit_scope(close,(None,None,None))
    asyncio.run(exercise())


def test_caller_cancellation_waits_for_cleanup():
    """A06/A20: async cancellation protects finite cleanup

    Categories: sdk
    What it checks: Cancels a scope exit while its cleanup waits on an explicit event, then releases cleanup and verifies cancellation propagates only after completion.
    Why it matters: Python cancellation must not silently abandon remote teardown.
    If it fails: Inspect cancellation shielding and cleanup deadlines.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline synthetic cleanup only.
    """
    from ringfour.lifecycle import exit_scope
    async def exercise():
        started=asyncio.Event();release=asyncio.Event();finished=[]
        async def close():started.set();await release.wait();finished.append(True)
        exiting=asyncio.create_task(exit_scope(close,(None,None,None)))
        await started.wait();exiting.cancel()
        await asyncio.sleep(0)
        assert not exiting.done()
        release.set()
        with pytest.raises(asyncio.CancelledError):await exiting
        assert finished==[True]
    asyncio.run(exercise())


def test_control_opener_is_lazy_and_explicit_open_is_idempotent():
    """A27: constructing a control scope does not acquire authority

    Categories: sdk
    What it checks: Creates a production async session opener without producing a coroutine or RPC, then explicitly opens it twice and observes one acquisition.
    Why it matters: A discarded scope must not leak motion ownership or controller changes.
    If it fails: Inspect lazy control acquisition.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline recording opener only.
    """
    from ringfour.aio.sensing import _SessionOpener
    calls=[]
    async def open_session():calls.append('acquire');return object()
    opener=_SessionOpener(open_session)
    assert not calls
    async def exercise():
        first=await opener.open();second=await opener.open()
        assert first is second
    asyncio.run(exercise())
    assert calls==['acquire']


def test_competing_acquisitions_are_serialized(environment):
    """A23: racing acquisitions have one consistent owner

    Categories: sdk
    What it checks: Starts two competing acquisitions concurrently in worker threads and checks one owner, one successful lifecycle switch and one rejected client.
    Why it matters: A check-then-use race must not let two clients drive the same resource.
    If it fails: Inspect controller lifecycle serialization and acquisition accounting.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline recording adapter only.
    """
    from concurrent.futures import ThreadPoolExecutor
    from threading import Barrier
    manager,lock,_,adapter,_=environment
    barrier=Barrier(2)
    def acquire(client):
        barrier.wait()
        try:return manager.start(client,'x',Controller.TRAJECTORY).client_id
        except RpcError:return None
    with ThreadPoolExecutor(max_workers=2) as pool:
        results=list(pool.map(acquire,[1,2]))
    winners=[r for r in results if r is not None]
    assert winners == [lock.owner]
    assert sum(c[0]=='switch' for c in adapter.calls)==1


def test_sync_control_opener_uses_lazy_async_opener():
    """A20/A27: synchronous control uses the same lazy acquisition

    Categories: sdk
    What it checks: Constructs and discards a sync opener without acquiring, then opens twice and verifies one acquisition and one scoped cleanup through the production facade.
    Why it matters: Sync and async must share the same acquisition contract.
    If it fails: Inspect sync Arm.control and SessionOpener.open.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline recording opener only.
    """
    from ringfour.sensing import Arm
    from ringfour.aio.sensing import _SessionOpener
    from ringfour.runner import Runner
    calls=[]
    class Scope:
        async def __aexit__(self,*exc):calls.append('cleanup')
    async def open_session():calls.append('acquire');return Scope()
    runner=Runner()
    group=Arm.__new__(Arm);group._runner=runner
    group._aio=SimpleNamespace(_control=lambda *a,**k:_SessionOpener(open_session))
    try:
        unused=group.control()
        assert calls==[]
        with unused:
            unused.open()
            assert calls==['acquire']
        assert calls==['acquire','cleanup']
    finally:runner.stop()


@pytest.mark.parametrize('values,dt', [([[True]],1.),([[float('nan')]],1.),([['.2']],1.),([[.2]],0.),([[.2]],float('inf')),([],1.)])
def test_sdk_trajectory_rejects_invalid_values_before_submission(values,dt):
    """A22: SDK validation does not coerce invalid motion intent

    Categories: sdk
    What it checks: Constructs trajectories with Boolean, string, non-finite, empty or invalid-duration inputs and verifies rejection before any task submission.
    Why it matters: Automatic numeric coercion must not turn invalid intent into a motion command.
    If it fails: Inspect the trajectory numeric boundary.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline fake connection; zero submissions.
    """
    from ringfour.models import JointValue,JointConfiguration,JointTrajectory
    from pydantic import ValidationError
    with pytest.raises((ValueError,ValidationError)):
        configs=[JointConfiguration(robot='fixture',group='x',model_revision='v1',
            joints=(JointValue(name='joint',position=row[0],unit='rad'),)) for row in values]
        JointTrajectory.from_waypoints(configs,step_duration=dt)



def test_stale_close_cannot_release_replacement_scope(environment):
    """A13/A23: stale release is atomic with replacement acquisition

    Categories: sdk
    What it checks: Closes one session, opens a replacement, then retries the old close and verifies the replacement and its ownership remain intact.
    Why it matters: A check-then-use release must not close a new generation's scope.
    If it fails: Inspect agent end-session identity validation and retained close records.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline recording adapter only.
    """
    manager,lock,_,_,_=environment
    old=manager.start(1,'x',Controller.TRAJECTORY)
    manager.end('x',client_id=1,session_id=old.id)
    replacement=manager.start(2,'x',Controller.TRAJECTORY)
    manager.end('x',client_id=1,session_id=old.id)
    assert manager.session('x',2,replacement.id) is replacement
    assert lock.owner==2


def test_repeated_cancel_does_not_interrupt_cleanup(environment):
    """A06/A23: repeated cancellation preserves in-flight cleanup

    Categories: sdk
    What it checks: Cancels a running task twice while its cleanup waits on an explicit event and verifies cleanup finishes once and yields one terminal cancellation.
    Why it matters: A second cancel must not interrupt the first cancellation's stop path.
    If it fails: Inspect cancellation request accounting.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline synthetic task only.
    """
    async def exercise():
        _,_,_,_,audit=environment;tasks=TaskManager(audit);owner=Session(Socket())
        started=asyncio.Event();cleaning=asyncio.Event();release=asyncio.Event();finished=[]
        async def body(handle):
            started.set()
            try:await asyncio.Event().wait()
            except asyncio.CancelledError:
                cleaning.set();await release.wait();finished.append(True);raise
        key=tasks.start('test',owner,body)
        await started.wait();await tasks.cancel(key,owner);await cleaning.wait()
        await tasks.cancel(key,owner);release.set()
        await tasks._tasks[key].aio_task
        assert finished==[True] and tasks.status(key,owner)['state']=='cancelled'
    asyncio.run(exercise())


@pytest.mark.parametrize('query_supported',[False,True])
def test_connection_loss_does_not_fabricate_terminal_state(query_supported):
    """A12/A18: lost transport means unknown execution, not completion

    Categories: sdk
    What it checks: Loses a connection while a task is running and verifies wait raises ConnectionLost while the last state stays running and done remains false.
    Why it matters: An observer's loss of knowledge cannot establish a physical outcome.
    If it fails: Inspect task transport failure handling.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline synthetic connection loss only.
    """
    from ringfour.errors import ConnectionLost
    async def exercise():
        task=Task(SimpleNamespace(),'test',query_supported=query_supported)
        task._apply(TaskUpdate(task_id='test',state='running'))
        task._fail(ConnectionLost('disconnected'))
        with pytest.raises(ConnectionLost):await task.wait()
        assert task.state=='running' and not task.done
    asyncio.run(exercise())


def test_late_agent_report_cannot_rewrite_terminal_record(environment):
    """A18/A23: authoritative terminal outcome cannot become running again

    Categories: sdk
    What it checks: Retains a task body's reporting handle after success, sends a late progress report and verifies the authoritative terminal query stays succeeded.
    Why it matters: Background feedback must not resurrect completed operations.
    If it fails: Inspect task terminal record immutability.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline synthetic task only.
    """
    async def exercise():
        _,_,_,_,audit=environment;tasks=TaskManager(audit);owner=Session(Socket());handles=[]
        async def body(handle):handles.append(handle);return {'verified':True}
        key=tasks.start('test',owner,body)
        await tasks._tasks[key].aio_task
        await handles[0].report(progress=.2)
        assert tasks.status(key,owner)['state']=='succeeded'
    asyncio.run(exercise())


@pytest.mark.parametrize('phase', ['load', 'configure', 'switch'])
def test_partial_controller_effect_then_lost_reply_restores(environment, phase):
    """A04/A09: lost controller acknowledgement still unwinds effects

    Categories: sdk
    What it checks: Applies load, configure or switch and then raises before acknowledgement; verifies prior controllers are restored and another child remains owned.
    Why it matters: A failed service reply does not prove that its controller change never happened.
    If it fails: Repair acquisition rollback bookkeeping, not profile tolerances.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline recording adapter only.
    """
    manager, lock, _, adapter, _ = environment
    earlier = manager.start(1, 'y', Controller.TRAJECTORY)
    if phase == 'configure':
        adapter.states['x_trajectory'] = {'state': 'unconfigured', 'joints': ('hinge',)}
    before = {k: dict(v) for k, v in adapter.states.items()}
    method = {'load': 'load_controller', 'configure': 'configure_controller', 'switch': 'switch_controllers'}[phase]
    original = getattr(adapter, method)
    failed = False
    def lose_reply(*args, **kwargs):
        nonlocal failed
        result = original(*args, **kwargs)
        if not failed:
            failed = True
            raise RuntimeError('effect applied, acknowledgement lost')
        return result
    setattr(adapter, method, lose_reply)
    with pytest.raises(RpcError) as error:
        manager.start(1, 'x', Controller.TRAJECTORY)
    assert error.value.extra['cleanup']['confirmed']
    assert adapter.states == before
    assert manager.session('y', 1, earlier.id) is earlier
    assert lock.owner == 1 and not lock.quarantine


@pytest.mark.parametrize('verification_fails', [False, True])
def test_cancel_during_stop_worker_retains_verification_outcome(environment, verification_fails):
    """Cancellation joins a stop worker and retains its outcome

    Categories: sdk
    What it checks: Cancels an accepted dispatched task while a stop-verification thread is blocked, then releases it with either measured evidence or failure.
    Why it matters: Cancelling a wait cannot stop its worker or turn an unverified stop into ordinary cancellation.
    If it fails: Join the stop worker before recording terminal state and preserve its failure.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline recording task and event-controlled worker only.
    """
    import threading
    async def exercise():
        _, _, _, _, audit = environment
        tasks = TaskManager(audit)
        owner = Session(Socket())
        entered, release = threading.Event(), threading.Event()
        evidence = {'verified': True, 'criterion': 'synthetic-rest', 'physical_stop': 'uncommissioned'}
        def verify():
            entered.set()
            assert release.wait(2), 'test did not release verifier'
            if verification_fails:
                raise RpcError('StopUnverified', 'injected stale feedback', {'reason': 'stale feedback'})
            return evidence
        async def body(handle):
            handle.mark_dispatched()
            await handle.verify_stop(verify)
            return {}
        key = tasks.start('synthetic-motion', owner, body)
        try:
            assert await asyncio.to_thread(entered.wait, 1)
            await tasks.cancel(key, owner)
            await asyncio.sleep(0)
            assert not tasks._tasks[key].aio_task.done()
            assert tasks.status(key, owner)['state'] == 'running'
        finally:
            release.set()
        await tasks._tasks[key].aio_task
        status = tasks.status(key, owner)
        if verification_fails:
            assert status['state'] == 'failed'
            assert status['error']['type'] == 'StopUnverified'
            assert status['error']['data']['reason'] == 'stale feedback'
        else:
            assert status['state'] == 'cancelled' and status['cleanup'] == evidence
    asyncio.run(exercise())
