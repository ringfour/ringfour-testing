"""B guard/measurement acceptance with a deterministic source-clock adapter."""
import asyncio
import threading
import time
from types import SimpleNamespace
import pytest
from ringfour.models.measurements import SignalCondition
from ringfour_agent.guards import GuardRegistry
from ringfour_agent.safety import AgentState,MotionLock,SafetyLayer
from ringfour_agent.measurements import measure_signal
from ringfour_agent.server import RpcError

class Audit:
    def record(self,*args,**kwargs):pass
class Tasks:
    def cancel_running(self):pass
    async def cancel_all(self):pass

def fixture():
    ros=SimpleNamespace(_lock=threading.RLock(),description_sha=lambda:'model-v1')
    ros.value=80.;ros.source_age=.01;ros.receive_age=.01
    ros.signal_provenance=lambda path:{'source_stamp':10.-ros.source_age,'source_now':10.,
        'received':time.monotonic()-ros.receive_age,'clock_domain':'ros'}
    state=AgentState();lock=MotionLock()
    ctx=SimpleNamespace(ros=ros,watch_paths={'battery.level':lambda:ros.value},
        profile=SimpleNamespace(info=SimpleNamespace(name='fixture')),state=state,lock=lock,
        audit=Audit(),events=SimpleNamespace(emit=lambda *args:None),tasks=Tasks(),
        background_halts=set(),teleop=None,actions=None,control=None)
    ctx.guards=GuardRegistry(ctx)
    return ctx

def condition():return SignalCondition(robot_id='fixture',path='battery.level',comparison='below',threshold=10,max_age=.2)

def test_guard_initial_freshness_trip_reset_and_other_inhibits():
    """Persistent guard trips and resets independently

    Categories: sdk
    What it checks: Entry rejects unsafe/stale/unknown signals; trip blocks new motion; reset requires fresh safe data; removal preserves e-stop and other guards.
    Why it matters: A threshold notification must enforce a persistent motion inhibit.
    If it fails: Repair guard admission, freshness or cleanup lifecycle.
    Profile fields: Discovered signals.
    Scenario fields: None.
    Robot effects: Offline only.
    """
    async def exercise():
        ctx=fixture();registry=ctx.guards;safety=SafetyLayer(ctx.state,ctx.lock)
        for value,age in ((5.,.01),(80.,1.)):
            ctx.ros.value=value;ctx.ros.source_age=age
            with pytest.raises(RpcError):await registry.install(1,condition())
        ctx.ros.value=80.;ctx.ros.source_age=.01
        first=await registry.install(1,condition());second=await registry.install(1,condition())
        with pytest.raises(RpcError):registry.status(first.guard_id,2)
        ctx.ros.value=5.
        await registry.evaluate()
        assert len(ctx.state.guard_inhibits)==2
        with pytest.raises(RpcError):safety.check_motion(1,'test')
        with pytest.raises(RpcError):await registry.reset(first.guard_id,1)
        ctx.ros.value=80.
        await registry.reset(first.guard_id,1)
        assert second.guard_id in ctx.state.guard_inhibits
        ctx.state.estop_engaged=True
        await registry.remove(first.guard_id,1)
        assert ctx.state.estop_engaged and second.guard_id in ctx.state.guard_inhibits
        await registry.remove(second.guard_id,1)
        assert ctx.state.estop_engaged and not ctx.state.guard_inhibits
    asyncio.run(exercise())

def test_replayed_unknown_and_nonfinite_signal_provenance():
    """Fresh receipt does not conceal old or unknown source data

    Categories: sdk
    What it checks: New receipt with old source remains stale; unknown source and nonfinite signals stay unavailable/invalid.
    Why it matters: Fresh transport activity does not prove fresh physical state.
    If it fails: Repair measurement freshness validation.
    Profile fields: Discovered signals.
    Scenario fields: None.
    Robot effects: Offline only.
    """
    ctx=fixture();ctx.ros.source_age=1.
    sample=measure_signal(ctx,'battery.level',.2)
    assert sample.validity=='stale' and sample.receive_age<.2 and sample.source_age>.2
    ctx.ros.signal_provenance=lambda path:{'received':time.monotonic(),'source_stamp':None}
    sample=measure_signal(ctx,'battery.level',.2)
    assert sample.source_age is None and sample.validity=='unavailable'
    ctx.ros.value=float('nan')
    assert measure_signal(ctx,'battery.level',.2).validity=='invalid'


def test_guard_stale_trip_failed_cleanup_retry_and_disconnect():
    """Guard disappearance and failed cleanup retain inhibits

    Categories: sdk
    What it checks: Source staleness trips an active guard; cleanup failure retains its inhibit; disconnect cannot erase quarantine; explicit removal can retry after recovery.
    Why it matters: A guard must survive failures until covered work has verified cleanup.
    If it fails: Repair guard/drain/quarantine accounting.
    Profile fields: Discovered signals.
    Scenario fields: None.
    Robot effects: Offline only.
    """
    async def exercise():
        ctx=fixture();status=await ctx.guards.install(1,condition())
        ctx.ros.source_age=1.
        ctx.lock.quarantine['fixture']={'reason':'unverified stop'}
        await ctx.guards.evaluate()
        record=ctx.guards.records[status.guard_id]
        with pytest.raises(RpcError):await record['cleanup']
        ctx.guards.disconnected_after_cleanup(1)
        assert status.guard_id in ctx.state.guard_inhibits
        with pytest.raises(RpcError):await ctx.guards.remove(status.guard_id,1)
        assert status.guard_id in ctx.state.guard_inhibits
        # Emulates authorized verified resource recovery, not guard removal.
        ctx.lock.quarantine.clear();ctx.ros.source_age=.01
        assert (await ctx.guards.remove(status.guard_id,1))['cleanup_confirmed']
        assert not ctx.state.guard_inhibits
    asyncio.run(exercise())


def test_profile_installed_guard_is_not_owned_by_a_client():
    """Integrator policy guard cannot be removed by a mission

    Categories: sdk
    What it checks: A profile-installed guard has no client owner; ordinary sessions cannot reset or remove it.
    Why it matters: Client scope cleanup must not erase integrator-required policy.
    If it fails: Repair guard ownership checks.
    Profile fields: Integrator policy installation.
    Scenario fields: None.
    Robot effects: Offline only.
    """
    async def exercise():
        ctx=fixture();status=await ctx.guards.install_profile(condition())
        for owner in (0,1,2):
            with pytest.raises(RpcError):await ctx.guards.remove(status.guard_id,owner)
            with pytest.raises(RpcError):await ctx.guards.reset(status.guard_id,owner)
        ctx.guards.disconnected_after_cleanup(1)
        assert status.guard_id in ctx.guards.records
    asyncio.run(exercise())


def test_slow_guard_reader_trips_without_blocking_the_event_loop():
    """Slow acquisition fails closed within the declared age budget

    Categories: sdk
    What it checks: A delayed read expires its guard observation budget while unrelated event-loop work remains responsive and motion inhibition latches.
    Why it matters: A slow adapter cannot suspend guard enforcement.
    If it fails: Repair bounded guard observation and scheduling.
    Profile fields: Guard max_age.
    Scenario fields: None.
    Robot effects: Offline only.
    """
    async def exercise():
        ctx=fixture();settings=condition().model_copy(update={'max_age':.03})
        status=await ctx.guards.install(1,settings)
        original=ctx.guards.observe
        def delayed(condition):time.sleep(.1);return original(condition)
        ctx.guards.observe=delayed
        ticks=[]
        async def heartbeat():
            for _ in range(4):ticks.append(time.monotonic());await asyncio.sleep(.005)
        await asyncio.gather(ctx.guards.evaluate(),heartbeat())
        assert len(ticks)==4 and status.guard_id in ctx.state.guard_inhibits
        assert ctx.guards.records[status.guard_id]['observation'].validity=='unavailable'
        await ctx.guards.records[status.guard_id]['cleanup']
    asyncio.run(exercise())
