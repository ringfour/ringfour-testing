"""Quarantine survives reboot and clears only through measured local recovery."""
import json
from types import SimpleNamespace
import pytest
from ringfour_agent.durability import DurableQuarantine
from ringfour_agent.recovery import recover_mobility
from ringfour_agent.safety import AgentState, MotionLock
from ringfour_agent.server import RpcError
from ringfour_agent.profile import load_profile
from .conftest import STUB_PROFILE
from .test_motion_stop_contract import BodyAdapter, configured
from .test_stop_recovery_contract import Clock, POLICY

pytestmark=pytest.mark.contract


def test_quarantine_survives_clean_restart_and_keeps_model_identity(tmp_path):
    """Quarantine survives clean restart and keeps model identity

    Categories: sdk
    What it checks: Exercises synthetic durable checkpoints or measured mobility recovery with explicit faults and a deterministic clock.
    Why it matters: Restart or operator recovery must not erase unresolved stopping failures or resume accepted work.
    If it fails: Repair atomic checkpointing or recovery eligibility; never clear the safety record to obtain a pass.
    Profile fields: safety.stop_verification
    Scenario fields: None.
    Robot effects: Offline files, fake clocks and recording adapters only.
    """
    first=DurableQuarantine(tmp_path,'synthetic-robot','revision-A')
    first['mobility']={'reason':'recorded failed stop'}
    first.finish_boot()
    second=DurableQuarantine(tmp_path,'synthetic-robot','revision-A')
    assert second['mobility']=={'reason':'recorded failed stop'} and 'restart' not in second
    second.finish_boot()
    changed=DurableQuarantine(tmp_path,'synthetic-robot','revision-B')
    assert 'mobility' in changed and 'restart-model' in changed
    changed.finish_boot()


def test_unclean_boot_cannot_restart_into_motion_eligibility(tmp_path):
    """Unclean boot cannot restart into motion eligibility

    Categories: sdk
    What it checks: Exercises synthetic durable checkpoints or measured mobility recovery with explicit faults and a deterministic clock.
    Why it matters: Restart or operator recovery must not erase unresolved stopping failures or resume accepted work.
    If it fails: Repair atomic checkpointing or recovery eligibility; never clear the safety record to obtain a pass.
    Profile fields: safety.stop_verification
    Scenario fields: None.
    Robot effects: Offline files, fake clocks and recording adapters only.
    """
    first=DurableQuarantine(tmp_path,'synthetic-robot','revision-A')
    path=first.path;first.finish_boot()
    data=json.loads(path.read_text());data['active_boot']=True;path.write_text(json.dumps(data))
    after=DurableQuarantine(tmp_path,'synthetic-robot','revision-A')
    assert 'restart' in after
    lock=MotionLock(quarantine=after)
    with pytest.raises(RpcError):lock.acquire(1,'scope')
    after.finish_boot()


def test_checkpoint_failure_cannot_clear_quarantine(tmp_path,monkeypatch):
    """Checkpoint failure cannot clear quarantine

    Categories: sdk
    What it checks: Exercises synthetic durable checkpoints or measured mobility recovery with explicit faults and a deterministic clock.
    Why it matters: Restart or operator recovery must not erase unresolved stopping failures or resume accepted work.
    If it fails: Repair atomic checkpointing or recovery eligibility; never clear the safety record to obtain a pass.
    Profile fields: safety.stop_verification
    Scenario fields: None.
    Robot effects: Offline files, fake clocks and recording adapters only.
    """
    checkpoint=DurableQuarantine(tmp_path,'synthetic-robot','revision-A')
    checkpoint['mobility']={'reason':'retained'}
    def fail(values):raise RpcError('CleanupFailed','injected storage failure')
    monkeypatch.setattr(checkpoint,'_persist',fail)
    with pytest.raises(RpcError):del checkpoint['mobility']
    assert 'mobility' in checkpoint
    with pytest.raises(RpcError):checkpoint['head']={'reason':'new failure'}
    assert 'head' in checkpoint


def test_corrupt_checkpoint_refuses_startup(tmp_path):
    """Corrupt checkpoint refuses startup

    Categories: sdk
    What it checks: Exercises synthetic durable checkpoints or measured mobility recovery with explicit faults and a deterministic clock.
    Why it matters: Restart or operator recovery must not erase unresolved stopping failures or resume accepted work.
    If it fails: Repair atomic checkpointing or recovery eligibility; never clear the safety record to obtain a pass.
    Profile fields: safety.stop_verification
    Scenario fields: None.
    Robot effects: Offline files, fake clocks and recording adapters only.
    """
    checkpoint=DurableQuarantine(tmp_path,'synthetic-robot','revision-A');path=checkpoint.path;checkpoint.finish_boot()
    path.write_text('{invalid')
    with pytest.raises(RpcError):DurableQuarantine(tmp_path,'synthetic-robot','revision-A')


@pytest.mark.parametrize('fault',[None,'vertical','frozen','nonfinite'])
def test_mobility_recovery_requires_real_measured_rest(monkeypatch,fault):
    """Mobility recovery requires real measured rest

    Categories: sdk
    What it checks: Exercises synthetic durable checkpoints or measured mobility recovery with explicit faults and a deterministic clock.
    Why it matters: Restart or operator recovery must not erase unresolved stopping failures or resume accepted work.
    If it fails: Repair atomic checkpointing or recovery eligibility; never clear the safety record to obtain a pass.
    Profile fields: safety.stop_verification
    Scenario fields: None.
    Robot effects: Offline files, fake clocks and recording adapters only.
    """
    clock=Clock();adapter=BodyAdapter(clock,fault)
    adapter.nav_cancel=lambda:adapter.calls.append('cancel')
    lock=MotionLock(quarantine={'mobility':{'reason':'retained failed stop'},'other':{'reason':'independent'}})
    ctx=SimpleNamespace(lock=lock,state=AgentState(estop_engaged=True),tasks=SimpleNamespace(has_running=lambda:False),
                        ros=adapter,profile=configured(load_profile(STUB_PROFILE)))
    import ringfour_agent.recovery as recovery
    original=recovery.verify_mobility_stop
    monkeypatch.setattr(recovery,'verify_mobility_stop',lambda a,p,m:original(a,p,m,clock=clock.read,sleep=clock.sleep))
    if fault:
        with pytest.raises(RpcError):recover_mobility(ctx)
        assert 'mobility' in lock.quarantine
    else:
        result=recover_mobility(ctx)
        assert result['stop_verification']['verified'] and result['inhibit_engaged']
        assert 'mobility' not in lock.quarantine
    assert ctx.state.estop_engaged and 'other' in lock.quarantine


@pytest.mark.parametrize('inhibited,owner,running',[(False,None,False),(True,1,False),(True,None,True)])
def test_mobility_recovery_checks_exclusive_inhibited_state(inhibited,owner,running):
    """Mobility recovery checks exclusive inhibited state

    Categories: sdk
    What it checks: Exercises synthetic durable checkpoints or measured mobility recovery with explicit faults and a deterministic clock.
    Why it matters: Restart or operator recovery must not erase unresolved stopping failures or resume accepted work.
    If it fails: Repair atomic checkpointing or recovery eligibility; never clear the safety record to obtain a pass.
    Profile fields: safety.stop_verification
    Scenario fields: None.
    Robot effects: Offline files, fake clocks and recording adapters only.
    """
    adapter=BodyAdapter(Clock());adapter.nav_cancel=lambda:adapter.calls.append('cancel')
    ctx=SimpleNamespace(lock=MotionLock(owner=owner,quarantine={'mobility':{'reason':'retained'}}),
        state=AgentState(estop_engaged=inhibited),tasks=SimpleNamespace(has_running=lambda:running),
        ros=adapter,profile=configured(load_profile(STUB_PROFILE)))
    with pytest.raises(RpcError):recover_mobility(ctx)
    assert adapter.calls==[] and 'mobility' in ctx.lock.quarantine


def test_second_agent_cannot_share_the_same_safety_checkpoint(tmp_path):
    """One live agent owns a durable robot checkpoint

    Categories: sdk
    What it checks: Opens a durable robot state twice and requires the second process-level lock acquisition to fail before any checkpoint update.
    Why it matters: Two agent boots must not overwrite each other's quarantine or shutdown marker.
    If it fails: Repair exclusive checkpoint ownership; do not delete the active lock file.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline temporary files only.
    """
    first=DurableQuarantine(tmp_path,'synthetic-robot','revision-A')
    with pytest.raises(RpcError):DurableQuarantine(tmp_path,'synthetic-robot','revision-A')
    first.finish_boot()


@pytest.mark.parametrize('resource', ['restart', 'journal'])
@pytest.mark.parametrize('fault', [None, 'missing', 'opaque', 'baseline', 'cancel', 'moving', 'changed'])
def test_restart_recovery_requires_reviewed_baseline_and_measured_rest(monkeypatch, fault, resource):
    """Restart recovery cancels orphan goals and verifies rest

    Categories: sdk
    What it checks: Injects controller mismatch, cancellation failure, moving feedback and a changed baseline into explicit restart recovery.
    Why it matters: An unclean boot must not erase unresolved motion or unrelated quarantine.
    If it fails: Repair recovery eligibility and verification before enabling motion.
    Profile fields: safety.restart_recovery_controllers, safety.stop_verification
    Scenario fields: None.
    Robot effects: Recording adapters and a deterministic clock only.
    """
    import ringfour_agent.recovery as recovery
    clock = Clock()
    adapter = BodyAdapter(clock, 'vertical' if fault == 'moving' else None)
    profile = configured(load_profile(STUB_PROFILE))
    profile = SimpleNamespace(spec=SimpleNamespace(
        safety=profile.spec.safety.model_copy(update={'restart_recovery_controllers': {'synthetic-base': 'active'}}),
        mobility=profile.spec.mobility.model_copy(update={'nav2_navigate_to_pose': '/synthetic/navigate'}),
        actions={}))
    if fault == 'missing':
        profile.spec.safety = profile.spec.safety.model_copy(update={'restart_recovery_controllers': None})
    if fault == 'opaque':
        profile.spec.actions = {'synthetic-motion': SimpleNamespace(is_motion=True)}
    reads = []
    def controllers():
        reads.append(True)
        state = 'inactive' if fault == 'baseline' or (fault == 'changed' and len(reads) > 1) else 'active'
        return {'synthetic-base': {'state': state}}
    def cancel(name):
        adapter.calls.append(('cancel_all', name))
        if fault == 'cancel':
            raise RpcError('CleanupFailed', 'injected cancellation failure')
    adapter.loaded_controllers = controllers
    adapter.cancel_all_goals = cancel
    lock = MotionLock(quarantine={resource: {'reason': 'checkpoint failure'}, 'other': {'reason': 'retained'}})
    ctx = SimpleNamespace(lock=lock, state=AgentState(estop_engaged=True), groups={},
        tasks=SimpleNamespace(has_running=lambda: False), ros=adapter, profile=profile,
        commands=SimpleNamespace(failed=False, prune=lambda:None))
    original = recovery.verify_mobility_stop
    monkeypatch.setattr(recovery, 'verify_mobility_stop',
        lambda a, p, m: original(a, p, m, clock=clock.read, sleep=clock.sleep))
    if fault:
        with pytest.raises(RpcError):
            recovery.recover_restart(ctx, resource)
        assert resource in lock.quarantine
    else:
        result = recovery.recover_restart(ctx, resource)
        assert result['resources']['mobility']['verified']
        assert ('cancel_all', '/synthetic/navigate') in adapter.calls
        assert resource not in lock.quarantine
    assert ctx.state.estop_engaged and 'other' in lock.quarantine


@pytest.mark.parametrize('fault', ['failed', 'commit'])
def test_journal_recovery_requires_a_repaired_writable_store(fault):
    """Journal recovery cannot clear an unhealthy store

    Categories: sdk
    What it checks: Rejects a previously failed ledger and an unsuccessful verification write before any controller or physical recovery effects.
    Why it matters: Storage repair and measured recovery must precede new motion eligibility.
    If it fails: Preserve journal quarantine and verify a fresh durable write before recovery.
    Profile fields: safety.restart_recovery_controllers
    Scenario fields: None.
    Robot effects: Offline failed ledger only; no robot or controller effects.
    """
    from ringfour_agent.recovery import recover_restart
    writes=[]
    def prune():
        writes.append(True)
        raise RpcError('CleanupFailed','verification write failed')
    ctx=SimpleNamespace(commands=SimpleNamespace(failed=fault=='failed',prune=prune))
    with pytest.raises(RpcError) as exc:recover_restart(ctx,'journal')
    assert exc.value.error_type=='CleanupFailed'
    assert bool(writes)==(fault=='commit')
