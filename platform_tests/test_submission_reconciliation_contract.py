"""Milestone C: deterministic reply-loss, retry, restart and result evidence."""
import asyncio
import json
import uuid
from types import SimpleNamespace
import pytest
from ringfour import aio
from ringfour.errors import SubmissionUnknown, CommandConflict
from ringfour_agent.access import Principal, PERMISSIONS
from ringfour_agent.bootstrap import build_agent
from ringfour_agent.commands import CommandLedger
from ringfour_agent.profile import load_profile
from ringfour_agent.server import RpcError, Session
from .conftest import STUB_PROFILE

pytestmark = pytest.mark.contract
EPOCH = 'a'*32


class Socket:
    async def send(self, message):
        pass


def identifier(ledger, lifetime=10):
    return f'{ledger.epoch}.{int((ledger.now()+lifetime)*1000)}.{uuid.uuid4().hex}'


def session(principal='alice', authenticated=True):
    return Session(Socket(), principal=Principal(principal, PERMISSIONS, authenticated))


@pytest.mark.parametrize('conflict', [False, True])
def test_concurrent_duplicate_commands_have_one_dispatch(tmp_path, conflict):
    """Concurrent duplicate IDs share one acceptance

    Categories: sdk
    What it checks: Holds a recording dispatch open while an identical or conflicting request arrives from a second authenticated connection.
    Why it matters: Retrying a lost reply must never dispatch the same command twice.
    If it fails: Repair durable claiming and shared pending dispatch; do not retry with a fresh ID.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline recording handler and temporary database only.
    """
    async def run():
        ledger = CommandLedger(EPOCH, path=tmp_path/'commands.sqlite3')
        key = identifier(ledger)
        entered, release = asyncio.Event(), asyncio.Event()
        calls = []
        async def dispatch():
            calls.append(True); entered.set(); await release.wait()
            return json.dumps({'jsonrpc':'2.0','id':1,'result':{'task_id':'one-operation'}})
        first = asyncio.create_task(ledger.execute(session(), key, 'synthetic.motion', {'target':1}, dispatch, 1))
        await entered.wait()
        second = asyncio.create_task(ledger.execute(session(), key, 'synthetic.motion', {'target':2 if conflict else 1}, dispatch, 2))
        await asyncio.sleep(0)
        assert calls == [True]
        release.set()
        original = json.loads(await first)
        if conflict:
            with pytest.raises(RpcError) as exc: await second
            assert exc.value.error_type == 'CommandConflict'
        else:
            duplicate = json.loads(await second)
            assert duplicate['id'] == 2 and duplicate['result'] == original['result']
        assert calls == [True]
        ledger.db.close()
    asyncio.run(run())


def test_lost_motion_reply_reconciles_over_real_protocol(tmp_path):
    """Lost acceptance reply reconciles to one motion operation

    Categories: sdk
    What it checks: Drops only a real socket's motion acceptance reply after recording dispatch, observes SubmissionUnknown, queries its command, and races identical retries plus a conflicting request.
    Why it matters: A transport timeout must preserve uncertainty and the original operation identity.
    If it fails: Repair the production transport, ledger or owner checks; never infer non-execution from reply loss.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Isolated Unix socket and recording actuator only; no running robot.
    """
    async def run():
        path = str(tmp_path/'agent.sock')
        server, ctx = build_agent(load_profile(STUB_PROFILE), unix_path=path, audit_dir=tmp_path)
        calls = []
        async def motion(owner, params):
            calls.append(params)
            async def body(handle):
                handle.mark_dispatched()
                handle.record_stop({'verified':True,'criterion':'synthetic-rest','physical_stop':'uncommissioned'})
                return {'arrived':True}
            return {'task_id':ctx.tasks.start('synthetic.motion', owner, body)}
        server._registry['mobility.navigate_to'] = motion
        original = server._serve_request
        dropped = []
        async def deliver(owner, message):
            if json.loads(message).get('method') == 'mobility.navigate_to' and not dropped:
                dropped.append(True)
                await server._dispatch(owner, message)  # Execute and commit; lose only its reply.
            else:
                await original(owner, message)
        server._serve_request = deliver
        await server.start()
        try:
            robot = await aio.Robot.connect('unix://'+path)
            key = robot.commands.new()
            params = {'goal':{'frame':'map','x':0.,'y':0.}}
            with pytest.raises(SubmissionUnknown) as exc:
                await robot.commands.retry(key, 'mobility.navigate_to', params, timeout=.04)
            assert exc.value.details['command_id'] == key
            record = await robot.commands.get(key)
            assert record.state == 'completed' and record.operation_id
            duplicate = await asyncio.gather(*[robot.commands.retry(key,'mobility.navigate_to',params) for _ in range(2)])
            assert duplicate == [record.result,record.result]
            with pytest.raises(CommandConflict):
                await robot.commands.retry(key,'mobility.navigate_to',{'goal':{'frame':'map','x':1.}})
            assert len(calls) == 1
            terminal = await robot.operations.get(record.operation_id)
            assert terminal.state == 'terminal' and terminal.snapshot.state == 'succeeded'
            await robot.close()
        finally:
            await server.close();ctx.audit.close();ctx.commands.db.close()
    asyncio.run(run())


@pytest.mark.parametrize('case', ['expired', 'old-epoch', 'capacity', 'missing-id'])
def test_invalid_or_evicted_ids_never_become_new_work(tmp_path, case):
    """Expiry epoch and capacity reject before dispatch

    Categories: sdk
    What it checks: Uses an injected clock and bounded ledger to expire an evicted ID, change boot epoch, exhaust record capacity or omit identity.
    Why it matters: Eviction and restart must not turn an old retry into a fresh actuator command.
    If it fails: Repair immutable ID expiry and admission ordering.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline recording handler and temporary database only.
    """
    async def run():
        clock = [1000.]
        ledger = CommandLedger(EPOCH, path=tmp_path/'commands.sqlite3', clock=lambda:clock[0])
        key = identifier(ledger, 1)
        owner = session()
        calls = []
        async def dispatch(): calls.append(True);return json.dumps({'result':{}})
        if case == 'expired':
            await ledger.execute(owner,key,'synthetic.motion',{},dispatch,1)
            clock[0] += 400;ledger.prune();assert ledger.db.execute('SELECT count(*) FROM commands').fetchone()[0] == 0
        elif case == 'old-epoch':
            ledger.epoch = 'b'*32
        elif case == 'capacity':
            ledger.capacity = 0
        else:
            key = None
        before = len(calls)
        with pytest.raises(RpcError) as exc:
            await ledger.execute(owner,key,'synthetic.motion',{},dispatch,2)
        assert exc.value.error_type == {'expired':'CommandExpired','old-epoch':'AgentEpochChanged','capacity':'ResourceBusy','missing-id':'CommandRequired'}[case]
        assert len(calls) == before
        ledger.db.close()
    asyncio.run(run())


def test_crash_preserves_terminal_and_marks_inflight_unknown(tmp_path):
    """Crash preserves terminal evidence and never resumes inflight work

    Categories: sdk
    What it checks: Reopens a durable ledger under a new epoch with completed and inflight operation records, including a pending acceptance record.
    Why it matters: Restart may preserve evidence but cannot manufacture a physical outcome or replay motion.
    If it fails: Repair crash reconciliation and terminal retention.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Temporary SQLite database and synthetic records only.
    """
    path = tmp_path/'commands.sqlite3'
    clock = [1000.]
    first = CommandLedger(EPOCH,path=path,clock=lambda:clock[0])
    owner = session()
    for name in ['finished','inflight']:
        first.task_started(SimpleNamespace(id=name,session=owner))
    first.task_finished(SimpleNamespace(id='finished'), {'task_id':'finished','state':'failed','error':{'type':'StopUnverified','message':'retained','data':{'physical_stop':'unverified'}}})
    key = identifier(first)
    with first.db:
        first.db.execute('INSERT INTO commands VALUES (?,?,?,?,?,?,?,?,?,NULL)',('alice',key,owner.reconciliation_key,EPOCH,'synthetic.motion','fingerprint',1010.,1300.,'pending'))
    first.db.close()
    second = CommandLedger('b'*32,path=path,clock=lambda:clock[0])
    reconnected = session()
    terminal = second.operation(reconnected,'finished')
    assert terminal['snapshot']['error']['type']=='StopUnverified' and terminal['state']=='terminal'
    unknown = second.operation(reconnected,'inflight')
    assert unknown['state']=='unknown' and unknown['physical_outcome']=='unknown'
    assert second.get(reconnected,key)['state']=='unknown'
    clock[0] += 301
    with pytest.raises(RpcError):second.operation(reconnected,'finished')
    second.db.close()


@pytest.mark.parametrize('authenticated,other_principal,allowed', [(True,False,True),(False,False,False),(True,True,False)])
def test_reconnect_query_requires_the_authenticated_owner(tmp_path, authenticated, other_principal, allowed):
    """Record references do not grant cross-owner query authority

    Categories: sdk
    What it checks: Reconnects under the same authenticated principal, an unverified connection or another principal and queries retained operation evidence.
    Why it matters: Guessing an operation ID or sharing development TCP permissions must not grant ownership.
    If it fails: Repair principal and unverified-connection binding before exposing retained evidence.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline principals and temporary database only.
    """
    ledger = CommandLedger(EPOCH,path=tmp_path/'commands.sqlite3')
    original = session(authenticated=authenticated)
    ledger.task_started(SimpleNamespace(id='synthetic-operation',session=original))
    reconnect = session('bob' if other_principal else 'alice',authenticated)
    if allowed:
        assert ledger.operation(reconnect,'synthetic-operation')['state']=='active'
    else:
        with pytest.raises(RpcError) as exc:ledger.operation(reconnect,'synthetic-operation')
        assert exc.value.error_type=='PermissionDenied'
    ledger.db.close()


def test_clock_rollback_cannot_revive_an_evicted_id(tmp_path):
    """Durable clock floor prevents expired command revival

    Categories: sdk
    What it checks: Advances beyond retention, prunes, restarts with a backwards wall clock and retries the expired ID in the original epoch.
    Why it matters: Clock correction after eviction must not make an old command executable.
    If it fails: Persist and enforce the journal's logical clock floor.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Injected clock and temporary database only.
    """
    async def run():
        clock=[1000.];path=tmp_path/'commands.sqlite3'
        first=CommandLedger(EPOCH,path=path,clock=lambda:clock[0]);key=identifier(first)
        clock[0]=1400.;first.prune();first.db.close();clock[0]=900.
        second=CommandLedger(EPOCH,path=path,clock=lambda:clock[0]);calls=[]
        async def dispatch():calls.append(True);return '{}'
        with pytest.raises(RpcError) as exc:await second.execute(session(),key,'synthetic.motion',{},dispatch,1)
        assert exc.value.error_type=='CommandExpired' and not calls
        second.db.close()
    asyncio.run(run())


def test_storage_failure_inhibits_motion_but_preserves_stop_authority(tmp_path):
    """Storage failure cannot dispatch motion or disable authorized stopping

    Categories: sdk
    What it checks: Makes the command database read-only, submits motion and then an independently authorized stop through the production dispatcher.
    Why it matters: Failed durability must inhibit new effects while preserving the stop path.
    If it fails: Repair failure quarantine and stop admission ordering.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline recording handlers and read-only temporary database only.
    """
    async def run():
        server,ctx=build_agent(load_profile(STUB_PROFILE),unix_path=str(tmp_path/'unused.sock'),audit_dir=tmp_path)
        owner=session();owner.handshaken=True;calls=[]
        async def motion(s,p):calls.append('motion');return {}
        async def stop(s,p):calls.append('stop');return {'stopped':True}
        server._registry['teleop.start']=motion
        server._registry['mobility.stop']=stop
        entered=asyncio.Event()
        async def active(handle):
            handle.mark_dispatched();entered.set();await asyncio.Event().wait()
        active_id=ctx.tasks.start('synthetic-active',owner,active)
        await entered.wait()
        key=identifier(ctx.commands)
        ctx.commands.db.execute('PRAGMA query_only=ON')
        def message(method,params):return json.dumps({'jsonrpc':'2.0','id':1,'method':method,'params':params,'command_id':key})
        refused=json.loads(await server._dispatch(owner,message('teleop.start',{'watchdog':.3})))
        assert refused['error']['data']['type']=='CleanupFailed'
        assert ctx.state.estop_engaged and 'journal' in ctx.lock.quarantine and not calls
        assert ctx.tasks._tasks[active_id].cancel_requested
        stopped=json.loads(await server._dispatch(owner,message('mobility.stop',{})))
        assert stopped['result']['stopped'] and calls==['stop']
        await asyncio.gather(*tuple(ctx.background_halts),return_exceptions=True)
        await asyncio.gather(ctx.tasks._tasks[active_id].aio_task,return_exceptions=True)
        ctx.commands.db.execute('PRAGMA query_only=OFF')
        unknown=await server._registry['operations.get'](owner,{'operation_id':active_id})
        assert unknown['state']=='unknown' and unknown['physical_outcome']=='unknown'
        ctx.commands.db.close();ctx.audit.close();ctx.shm.close()
    asyncio.run(run())


def test_cancelled_reply_wait_does_not_cancel_accepted_dispatch(tmp_path):
    """A cancelled observer cannot detach an accepted command record

    Categories: sdk
    What it checks: Cancels the coroutine waiting for an accepted dispatch, then reconciles and retries while the shared dispatch is still pending.
    Why it matters: Request-observer lifetime must not create duplicate effects or lose the acceptance record.
    If it fails: Shield the shared dispatch and retain its result independently of the caller.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline event-controlled handler and temporary database only.
    """
    async def run():
        ledger=CommandLedger(EPOCH,path=tmp_path/'commands.sqlite3')
        owner=session();key=identifier(ledger);entered=asyncio.Event();release=asyncio.Event();calls=[]
        async def dispatch():
            calls.append(True);entered.set();await release.wait();return json.dumps({'result':{'accepted':True}})
        waiting=asyncio.create_task(ledger.execute(owner,key,'synthetic.motion',{},dispatch,1))
        await entered.wait();waiting.cancel()
        with pytest.raises(asyncio.CancelledError):await waiting
        assert ledger.get(owner,key)['state']=='pending'
        retry=asyncio.create_task(ledger.execute(owner,key,'synthetic.motion',{},dispatch,2))
        release.set();assert json.loads(await retry)['result']['accepted']
        assert calls==[True] and ledger.get(owner,key)['state']=='completed'
        ledger.db.close()
    asyncio.run(run())


def test_shutdown_drain_failure_still_stops_and_retains_unclean_marker(tmp_path):
    """Shutdown cannot let a stuck acceptance suppress stopping

    Categories: sdk
    What it checks: Injects a command-drain failure during shutdown with accepted work, verifies independent stop/cancellation still execute, and requires an unresolved dispatch to retain the active boot marker.
    Why it matters: Graceful shutdown must not become a stopping bypass or a false clean restart checkpoint.
    If it fails: Preserve bounded independent stop paths and do not finish the boot while accepted handlers remain.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline recording task and temporary safety checkpoint only.
    """
    async def run():
        server,ctx=build_agent(load_profile(STUB_PROFILE),unix_path=str(tmp_path/'unused.sock'),
                               audit_dir=tmp_path/'audit',state_dir=tmp_path/'state')
        owner=session();entered=asyncio.Event();calls=[]
        async def body(handle):
            handle.mark_dispatched();entered.set()
            try:await asyncio.Event().wait()
            except asyncio.CancelledError:
                calls.append('task-stop')
                handle.record_stop({'verified':True,'criterion':'synthetic-rest','physical_stop':'uncommissioned'})
                raise
        key=ctx.tasks.start('synthetic-active',owner,body)
        await entered.wait()
        ctx.teleop=SimpleNamespace(end_all=lambda:calls.append('teleop-stop'))
        ctx.commands.pending['synthetic-stuck']=(owner.reconciliation_key,None)
        async def blocked(session=None):raise RpcError('CleanupFailed','injected dispatch drain timeout')
        ctx.commands.drain=blocked
        await server.close()
        assert set(calls)=={'task-stop','teleop-stop'}
        assert ctx.tasks.status(key,owner)['state']=='cancelled'
        assert 'shutdown' in ctx.lock.quarantine
        assert json.loads(ctx.lock.quarantine.path.read_text())['active_boot'] is True
        ctx.commands.pending.clear();ctx.lock.quarantine.finish_boot()
        ctx.commands.db.close();ctx.audit.close()
    asyncio.run(run())
