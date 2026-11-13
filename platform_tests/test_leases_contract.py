"""A12/A13/A15: finite authority without replaying motion intent."""
import asyncio
import json
from dataclasses import replace
from types import SimpleNamespace

import pytest
from .conftest import command_id
from ringfour import wire
from ringfour.aio.client import Robot
from ringfour.aio.transport import Connection
from ringfour_agent.access import AccessPolicy
from ringfour_agent.bootstrap import build_agent
from ringfour_agent.leases import Lease
from ringfour_agent.models.profile import LeasePolicy
from ringfour_agent.profile import load_profile
from ringfour_agent.safety import MotionLock
from ringfour_agent.server import RpcError, Session
from .conftest import STUB_PROFILE, _free_tcp_port

pytestmark = pytest.mark.contract


@pytest.mark.parametrize('field,value', [('lifetime', 0), ('lifetime', True), ('lifetime', float('inf')),
    ('renew_interval', '1'), ('renew_interval', 0), ('renew_interval', 5)])
def test_policy_rejects_unusable_budgets(field, value):
    """Policy rejects unusable budgets

    Categories: sdk
    What it checks: Rejects non-finite, non-positive or inconsistent lease timing before startup.
    Why it matters: Lost authority must never admit new motion or refresh stale command intent.
    If it fails: Repair lease identity, revocation and cleanup ordering; do not tune the robot to bypass it.
    Profile fields: safety.lease
    Scenario fields: None.
    Robot effects: Offline synthetic fixtures and isolated protocol sockets only; no robot is contacted.
    """
    with pytest.raises(ValueError):
        LeasePolicy(**({'lifetime': 10, 'renew_interval': 1} | {field: value}))


def test_expiry_and_identity_cannot_be_renewed_into_new_authority():
    """Expiry and identity cannot be renewed into new authority

    Categories: sdk
    What it checks: Uses a fake clock to check expiry and mismatched principal, connection, token and boot epoch.
    Why it matters: Lost authority must never admit new motion or refresh stale command intent.
    If it fails: Repair lease identity, revocation and cleanup ordering; do not tune the robot to bypass it.
    Profile fields: safety.lease
    Scenario fields: None.
    Robot effects: Offline synthetic fixtures and isolated protocol sockets only; no robot is contacted.
    """
    lease = Lease.issue('boot-A', 'alice', 7, 10, now=100)
    for args in [('wrong', 'boot-A', 'alice', 7), (lease.id, 'boot-B', 'alice', 7),
                 (lease.id, 'boot-A', 'bob', 7), (lease.id, 'boot-A', 'alice', 8)]:
        with pytest.raises(RpcError): lease.renew(*args, now=101)
        assert lease.deadline == 110
    lease.renew(lease.id, 'boot-A', 'alice', 7, now=102)
    assert lease.deadline == 112
    with pytest.raises(RpcError): lease.renew(lease.id, 'boot-A', 'alice', 7, now=112)
    with pytest.raises(RpcError): lease.check(now=100)
    assert lease.revoked


def test_revoked_acquisition_cannot_race_cleanup():
    """Revoked acquisition cannot race cleanup

    Categories: sdk
    What it checks: Rejects replacement acquisitions from a revoked connection while allowing a new owner after release.
    Why it matters: Lost authority must never admit new motion or refresh stale command intent.
    If it fails: Repair lease identity, revocation and cleanup ordering; do not tune the robot to bypass it.
    Profile fields: safety.lease
    Scenario fields: None.
    Robot effects: Offline synthetic fixtures and isolated protocol sockets only; no robot is contacted.
    """
    lock = MotionLock()
    assert lock.acquire(1, 'first')
    with lock.mutex: lock.revoked_clients.add(1)
    lock.release_all(1)
    with pytest.raises(RpcError): lock.acquire(1, 'replacement')
    assert lock.acquire(2, 'new-owner')


class Socket:
    async def send(self, message): pass
    async def close(self, **kwargs): pass


def configured(tmp_path):
    profile = load_profile(STUB_PROFILE)
    policy = LeasePolicy(lifetime=10, renew_interval=1)
    profile = replace(profile, spec=profile.spec.model_copy(update={
        'safety': profile.spec.safety.model_copy(update={'lease': policy})}))
    return build_agent(profile, unix_path=None, host='127.0.0.1', port=_free_tcp_port(),
                       audit_dir=tmp_path, access_policy=AccessPolicy(trusted_tcp=True), allow_unverified_stops=True)


def test_expiry_revokes_before_cleanup_and_rejects_stale_motion(tmp_path):
    """A12: fake-clock expiry rejects every later dispatch before adapter effects.

    Categories: sdk
    What it checks: Expires a connection lease and verifies that a later motion request causes zero adapter dispatches.
    Why it matters: Lost authority must never admit new motion or refresh stale command intent.
    If it fails: Repair lease identity, revocation and cleanup ordering; do not tune the robot to bypass it.
    Profile fields: safety.lease
    Scenario fields: None.
    Robot effects: Offline synthetic fixtures and isolated protocol sockets only; no robot is contacted.
    """
    async def run():
        server, ctx = configured(tmp_path)
        session = Session(Socket())
        server._sessions.add(session)
        reply = json.loads(await server._dispatch(session, wire.request(1, 'hello', {'protocol':1,'client_version':'test'})))
        metadata = reply['result']['features']['connection_lease']
        assert metadata['epoch'] == ctx.boot_epoch and metadata['principal'] == 'in-process'
        calls=[]
        async def actuator(s, p): calls.append(p); return {}
        server._registry['teleop.start'] = actuator
        server._lease_clock = lambda: session.lease.deadline
        server._expire_leases()
        assert session.client_id in ctx.lock.revoked_clients
        response = json.loads(await server._dispatch(session, json.dumps({**json.loads(wire.request(2, 'teleop.start', {'watchdog':.5})), 'command_id': command_id(ctx)})))
        assert response['error']['data']['reason'] == 'lease expired'
        assert calls == []
        await asyncio.gather(*tuple(ctx.background_halts))
        assert not ctx.lock._acquisitions
        ctx.audit.close()
    asyncio.run(run())


def test_protocol_renewal_bound_to_session_epoch_and_hello(tmp_path):
    """A13/A15: real protocol never grants another connection's lease.

    Categories: sdk
    What it checks: Checks renewal through real TCP sockets, including replay from another connection and repeated hello.
    Why it matters: Lost authority must never admit new motion or refresh stale command intent.
    If it fails: Repair lease identity, revocation and cleanup ordering; do not tune the robot to bypass it.
    Profile fields: safety.lease
    Scenario fields: None.
    Robot effects: Offline synthetic fixtures and isolated protocol sockets only; no robot is contacted.
    """
    from websockets.asyncio.client import connect
    async def run():
        server,ctx=configured(tmp_path)
        await server.start()
        try:
            async with connect(f'ws://127.0.0.1:{server._port}') as ws:
                async def call(i,m,p):
                    await ws.send(wire.request(i,m,p));return json.loads(await ws.recv())
                welcome=(await call(1,'hello',{'protocol':1,'client_version':'test'}))['result']
                lease=welcome['features']['connection_lease']
                assert 'result' in await call(2,'lease.renew',{'lease_id':lease['id'],'epoch':lease['epoch']})
                assert (await call(3,'lease.renew',{'lease_id':lease['id'],'epoch':'old-boot'}))['error']['data']['reason']=='lease identity'
                assert 'error' in await call(4,'hello',{'protocol':1,'client_version':'test'})
                async with connect(f'ws://127.0.0.1:{server._port}') as other:
                    await other.send(wire.request(1,'hello',{'protocol':1,'client_version':'test'}));await other.recv()
                    await other.send(wire.request(2,'lease.renew',{'lease_id':lease['id'],'epoch':lease['epoch']}))
                    assert json.loads(await other.recv())['error']['data']['reason']=='lease identity'
        finally:
            await server.close();ctx.audit.close()
    asyncio.run(run())


def test_sdk_heartbeat_sends_only_lease_renewal(monkeypatch):
    """A12: an automatic heartbeat never makes old drive intent fresh.

    Categories: sdk
    What it checks: Records automatic SDK heartbeat messages and verifies that no drive command is replayed.
    Why it matters: Lost authority must never admit new motion or refresh stale command intent.
    If it fails: Repair lease identity, revocation and cleanup ordering; do not tune the robot to bypass it.
    Profile fields: safety.lease
    Scenario fields: None.
    Robot effects: Offline synthetic fixtures and isolated protocol sockets only; no robot is contacted.
    """
    async def run():
        conn=Connection(); calls=[]
        original_sleep=asyncio.sleep
        async def next_tick(delay):
            assert delay == 1
        monkeypatch.setattr('ringfour.aio.transport.asyncio.sleep',next_tick)
        async def call(method,params,timeout):
            calls.append((method,params));conn._closed=True
        conn.call=call
        conn.start_lease({'id':'opaque','epoch':'boot','lifetime':10,'renew_interval':1})
        await conn._lease_renewer
        assert calls==[('lease.renew',{'lease_id':'opaque','epoch':'boot'})]
    asyncio.run(run())


def test_boot_epoch_changes_across_agent_instances(tmp_path):
    """Boot epoch changes across agent instances

    Categories: sdk
    What it checks: Creates two agent instances and checks that their boot epochs differ.
    Why it matters: Lost authority must never admit new motion or refresh stale command intent.
    If it fails: Repair lease identity, revocation and cleanup ordering; do not tune the robot to bypass it.
    Profile fields: safety.lease
    Scenario fields: None.
    Robot effects: Offline synthetic fixtures and isolated protocol sockets only; no robot is contacted.
    """
    first,a=configured(tmp_path/'a');second,b=configured(tmp_path/'b')
    assert a.boot_epoch != b.boot_epoch
    a.audit.close();b.audit.close()


@pytest.mark.protocol
@pytest.mark.parametrize('method,params', [
    ('lease.renew', None), ('mobility.stop', {}),
    ('task.cancel', {'task_id':'recording-only'}), ('debug.set_estop', {'engaged':True}),
])
def test_slow_observation_cannot_starve_lease_or_stop(tmp_path, method, params):
    """A12: lease and stop requests bypass a blocked ordinary request

    Categories: sdk
    What it checks: Holds a real TCP observation handler on an event, sends a renewal or stop on the same connection, and requires its reply before releasing the observation.
    Why it matters: A healthy client must not lose authority or wait to stop merely because an ordinary adapter call is slow.
    If it fails: Repair the bounded priority request path; do not increase robot lease times to hide serialized heartbeat handling.
    Profile fields: safety.lease
    Scenario fields: None.
    Robot effects: Offline isolated protocol handlers only; no robot is contacted.
    """
    async def run():
        server, ctx = configured(tmp_path)
        entered, release = asyncio.Event(), asyncio.Event()
        effects=[]
        async def slow(session, params):
            entered.set()
            await release.wait()
            return {}
        async def stop(session, params):
            effects.append(method)
            return {'stopped':True}
        server._registry['tf.wait_for'] = slow
        if method != 'lease.renew':server._registry[method] = stop
        await server.start()
        connection=None
        try:
            connection=await Connection.open(f'ws://127.0.0.1:{server._port}')
            welcome=await connection.call('hello',{'protocol':1,'client_version':'priority-contract'})
            held=asyncio.create_task(connection.call('tf.wait_for',{'frame':'blocked'},timeout=3))
            await asyncio.wait_for(entered.wait(),1)
            lease=welcome['features']['connection_lease']
            payload=params if params is not None else {'lease_id':lease['id'],'epoch':lease['epoch']}
            result=await connection.call(method,payload,timeout=1)
            assert not held.done(), 'ordinary request must remain blocked while priority reply arrives'
            if method=='lease.renew':assert result['lease_id']==lease['id']
            else:assert effects==[method] and result['stopped']
            release.set()
            await held
        finally:
            release.set()
            if connection is not None:await connection.close()
            await server.close();ctx.audit.close()
    asyncio.run(run())
