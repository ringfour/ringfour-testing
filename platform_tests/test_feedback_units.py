"""Timestamped feedback and non-destructive mapping without a ROS robot."""
import asyncio
import pytest
from ringfour.models import MobilityState, KinematicSnapshot
from ringfour.aio.sensing import Arm
from ringfour.aio.tf import TfAPI
from ringfour.aio.autonomy import SlamAPI
from ringfour.errors import CapabilityNotSupported
from ringfour_agent.places import PlaceStore
from ringfour_agent.slam import SCRATCH_MAP
from .test_phase5_units import slam


class Connection:
    def __init__(self, result):
        self.result = result
        self.calls = []
    async def call(self, method, params=None, **kwargs):
        self.calls.append((method, params, kwargs))
        return self.result


@pytest.mark.parametrize('count', [1, 3, 7, 12])
def test_snapshot_preserves_external_positions_and_time(count):
    """Joint snapshots retain measured state and time

    Categories: sdk
    What it checks: Reads arbitrary one-, three-, seven-, and twelve-joint snapshots, including an independently moving mount, and verifies positions and ROS time survive decoding without an additional state query.
    Why it matters: Comparing different instants or silently dropping mount feedback invalidates kinematics.
    If it fails: Inspect arm.joint_states decoding and snapshot transport.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline synthetic feedback only.
    """
    positions = {f'axis-{i}':i*.13 for i in range(count)} | {'external-mount':.42}
    conn = Connection({'positions':positions, 'stamp':123.456, 'joints':{}})
    group = Arm(conn, 'arbitrary-group')
    value = asyncio.run(group.get_kinematic_snapshot())
    assert isinstance(value,KinematicSnapshot)
    assert value.positions == positions and value.stamp == 123.456
    assert len(conn.calls) == 1
    assert conn.calls[0][1] == {'arm':'arbitrary-group','include_external':True}


def test_legacy_feedback_does_not_fabricate_timestamps():
    """Older feedback keeps unknown timestamps explicit

    Categories: sdk
    What it checks: Decodes legacy mobility and joint feedback and verifies missing timestamps and raw odometry remain unknown rather than receiving client wall time or zero state.
    Why it matters: Fabricated timestamps produce plausible but invalid movement and transform comparisons.
    If it fails: Inspect optional feedback model defaults.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline synthetic feedback only.
    """
    value = MobilityState(pose={'frame':'map'},velocity={},is_moving=False,distance_traveled=0.)
    assert value.stamp is None and value.odometry_pose is None
    assert KinematicSnapshot(positions={'external-slide':.4}).stamp is None


def test_tf_lookup_sends_requested_ros_time():
    """Transforms can be read at the joint measurement time

    Categories: sdk
    What it checks: Requests a transform at a specific ROS timestamp and checks its wire parameter; an ordinary latest-transform request omits that parameter.
    Why it matters: Timestamp alignment prevents motion between state and TF reads from appearing as incorrect model geometry.
    If it fails: Inspect the TF facade and request serialization.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline synthetic transform only.
    """
    conn=Connection({'translation':{'x':0,'y':0,'z':0},'rotation':{'x':0,'y':0,'z':0,'w':1},'parent':'mount','child':'tip'})
    api=TfAPI(conn)
    asyncio.run(api.pose_of('tip','mount',stamp=12.5))
    assert conn.calls[-1][1]['stamp']==12.5
    asyncio.run(api.lookup('mount','tip'))
    assert 'stamp' not in conn.calls[-1][1]


def test_old_agent_cannot_clear_map_when_preservation_requested():
    """Map preservation refuses unsupported agents before mutation

    Categories: sdk
    What it checks: Requests mapping with graph preservation against an older agent and verifies a capability error occurs before any slam.start command is sent.
    Why it matters: An older agent may ignore unknown parameters and clear the map silently.
    If it fails: Inspect SLAM feature negotiation before its mutating call.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline fake connection only.
    """
    conn=Connection({'mode':'localization'})
    with pytest.raises(CapabilityNotSupported):asyncio.run(SlamAPI(conn).start(preserve_graph=True))
    assert [c[0] for c in conn.calls]==['slam.get_state']


def test_resume_mapping_preserves_graph_and_existing_places(slam):
    """Resuming mapping can retain its graph and saved places

    Categories: sdk
    What it checks: Saves and localizes a synthetic map, resumes mapping with graph preservation, then checks no reset occurred and existing places remain available after temporary-map deletion.
    Why it matters: A conformance run must not erase the user's map as cleanup.
    If it fails: Inspect preserved SLAM transitions and place namespaces.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline temporary map files and a fake SLAM backend.
    """
    places=PlaceStore(slam)
    places.save('existing',{'x':1.,'y':2.,'frame':'map'})
    slam.save('temporary')
    slam.localize(name='temporary')
    slam._ros.calls.clear()
    slam.start(preserve_graph=True)
    assert 'reset' not in slam._ros.calls
    assert slam.active_map==SCRATCH_MAP and slam.mode=='mapping'
    assert places.list()==['existing']
    slam.delete('temporary')
    assert places.get('existing')['x']==1.


def test_localization_quality_recovers_after_many_jumps(slam):
    """Localization confidence can recover after repeated corrections

    Categories: sdk
    What it checks: Supplies a thousand large map-to-odometry jumps, then stable transforms, and checks confidence remains low during jumps and recovers afterward. Missing transforms still report zero confidence.
    Why it matters: Multiplying confidence indefinitely can underflow to zero and leave wait_for_localization permanently blocked after the robot stabilizes.
    If it fails: Inspect the localization heuristic's bounded history and missing-TF handling.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline synthetic transforms only.
    """
    for i in range(1000):
        slam._ros.correction=(float(i%2),0.,0.)
        slam.sample_quality()
    assert 0<slam.quality<.2
    for i in range(100):slam.sample_quality()
    assert slam.quality>=.5 and not slam.is_lost
    slam._ros.correction=None;slam.sample_quality()
    assert slam.quality==0. and slam.is_lost


def test_partial_joint_messages_never_fabricate_positions_or_velocities():
    """Partial joint feedback keeps unknown measurements explicit

    Categories: sdk
    What it checks: Merges position-only and effort-only joint messages, verifies absent positions never become zero, old positions retain their original timestamp, and absent velocity is unknown rather than stationary.
    Why it matters: Fabricated zero positions corrupt external-mount geometry, and fabricated zero velocity can falsely suggest a gripper has stopped against an object.
    If it fails: Inspect ROS joint-state merging and per-joint timestamps.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline partial joint messages only.
    """
    from types import SimpleNamespace
    from ringfour_agent.feedback import update_joint_feedback
    import math
    joints={};stamps={}
    update_joint_feedback(joints,stamps,SimpleNamespace(name=['mount'],position=[.4],velocity=[],effort=[]),1.)
    assert joints['mount'][0]==.4 and math.isnan(joints['mount'][1])
    update_joint_feedback(joints,stamps,SimpleNamespace(name=['mount','unmeasured-axis'],position=[],velocity=[],effort=[2.,3.]),2.)
    assert joints['mount'][0]==.4 and stamps['mount']==1.
    assert 'unmeasured-axis' not in joints


def test_tf_snapshot_selects_actual_joint_sample_and_rejects_interpolation():
    """TF comparisons use actual recorded joint samples

    Categories: sdk
    What it checks: Chooses a recorded joint sample at a TF timestamp, verifies the returned positions belong to that instant even after newer feedback arrives, and rejects a timestamp between samples or outside history.
    Why it matters: Interpolated TF and a faster joint stream can disagree while both are correct; the conformance comparison must use a common actual sample.
    If it fails: Inspect joint-feedback history selection and ROS timestamp transport.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline feedback history only.
    """
    from ringfour_agent.feedback import snapshot_at
    history=[(1.,{'axis':(.1,0.,0.)}),(1.01,{'axis':(.3,0.,0.)})]
    values,stamp=snapshot_at(history,1.)
    assert values['axis'][0]==.1 and stamp==1.
    values['axis']=(9.,0.,0.)
    assert history[0][1]['axis'][0]==.1
    with pytest.raises(LookupError):snapshot_at(history,1.005)
    with pytest.raises(LookupError):snapshot_at(history,.5)
    conn=Connection({'positions':{'axis':.3},'stamp':1.01})
    with pytest.raises(CapabilityNotSupported):
        asyncio.run(Arm(conn,'arbitrary-group').get_kinematic_snapshot(stamp=1.))


def test_arrival_rejects_missing_and_nonfinite_joint_feedback():
    """Trajectory arrival requires complete finite joint positions

    Categories: sdk
    What it checks: Rejects successful-controller arrival confirmation when any commanded joint lacks a measured position or reports NaN/infinity; verifies complete feedback retains real residuals.
    Why it matters: Unknown feedback must not become successful arrival, especially for partially published robot joint states.
    If it fails: Inspect the agent's trajectory arrival validation.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline joint feedback only.
    """
    from ringfour_agent.control import checked_arrival_residual
    for bad in ({},{'slide':(float('nan'),0.,0.)},{'slide':(float('inf'),0.,0.)}):
        with pytest.raises(ValueError):checked_arrival_residual(['slide'],[.2],bad)
    assert checked_arrival_residual(['slide'],[.2],{'slide':(.3,0.,0.)})['slide']==pytest.approx(.1)


def test_arrival_error_uses_urdf_units_including_mixed_joint_types():
    """Trajectory errors distinguish rotary and linear joint units

    Categories: sdk
    What it checks: Formats a prismatic residual in metres, a rotary residual in radians/degrees, and an unknown joint type without guessing units from its group name.
    Why it matters: Calling a lift displacement an angle misleads operators trying to configure timing and accuracy.
    If it fails: Inspect joint-type lookup and arrival error formatting.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline error formatting only.
    """
    from ringfour_agent.control import arrival_failure_message
    linear=arrival_failure_message('slide',.08,.005,'prismatic')
    assert '0.080 m' in linear and '0.005 m' in linear and 'deg' not in linear and 'rad' not in linear
    assert '0.080 rad' in arrival_failure_message('hinge',.08,.05,'revolute')
    assert 'joint units' in arrival_failure_message('unknown',.08,.05)
