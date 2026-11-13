"""Portability and scenario checks without a ROS stack or robot movement."""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from types import SimpleNamespace

import numpy as np
import pytest

from ringfour.models.protocol import Welcome
from conformance import expected_capabilities, instance_names, validate_scenarios, validate_vector
import importlib.util
spec=importlib.util.spec_from_file_location("live_harness_tests", Path(__file__).resolve().parents[2]/"conftest.py")
harness=importlib.util.module_from_spec(spec)
sys.modules[spec.name]=harness
spec.loader.exec_module(harness)
import test_phase4_exit as sensor_checks
import server


def welcome(name, groups=None, cameras=(), capabilities=()):
    groups = groups or {}
    return Welcome.model_validate({
        "protocol":1,"profile":{"name":name,"model":name+" model","version":"1","capabilities":capabilities},
        "capabilities":capabilities, "groups":groups,
        "instances":{"groups":list(groups),"cameras":list(cameras),
                     "arms":[n for n,g in groups.items() if g["kind"]=="arm"]}})


PLATFORMS = [
    ("wheeled", {"deck":{"kind":"arm","joints":["servo-a","servo-b"]}}, ["front"], ["mobility","groups","cameras"]),
    ("quadruped", {"forelegs":{"kind":"arm","joints":[f"leg-{i}" for i in range(12)]}}, ["belly"], ["groups","cameras","imu"]),
    ("humanoid", {"left_hand":{"kind":"arm","joints":[f"L{i}" for i in range(7)]},
                  "gaze":{"kind":"pointing","joints":["pan","tilt"]},
                  "torso":{"kind":"linear","joints":["lift"]}}, ["eye-left","eye-right"], ["groups","cameras"]),
    ("flying", {}, ["downward"], ["cameras","imu","flight"]),
]


@pytest.mark.parametrize("name,groups,cameras,capabilities", PLATFORMS)
def test_collection_expands_live_instances_without_morphology_gate(name,groups,cameras,capabilities):
    value = welcome(name,groups,cameras,capabilities)
    assert instance_names(value,"groups")==list(groups)
    assert instance_names(value,"cameras")==cameras
    assert instance_names(value,"capabilities")==capabilities
    calls = {}
    metafunc = SimpleNamespace(fixturenames=["group_name","camera_name"],
                              config=SimpleNamespace(_conformance_welcome=value),
                              parametrize=lambda name,params: calls.update({name:params}))
    harness.pytest_generate_tests(metafunc)
    if groups:
        assert calls["group_name"]==list(groups)
    else:
        assert calls["group_name"][0].marks[0].kwargs["reason"].startswith("NOT APPLICABLE")
    assert calls["camera_name"]==cameras


@pytest.mark.parametrize("width,height", [(160,120),(1280,720),(1920,1080),(83,57)])
def test_camera_contract_accepts_advertised_resolution(width,height):
    camera = SimpleNamespace(get_frame=lambda:SimpleNamespace(image=np.zeros((height,width,3),dtype=np.uint8),stamp=0,id='capture'),
                             get_intrinsics=lambda:SimpleNamespace(width=width,height=height,fx=1,fy=1))
    camera.name = "any-camera"
    payload = {"transport":"shm", "name":"test", "dtype":"uint8", "shape":[height,width,3], "size":height*width*3}
    robot = SimpleNamespace(_runner=SimpleNamespace(run=lambda x:x), _robot=SimpleNamespace(connection=SimpleNamespace(call=lambda *a,**kw:{"payload":payload})))
    sensor_checks.test_camera_frame_matches_its_intrinsics(camera,robot)


@pytest.mark.parametrize("count", [1,2,6,7,12])
def test_joint_contract_accepts_arbitrary_count_names_and_continuous_joints(count, monkeypatch):
    names = [f'joint#{i}' for i in range(count)]
    group = SimpleNamespace(joint_names=tuple(names),get_joint_positions=lambda:{n:0. for n in names},
                            get_joints=lambda:{n:SimpleNamespace(position=0.,limits=None) for n in names})
    from ringfour import kinematics as kin
    monkeypatch.setattr(kin,"limits",lambda g:{n:None for n in names})
    description = SimpleNamespace(joints={n:SimpleNamespace(limits=None) for n in names})
    sensor_checks.test_group_joint_states_with_urdf_limits(group,{},description)


def test_absent_capability_skips_before_opening_connection(monkeypatch):
    value=welcome('drone',capabilities=['imu'])
    request=SimpleNamespace(node=SimpleNamespace(
        iter_markers=lambda _: [SimpleNamespace(args=('mobility',))],user_properties=[]))
    import ringfour
    monkeypatch.setattr(ringfour,'Robot',lambda *a,**k:pytest.fail('A skipped check opened a robot connection'))
    generator=harness.robot.__wrapped__(request,None,value)
    with pytest.raises(pytest.skip.Exception,match='NOT APPLICABLE'):
        next(generator)
    assert ('coverage','not_applicable') in request.node.user_properties


def scenario_fixture():
    value=welcome('humanoid', {'torso':{'kind':'linear','joints':['slide'],'control_modes':['trajectory']}}, capabilities=['groups'])
    data={'version':1,'profile':{'name':'humanoid','model':'humanoid model'},
          'groups':{'torso':{'waypoints':[{'slide':.1},{'slide':.2}],'dt':3}}}
    return value,data


def test_scenario_vectors_follow_resolved_order():
    value,data=scenario_fixture()
    assert validate_scenarios(data,value)==data
    assert validate_vector({'b':2.,'a':1.},('a','b'),'probe')==[1.,2.]


@pytest.mark.parametrize('change', ['wrong_robot','wrong_joint','nan','unknown_group','short_path','typo','negative_tolerance','wrong_sha'])
def test_incompatible_scenarios_are_rejected_before_motion(change):
    value,data=scenario_fixture()
    if change=='wrong_robot':data['profile']['name']='another'
    if change=='wrong_joint':data['groups']['torso']['waypoints'][0]={'wrong':.1}
    if change=='nan':data['groups']['torso']['waypoints'][0]['slide']=float('nan')
    if change=='unknown_group':data['groups']['arm']={}
    if change=='short_path':data['groups']['torso']['waypoints']=[]
    if change=='typo':data['groups']['torso']['duration']=1
    if change=='negative_tolerance':data['tolerances']={'joint_limits':{'slide':-.1}}
    if change=='wrong_sha':data['profile']['description_sha']='stale'
    with pytest.raises(ValueError):validate_scenarios(data,value)


def test_profile_claims_have_no_locomotion_or_identity_whitelist():
    profile={'name':'drone','model':'quadrotor','protocol':1,'version':'1','frames':{},
             'mobility':{'locomotion':'aerial'},'cameras':{},'imu':{},'flight':{},'safety':{}}
    assert expected_capabilities(profile)=={'mobility','cameras','imu','flight'}


def test_coverage_distinguishes_unsupported_and_unexercised():
    assert server.coverage('Skipped: NOT APPLICABLE: no camera')=='not_applicable'
    assert server.coverage('Skipped: NOT CONFIGURED: no flight contract')=='not_configured'
    assert server.coverage('Skipped: DISABLED: no motion opt-in')=='disabled'
    assert server.coverage('ordinary skip')=='not_exercised'


def test_unexercised_run_is_incomplete(tmp_path,monkeypatch):
    source=tmp_path/'test_uncovered.py'
    source.write_text('import pytest\ndef test_missing():\n    pytest.skip("NOT CONFIGURED: missing scenario")\n')
    monkeypatch.setattr(server,'catalog',lambda:[{'id':'sdk','path':str(source)}])
    runner=server.Runner(tmp_path/'runs')
    run=runner.start({'suites':['sdk']})
    from test_runner import wait_until
    wait_until(lambda:runner.active is None)
    assert runner.snapshot(run['id'])['status']=='incomplete'
    assert runner.snapshot(run['id'])['cases'][0]['coverage']=='not_configured'


def test_native_fk_comparison_uses_articulated_mount_frame():
    from ringfour.description import RobotDescription
    from ringfour.models import GroupInfo
    from test_phase6_kinematics_exit import in_mount_frame
    shape=RobotDescription.parse('''<robot name="fixture">
<link name="airframe"/><link name="platform"/><link name="motor"/><link name="tool"/>
<joint name="lift" type="prismatic"><parent link="airframe"/><child link="platform"/>
<origin xyz="1 2 3" rpy="0.3 0.5 0.2"/><axis xyz="0 0 1"/><limit lower="0" upper="1" effort="1" velocity="1"/></joint>
<joint name="actuator" type="revolute"><parent link="platform"/><child link="motor"/>
<origin xyz="0.1 0 0.2"/><axis xyz="0 0 1"/><limit lower="-3.14" upper="3.14" effort="1" velocity="1"/></joint>
<joint name="flange" type="fixed"><parent link="motor"/><child link="tool"/><origin xyz="0.4 0 0"/></joint>
</robot>''')
    info=GroupInfo(kind='arm',tip='tool',joints=('actuator',))
    owner=SimpleNamespace(description=shape,group_info=lambda _:info)
    group=SimpleNamespace(name='appendage',joint_names=('actuator',),tip='tool',get_joint_positions=lambda:{'actuator':np.pi/2},get_kinematic_positions=lambda:{'lift':.4,'actuator':np.pi/2},_owner=owner)
    local=in_mount_frame(group,shape)
    assert local.frame=='platform'
    assert (local.x,local.y,local.z)==pytest.approx((.1,.4,.2),abs=1e-10)
    assert local.q.to_euler()[2]==pytest.approx(np.pi/2)


def test_moving_mount_ik_uses_snapshot_instead_of_obsolete_topology_skip(monkeypatch):
    import importlib.util
    monkeypatch.setattr(importlib.util,'find_spec',lambda _:object())
    state={'torso':.4,'knee':.2,'ankle':.1}
    request=SimpleNamespace(node=SimpleNamespace(user_properties=[]),getfixturevalue=lambda name:state)
    group=SimpleNamespace(name='leg',tip='foot',joint_names=('knee','ankle'))
    description=SimpleNamespace(moving_chain_to=lambda _:('torso','knee','ankle'))
    assert harness.kin_backend.__wrapped__(request,group,description)==state
    assert request.node.user_properties==[]


@pytest.mark.parametrize('recipe',[{'command':['change']},{'command':'change','restore_command':['restore']},
                                  {'command':['change'],'restore_command':['restore'],'typo':1}])
def test_model_change_requires_explicit_restore_argv(recipe):
    value,data=scenario_fixture()
    data['description_change']=recipe
    with pytest.raises(ValueError):validate_scenarios(data,value)


def test_old_agent_is_explicitly_untested_not_zero_filled():
    from ringfour.errors import CapabilityNotSupported
    class OlderGroup:
        joint_names=('arm',)
        def get_kinematic_snapshot(self):raise CapabilityNotSupported('agent lacks kinematic snapshot support')
    request=SimpleNamespace(node=SimpleNamespace(user_properties=[]))
    description=SimpleNamespace(joints={'lift':SimpleNamespace(name='lift',moves=True)})
    with pytest.raises(pytest.skip.Exception,match='rebuild/reload the agent'):
        harness.kinematic_state.__wrapped__(request,OlderGroup(),description)
    assert ('coverage','not_configured') in request.node.user_properties


def test_tool_applicability_precedes_missing_recipe_and_motion_opt_in():
    request=SimpleNamespace(node=SimpleNamespace(user_properties=[]),
        getfixturevalue=lambda name:pytest.fail('An absent tool requested '+name))
    generator=harness.tool_case.__wrapped__(request,SimpleNamespace(name='head',tool=None))
    with pytest.raises(pytest.skip.Exception,match='NOT APPLICABLE'):
        next(generator)
    assert ('coverage','not_applicable') in request.node.user_properties


def test_missing_controller_precedes_missing_recipe():
    request=SimpleNamespace(node=SimpleNamespace(user_properties=[]),
        getfixturevalue=lambda name:pytest.fail('Missing controller requested '+name))
    group=SimpleNamespace(name='limb',_owner=SimpleNamespace(integration=SimpleNamespace(group_controllers=lambda name:{})))
    with pytest.raises(pytest.skip.Exception,match='NOT APPLICABLE'):
        harness.session_group.__wrapped__(request,group)


def test_missing_or_nan_snapshot_is_a_failure_not_a_skip():
    request=SimpleNamespace(node=SimpleNamespace(user_properties=[]))
    description=SimpleNamespace(joints={'lift':SimpleNamespace(name='lift',moves=True)})
    for state in ({'arm':0.},{'arm':0.,'lift':float('nan')}):
        group=SimpleNamespace(joint_names=('arm',),get_kinematic_snapshot=lambda:SimpleNamespace(positions=state,stamp=1.))
        with pytest.raises(AssertionError):harness.kinematic_state.__wrapped__(request,group,description)
    assert request.node.user_properties==[]


def test_navigation_is_not_applicable_without_a_profile_binding():
    request=SimpleNamespace(node=SimpleNamespace(user_properties=[]),
        getfixturevalue=lambda name:pytest.fail('Absent navigation requested '+name))
    with pytest.raises(pytest.skip.Exception,match='NOT APPLICABLE'):
        harness.navigation_recipe.__wrapped__(request,None,{}, {'mobility':{'locomotion':'legged','odom_topic':'/walking/odom'}},lambda *a:None)
    assert ('coverage','not_applicable') in request.node.user_properties


def test_stop_watch_helper_unwinds_on_termination(tmp_path):
    import subprocess,json,time,os
    # A real subprocess/signal check; fake only the robot transport.
    package=tmp_path/'ringfour';package.mkdir()
    package.joinpath('__init__.py').write_text('''
from pathlib import Path
import os
class Robot:
 def __init__(self,*args): self.signals=Signal()
 def get_is_moving(self):return True
 def __enter__(self): return self
 def __exit__(self,*args): pass
 def teleop(self,**kwargs):return self
 def drive(self,**kwargs):pass
 def guard(self,*args,**kwargs):return Watch()
class Signal:
 def __getitem__(self,key):return self
 def above(self,*args,**kwargs):return self
class Watch:
 id='owned-test-watch'
 def __enter__(self):return self
 def __exit__(self,*args):Path(os.environ['WATCH_CLOSED']).write_text('closed')
''')
    package.joinpath('errors.py').write_text('class RingfourError(Exception):pass\n')
    closed=tmp_path/'closed';output=tmp_path/'output'
    helper=Path(__file__).resolve().parents[2]/'helpers/drive_ignoring_watch.py'
    recipe={'duration':.02,'watchdog':.1,'command':{'linear':.2},'watch_stop':{'path':'velocity.linear.x','above':.1}}
    with output.open('w') as stream:
        proc=subprocess.Popen([sys.executable,str(helper),'fake://robot',json.dumps(recipe)],
             stdout=stream,stderr=subprocess.STDOUT,env=dict(os.environ,PYTHONPATH=str(tmp_path),WATCH_CLOSED=str(closed)))
    try:
        deadline=time.monotonic()+5
        while 'GUARD ACTIVE owned-test-watch' not in output.read_text():
            assert proc.poll() is None, output.read_text()
            assert time.monotonic()<deadline
            time.sleep(.02)
        proc.terminate();proc.wait(timeout=5)
        assert closed.read_text()=='closed'
    finally:
        if proc.poll() is None:proc.kill();proc.wait(timeout=5)


@pytest.mark.parametrize('moves,read_updates',[(False,False),(True,True),(True,False)])
def test_returning_to_start_requires_observed_motion(moves,read_updates):
    from contextlib import nullcontext
    from ringfour import Controller
    from ringfour.enums import Phase
    import test_phase6_motion_exit as checks
    values={'axis':0.}
    class Session:
        controller='synthetic-controller'
        def start_trajectory(self,trajectory):
            class Op:
                def __enter__(self):return self
                def __exit__(self,*args):pass
                def updates(self):
                    from ringfour.models import JointValue
                    for config in trajectory.configurations:
                        point=config.positions['axis']
                        if read_updates:values['axis']=point
                        feedback=point if moves else values['axis']
                        yield SimpleNamespace(progress=SimpleNamespace(positions=(JointValue(name='axis',position=feedback,unit='rad'),)))
                def result(self):return SimpleNamespace(stop=SimpleNamespace(verified=True))
            return Op()
    def configuration(values):
        from ringfour.models import JointConfiguration,JointValue
        return JointConfiguration(robot='fixture',group='group',model_revision='v1',joints=tuple(JointValue(name=n,position=v,unit='rad') for n,v in values.items()))
    group=SimpleNamespace(configuration=configuration,joint_names=('axis',),get_joint_positions=lambda:dict(values),
         _owner=SimpleNamespace(integration=SimpleNamespace(group_controllers=lambda name:{Controller.TRAJECTORY:'synthetic-controller'})),
         name='arbitrary-group',control=lambda role=None:nullcontext(Session()))
    recipe={'tolerance':.01,'settle':0.}
    if moves and read_updates:
        checks.test_follow_waypoints_and_converge((group,[[.1],[0.]],recipe))
    else:
        with pytest.raises(AssertionError,match='observable measured joint motion|joint-state reads'):
            checks.test_follow_waypoints_and_converge((group,[[.1],[0.]],recipe))


def test_map_journey_restores_an_existing_named_map_namespace(tmp_path,monkeypatch):
    monkeypatch.syspath_prepend(str(Path(__file__).resolve().parents[4]/"ringfour-agent/src/ringfour_agent"))
    monkeypatch.syspath_prepend(str(Path(__file__).resolve().parents[4]/"ringfour-testing"))
    from platform_tests.test_phase5_units import FakeSlamRos
    from ringfour_agent.slam import SlamManager
    from ringfour_agent.places import PlaceStore
    from ringfour import Pose
    import test_phase5_exit as checks
    manager=SlamManager(FakeSlamRos(),str(tmp_path))
    store=PlaceStore(manager)
    goal={'x':1.,'y':2.,'frame':'map'}
    store.save('existing',goal);manager.save('existing-map')
    class Slam:
        def get_mode(self):return manager.mode
        def get_saved_maps(self):return manager.saved_maps()
        def get_state(self):return manager.state()
        def save(self,name):return SimpleNamespace(name=manager.save(name)['name'])
        def localize(self,**kwargs):return manager.localize(**kwargs)
        def wait_for_localization(self,**kwargs):return 1.
        def load(self,name):manager.load(name)
        def start(self,**kwargs):manager.start(**kwargs)
        def delete(self,name):manager.delete(name)
    places=SimpleNamespace(list=store.list,delete=store.delete,save=lambda name,pose:store.save(name,pose.model_dump(mode='json')))
    robot=SimpleNamespace(slam=Slam(),places=places,get_pose=lambda:Pose(**goal),stop=lambda:None,navigate_to=lambda *a,**kw:None)
    checks.test_map_save_localize_and_go_journey(robot=robot,scenarios={'mapping':{'save_localize':True}},
             navigation_recipe={'goals':[goal]},request=SimpleNamespace())
    assert manager.active_map=='existing-map'
    assert store.list()==['existing']
    assert manager.saved_maps()==['existing-map']
