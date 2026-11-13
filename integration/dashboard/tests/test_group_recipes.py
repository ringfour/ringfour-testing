from types import SimpleNamespace
import pytest
from group_recipes import generate_probe
from conformance import validate_scenarios
from test_requirements import welcome


def specs():
    return {'axis-A':SimpleNamespace(type='revolute',limits=(-1,1)),
            'slide':SimpleNamespace(type='prismatic',limits=(0,.5))}


def test_generated_probes_cover_every_joint_and_return_to_measured_baseline():
    home={'axis-A':.99,'slide':.1}
    result=generate_probe(list(home),home,specs(),{'mode':'small','reviewed':True},.001)
    assert result['waypoints'][0]['axis-A']==pytest.approx(.94)
    assert result['waypoints'][0]['slide']==.1
    assert result['waypoints'][1]==home
    assert result['waypoints'][2]['axis-A']==.99
    assert result['waypoints'][2]['slide']==pytest.approx(.105)
    assert result['waypoints'][3]==home
    assert home=={'axis-A':.99,'slide':.1}
    assert result['collision_checked'] is False


@pytest.mark.parametrize('change',['unreviewed','nan','missing','outside','tiny','unknown_type'])
def test_refuses_unusable_probes(change):
    home={'axis-A':0,'slide':.1};model=specs();policy={'mode':'small','reviewed':True}
    if change=='unreviewed':policy['reviewed']=False
    if change=='nan':home['axis-A']=float('nan')
    if change=='missing':del home['slide']
    if change=='outside':home['slide']=1
    if change=='tiny':policy['linear_step']=.0001
    if change=='unknown_type':model['slide'].type='floating'
    with pytest.raises(ValueError):generate_probe(['axis-A','slide'],home,model,policy,.001)


def test_policy_schema_works_without_waypoint_arrays():
    data={'version':1,'profile':{'name':'fixture','model':'arbitrary'},'groups':{'appendage':{'probe':{'mode':'small','reviewed':True,'angular_step':.05,'linear_step':.005}}}}
    validate_scenarios(data,welcome())
    data['groups']['appendage']['probe']['linear_step']=-1
    with pytest.raises(ValueError):validate_scenarios(data,welcome())


def test_default_group_recipe_needs_no_manual_configuration():
    import importlib.util
    from pathlib import Path
    spec=importlib.util.spec_from_file_location('live_group_fixtures',Path(__file__).resolve().parents[2]/'conftest.py')
    fixtures=importlib.util.module_from_spec(spec);spec.loader.exec_module(fixtures)
    assert fixtures.group_recipe.__wrapped__(None,SimpleNamespace(name='new-group'),{})=={}
    request=SimpleNamespace(getfixturevalue=lambda name:{'probe':{'mode':'read'}},node=SimpleNamespace(user_properties=[]))
    group=SimpleNamespace(name='new-group',_owner=SimpleNamespace(integration=SimpleNamespace(group_controllers=lambda name:{'trajectory':'controller'})))
    with pytest.raises(pytest.skip.Exception,match='read-only'):
        fixtures.session_group.__wrapped__(request,group)


def test_model_change_quarantines_old_fixtures_without_touching_file(tmp_path,monkeypatch):
    import json
    import server
    w=welcome();path=tmp_path/'settings.json'
    old={'profile':{'description_sha':'b'*64},'groups':{'appendage':{'allow_sessions':True}}}
    path.write_text(json.dumps(old));original=path.read_bytes()
    monkeypatch.setattr(server,'test_settings_path',lambda welcome:path)
    monkeypatch.setattr(server,'matching_profile',lambda welcome:None)
    result=server.test_settings(w)
    assert result['requirements']['stale_model'] is True
    assert not result['settings'].get('groups')
    assert path.read_bytes()==original


def test_runner_rejects_commands_using_stale_model_settings(tmp_path,monkeypatch):
    import server,json
    w=welcome()
    monkeypatch.setattr(server,'SCENARIOS',tmp_path/'settings')
    monkeypatch.setattr(server,'inspect_agent',lambda uri:w.model_dump(mode='json'))
    monkeypatch.setattr(server,'matching_profile',lambda welcome:None)
    monkeypatch.setattr(server,'catalog',lambda:[{'id':'live','path':'unused.py'}])
    path=server.test_settings_path(w);path.parent.mkdir()
    path.write_text(json.dumps({'profile':{'description_sha':'b'*64},'groups':{'appendage':{'allow_sessions':True}}}))
    runner=server.Runner(tmp_path/'runs')
    with pytest.raises(ValueError,match='review and save fresh'):
        runner.start({'suites':['live'],'uri':'ws://fixture:10101','profile':'@live','motion_ack':True})
    assert runner.active is None
    assert not runner.results.exists()


def test_hold_samples_after_takeover_and_checks_before_release(monkeypatch):
    import test_phase6_motion_exit as checks
    measured={'joint':0.0}
    class Session:
        def __enter__(self):measured['joint']=.02;return self
        def __exit__(self,*args):measured['joint']=0
        def hold(self):return dict(measured)
    group=SimpleNamespace(get_joint_positions=lambda:dict(measured),control=lambda role=None:Session(),joint_names=['joint'])
    # Each SDK read returns a fresh snapshot, not an alias to mutable feedback.
    class Group:
        joint_names=['joint']
        def configuration(self,values):
            from ringfour.models import JointConfiguration,JointValue
            return JointConfiguration(robot='fixture',group='group',model_revision='v1',joints=tuple(JointValue(name=n,position=v,unit='rad') for n,v in values.items()))
        def configuration(self,values):
            from ringfour.models import JointConfiguration,JointValue
            return JointConfiguration(robot='fixture',group='group',model_revision='v1',joints=tuple(JointValue(name=n,position=v,unit='rad') for n,v in values.items()))
        def get_joint_positions(self):return dict(measured)
        def control(self,role=None):return Session()
    monkeypatch.setattr(checks.time,'sleep',lambda seconds:None)
    checks.test_hold_commands_current_position(Group(),{'settle':.1,'tolerance':.001})
    assert measured['joint']==0


def test_numerical_limit_noise_is_recorded_and_never_commanded_outside_limits():
    result=generate_probe(['slide'],{'slide':-2e-8},specs(),{'mode':'small','reviewed':True},.001)
    assert result['measured_baseline']['slide']==-2e-8
    assert result['baseline']['slide']==0
    assert result['limit_adjustments']['slide']['allowance']==1e-5
    assert all(0<=target['slide']<=.5 for target in result['waypoints'])
    with pytest.raises(ValueError):
        generate_probe(['slide'],{'slide':-.0001},specs(),{'mode':'small','reviewed':True},.001)


def test_arrival_is_measured_before_controller_release(monkeypatch):
    import test_phase6_motion_exit as checks
    from ringfour import Controller
    from ringfour.enums import Phase
    measured={'joint':0.0}
    class Session:
        controller='trajectory'
        def __enter__(self):return self
        def __exit__(self,*args):measured['joint']=.5
        def start_trajectory(self,trajectory):
            class Op:
                def __enter__(self):return self
                def __exit__(self,*args):pass
                def updates(self):
                    measured['joint']=.04
                    from ringfour.models import JointValue
                    yield SimpleNamespace(progress=SimpleNamespace(positions=(JointValue(name='joint',position=.04,unit='rad'),)))
                def result(self):return SimpleNamespace(stop=SimpleNamespace(verified=True))
            return Op()
    class Group:
        name='arbitrary';joint_names=['joint']
        def configuration(self,values):
            from ringfour.models import JointConfiguration,JointValue
            return JointConfiguration(robot='fixture',group='group',model_revision='v1',joints=tuple(JointValue(name=n,position=v,unit='rad') for n,v in values.items()))
        _owner=SimpleNamespace(integration=SimpleNamespace(group_controllers=lambda name:{Controller.TRAJECTORY:'trajectory'}))
        def get_joint_positions(self):return dict(measured)
        def control(self,role=None):return Session()
    monkeypatch.setattr(checks.time,'sleep',lambda seconds:None)
    checks.test_follow_waypoints_and_converge((Group(),[[.05],[.04]],{'tolerance':.001,'probe':{'mode':'small'}}))
    assert measured['joint']==.5
