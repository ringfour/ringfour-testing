"""Movement profile validation, configuration persistence, and motion feedback."""
import math
from pathlib import Path
from types import SimpleNamespace
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import server
import movement
import test_movement_profiles as checks
from conformance import validate_scenarios
from ringfour.models.protocol import Welcome
from ringfour.models.spatial import Pose


def live_welcome(capabilities=('mobility','teleop')):
    return Welcome.model_validate({'protocol':1,'profile':{'name':'fixture','model':'synthetic','version':'1','capabilities':list(capabilities)},'capabilities':list(capabilities),'groups':{},'instances':{}})


def recipe(name):
    return dict((key,value) for key,_,value in next(p for p in movement.MOVEMENT_PROFILES if p['id']==name)['fields'])


def draft(name):
    return {'version':1,'profile':{'name':'fixture','model':'synthetic'},'movement_profiles':{name:recipe(name)}}


@pytest.mark.parametrize('name', ['figure_eight','braking','odometry_spin'])
def test_profiles_validate_without_robot_morphology(name):
    assert validate_scenarios(draft(name),live_welcome())['movement_profiles'][name] == recipe(name)
    with pytest.raises(ValueError,match='require mobility'):
        validate_scenarios(draft(name),live_welcome(('cameras','flight')))


@pytest.mark.parametrize('field,value', [('angular',0),('turns',float('nan')),('watchdog',1.1),('max_drift',-1)])
def test_invalid_profile_settings_are_rejected(field,value):
    data=draft('odometry_spin')
    data['movement_profiles']['odometry_spin'][field]=value
    with pytest.raises(ValueError):validate_scenarios(data,live_welcome())


def test_scenario_lifecycle_preserves_other_sections_and_checks_stale_files(tmp_path,monkeypatch):
    import yaml
    monkeypatch.setattr(server,'SCENARIOS',tmp_path)
    monkeypatch.setattr(server,'inspect_agent',lambda uri:live_welcome().model_dump(mode='json'))
    data=draft('braking');data['tolerances']={'fk_position':.001}
    request={'uri':'fixture','yaml':yaml.safe_dump(data),'name':'fixture.conformance.yaml'}
    review=server.scenario_build({**request,'movement_profile':'odometry_spin','settings':recipe('odometry_spin')})
    assert 'braking' in review['data']['movement_profiles']
    assert review['data']['tolerances']==data['tolerances']
    assert not list(tmp_path.iterdir())
    saved=server.save_scenario({**request,'yaml':review['yaml']})
    loaded=server.load_scenario({'path':saved['path']})
    assert loaded['managed'] and loaded['data']==review['data']
    with pytest.raises(ValueError,match='changed'):
        server.save_scenario(request)
    with pytest.raises(ValueError,match='filename'):
        server.save_scenario({**request,'name':'../escape.conformance.yaml'})
    removed=server.scenario_build({**request,'yaml':loaded['yaml'],'movement_profile':'braking','settings':None})
    assert set(removed['data']['movement_profiles'])=={'odometry_spin'}
    updated=server.save_scenario({**request,'yaml':removed['yaml'],'expected_sha':saved['sha']})
    assert Path(updated['backup']).read_text()==review['yaml']


def test_trace_unwraps_revolutions_and_rejects_frame_changes():
    trace=movement.MotionTrace(Pose(frame='odom'))
    for i in range(1,101):trace.sample(Pose(frame='odom',yaw=i*4*math.pi/100))
    assert trace.yaw==pytest.approx(4*math.pi)
    with pytest.raises(AssertionError,match='frame changed'):
        trace.sample(Pose(frame='map'))


class FakeRobot:
    def __init__(self):
        self.now=self.last=0.;self.x=self.y=self.angle=0.;self.linear=self.angular=0.
        self.turn_scale=1.;self.slip=0.;self.cleanup=False;self.brake_delay=0.;self.stopping_at=None
    def tick(self):
        dt=self.now-self.last;self.last=self.now
        if self.stopping_at is not None and self.now-self.stopping_at >= self.brake_delay:
            self.linear=self.angular=0.
        angular=self.angular*self.turn_scale
        if angular:
            self.x+=self.linear/angular*(math.sin(self.angle+angular*dt)-math.sin(self.angle))
            self.y-=self.linear/angular*(math.cos(self.angle+angular*dt)-math.cos(self.angle))
        else:self.x+=self.linear*math.cos(self.angle)*dt;self.y+=self.linear*math.sin(self.angle)*dt
        self.angle+=angular*dt;self.x+=self.slip*dt
    def get_pose(self):
        self.tick();return Pose(x=self.x,y=self.y,yaw=self.angle,frame='odom')
    def get_velocity(self):
        self.tick();return SimpleNamespace(linear=SimpleNamespace(x=self.linear,y=0.,z=0.),angular=SimpleNamespace(x=0.,y=0.,z=self.angular))
    def get_mobility_state(self):
        from ringfour.models import MobilityState
        return MobilityState(pose=self.get_pose(),odometry_pose=self.get_pose(),velocity={"linear":vars(self.get_velocity().linear),"angular":vars(self.get_velocity().angular)},
                             stamp=self.now,is_moving=abs(self.linear)>0,distance_traveled=0.)
    def teleop(self,watchdog):return self
    def __enter__(self):return self
    def __exit__(self,*args):self.stop()
    def drive(self,linear=0.,angular=0.,**kwargs):
        self.tick();self.linear=linear;self.angular=angular;self.stopping_at=None
    def stop(self):
        self.tick();self.stopping_at=self.now;self.cleanup=True
    def sleep(self,dt):self.now+=dt


@pytest.fixture
def fake_robot(monkeypatch):
    robot=FakeRobot()
    clock=SimpleNamespace(monotonic=lambda:robot.now,sleep=robot.sleep)
    monkeypatch.setattr(movement,'time',clock);monkeypatch.setattr(checks,'time',clock)
    return robot


@pytest.mark.parametrize('name,check', [('figure_eight',checks.test_figure_eight_closes_both_loops),('braking',checks.test_braking_time_and_distance),('odometry_spin',checks.test_odometry_spin_rotation_and_drift)])
def test_movement_contracts_measure_feedback_and_stop(name,check,fake_robot):
    properties={}
    check(fake_robot,recipe(name),lambda key,value:properties.update({key:value}))
    assert properties and fake_robot.cleanup


def test_bad_spin_feedback_fails_and_stops(fake_robot):
    fake_robot.turn_scale=.5
    with pytest.raises(AssertionError,match='Accumulated rotation'):
        checks.test_odometry_spin_rotation_and_drift(fake_robot,recipe('odometry_spin'),lambda *args:None)
    assert fake_robot.cleanup


def test_excessive_braking_is_measured_and_fails(fake_robot):
    fake_robot.brake_delay=3.
    limits=recipe("braking");limits["halt_timeout"]=4.
    properties={}
    with pytest.raises(AssertionError,match='configured time'):
        checks.test_braking_time_and_distance(fake_robot,limits,lambda key,value:properties.update({key:value}))
    assert properties['measured.braking_time_s'] > 1.4
    assert properties['measured.braking_distance_m'] > .14
    assert fake_robot.cleanup


def test_measured_values_survive_events_and_junit(tmp_path):
    import json
    directory=tmp_path/'sample';directory.mkdir()
    server.save_meta(directory,{'id':'sample','status':'running','started':0})
    events=[{'event':'collection','tests':['a']}, {'event':'report','nodeid':'a','when':'call','outcome':'passed','duration':.1,'detail':'','properties':{'measured.braking_time_s':.4}}]
    (directory/'events.jsonl').write_text('\n'.join(json.dumps(e) for e in events))
    assert server.Runner(tmp_path).snapshot('sample')['cases'][0]['properties']['measured.braking_time_s']==.4
    xml=directory/'results.xml'
    xml.write_text('<testsuite><testcase name="test_braking"><properties><property name="measured.braking_time_s" value="0.4"/></properties></testcase></testsuite>')
    assert server.read_cases(xml)[0]['properties']['measured.braking_time_s']=='0.4'


def test_builtin_defaults_match_old_drive_check_and_follow_capabilities():
    from movement import default_scenarios
    value=live_welcome()
    defaults=default_scenarios(value)
    assert defaults['teleop']['command']['linear']==.3
    assert defaults['teleop']['duration']==2
    assert defaults['teleop']['min_distance']==.2
    assert set(defaults['movement_profiles'])=={'figure_eight','braking','odometry_spin'}
    assert validate_scenarios(defaults,value)==defaults
    assert 'movement_profiles' not in default_scenarios(live_welcome(('cameras','flight')))
    defaults['teleop']['command']['linear']=9
    assert default_scenarios(value)['teleop']['command']['linear']==.3


def test_simple_settings_save_reset_and_stale_update(tmp_path,monkeypatch):
    from movement import default_scenarios
    monkeypatch.setattr(server,'SCENARIOS',tmp_path)
    monkeypatch.setattr(server,'inspect_agent',lambda uri:live_welcome().model_dump(mode='json'))
    first=server.inspect_test_settings({'uri':'fixture'})
    assert first['sha'] is None and first['settings']['teleop']['command']['linear']==.3
    edits={k:first['settings'][k] for k in ('teleop','movement_profiles')}
    edits['teleop']['command']['linear']=.12
    saved=server.save_test_settings({'uri':'fixture','settings':edits,'expected_sha':None})
    assert saved['settings']['teleop']['command']['linear']==.12
    assert server.test_settings(live_welcome())['settings']==saved['settings']
    with pytest.raises(ValueError,match='Settings changed'):
        server.save_test_settings({'uri':'fixture','settings':edits,'expected_sha':None})
    reset=server.save_test_settings({'uri':'fixture','reset':True,'expected_sha':saved['sha']})
    assert reset['settings']==default_scenarios(live_welcome())
    assert len(list(tmp_path.glob('*.bak')))==1


def test_runner_freezes_builtin_defaults_without_scenario_file(tmp_path,monkeypatch):
    import json,time
    source=tmp_path/'test_defaults.py'
    source.write_text('import os,yaml\ndef test_defaults():\n    data=yaml.safe_load(open(os.environ["RINGFOUR_CONFORMANCE"]))\n    assert data["teleop"]["command"]["linear"]==0.3\n    assert os.environ["RINGFOUR_TEST_MOTION"]=="0"\n')
    monkeypatch.setattr(server,'SCENARIOS',tmp_path/'settings')
    monkeypatch.setattr(server,'inspect_agent',lambda uri:live_welcome().model_dump(mode='json'))
    monkeypatch.setattr(server,'catalog',lambda:[{'id':'live','path':str(source)}])
    runner=server.Runner(tmp_path/'runs')
    result=runner.start({'suites':['live'],'uri':'ws://fixture:10101','profile':'@live','motion_ack':False})
    deadline=time.monotonic()+10
    while runner.active and time.monotonic()<deadline:time.sleep(.02)
    run=runner.snapshot(result['id'])
    assert run['status']=='passed'
    assert run['scenarios']['teleop']['duration']==2
    assert (runner.results/result['id']/'conformance.yaml').exists()


def test_slow_simulation_uses_robot_clock_and_odometry_frame(fake_robot,monkeypatch):
    original_tick=fake_robot.tick
    def slow_tick():
        fake_robot.last=fake_robot.now-(fake_robot.now-fake_robot.last)*.4
        original_tick()
    monkeypatch.setattr(fake_robot,'tick',slow_tick)
    from ringfour.models import MobilityState
    def feedback(self):
        odom=self.get_pose()
        return MobilityState(pose=Pose(x=100,y=100,frame='map'),odometry_pose=odom,
            velocity={'linear':vars(self.get_velocity().linear),'angular':vars(self.get_velocity().angular)},
            stamp=self.now*.4,is_moving=True,distance_traveled=0.)
    monkeypatch.setattr(FakeRobot,'get_mobility_state',feedback)
    properties={}
    checks.test_odometry_spin_rotation_and_drift(fake_robot,recipe('odometry_spin'),lambda k,v:properties.update({k:v}))
    assert properties['measured.rotation_rad']==pytest.approx(2*math.pi,abs=.05)
    assert properties['measured.wall_seconds']>2*properties['measured.robot_seconds']


@pytest.mark.parametrize('linear,angular', [(.02,.1),(.05,.2),(.5,1.5)])
def test_defaults_respect_selected_profile_limits(linear,angular):
    value=movement.default_scenarios(live_welcome(),{'mobility':{'max_linear':linear,'max_angular':angular}})
    assert value['teleop']['command']['linear']<=linear
    assert value['teleop']['stop_speed']<value['teleop']['command']['linear']
    for name, recipe in value['movement_profiles'].items():
        if 'linear' in recipe:assert recipe['linear']<=linear
        if 'angular' in recipe:assert recipe['angular']<=angular
    validate_scenarios(value,live_welcome())


def test_recorded_fixture_sha_is_not_silently_rebound(monkeypatch,tmp_path):
    monkeypatch.setattr(server,'SCENARIOS',tmp_path)
    first=live_welcome().model_copy(update={'description_sha':'a'*64})
    settings=movement.default_scenarios(first)
    settings['profile']['description_sha']='a'*64
    server.test_settings_path(first).write_text(__import__('json').dumps(settings))
    changed=first.model_copy(update={'description_sha':'b'*64})
    result = server.test_settings(changed)
    assert result['requirements']['stale_model'] is True
    assert 'description_sha' not in result['settings']['profile']
    recorded = __import__('json').loads(server.test_settings_path(first).read_text())
    assert recorded['profile']['description_sha'] == 'a'*64


def test_precision_multiplier_changes_only_geometry_and_never_compounds():
    spin = recipe('odometry_spin')
    adjusted = movement.effective_movement_recipe('odometry_spin',spin,1.5)
    assert adjusted['max_drift'] == pytest.approx(.15)
    assert adjusted['angle_tolerance'] == pytest.approx(.45)
    assert spin['max_drift'] == .1
    assert movement.effective_movement_recipe('odometry_spin',spin,1.5) == adjusted
    for key in ('angular','turns','watchdog','stop_speed','stop_angular_speed','halt_timeout'):
        assert adjusted[key] == spin[key]
    loop = recipe('figure_eight')
    assert movement.effective_movement_recipe('figure_eight',loop,2)['position_tolerance'] == .4
    braking = recipe('braking')
    assert movement.effective_movement_recipe('braking',braking,5) == braking


@pytest.mark.parametrize('factor', [0,.5,5.1,True,float('inf'),float('nan'),'2'])
def test_invalid_precision_multiplier_is_rejected(factor):
    data=draft('odometry_spin');data['movement_tolerance']=factor
    with pytest.raises(ValueError):
        validate_scenarios(data,live_welcome())


def test_precision_setting_is_saved_per_robot_and_preserves_other_fixtures(tmp_path,monkeypatch):
    monkeypatch.setattr(server,'SCENARIOS',tmp_path)
    welcome=live_welcome()
    monkeypatch.setattr(server,'inspect_agent',lambda uri:welcome.model_dump(mode='json'))
    initial=server.inspect_test_settings({'uri':'fixture'})
    # A precision-only save must preserve existing drive/profile values.
    saved=server.save_test_settings({'uri':'fixture','settings':{'movement_tolerance':1.5},'expected_sha':initial['sha']})
    assert saved['settings']['movement_tolerance']==1.5
    assert saved['settings']['teleop']==initial['settings']['teleop']
    assert saved['settings']['movement_profiles']==initial['settings']['movement_profiles']
    assert server.test_settings(welcome)['settings']['movement_tolerance']==1.5
    other=welcome.model_copy(update={'profile':welcome.profile.model_copy(update={'name':'another-robot'})})
    assert server.test_settings(other)['settings']['movement_tolerance']==1


def test_movement_fixture_records_effective_limits_used_by_the_assertions():
    from test_conformance import harness
    request=SimpleNamespace(node=SimpleNamespace(get_closest_marker=lambda name:SimpleNamespace(args=['odometry_spin'])))
    properties={}
    data=draft('odometry_spin');data['movement_tolerance']=1.5
    result=harness.movement_profile_recipe.__wrapped__(request,data,None,lambda k,v:properties.update({k:v}))
    assert result['max_drift']==pytest.approx(.15)
    assert properties['movement.effective.max_drift']==result['max_drift']
    assert properties['movement.tolerance_multiplier']==1.5
    assert data['movement_profiles']['odometry_spin']['max_drift']==.1
