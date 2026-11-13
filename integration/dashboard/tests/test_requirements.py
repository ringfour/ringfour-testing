"""Prerequisite editing preserves fixtures and never invents motion or feedback."""
import copy
from types import SimpleNamespace
from contextlib import nullcontext
import sys
from pathlib import Path

import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import server
from conformance import validate_scenarios
from ringfour.models.protocol import Welcome


def welcome():
    return Welcome.model_validate({'protocol':1,'profile':{'name':'fixture','model':'arbitrary','version':'1','capabilities':['groups','mobility','teleop','slam','places','actions']},
       'capabilities':['groups','mobility','teleop','slam','places','actions'], 'description_sha':'a'*64,
       'groups':{'appendage':{'kind':'arm','joints':['axis-A','slide'],'control_modes':['trajectory']}},
       'actions':{'set_signal':{'parameters':{'value':{'type':'float','min':0,'max':10,'default':None}},'is_motion':False,'max_rate':10}},'instances':{'groups':['appendage']}})


def fixture_data():
    return {'groups':{'appendage':{'allow_sessions':True,'waypoints':[{'axis-A':.1,'slide':.2},{'axis-A':0,'slide':0}],
             'clear_configurations':[{'axis-A':0,'slide':0}], 'collision_configurations':[{'axis-A':.8,'slide':.8}],
             'clear_state':{'independent-mount':.2},'collision_state':{'independent-mount':.3},'max_check_seconds':.5}},
            'mapping':{'save_localize':True},'artifacts':{'snapshot_dir':'/tmp/recorded-geometry'},
            'description_change':{'command':['python3','change.py'],'restore_command':['python3','restore.py'],'timeout':30},
            'actions':{'set_signal':{'arguments':{'value':2},'restore_arguments':{'value':0},'feedback':{'path':'sensors.board.signal','expected':2,'tolerance':.01},'timeout':5}},
            'tolerances':{'joint_limits':{'slide':.001}},'debug':True}


@pytest.fixture
def editable_settings(tmp_path,monkeypatch):
    w=welcome()
    monkeypatch.setattr(server,'SCENARIOS',tmp_path)
    monkeypatch.setattr(server,'inspect_agent',lambda uri:w.model_dump(mode='json'))
    monkeypatch.setattr(server,'matching_profile',lambda w:None)
    import ringfour
    group=SimpleNamespace(joints={'axis-A':SimpleNamespace(limits=(-1,1)),'slide':SimpleNamespace(limits=(0,1))})
    monkeypatch.setattr(ringfour,'Robot',lambda uri:nullcontext(SimpleNamespace(groups={'appendage':group})))
    return w


def test_all_requirements_can_be_saved_and_round_trip_without_erasing_other_sections(editable_settings):
    initial=server.inspect_test_settings({'uri':'fixture'})
    saved=server.save_test_settings({'uri':'fixture','settings':fixture_data(),'expected_sha':initial['sha']})
    for key,value in fixture_data().items():assert saved['settings'][key]==value
    assert saved['settings']['profile']['description_sha']=='a'*64
    assert saved['requirements']['groups'][0]['sessions_configured'] is True
    assert saved['requirements']['groups'][0]['waypoints']==2
    newer=server.save_test_settings({'uri':'fixture','settings':{'movement_tolerance':2},'expected_sha':saved['sha']})
    for key,value in fixture_data().items():assert newer['settings'][key]==value


def test_out_of_limit_targets_are_rejected_before_writing(editable_settings):
    initial=server.inspect_test_settings({'uri':'fixture'})
    data=fixture_data();data['groups']['appendage']['waypoints'][0]['slide']=5
    with pytest.raises(ValueError,match='exceeds live limits'):
        server.save_test_settings({'uri':'fixture','settings':data,'expected_sha':initial['sha']})
    assert not server.test_settings_path(editable_settings).exists()


@pytest.mark.parametrize('change', ['missing_joint','missing_restore','missing_feedback','unknown_action','out_of_bounds_argument'])
def test_incomplete_requirements_get_specific_validation_errors(change):
    w=welcome();data={'version':1,'profile':{'name':'fixture','model':'arbitrary'},**fixture_data()}
    if change=='missing_joint':del data['groups']['appendage']['waypoints'][0]['slide']
    elif change=='missing_restore':del data['description_change']['restore_command']
    elif change=='missing_feedback':del data['actions']['set_signal']['feedback']
    elif change=='unknown_action':data['actions']['another']=data['actions'].pop('set_signal')
    else:data['actions']['set_signal']['arguments']['value']=100
    with pytest.raises(ValueError):validate_scenarios(data,w)


def test_action_requires_observed_change_and_restores_feedback():
    import test_capability_sensors as checks
    state={'value':0};calls=[]
    def invoke(**args):calls.append(args);state['value']=args['value'];return {'published':True}
    robot=SimpleNamespace(actions=SimpleNamespace(set_signal=invoke),sensors=SimpleNamespace(board=SimpleNamespace(reading=lambda signal:{'value':state['value']})))
    request=SimpleNamespace(node=SimpleNamespace(user_properties=[]),getfixturevalue=lambda name:None)
    data=fixture_data();properties={}
    checks.test_vendor_action_changes_feedback(robot,'set_signal',welcome(),request,data,lambda k,v:properties.update({k:v}))
    assert state['value']==0 and calls==[{'value':2},{'value':0}]
    assert properties['action.observed_value']==2
    data['actions']['set_signal']['feedback']['expected']=0
    with pytest.raises(pytest.skip.Exception,match='observable state change'):
        checks.test_vendor_action_changes_feedback(robot,'set_signal',welcome(),request,data,lambda *a:None)
    assert len(calls)==2


def test_suite_descriptions_are_specific_and_available_in_the_catalog():
    from suites import SUITES
    assert len(SUITES)==7 and all(len(s['description'])>40 for s in SUITES)
    assert 'synthetic' in next(s['description'] for s in SUITES if s['id']=='sdk')
