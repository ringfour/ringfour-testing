"""Map goal selection rejects unknown cells, obstacles and disconnected space."""
import math
from types import SimpleNamespace
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from navigation import choose_map_goals, NAVIGATION_FIELDS
from movement import default_scenarios
from ringfour import Pose
from test_movement_config import live_welcome


def grid(data=None, origin=None):
    if data is None:data=np.zeros((100,100),dtype=np.int8)
    return SimpleNamespace(data=data,height=data.shape[0],width=data.shape[1],resolution=.1,
                           origin=origin or Pose(x=-5,y=-5,frame='test_map'))


def recipe():
    return {'auto':True,**{key:value for key,_,value in NAVIGATION_FIELDS},'clearance':.3}


def test_navigation_defaults_follow_binding_and_debug_is_ready_by_default():
    w=live_welcome(('mobility','teleop','slam'))
    assert default_scenarios(w)['debug'] is True
    assert 'navigation' not in default_scenarios(w,{'mobility':{'odom_topic':'/odom'}})
    assert default_scenarios(w,{'mobility':{'nav2_navigate_to_pose':'/navigate'}})['navigation']['auto'] is True


@pytest.mark.parametrize('barrier',[100,-1,10])
def test_goals_remain_in_connected_free_space_and_clear_of_barriers(barrier):
    data=np.zeros((100,100),dtype=np.int8);data[:,55]=barrier
    current=Pose(x=0,y=0,frame='test_map')
    goals=choose_map_goals(grid(data),current,recipe())
    assert len(goals)==2
    for g in goals:
        assert g['x'] < .2 # Other side remains free but is disconnected.
        assert .75 <= math.hypot(g['x'],g['y']) <= 2
        row,col=math.floor((g['y']+5)/.1),math.floor((g['x']+5)/.1)
        assert np.all(data[row-3:row+4,col-3:col+4]==0)
    assert math.hypot(goals[0]['x']-goals[1]['x'],goals[0]['y']-goals[1]['y'])>=.75


def test_rotated_map_origin_is_respected():
    origin=Pose(x=10,y=20,yaw=math.pi/2,frame='test_map')
    current=Pose(x=5,y=25,frame='test_map')
    goals=choose_map_goals(grid(origin=origin),current,recipe())
    assert all(g['frame']=='test_map' and .75 <= math.hypot(g['x']-5,g['y']-25)<=2 for g in goals)


def test_no_goals_are_invented_when_pose_or_clearance_is_invalid():
    current=Pose(x=0,y=0,frame='test_map');data=np.full((100,100),-1,dtype=np.int8)
    with pytest.raises(ValueError,match='clearance'):
        choose_map_goals(grid(data),current,recipe())
    with pytest.raises(ValueError,match='outside'):
        choose_map_goals(grid(),Pose(x=-5.01,y=0,frame='test_map'),recipe())
    with pytest.raises(ValueError,match='same frame'):
        choose_map_goals(grid(),Pose(x=0,y=0,frame='odom'),recipe())


def test_debug_fixture_requires_no_motion_opt_in_but_honors_explicit_opt_out():
    from test_conformance import harness
    request=SimpleNamespace(node=SimpleNamespace(user_properties=[]))
    welcome = SimpleNamespace(features={})
    harness.debug_enabled.__wrapped__(request,{},welcome)
    with pytest.raises(pytest.skip.Exception,match='explicitly disabled'):
        harness.debug_enabled.__wrapped__(request,{'debug':False},welcome)


def test_fault_checks_require_connection_permissions():
    from test_conformance import harness
    request = SimpleNamespace(node=SimpleNamespace(user_properties=[]))
    reader = SimpleNamespace(features={'access': {'permissions': ['observe']}})
    with pytest.raises(pytest.skip.Exception, match='lacks permissions'):
        harness.debug_enabled.__wrapped__(request, {}, reader)
    operator = SimpleNamespace(features={'access': {'permissions': ['configure', 'stop', 'reset-inhibit']}})
    harness.debug_enabled.__wrapped__(request, {}, operator)
