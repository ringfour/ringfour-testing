"""Startup/profile regressions formerly tied to base101's live exit suite.

These need no robot; the synthetic names intentionally describe no real model.
"""
import pytest
import yaml
from ringfour.models import ProfileInfo
from ringfour.enums import Controller, GroupKind
from ringfour_agent.bootstrap import build_agent
from ringfour_agent.groups import GroupValidationError
from ringfour_agent.models.profile import RobotProfile
from ringfour_agent.profile import Profile, ProfileError, load_profile


class StartupRos:
    def description(self):
        return '''<robot name="fixture"><link name="root"/><link name="flange"/><link name="sensor"/>
<joint name="hinge" type="revolute"><parent link="root"/><child link="flange"/></joint>
<joint name="sensor-mount" type="fixed"><parent link="root"/><child link="sensor"/></joint>
<ros2_control name="tool" type="system"><joint name="jaw"/></ros2_control></robot>'''

    def loaded_controllers(self):
        return {'position':{'state':'active','type':'fixture','joints':('hinge',)},
                'tool':{'state':'active','type':'fixture','joints':('jaw',)}}

    def declared_controllers(self):
        return frozenset({'position','trajectory','tool'})

    def joint_snapshot(self):
        return {}


def data(tip='flange'):
    return {'name':'fixture','model':'synthetic','version':'1','protocol':1,
            'frames':{'map':'map-fixture','odom':'odom-fixture','base':'root'},
            'groups':{'appendage':{'kind':'arm','tip':tip,
                       'controllers':{'stream':'position','trajectory':'trajectory','tool':'tool'},
                       'tool':{'type':'fixture','joint':'jaw','open':1.,'closed':0.}}}}


def profile(tip):
    spec=RobotProfile.model_validate(data(tip))
    return Profile(spec=spec,info=ProfileInfo(name=spec.name,version=spec.version,
                   model=spec.model,capabilities=spec.capabilities))


def test_missing_tip_names_the_profile_field(tmp_path):
    """Missing tip names the profile field

    Categories: sdk
    What it checks: Verifies that missing tip names the profile field. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that invalid tips and profile migrations are caught before an agent boots.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the group startup validation code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    with pytest.raises(GroupValidationError) as exc:
        build_agent(profile('missing'),unix_path=None,audit_dir=tmp_path,ros=StartupRos(), allow_unverified_stops=True)
    assert 'missing' in str(exc.value)
    assert 'groups.appendage.tip' in str(exc.value)
    assert 'not a link' in str(exc.value)


def test_tip_on_another_branch_is_rejected(tmp_path):
    """Tip on another branch is rejected

    Categories: sdk
    What it checks: Verifies that tip on another branch is rejected. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that invalid tips and profile migrations are caught before an agent boots.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the group startup validation code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    with pytest.raises(GroupValidationError,match='not downstream'):
        build_agent(profile('sensor'),unix_path=None,audit_dir=tmp_path,ros=StartupRos(), allow_unverified_stops=True)


def test_valid_tip_and_resolved_controller_boot(tmp_path):
    """Valid tip and resolved controller boot

    Categories: sdk
    What it checks: Verifies that valid tip and resolved controller boot. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that invalid tips and profile migrations are caught before an agent boots.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the group startup validation code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    _,ctx=build_agent(profile('flange'),unix_path=str(tmp_path/'agent.sock'),audit_dir=tmp_path,ros=StartupRos(), allow_unverified_stops=True)
    try:
        group=ctx.groups['appendage']
        assert group.kind is GroupKind.ARM
        assert group.tip=='flange' and group.tool_joint=='jaw'
        assert group.controllers[Controller.TRAJECTORY]=='trajectory'
    finally:
        ctx.audit.close()


def test_retired_profile_section_explains_migration(tmp_path):
    """Retired profile section explains migration

    Categories: sdk
    What it checks: Verifies that retired profile section explains migration. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that invalid tips and profile migrations are caught before an agent boots.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the group startup validation code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    stale=data()
    stale['manipulation']={'arms':{'old':{'joints':['hinge']}}}
    path=tmp_path/'stale.yaml'
    path.write_text(yaml.safe_dump(stale))
    with pytest.raises(ProfileError) as exc:
        load_profile(path)
    assert 'groups' in str(exc.value) and 'manipulation' in str(exc.value)
