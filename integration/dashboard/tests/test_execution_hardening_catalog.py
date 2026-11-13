"""Shared hardening checks remain discoverable and reject incomplete robot faults."""
import copy
import sys
from pathlib import Path
import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from guides import all_guides
from suites import suite_for
from test_conformance import scenario_fixture
from conformance import validate_scenarios


def test_live_hardening_checks_are_selectable_diagnostics():
    expected={
        'test_lost_motion_reply_reconciles_without_duplicate_dispatch',
        'test_agent_restart_retains_results_and_rejects_old_epoch',
        'test_killed_mobility_client_verifies_stop_and_releases',
        'test_operation_wait_and_observer_exit_do_not_cancel',
        'test_joint_guard_rejects_unsafe_entry_and_latches',
        'test_healthy_lease_survives_slow_observation',
        'test_group_lease_expiry_stops_a_live_nonrenewing_connection',
        'test_killed_group_client_stops_and_releases',
        'test_feedback_loss_requires_quarantine_and_local_recovery',
        'test_unverified_network_identity_cannot_recover',
    }
    entries={g['function']:g for g in all_guides()}
    for name in expected:
        assert entries[name]['scope']=='robot'
        assert entries[name]['suite']=='diagnostics'
        assert suite_for(name,'robot')=='diagnostics'
        assert 'safety' in entries[name]['categories']
    assert entries['test_group_lease_expiry_stops_a_live_nonrenewing_connection']['requires_motion']
    assert not entries['test_healthy_lease_survives_slow_observation']['requires_motion']


@pytest.mark.parametrize('hardening',[
    {'lease_expiry':True,'client_death':False},
    {'feedback_loss':{'reviewed':True,'command':['fixture','disable'],
                      'restore_command':['fixture','restore'],
                      'recovery_command':['operator','recover','{resource}'],'timeout':30}},
])
def test_fault_fixtures_are_independent_of_robot_and_joint_names(hardening):
    welcome,data=scenario_fixture()
    data['debug']=True;data['groups']['torso']['hardening']=hardening
    assert validate_scenarios(data,welcome)==data


@pytest.mark.parametrize('change',[
    'unknown_fault','string_flag','missing_restore','shell_string','unreviewed_type','missing_debug','invalid_timeout',
])
def test_fault_configuration_rejects_incomplete_effectful_commands(change):
    welcome,data=scenario_fixture();data['debug']=True
    fault={'reviewed':True,'command':['fixture','disable'],'restore_command':['fixture','restore'],
           'recovery_command':['operator','recover','{resource}'],'timeout':30}
    hardening={'feedback_loss':fault};data['groups']['torso']['hardening']=hardening
    if change=='unknown_fault':hardening['invented']=True
    if change=='string_flag':hardening['lease_expiry']='yes'
    if change=='missing_restore':fault.pop('restore_command')
    if change=='shell_string':fault['command']='disable feedback'
    if change=='unreviewed_type':fault['reviewed']='true'
    if change=='missing_debug':data.pop('debug')
    if change=='invalid_timeout':fault['timeout']=float('inf')
    with pytest.raises(ValueError):validate_scenarios(data,welcome)


@pytest.mark.parametrize('opt_in,inhibited,coverage', [
    (False, True, 'disabled'), (True, True, 'not_configured'), (True, False, None),
])
def test_motion_preflight_distinguishes_opt_in_and_existing_inhibit(monkeypatch, opt_in, inhibited, coverage):
    from types import SimpleNamespace
    from test_conformance import harness
    monkeypatch.setenv('RINGFOUR_TEST_MOTION', '1' if opt_in else '0')
    request = SimpleNamespace(node=SimpleNamespace(user_properties=[]))
    welcome = SimpleNamespace(features={'access': {'permissions': ['operate', 'stop']}})
    robot = SimpleNamespace(get_estop_engaged=lambda: inhibited)
    if coverage is None:
        harness.motion_enabled.__wrapped__(request, welcome, robot)
        assert request.node.user_properties == []
    else:
        with pytest.raises(pytest.skip.Exception):
            harness.motion_enabled.__wrapped__(request, welcome, robot)
        assert ('coverage', coverage) in request.node.user_properties


@pytest.mark.parametrize('section,fixture', [
    ('lifecycle', {'reviewed':True, 'probe_command':['fixture','probe'], 'restart_command':['fixture','restart']}),
    ('mobility_fault', {'reviewed':True, 'setup_command':['fixture','isolate'], 'restore_command':['fixture','restore'], 'command':{'angular':.04}, 'watchdog':.3}),
])
@pytest.mark.parametrize('fault', [None,'not-reviewed','shell-string','no-debug','nonfinite-timeout'])
def test_lifecycle_faults_require_reviewed_portable_argv(section, fixture, fault):
    welcome,data=scenario_fixture()
    # Synthetic identity/capabilities only: no XLeRobot-specific test binding.
    welcome=welcome.model_copy(update={'capabilities':['groups','teleop','mobility']})
    data['debug']=True;data[section]=copy.deepcopy(fixture)
    if fault=='not-reviewed':data[section]['reviewed']=False
    if fault=='shell-string':data[section][next(key for key in fixture if key.endswith('_command'))]='unsafe shell string'
    if fault=='no-debug':data.pop('debug')
    if fault=='nonfinite-timeout':data[section]['timeout']=float('inf')
    if fault:
        with pytest.raises(ValueError):validate_scenarios(data,welcome)
    else:
        assert validate_scenarios(data,welcome)==data


def test_submission_fault_contracts_have_dashboard_guides():
    from pathlib import Path
    import ast
    path=Path(__file__).resolve().parents[4]/'ringfour-testing/platform_tests/test_submission_reconciliation_contract.py'
    names={node.name for node in ast.parse(path.read_text()).body if isinstance(node,ast.FunctionDef) and node.name.startswith('test_')}
    entries={guide['function']:guide for guide in all_guides()}
    assert names
    assert all(entries[name]['scope']=='sdk' and entries[name]['suite']=='sdk' for name in names)
