"""Acceptance boundaries and honest robot/offline coverage reporting."""
import json
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import server
from guides import all_guides, resolve_selection


def test_default_does_not_include_sdk_qualification_or_duplicate_motion():
    categories = [c['id'] for c in server.category_catalog()]
    targets, entries = resolve_selection(categories)
    assert entries and all(g['scope'] == 'robot' and g['suite'] == 'compatibility' for g in entries)
    names = {g['function'] for g in entries}
    assert 'test_follow_waypoints_and_converge' in names
    assert 'test_navigate_to_configured_goal' in names
    assert 'test_progress_climbs_monotonically' not in names
    assert 'test_nav_status_phase_is_typed' not in names
    assert 'test_figure_eight_closes_both_loops' not in names
    assert 'ringfour-testing/platform_tests' not in targets
    assert {'test_watchdog_halts_on_command_silence', 'test_killed_client_halts_robot_within_watchdog',
            'test_navigation_cancellation_halts', 'test_stop_watch_halts_motion_without_client_polling'} <= names


@pytest.mark.parametrize('suite', ['qualification', 'diagnostics', 'regression', 'sdk'])
def test_optional_suites_are_explicit_and_single_checks_remain_selectable(suite):
    categories = [c['id'] for c in server.category_catalog()]
    targets, entries = resolve_selection(categories, suite=suite)
    assert entries and all(g['suite'] == suite for g in entries)
    guide = entries[0]
    assert resolve_selection([], [guide['id']])[0] == [guide['id']]
    if suite == 'sdk':
        assert targets == ['ringfour-testing/platform_tests']


def test_all_suites_preserves_the_full_catalog_without_duplicate_targets():
    targets, entries = resolve_selection([c['id'] for c in server.category_catalog()], suite='all')
    assert len(targets) == len(set(targets))
    assert {g['id'] for g in entries} == {g['id'] for g in all_guides()}
    with pytest.raises(ValueError, match='Unknown test suite'):
        resolve_selection(['movement'], suite='missing')


def test_historical_mixed_run_separates_robot_and_offline_results(tmp_path):
    directory = tmp_path / 'mixed'
    directory.mkdir()
    server.save_meta(directory, {'id':'mixed', 'status':'passed', 'started':0})
    entries = all_guides()
    robot = next(g for g in entries if g['scope'] == 'robot')
    sdk = next(g for g in entries if g['scope'] == 'sdk')
    (directory / 'test-guide.json').write_text(json.dumps([robot, sdk]))
    (directory / 'results.xml').write_text(f'<testsuite><testcase classname="{Path(robot["source"]).stem}" name="{robot["function"]}"/><testcase classname="{Path(sdk["source"]).stem}" name="{sdk["function"]}"/></testsuite>')
    runner = server.Runner(tmp_path)
    counts = runner.snapshot('mixed')['scope_counts']
    assert counts['robot']['passed'] == counts['sdk']['passed'] == 1
    assert runner.summaries()[0]['scope_counts'] == counts


def test_unsupported_grouping_preserves_raw_checks_and_never_hides_failures():
    def case(name, instance='arm-a', status='skipped', coverage='not_applicable'):
        return {'nodeid':name, 'instance':instance, 'status':status, 'coverage':coverage,
                'detail':'NOT APPLICABLE: no Cartesian tip', 'guide':{'scope':'robot', 'title':name}}
    cases = [case('fk'), case('ik'), case('fk-b','arm-b'), case('failed', status='failed'),
             case('no-fixture', coverage='not_configured')]
    rows = server.display_cases(cases)
    assert len(rows) == 4
    assert rows[0]['related_checks'] == ['fk', 'ik']
    assert len(cases) == 5 and 'related_checks' not in cases[0]
    assert any(c['status'] == 'failed' for c in rows)
    assert any(c['coverage'] == 'not_configured' for c in rows)


def test_live_event_skip_locations_do_not_split_same_unsupported_feature():
    cases = [{"nodeid":name, "instance":"leg", "status":"skipped", "coverage":"not_applicable",
              "guide":{"scope":"robot", "categories":[category]},
              "detail":repr((source, line, "Skipped: NOT APPLICABLE: leg has no Cartesian tip"))}
             for name, source, line, category in [("fk","fk.py",10,"kinematics"),("chain","model.py",20,"description")]]
    rows = server.display_cases(cases)
    assert len(rows) == 1
    assert rows[0]["guide"]["title"] == "leg has no Cartesian tip"
    assert rows[0]["guide"]["categories"] == ["kinematics", "description"]
    assert rows[0]["related_checks"] == ["fk", "chain"]
