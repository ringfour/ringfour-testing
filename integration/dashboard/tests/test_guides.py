"""Operator guide coverage, precise category selection, and profile editing."""
import hashlib
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import server
from guides import all_guides, guide_for, resolve_selection


def test_every_selectable_test_has_complete_operator_guidance():
    entries = all_guides()
    import ast
    from guides import ROOT
    files = list((ROOT / "ringfour-testing/integration").glob("test_*.py")) + list((ROOT / "ringfour-testing/platform_tests").glob("test_*.py")) + list((ROOT / "missions/tests").glob("test_*.py"))
    expected = {f"{p.relative_to(ROOT)}::{n.name}" for p in files for n in ast.parse(p.read_text()).body
                if isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef)) and n.name.startswith("test_")}
    assert {g["id"] for g in entries} == expected
    assert sum(g['scope'] == 'robot' for g in entries) == sum(
        key.startswith('ringfour-testing/integration/') for key in expected)
    assert len({g['id'] for g in entries}) == len(entries)
    for guide in entries:
        assert all(guide[field] for field in ('title', 'what', 'why', 'fix', 'effects', 'categories'))
        assert isinstance(guide['profile_fields'], list)
        assert isinstance(guide['scenario_fields'], list)


def test_camera_category_selects_only_camera_checks_across_modules():
    targets, entries = resolve_selection(['cameras'])
    assert len(entries) == 4
    assert len({g['source'] for g in entries}) > 1
    assert all('cameras' in g['categories'] and not g['requires_motion'] for g in entries)
    assert targets == [g['id'] for g in entries]
    assert all('::' in target for target in targets)


def test_overlapping_categories_run_each_check_once_and_sdk_stays_offline():
    targets, entries = resolve_selection(['movement', 'manipulation', 'safety'])
    assert len(targets) == len(set(targets)) == len(entries)
    targets, entries = resolve_selection(['sdk'])
    assert targets == ['ringfour-testing/platform_tests']
    assert all(g['scope'] == 'sdk' for g in entries)
    check = all_guides()[0]
    assert resolve_selection([], [check['id']])[0] == [check['id']]
    with pytest.raises(ValueError):
        resolve_selection(['unknown'])
    with pytest.raises(ValueError):
        resolve_selection([], ['missing'])


def test_guides_resolve_parameterized_and_historical_junit_ids():
    entries = all_guides()
    guide = entries[0]
    assert guide_for(guide['id'] + '[left-arm]', entries) == guide
    historical = Path(guide['source']).stem + '::' + guide['function'] + '[left-arm]'
    assert guide_for(historical, entries) == guide
    assert guide_for('unknown::test_missing', entries) is None


@pytest.fixture
def editable_profile(tmp_path, monkeypatch):
    monkeypatch.setattr(server, 'PROFILE_DIRS', [tmp_path])
    path = tmp_path / 'fixture.yaml'
    path.write_text("name: fixture\nmodel: synthetic\nversion: '0'\nprotocol: 1\nframes:\n  map: map\n  odom: odom\n  base: base_link\n")
    return path


def test_profile_review_validates_schema_without_writing(editable_profile):
    original = editable_profile.read_text()
    draft = original.replace('name: fixture', 'name: reviewed')
    _, review = server.validate_profile_draft({'profile': str(editable_profile), 'yaml': draft})
    assert review['changed'] and review['valid']
    assert '+name: reviewed' in review['diff']
    assert editable_profile.read_text() == original
    with pytest.raises(ValueError):
        server.validate_profile_draft({'profile': str(editable_profile), 'yaml': 'protocol: invalid'})
    with pytest.raises(ValueError, match='read-only'):
        server.validate_profile_draft({'profile': '@live', 'yaml': draft})


def test_profile_save_checks_fingerprint_and_preserves_backup(editable_profile):
    original = editable_profile.read_text()
    draft = original.replace('name: fixture', 'name: reviewed')
    request = {'profile': str(editable_profile), 'yaml': draft, 'expected_sha': 'stale'}
    with pytest.raises(ValueError, match='changed since'):
        server.save_profile_draft(request)
    assert editable_profile.read_text() == original
    request['expected_sha'] = hashlib.sha256(original.encode()).hexdigest()
    result = server.save_profile_draft(request)
    assert result['saved']
    assert Path(result['backup']).read_text() == original
    assert editable_profile.read_text() == draft
    request['expected_sha'] = result['sha']
    assert not server.save_profile_draft(request)['saved']
    assert len(list(editable_profile.parent.glob('*.bak'))) == 1


def test_run_retains_original_guide_and_named_instances(tmp_path):
    import json
    directory = tmp_path / 'sample'
    directory.mkdir()
    guide = dict(all_guides()[0], title='Description when this run started')
    nodeid = guide['id'] + '[camera-one]'
    server.save_meta(directory, {'id': 'sample', 'status': 'running', 'started': 0})
    (directory / 'test-guide.json').write_text(json.dumps([guide]))
    (directory / 'events.jsonl').write_text(json.dumps({'event': 'collection', 'tests': [nodeid],
        'parameters': {nodeid: {'camera_name': 'camera-one'}}}) + '\n')
    case = server.Runner(tmp_path).snapshot('sample')['cases'][0]
    assert case['guide']['title'] == guide['title']
    assert case['parameters'] == {'camera_name': 'camera-one'}
    assert case['instance'] == 'camera-one'


def test_mission_helpers_are_separate_from_platform_and_robot_acceptance():
    from guides import ROOT
    if not (ROOT / "missions/tests").is_dir():
        pytest.skip("Optional missions checkout is absent")
    targets, entries = resolve_selection(["missions"])
    assert len(entries) >= 6
    assert all(g["scope"] == "missions" and g["suite"] == "missions" for g in entries)
    assert all(t.startswith("missions/tests/") for t in targets)
    assert not any(g["requires_motion"] for g in entries)
    platform_targets, platform = resolve_selection(["sdk"])
    assert platform_targets == ["ringfour-testing/platform_tests"]
    assert not set(g["id"] for g in entries) & set(g["id"] for g in platform)
