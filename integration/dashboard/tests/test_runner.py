"""Offline checks for process ownership, event reduction and endpoint routing."""
import importlib.util
import json
import sys
import time
from pathlib import Path

import pytest

HERE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE))
import server


def saved_run(tmp_path, events, status="running"):
    directory = tmp_path / "sample"
    directory.mkdir()
    server.save_meta(directory, {"id": "sample", "status": status, "started": time.time()})
    (directory / "events.jsonl").write_text("\n".join(json.dumps(e) for e in events) + '\n{"partial":')
    return server.Runner(tmp_path), directory


def test_setup_errors_teardown_errors_and_skips_are_not_passes(tmp_path):
    ids = ["a", "b", "c"]
    events = [{"event": "collection", "tests": ids}]
    for name, phase, outcome in [("a", "setup", "failed"), ("b", "call", "passed"),
                                  ("b", "teardown", "failed"), ("c", "setup", "skipped")]:
        events.append({"event": "start", "nodeid": name})
        events.append({"event": "report", "nodeid": name, "when": phase,
                       "outcome": outcome, "duration": .1, "detail": "reason"})
    for name in ("a", "c"):
        events.append({"event": "report", "nodeid": name, "when": "teardown",
                       "outcome": "passed", "duration": .1, "detail": ""})
    runner, _ = saved_run(tmp_path, events, "failed")
    snapshot = runner.snapshot("sample")
    assert [c["status"] for c in snapshot["cases"]] == ["error", "error", "skipped"]
    assert snapshot["cases"][0]["duration"] == pytest.approx(.2)
    assert snapshot["current"] is None
    with pytest.raises(ValueError):
        runner.snapshot("../sample")


def test_cancelled_pending_cases_are_not_run(tmp_path):
    runner, _ = saved_run(tmp_path, [{"event": "collection", "tests": ["a", "b"]},
                                   {"event": "start", "nodeid": "a"}], "cancelled")
    assert [c["status"] for c in runner.snapshot("sample")["cases"]] == ["not run", "not run"]


def test_junit_collection_error_fallback(tmp_path):
    runner, directory = saved_run(tmp_path, [], "error")
    (directory / "results.xml").write_text('<testsuites><testsuite><testcase name="collection"><error message="ImportError">traceback</error></testcase></testsuite></testsuites>')
    case = runner.snapshot("sample")["cases"][0]
    assert case["status"] == "error"
    assert "ImportError" in case["detail"]


def test_any_live_profile_can_run_read_checks_and_yaml_must_match(tmp_path, monkeypatch):
    # This contract must not depend on an installed robot's profile catalog.
    selected = tmp_path / "selected.yaml"
    selected.write_text("name: synthetic-ground-robot\nmodel: synthetic\nversion: '1'\n")
    profile_id = str(selected)
    monkeypatch.setattr(server, "robot_profiles", lambda: [{"id": profile_id, "name": "synthetic", "robot": "fixture"}])
    request = {"suites": ["test_phase2_exit"], "uri": "ws://robot.local:10101", "profile": profile_id}
    monkeypatch.setattr(server, "inspect_agent", lambda _: {"protocol": 1, "profile": {"name": "aerial-robot", "model": "quadrotor", "version": "1", "capabilities": []}, "capabilities": []})
    runner = server.Runner(tmp_path)
    with pytest.raises(ValueError, match="does not match"):
        runner.start(request)
    assert list(tmp_path.iterdir()) == [selected]
    monkeypatch.setattr(server.subprocess, "Popen", lambda command, **kwargs: (_ for _ in ()).throw(OSError("spawn sentinel")))
    request["profile"] = "@live"
    with pytest.raises(OSError, match="spawn sentinel"):
        runner.start(request)
    meta = runner.runs()[0]
    assert meta["profile"] == "loaded: aerial-robot"
    assert meta["motion_enabled"] is False
    assert meta["status"] == "error"


def test_external_endpoint_preserves_host_path_and_scheme(monkeypatch):
    spec = importlib.util.spec_from_file_location("e2e_fixtures", HERE.parent / "conftest.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    uri = "wss://robot.local:10443/agent"
    monkeypatch.setenv("RINGFOUR_AGENT_TCP", uri)
    monkeypatch.delenv("RINGFOUR_AGENT_UNIX", raising=False)
    endpoint = module._external_endpoints("RINGFOUR_AGENT_UNIX", "RINGFOUR_AGENT_TCP")
    assert endpoint.tcp_uri == uri
    assert endpoint.unix_uri == uri
    assert endpoint.tcp_port == 10443


def test_runner_owns_one_subprocess_and_forces_external_mode(tmp_path, monkeypatch):
    captured = {}
    gate = __import__('threading').Event()
    class Process:
        pid = 123456789
        def __init__(self, command, **kwargs):
            captured.update(command=command, **kwargs)
        def wait(self, timeout=None):
            gate.wait(3)
            return 0
        def poll(self):
            return None if not gate.is_set() else 0
    monkeypatch.setattr(server.subprocess, "Popen", Process)
    monkeypatch.setenv("RINGFOUR_STUB_TCP", "ws://stale:1234")
    monkeypatch.setenv("RINGFOUR_AGENT_UNIX", "/stale.sock")
    runner = server.Runner(tmp_path)
    request = {"suites": ["sdk"], "uri": "", "stub": ""}
    try:
        result = runner.start(request)
        assert captured["env"]["RINGFOUR_EXTERNAL_STACK"] == "1"
        assert "RINGFOUR_STUB_TCP" not in captured["env"]
        assert "RINGFOUR_AGENT_UNIX" not in captured["env"]
        assert captured["start_new_session"] is True
        assert "ringfour-testing/platform_tests" in captured["command"]
        with pytest.raises(ValueError, match="already active"):
            runner.start(request)
        with pytest.raises(ValueError):
            runner.start({"suites": ["--bad"]})
        assert runner.active == result["id"]
    finally:
        gate.set()
    for _ in range(100):
        if runner.active is None:
            break
        time.sleep(.01)
    assert runner.snapshot(result["id"])["status"] == "passed"


def wait_until(predicate, seconds=10):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(.05)
    raise AssertionError("Runner did not reach the expected state")


def test_real_pytest_failure_skip_and_teardown_reports(tmp_path, monkeypatch):
    source = tmp_path / "test_outcomes.py"
    source.write_text('''import pytest
@pytest.fixture
def bad_cleanup():
    yield
    raise RuntimeError("cleanup broke")
def test_pass():
    assert True
def test_failure():
    assert False, "expected failure"
def test_skip():
    pytest.skip("missing fixture")
def test_teardown(bad_cleanup):
    assert True
''')
    monkeypatch.setattr(server, "catalog", lambda: [{"id": "sdk", "path": str(source)}])
    runner = server.Runner(tmp_path / "runs")
    result = runner.start({"suites": ["sdk"], "uri": "", "stub": ""})
    wait_until(lambda: runner.active is None)
    snapshot = runner.snapshot(result["id"])
    assert snapshot["status"] == "failed"
    assert [c["status"] for c in snapshot["cases"]] == ["passed", "failed", "skipped", "error"]
    assert "cleanup broke" in snapshot["cases"][-1]["detail"]
    assert (tmp_path / "runs" / result["id"] / "results.xml").exists()


def test_real_cancel_preserves_partial_results(tmp_path, monkeypatch):
    source = tmp_path / "test_slow.py"
    source.write_text('import time\ndef test_first():\n    assert True\ndef test_slow():\n    time.sleep(60)\ndef test_last():\n    assert True\n')
    monkeypatch.setattr(server, "catalog", lambda: [{"id": "sdk", "path": str(source)}])
    runner = server.Runner(tmp_path / "runs")
    result = runner.start({"suites": ["sdk"], "uri": "", "stub": ""})
    try:
        wait_until(lambda: (runner.snapshot(result["id"])["current"] or "").endswith("test_slow"))
        assert runner.cancel()["status"] == "cancelling"
        wait_until(lambda: runner.active is None)
        snapshot = runner.snapshot(result["id"])
        assert snapshot["status"] == "cancelled"
        assert [c["status"] for c in snapshot["cases"]] == ["passed", "not run", "not run"]
    finally:
        if runner.active:
            runner.cancel()


def test_collection_errors_are_visible_even_with_partial_collection(tmp_path):
    runner,directory=saved_run(tmp_path,[{'event':'collection','tests':['collected']}],'error')
    (directory/'results.xml').write_text('<testsuites><testsuite><testcase classname="" name="broken.module"><error message="collection failure">ImportError</error></testcase></testsuite></testsuites>')
    cases=runner.snapshot('sample')['cases']
    assert [c['status'] for c in cases]==['not run','error']
    assert 'ImportError' in cases[-1]['detail']


def test_invalid_junit_remains_reviewable(tmp_path):
    runner,directory=saved_run(tmp_path,[],'error')
    (directory/'results.xml').write_text('<broken')
    case=runner.snapshot('sample')['cases'][0]
    assert case['status']=='error'
    assert 'Invalid JUnit report' in case['detail']
