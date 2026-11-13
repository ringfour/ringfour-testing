#!/usr/bin/env python3
"""Local test console. Run with the SDK virtualenv; no web dependencies."""
from __future__ import annotations

import argparse
import ast
import difflib
import hashlib
import json
import os
import re
import signal
import subprocess
import sys
import threading
import tempfile
import shutil
import time
import uuid
import xml.etree.ElementTree as ET
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse

ROOT = Path(__file__).resolve().parents[3]
HERE = Path(__file__).resolve().parent
RESULTS = ROOT / "ringfour-testing" / "test-results" / "dashboard"
sys.path.insert(0, str(ROOT / "ringfour-sdk"))
sys.path.insert(0, str(ROOT / "ringfour-testing/integration"))
PROFILE_DIRS = []
PROFILE_LOCK = threading.Lock()
IMPORTED_PROFILES = HERE / "configurations" / "profiles"
from suites import SUITES
from guides import all_guides, category_catalog, guide_for, resolve_selection
from requirements import requirement_metadata, group_state
from navigation import NAVIGATION_FIELDS, live_map_goals
from movement import MOVEMENT_PROFILES, default_scenarios, DEFAULT_TELEOP, DRIVE_FIELDS


def save_meta(directory, meta):
    temporary = directory / "run.json.tmp"
    temporary.write_text(json.dumps(meta, indent=2))
    temporary.replace(directory / "run.json")


def catalog():
    suites = [{"id": "sdk", "label": "Platform · offline", "path": "ringfour-testing/platform_tests", "count": None}]
    if (ROOT / "missions/tests").is_dir():
        suites.append({"id": "missions", "label": "Mission recipes · offline", "path": "missions/tests", "count": None})
    labels = {
        "test_phase1_exit": "01 · Connection & handshake",
        "test_phase2_exit": "02 · State & transforms",
        "test_phase3_exit": "03 · Teleop & watchdog",
        "test_phase4_exit": "04 · Safety, watches & audit",
        "test_phase5_exit": "05 · Navigation & mapping",
        "test_phase6_exit": "06 · Arm & tools",
        "test_phase6_description_exit": "06 · Robot description",
        "test_phase6_groups_exit": "06 · Controller groups",
        "test_phase6_kinematics_exit": "06 · Kinematics & collision",
        "test_phase6_motion_exit": "06 · Motion & trajectories",
        "test_capability_sensors": "Sensors & platform interfaces",
        "test_movement_profiles": "Movement test profiles",
    }
    for path in sorted((ROOT / "ringfour-testing/integration").glob("test_*.py")):
        tree = ast.parse(path.read_text())
        suites.append({"id": path.stem, "label": labels.get(path.stem, path.stem),
                       "path": str(path.relative_to(ROOT)),
                       "count": sum(isinstance(n, ast.FunctionDef) and n.name.startswith("test_") for n in ast.walk(tree))})
    return suites


def robot_profiles():
    files = list((ROOT / "robots").glob("*/profiles/*.yaml"))
    files.extend(IMPORTED_PROFILES.glob("*.yaml"))
    for directory in PROFILE_DIRS:
        files.extend(directory.glob("*.yaml"))
    return [{"id": str(p.relative_to(ROOT)) if p.is_relative_to(ROOT) else str(p),
             "robot": p.parents[1].name, "name": p.stem}
            for p in sorted(set(files)) if not p.name.endswith(".conformance.yaml")]


def profile(path, welcome=None):
    if path == "@live":
        import yaml
        # This is deliberately labelled as a handshake snapshot: it is
        # not the agent's original ROS interface configuration.
        data = {k: v for k, v in (welcome or {}).items() if k != "description"}
        return {"path": "@live", "yaml": yaml.safe_dump(data, sort_keys=False) if welcome else
                "# Inspect the running robot to load its handshake snapshot.\n", "data": data}
    allowed = {p["id"] for p in robot_profiles()}
    if path not in allowed:
        raise ValueError("Select a configured profile file or the loaded agent.")
    import yaml
    text = (ROOT / path).read_text()
    return {"path": path, "yaml": text, "data": yaml.safe_load(text),
            "sha": hashlib.sha256(text.encode()).hexdigest()}


def import_profile(request):
    draft = request.get("yaml")
    if not isinstance(draft, str) or not draft.strip() or len(draft.encode()) > 256000:
        raise ValueError("Choose a nonempty YAML profile under 256 KB")
    import yaml
    agent_dir = ROOT / "ringfour-agent/src/ringfour_agent"
    if str(agent_dir) not in sys.path:
        sys.path.insert(0, str(agent_dir))
    from ringfour_agent.models.profile import RobotProfile
    spec = RobotProfile.model_validate(yaml.safe_load(draft))
    # Store an independent copy; browser uploads never name arbitrary server paths.
    name = re.sub(r"[^a-zA-Z0-9_-]+", "-", spec.name).strip("-")[:60] or "profile"
    with PROFILE_LOCK:
        IMPORTED_PROFILES.mkdir(parents=True, exist_ok=True)
        destination = IMPORTED_PROFILES / f"{name}-{uuid.uuid4().hex[:8]}.yaml"
        with destination.open("x", encoding="utf-8") as stream:
            stream.write(draft)
    identity = str(destination.relative_to(ROOT))
    return {"profile": identity, "name": spec.name, "profiles": robot_profiles()}


def validate_profile_draft(request):
    current = profile(request.get("profile", "@live"))
    if current["path"] == "@live":
        raise ValueError("Choose an original profile YAML to edit. The handshake snapshot is read-only.")
    draft = request.get("yaml")
    if not isinstance(draft, str) or len(draft.encode()) > 256000:
        raise ValueError("Provide a profile YAML under 256 KB")
    import yaml
    agent_dir = ROOT / "ringfour-agent/src/ringfour_agent"
    if str(agent_dir) not in sys.path:
        sys.path.insert(0, str(agent_dir))
    from ringfour_agent.models.profile import RobotProfile
    spec = RobotProfile.model_validate(yaml.safe_load(draft))
    diff = "".join(difflib.unified_diff(current["yaml"].splitlines(keepends=True), draft.splitlines(keepends=True),
                                      fromfile="Saved profile", tofile="Your draft"))
    return current, {"valid": True, "diff": diff, "changed": draft != current["yaml"],
                     "capabilities": list(spec.capabilities), "name": spec.name,
                     "note": "Validation checks the profile schema. Reload the agent with a saved profile, then rerun the affected checks to verify the bindings and hardware."}


def save_profile_draft(request):
    with PROFILE_LOCK:
        current, review = validate_profile_draft(request)
        if request.get("expected_sha") != current["sha"]:
            raise ValueError("The file changed since you loaded it. Reload the profile before saving your draft.")
        if not review["changed"]:
            return {**review, "sha": current["sha"], "saved": False}
        path = (ROOT / current["path"]).resolve()
        backup = path.with_name(path.name + "." + uuid.uuid4().hex[:8] + ".bak")
        shutil.copy2(path, backup)
        temporary = None
        try:
            with tempfile.NamedTemporaryFile("w", encoding="utf-8", dir=path.parent, delete=False) as stream:
                temporary = Path(stream.name)
                stream.write(request["yaml"])
            temporary.chmod(path.stat().st_mode)
            temporary.replace(path)
        finally:
            if temporary and temporary.exists():
                temporary.unlink()
        return {**review, "saved": True, "backup": str(backup),
                "sha": hashlib.sha256(request["yaml"].encode()).hexdigest(),
                "note": "Profile saved with a backup. Reload the agent with this file, inspect the loaded profile, then rerun the affected category."}


SCENARIOS = HERE / "configurations"
SCENARIO_LOCK = threading.Lock()


def scenario_files():
    return [{"name": p.name, "path": str(p), "sha": hashlib.sha256(p.read_bytes()).hexdigest()}
            for p in sorted(SCENARIOS.glob("*.conformance.yaml"))]


def scenario_draft(request):
    import yaml
    from conformance import validate_scenarios
    from ringfour.models.protocol import Welcome
    draft = request.get("yaml")
    if not isinstance(draft, str) or len(draft.encode()) > 256000:
        raise ValueError("Provide scenario YAML under 256 KB")
    data = yaml.safe_load(draft)
    if not isinstance(data, dict) or not data:
        raise ValueError("Provide a scenario mapping with version and robot identity")
    welcome = inspect_agent(request["uri"])
    validate_scenarios(data, Welcome.model_validate(welcome))
    sections = [k for k in data if k not in ("version", "profile")]
    return {"valid": True, "yaml": yaml.safe_dump(data, sort_keys=False),
            "sections": sections, "welcome": welcome, "data": data,
            "note": "Configuration matches the connected robot. Saving does not send commands. Enable configured commands in Tests to run it."}


def scenario_template(request):
    import yaml
    welcome = inspect_agent(request["uri"])
    data = {"version": 1, "profile": {k: welcome["profile"][k] for k in ("name", "model", "version")}}
    return {"yaml": yaml.safe_dump(data, sort_keys=False), "welcome": welcome}


def scenario_build(request):
    import yaml
    data = yaml.safe_load(request.get("yaml", ""))
    if not isinstance(data, dict):
        raise ValueError("Load a robot template or configuration first")
    if "movement_profile" in request:
        name = request["movement_profile"]
        if name not in {p['id'] for p in MOVEMENT_PROFILES}:
            raise ValueError("Unknown movement profile")
        profiles = data.setdefault("movement_profiles", {})
        if request.get("settings") is None:
            profiles.pop(name, None)
            if not profiles:
                data.pop("movement_profiles", None)
        else:
            profiles[name] = request["settings"]
    elif request.get("teleop") is None:
        data.pop("teleop", None)
    else:
        data["teleop"] = request["teleop"]
    return scenario_draft({**request, "yaml": yaml.safe_dump(data, sort_keys=False)})


def save_scenario(request):
    with SCENARIO_LOCK:
        review = scenario_draft(request)
        name = request.get("name", "")
        if not isinstance(name, str) or not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_-]{0,79}\.conformance\.yaml", name):
            raise ValueError("Use a filename such as my-robot.conformance.yaml")
        SCENARIOS.mkdir(parents=True, exist_ok=True)
        path = SCENARIOS / name
        if path.is_symlink():
            raise ValueError("Configuration must be a regular file")
        current_sha = hashlib.sha256(path.read_bytes()).hexdigest() if path.exists() else None
        if request.get("expected_sha") != current_sha:
            raise ValueError("Configuration already exists or changed. Load it before saving.")
        backup = None
        if path.exists():
            backup = path.with_name(path.name + "." + uuid.uuid4().hex[:8] + ".bak")
            shutil.copy2(path, backup)
        temporary = path.with_name(path.name + "." + uuid.uuid4().hex + ".tmp")
        try:
            temporary.write_text(review["yaml"])
            temporary.replace(path)
        finally:
            temporary.unlink(missing_ok=True)
        return {**review, "path": str(path), "name": name,
                "sha": hashlib.sha256(path.read_bytes()).hexdigest(), "backup": str(backup) if backup else None}


def load_scenario(request):
    from conformance import read_yaml
    import yaml
    path = Path(request["path"]).expanduser().resolve()
    data = read_yaml(path)
    return {"yaml": yaml.safe_dump(data, sort_keys=False), "path": str(path),
            "name": path.name, "sha": hashlib.sha256(path.read_bytes()).hexdigest(),
            "managed": path.parent == SCENARIOS.resolve(), "data": data}



def test_settings_path(welcome):
    identity = json.dumps([welcome.profile.name, welcome.profile.model])
    return SCENARIOS / ("settings-" + hashlib.sha256(identity.encode()).hexdigest()[:16] + ".json")


def matching_profile(welcome):
    candidates = []
    for entry in robot_profiles():
        candidate = profile(entry['id'])
        spec = candidate['data']
        if isinstance(spec,dict) and all(str(spec.get(k)) == str(welcome['profile'].get(k)) for k in ('name','model','version')):
            candidates.append(candidate)
    return candidates[0] if len(candidates) == 1 else None


def test_settings(welcome, source=None):
    from conformance import validate_scenarios
    source = source or matching_profile(welcome.model_dump(mode='json'))
    data = default_scenarios(welcome, source['data'] if source else None)
    path = test_settings_path(welcome)
    sha = None
    stale_model = False
    if path.exists():
        raw = path.read_bytes()
        saved = json.loads(raw)
        # Bind saved settings to the current profile version at load time.
        recorded_sha = saved.get("profile", {}).get("description_sha")
        stale_model = recorded_sha is not None and recorded_sha != welcome.description_sha
        if not stale_model:
            data.update({key:value for key,value in saved.items() if key != 'profile'})
            if recorded_sha is not None:
                data["profile"]["description_sha"] = recorded_sha
        # Keep stale fixtures on disk, but expose defaults for the new model.
        # Explicit saving creates the usual backup before replacing them.
        sha = hashlib.sha256(raw).hexdigest()
    validate_scenarios(data, welcome)
    return {'settings':data, 'sha':sha, 'robot':welcome.profile.name,
            'supported':{'mobility','teleop'} <= set(welcome.capabilities),
            'requirements':{**requirement_metadata(welcome,data,source),
                            'stale_model':stale_model,'saved_settings_path':str(path)}}


def selected_settings_profile(request, welcome):
    selected = request.get('profile', '@live')
    source = profile(selected) if selected != '@live' else None
    if source and any(str(source['data'].get(k)) != str(getattr(welcome.profile,k)) for k in ('name','model','version')):
        raise ValueError('Selected profile does not match the running robot')
    return source


def preview_navigation(request):
    from ringfour import Robot
    from ringfour.models.protocol import Welcome
    from conformance import validate_scenarios
    welcome = Welcome.model_validate(inspect_agent(request['uri']))
    source = selected_settings_profile(request,welcome) or matching_profile(welcome.model_dump(mode='json'))
    if not source or not (source['data'].get('mobility') or {}).get('nav2_navigate_to_pose'):
        raise ValueError('Select an effective profile with a navigation action binding')
    data = test_settings(welcome, source)['settings']
    recipe = request.get('navigation') or data.get('navigation')
    if not recipe:
        raise ValueError('Map-based navigation is not configured')
    validate_scenarios({**data,'navigation':recipe},welcome)
    with Robot(request['uri']) as robot:
        goals = live_map_goals(robot,recipe)
    return {'goals':goals,'clearance':recipe.get('clearance',1)}


def inspect_test_settings(request):
    from ringfour.models.protocol import Welcome
    welcome = Welcome.model_validate(inspect_agent(request['uri']))
    source = selected_settings_profile(request, welcome)
    return test_settings(welcome, source)


def save_test_settings(request):
    from ringfour.models.protocol import Welcome
    from conformance import validate_scenarios
    with SCENARIO_LOCK:
        welcome = Welcome.model_validate(inspect_agent(request['uri']))
        source = selected_settings_profile(request,welcome)
        current = test_settings(welcome,source)
        if current['sha'] != request.get('expected_sha'):
            raise ValueError('Settings changed. Reload them before saving.')
        if request.get('reset'):
            source = source or matching_profile(welcome.model_dump(mode="json"))
            settings = {**current["settings"], **default_scenarios(welcome, source["data"] if source else None)}
        else:
            settings = request.get('settings')
            if not isinstance(settings, dict) or set(settings) - {'teleop','movement_profiles','movement_tolerance','navigation','debug','groups','mapping','tolerances','description_change','artifacts','actions','connection','runtime'}:
                raise ValueError('Provide movement and drive settings')
            settings = {**current["settings"], **settings}
        # Bind reviewed physical fixtures to the model that was inspected.
        if any(any(recipe.get(key) for key in ('waypoints','clear_configurations','collision_configurations','tool','probe')) for recipe in settings.get('groups',{}).values()):
            settings['profile'].setdefault('description_sha',welcome.description_sha)
        validate_scenarios(settings,welcome)
        if settings.get('groups'):
            from ringfour import Robot
            from conformance import validate_vector
            with Robot(request['uri']) as robot:
                for name, recipe in settings['groups'].items():
                    states=robot.groups[name].joints
                    for field in ('waypoints','clear_configurations','collision_configurations'):
                        for vector in recipe.get(field,[]):
                            for joint,value in vector.items():
                                bounds=states[joint].limits
                                if bounds and not bounds[0]<=value<=bounds[1]:
                                    raise ValueError(f'{name}: {joint} target {value} exceeds live limits {bounds}')
        path = test_settings_path(welcome)
        SCENARIOS.mkdir(parents=True,exist_ok=True)
        if path.is_symlink():
            raise ValueError('Settings must be a regular file')
        if path.exists():
            shutil.copy2(path, path.with_name(path.name + '.' + uuid.uuid4().hex[:8] + '.bak'))
        temporary = path.with_name(path.name + '.' + uuid.uuid4().hex + '.tmp')
        try:
            temporary.write_text(json.dumps(settings,indent=2))
            temporary.replace(path)
        finally:
            temporary.unlink(missing_ok=True)
        return test_settings(welcome, source)


def coverage(detail, properties=None):
    if properties and properties.get("coverage"):
        return properties["coverage"]
    for label in ("not_applicable", "not_configured", "disabled"):
        if label.upper().replace("_", " ") + ":" in detail:
            return label
    return "not_exercised"


def validate_uri(uri, optional=False):
    if not uri and optional:
        return ""
    parsed = urlparse(uri)
    if parsed.scheme in ("ws", "wss") and parsed.hostname and parsed.port:
        return uri
    if parsed.scheme == "unix" and parsed.path.startswith("/"):
        return uri
    raise ValueError("Agent address must be ws://host:port, wss://host:port, or unix:///path.")


def inspect_agent(uri):
    from ringfour.transport import Connection
    from ringfour.models.protocol import Welcome
    conn = Connection(validate_uri(uri), timeout=3)
    try:
        result = conn.call("hello", {"protocol": 1, "client_version": "test-dashboard"}, timeout=3)
        return Welcome.model_validate(result).model_dump(mode="json")
    finally:
        conn.close()


def read_cases(path):
    if not path.exists():
        return []
    cases = []
    try:
        root = ET.parse(path).getroot()
    except ET.ParseError as exc:
        return [{"nodeid": "JUnit report", "status": "error", "duration": 0,
                 "detail": f"Invalid JUnit report: {exc}", "coverage": None, "collection_error": True}]
    for case in root.iter("testcase"):
        status, detail = "passed", ""
        for tag in ("error", "failure", "skipped"):
            nodes = case.findall(tag)
            if nodes:
                status = {"error": "error", "failure": "failed", "skipped": "skipped"}[tag]
                detail = "\n".join((n.get("message", "") + "\n" + (n.text or "")).strip() for n in nodes)
                break
        cases.append({"nodeid": case.get("classname", "") + "::" + case.get("name", ""),
            "status": status, "duration": float(case.get("time", 0)), "detail": detail,
            "coverage": coverage(detail) if status == "skipped" else None,
            "properties": {p.get("name"): p.get("value", "") for p in case.findall("properties/property")},
            "collection_error": status == "error" and not case.get("classname")})
    return cases


def display_cases(cases):
    rows, unsupported = [], {}
    for case in cases:
        if case["status"] != "skipped" or case.get("coverage") != "not_applicable":
            rows.append(case)
            continue
        reason = case.get("detail", "").splitlines()[0]
        # Live pytest events encode skips as (source, line, reason), unlike JUnit.
        try:
            encoded = ast.literal_eval(reason)
            if isinstance(encoded, tuple) and len(encoded) == 3:
                reason = str(encoded[2])
        except (ValueError, SyntaxError):
            pass
        if "NOT APPLICABLE:" in reason:
            reason = reason[reason.index("NOT APPLICABLE:"):]
        key = ((case.get("guide") or {}).get("scope", "robot"), case.get("instance", ""), reason)
        if key not in unsupported:
            row = {**case, "related_checks": [case["nodeid"]]}
            if case.get("guide"):
                row["guide"] = {**case["guide"], "title": reason.removeprefix("NOT APPLICABLE: ")}
            rows.append(row)
            unsupported[key] = row
        else:
            row = unsupported[key]
            row["related_checks"].append(case["nodeid"])
            if row.get("guide") and case.get("guide"):
                row["guide"]["categories"] = list(dict.fromkeys(row["guide"].get("categories", []) + case["guide"].get("categories", [])))
    return rows


def result_counts(cases):
    counts = {key: sum(c["status"] == key for c in cases) for key in ("passed", "failed", "error", "skipped")}
    counts["not_applicable"] = sum(c.get("coverage") == "not_applicable" for c in cases)
    counts["untested"] = counts["skipped"] - counts["not_applicable"]
    return counts


class Runner:
    def __init__(self, results=RESULTS):
        self.results = results
        self.lock = threading.Lock()
        self.proc = None
        self.active = None

    def start(self, request):
        suites = {s["id"]: s for s in catalog()}
        categories = request.get("categories", [])
        if "categories" in request or "checks" in request:
            targets, selected_guides = resolve_selection(categories, request.get("checks"), request.get("suite", "compatibility"))
            selected = list(dict.fromkeys(categories))
            needs_robot = any(g["scope"] == "robot" for g in selected_guides)
        else:
            selected = request.get("suites", [])
            if not isinstance(selected, list) or not selected or any(s not in suites for s in selected):
                raise ValueError("Select at least one known suite.")
            selected = list(dict.fromkeys(selected))
            targets = [suites[s]["path"] for s in selected]
            selected_guides = all_guides()
            needs_robot = any(s not in ("sdk", "missions") for s in selected)
        uri = validate_uri(request.get("uri", ""), optional=not needs_robot)
        stub = validate_uri(request.get("stub", ""), optional=True)
        selected_profile = request.get("profile", "@live")
        chosen = profile(selected_profile) if needs_robot and selected_profile != "@live" else None
        # Verify connectivity before creating a run. Offline suites need no agent.
        welcome = inspect_agent(uri) if needs_robot else None
        if needs_robot and selected_profile == "@live":
            chosen = matching_profile(welcome)
        if welcome and chosen and (welcome["profile"]["name"] != chosen["data"]["name"] or
                        welcome["profile"]["model"] != chosen["data"]["model"]):
            raise ValueError("The selected profile does not match the running agent's name/model. Select its loaded profile.")
        scenario_path = request.get("conformance", "").strip()
        scenario_data = None
        if needs_robot and not scenario_path:
            from ringfour.models.protocol import Welcome
            settings_state = test_settings(Welcome.model_validate(welcome), chosen)
            if settings_state['requirements']['stale_model'] and request.get('motion_ack') is True:
                raise ValueError('Robot model changed: review and save fresh Test settings before enabling commands')
            scenario_data = settings_state["settings"]
        if needs_robot and scenario_path:
            from conformance import read_yaml, validate_scenarios
            from ringfour.models.protocol import Welcome
            scenario_path = str(Path(scenario_path).expanduser().resolve())
            scenario_data = validate_scenarios(read_yaml(scenario_path), Welcome.model_validate(welcome))
        with self.lock:
            if self.active is not None:
                raise ValueError("A run is already active. Wait for it or cancel it.")
            run_id = time.strftime("%Y%m%d-%H%M%S") + "-" + uuid.uuid4().hex[:8]
            directory = self.results / run_id
            directory.mkdir(parents=True)
            meta = {"id": run_id, "started": time.time(), "status": "running", "suites": selected,
                    "categories": categories, "suite": request.get("suite", "compatibility"), "targets": targets,
                    "uri": uri, "profile": chosen["path"] if chosen else
                    ("loaded: " + welcome["profile"]["name"] if welcome else None), "welcome": welcome,
                    "conformance": scenario_path or None, "scenarios": scenario_data,
                    "motion_enabled": needs_robot and request.get("motion_ack") is True}
            save_meta(directory, meta)
            (directory / "test-guide.json").write_text(json.dumps(selected_guides, indent=2))
            env = dict(os.environ)
            for key in ("RINGFOUR_AGENT_UNIX", "RINGFOUR_AGENT_TCP", "RINGFOUR_STUB_UNIX", "RINGFOUR_STUB_TCP",
                        "RINGFOUR_PROFILE", "RINGFOUR_CONFORMANCE", "RINGFOUR_TEST_MOTION", "RINGFOUR_ROBOT"):
                env.pop(key, None)
            env.update(RINGFOUR_EXTERNAL_STACK="1", RINGFOUR_TEST_EVENTS=str(directory / "events.jsonl"),
                       PYTHONPATH=os.pathsep.join([str(HERE), str(ROOT / "ringfour-sdk"), env.get("PYTHONPATH", "")]))
            if scenario_data and scenario_data.get('runtime',{}).get('test_timeout'):
                env['RINGFOUR_TEST_TIMEOUT']=str(scenario_data['runtime']['test_timeout'])
            env["RINGFOUR_TEST_MOTION"] = "1" if meta["motion_enabled"] else "0"
            snapshot_dirs = [str(p) for p in PROFILE_DIRS]
            if scenario_data and scenario_data.get('artifacts',{}).get('snapshot_dir'):
                snapshot_dirs.insert(0,str(Path(scenario_data['artifacts']['snapshot_dir']).expanduser().resolve()))
            if chosen:
                snapshot_dirs.insert(0, str((ROOT / chosen["path"]).parent))
            if snapshot_dirs:
                env["RINGFOUR_SNAPSHOTS"] = os.pathsep.join(snapshot_dirs + ([env["RINGFOUR_SNAPSHOTS"]] if env.get("RINGFOUR_SNAPSHOTS") else []))
            if chosen:
                frozen_profile = directory / "profile.yaml"
                frozen_profile.write_text(chosen["yaml"])
                env["RINGFOUR_PROFILE"] = str(frozen_profile)
            if scenario_data is not None:
                # Execute the exact reviewed recipe, even if the author
                # edits the source YAML while pytest is collecting.
                import yaml
                frozen = directory / "conformance.yaml"
                frozen.write_text(yaml.safe_dump(scenario_data, sort_keys=False))
                env["RINGFOUR_CONFORMANCE"] = str(frozen)
            for address, prefix in ((uri, "RINGFOUR_AGENT"), (stub, "RINGFOUR_STUB")):
                if address:
                    is_unix = address.startswith("unix://")
                    env[prefix + ("_UNIX" if is_unix else "_TCP")] = address[7:] if is_unix else address
            command = [sys.executable, "-m", "pytest", *targets,
                       "--rootdir", str(ROOT),
                       "-v", "--tb=short", "--color=no", "-p", "events", "--junitxml", str(directory / "results.xml")]
            try:
                with (directory / "output.log").open("w") as log:
                    self.proc = subprocess.Popen(command, cwd=ROOT, env=env, stdout=log,
                                                 stderr=subprocess.STDOUT, start_new_session=True)
            except OSError as exc:
                meta.update(status="error", error=str(exc), finished=time.time())
                save_meta(directory, meta)
                raise
            self.active = run_id
            threading.Thread(target=self._wait, args=(self.proc, directory, meta), daemon=True).start()
            return {"id": run_id}

    def _wait(self, proc, directory, meta):
        code = proc.wait()
        with self.lock:
            final_cases = read_cases(directory / "results.xml")
            untested = any(c["status"] == "skipped" and c.get("coverage") != "not_applicable"
                          for c in final_cases)
            report_error = code == 0 and any(c["status"] in ("error", "failed") for c in final_cases)
            counts = {key: sum(c["status"] == key for c in final_cases) for key in ("passed", "failed", "error", "skipped")}
            counts["not_applicable"] = sum(c.get("coverage") == "not_applicable" for c in final_cases)
            counts["untested"] = counts["skipped"] - counts["not_applicable"]
            meta["counts"] = counts
            frozen = json.loads((directory / "test-guide.json").read_text())
            meta["scope_counts"] = {scope: result_counts([c for c in final_cases if (guide_for(c["nodeid"], frozen) or {}).get("scope", "robot") == scope]) for scope in ("robot", "sdk", "missions")}
            meta.update(exit_code=code, finished=time.time(), status="cancelled" if (directory / "cancel").exists() else
                        ("error" if report_error else "incomplete" if code == 0 and untested else "passed" if code == 0 else "failed" if code == 1 else "error"))
            save_meta(directory, meta)
            self.active = None
            self.proc = None

    def cancel(self):
        with self.lock:
            if self.proc is None or self.proc.poll() is not None:
                return {"status": "idle"}
            proc = self.proc
            directory = self.results / self.active
            (directory / "cancel").touch()
            try:
                os.killpg(proc.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
            threading.Thread(target=self._cancel_fallback, args=(proc,), daemon=True).start()
            return {"status": "cancelling"}

    @staticmethod
    def _cancel_fallback(proc):
        try:
            proc.wait(timeout=15)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(proc.pid, signal.SIGTERM)
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(proc.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
            except ProcessLookupError:
                pass

    def runs(self):
        return [json.loads(p.read_text()) for p in sorted(self.results.glob("*/run.json"), reverse=True)]

    def summaries(self):
        summaries = []
        for meta in self.runs():
            summary = {key: meta.get(key) for key in ("id", "started", "finished", "status", "profile", "categories", "suites", "suite", "motion_enabled", "counts", "scope_counts")}
            if not summary.get("scope_counts"):
                directory = self.results / meta["id"]
                saved_guides = directory / "test-guide.json"
                entries = json.loads(saved_guides.read_text()) if saved_guides.exists() else all_guides()
                cases = read_cases(directory / "results.xml")
                summary["scope_counts"] = {scope: result_counts([c for c in cases if (guide_for(c["nodeid"], entries) or {}).get("scope", "robot") == scope]) for scope in ("robot", "sdk", "missions")}
            summary["robot"] = (meta.get("welcome") or {}).get("profile", {}).get("name", "Platform · offline")
            summaries.append(summary)
        return summaries

    def snapshot(self, run_id):
        if run_id not in {r["id"] for r in self.runs()}:
            raise ValueError("Unknown run.")
        directory = self.results / run_id
        meta = json.loads((directory / "run.json").read_text())
        cases = {}
        current = None
        events = directory / "events.jsonl"
        parameters = {}
        if events.exists():
            for line in events.read_text().splitlines():
                try:
                    e = json.loads(line)
                except json.JSONDecodeError:
                    continue  # writer may be midway through the last line
                if e["event"] == "collection":
                    cases.update({n: {"nodeid": n, "status": "pending", "duration": 0, "detail": ""} for n in e["tests"]})
                    parameters = e.get("parameters", {})
                elif e["event"] == "start":
                    current = e["nodeid"]
                    cases[current]["status"] = "running"
                elif e["event"] == "report":
                    c = cases.setdefault(e["nodeid"], {"nodeid": e["nodeid"], "status": "pending", "duration": 0, "detail": ""})
                    c["duration"] += e["duration"]
                    c.setdefault("properties", {}).update(e.get("properties", {}))
                    if e["outcome"] == "failed":
                        c["status"] = "failed" if e["when"] == "call" else "error"
                        c["detail"] += e["detail"] + "\n"
                    elif e["outcome"] == "skipped" and c["status"] not in ("failed", "error"):
                        c.update(status="skipped", detail=e["detail"], coverage=coverage(e["detail"], e.get("properties")))
                    elif e["when"] == "teardown":
                        if c["status"] not in ("failed", "error", "skipped"):
                            c["status"] = "passed"
                        current = None
        if meta["status"] != "running":
            current = None
            for c in cases.values():
                if c["status"] in ("pending", "running"):
                    c["status"] = "not run"
            if not cases:
                cases = {c["nodeid"]: c for c in read_cases(directory / "results.xml")}
            else:
                # Collection can fail after some modules were already
                # enumerated. Preserve the XML's module errors alongside
                # those pending/not-run cases instead of hiding them.
                for case in read_cases(directory / "results.xml"):
                    if case["collection_error"]:
                        cases[case["nodeid"]] = case
        log = directory / "output.log"
        if log.exists():
            with log.open("rb") as stream:
                stream.seek(max(0, log.stat().st_size - 60000))
                output = stream.read().decode(errors="replace")
        else:
            output = ""
        frozen_guide = directory / "test-guide.json"
        entries = json.loads(frozen_guide.read_text()) if frozen_guide.exists() else all_guides()
        for case in cases.values():
            case["guide"] = guide_for(case["nodeid"], entries)
            case["parameters"] = parameters.get(case["nodeid"], {})
            if not case["parameters"] and "[" in case["nodeid"]:
                case["instance"] = case["nodeid"].split("[", 1)[1].rsplit("]", 1)[0]
            else:
                case["instance"] = " · ".join(str(v) for v in case["parameters"].values() if v is not None)
        scope_counts = {scope: result_counts([c for c in cases.values() if (c.get("guide") or {}).get("scope", "robot") == scope]) for scope in ("robot", "sdk", "missions")}
        return {**meta, "scope_counts": scope_counts, "display_cases": display_cases(list(cases.values())), "cases": list(cases.values()), "current": current, "log": output}


RUNNER = Runner()


class Handler(BaseHTTPRequestHandler):
    def reply(self, data, status=200, mime="application/json"):
        body = json.dumps(data).encode() if mime == "application/json" else data
        self.send_response(status)
        self.send_header("Content-Type", mime)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        path = urlparse(self.path).path
        try:
            if path == "/":
                return self.reply((HERE / "index.html").read_bytes(), mime="text/html; charset=utf-8")
            # Explicit local asset allowlist; never resolve request paths on disk.
            assets = {
                "/dashboard.css": ("dashboard.css", "text/css; charset=utf-8"),
                **{f"/design-system/{name}.css": (f"design-system/{name}.css", "text/css; charset=utf-8")
                   for name in ("tokens", "base", "components")},
                **{f"/design-system/assets/fonts/SpaceGrotesk-{weight}.woff2":
                   (f"design-system/assets/fonts/SpaceGrotesk-{weight}.woff2", "font/woff2")
                   for weight in ("Regular", "Medium", "Bold")},
            }
            if path in assets:
                filename, mime = assets[path]
                return self.reply((HERE / filename).read_bytes(), mime=mime)
            if path == "/robot-preparation.md":
                return self.reply((ROOT / "ringfour-testing/integration" / "ROBOT_PREPARATION.md").read_bytes(), mime="text/plain; charset=utf-8")
            if path == "/preparation.js":
                return self.reply((HERE / "preparation.js").read_bytes(), mime="application/javascript")
            if path == "/requirements.js":
                return self.reply((HERE / "requirements.js").read_bytes(), mime="application/javascript")
            if path == "/ringfour_logo.png":
                return self.reply((HERE / "ringfour_logo.png").read_bytes(), mime="image/png")
            if path == "/api/config":
                return self.reply({"suites": catalog(), "profiles": robot_profiles(),
                                   "categories": category_catalog(), "test_suites": SUITES, "tests": all_guides(), "scenarios": scenario_files(), "movement_profiles": MOVEMENT_PROFILES, "drive_defaults": DEFAULT_TELEOP, "drive_fields": DRIVE_FIELDS, "navigation_fields": NAVIGATION_FIELDS})
            if path == "/api/preparation/environments":
                from preparation import environments
                return self.reply({'environments':environments()})
            if path.startswith('/api/preparation/'):
                from preparation import status
                return self.reply(status(path.rsplit('/',1)[-1]))
            if path == "/api/runs":
                return self.reply({"runs": RUNNER.summaries(), "active": RUNNER.active})
            if path.startswith("/api/runs/"):
                parts = path.split("/")
                snapshot = RUNNER.snapshot(parts[3])
                if len(parts) == 5 and parts[4] in ("results.xml", "output.log"):
                    file = RUNNER.results / snapshot["id"] / parts[4]
                    if not file.exists():
                        return self.reply({"error": "Artifact not available yet."}, 404)
                    return self.reply(file.read_bytes(), mime="application/xml" if parts[4].endswith("xml") else "text/plain")
                return self.reply(snapshot)
            self.reply({"error": "Not found"}, 404)
        except (ValueError, OSError, ET.ParseError) as exc:
            self.reply({"error": str(exc)}, 400)

    def do_POST(self):
        # Only same-origin local browser requests can invoke test actions.
        origin = self.headers.get("Origin")
        if origin and urlparse(origin).netloc != self.headers.get("Host"):
            return self.reply({"error": "Origin rejected"}, 403)
        if self.headers.get("Content-Type", "").split(";")[0] != "application/json":
            return self.reply({"error": "JSON required"}, 415)
        try:
            size = int(self.headers.get("Content-Length", "0"))
            if not 0 < size <= 300000:
                raise ValueError("Invalid request size")
            data = json.loads(self.rfile.read(size))
            path = urlparse(self.path).path
            if path == "/api/run":
                result = RUNNER.start(data)
            elif path == "/api/cancel":
                result = RUNNER.cancel()
            elif path == "/api/inspect":
                welcome = inspect_agent(data["uri"])
                result = {"welcome": welcome, "profile": profile(data.get("profile", "@live"), welcome)}
            elif path == "/api/test-settings":
                result = inspect_test_settings(data)
            elif path in ('/api/preparation/find','/api/preparation/load'):
                from preparation import matching_bundles, load_bundle
                from ringfour.models.protocol import Welcome
                welcome=Welcome.model_validate(inspect_agent(data['uri']))
                selected_settings_profile(data,welcome)
                result = {'bundles':matching_bundles(welcome.description_sha)} if path.endswith('/find') else load_bundle(data.get('key'),welcome.description_sha)
            elif path == "/api/preparation/review":
                from preparation import status, reviewed_settings
                from ringfour.models.protocol import Welcome
                welcome=Welcome.model_validate(inspect_agent(data['uri']))
                selected_settings_profile(data,welcome)
                if data.get('reviewed') is not True:
                    raise ValueError('Review geometry, contact masks and exclusions before accepting fixtures')
                bundle=status(data['job'])['result']
                if bundle is None:raise ValueError('Preparation has not completed')
                result=reviewed_settings(bundle,data.get('selected'),welcome.description_sha)
            elif path == "/api/preparation/start":
                if RUNNER.active is not None:
                    raise ValueError('Wait for the active test run before preparing geometry')
                from preparation import start
                from ringfour.models.protocol import Welcome
                selected_settings_profile(data, Welcome.model_validate(inspect_agent(data['uri'])))
                result = start(data)
            elif path == "/api/test-settings/group-state":
                from ringfour.models.protocol import Welcome
                selected_settings_profile(data, Welcome.model_validate(inspect_agent(data['uri'])))
                result = group_state(data['uri'],data['group'])
            elif path == "/api/test-settings/group-preview":
                from ringfour.models.protocol import Welcome
                from conformance import validate_scenarios
                from group_recipes import generate_probe
                from types import SimpleNamespace
                welcome = Welcome.model_validate(inspect_agent(data['uri']))
                selected_settings_profile(data, welcome)
                recipe = data.get('recipe', {})
                validate_scenarios({'version':1,'profile':{'name':welcome.profile.name,'model':welcome.profile.model},
                                    'groups':{data['group']:recipe}}, welcome)
                state = group_state(data['uri'],data['group'])
                result = generate_probe([j['name'] for j in state['joints']],
                    {j['name']:j['position'] for j in state['joints']},
                    {j['name']:SimpleNamespace(type=j['type'],limits=j['limits']) for j in state['joints']},
                    recipe.get('probe',{}), recipe.get('tolerance', .001))
                result['description_sha'] = state['description_sha']
            elif path == "/api/navigation/preview":
                result = preview_navigation(data)
            elif path == "/api/test-settings/save":
                result = save_test_settings(data)
            elif path == "/api/scenario/template":
                result = scenario_template(data)
            elif path == "/api/scenario/build":
                result = scenario_build(data)
            elif path == "/api/scenario/validate":
                result = scenario_draft(data)
            elif path == "/api/scenario/save":
                result = save_scenario(data)
            elif path == "/api/scenario/load":
                result = load_scenario(data)
            elif path == "/api/profile":
                result = profile(data["profile"])
            elif path == "/api/profile/import":
                result = import_profile(data)
            elif path == "/api/profile/validate":
                _, result = validate_profile_draft(data)
            elif path == "/api/profile/save":
                result = save_profile_draft(data)
            else:
                return self.reply({"error": "Not found"}, 404)
            self.reply(result)
        except Exception as exc:
            self.reply({"error": str(exc)}, 400)

    def log_message(self, *args):
        pass


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--profiles-dir", type=Path, action="append", default=[],
                        help="Additional profile directory; repeat for multiple robot workspaces")
    args = parser.parse_args()
    PROFILE_DIRS.extend(p.expanduser().resolve() for p in args.profiles_dir)
    server = ThreadingHTTPServer(("127.0.0.1", args.port), Handler)
    # Interrupted server sessions remain reviewable on restart.
    for p in RESULTS.glob("*/run.json"):
        meta = json.loads(p.read_text())
        if meta["status"] == "running":
            meta.update(status="interrupted", finished=time.time())
            save_meta(p.parent, meta)
    print(f"Ringfour test console: http://127.0.0.1:{args.port}", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        RUNNER.cancel()
        proc = RUNNER.proc
        if proc:
            proc.wait(timeout=25)
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
