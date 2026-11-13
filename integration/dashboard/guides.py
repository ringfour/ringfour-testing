"""Read operator descriptions from test docstrings without importing test code."""
from __future__ import annotations

import ast
import re
from pathlib import Path

from suites import SUITES, suite_for

ROOT = Path(__file__).resolve().parents[3]
CATEGORIES = [
    {"id": "missions", "label": "Mission recipes", "description": "Optional mission helper tests using synthetic inputs; no robot required."},
    {"id": "movement", "label": "Movement", "description": "Position feedback, drive commands, navigation, and stopping."},
    {"id": "manipulation", "label": "Manipulation", "description": "Joint groups, controllers, trajectories, and fitted tools."},
    {"id": "cameras", "label": "Cameras", "description": "Images, calibration, fresh frames, depth, and 3D points."},
    {"id": "sensors", "label": "Sensors & status", "description": "Lidar, IMU, range, force, custom readings, and power state."},
    {"id": "safety", "label": "Safety", "description": "Stop watches, watchdogs, command ownership, and audit history."},
    {"id": "mapping", "label": "Mapping & places", "description": "Occupancy grids, saved locations, and map/localization workflows."},
    {"id": "profile", "label": "Profile & bindings", "description": "Whether the loaded configuration names the actual interfaces."},
    {"id": "description", "label": "Robot model & frames", "description": "Model identity, joint chains, limits, and camera/TF frames."},
    {"id": "kinematics", "label": "Kinematics", "description": "Calculated poses, inverse kinematics, reach, and collision geometry."},
    {"id": "connection", "label": "Connection", "description": "Agent access, protocol agreement, clients, tasks, and payloads."},
    {"id": "sdk", "label": "Platform · offline", "description": "Software regressions using synthetic fixtures; does not use your robot."},
]
LABELS = {
    "Categories": "categories", "What it checks": "what", "Why it matters": "why",
    "If it fails": "fix", "Profile fields": "profile_fields",
    "Scenario fields": "scenario_fields", "Robot effects": "effects",
    "Technical notes": "notes",
}
MOTION_FIXTURES = {"motion_enabled", "group_motion", "session_group", "tool_case",
                   "teleop_recipe", "navigation_recipe", "movement_profile_recipe"}
_cache_key = None
_cache = None


def parse_doc(doc):
    if not doc:
        raise ValueError("Missing operator description")
    lines = doc.splitlines()
    fields = {"title": lines[0].strip()}
    current = None
    for line in lines[1:]:
        match = re.match(r"^([A-Za-z ]+):\s*(.*)$", line)
        if match and match[1] in LABELS:
            current = LABELS[match[1]]
            fields[current] = match[2].strip()
        elif current and line.strip():
            fields[current] += " " + line.strip()
    required = {"title", "categories", "what", "why", "fix", "profile_fields", "scenario_fields", "effects"}
    if required - fields.keys():
        raise ValueError(f"Missing guide fields: {sorted(required-fields.keys())}")
    fields.pop("notes", None)
    fields["categories"] = [v.strip() for v in fields["categories"].split(",")]
    for key in ("profile_fields", "scenario_fields"):
        fields[key] = [] if fields[key] == "None." else [v.strip() for v in fields[key].split(";") if v.strip()]
    if set(fields["categories"]) - {c["id"] for c in CATEGORIES}:
        raise ValueError("Unknown test category")
    return fields


def marks(nodes):
    result = set()
    requirements = set()
    for node in nodes:
        for child in ast.walk(node):
            if isinstance(child, ast.Attribute) and isinstance(child.value, ast.Attribute) and child.value.attr == "mark":
                result.add(child.attr)
            if isinstance(child, ast.Call) and isinstance(child.func, ast.Attribute) and child.func.attr == "requires":
                requirements.update(a.value for a in child.args if isinstance(a, ast.Constant) and isinstance(a.value, str))
    return result, sorted(requirements)


def all_guides():
    global _cache_key, _cache
    files = sorted((ROOT / "ringfour-testing/integration").glob("test_*.py")) + sorted((ROOT / "ringfour-testing/platform_tests").glob("test_*.py")) + sorted((ROOT / "missions/tests").glob("test_*.py"))
    key = tuple((str(p), p.stat().st_mtime_ns) for p in files)
    if key == _cache_key:
        return _cache
    entries = []
    for path in files:
        tree = ast.parse(path.read_text())
        module_marks = [n.value for n in tree.body if isinstance(n, ast.Assign) and any(isinstance(t, ast.Name) and t.id == "pytestmark" for t in n.targets)]
        scope = "sdk" if path.parent.name == "platform_tests" else ("missions" if path.parent == ROOT / "missions/tests" else "robot")
        for node in tree.body:
            if not isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)) or not node.name.startswith("test_"):
                continue
            try:
                entry = parse_doc(ast.get_docstring(node))
            except ValueError as exc:
                raise ValueError(f"{path.name}::{node.name}: {exc}") from exc
            flags, requires = marks(module_marks + node.decorator_list)
            fixtures = {a.arg for a in node.args.args}
            motion = "motion" in flags or bool(fixtures & MOTION_FIXTURES)
            source = str(path.relative_to(ROOT))
            entry.update(id=f"{source}::{node.name}", source=source, function=node.name,
                         line=node.lineno, scope=scope, suite=suite_for(node.name, scope), requires=requires, requires_motion=motion)
            entries.append(entry)
    _cache_key, _cache = key, entries
    return entries


def category_catalog():
    entries = all_guides()
    return [{**category,
             "count": sum(category["id"] in g["categories"] and g["suite"] == "compatibility" for g in entries),
             "motion_count": sum(category["id"] in g["categories"] and g["requires_motion"] for g in entries)}
            for category in CATEGORIES]


def resolve_selection(categories, checks=None, suite="compatibility"):
    if suite not in {s["id"] for s in SUITES}:
        raise ValueError("Unknown test suite")
    if categories == ["sdk"]:
        suite = "sdk"
    elif categories == ["missions"]:
        suite = "missions"
    if not isinstance(categories, list) or any(not isinstance(c, str) for c in categories):
        raise ValueError("Select known test categories")
    if set(categories) - {c["id"] for c in CATEGORIES}:
        raise ValueError("Unknown test category")
    if checks is not None and (not isinstance(checks, list) or any(not isinstance(c, str) for c in checks)):
        raise ValueError("Select known checks")
    entries = all_guides()
    known = {g["id"]: g for g in entries}
    if checks and set(checks) - known.keys():
        raise ValueError("Unknown test check")
    selected = [g for g in entries if (set(g["categories"]) & set(categories) and (suite == "all" or g["suite"] == suite)) or g["id"] in (checks or [])]
    if not selected:
        raise ValueError("Select at least one category or check")
    targets = [g["id"] for g in selected]
    if any(g["scope"] == "sdk" for g in selected) and (suite in ("sdk", "all")):
        targets = ["ringfour-testing/platform_tests"] + [g["id"] for g in selected if g["scope"] != "sdk"]
    return targets, selected


def guide_for(nodeid, entries):
    # Runtime IDs include parameter suffixes; JUnit uses dotted module names.
    parts = nodeid.split("::")
    function = parts[-1].split("[", 1)[0]
    exact = nodeid.split("[", 1)[0]
    module = parts[0].replace("\\", "/").rsplit("/", 1)[-1].removesuffix(".py").split(".")[-1]
    matches = [g for g in entries if g["id"] == exact]
    if not matches:
        matches = [g for g in entries if g["function"] == function and Path(g["source"]).stem == module]
    if not matches:
        # Historical IDs sometimes omitted the module. Only resolve if
        # the function name is unambiguous across all selected sources.
        matches = [g for g in entries if g["function"] == function]
    return matches[0] if len(matches) == 1 else None
