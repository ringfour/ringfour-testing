"""The committed generated artifacts must match the source they came from.

``protocol.json`` and ``docs/types-reference.md`` are generated and
committed so a wire or type change shows up as a reviewable diff. That
only works if nobody can land a model change without regenerating — which
is what these tests enforce. The hand-written reference this replaces
(``archive/ringfour-types.md``) drifted precisely because nothing checked.
"""

from __future__ import annotations

import importlib.util
import json
import sys
from pathlib import Path

import pytest

SDK_DIR = Path(__file__).resolve().parents[2] / "ringfour-sdk"
SCRIPTS = SDK_DIR / "scripts"

REGENERATE = ("regenerate with:\n"
              "  python3 ringfour/scripts/gen_protocol.py\n"
              "  python3 ringfour/scripts/gen_types_reference.py")


def _load(name: str):
    """Import a script from ringfour/scripts/ (not an installed package)."""
    spec = importlib.util.spec_from_file_location(name, SCRIPTS / f"{name}.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope="module")
def gen_protocol():
    return _load("gen_protocol")


@pytest.fixture(scope="module")
def gen_types():
    return _load("gen_types_reference")


def test_protocol_json_is_current(gen_protocol):
    """Protocol json is current

    Categories: sdk
    What it checks: Verifies that protocol json is current. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that the generated API remains synchronized with the wire protocol.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the generated sdk interface code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    committed = (SDK_DIR / "protocol.json").read_text()
    assert committed == gen_protocol.render(), (
        "protocol.json is stale — " + REGENERATE)


def test_types_reference_is_current(gen_types):
    """Types reference is current

    Categories: sdk
    What it checks: Verifies that types reference is current. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that the generated API remains synchronized with the wire protocol.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the generated sdk interface code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    committed = (SDK_DIR / "docs" / "types-reference.md").read_text()
    assert committed == gen_types.build(), (
        "docs/types-reference.md is stale — " + REGENERATE)


def test_protocol_carries_enum_constraints():
    """Protocol carries enum constraints

    Categories: sdk
    What it checks: Plan 6.2 exit criterion: the enums reach the wire schema as real
    JSON-schema constraints, not prose. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that the generated API remains synchronized with the wire protocol.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the generated sdk interface code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.

    Technical notes:
    Plan 6.2 exit criterion: the enums reach the wire schema as real
    JSON-schema constraints, not prose.
    """
    from ringfour import enums

    schema = json.loads((SDK_DIR / "protocol.json").read_text())

    # hoisted, for consumers not walking a model's $defs
    assert set(schema["enums"]) == set(enums.__all__)
    for name in enums.__all__:
        cls = getattr(enums, name)
        assert schema["enums"][name]["values"] == [m.value for m in cls]

    # and inside the models that use them
    audit = schema["models"]["AuditEvent"]["$defs"]
    assert audit["AuditKind"]["enum"] == [m.value for m in enums.AuditKind]
    assert audit["Outcome"]["enum"] == [m.value for m in enums.Outcome]
    assert audit["AuditKind"]["type"] == "string"

    operation=schema["models"]["OperationSnapshot"]["$defs"]
    assert operation["OperationState"]["enum"]==[m.value for m in enums.OperationState]
    assert "NavStatus" not in schema["models"] and "TrajStatus" not in schema["models"]

    payload = schema["models"]["PayloadRef"]["$defs"]
    assert payload["Transport"]["enum"] == [m.value for m in enums.Transport]


def test_types_reference_covers_every_public_type():
    """Types reference covers every public type

    Categories: sdk
    What it checks: A type the SDK exports but the reference omits is the drift the
    generated document exists to prevent. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that the generated API remains synchronized with the wire protocol.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the generated sdk interface code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.

    Technical notes:
    A type the SDK exports but the reference omits is the drift the
    generated document exists to prevent.
    """
    import ringfour
    from ringfour import enums, models

    text = (SDK_DIR / "docs" / "types-reference.md").read_text()
    for name in list(models.__all__) + list(enums.__all__):
        assert f"### `{name}`" in text, f"{name} missing from the reference"
    # the array-bearing types too (they are not pydantic models)
    for name in ("ImageFrame", "DepthFrame", "PointCloud", "LaserScan",
                 "OccupancyGrid", "JointInfo", "Event"):
        assert f"### `{name}`" in text, f"{name} missing from the reference"
        assert name in ringfour.__all__
