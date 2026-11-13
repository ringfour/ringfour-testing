"""Enum units (spec section 27, plan 6.2).

The whole value of a ``(str, Enum)`` vocabulary is that *nothing converts
at the boundary*: the member is the wire value, the YAML value and the
dict key. These tests pin that property everywhere it is relied on, plus
the migrated surfaces (audit, nav/traj phase, payload transport,
locomotion, slam mode). No ROS, no sim.
"""

from __future__ import annotations

import json

import pytest
from pydantic import ValidationError

import ringfour
from ringfour import enums
from ringfour.enums import (
    AuditKind,
    Controller,
    Frame,
    GroupKind,
    Locomotion,
    Outcome,
    Phase,
    SlamMode,
    Transport,
)
from ringfour.models import AuditEvent, PayloadRef, Pose
from ringfour.models.status import NavStatus,TrajStatus
from ringfour_agent.models.profile import MobilityConfig

ALL_ENUMS = [Controller, Phase, Frame, GroupKind, AuditKind, Outcome,
             Locomotion, SlamMode, Transport]


# -- the str-subclass contract -------------------------------------------------

def test_nine_enums_all_str_subclasses():
    """Nine enums all str subclasses

    Categories: sdk
    What it checks: Plan 6.2 promises nine; a tenth (or a missing one) should be a
    deliberate edit, not a drift. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that shared status and control vocabularies stay consistent across serialization.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the protocol vocabulary code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.

    Technical notes:
    Plan 6.2 promises nine; a tenth (or a missing one) should be a
    deliberate edit, not a drift.
    """
    assert len(ALL_ENUMS) == 9
    assert sorted(enums.__all__) == sorted(cls.__name__ for cls in ALL_ENUMS)
    for cls in ALL_ENUMS:
        assert issubclass(cls, str), cls


@pytest.mark.parametrize("cls", ALL_ENUMS)
def test_member_is_its_own_wire_value(cls):
    """Member is its own wire value

    Categories: sdk
    What it checks: Verifies that member is its own wire value. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that shared status and control vocabularies stay consistent across serialization.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the protocol vocabulary code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    for member in cls:
        assert member == member.value            # bare strings compare equal
        assert str(member) == member.value       # logging / f-strings
        assert f"{member}" == member.value
        assert json.dumps(member) == json.dumps(member.value)
        assert cls(member.value) is member       # parse back


def test_repr_names_the_member():
    """Repr names the member

    Categories: sdk
    What it checks: Verifies that repr names the member. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that shared status and control vocabularies stay consistent across serialization.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the protocol vocabulary code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    assert repr(Controller.TRAJECTORY) == "Controller.TRAJECTORY"


def test_members_hash_as_their_strings():
    """Members hash as their strings

    Categories: sdk
    What it checks: This is what lets a YAML-loaded ``controllers:`` dict (plain string
    keys) be indexed by ``Controller.TRAJECTORY`` without a conversion
    pass — and the reverse. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that shared status and control vocabularies stay consistent across serialization.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the protocol vocabulary code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.

    Technical notes:
    This is what lets a YAML-loaded ``controllers:`` dict (plain string
    keys) be indexed by ``Controller.TRAJECTORY`` without a conversion
    pass — and the reverse.
    """
    from_yaml = {"trajectory": "arm_trajectory_controller"}
    assert from_yaml[Controller.TRAJECTORY] == "arm_trajectory_controller"
    typed = {Controller.TOOL: "gripper_controller"}
    assert typed["tool"] == "gripper_controller"


def test_illegal_value_rejected():
    """Illegal value rejected

    Categories: sdk
    What it checks: Verifies that illegal value rejected. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that shared status and control vocabularies stay consistent across serialization.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the protocol vocabulary code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    with pytest.raises(ValueError):
        Controller("moveit")
    with pytest.raises(ValueError):
        SlamMode("MAPPING")  # values are lowercase; no case folding


def test_exported_from_the_package_root():
    """Exported from the package root

    Categories: sdk
    What it checks: Verifies that exported from the package root. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that shared status and control vocabularies stay consistent across serialization.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the protocol vocabulary code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    for cls in ALL_ENUMS:
        if cls.__name__ in ('Transport','Phase'):
            assert not hasattr(ringfour,cls.__name__)
        else:assert getattr(ringfour, cls.__name__) is cls
        assert (cls.__name__ in ringfour.__all__)==(cls.__name__ not in ("Transport","Phase"))


def test_frame_enum_owns_the_name():
    """Frame enum owns the name

    Categories: sdk
    What it checks: ``Frame`` is the frame-alias enum (spec 27). The Phase 4 RGB image
    type was renamed ``ImageFrame`` to free it (decision 17, revised
    2026-08-24); both must be reachable from the package root. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that shared status and control vocabularies stay consistent across serialization.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the protocol vocabulary code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.

    Technical notes:
    ``Frame`` is the frame-alias enum (spec 27). The Phase 4 RGB image
    type was renamed ``ImageFrame`` to free it (decision 17, revised
    2026-08-24); both must be reachable from the package root.
    """
    assert ringfour.Frame is Frame
    assert Frame.BASE == "@base"
    from ringfour.data import ImageFrame
    assert ringfour.ImageFrame is ImageFrame
    assert not issubclass(ImageFrame, Frame)


# -- migrated surfaces ---------------------------------------------------------

def test_nav_and_traj_status_coerce_bare_strings():
    """Nav and traj status coerce bare strings

    Categories: sdk
    What it checks: Source compatibility: Phase 3-5 code passes plain strings. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that shared status and control vocabularies stay consistent across serialization.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the protocol vocabulary code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.

    Technical notes:
    Source compatibility: Phase 3-5 code passes plain strings.
    """
    nav = NavStatus(phase="moving", progress=0.5,
                    pose=Pose(x=1, y=2, frame="map"),
                    remaining_distance=3.0, stamp=1.0)
    assert nav.phase is Phase.MOVING
    assert nav.phase == "moving"
    assert nav.model_dump(mode="json")["phase"] == "moving"

    traj = TrajStatus(phase=Phase.PLANNING, progress=0.0, joint_positions={})
    assert traj.model_dump(mode="json")["phase"] == "planning"

    with pytest.raises(ValidationError):
        NavStatus(phase="wandering", progress=0.0,
                  pose=Pose(x=0, y=0, frame="map"),
                  remaining_distance=0.0, stamp=1.0)


def test_audit_event_outcome_is_closed():
    """Audit event outcome is closed

    Categories: sdk
    What it checks: The free text that used to live in ``outcome`` ("error: ...",
    "failed: ...", a watch id) now belongs in ``detail`` — that is what
    makes the field filterable. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that shared status and control vocabularies stay consistent across serialization.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the protocol vocabulary code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.

    Technical notes:
    The free text that used to live in ``outcome`` ("error: ...",
    "failed: ...", a watch id) now belongs in ``detail`` — that is what
    makes the field filterable.
    """
    event = AuditEvent(stamp="2026-08-24T00:00:00+00:00", kind="safety",
                       outcome="rejected", detail={"reason": "estop"})
    assert event.kind is AuditKind.SAFETY
    assert event.outcome is Outcome.REJECTED
    assert json.loads(event.model_dump_json())["outcome"] == "rejected"

    with pytest.raises(ValidationError):
        AuditEvent(stamp="x", kind="safety", outcome="error: it broke")
    with pytest.raises(ValidationError):
        AuditEvent(stamp="x", kind="telemetry")


def test_outcome_keeps_cancelled_distinct_from_error():
    """Outcome keeps cancelled distinct from error

    Categories: sdk
    What it checks: Decision 18: the audit log has to tell "the operator stopped it"
    from "it broke". Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that shared status and control vocabularies stay consistent across serialization.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the protocol vocabulary code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.

    Technical notes:
    Decision 18: the audit log has to tell "the operator stopped it"
    from "it broke".
    """
    assert Outcome.CANCELLED != Outcome.ERROR
    assert Outcome.CANCELLED == "cancelled"


def test_payload_ref_transport():
    """Payload ref transport

    Categories: sdk
    What it checks: Verifies that payload ref transport. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that shared status and control vocabularies stay consistent across serialization.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the protocol vocabulary code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    ref = PayloadRef(name="rc-x", dtype="uint8", shape=(4,), size=4)
    assert ref.transport is Transport.SHM
    assert ref.model_dump(mode="json")["transport"] == "shm"

    # A remote agent may describe a payload we cannot fetch: the descriptor
    # must still parse, so the failure is a readable client-side error
    # rather than a schema rejection nobody can act on.
    remote = PayloadRef(transport="net", name="rc-y", dtype="uint8",
                        shape=(4,), size=4)
    assert remote.transport is Transport.NET
    from ringfour.payloads import fetch_array
    with pytest.raises(ringfour.RingfourError, match="not supported"):
        fetch_array(remote)


def test_mobility_locomotion_from_yaml_string():
    """Mobility locomotion from yaml string

    Categories: sdk
    What it checks: Verifies that mobility locomotion from yaml string. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that shared status and control vocabularies stay consistent across serialization.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the protocol vocabulary code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    config = MobilityConfig(locomotion="differential")
    assert config.locomotion is Locomotion.DIFFERENTIAL
    with pytest.raises(ValidationError):
        MobilityConfig(locomotion="tracked")


def test_group_kind_and_slam_mode_vocabularies():
    """Group kind and slam mode vocabularies

    Categories: sdk
    What it checks: Verifies that group kind and slam mode vocabularies. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that shared status and control vocabularies stay consistent across serialization.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the protocol vocabulary code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    assert {k.value for k in GroupKind} == {"arm", "pointing", "linear"}
    assert {m.value for m in SlamMode} == {"mapping", "localization", "idle"}
