"""A trajectory that ran out of time must not report ARRIVED.

The controller says `succeeded` when it finishes executing its plan,
which is not the same claim as the joints being where the plan ended.
Give a trajectory less time than a joint needs and it interpolates to
the end of its schedule, stops, and succeeds — with the arm elsewhere.
On 2026-09-01 that cost a day: it was mistaken in turn for a
deprojection bug, a TF bug and an IK bug, because every layer reported
success. See docs/journal/found_along_the_way.md section 16.

These cover the part with an answer that can be wrong. The handler
around them is ROS and asyncio; this is arithmetic.
"""

from __future__ import annotations

import math

import pytest

from ringfour_agent.control import arrival_residual, worst_joint

JOINTS = ["shoulder", "elbow", "wrist"]


def snapshot(**positions: float) -> dict[str, tuple]:
    """joint -> (position, velocity, effort), as ros.joint_snapshot gives."""
    return {name: (value, 0.0, 0.0) for name, value in positions.items()}


def test_residual_is_actual_minus_commanded():
    """Residual is actual minus commanded

    Categories: sdk
    What it checks: Verifies that residual is actual minus commanded. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that actual joint arrival and residuals stay distinct from a controller reporting success.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the trajectory arrival code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    residual = arrival_residual(
        JOINTS, [1.0, 2.0, 3.0],
        snapshot(shoulder=1.1, elbow=2.0, wrist=2.5))
    assert residual["shoulder"] == pytest.approx(0.1)
    assert residual["elbow"] == pytest.approx(0.0)
    assert residual["wrist"] == pytest.approx(-0.5)


def test_a_joint_with_no_state_is_omitted_not_assumed_zero():
    """A joint with no state is omitted not assumed zero

    Categories: sdk
    What it checks: No reading is not a reading of no error. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that actual joint arrival and residuals stay distinct from a controller reporting success.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the trajectory arrival code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.

    Technical notes:
    No reading is not a reading of no error.
    """
    residual = arrival_residual(JOINTS, [1.0, 2.0, 3.0],
                                snapshot(shoulder=1.0, elbow=2.0))
    assert set(residual) == {"shoulder", "elbow"}


def test_worst_joint_picks_the_largest_absolute_error():
    """Worst joint picks the largest absolute error

    Categories: sdk
    What it checks: Verifies that worst joint picks the largest absolute error. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that actual joint arrival and residuals stay distinct from a controller reporting success.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the trajectory arrival code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    name, gap = worst_joint({"a": 0.1, "b": -0.7, "c": 0.3})
    assert name == "b"
    assert gap == pytest.approx(0.7)


def test_worst_joint_of_nothing_is_not_a_perfect_arrival():
    """Worst joint of nothing is not a perfect arrival

    Categories: sdk
    What it checks: An empty residual means no joint reported state at all. Returning
    0.0 with a name of None lets the caller tell that apart from a move
    that genuinely landed. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that actual joint arrival and residuals stay distinct from a controller reporting success.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the trajectory arrival code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.

    Technical notes:
    An empty residual means no joint reported state at all. Returning
    0.0 with a name of None lets the caller tell that apart from a move
    that genuinely landed.
    """
    name, gap = worst_joint({})
    assert name is None
    assert gap == 0.0


def test_a_truncated_move_is_caught_at_the_default_tolerance():
    """A truncated move is caught at the default tolerance

    Categories: sdk
    What it checks: The real case: a 40 degree move that only got 12 degrees along.

    The default goal_tolerance is 0.05 rad (~2.9 deg) — well above the
    ~1.1 deg settling measured on the harness, far below this. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that actual joint arrival and residuals stay distinct from a controller reporting success.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the trajectory arrival code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.

    Technical notes:
    The real case: a 40 degree move that only got 12 degrees along.

    The default goal_tolerance is 0.05 rad (~2.9 deg) — well above the
    ~1.1 deg settling measured on the harness, far below this.
    """
    from ringfour_agent.models.profile import GroupConfig

    commanded = math.radians(40.0)
    reached = math.radians(12.0)
    residual = arrival_residual(["pan"], [commanded],
                                snapshot(pan=reached))
    _name, gap = worst_joint(residual)
    assert gap > GroupConfig.model_fields["goal_tolerance"].default


def test_normal_settling_is_not_a_failure():
    """Normal settling is not a failure

    Categories: sdk
    What it checks: 1.1 degrees is what an arrived joint looks like on this hardware;
    a check that fires on that is a check nobody will keep. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that actual joint arrival and residuals stay distinct from a controller reporting success.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the trajectory arrival code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.

    Technical notes:
    1.1 degrees is what an arrived joint looks like on this hardware;
    a check that fires on that is a check nobody will keep.
    """
    from ringfour_agent.models.profile import GroupConfig

    residual = arrival_residual(
        ["pan"], [math.radians(40.0)],
        snapshot(pan=math.radians(40.0) - math.radians(1.1)))
    _name, gap = worst_joint(residual)
    assert gap < GroupConfig.model_fields["goal_tolerance"].default


# -- a stopped trajectory is an outcome, not an assertion ---------------------

def test_cancelled_run_is_a_status_not_an_assertion():
    """Cancelled run raises a typed execution error

    Categories: sdk
    What it checks: Cancellation must produce a catchable OperationCancelled error with cleanup evidence.

    `run()` used to end with `assert last is not None`, and the iterator
    it drains yields NOTHING when the task is cancelled — so pressing
    STOP raised a bare AssertionError out of the SDK. Not a ringfour
    error, not catchable by anything a mission would write, and compiled
    away entirely under `python -O`, where it would have returned None
    instead. Found on hardware the first time stop() was ever pressed. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that actual joint arrival and residuals stay distinct from a controller reporting success.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the trajectory arrival code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.

    Technical notes:
    Cancellation must produce a catchable OperationCancelled error with cleanup evidence.

    `run()` used to end with `assert last is not None`, and the iterator
    it drains yields NOTHING when the task is cancelled — so pressing
    STOP raised a bare AssertionError out of the SDK. Not a ringfour
    error, not catchable by anything a mission would write, and compiled
    away entirely under `python -O`, where it would have returned None
    instead. Found on hardware the first time stop() was ever pressed.
    """
    import asyncio

    from ringfour.aio.control import ControlSession
    from ringfour.errors import OperationCancelled

    class CancelledOperation:
        async def result(self, timeout=None):
            raise OperationCancelled("operator cancelled trajectory", details={
                "cleanup": {"verified": True, "criterion": "measured-rest"}})

    async def exercise():
        session = ControlSession.__new__(ControlSession)
        async def start(trajectory):
            return CancelledOperation()
        session.start_trajectory = start
        with pytest.raises(OperationCancelled) as caught:
            await session.run(object())
        assert caught.value.details["cleanup"]["verified"] is True

    asyncio.run(exercise())
