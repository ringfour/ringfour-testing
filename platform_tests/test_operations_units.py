"""Milestone B operation lifetimes and model-bound command rejection."""
import asyncio
import time

import pytest
from pydantic import ValidationError
from ringfour.aio.operations import Operation, complete
from ringfour.aio.tasks import Task
from ringfour.errors import CompletionUnverified, OperationCancelled, OperationTimeout, WaitTimeout
from ringfour.models import JointConfiguration, JointTrajectory, JointValue, TaskUpdate


class Connection:
    def __init__(self):
        self.cancels = 0
        self.task = Task(self, 'operation-fixture')
        self.evidence = {'verified': True, 'criterion': 'measured-rest', 'physical_stop': 'uncommissioned'}

    async def call(self, method, params):
        assert method == 'task.cancel'
        self.cancels += 1
        self.task._apply(TaskUpdate(task_id=self.task.id, state='cancelled', sequence=2, cleanup=self.evidence))
        return {'accepted': True, 'already_terminal': False}

    def operation(self):
        return Operation(self.task, lambda identifier, result: result, lambda task: None,
                         resource='group:fixture', cleanup_timeout=.5)


def test_wait_timeout_and_observer_close_leave_work_running():
    """Operation observation does not own execution

    Categories: sdk
    What it checks: Wait expiration and closing an update iterator leave accepted work running; explicit close requests cancellation and joins verified cleanup.
    Why it matters: A dashboard or timed-out waiter must not silently change robot commands.
    If it fails: Repair SDK operation lifetime handling.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only.
    """
    async def exercise():
        conn = Connection()
        op = conn.operation()
        observer = op.updates()
        assert (await anext(observer)).operation_id == op.id
        await observer.aclose()
        assert not conn.task._listeners
        with pytest.raises(WaitTimeout):
            await op.result(.001)
        assert not conn.task.done and conn.cancels == 0
        report = await op.close()
        assert report.verified and conn.cancels == 1
        assert await op.close() == report
        assert conn.cancels == 1
        with pytest.raises(OperationCancelled):
            await op.result()
    asyncio.run(exercise())


def test_convenience_timeout_cancels_and_retains_evidence():
    """Convenience timeout joins cleanup

    Categories: sdk
    What it checks: Blocking convenience timeout cancels unfinished work and includes stopping evidence in OperationTimeout.
    Why it matters: Returning after an action deadline must not abandon motion.
    If it fails: Repair convenience timeout cleanup.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only.
    """
    async def exercise():
        conn = Connection()
        with pytest.raises(OperationTimeout) as caught:
            await complete(conn.operation(), .001)
        assert conn.cancels == 1 and conn.task.done
        assert caught.value.details['cleanup']['verified']
        bad = Connection()
        bad.evidence = None
        with pytest.raises(CompletionUnverified):
            await bad.operation().close()
    asyncio.run(exercise())


def test_retained_report_age_and_delayed_update():
    """Retained progress cannot look freshly measured

    Categories: sdk
    What it checks: Cached report age survives delivery and older sequence updates cannot overwrite newer state.
    Why it matters: Querying old status must not manufacture fresh measurement evidence.
    If it fails: Repair operation provenance and notification ordering.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only.
    """
    async def exercise():
        conn = Connection()
        conn.task._apply(TaskUpdate(task_id=conn.task.id, state='running', sequence=5, report_age=10))
        assert conn.operation().snapshot.observed_age >= 10
        conn.task._apply(TaskUpdate(task_id=conn.task.id, state='running', sequence=4, progress=.1))
        assert conn.task.sequence == 5 and conn.task.progress is None
    asyncio.run(exercise())


def test_trajectory_rejects_mixed_models_units_and_invalid_times():
    """Typed trajectories preserve model identity and native units

    Categories: sdk
    What it checks: Mixed revisions/units, invalid bounds, booleans, nonfinite values and nonincreasing times fail before submission; configurations are immutable.
    Why it matters: Robot commands must not depend on positional list guesses or incompatible models.
    If it fails: Repair model-bound command validation.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only.
    """
    value = JointValue(name='lift', position=.2, unit='m', lower=0, upper=.5)
    config = JointConfiguration(robot='fixture', group='lift', model_revision='v1', joints=(value,))
    for invalid in (True, float('nan'), float('inf'), .6):
        with pytest.raises(ValidationError):
            JointValue(name='lift', position=invalid, unit='m', lower=0, upper=.5)
    for changed in (config.model_copy(update={'model_revision': 'v2'}),
                    config.model_copy(update={'joints': (value.model_copy(update={'unit': 'rad'}),)})):
        with pytest.raises(ValidationError):
            JointTrajectory(configurations=(config, changed), times=(1, 2))
    for times in ((0,), (float('nan'),), (True,)):
        with pytest.raises(ValidationError):
            JointTrajectory(configurations=(config,), times=times)
    with pytest.raises(TypeError):
        config.positions['lift'] = .3
    with pytest.raises(ValidationError):
        config.group = 'other'


def test_pending_queries_do_not_overtake_agent_sequence():
    """Pending queries preserve authoritative sequence zero

    Categories: sdk
    What it checks: Repeated pending status queries cannot invent sequence numbers that suppress the real running/terminal updates.
    Why it matters: An accepted operation must remain observable even if polled before its task starts.
    If it fails: Repair task sequence handling.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only.
    """
    async def exercise():
        conn=Connection()
        for _ in range(5):conn.task._apply(TaskUpdate(task_id=conn.task.id,state='pending',sequence=0))
        assert conn.task.sequence==0
        conn.task._apply(TaskUpdate(task_id=conn.task.id,state='succeeded',sequence=1,result={'verified':True}))
        assert await conn.operation().result()=={'verified':True}
    asyncio.run(exercise())


def test_execution_timeout_is_not_an_observer_wait_timeout():
    """Terminal execution failure keeps its error type

    Categories: sdk
    What it checks: A failed agent operation carrying TimeoutError is not mislabeled as WaitTimeout with work supposedly still running.
    Why it matters: Observer waiting and terminal execution failure require different recovery decisions.
    If it fails: Repair operation error translation.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only.
    """
    async def exercise():
        from ringfour.models import TaskError
        from ringfour.errors import TimeoutError
        conn=Connection()
        conn.task._apply(TaskUpdate(task_id=conn.task.id,state='failed',sequence=1,
            error=TaskError(type='TimeoutError',message='adapter execution deadline')))
        with pytest.raises(TimeoutError) as caught:await conn.operation().result()
        assert not isinstance(caught.value,WaitTimeout) and conn.task.done
    asyncio.run(exercise())
