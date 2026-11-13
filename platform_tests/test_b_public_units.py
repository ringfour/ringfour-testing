"""Public B API and semantic result contracts, independent of robot morphology."""
import asyncio,inspect
from types import SimpleNamespace
import pytest
from ringfour import Robot,JointConfiguration,JointValue,Pose
from ringfour.aio import Robot as AioRobot
from ringfour.aio.execution import route_operation,tool_operation
from ringfour.aio.tasks import Task
from ringfour.errors import CompletionUnverified


def test_retired_surface_and_local_properties():
    """One public execution/read vocabulary

    Categories: sdk
    What it checks: Superseded aliases/remote properties are absent and retained property getters do not invoke the blocking RPC runner.
    Why it matters: Properties must be local and execution must not depend on iteration.
    If it fails: Repair sync/async API migration and its documentation.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only.
    """
    from ringfour.sensing import Arm,Tool,Camera
    from ringfour.control import ControlSession
    for cls in (Robot,AioRobot):
        for name in ('navigate_to_async','navigate','patrol','joint_groups','watch','pose','velocity'):
            assert not hasattr(cls,name),(cls,name)
        assert callable(cls.start_navigation) and callable(cls.start_route) and callable(cls.get_pose)
    assert not hasattr(Arm,'gripper') and not hasattr(ControlSession,'follow')
    for cls in (Robot,Arm,Tool,Camera,ControlSession):
        for name,value in vars(cls).items():
            if isinstance(value,property):assert '_runner.run' not in inspect.getsource(value.fget),name


def test_sync_power_reads_are_values_and_unknown_stays_unknown():
    """Sync/async power reads have the same value contract

    Categories: sdk
    What it checks: Synchronous battery/source reads return values, not coroutine objects; absent source remains None in both facades.
    Why it matters: Unknown power state must not masquerade as external power.
    If it fails: Repair power read wrappers and availability handling.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only.
    """
    from ringfour.client import PowerAPI
    from ringfour.aio.ui import PowerAPI as AsyncPower
    class Connection:
        async def call(self,method):return {'battery':None} if method=='power.battery' else {'on_battery':None}
    async_power=AsyncPower(Connection())
    power=PowerAPI(SimpleNamespace(run=asyncio.run),async_power)
    assert power.get_battery() is None and power.get_is_on_battery() is None
    assert asyncio.run(async_power.get_is_on_battery()) is None
    assert not inspect.iscoroutinefunction(PowerAPI.get_battery)


def test_route_and_tool_results_require_complete_native_evidence():
    """Finite route and tool results validate every postcondition

    Categories: sdk
    What it checks: Route result covers each requested endpoint and tool result preserves native units/residual/rest; missing arrivals or out-of-tolerance endpoint evidence rejects.
    Why it matters: Generic accepted work must not convert acknowledgement into success.
    If it fails: Repair typed result producers.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only.
    """
    robot=SimpleNamespace(_welcome=SimpleNamespace(features={'operation_cleanup_timeout':1.}))
    task=Task(object(),'fixture')
    stop={'verified':True,'criterion':'native-joint-rest','physical_stop':'uncommissioned'}
    arrival={'arrival_verification':{'verified':True,'final_pose':Pose(frame='map').model_dump(),
        'position_error':.01,'orientation_error':.02,'position_tolerance':.1,'orientation_tolerance':.1},'stop_verification':stop}
    route=route_operation(robot,task,2)
    assert len(route._result_factory('fixture',{'arrivals':[arrival,arrival]}).arrivals)==2
    with pytest.raises(CompletionUnverified):route._result_factory('fixture',{'arrivals':[arrival]})
    tool=tool_operation(robot,task)
    evidence={'group':'tool-group','joint':'slide','position':.12,'unit':'m','target':.12,'residual':0.,'tolerance':.001,'stop_verification':stop}
    assert tool._result_factory('fixture',evidence).unit=='m'
    with pytest.raises(CompletionUnverified):tool._result_factory('fixture',{**evidence,'residual':.01})
