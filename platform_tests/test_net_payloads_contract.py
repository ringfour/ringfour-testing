"""Network arrays cross real protocol connections without shared memory."""
import asyncio
from dataclasses import replace
import threading

import numpy as np
import pytest
from ringfour import Robot
from ringfour.enums import Transport
from ringfour.aio import Robot as AsyncRobot
from ringfour.aio.transport import Connection
from ringfour.errors import FrameExpired, RingfourError
from ringfour.models.sensing import PayloadRef
from ringfour.payloads import array_from_bytes
from ringfour_agent.bootstrap import build_agent
from ringfour_agent.models.profile import CameraConfig, SlamConfig
from ringfour_agent.profile import load_profile
from .conftest import STUB_PROFILE, _free_tcp_port

pytestmark = pytest.mark.contract


class Sensors:
    tick=0
    def description(self, timeout=0): return None
    def camera_sample(self, name, need_rgb, need_depth):
        self.tick+=1
        return dict(frame_id='optical', stamp=float(self.tick),rgb_stamp=float(self.tick),depth_stamp=float(self.tick),
                    rgb=np.arange(36,dtype=np.uint8).reshape(3,4,3),depth=np.ones((3,4),dtype=np.float32),
                    intrinsics=dict(fx=10.,fy=10.,cx=2.,cy=1.,width=4,height=3))
    def snapshot_transforms(self, frame): return {}
    def lidar_sample(self):
        self.tick+=1
        return dict(ranges=np.array([1.,2.,3.],dtype=np.float32),intensities=np.array([3.,2.,1.],dtype=np.float32),
                    angle_min=0.,angle_max=2.,angle_increment=1.,stamp=float(self.tick),frame='laser',range_min=.1,range_max=10.)
    def map_grid(self):
        return dict(width=2,height=2,resolution=.1,origin={'frame':'map','x':0.,'y':0.},
                    data=np.array([0,100,-1,0],dtype=np.int8))


@pytest.fixture
def net_agent(tmp_path):
    profile=load_profile(STUB_PROFILE)
    spec=profile.spec.model_copy(update={'groups':None,'slam':SlamConfig(map_dir=str(tmp_path/'maps')),
        'cameras':{'base':CameraConfig(rgb='/rgb',depth='/depth',info='/info',optical=True)}})
    profile=replace(profile,spec=spec,info=profile.info.model_copy(update={'capabilities':spec.capabilities}))
    server,ctx=build_agent(profile,unix_path=None,host='127.0.0.1',port=_free_tcp_port(),audit_dir=tmp_path/'audit',ros=Sensors())
    # Any accidental agent-side SHM allocation must fail this suite.
    def forbidden(*args,**kwargs):raise AssertionError('NET must not allocate shared memory')
    ctx.shm.put=forbidden
    loop=asyncio.new_event_loop();thread=threading.Thread(target=loop.run_forever,daemon=True);thread.start()
    asyncio.run_coroutine_threadsafe(server.start(),loop).result(5)
    yield f'ws://127.0.0.1:{server._port}',server,ctx
    asyncio.run_coroutine_threadsafe(server.close(),loop).result(5)
    ctx.audit.close();loop.call_soon_threadsafe(loop.stop);thread.join(5);loop.close()


def test_native_sync_camera_depth_cloud_scan_map_and_stream(net_agent):
    """Read-only TCP negotiates NET; every array family works with no SHM.

    Categories: sdk
    What it checks: Reads every supported array family through the sync SDK over real TCP, with SHM allocation forbidden.
    Why it matters: Native clients must receive exact arrays without relying on the agent filesystem.
    If it fails: Repair payload negotiation, binary ordering or layout validation; do not change robot calibration.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline synthetic fixtures and isolated protocol sockets only; no robot is contacted.
    """
    uri,_,_=net_agent
    with Robot(uri) as robot:
        frame=robot.camera.get_frame()
        assert frame.image.shape==(3,4,3) and frame.image[0,0,1]==1
        assert not frame.image.flags.writeable
        assert robot.camera.get_depth().depth.dtype==np.float32
        rgb,depth=robot.camera.get_synced();assert rgb.id==depth.id
        cloud=robot.camera.get_cloud(voxel=.01)
        assert cloud.points.shape[1]==3 and cloud.colors.shape[1]==3
        scan=robot.lidar.scan();np.testing.assert_array_equal(scan.ranges,[1,2,3])
        grid=robot.get_map();assert grid.width==2
        stream=robot.camera.stream(fps=15)
        assert next(stream).image.shape==(3,4,3);stream.close()
        scanstream=robot.lidar.stream(hz=15)
        assert next(scanstream).ranges.shape==(3,);scanstream.close()
        assert len(robot._robot._conn._payloads)==0


def test_async_payloads_and_connection_local_ids(net_agent):
    """Async payloads and connection local ids

    Categories: sdk
    What it checks: Reads independent concurrent connections and async streams without a shared filesystem.
    Why it matters: Native clients must receive exact arrays without relying on the agent filesystem.
    If it fails: Repair payload negotiation, binary ordering or layout validation; do not change robot calibration.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline synthetic fixtures and isolated protocol sockets only; no robot is contacted.
    """
    uri,_,_=net_agent
    async def run():
        async with await AsyncRobot.connect(uri,transport=Transport.NET) as a:
            async with await AsyncRobot.connect(uri,transport=Transport.NET) as b:
                x,y=await asyncio.gather(a.camera.get_frame(),b.camera.get_frame())
                np.testing.assert_array_equal(x.image,y.image)
                assert (await a.camera.get_depth()).depth.shape==(3,4)
                assert (await a.lidar.scan()).ranges.shape==(3,)
                stream=a.camera.stream(fps=15)
                assert (await anext(stream)).image.shape==(3,4,3)
                await stream.aclose()
    asyncio.run(run())


@pytest.mark.parametrize('dtype,shape,size,data', [
    ('O',(1,),8,b'12345678'),('uint8',(2,),1,b'x'),('float32',(1,),4,b'x'),
    ('uint8',(-1,),1,b'x'),('uint8',(1,)*9,1,b'x')])
def test_invalid_layout_never_becomes_an_array(dtype,shape,size,data):
    """Invalid layout never becomes an array

    Categories: sdk
    What it checks: Rejects object dtype, negative or excessive dimensions and inconsistent raw byte counts.
    Why it matters: Native clients must receive exact arrays without relying on the agent filesystem.
    If it fails: Repair payload negotiation, binary ordering or layout validation; do not change robot calibration.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline synthetic fixtures and isolated protocol sockets only; no robot is contacted.
    """
    ref=PayloadRef(transport=Transport.NET,name='1',dtype=dtype,shape=shape,size=size)
    with pytest.raises(RingfourError):array_from_bytes(ref,data)


def test_eviction_is_oldest_first_and_explicit():
    """Eviction is oldest first and explicit

    Categories: sdk
    What it checks: Overfills the pending network buffer and checks oldest-first eviction and an explicit expired-payload error.
    Why it matters: Native clients must receive exact arrays without relying on the agent filesystem.
    If it fails: Repair payload negotiation, binary ordering or layout validation; do not change robot calibration.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline synthetic fixtures and isolated protocol sockets only; no robot is contacted.
    """
    async def run():
        conn=Connection()
        for i in range(65):conn._handle_notification('payload.header',{'payload_id':i,'kind':'array/raw','meta':{}})
        assert 0 not in conn._payloads and 64 in conn._payloads and len(conn._payloads)==64
        with pytest.raises(FrameExpired):conn.take_payload('0')
        with pytest.raises(RingfourError):conn.take_payload('not-an-id')
        await conn.close()
    asyncio.run(run())
