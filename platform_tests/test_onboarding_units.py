"""Mesh preparation uses the same transforms whether run on ROS or a test host."""
from pathlib import Path
from types import SimpleNamespace
import math
import pytest
from ringfour.description import CollisionShape
from ringfour.meshes import Bounds
from ringfour.onboarding import measure_meshes


def test_preparation_preserves_mesh_scale_and_collision_origin(tmp_path,monkeypatch):
    """Mesh preparation preserves geometry transforms

    Categories: sdk
    What it checks: Checks that mesh scale and collision origin combine into the correct fitted box.
    Why it matters: Reusable geometry must preserve the robot model and expose missing assets.
    If it fails: Inspect mesh preparation transforms and missing-file handling in the SDK.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline regression using synthetic geometry; no running robot is accessed.
    """
    import ringfour.onboarding as module
    mesh=tmp_path/'mesh.stl';mesh.write_bytes(b'fixture')
    bounds=Bounds(centre=(1,0,0),half=(.1,.2,.3),triangles=1,rpy=(0,0,0))
    monkeypatch.setattr(module,'bounds_of_stl',lambda path:bounds)
    shape=CollisionShape(link='part',kind='mesh',half=None,mesh='package://fixture/mesh.stl',scale=(2,2,2),origin_xyz=(3,4,5),origin_rpy=(0,0,math.pi/2))
    result=measure_meshes(SimpleNamespace(collisions={'part':(shape,)}),lambda uri:mesh)
    assert result['part'][0].origin_xyz==pytest.approx((3,6,5))
    assert result['part'][0].half==pytest.approx((.2,.4,.6))
    assert result['part'][0].modelled


def test_unresolved_mesh_is_never_replaced_with_invented_geometry():
    """Unresolved meshes remain explicitly missing

    Categories: sdk
    What it checks: Checks that unavailable mesh files cannot become fabricated collision geometry.
    Why it matters: Reusable geometry must preserve the robot model and expose missing assets.
    If it fails: Inspect mesh preparation transforms and missing-file handling in the SDK.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline regression using synthetic geometry; no running robot is accessed.
    """
    shape=CollisionShape(link='missing',kind='mesh',half=None,mesh='package://missing/mesh.stl')
    assert measure_meshes(SimpleNamespace(collisions={'missing':(shape,)}),lambda uri:None)=={}
