"""Read-only operator metadata for every configurable live-test prerequisite."""
import importlib.util
from pathlib import Path


def requirement_metadata(welcome, settings, source=None):
    groups=[]
    for name,info in welcome.groups.items():
        recipe=settings.get('groups',{}).get(name,{})
        groups.append({'name':name, **info.model_dump(mode='json'),
                       'sessions_configured':bool(recipe.get('allow_sessions') or recipe.get('waypoints')),
                       'waypoints':len(recipe.get('waypoints',[])),
                       'clear_poses':len(recipe.get('clear_configurations',[])),
                       'collision_poses':len(recipe.get('collision_configurations',[]))})
    snapshot_dir=settings.get('artifacts',{}).get('snapshot_dir')
    snapshot=None
    from ringfour.snapshot import Snapshot
    if welcome.description_sha:
        snapshot=Snapshot.find(welcome.description_sha,root=snapshot_dir or (source['path'] and str(Path(source['path']).parent) if source else None))
    return {'groups':groups,'actions':welcome.actions,
            'capabilities':list(welcome.capabilities), 'frames':(source or {}).get('data',{}).get('frames',{}),
            'profile_selected':source is not None,'description_sha':welcome.description_sha,
            'kinematics_installed':importlib.util.find_spec('roboticstoolbox') is not None,
            'snapshot':str(snapshot.path) if snapshot else None,
            'reach_map':bool(snapshot and snapshot.reach),
            'separate_descriptions':list(welcome.group_description_shas)}


def group_state(uri,name):
    from ringfour import Robot
    from ringfour.errors import CapabilityNotSupported
    with Robot(uri) as robot:
        if name not in robot.groups:
            raise ValueError('Select a declared robot group')
        group=robot.groups[name]
        states=group.get_joints()
        import math
        if any(not math.isfinite(state.position) for state in states.values()):
            raise ValueError('Joint feedback contains missing or non-finite positions')
        description=robot.description
        joints=[]
        for joint in group.joint_names:
            spec=description.joints.get(joint) if description else None
            kind=spec.type if spec else None
            joints.append({'name':joint,'position':states[joint].position,'limits':states[joint].limits,
                           'type':kind,'unit':'m' if kind=='prismatic' else 'rad' if kind in ('continuous','revolute') else 'joint units'})
        try:
            snapshot=group.get_kinematic_snapshot()
            positions=snapshot.positions
            stamp=snapshot.stamp
        except CapabilityNotSupported:
            positions=group.get_joint_positions()
            stamp=None
        return {'name':name,'joints':joints,'positions':positions,'stamp':stamp,'description_sha':robot.description_sha}
