"""Standalone offline adapter. Run with a Python that has MuJoCo installed."""
import hashlib
import json
from pathlib import Path
import sys


def sample(model_path, request, output):
    import mujoco
    import numpy as np
    model=(mujoco.MjModel.from_binary_path(model_path) if Path(model_path).suffix=='.mjb' else mujoco.MjModel.from_xml_path(model_path))
    data=mujoco.MjData(model)
    output=Path(output);output.mkdir(parents=True,exist_ok=True)
    (output/'request.json').write_text(json.dumps(request,indent=2))
    (output/'adapter.py').write_bytes(Path(__file__).read_bytes())
    mujoco.mj_saveModel(model,str(output/'model.mjb'))
    digest=hashlib.sha256((output/'model.mjb').read_bytes()).hexdigest()
    names={mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_JOINT,j):j for j in range(model.njnt)}
    body_names={i:mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_BODY,i) for i in range(model.nbody)}
    allowed={tuple(sorted(pair)) for pair in request['pairs']}
    robot_links=set(request['links'])
    baseline=data.qpos.copy()
    missing=[name for name,jid in names.items() if name in request['limits'] and model.jnt_type[jid] in (mujoco.mjtJoint.mjJNT_HINGE,mujoco.mjtJoint.mjJNT_SLIDE) and name not in request['positions']]
    for name,value in request['positions'].items():
        jid=names.get(name)
        if jid is not None and model.jnt_type[jid] in (mujoco.mjtJoint.mjJNT_HINGE,mujoco.mjtJoint.mjJNT_SLIDE):
            baseline[model.jnt_qposadr[jid]]=value
    geometry_by_link={}
    for geom in range(model.ngeom):
        # Both zero marks visual-only geometry in these exported models.
        # Other masks affect dynamics, not a direct geometric distance query.
        if model.geom_contype[geom] or model.geom_conaffinity[geom]:
            geometry_by_link.setdefault(body_names[int(model.geom_bodyid[geom])],[]).append(geom)
    uncovered=[list(pair) for pair in sorted(allowed) if any(link not in geometry_by_link for link in pair)]
    queries=[(pair,a,b) for pair in sorted(allowed)
             for a in geometry_by_link.get(pair[0],[]) for b in geometry_by_link.get(pair[1],[])]
    groups={};rng=np.random.default_rng(request.get('seed',0))
    for group,joints in request['groups'].items():
        absent=[j for j in joints if j not in names]
        if missing:
            groups[group]={'error':'Full measured context is missing simulator joints: '+', '.join(missing),'candidates':[]};continue
        if absent:
            groups[group]={'error':'Simulator lacks named joints: '+', '.join(absent),'candidates':[]};continue
        bounds=[]
        for name in joints:
            jid=names[name];limits=request['limits'].get(name)
            if model.jnt_type[jid] not in (mujoco.mjtJoint.mjJNT_HINGE,mujoco.mjtJoint.mjJNT_SLIDE) or not limits:
                absent.append(name);continue
            lo,hi=limits
            if model.jnt_limited[jid]:lo=max(lo,float(model.jnt_range[jid,0]));hi=min(hi,float(model.jnt_range[jid,1]))
            if not np.isfinite([lo,hi]).all() or not lo<hi:absent.append(name)
            bounds.append((name,int(model.jnt_qposadr[jid]),lo,hi))
        if absent:
            groups[group]={'error':'Missing compatible scalar limits: '+', '.join(absent),'candidates':[]};continue
        candidates=[];counts={'clear':0,'colliding':0,'unknown':0};ignored=set()
        for index in range(request.get('samples',100)):
            data.qpos[:]=baseline
            q={}
            for name,address,lo,hi in bounds:
                value=float(np.clip(baseline[address],lo,hi)) if index==0 else float(rng.uniform(lo,hi))
                data.qpos[address]=value;q[name]=value
            # Forward kinematics/contact computation only: no physics stepping,
            # actuator command, ROS connection or modification of the live model.
            mujoco.mj_forward(model,data)
            contacts=set()
            for pair,geom_a,geom_b in queries:
                distance=mujoco.mj_geomDistance(model,data,geom_a,geom_b,1.0,None)
                if distance < -1e-6:contacts.add(pair)
            label='colliding' if contacts else 'unknown' if uncovered else 'clear'
            if counts[label]<3:
                candidates.append({'id':f'{group}:{index}','label':label,'positions':q,'state':request['positions'],
                                   'expected_pairs':[list(p) for p in sorted(contacts)],'reviewed':False})
                counts[label]+=1
            if counts['clear']>=3 and counts['colliding']>=3:break
        groups[group]={'candidates':candidates,'ignored_contacts':[list(p) for p in sorted(ignored)],
                       'covered_links':sorted(robot_links.intersection(body_names.values())),
                       'uncovered_pairs':uncovered,'coverage_complete':not uncovered}
    result={'format':1,'oracle':'mujoco','adapter_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),'method':'direct_geometry_distance','penetration_epsilon_m':1e-6,'oracle_version':mujoco.__version__,'model_sha256':digest,
            'description_sha':request['description_sha'],'seed':request.get('seed',0),
            'requested_samples':request.get('samples',100),'groups':groups,
            'review_required':True,'note':'Independent direct geometry distances for aligned pairs. Dynamics contact masks do not gate queries; visual-only geometry is excluded. Review geometry and collision exclusions before use.'}
    (output/'fixtures.json').write_text(json.dumps(result,indent=2))
    return result


if __name__=='__main__':
    sample(sys.argv[1],json.loads(Path(sys.argv[2]).read_text()),sys.argv[3])
