"""Prototype reusable geometry bundles and offline, independently labelled fixtures."""
from dataclasses import replace
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import re
import shlex
import shutil
import subprocess
import tempfile
import threading
import uuid

HERE=Path(__file__).resolve().parent
BUNDLES=HERE/'configurations'/'bundles'
JOBS={}
LOCK=threading.Lock()


def command(argv,timeout=60):
    result=subprocess.run(argv,stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True,timeout=timeout)
    if result.returncode:
        raise ValueError(result.stderr.strip()[-2000:] or result.stdout.strip()[-2000:] or 'Environment command failed')
    return result.stdout.strip()


def container_name(value):
    if not isinstance(value,str) or not re.fullmatch(r'[a-zA-Z0-9][a-zA-Z0-9_.-]*',value):
        raise ValueError('Choose a running Docker container')
    return value


def environments():
    entries=[]
    for name in command(['docker','ps','--format','{{.Names}}']).splitlines():
        try:
            model=None
            for line in command(['docker','exec',name,'ps','-eo','args']).splitlines():
                args=shlex.split(line)
                if '--model' in args:
                    model=args[args.index('--model')+1];break
            if model:entries.append({'container':name,'model':model,'python':'/opt/mjviser/bin/python'})
        except (ValueError,subprocess.TimeoutExpired):continue
    return entries


class DockerMeshes:
    def __init__(self,container,cache):
        self.container=container_name(container);self.cache=Path(cache);self.files={}
        script='''for prep_setup in /opt/ros/"$ROS_DISTRO"/setup.bash /opt/*/install/setup.bash /workspace/*/install/setup.bash /root/workspace/install/setup.bash /workspace/install/setup.bash; do
 if [ -f "$prep_setup" ]; then source "$prep_setup" >/dev/null; fi
done
python3 -c 'import json;from ament_index_python.packages import get_packages_with_prefixes;print(json.dumps(get_packages_with_prefixes()))' '''
        prefixes=json.loads(command(['docker','exec',self.container,'bash','-lc',script]))
        self.share={k:str(Path(v)/'share'/k) for k,v in prefixes.items()}

    def __call__(self,uri):
        from ringfour.meshes import resolve_mesh_path
        if uri in self.files:return self.files[uri]
        source=resolve_mesh_path(uri,self.share)
        if source is None:return None
        target=self.cache/(hashlib.sha256(uri.encode()).hexdigest()+source.suffix)
        try:command(['docker','cp','-L',f'{self.container}:{source}',str(target)])
        except ValueError:return None
        self.files[uri]=target
        return target


def prepare(uri,container,model_path='',python='/opt/mjviser/bin/python',samples=100,progress=lambda stage:None):
    from ringfour import Robot
    from ringfour.onboarding import measure_meshes
    from ringfour.collision import CollisionModel
    from ringfour.snapshot import Snapshot
    from requirements import group_state
    container=container_name(container)
    if not isinstance(samples,int) or isinstance(samples,bool) or not 1<=samples<=1000:
        raise ValueError('Sample count must be between 1 and 1000')
    progress('Reading live model and group feedback')
    with Robot(uri) as robot:
        description=robot.description
        if description is None:raise ValueError('Agent provides no live description')
        # Typed group metadata and identity come from the same live connection.
        from conformance import discover
        welcome=discover(uri)
        groups={n:list(info.joints) for n,info in welcome.groups.items()}
        if welcome.description_sha != description.sha:raise ValueError('Robot model changed during preparation')
    positions=group_state(uri,next(iter(groups)))['positions'] if groups else {}
    if any(not math.isfinite(value) for value in positions.values()):
        raise ValueError('Full measured joint context contains non-finite values')
    slug=re.sub(r'[^a-zA-Z0-9_.-]','-',welcome.profile.name)
    with tempfile.TemporaryDirectory(prefix='ringfour-prepare-') as temporary:
        stage=Path(temporary);geometry=stage/'geometry';geometry.mkdir()
        progress('Resolving collision meshes in the ROS environment')
        resolver=DockerMeshes(container,stage)
        hulls=measure_meshes(description,resolver)
        missing=[]
        for link,shapes in description.collisions.items():
            expected=sum(not shape.modelled and bool(shape.mesh) for shape in shapes)
            if len(hulls.get(link,()))!=expected:missing.append(link)
        hulls={link:shapes for link,shapes in hulls.items() if link not in missing}
        snapshot=Snapshot(robot=welcome.profile.name,sha=description.sha,description=description,hulls=hulls,reach=None,
                          meta={'recorded':datetime.now(timezone.utc).isoformat(),'source':'e2e preparation prototype',
                                'container':container,'unmodelled':missing,'reach_status':'not generated'})
        resolved=replace(description,collisions=snapshot.collisions)
        progress('Building collision exclusions using the existing SDK onboarding algorithm')
        collision=CollisionModel.build(resolved)
        snapshot=replace(snapshot,meta={**snapshot.meta,'pairs':len(collision.pairs),'always_touching':list(collision.always_touching)})
        snapshot.write(geometry)
        geometry_sha=hashlib.sha256((geometry/'hulls.json').read_bytes()).hexdigest()
        destination=BUNDLES/(slug+'-'+geometry_sha[:12])/'snapshot'/description.sha
        fixtures=None
        fixtures_error=None
        if model_path and not missing:
            progress('Sampling an independent MuJoCo model offline; no live commands')
            remote='/tmp/ringfour-prepare-'+uuid.uuid4().hex
            payload={'description_sha':description.sha,'groups':groups,'positions':positions,
                     'limits':{n:j.limits for n,j in description.joints.items()},'links':list(description.links),
                     'pairs':list(collision.pairs),'samples':samples,'seed':0}
            (stage/'request.json').write_text(json.dumps(payload))
            command(['docker','exec',container,'mkdir','-p',remote])
            try:
                command(['docker','cp',str(HERE.parent/'mujoco_fixtures.py'),f'{container}:{remote}/adapter.py'])
                command(['docker','cp',str(stage/'request.json'),f'{container}:{remote}/request.json'])
                command(['docker','exec',container,python,remote+'/adapter.py',model_path,remote+'/request.json',remote+'/output'],timeout=60)
                command(['docker','cp',f'{container}:{remote}/output',str(stage/'fixtures')])
                fixtures=json.loads((stage/'fixtures'/'fixtures.json').read_text())
            except (ValueError,OSError,subprocess.TimeoutExpired) as exc:
                fixtures_error=str(exc)
            finally:command(['docker','exec',container,'rm','-rf',remote])
        from ringfour.version import CLIENT_VERSION
        manifest={'sdk_version':CLIENT_VERSION,'format':1,'robot':welcome.profile.name,'description_sha':description.sha,
                  'recorded':datetime.now(timezone.utc).isoformat(),'geometry_ready':not missing,
                  'unmodelled_links':missing,'snapshot_root':str(destination.parent.parent),'geometry_sha256':geometry_sha,'snapshot_path':str(destination),
                  'collision_pairs':len(collision.pairs),'always_touching':list(collision.always_touching),
                  'fixture_status':'unreviewed' if fixtures else 'not generated','fixtures':fixtures,'fixtures_error':fixtures_error,
                  'reach_status':'not generated','source_container':container,'source_model':model_path or None}
        with LOCK:
            destination.mkdir(parents=True,exist_ok=True)
            # Preserve an existing recorded reach map; preparation only fills geometry.
            for file in geometry.iterdir():shutil.copy2(file,destination/file.name)
            if fixtures:
                context_sha=hashlib.sha256(json.dumps({'request':payload,'adapter':fixtures.get('adapter_sha256')},sort_keys=True).encode()).hexdigest()[:12]
                target=destination/'fixtures'/(fixtures['model_sha256']+'-'+context_sha);target.parent.mkdir(exist_ok=True)
                if not target.exists():shutil.copytree(stage/'fixtures',target)
                manifest['fixtures_path']=str(target/'fixtures.json')
            (destination/'bundle.json').write_text(json.dumps(manifest,indent=2))
        if discover(uri).description_sha != description.sha:
            raise ValueError('Robot model changed during preparation; this recorded bundle is not current')
        return manifest


def start(request):
    identifier=uuid.uuid4().hex
    job={'id':identifier,'status':'running','stage':'Starting preparation','result':None,'error':None}
    with LOCK:
        if any(j['status']=='running' for j in JOBS.values()):raise ValueError('Preparation is already running')
        JOBS[identifier]=job
    def work():
        try:
            result=prepare(request['uri'],request['container'],request.get('model',''),request.get('python','/opt/mjviser/bin/python'),request.get('samples',100),lambda stage:job.update(stage=stage))
            job.update(status='complete',stage='Bundle ready for review',result=result)
        except Exception as exc:job.update(status='failed',stage='Preparation stopped',error=str(exc))
    threading.Thread(target=work,daemon=True).start()
    return {'id':identifier}


def status(identifier):
    with LOCK:
        if identifier not in JOBS:raise ValueError('Unknown preparation job')
        return dict(JOBS[identifier])


def reviewed_settings(bundle,selected,description_sha):
    if not bundle.get('geometry_ready') or bundle.get('description_sha')!=description_sha:
        raise ValueError('Complete geometry matching the live model is required')
    fixtures=bundle.get('fixtures')
    if not fixtures or fixtures.get('description_sha')!=description_sha:
        raise ValueError('Independent fixture labels matching the live model are required')
    candidates={c['id']:(name,c) for name,g in fixtures['groups'].items() for c in g['candidates']}
    if not isinstance(selected,list) or not selected or any(identifier not in candidates for identifier in selected):
        raise ValueError('Select fixture candidates from this bundle')
    groups={}
    for identifier in selected:
        name,c=candidates[identifier]
        if c['label'] not in ('clear','colliding'):
            raise ValueError('Unknown simulator coverage cannot supply a clear fixture')
        if c['label']=='clear' and not fixtures['groups'][name].get('coverage_complete'):
            raise ValueError('Incomplete simulator coverage cannot establish a clear configuration')
        recipe=groups.setdefault(name,{})
        field='clear_configurations' if c['label']=='clear' else 'collision_configurations'
        state='clear_state' if c['label']=='clear' else 'collision_state'
        if state in recipe and recipe[state]!=c['state']:
            raise ValueError('Selected fixtures have different full-state contexts')
        recipe.setdefault(field,[]).append(c['positions']);recipe[state]=c['state']
    for name,recipe in groups.items():
        recipe['fixture_provenance']={'description_sha':description_sha,
            'geometry_sha256':bundle.get('geometry_sha256',''), 'oracle':fixtures.get('oracle',''),
            'oracle_version':fixtures.get('oracle_version',''), 'model_sha256':fixtures.get('model_sha256',''),
            'method':fixtures.get('method',''), 'adapter_sha256':fixtures.get('adapter_sha256',''), 'reviewed':True,
            'reviewed_at':datetime.now(timezone.utc).isoformat(),
            'candidate_ids':[identifier for identifier in selected if candidates[identifier][0]==name]}
    return {'groups':groups,'artifacts':{'snapshot_dir':bundle['snapshot_root']}}


def matching_bundles(description_sha):
    if not re.fullmatch(r'[a-f0-9]{64}',description_sha or ''):return []
    results=[]
    for path in BUNDLES.glob('*/snapshot/'+description_sha+'/bundle.json'):
        bundle=json.loads(path.read_text())
        results.append({'key':path.parent.parent.parent.name,'robot':bundle['robot'],'recorded':bundle['recorded'],
                        'geometry_ready':bundle['geometry_ready'],'source_container':bundle.get('source_container'),
                        'source_model':bundle.get('source_model')})
    return sorted(results,key=lambda b:b['recorded'],reverse=True)


def load_bundle(key,description_sha):
    if not isinstance(key,str) or not re.fullmatch(r'[a-zA-Z0-9_.-]+',key) or key in ('.','..'):
        raise ValueError('Choose a recorded bundle')
    if not re.fullmatch(r'[a-f0-9]{64}',description_sha or ''):raise ValueError('No live model SHA')
    bundle=json.loads((BUNDLES/key/'snapshot'/description_sha/'bundle.json').read_text())
    if bundle['description_sha']!=description_sha:raise ValueError('Bundle does not match the live model')
    identifier=uuid.uuid4().hex
    JOBS[identifier]={'id':identifier,'status':'complete','stage':'Recorded bundle loaded; review sources before reuse','result':bundle,'error':None}
    return {'id':identifier}
