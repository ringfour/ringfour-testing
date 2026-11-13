"""Preparation must not turn missing coverage or stale models into accepted fixtures."""
import copy
import json
from pathlib import Path
import pytest
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import preparation


def bundle():
    return {'description_sha':'a'*64,'geometry_ready':True,'snapshot_root':'/recorded/robot',
            'fixtures':{'description_sha':'a'*64,'groups':{'arbitrary':{'coverage_complete':True,'candidates':[
                {'id':'clear','label':'clear','positions':{'axis':.2},'state':{'mount':.4}},
                {'id':'hit','label':'colliding','positions':{'axis':.8},'state':{'mount':.4}},
                {'id':'unknown','label':'unknown','positions':{'axis':.5},'state':{'mount':.4}}]}}}}


def test_reviewed_fixture_export_keeps_full_context_and_geometry_binding():
    result=preparation.reviewed_settings(bundle(),['clear','hit'],'a'*64)
    assert result['groups']['arbitrary']['clear_configurations']==[{'axis':.2}]
    assert result['groups']['arbitrary']['collision_configurations']==[{'axis':.8}]
    assert result['groups']['arbitrary']['clear_state']=={'mount':.4}
    assert result['artifacts']['snapshot_dir']=='/recorded/robot'


@pytest.mark.parametrize('change',['model','incomplete_geometry','uncovered_pair','unknown_label','foreign_id','different_context'])
def test_fixture_review_refuses_invalid_evidence(change):
    b=bundle();ids=['clear'];sha='a'*64
    if change=='model':sha='b'*64
    if change=='incomplete_geometry':b['geometry_ready']=False
    if change=='uncovered_pair':b['fixtures']['groups']['arbitrary']['coverage_complete']=False
    if change=='unknown_label':ids=['unknown']
    if change=='foreign_id':ids=['outside-bundle']
    if change=='different_context':
        candidate=copy.deepcopy(b['fixtures']['groups']['arbitrary']['candidates'][0]);candidate['id']='other';candidate['state']['mount']=.9
        b['fixtures']['groups']['arbitrary']['candidates'].append(candidate);ids.append('other')
    with pytest.raises(ValueError):preparation.reviewed_settings(b,ids,sha)


def test_bundle_discovery_is_bound_to_model_and_survives_job_restart(tmp_path,monkeypatch):
    monkeypatch.setattr(preparation,'BUNDLES',tmp_path)
    b=bundle();b.update(robot='arbitrary',recorded='2026-10-08',source_container='simulation')
    path=tmp_path/'robot-fixture'/'snapshot'/(b['description_sha']);path.mkdir(parents=True)
    (path/'bundle.json').write_text(json.dumps(b))
    assert preparation.matching_bundles('b'*64)==[]
    assert preparation.matching_bundles('a'*64)[0]['key']=='robot-fixture'
    loaded=preparation.load_bundle('robot-fixture','a'*64)
    assert preparation.status(loaded['id'])['result']==b
    with pytest.raises(ValueError):preparation.load_bundle('../robot-fixture','a'*64)


def test_container_paths_are_not_shell_arguments():
    with pytest.raises(ValueError):preparation.container_name('container; command')
