"""MONITOR11: whole-map turn coverage, N011 corridors and pre-edit checklist."""
from pathlib import Path
import re,difflib,hashlib,json
root=Path(__file__).resolve().parents[1];base=root/'Tests/before_general_turns'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
read=lambda p:json.loads(p.read_text(encoding='utf-8'))
axf=root/'MDK-ARM'/root.name/(root.name+'.axf');firmware=sha(axf)
old=read(base/'VALIDATION_MONITOR10.json')
assert sha(base/'original.axf')==old['firmware_sha256']
before=read(base/'regression_before.json');general=read(root/'Tests/results_general_turns.json')
assert before['axf_sha256']==old['firmware_sha256'] and len(before['failures'])==3
assert before['test_sha256']==general['test_sha256']==sha(root/'Tests/test_general_turns.py')
assert general['axf_sha256']==firmware and not general['failures']
matrix=general['cases']['all_map_quarter_turns']
assert matrix['combinations']==1016 and 78 in matrix['nodes'] and general['cases']['n078_original_log']==4
assert general['cases']['n011_exit_policy']==dict(body_directions=4,world_threats=2,blocked_cases=3)
changed=['User/App_ThiefStrategy.c','Algorithm/ThiefStrategy/ThiefMapPlanner.c','Tests/test_general_turns.py',
 'Tests/test_yaw_reference.py','Tests/test_monitor7_restore.py','Tests/test_documents24.py',
 'Tests/test_app_runtime.py','Tests/test_documents15.py','Tests/test_documents22.py','Tests/test_speed16t.py',
 'README.md','AGENTS.md','Docs/修改规范与回归清单.md']
for name,value in old['files'].items():
    if name.endswith(('.c','.h','.py')) and name not in changed:assert sha(root/name)==value,name
def function(src,name):
    match=re.search(r'^static [^;{}]*\b'+name+r'\([^;{}]*\)\s*\{',src,re.M);assert match,name
    return src[match.start():src.index('\n}',match.end())+2]
source=(root/'User/App_ThiefStrategy.c').read_text(encoding='utf-8')
mon7=(root/'Tests/before_turn_resume/User/App_ThiefStrategy.c').read_text(encoding='utf-8')
normalized=source.replace('turn_axis_active','treasure_turn_align_active').replace('turn_axis_heading','treasure_turn_heading').replace('Route_AlignTurnAxis','Route_AlignTreasureTurn')
for name in ['Route_DriveLeftTurn','Route_DriveRightTurn','Route_AlignTreasureTurn','Route_UpdateYawReference']:
    assert function(normalized,name)==function(mon7,name),name
assert 'node==4U || node==6U' not in function(source,'Route_EnterAfterTurnForward')
assert 'node==4U || node==6U' not in function(source,'Route_HasReachedTurnAngle')
prefixes={}
for name,marker in [('test_app_runtime.py','def documents8_cases():'),('test_documents15.py','parking_pwm=[]')]:
    actual=(root/'Tests'/name).read_text(encoding='utf-8').split(marker)[0]
    saved=(base/'Tests'/name).read_text(encoding='utf-8').split(marker)[0]
    assert actual==saved,'executed fixture prefix changed: '+name
    prefixes[name]=hashlib.sha256(actual.encode()).hexdigest()
extra={}
for stem in ['yaw_reference','turn_resume','monitor7_restore']:
    r=read(root/'Tests'/('results_'+stem+'.json'))
    assert r['axf_sha256']==firmware and not r['failures']
    assert r['test_sha256']==sha(root/'Tests'/('test_'+stem+'.py'))
    extra[stem]=r['cases']
suite=read(root/'Tests/results_monitor_suite.json')
assert len(suite)==22 and len({r['name'] for r in suite})==22
for row in suite:
    assert row['exit_code']==0 and row['axf_sha256']==firmware,row['name']
    assert row['log_sha256']==sha(root/'Tests'/('monitor_'+row['name']+'.log'))
roots=read(root/'Tests/results_documents15.json')
assert roots['cases']['turns']==34 and not roots['failures']
supplemental={}
for stem,minimum in [('route_identity',92),('exit_stall',988),('road_probe',1798),('left800',442)]:
    r=read(root/'Tests'/('results_'+stem+'.json'))
    assert r['axf_sha256']==firmware and r['checks']>=minimum
    assert not r.get('failures') and r.get('failure') is None
    assert r['test_sha256']==sha(root/'Tests'/('test_'+stem+'.py'))
    supplemental[stem]=r['checks']
pure=read(root/'Tests/results_16m.json')
assert pure['test_failures']==0 and pure['test_done']==1 and pure['test_case']==378
for name,value in pure['source_sha256'].items():assert sha(root/name)==value,name
assert '0 Error(s), 0 Warning(s)' in (root/'build_monitor11.log').read_text(encoding='utf-8',errors='replace')
assert 'UARTCH4_16U_MONITOR11' in source
norm=(root/'Docs/修改规范与回归清单.md').read_text(encoding='utf-8')
assert '每次改代码前必读：防遗漏核对表' in norm
assert '防遗漏核对表' in (root/'AGENTS.md').read_text(encoding='utf-8')
patch=[]
for name in changed:
    previous=(base/name).read_text(encoding='utf-8') if (base/name).exists() else ''
    patch.extend(difflib.unified_diff(previous.splitlines(keepends=True),
        (root/name).read_text(encoding='utf-8').splitlines(keepends=True),
        fromfile='MONITOR10/'+name,tofile='MONITOR11/'+name))
(root/'GENERAL_TURNS_CHANGESET.patch').write_text(''.join(patch),encoding='utf-8')
files=changed+['Tests/validate_monitor11.py','GENERAL_TURNS_CHANGESET.patch','build_monitor11.log',
 'Tests/results_general_turns.json','Tests/results_16m.json','Tests/results_monitor_suite.json',
 'Tests/results_documents15.json','Tests/results_documents24.json']
files+=['Tests/results_'+stem+'.json' for stem in list(extra)+list(supplemental)]
data=dict(firmware='UARTCH4_16U_MONITOR11',firmware_sha256=firmware,hex_sha256=sha(axf.with_suffix('.hex')),
 before_failures=before['failures'],general=general['cases'],general_checks=general['checks'],root_turn_cases=34,
 extra=extra,suite=suite,supplemental=supplemental,pure_checks=pure['test_checks'],pure_cases=pure['test_case'],
 executed_fixture_prefix_sha256=prefixes,files={name:sha(root/name) for name in files},
 hardware_tested=False,flashed=False,serial_opened=False,
 limitations=['1016 map-turn combinations use synthetic wheel/gray/gyro feedback; physical coverage, traction and settling still require vehicle tests.',
 'N011 corridors are preferences with real blocked/observed-exit fallback, not a guarantee against moving opponents.',
 'Legacy recovery fixtures now actually complete the common turn stage before injecting a disturbed exit; old scripts are preserved.'])
(root/'VALIDATION_MONITOR11.json').write_text(json.dumps(data,ensure_ascii=False,indent=2),encoding='utf-8')
print('MONITOR11 verified:1016 legal map turns,N011 world-direction/fallback cases,34 root cases,22 ARM scripts and pure algorithms; hardware untested.')
