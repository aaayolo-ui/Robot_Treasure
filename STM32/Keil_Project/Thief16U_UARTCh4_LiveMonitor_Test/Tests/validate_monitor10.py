"""MONITOR10 provenance, saved MONITOR7 function equivalence and regressions."""
from pathlib import Path
import re,difflib,hashlib,json
root=Path(__file__).resolve().parents[1];base=root/'Tests/before_monitor7_restore'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
read=lambda p:json.loads(p.read_text(encoding='utf-8'))
axf=root/'MDK-ARM'/root.name/(root.name+'.axf');firmware=sha(axf)
old=read(base/'VALIDATION_MONITOR9.json')
assert sha(base/'original.axf')==old['firmware_sha256']
before=read(base/'regression_before.json');after=read(root/'Tests/results_monitor7_restore.json')
assert before['axf_sha256']==old['firmware_sha256'] and len(before['failures'])==1
assert before['test_sha256']==after['test_sha256']==sha(root/'Tests/test_monitor7_restore.py')
assert after['axf_sha256']==firmware and not after['failures'] and after['cases']['overshoot_axis']==4
for name,value in old['files'].items():
    if name.endswith(('.c','.h','.py')) and name not in ['User/App_ThiefStrategy.c','Tests/test_yaw_reference.py']:
        assert sha(root/name)==value,name
def function(src,name):
    match=re.search(r'^static [^;{}]*\b'+name+r'\([^;{}]*\)\s*\{',src,re.M)
    assert match,name
    return src[match.start():src.index('\n}',match.end())+2]
source=(root/'User/App_ThiefStrategy.c').read_text(encoding='utf-8')
mon7=root/'Tests/before_turn_resume/User/App_ThiefStrategy.c'
original=mon7.read_text(encoding='utf-8')
mon7_report=read(root/'Tests/before_turn_resume/VALIDATION_MONITOR7.json')
assert sha(mon7)==mon7_report['files']['User/App_ThiefStrategy.c']
same_functions={}
for name in ['Route_DriveLeftTurn','Route_DriveRightTurn','Route_HasReachedTurnAngle',
             'Route_EnterAfterTurnForward','Route_AlignTreasureTurn','Route_UpdateYawReference']:
    assert function(source,name)==function(original,name),name
    same_functions[name]=hashlib.sha256(function(source,name).encode()).hexdigest()
assert 'turn_coast_braking' not in source and 'Route_TurnPwm' not in source
assert sha(root/'Tests/test_yaw_reference.py')==sha(root/'Tests/before_turn_resume/Tests/test_yaw_reference.py')
extra={}
for stem in ['yaw_reference','turn_resume']:
    r=read(root/'Tests'/('results_'+stem+'.json'))
    assert r['axf_sha256']==firmware and not r['failures']
    assert r['test_sha256']==sha(root/'Tests'/('test_'+stem+'.py'))
    extra[stem]=r['cases']
suite=read(root/'Tests/results_monitor_suite.json')
assert len(suite)==22 and len({r['name'] for r in suite})==22
for row in suite:
    assert row['exit_code']==0 and row['axf_sha256']==firmware,row['name']
    assert row['log_sha256']==sha(root/'Tests'/('monitor_'+row['name']+'.log'))
supplemental={}
for stem,count in [('route_identity',92),('exit_stall',988),('road_probe',1798),('left800',442)]:
    r=read(root/'Tests'/('results_'+stem+'.json'))
    assert r['axf_sha256']==firmware and r['checks']==count
    assert not r.get('failures') and r.get('failure') is None
    assert r['test_sha256']==sha(root/'Tests'/('test_'+stem+'.py'))
    supplemental[stem]=count
pure=read(root/'Tests/results_16m.json')
assert pure['test_failures']==0 and pure['test_done']==1
for name,value in pure['source_sha256'].items():assert sha(root/name)==value,name
assert '0 Error(s), 0 Warning(s)' in (root/'build_monitor10.log').read_text(encoding='utf-8',errors='replace')
assert 'UARTCH4_16U_MONITOR10' in source
changed=['User/App_ThiefStrategy.c','Tests/test_monitor7_restore.py','Tests/test_yaw_reference.py',
         'Tests/restore_monitor7.py','README.md','AGENTS.md','Docs/修改规范与回归清单.md']
patch=[]
for name in changed:
    previous=(base/name).read_text(encoding='utf-8') if (base/name).exists() else ''
    patch.extend(difflib.unified_diff(previous.splitlines(keepends=True),
        (root/name).read_text(encoding='utf-8').splitlines(keepends=True),
        fromfile='MONITOR9/'+name,tofile='MONITOR10/'+name))
(root/'MONITOR7_RESTORE_CHANGESET.patch').write_text(''.join(patch),encoding='utf-8')
files=changed+['Tests/validate_monitor10.py','MONITOR7_RESTORE_CHANGESET.patch','build_monitor10.log',
    'Tests/results_monitor7_restore.json','Tests/results_yaw_reference.json','Tests/results_turn_resume.json',
    'Tests/results_monitor_suite.json']+['Tests/results_'+stem+'.json' for stem in supplemental]
data=dict(firmware='UARTCH4_16U_MONITOR10',firmware_sha256=firmware,hex_sha256=sha(axf.with_suffix('.hex')),
    before_failures=before['failures'],new_cases=after['cases'],extra=extra,suite=suite,supplemental=supplemental,
    saved_monitor7_source_sha256=sha(mon7),exact_monitor7_functions=same_functions,
    files={name:sha(root/name) for name in files},hardware_tested=False,flashed=False,serial_opened=False,
    limitations=['Saved MONITOR7 turn/trim functions restored exactly; the current combined firmware still needs physical validation.',
    'Known-axis over110degree recovery is limited to N004/N006 with learned gyro direction.',
    'Physical traction, gyro delay, centering and actual escape performance remain unverified.'])
(root/'VALIDATION_MONITOR10.json').write_text(json.dumps(data,ensure_ascii=False,indent=2),encoding='utf-8')
print('MONITOR10 verified: six saved MONITOR7 functions match, real PI resume, over110degree recovery and22 ARM regressions; hardware untested.')
