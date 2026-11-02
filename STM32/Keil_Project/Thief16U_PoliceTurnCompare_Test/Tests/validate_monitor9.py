"""Bind MONITOR9 coast/recovery evidence to compiled ARM firmware."""
from pathlib import Path
import difflib,hashlib,json
root=Path(__file__).resolve().parents[1];base=root/'Tests/before_turn_coast'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
read=lambda p:json.loads(p.read_text(encoding='utf-8'))
axf=root/'MDK-ARM'/root.name/(root.name+'.axf');firmware=sha(axf)
old=read(base/'VALIDATION_MONITOR8.json')
assert sha(base/'original.axf')==old['firmware_sha256']
before=read(base/'regression_before.json');after=read(root/'Tests/results_turn_coast.json')
assert before['axf_sha256']==old['firmware_sha256'] and len(before['failures'])==3
assert before['test_sha256']==after['test_sha256']==sha(root/'Tests/test_turn_coast.py')
assert after['axf_sha256']==firmware and not after['failures']
assert after['cases']==dict(terminal_pwm=8,measured_brake_handoff=8,road_heading_context=4)
for name,value in old['files'].items():
    if name.endswith(('.c','.h','.py')) and name not in ['User/App_ThiefStrategy.c','Tests/test_yaw_reference.py']:
        assert sha(root/name)==value,name
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
assert '0 Error(s), 0 Warning(s)' in (root/'build_monitor9.log').read_text(encoding='utf-8',errors='replace')
assert 'UARTCH4_16U_MONITOR9' in (root/'User/App_ThiefStrategy.c').read_text(encoding='utf-8')
changed=['User/App_ThiefStrategy.c','Tests/test_turn_coast.py','Tests/test_yaw_reference.py','Tests/test_documents14.py',
         'README.md','AGENTS.md','Docs/修改规范与回归清单.md']
patch=[]
for name in changed:
    previous=(base/name).read_text(encoding='utf-8') if (base/name).exists() else ''
    patch.extend(difflib.unified_diff(previous.splitlines(keepends=True),
        (root/name).read_text(encoding='utf-8').splitlines(keepends=True),
        fromfile='MONITOR8/'+name,tofile='MONITOR9/'+name))
(root/'TURN_COAST_CHANGESET.patch').write_text(''.join(patch),encoding='utf-8')
files=changed+['Tests/validate_monitor9.py','TURN_COAST_CHANGESET.patch','build_monitor9.log',
    'Tests/results_turn_coast.json','Tests/results_yaw_reference.json','Tests/results_turn_resume.json',
    'Tests/results_monitor_suite.json']+['Tests/results_'+stem+'.json' for stem in supplemental]
data=dict(firmware='UARTCH4_16U_MONITOR9',firmware_sha256=firmware,hex_sha256=sha(axf.with_suffix('.hex')),
    before_failures=before['failures'],new_cases=after['cases'],extra=extra,suite=suite,supplemental=supplemental,
    unchanged_algorithm_checks=pure['test_checks'],files={name:sha(root/name) for name in files},
    hardware_tested=False,flashed=False,serial_opened=False,
    limitations=['Actual terminal350PWM deceleration, traction, braking time and centering need physical verification.',
    'Synthetic feedback cannot prove actual overshoot has disappeared; only controller decisions/output are verified.',
    'A single measured coast brake is retained; no stationary +/-2degree fine alignment stage.',
    '12.17degree road recovery no longer fails CONTEXT, but genuine identity/hardware/localization faults remain.'])
(root/'VALIDATION_MONITOR9.json').write_text(json.dumps(data,ensure_ascii=False,indent=2),encoding='utf-8')
print('MONITOR9 final firmware,20 coast/recovery combinations,22 ARM scripts and supplemental evidence verified; hardware untested.')
