"""MONITOR8: bind control-resume proof and regression evidence to current AXF."""
from pathlib import Path
import difflib,hashlib,json
root=Path(__file__).resolve().parents[1];base=root/'Tests/before_turn_resume'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
read=lambda p:json.loads(p.read_text(encoding='utf-8'))
axf=root/'MDK-ARM'/root.name/(root.name+'.axf');firmware=sha(axf)
old=read(base/'VALIDATION_MONITOR7.json')
assert sha(base/'original.axf')==old['firmware_sha256']
before=read(base/'regression_before.json');after=read(root/'Tests/results_turn_resume.json')
assert before['axf_sha256']==old['firmware_sha256'] and len(before['failures'])==1
assert 'actually disabled speed loop' in before['failures'][0]['error']
assert before['test_sha256']==after['test_sha256']==sha(root/'Tests/test_turn_resume.py')
assert after['axf_sha256']==firmware and not after['failures']
assert after['cases']==dict(resume_after_brake=2,continuous_loop_kept=1)
for name,value in old['files'].items():
    if name.endswith(('.c','.h','.py')) and name not in ['User/App_ThiefStrategy.c','Tests/test_yaw_reference.py']:
        assert sha(root/name)==value,name
yaw=read(root/'Tests/results_yaw_reference.json')
assert yaw['axf_sha256']==firmware and not yaw['failures']
assert yaw['test_sha256']==sha(root/'Tests/test_yaw_reference.py')
assert yaw['cases']['treasure_turn_handoff']==8 and yaw['cases']['reference_wrap_reverse']==4
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
assert '0 Error(s), 0 Warning(s)' in (root/'build_monitor8.log').read_text(encoding='utf-8',errors='replace')
assert 'UARTCH4_16U_MONITOR8' in (root/'User/App_ThiefStrategy.c').read_text(encoding='utf-8')
changed=['User/App_ThiefStrategy.c','Tests/test_turn_resume.py','Tests/test_yaw_reference.py',
         'README.md','AGENTS.md','Docs/修改规范与回归清单.md']
patch=[]
for name in changed:
    previous=(base/name).read_text(encoding='utf-8') if (base/name).exists() else ''
    patch.extend(difflib.unified_diff(previous.splitlines(keepends=True),
        (root/name).read_text(encoding='utf-8').splitlines(keepends=True),
        fromfile='MONITOR7/'+name,tofile='MONITOR8/'+name))
(root/'TURN_RESUME_CHANGESET.patch').write_text(''.join(patch),encoding='utf-8')
files=changed+['Tests/validate_monitor8.py','TURN_RESUME_CHANGESET.patch','build_monitor8.log',
    'Tests/results_turn_resume.json','Tests/results_yaw_reference.json','Tests/results_monitor_suite.json']
files+=['Tests/results_'+stem+'.json' for stem in supplemental]
data=dict(firmware='UARTCH4_16U_MONITOR8',firmware_sha256=firmware,hex_sha256=sha(axf.with_suffix('.hex')),
    before_failures=before['failures'],new_cases=after['cases'],yaw_cases=yaw['cases'],suite=suite,
    supplemental=supplemental,unchanged_algorithm_checks=pure['test_checks'],
    files={name:sha(root/name) for name in files},hardware_tested=False,flashed=False,serial_opened=False,
    limitations=['Real speed-control enable/brake/PI/PWM calculations executed in emulation; physical feedback and PWM pins are mocked.',
    'No physical motion, traction or obstacle-escape validation. Treasure-exit stationary fine trim has been removed.',
    'Map yaw reference calibration and normal road/hide recoveries remain. No desktop/helper changes.'])
(root/'VALIDATION_MONITOR8.json').write_text(json.dumps(data,ensure_ascii=False,indent=2),encoding='utf-8')
print('MONITOR8 current firmware, real speed PI/PWM resume proof, yaw handoff and22 regression scripts verified; hardware untested.')
