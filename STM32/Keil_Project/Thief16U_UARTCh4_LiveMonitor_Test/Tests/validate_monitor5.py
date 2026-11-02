"""Fresh MONITOR5 evidence; hardware and physical recovery remain unverified."""
from pathlib import Path
import json,hashlib,difflib
root=Path(__file__).resolve().parents[1];base=root/'Tests/before_exit_stall'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
read=lambda p:json.loads(p.read_text(encoding='utf-8'))
axf=root/'MDK-ARM'/root.name/(root.name+'.axf');firmware=sha(axf)
old=read(base/'VALIDATION_MONITOR4.json')
assert sha(base/'original.axf')==old['firmware_sha256']
before=read(base/'regression_before.json');after=read(root/'Tests/results_exit_stall.json')
assert before['axf_sha256']==sha(base/'original.axf') and len(before['failures'])==5
assert before['test_sha256']==after['test_sha256']==sha(root/'Tests/test_exit_stall.py')
assert after['axf_sha256']==firmware and not after['failures'] and after['checks']==988
assert len(after['cases'])==6
for name,value in old['files'].items():
    if name.endswith(('.c','.h','.py')) and name not in ('User/App_ThiefStrategy.c','Control/RestartLineSearch.c'):
        assert sha(root/name)==value,name
for stem,count in [('road_probe',1798),('left800',442)]:
    r=read(root/'Tests'/('results_'+stem+'.json'))
    assert r['axf_sha256']==firmware and r['checks']==count
    assert not r.get('failures') and r.get('failure') is None
suite=read(root/'Tests/results_monitor_suite.json')
assert len(suite)==22 and len({r['name'] for r in suite})==22
for row in suite:
    assert row['exit_code']==0 and row['axf_sha256']==firmware,row['name']
    assert row['log_sha256']==sha(root/'Tests'/('monitor_'+row['name']+'.log'))
pure=read(root/'Tests/results_16m.json');assert pure['test_failures']==0
for name,value in pure['source_sha256'].items():assert sha(root/name)==value,name
assert '0 Error(s), 0 Warning(s)' in (root/'build_monitor5.log').read_text(encoding='utf-8',errors='replace')
assert 'UARTCH4_16U_MONITOR5' in (root/'User/App_ThiefStrategy.c').read_text(encoding='utf-8')
changed=['User/App_ThiefStrategy.c','Control/RestartLineSearch.c','Tests/test_exit_stall.py',
 'README.md','AGENTS.md','Docs/修改规范与回归清单.md']
patch=[]
for name in changed:
    previous=(base/name).read_text(encoding='utf-8') if (base/name).exists() else ''
    patch.extend(difflib.unified_diff(previous.splitlines(keepends=True),(root/name).read_text(encoding='utf-8').splitlines(keepends=True),fromfile='MONITOR4/'+name,tofile='MONITOR5/'+name))
(root/'EXIT_STALL_CHANGESET.patch').write_text(''.join(patch),encoding='utf-8')
files=changed+['Tests/validate_monitor5.py','EXIT_STALL_CHANGESET.patch','build_monitor5.log',
 'Tests/results_exit_stall.json','Tests/results_road_probe.json','Tests/results_left800.json',
 'Tests/results_monitor_suite.json','Tests/results_16m.json']
data=dict(firmware='UARTCH4_16U_MONITOR5',firmware_sha256=firmware,hex_sha256=sha(axf.with_suffix('.hex')),
 limits_raw_mm=dict(front=1000,left=800,right=800,rear=1000),new_checks=after['checks'],road_probe_checks=1798,
 boundary_checks=442,pure_checks=pure['test_checks'],suite=suite,before_failures=before['failures'],
 files={name:sha(root/name) for name in files},hardware_tested=False,flashed=False,serial_opened=False,
 limitations=['Only simulated hardware feedback; torque, traction, actual gray coverage and physical recovery are unverified.',
 'Continuous BOTH-board loss still stops after2000ms; bounded probing cannot guarantee finding an arbitrary displaced line.'])
(root/'VALIDATION_MONITOR5.json').write_text(json.dumps(data,ensure_ascii=False,indent=2),encoding='utf-8')
print('MONITOR5 final AXF/HEX,%s new checks,1798 probe checks,442 limits,22 ARM scripts and pure tests verified;hardware untested.'%after['checks'])
