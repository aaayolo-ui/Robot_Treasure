"""Bind MONITOR4 build, unchanged baselines and fresh offline evidence."""
from pathlib import Path
import json,hashlib,difflib
root=Path(__file__).resolve().parents[1];base=root/'Tests/before_road_probe'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
read=lambda p:json.loads(p.read_text(encoding='utf-8'))
axf=root/'MDK-ARM'/root.name/(root.name+'.axf');firmware=sha(axf)
before=read(base/'regression_before.json');after=read(root/'Tests/results_road_probe.json')
assert before['axf_sha256']==sha(base/'original.axf')==read(base/'VALIDATION_MONITOR3.json')['firmware_sha256']
assert len(before['failures'])==3 and not after['failures']
assert before['test_sha256']==after['test_sha256']==sha(root/'Tests/test_road_probe.py')
assert after['axf_sha256']==firmware and after['checks']==1798
assert len(after['cases'])==4
boundary=read(root/'Tests/results_left800.json')
assert boundary['failure'] is None and boundary['checks']==442 and boundary['axf_sha256']==firmware
suite=read(root/'Tests/results_monitor_suite.json')
assert len(suite)==22 and len({r['name'] for r in suite})==22
for row in suite:
    assert row['exit_code']==0 and row['axf_sha256']==firmware,row['name']
    assert row['log_sha256']==sha(root/'Tests'/('monitor_'+row['name']+'.log'))
pure=read(root/'Tests/results_16m.json');assert pure['test_failures']==0
for name,value in pure['source_sha256'].items():assert sha(root/name)==value,name
for name,value in read(root/'VALIDATION_MONITOR2.json')['files'].items():
    if name.endswith(('.c','.h')) and not name.startswith('Tests/') and name!='User/App_ThiefStrategy.c':
        assert sha(root/name)==value,name
assert sha(root/'Algorithm/ThiefStrategy/ThiefStrategyConfig.h')==read(base/'VALIDATION_MONITOR3.json')['files']['Algorithm/ThiefStrategy/ThiefStrategyConfig.h']
build=root/'build_monitor4.log'
assert '0 Error(s), 0 Warning(s)' in build.read_text(encoding='utf-8',errors='replace')
assert 'UARTCH4_16U_MONITOR4' in (root/'User/App_ThiefStrategy.c').read_text(encoding='utf-8')
changed=['User/App_ThiefStrategy.c','Control/RestartLineSearch.c','Control/RestartLineSearch.h',
 'Tests/test_documents13.py','Tests/test_documents20.py','Tests/test_documents22.py',
 'Tests/test_documents24.py','Tests/test_documents31.py','Tests/run_monitor_suite.py',
 'Tests/test_road_probe.py','README.md','AGENTS.md','Docs/修改规范与回归清单.md']
patch=[]
for name in changed:
    original=(base/name).read_text(encoding='utf-8') if (base/name).exists() else ''
    patch.extend(difflib.unified_diff(original.splitlines(keepends=True),(root/name).read_text(encoding='utf-8').splitlines(keepends=True),fromfile='MONITOR3/'+name,tofile='MONITOR4/'+name))
(root/'ROAD_PROBE_CHANGESET.patch').write_text(''.join(patch),encoding='utf-8')
files=changed+['Tests/validate_monitor4.py','ROAD_PROBE_CHANGESET.patch','build_monitor4.log',
 'Tests/results_road_probe.json','Tests/results_left800.json','Tests/results_monitor_suite.json','Tests/results_16m.json']
result=dict(firmware='UARTCH4_16U_MONITOR4',firmware_sha256=firmware,hex_sha256=sha(axf.with_suffix('.hex')),
 limits_raw_mm=dict(front=1000,left=800,right=800,rear=1000),new_checks=after['checks'],boundary_checks=442,
 pure_checks=pure['test_checks'],suite=suite,before_failures=before['failures'],
 files={name:sha(root/name) for name in files},hardware_tested=False,flashed=False,serial_opened=False,
 limitations=['Local recovery on known topology; similar junctions are not uniquely identifiable from gray alone.',
 'Synthetic gray/yaw/encoder feedback validates software state and outputs, not physical traction or successful line acquisition.'])
(root/'VALIDATION_MONITOR4.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
print('MONITOR4: final AXF/HEX,1798 new checks,442 limits,22 ARM scripts and pure algorithm verified;hardware untested.')
