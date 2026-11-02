"""Bind MONITOR2 source, unchanged baseline, real ARM tests and build artifacts."""
from pathlib import Path
import json,hashlib,difflib
root=Path(__file__).resolve().parents[1]
baseline=root/'Tests/before_treasure_fix'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
production=['User/App_ThiefStrategy.c','Algorithm/MapRoute/TreasureGrayMask.c','Algorithm/MapRoute/TreasureGrayMask.h','Algorithm/MapLocalization/ThiefLocalization.c','Algorithm/ThiefStrategy/ThiefMapPlanner.c','Algorithm/ThiefStrategy/ThiefRiskSearch.c','Algorithm/ThiefStrategy/ThiefRiskSearch.h']
patch=[]
for name in production:
    patch.extend(difflib.unified_diff((baseline/name).read_text(encoding='utf-8').splitlines(keepends=True),(root/name).read_text(encoding='utf-8').splitlines(keepends=True),fromfile='MONITOR1/'+name,tofile='MONITOR2/'+name))
(root/'TREASURE_ROUTE_CHANGESET.patch').write_text(''.join(patch),encoding='utf-8')
for folder in ['User','Hardware','Algorithm','System']:
    for old in (baseline/folder).rglob('*'):
        if old.suffix not in ('.c','.h'):continue
        name=old.relative_to(baseline).as_posix()
        if name not in production:assert sha(old)==sha(root/name),name
before=json.loads((baseline/'regression_before.json').read_text())
after=json.loads((root/'Tests/results_treasure_fix.json').read_text())
axf=root/'MDK-ARM'/root.name/(root.name+'.axf');hexfile=axf.with_suffix('.hex')
assert len(before['failures'])==6 and sha(baseline/'original.axf')==before['axf_sha256']
assert before['test_sha256']==after['test_sha256']==sha(root/'Tests/test_treasure_fix.py')
assert not after['failures'] and after['axf_sha256']==sha(axf)
suite=json.loads((root/'Tests/results_monitor_suite.json').read_text())
assert len(suite)==22 and len({item['name'] for item in suite})==22
assert all(item['exit_code']==0 for item in suite)
for item in suite:assert item['log_sha256']==sha(root/'Tests'/('monitor_'+item['name']+'.log'))
pure=json.loads((root/'Tests/results_16m.json').read_text())
assert pure['test_checks']==366181 and pure['test_failures']==0
for name,value in pure['source_sha256'].items():assert sha(root/name)==value,name
build=(root/'build_monitor2.log').read_text(encoding='utf-8',errors='replace')
assert '0 Error(s), 0 Warning(s)' in build and hexfile.exists()
assert 'UARTCH4_16U_MONITOR2' in (root/'User/App_ThiefStrategy.c').read_text(encoding='utf-8')
live=json.loads((root/'Tests/results_live_monitor.json').read_text())
assert live['axf_sha256']==sha(axf) and live['failures']==0
tracked=production+['README.md','AGENTS.md','Docs/修改规范与回归清单.md','Tests/test_treasure_fix.py','Tests/test_documents12.py','Tests/test_documents22.py','Tests/test_documents123154.py','Tests/run_monitor_suite.py','Tests/validate_monitor2.py','TREASURE_ROUTE_CHANGESET.patch','build_monitor2.log']
data=dict(firmware='UARTCH4_16U_MONITOR2',baseline_axf_sha256=before['axf_sha256'],firmware_sha256=sha(axf),hex_sha256=sha(hexfile),checks=after['checks'],cases=after['cases'],pure_checks=pure['test_checks'],regression_scripts=suite,changed_production=production,files={name:sha(root/name) for name in tracked},hardware_tested=False,flashed=False,serial_opened=False,limits=['actual text footprint and sensor mounting remain unmeasured','no independent absolute positioning','not a guarantee against every physical line-loss fault'])
(root/'VALIDATION_MONITOR2.json').write_text(json.dumps(data,ensure_ascii=False,indent=2),encoding='utf-8')
print('MONITOR2 final AXF/HEX, source diff, six baseline failures, pure algorithms and22 scripts verified.')
