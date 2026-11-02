"""Validate only the freshly rerun MONITOR3 checks, not inherited old passes."""
from pathlib import Path
import hashlib,json,difflib
root=Path(__file__).resolve().parents[1];base=root/'Tests/before_left800'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
axf=root/'MDK-ARM'/root.name/(root.name+'.axf')
before=json.loads((base/'regression_before.json').read_text())
after=json.loads((root/'Tests/results_left800.json').read_text())
assert before['expected_before_failure'] and before['axf_sha256']==sha(base/'original.axf')
assert before['test_sha256']==after['test_sha256']==sha(root/'Tests/test_left800.py')
assert after['failure'] is None and after['checks']==442 and after['axf_sha256']==sha(axf)
app=(root/'User/App_ThiefStrategy.c').read_text(encoding='utf-8')
restored=app.replace('MONITOR3','MONITOR2').replace(' LEFT_RAW_MM=800','').replace('RIGHT_RAW_MM=800','RIGHT_RAW_MM=900')
assert restored==(base/'User/App_ThiefStrategy.c').read_text(encoding='utf-8'),'unrequested application behavior changed'
old=json.loads((base/'VALIDATION_MONITOR2.json').read_text(encoding='utf-8'))
for name,value in old['files'].items():
    if name.endswith(('.c','.h')) and not name.startswith('Tests/') and name!='User/App_ThiefStrategy.c':assert sha(root/name)==value,name
names=['test_right900','test_documents123154','test_treasure_fix','test_live_monitor']
suite=json.loads((root/'Tests/results_monitor_suite.json').read_text())
selected=[row for row in suite if row['name'] in names]
assert len(selected)==4 and all(row['exit_code']==0 for row in selected)
for row in selected:assert row['log_sha256']==sha(root/'Tests'/('monitor_'+row['name']+'.log'))
for name in ['right900','documents123154','treasure_fix','live_monitor']:
    report=json.loads((root/'Tests'/('results_'+name+'.json')).read_text())
    assert report.get('axf_sha256',report.get('firmware_sha256'))==sha(axf),name
pure=json.loads((root/'Tests/results_16m.json').read_text());assert pure['test_failures']==0
for name,value in pure['source_sha256'].items():assert sha(root/name)==value,name
assert '0 Error(s), 0 Warning(s)' in (root/'build_monitor3.log').read_text(encoding='utf-8',errors='replace')
changed=['Algorithm/ThiefStrategy/ThiefStrategyConfig.h','User/App_ThiefStrategy.c']
patch=[]
for name in changed:patch.extend(difflib.unified_diff((base/name).read_text(encoding='utf-8').splitlines(keepends=True),(root/name).read_text(encoding='utf-8').splitlines(keepends=True),fromfile='MONITOR2/'+name,tofile='MONITOR3/'+name))
(root/'LEFT800_CHANGESET.patch').write_text(''.join(patch),encoding='utf-8')
files=changed+['Tests/test_left800.py','Tests/test_right900.py','Tests/test_documents123154.py','Tests/test_16m.c','Tests/validate_monitor3.py','README.md','AGENTS.md','Docs/修改规范与回归清单.md','LEFT800_CHANGESET.patch','build_monitor3.log']
data=dict(firmware='UARTCH4_16U_MONITOR3',limits_raw_mm=dict(front=1000,left=800,right=800,rear=1000),firmware_sha256=sha(axf),hex_sha256=sha(axf.with_suffix('.hex')),new_checks=442,pure_checks=pure['test_checks'],freshly_rerun_scripts=selected,remaining_old_suite_not_rerun=18,files={name:sha(root/name) for name in files},hardware_tested=False,flashed=False,serial_opened=False)
(root/'VALIDATION_MONITOR3.json').write_text(json.dumps(data,ensure_ascii=False,indent=2),encoding='utf-8')
print('MONITOR3 both sides800 boundaries, unchanged application logic, current ARM tests and AXF/HEX verified.')
