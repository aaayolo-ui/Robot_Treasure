"""Audit COMPARE1 evidence and preserved main baseline; no hardware access."""
from pathlib import Path
import json,hashlib,re,xml.etree.ElementTree as ET
ROOT=Path(__file__).absolute().parents[1]
BASE=ROOT.parent/'Thief16U_UARTCh4_LiveMonitor_Test'
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
manifest=json.loads((ROOT/'Tests/BASE_MONITOR12_SHA256.json').read_text(encoding='utf-8'))
changed=[p for p,h in manifest.items() if sha(ROOT/p)!=h]
assert changed==['User/App_ThiefStrategy.c'],changed
assert all(sha(BASE/p)==h for p,h in manifest.items()),'Main source no longer matches captured baseline'
baseline_axf=sha(ROOT/'Tests/reference/MONITOR12.axf')
assert sha(BASE/'MDK-ARM'/BASE.name/(BASE.name+'.axf'))==baseline_axf
axf=ROOT/'MDK-ARM'/ROOT.name/(ROOT.name+'.axf');axf_hash=sha(axf)
log=(ROOT/'build_compare1.log').read_text(encoding='utf-8',errors='replace')
assert '0 Error(s), 0 Warning(s)' in log
paths=[e.text for e in ET.parse(ROOT/'MDK-ARM'/(ROOT.name+'.uvprojx')).findall('.//FilePath')]
assert len(paths)==len(set(paths))
assert all((ROOT/'MDK-ARM'/p.replace('\\','/')).is_file() for p in paths)
rows=[]
for n in ['police_compare','live_monitor','turn_resume','escape_completion','treasure_fix','left800','speed_compare']:
    p=ROOT/'Tests'/('results_'+n+'.json');r=json.loads(p.read_text(encoding='utf-8'))
    assert not r.get('failures') and not r.get('failure'),n
    assert r.get('axf_sha256',r.get('firmware_sha256'))==axf_hash,n
    rows.append(dict(test=n,checks=r.get('checks'),report_sha256=sha(p)))
turns=json.loads((ROOT/'Tests/results_police_compare.json').read_text(encoding='utf-8'))
assert turns['test_sha256']==sha(ROOT/'Tests/test_police_compare.py')
assert turns['cases']['whole_map_handoff']['combinations']==1016
assert turns['cases']['relative_threshold']==72
before=json.loads((ROOT/'Tests/results_compare_baseline.json').read_text(encoding='utf-8'))
assert len(before['failures'])==3 and before['axf_sha256']==baseline_axf
assert before['test_sha256']==turns['test_sha256']
pure=json.loads((ROOT/'Tests/results_16m.json').read_text(encoding='utf-8'))
assert pure['test_failures']==0 and pure['test_done']==1
assert all(sha(ROOT/p)==h for p,h in pure['source_sha256'].items())
files=['User/App_ThiefStrategy.c','README.md','AGENTS.md','Docs/修改规范与回归清单.md',
       'MDK-ARM/'+ROOT.name+'.uvprojx','MDK-ARM/'+ROOT.name+'.uvoptx',
       'Tests/test_police_compare.py','Tests/test_speed_compare.py','Tests/validate_compare.py',
       'POLICE_COMPARE_CHANGESET.patch','build_compare1.log']
result=dict(firmware='UARTCH4_16U_POLICE_COMPARE1',base='MONITOR12',
    base_axf_sha256=baseline_axf,axf_sha256=axf_hash,hex_sha256=sha(axf.with_suffix('.hex')),
    build_errors=0,build_warnings=0,main_source_and_axf_unchanged=True,
    changed_production_files=changed,project_file_references=len(paths),tests=rows,
    pure_algorithm_checks=pure['test_checks'],pure_algorithm_cases=pure['test_case'],
    baseline_expected_differences=3,hardware_tested=False,flashed=False,serial_opened=False,
    files={p:sha(ROOT/p) for p in files},
    inherited_reports_are_not_current=True,
    limitations=['Synthetic sensor inputs do not prove physical alignment, line acquisition or localization.',
    'Treasure gray mask remains MONITOR12; police narrower mask is not transplanted.',
    'Old map-axis handoff assertions describe MONITOR12, not the chosen comparison behavior.'])
(ROOT/'VALIDATION_COMPARE1.json').write_text(json.dumps(result,indent=2,ensure_ascii=False),encoding='utf-8')
print(json.dumps(result,indent=2,ensure_ascii=False))
