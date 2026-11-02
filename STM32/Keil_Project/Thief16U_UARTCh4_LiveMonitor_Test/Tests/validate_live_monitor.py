"""Bind this separate monitor build, its changes and offline results."""
from pathlib import Path
import hashlib,json,difflib,xml.etree.ElementTree as ET
root=Path(__file__).resolve().parents[1]
baseline=Path('C:/Users/Administrator/Desktop/jiezhezuo/Robot_Treasure-main/Robot_Treasure-main/STM32/Keil_Project/Thief16U_UARTCh4_ImmediateRetreat700_Test')
def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()
files=['Hardware/LaserRx.c','Hardware/LaserRx.h','User/App_ThiefStrategy.c']
patch=[]
for name in files:
    old=(baseline/name).read_text(encoding='utf-8').splitlines(keepends=True)
    new=(root/name).read_text(encoding='utf-8').splitlines(keepends=True)
    patch.extend(difflib.unified_diff(old,new,fromfile='RUN21/'+name,tofile='MONITOR1/'+name))
(root/'LIVE_MONITOR_CHANGESET.patch').write_text(''.join(patch),encoding='utf-8')
suite=json.loads((root/'Tests/results_monitor_suite.json').read_text())
assert len(suite)==21 and all(item['exit_code']==0 for item in suite)
for item in suite:assert digest(root/'Tests'/('monitor_'+item['name']+'.log'))==item['log_sha256']
before=json.loads((root/'Tests/results_live_monitor_before.json').read_text())
after=json.loads((root/'Tests/results_live_monitor.json').read_text())
axf=root/'MDK-ARM'/root.name/(root.name+'.axf')
hexfile=axf.with_suffix('.hex')
assert before['expected_failure'] and digest(root/'Tests/before_live_monitor/original.axf')==before['axf_sha256']
assert after['checks']==21 and after['failures']==0 and digest(axf)==after['axf_sha256']
assert digest(baseline/'MDK-ARM'/baseline.name/(baseline.name+'.axf'))==before['axf_sha256']
unchanged=[]
for name in ['User/MapTimingLog.c','User/MapTimingLog.h','Algorithm/MapLocalization/ThiefLocalization.c','Algorithm/MapRoute/MapRouteNavigator.c','Algorithm/ThiefStrategy/ThiefMapPlanner.c','System/UartDebug.c']:
    assert digest(root/name)==digest(baseline/name),name
    unchanged.append(name)
for path in root.rglob('*.c'):
    if any(p in path.parts for p in ('Tests','MDK-ARM')):continue
    name=path.relative_to(root).as_posix()
    if (baseline/name).exists() and name not in files:assert digest(path)==digest(baseline/name),name
project=ET.parse(root/'MDK-ARM'/(root.name+'.uvprojx')).getroot()
source_paths=[x.text for x in project.findall('.//FilePath')]
assert len(source_paths)==len(set(source_paths))
build=(root/'build_live_monitor.log').read_text(encoding='utf-8',errors='replace')
assert '0 Error(s), 0 Warning(s)' in build and hexfile.exists()
tracked=files+['MDK-ARM/'+root.name+'.uvprojx','MDK-ARM/'+root.name+'.uvoptx','AGENTS.md','README.md','Docs/修改规范与回归清单.md','Tests/test_live_monitor.py','Tests/run_monitor_suite.py','Tests/validate_live_monitor.py','Tests/test_speed16t.py','Tests/reference_16T/User/App_ThiefStrategy.c','LIVE_MONITOR_CHANGESET.patch','build_live_monitor.log']
report=dict(firmware='UARTCH4_16U_MONITOR1',baseline=str(baseline),baseline_axf_sha256=before['axf_sha256'],baseline_preserved=True,armcc_errors=0,armcc_warnings=0,firmware_sha256=digest(axf),hex_sha256=digest(hexfile),new_checks=after['checks'],suite=suite,unchanged_control_and_map_modules=unchanged,files={name:digest(root/name) for name in tracked},hardware_tested=False,flashed=False,serial_opened=False,known_unresolved=['RUN21 early/source junction ambiguity','RUN21 reverse line-follow and turn-exit failures'],position_source='MCU map events plus road-constrained display prediction; not independent absolute localization')
(root/'VALIDATION_MONITOR.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
print('Final source, original baseline, AXF/HEX and 21 regression scripts verified.')
