"""Bind MONITOR13 source, firmware, targeted regressions and memory evidence."""
from pathlib import Path
import json,re,hashlib,difflib
P=Path(__file__).resolve().parents[1]
def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
image=P/'MDK-ARM'/P.name/(P.name+'.axf');image_sha=sha(image)
hexfile=image.with_suffix('.hex');mapfile=image.with_suffix('.map');callgraph=image.with_suffix('.htm')
build=(P/'build_monitor13.log').read_text(encoding='utf-8',errors='replace')
assert '0 Error(s), 0 Warning(s)' in build and 'Target not created' not in build
maptext=mapfile.read_text(encoding='utf-8',errors='replace')
ram=int(re.search(r'Total RW\s+Size.*?\s(\d+)\s+\(',maptext)[1])
rom=int(re.search(r'Total ROM Size.*?\s(\d+)\s+\(',maptext)[1])
assert ram<48*1024 and rom<256*1024
stack=int(re.search(r'Stack_Size\s+EQU\s+(0x[0-9A-Fa-f]+)',(P/'MDK-ARM/startup_stm32f103xe.s').read_text())[1],16)
depth=int(re.search(r'Maximum Stack Usage =\s+(\d+)',callgraph.read_text(encoding='utf-8',errors='replace'))[1])
assert stack>=depth+512
baseline=P/'Tests/before_topology_recovery'
assert sha(baseline/'original.axf')=='b58e22c80def8c074e22ca5451e963fa6af7f96d8fad2ce5d7415da02cf68750'
before=json.loads((baseline/'regression_before.json').read_text(encoding='utf-8'))
assert before['expected_old_rule_violation'] and before['axf_sha256']==sha(baseline/'original.axf')
suite=json.loads((P/'Tests/results_monitor13_suite.json').read_text(encoding='utf-8'))
assert len(suite)==9 and all(r['exit_code']==0 and r['axf_sha256']==image_sha for r in suite)
for r in suite:
    assert sha(P/'Tests'/(r['name']+'.py'))==r['test_sha256']
    assert sha(P/'Tests'/('monitor13_'+r['name']+'.log'))==r['log_sha256']
pure=json.loads((P/'Tests/results_16m.json').read_text(encoding='utf-8'))
assert pure['test_done'] and pure['test_failures']==0
for name,digest in pure['source_sha256'].items():assert sha(P/name)==digest,name
topo=json.loads((P/'Tests/results_topology_linefirst.json').read_text(encoding='utf-8'))
turns=json.loads((P/'Tests/results_police_handoff13.json').read_text(encoding='utf-8'))
assert topo['axf_sha256']==image_sha and topo['cases']['application_reanchor']
assert turns['axf_sha256']==image_sha and not turns['failures']
assert turns['cases']['whole_map_handoff']['combinations']==1016
assert 'white_at_coarse_handoff' in turns['cases']
files=['User/App_ThiefStrategy.c','Algorithm/MapLocalization/ThiefLocalization.c',
       'Algorithm/MapLocalization/ThiefLocalization.h','Algorithm/ThiefStrategy/ThiefMapPlanner.c',
       'Algorithm/ThiefStrategy/ThiefRiskSearch.c','Control/RestartLineSearch.c','Control/RestartLineSearch.h',
       'MDK-ARM/startup_stm32f103xe.s','Docs/修改规范与回归清单.md','README.md','AGENTS.md']
assert 'UARTCH4_16U_MONITOR13' in (P/files[0]).read_text(encoding='utf-8')
assert '入口只校验一次' not in (P/'Docs/修改规范与回归清单.md').read_text(encoding='utf-8')
# Normal gains and smoothing are byte-identical to the untouched comparison copy.
for f in ['Control/LineFollowPID.c','Control/LineFollowPID.h','Control/FollowSmoothing.c','Control/FollowSmoothing.h',
          'Control/ChassisSpeedControl.c','Control/ChassisSpeedControl.h']:
    assert sha(P/f)==sha(P.parent/'Thief16U_PoliceTurnCompare_Test'/f),f
patch=[]
for name in files:
    source=baseline/name
    if name=='MDK-ARM/startup_stm32f103xe.s':source=baseline/'startup_stm32f103xe.s'
    assert source.exists(),name
    patch.extend(difflib.unified_diff(source.read_text(encoding='utf-8-sig').splitlines(True),
        (P/name).read_text(encoding='utf-8-sig').splitlines(True),fromfile='MONITOR12/'+name,tofile='MONITOR13/'+name))
(P/'TOPOLOGY_LINE_FIRST_CHANGESET.patch').write_text(''.join(patch),encoding='utf-8')
result=dict(firmware='UARTCH4_16U_MONITOR13',firmware_sha256=image_sha,hex_sha256=sha(hexfile),
    target=dict(device='STM32F103RC',flash_capacity_bytes=262144,ram_capacity_bytes=49152,
        flash_used_bytes=rom,ram_used_bytes=ram,ram_free_bytes=49152-ram,stack_bytes=stack,
        known_callgraph_stack_bytes=depth,candidate_bytes=topo['candidate_bytes'],
        caveat='Unknown library/interrupt depth, actual stack high-water and worst-case CPU time not measured on hardware'),
    new_checks=topo['checks'],new_cases=topo['cases'],turn_checks=turns['checks'],turn_cases=turns['cases'],
    pure_algorithm_checks=pure['test_checks'],pure_algorithm_cases=pure['test_case'],suite=suite,
    baseline_axf_sha256=sha(baseline/'original.axf'),baseline_control_source='Untouched COMPARE1 copy of MONITOR12 Control files',
    source_sha256={name:sha(P/name) for name in files},build_log_sha256=sha(P/'build_monitor13.log'),
    differences_sha256=sha(P/'TOPOLOGY_LINE_FIRST_CHANGESET.patch'),
    hardware_tested=False,flashed=False,motors_run=False,serial_opened=False,
    historical_tests='Old mandatory map-axis fine trim and blind-probe assertions remain historical, not claimed passing MONITOR13',
    limitations=['At most8 hypotheses and3-junction local seed; repeated topology can remain ambiguous',
        'Sensor-only line following cannot guarantee recovery from arbitrary physical displacement',
        'Two-side vs four-side treasure text masking has not been compared on the real car'])
(P/'VALIDATION_MONITOR13.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps(dict(firmware=result['firmware'],axf=image_sha,flash=rom,ram=ram,free=49152-ram,
    stack=stack,topology_checks=topo['checks'],map_turns=1016,targeted_suites=9,hardware_tested=False),ensure_ascii=False))
