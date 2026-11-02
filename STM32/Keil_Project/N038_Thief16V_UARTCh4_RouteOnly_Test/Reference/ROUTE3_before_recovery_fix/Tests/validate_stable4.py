"""Bind the supported test reports, build and baseline comparison to this image."""
from pathlib import Path
import hashlib
import json
import re
import difflib

root = Path(__file__).resolve().parents[1]
def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()
def save(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')

output = root/'MDK-ARM'/root.name
axf = output/(root.name+'.axf')
image_hash = digest(axf)
log = (root/'build_stable4.log').read_text(encoding='utf-8', errors='replace')
assert '0 Error(s), 0 Warning(s)' in log
reports = {}
for suffix in ['stable1', 'escape_completion', 'turn_resume', 'left800', 'live_monitor', 'strategy_stable2', 'temp78_stable3', 'hide_routes_stable4']:
    data = json.loads((root/'Tests'/('results_'+suffix+'.json')).read_text(encoding='utf-8'))
    assert data.get('axf_sha256', data.get('firmware_sha256')) == image_hash, suffix+' stale image'
    assert not data.get('failures') and not data.get('failure'), suffix+' failed'
    reports[suffix] = data
assert reports['escape_completion']['app_source_sha256'] == digest(root/'User/App_ThiefStrategy.c')
assert reports['stable1']['cases']['unchanged_control_files'] == 16
assert reports['stable1']['cases']['motion_functions'] == 13

baseline = json.loads((root/'Reference/RUN8_sources.json').read_text(encoding='utf-8'))
desktop = Path('C:/Users/Administrator/Desktop/Thief16U_UARTCh4_ImmediateRetreat700_Test')
changes = []
patch = []
for rel, old_hash in baseline.items():
    current = root/rel
    assert digest(desktop/rel) == old_hash, 'original desktop changed: '+rel
    if digest(current) != old_hash:
        changes.append(rel)
        patch.extend(difflib.unified_diff((desktop/rel).read_text(encoding='utf-8-sig').splitlines(True),
                     current.read_text(encoding='utf-8-sig').splitlines(True),
                     fromfile='RUN8/'+rel, tofile='STABLE4/'+rel))
    elif rel.startswith('Control/'):
        assert digest(current) == old_hash
production = {str(p.relative_to(root)).replace('\\','/'):digest(p)
              for directory in ['User','Control','Algorithm','Hardware','System','Core']
              for p in (root/directory).rglob('*') if p.suffix.lower() in ['.c','.h','.s']}
added = sorted(set(production)-set(baseline))
for rel in added:
    patch.extend(difflib.unified_diff([], (root/rel).read_text(encoding='utf-8-sig').splitlines(True),
                                     fromfile='/dev/null', tofile='STABLE4/'+rel))
(root/'STABLE_BASE_CHANGESET.patch').write_text(''.join(patch), encoding='utf-8')
before = json.loads((root/'Reference/observer_before.json').read_text(encoding='utf-8'))
assert before['axf_sha256'] == digest(root/'Reference/RUN8.axf')
assert before['expected_rule_violation'] and before['reason'] == 'LOC_DISTANCE'

interfaces = ['Hardware/LaserRx.c', 'Hardware/LaserRx.h',
    'Algorithm/ThiefStrategy/FrontLaserGate.c', 'Algorithm/ThiefStrategy/FrontLaserGate.h',
    'Algorithm/ThiefStrategy/ThiefStrategyConfig.h',
    'Algorithm/LaserMux/AI1811Mux.c', 'Algorithm/LaserMux/AI1811Mux.h',
    'Algorithm/MapLocalization/MapLaserMask.c', 'Algorithm/MapLocalization/MapLaserMask.h']
monitor = root.parent/'Thief16U_UARTCh4_LiveMonitor_Test'
for rel in interfaces:
    assert digest(root/rel) == digest(monitor/rel), 'interface source differs: '+rel
save(root/'Reference/ported_interfaces.json', {rel:digest(root/rel) for rel in interfaces})

source = (root/'User/App_ThiefStrategy.c').read_text(encoding='utf-8')
for removed in ['Route_MapGrayRecover', 'Route_TurnExitRecover',
                'MAP_EXPECTED_NODE_MISSED', 'MAP_TURN_NOT_ALIGNED',
                'ROOT_TURN_POSE', 'INTERCEPT_REVERSE_POSE']:
    assert removed not in source, removed
assert 'OBSERVER_ONLY=1' in source and 'STATUS=%s' in source
assert 'UARTCH4_16V_STABLE4' in source
map_text = (output/(root.name+'.map')).read_text(errors='replace')
ram = int(re.search(r'Total RW\s+Size.*?\s(\d+) \(', map_text)[1])
rom = int(re.search(r'Total ROM Size.*?\s(\d+) \(', map_text)[1])
stack_text = (root/'MDK-ARM/startup_stm32f103xe.s').read_text()
stack = int(re.search(r'Stack_Size\s+EQU\s+(0x[0-9A-Fa-f]+)', stack_text)[1], 16)
callgraph = (output/(root.name+'.htm')).read_text(errors='replace')
known_stack = int(re.search(r'Maximum Stack Usage\s*=\s*(\d+)', callgraph)[1])
assert ram <= 48*1024 and rom <= 256*1024 and stack == 3072 and known_stack < stack
result = dict(firmware='UARTCH4_16V_STABLE4', build_errors=0, build_warnings=0,
    axf_sha256=image_hash, hex_sha256=digest(output/(root.name+'.hex')),
    memory=dict(rom_bytes=rom, ram_bytes=ram, ram_free_bytes=48*1024-ram,
                reserved_stack_bytes=stack, linker_known_stack_bytes=known_stack,
                stack_limit='Linker lists unknown calls; no physical stack high-water measurement'),
    baseline_sources_verified=True, modified_baseline_sources=changes, added_production_sources=added,
    project_configuration_changes=['MDK-ARM/startup_stm32f103xe.s: stack 3KB',
                                   'MDK-ARM/'+root.name+'.uvprojx: independent target/output'],
    source_sha256=production,
    tests=reports, hardware_tested=False, flashed=False,
    limitations=['Synthetic gray/yaw/encoder inputs do not prove physical stability or sensor timing',
                 'Coordinates are route estimates, not absolute relocalization after a missed junction',
                 'Real sensor faults, stall, user stop and continuous 2s both-board no-line can still stop the car'])

saved=root/'Reference/STABLE3_before_hide_routes'
regression=json.loads((saved/'results_hide_routes.json').read_text(encoding='utf-8'))
assert regression['expected_before_failure'] and regression['failures']
assert regression['axf_sha256']==digest(saved/'MDK-ARM'/root.name/(root.name+'.axf'))
assert regression['test_sha256']==reports['hide_routes_stable4']['test_sha256']
assert len(reports['hide_routes_stable4']['lifecycles'])==7
round_changes=[];round_patch=[]
for rel in ['User/App_ThiefStrategy.c', 'Algorithm/ThiefStrategy/ThiefMapPlanner.c',
            'Algorithm/ThiefStrategy/ThiefRiskSearch.c', 'Algorithm/ThiefStrategy/ThiefInterceptPolicy.c']:
    assert digest(saved/rel)!=digest(root/rel), 'missing intended change: '+rel
    round_changes.append(rel)
    round_patch.extend(difflib.unified_diff((saved/rel).read_text(encoding='utf-8-sig').splitlines(True),
                       (root/rel).read_text(encoding='utf-8-sig').splitlines(True),
                       fromfile='STABLE3/'+rel,tofile='STABLE4/'+rel))
(root/'STABLE4_CHANGESET.patch').write_text(''.join(round_patch),encoding='utf-8')
result['changed_this_round']=round_changes
result['before_regression']=dict(checks=regression['checks'],failed_expectations=len(regression['failures']),
                                axf_sha256=regression['axf_sha256'])

save(root/'VALIDATION_STABLE4.json', result)
print(json.dumps(dict(status='PASS', axf_sha256=image_hash, rom_bytes=rom, ram_bytes=ram,
                     suites=len(reports), modified_sources=changes), ensure_ascii=False, indent=2))
