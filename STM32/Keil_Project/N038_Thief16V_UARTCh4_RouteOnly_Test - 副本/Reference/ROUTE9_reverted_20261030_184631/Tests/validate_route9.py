"""Bind T-entry fix, immutable SL1 baseline and linked ARM offline evidence."""
from pathlib import Path
import json,re,hashlib,difflib
r=Path(__file__).absolute().parents[1];b=r/'Reference/ROUTE6_SL1_before_t_junction'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
m=json.loads((b/'SNAPSHOT_SHA256.json').read_text());assert len(m)==458 and all(sha(b/n)==h for n,h in m.items())
a=r/'MDK-ARM'/r.name/(r.name+'.axf');ah=sha(a)
i=json.loads((r/'Tests/route9_inputs.json').read_text())
assert i['axf_sha256']==ah
assert all(sha(r/n)==h for n,h in i['production'].items())
assert all(sha(r/n)==h for n,h in i['test_sources'].items())
allowed={'User/App_ThiefStrategy.c','Algorithm/MapRoute/MapRouteNavigator.c'}
assert {n for n in i['production'] if n not in m}==set()
assert {n for n,h in i['production'].items() if h!=m[n]}==allowed
new=(r/'User/App_ThiefStrategy.c').read_text(encoding='utf-8');old=(b/'User/App_ThiefStrategy.c').read_text(encoding='utf-8')
def fn(text,name):
    q=re.search(r'(?:static )?[^;\n]+\b'+name+r'\([^;]*?\)\s*\{',text);assert q,name
    j=text.index('{',q.start());k=j+1;depth=1
    while depth:depth+=(text[k]=='{')-(text[k]=='}');k+=1
    return text[q.start():k]
frozen=['Route_ReadFront','Route_ReadRear','Route_ClassifyPosition','Route_UpdateErrors','Route_UpdateFollowMode',
        'Route_DriveStraight','Route_DriveUnlocatedFollow','Route_DriveTurnApproach','Route_DriveLeftTurn',
        'Route_DriveRightTurn','Route_HasReachedTurnAngle','Route_IsTurnAngleFresh','Route_EnterFollow',
        'Route_EnterAfterTurnForward','Route_EnterTurnApproach','Route_RunFollow']
assert all(fn(new,n)==fn(old,n) for n in frozen),'normal motion function changed'
assert fn(new,'Route_EnterTurnStop').replace('  turn_t_valid_count = 0U;\n','')==fn(old,'Route_EnterTurnStop')
def spin(s):return s[s.index('    case ROUTE_STATE_TURN_RIGHT:'):s.index('    case ROUTE_STATE_AFTER_TURN_FORWARD:')]
assert spin(new)==spin(old),'ordinary main spin changed'
assert new[new.index('    case ROUTE_STATE_AFTER_TURN_FORWARD:'):]==old[old.index('    case ROUTE_STATE_AFTER_TURN_FORWARD:'):],'line acquisition/recovery/task tail changed'
n='Algorithm/MapRoute/MapRouteNavigator.c';nav=(r/n).read_text(encoding='utf-8');prior=(b/n).read_text(encoding='utf-8')
addition='  /* Keep this leg\'s near-junction observations across the arrival latch.\n   * Start/Advance and the application\'s new-leg/rearm reset own their scope. */\n'
assert nav.replace(addition,'  nav->seen_ports = 0U;\n')==prior
assert b'UARTCH4_16V_ROUTE9_T_EXIT_SL1' in a.read_bytes()
assert not (r/'Algorithm/ThiefStrategy/ThiefPursuit.c').exists()
suites=json.loads((r/'Tests/route9_suites.json').read_text());assert len(suites)==16 and all(x['exit_code']==0 for x in suites)
reports={}
for n in ['results_strategy_stable2.json','results_temp78_stable3.json','results_n011_first_plan_stable10.json',
          'results_hide_routes_stable4.json','results_escape_completion.json','results_turn_resume.json',
          'results_left800.json','results_live_monitor.json','west63_after.json','trace_front_after.json',
          'results_trace_return.json','results_trace_recovery.json','results_n068_recovery.json',
          'results_rear_retrace.json','results_short_return.json','results_t_junction.json']:
    p=r/'Tests'/n;z=json.loads(p.read_text());assert not z.get('failures'),n
    assert z.get('axf_sha256',z.get('firmware_sha256'))==ah,n
    reports[n]=dict(checks=z.get('checks'),sha256=sha(p))
pre=json.loads((r/'Tests/t_junction_before.json').read_text());z=json.loads((r/'Tests/results_t_junction.json').read_text())
assert {f['case'] for f in pre['failures']}=={'preserved_ports','centered_t'}
assert pre['axf_sha256']==sha(b/'MDK-ARM'/r.name/(r.name+'.axf'))
assert pre['test_sha256']==z['test_sha256']==sha(r/'Tests/test_t_junction.py')
assert len([c for c in z['cases'] if c['case']=='centered_t'])==8
assert len([c for c in z['cases'] if c['case']=='full_n007_corridor'])==2
mp=a.with_suffix('.map').read_text(encoding='utf-8',errors='replace')
rom=int(re.search(r'Total ROM Size[^\n]+?\)\s+(\d+)',mp)[1]);ram=int(re.search(r'Total RW\s+Size[^\n]+?\)\s+(\d+)',mp)[1])
stack=int(re.search(r'Maximum Stack Usage =\s*(\d+)',a.with_suffix('.htm').read_text(encoding='utf-8',errors='replace'))[1])
assert rom<256*1024 and ram==44888 and stack<3072
assert '0 Error(s), 0 Warning(s)' in (r/'build_route9.log').read_text(encoding='utf-8',errors='replace')
patch=''.join(''.join(difflib.unified_diff((b/n).read_text(encoding='utf-8').splitlines(True),(r/n).read_text(encoding='utf-8').splitlines(True),fromfile='ROUTE6_SL1/'+n,tofile='ROUTE9/'+n)) for n in sorted(allowed))
(r/'ROUTE9_CHANGESET.patch').write_text(patch,encoding='utf-8')
out=dict(firmware='UARTCH4_16V_ROUTE9_T_EXIT_SL1',source_hashes=i['production'],axf_sha256=ah,
         hex_sha256=sha(a.with_suffix('.hex')),baseline_axf_sha256=pre['axf_sha256'],changed_production=sorted(allowed),
         frozen_motion_functions=frozen,ordinary_main_spin_and_acquisition_unchanged=True,
         strategy_and_sensors_and_pa3_unchanged=True,suites=suites,reports=reports,
         rom_bytes=rom,ram_bytes=ram,ram_remaining_bytes=48*1024-ram,reserved_stack_bytes=3072,
         linker_known_stack_bytes=stack,hardware_stack_watermark_measured=False,
         hardware_tested=False,flashed=False,serial_opened=False,physical_motors_run=False)
(r/'VALIDATION_ROUTE9.json').write_text(json.dumps(out,indent=2),encoding='utf-8')
print(json.dumps(dict(firmware=out['firmware'],suites=len(suites),rom_bytes=rom,ram_bytes=ram,stack=stack,axf_sha256=ah),indent=2))
