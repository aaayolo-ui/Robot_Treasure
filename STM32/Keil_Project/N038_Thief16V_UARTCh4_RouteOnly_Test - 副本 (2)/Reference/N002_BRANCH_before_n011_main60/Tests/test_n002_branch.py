from pathlib import Path
import sys,json,hashlib
ROOT=Path(__file__).absolute().parents[1]
setup=(ROOT/'Tests/test_arrival_gate.py').read_text(encoding='utf-8').split('try:')[0]
setup=setup.replace('Reference/MAIN60_before_arrival_gate/MDK-ARM','Reference/N063_MAIN60_before_n002_branch/MDK-ARM')
exec(compile(setup,str(ROOT/'Tests/test_arrival_gate.py'),'exec'))
try:
 for raw in [0xC40,0xF00]:
  begin();odom=180;front_raw=raw;commands.clear()
  for _ in range(8):tick()
  check(not member('map_nav','arrived') and var('junction_count')==0,'opposite-side evidence does not become N002')
  check(var('thief_current_node')==11,'no premature node projection')
  check(var('route_state')==2 and any(c[0]=='drive' for c in commands),'continues original line follow without entry brake')
  odom=250;front_raw=0x07F;tick()
  check(member('map_nav','arrived') and var('route_state')==3,'later real right exit starts original entry action')
  check(member('map_nav','seen_ports')&2,'right-branch evidence kept')
 begin();odom=180;front_raw=0;tick();front_raw=0xC40
 for _ in range(3):tick()
 check(not member('map_nav','arrived') and var('junction_count')==0,'recovery cannot bypass scoped branch requirement')
 odom=260;front_raw=0x07F;tick()
 check(member('map_nav','arrived'),'recovered genuine right branch still accepted')
 begin();odom=180;call('Route_HandleJunction',clock,20,4,0xC40)
 check(not member('map_nav','arrived') and var('junction_count')==0,'shared handler also rejects logged C40')
 begin();odom=1000;front_raw=0xC40;tick()
 check(not member('map_nav','arrived') and var('route_state')==2,'distance overshoot alone neither blesses C40 nor locks vehicle')
 # Same wrong-side raw on a different map edge retains existing behavior.
 fresh(21,0);setstep(21,38,2,0,600);call('Route_Start');odom=300
 check(call('Route_ArrivalPlausible',0xC40)==1,'other edges retain gray arrival acceptance')
except Exception as e:failures.append(str(e))
out=dict(checks=checks,failures=failures,axf_sha256=hashlib.sha256(axf.read_bytes()).hexdigest(),hardware_tested=False,before='--before' in sys.argv)
name='results_n002_branch_before.json' if out['before'] else 'results_n002_branch.json'
(ROOT/'Tests'/name).write_text(json.dumps(out,indent=2),encoding='utf-8')
print(json.dumps(out,indent=2));assert not failures,failures
