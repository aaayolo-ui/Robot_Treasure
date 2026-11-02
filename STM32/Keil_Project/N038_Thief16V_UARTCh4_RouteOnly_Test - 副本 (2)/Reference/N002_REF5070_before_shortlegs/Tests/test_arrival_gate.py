from pathlib import Path
import json,hashlib,sys
ROOT=Path(__file__).absolute().parents[1]
setup=(ROOT/'Tests/test_n068_recovery.py').read_text(encoding='utf-8').split('failures=[]')[0]
if '--before' in sys.argv:
 setup=setup.replace('exec(compile(runtime,', "runtime=runtime.replace(\"axf=ROOT/'MDK-ARM'\", \"axf=ROOT/'Reference/MAIN60_before_arrival_gate/MDK-ARM'\")\nexec(compile(runtime,")
exec(compile(setup,str(ROOT/'Tests/test_n068_recovery.py'),'exec'))
failures=[]
def begin():
 global front_raw,back_raw,physical_rpm
 fresh(11,0);setstep(11,2,0,0,300,1);setstep(2,3,1,0,600,2,1)
 record(route,'MapRoute_t',dict(id=1601,count=2,start_node=11,heading=0,drive_mode=0))
 front_raw=back_raw=0x60;physical_rpm=0;cache(0);commands.clear();call('Route_Start')
 for _ in range(8):tick()
 check(var('route_state')==2 and var('junction_armed'),'centered line arms departure normally')
try:
 begin();odom=80;front_raw=0xF00;tick()
 check(not member('map_nav','arrived') and var('thief_current_node')==11,'logged early F00 must not become N002 in normal follow')
 check(var('route_state')==2 and any(x[0]=='drive' for x in commands),'early edge keeps actual line-follow control running')
 begin();odom=80;front_raw=0;tick()
 check(var('route_state')==8,'temporary leading-line loss enters real recovery path')
 front_raw=0xF00;tick()
 check(not member('map_nav','arrived') and var('thief_current_node')==11,'recovery cannot bypass early-node gate with F00')
 begin();odom=80;call('Route_HandleJunction',clock,20,4,0xF00)
 check(not member('map_nav','arrived') and var('junction_count')==0,'shared handler rejects early candidate without mutating node or count')
 for distance,expected in [(119,0),(120,0),(149,0),(150,1),(180,1),(210,1),(211,1),(1000,1)]:
  odom=distance;check(call('Route_ArrivalPlausible',0xFFF)==expected,f'300mm edge arrival gate at {distance}mm')
 begin();odom=300;front_raw=0x60;tick()
 check(not member('map_nav','arrived'),'distance alone never confirms a node')
 front_raw=0x07F;tick()
 check(member('map_nav','arrived') and var('route_state')==3,'real later right branch starts N002 entry normally')
 check(member('map_nav','seen_ports') & 2,'real right branch evidence retained for turn confirmation')
 begin();odom=80;front_raw=0;tick();odom=180;front_raw=0x07F;tick()
 check(member('map_nav','arrived') and var('route_state')==3,'recovery still accepts a later plausible real branch')
except Exception as e:failures.append(str(e))
out=dict(checks=checks,failures=failures,axf_sha256=hashlib.sha256(axf.read_bytes()).hexdigest(),hardware_tested=False,before='--before' in sys.argv)
name='results_arrival_gate_before.json' if out['before'] else 'results_arrival_gate.json'
(ROOT/'Tests'/name).write_text(json.dumps(out,indent=2),encoding='utf-8')
print(json.dumps(out,indent=2));assert not failures,failures
