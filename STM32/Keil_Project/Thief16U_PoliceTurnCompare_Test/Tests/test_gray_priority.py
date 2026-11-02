"""Gray node evidence takes precedence over auxiliary encoder distance."""
from pathlib import Path
import hashlib,json,sys
src=Path(__file__).with_name('test_documents15.py').read_text(encoding='utf-8').split('parking_pwm=[]')[0]
src=src.replace("if '--before' in sys.argv:","if False:")
if '--before' in sys.argv:
 src=src.replace('exec(compile(runtime,',"runtime=runtime.replace(\"axf=ROOT/'MDK-ARM'/ROOT.name/(ROOT.name+'.axf')\",\"axf=ROOT/'Tests/before_gray_priority/original.axf'\")\nexec(compile(runtime,")
exec(compile(src,str(Path(__file__).with_name('test_documents15.py')),'exec'))
trail_raw=0x60
def trailing():
 record(u.reg_read(UC_ARM_REG_R0),'RouteGraySample_t',dict(rear_raw12=trail_raw,rear_error_x100=0));return rear_valid
hook('Route_ReadRear',trailing)
def gray_at(node,h):
 raw=0x60
 if call('ThiefMapPlanner_HasRoad',node,(h+3)%4,0,0):raw|=0xc00
 if call('ThiefMapPlanner_HasRoad',node,(h+1)%4,0,0):raw|=3
 return raw
def begin(a,b,h,mode,length):
 global gray_raw,gray_state,rear_valid,trail_raw,odom
 fresh(a,(h+2*mode)%4);rear_valid=1;trail_raw=0x60;gray_raw=0x60;gray_state=1
 setstep(a,b,h,mode,length)
 if (a,b)==(4,6):record(steps,'MapRouteStep_t',dict(segment_count=2))
 call('Route_Start')
 for _ in range(5):tick()
 check(var('junction_armed') and var('front_was_valid'),'actual outgoing narrow-line samples rearm node detection')
def observed_node_matrix():
 global odom,gray_raw,gray_state
 total=0
 for a,b,h,length in [(4,6,1,300),(38,39,1,300),(2,3,1,600),(58,68,2,300)]:
  for mode in [0,1]:
   for distance in [0,1,49,120,601,1101,-100]:
    begin(a,b,h,mode,length);fixes=member('thief_localization','confirmations');odom=distance
    tick();check(var('route_active') and member('thief_localization','confirmations')==fixes,'auxiliary distance alone neither stops nor promotes a node')
    gray_raw=gray_at(b,h);gray_state=4 if gray_raw&0xc00 else 5;tick()
    check(member('thief_localization','confirmations')==fixes+1 and member('thief_localization','confirmed_node')==b,'real mapped junction accepted despite early/late/signed odometry')
    total+=1
 return total
def wrong_branch_and_origin():
 global odom,gray_raw,gray_state
 begin(68,58,0,0,300);odom=900;fixes=member('thief_localization','confirmations');count=var('junction_count')
 gray_raw=0xc60;gray_state=4;tick()
 check(var('route_active') and member('thief_localization','confirmations')==fixes and var('junction_count')==count,'wrong real branch cannot consume next node when encoder is late')
 gray_raw=0x63;gray_state=5;tick()
 check(member('thief_localization','confirmed_node')==58,'correct branch subsequently confirms at the same late odometry')
 fresh(4,3);setstep(4,6,1,1,300);record(steps,'MapRouteStep_t',dict(segment_count=2));gray_raw=0xfff;gray_state=3;call('Route_Start');fixes=member('thief_localization','confirmations')
 for _ in range(4):tick()
 check(member('thief_localization','confirmations')==fixes,'unarmed source all-black must not fabricate the next node')
 return 1
def reverse_signed_ledger():
 global odom,clock,gray_raw,gray_state
 total=0
 for mode in [0,1]:
  for progress in [1,70,299,485,650,1100]:
   begin(4,3,3,mode,300);odom=progress;fixes=member('thief_localization','confirmations')
   var('route_state',12);var('route_state_tick',clock-100);var('turn_stop_stable_count',2);var('replan_after_leg',1)
   clock+=20;call('Thief_InterceptStopped',clock)
   check(var('route_active') and var('retreat_active'),'encoder range alone cannot veto a topologically valid measured-stop reversal')
   check(var('map_progress_offset_mm')==(300-progress)%2**32 and member('thief_localization','reversal_entry_mm')==(300-progress)%2**32,'retain full signed length-minus-measured-progress ledger')
   check(member('thief_localization','confirmations')==fixes and member('thief_localization','confirmed_node')==4,'reversal arithmetic never fabricates a new confirmed node')
   total+=1
 return total
def ambiguous_far_node_sequence():
 global clock,odom,gray_raw,gray_state
 begin(4,6,1,1,300);odom=485
 var('route_state',12);var('route_state_tick',clock-100);var('turn_stop_stable_count',2);var('replan_after_leg',1)
 clock+=20;call('Thief_InterceptStopped',clock)
 fixes=member('thief_localization','confirmations')
 # N006 and N004 both expose a south branch. Repeated frames of N006
 # through/after the encoder zero point must never be counted as N004.
 gray_raw=0xfc0;gray_state=4
 for offset in [185,190,200,225]:
  odom=offset;tick()
  check(not member('map_nav','arrived') and member('thief_localization','confirmations')==fixes,'same unconfirmed far T cannot become the original node on its repeated frames')
 gray_raw=0x60;gray_state=1
 for _ in range(5):tick()
 odom=1085;gray_raw=gray_at(4,3);gray_state=4;tick()
 check(not member('map_nav','arrived') and member('thief_localization','confirmations')==fixes,'MONITOR6 same-shape late return stays ambiguous until further observations')
 check(var('route_active') and var('route_identity_phase')==1,'ambiguity is an observation state, not odometry fault')
 return 1
def no_line_still_stops():
 global odom,gray_raw,gray_state,rear_valid,trail_raw,clock
 begin(38,39,1,0,300);odom=900;tick();check(var('route_active'),'line-supported overdue leg continues')
 gray_raw=0;gray_state=2;trail_raw=0;rear_valid=2
 started=clock
 for _ in range(120):
  clock+=20;call('App_ThiefStrategy_Task')
  if not var('route_active'):break
 check(not var('route_active') and var('route_state')==10,'continuous both-board no-line still terminates; no blind indefinite drive')
 check(2000<=clock-started<=2300,'only the continuous two-board no-line deadline ends this test')
 return 1
def n004_to_n006_complete():
 global odom,gray_raw,gray_state,rear_valid,trail_raw
 fresh(4,3);rear_valid=1;trail_raw=0x60;gray_raw=0x60;gray_state=1
 call('ThiefIntercept_BlockLeg',4,3,clock) #Preserve the explicit N004-N006 replay under the new global planner.
 check(call('ThiefMapPlanner_PlanEscape',4,3,4,0,clock,plan),'actual N004 south-threat escape plan exists')
 check(at(steps,'MapRouteStep_t','from_node')==4 and at(steps,'MapRouteStep_t','to_node')==6,'log160117 first escape leg is really N004 toN006')
 call('Route_Start')
 for _ in range(5):tick()
 odom=900;tick();check(var('route_active'),'log route can seek real N006 after encoder auxiliary window')
 gray_raw=gray_at(6,1);gray_state=5;tick()
 check(member('thief_localization','confirmed_node')==6,'actual N006 branch commits the true node despite odometry mismatch')
 result=complete_route();check(result['node'] in [11,26,35,38,63],'after N006 actual turns and route reach a legal refuge')
 return result
failures=[];cases={}
for fn in [observed_node_matrix,wrong_branch_and_origin,reverse_signed_ledger,ambiguous_far_node_sequence,no_line_still_stops,n004_to_n006_complete]:
 try:cases[fn.__name__]=fn();print(fn.__name__+': PASS',flush=True)
 except AssertionError as e:failures.append(dict(case=fn.__name__,error=str(e)));print(fn.__name__+': FAIL '+str(e),flush=True)
sha=lambda f:hashlib.sha256(f.read_bytes()).hexdigest();before='--before' in sys.argv;base=ROOT/'Tests/before_gray_priority' if before else ROOT
r=dict(checks=checks,failures=failures,cases=cases,hardware_tested=False,firmware_sha256=sha(axf),test_sha256=sha(Path(__file__)),source_sha256={n:sha(base/n) for n in ['User/App_ThiefStrategy.c','Algorithm/MapLocalization/ThiefLocalization.c','Algorithm/MapRoute/MapRouteNavigator.c']},helper_sha256={n:sha(ROOT/'Tests'/n) for n in ['test_documents15.py','test_app_runtime.py','test_escape_completion.py','test_documents13.py']})
(ROOT/'Tests'/('before_gray_priority/regression_before.json' if before else 'results_gray_priority.json')).write_text(json.dumps(r,indent=2),encoding='utf-8');print(json.dumps(r,indent=2));sys.exit(bool(failures))
