"""RUN19: short-return dual-board junction and four raw1000mm laser rays.
Actual linked Cortex-M3 firmware with explicit synthetic hardware inputs.
"""
from pathlib import Path
import sys,json,hashlib
src=Path(__file__).with_name('test_documents15.py').read_text(encoding='utf-8').split('parking_pwm=[]')[0]
src=src.replace("if '--before' in sys.argv:","if False:")
if '--before' in sys.argv:
 src=src.replace('exec(compile(runtime,',"runtime=runtime.replace(\"axf=ROOT/'MDK-ARM'/ROOT.name/(ROOT.name+'.axf')\",\"axf=ROOT/'Tests/before_documents123154/original.axf'\")\nexec(compile(runtime,")
exec(compile(src,str(Path(__file__).with_name('test_documents15.py')),'exec'))
for n in ['Route_ResetControl','Route_BrakeControl','ChassisSpeedControl_Brake','ChassisSpeedControl_Enable']:
 if n in hooks:u.hook_del(hooks.pop(n))
rear_raw=0x60
def trailing():
 record(u.reg_read(UC_ARM_REG_R0),'RouteGraySample_t',dict(rear_raw12=rear_raw,rear_error_x100=0));return rear_valid
hook('Route_ReadRear',trailing)
def mirror(raw):return int(f'{raw:012b}'[::-1],2)
def junction(node,h):
 raw=0x60
 if call('ThiefMapPlanner_HasRoad',node,(h+3)%4,0,0):raw|=0xc00
 if call('ThiefMapPlanner_HasRoad',node,(h+1)%4,0,0):raw|=3
 return raw
def entry(a,b,h,mode,length,progress):
 global clock,odom,gray_raw,gray_state,rear_valid,rear_raw
 fresh(a,(h+2*mode)%4);rear_valid=1;rear_raw=0x60;ret('Route_AllWheelsStopped',1)
 setstep(a,b,h,mode,length);call('Route_Start');tick();odom=progress
 call('Route_BrakeControl');var('route_state',12);var('route_state_tick',clock-100);var('turn_stop_stable_count',2);var('replan_after_leg',1)
 gray_raw=0x60;gray_state=1;clock+=20;call('Thief_InterceptStopped',clock)
 check(var('retreat_active') and var('map_progress_offset_mm')==length-progress,'actual measured reversal preserves signed return entry')
def trailing_return_matrix():
 global clock,odom,rear_raw
 cases=[]
 for a,b,h,length in [(2,3,1,600),(38,39,1,300),(39,40,1,300),(58,68,2,300)]:
  for mode in [0,1]:
   for progress in [1,24,27,44,90]:
    entry(a,b,h,mode,length,progress);fixes=member('thief_localization','confirmations')
    odom=progress+80;rear_raw=mirror(junction(a,(h+2)%4));tick()
    check(member('thief_localization','confirmations')==fixes+1,'real trailing junction must recover the short-return node before LOC_DISTANCE')
    check(member('map_nav','arrived') and var('thief_current_node')==a,'only actual dual-board gray confirms return, not encoder alone')
    check(not var('route_active') and not var('speed_control_enabled'),'confirmed return brakes actual speed control before replanning')
    if progress==44:
     rear_raw=0x60;result=complete_route()
     check(result['node'] in [11,26,35,38,63],'44mm return completes actual next refuge route')
     cases.append(dict(start=a,mode=mode,completion=result))
 return dict(combinations=40,completed=cases)
def return_requires_real_evidence():
 global clock,odom,rear_raw,yaw,rear_valid
 entry(2,3,1,1,600,44);fixes=member('thief_localization','confirmations');odom=44
 tick();check(member('thief_localization','confirmations')==fixes,'odometry at target without branch must not fabricate a fix')
 rear_raw=mirror(0x063);odom=124;tick()
 check(member('thief_localization','confirmations')==fixes,'wrong mapped branch on trailing board cannot confirm target')
 rear_valid=7;rear_raw=mirror(0xc60);tick()
 check(member('thief_localization','confirmations')==fixes,'I2C failure cannot provide trailing junction evidence')
 rear_valid=1;yaw=1500;tick()
 check(member('thief_localization','confirmations')==fixes,'outside10degree recovery cannot promote trailing branch')
 yaw=500;tick();check(member('thief_localization','confirmations')==fixes+1,'real correct branch and5degree heading confirm without changing odometry limits')
 return 1
def raw1000_four_channels():
 global clock
 for ch in range(4):
  check(call('FrontLaserGate_CenterDistanceMm',1000,ch)==(1150 if ch in [0,3] else 1100),'front/rear calibration equal; offsets do not shorten raw1000 trigger range')
  fresh(49,1);clock+=1
  limit=800 if ch in [1,2] else 1000
  for _ in range(8):clock+=20;call('Thief_OnLaserFrame',ch,limit,100,1,clock)
  check(var('thief_escape_requested') and get(symbols['thief_laser_seen']+ch,1),'per-channel raw boundary triggers: CH2/CH3=800, front/rear=1000')
  fresh(49,1);clock+=1
  for _ in range(8):clock+=20;call('Thief_OnLaserFrame',ch,limit+1,100,1,clock)
  check(not var('thief_escape_requested'),'one above the physical channel limit remains rejected')
 # N038 south is connected, but x=600,y=1500 pointing north crosses an empty
 # map gap. Detection must depend on the in-field hit, not connected road rays.
 fresh(49,0);member('thief_pose','x_mm',600);member('thief_pose','y_mm',1500);clock+=1
 for _ in range(3):clock+=20;call('Thief_OnLaserFrame',0,300,100,1,clock)
 check(var('thief_escape_requested'),'in-field target across a disconnected road ray must trigger')
 fresh(1,3);clock+=1
 for _ in range(8):clock+=20;call('Thief_OnLaserFrame',0,300,100,1,clock)
 check(not var('thief_escape_requested'),'clearly outward map-edge target remains ignored')
 return dict(channels=4,max_raw_mm=1000,front_rear_center_at_limit=1150)
def opposite_departure():
 routes=[]
 for mask in [1,2,4,8]:
  fresh(38,1);check(call('ThiefMapPlanner_PlanEscape',38,1,mask,0,clock,plan),'single-direction threat has an escape')
  h=1 if mask==1 else (mask.bit_length()-1+2)%4
  check(at(steps,'MapRouteStep_t','heading')==h,'N038 north uses east runway; other single threats retain opposite preference')
  routes.append(dict(mask=mask,first=at(steps,'MapRouteStep_t','to_node')))
 return routes
failures=[];cases={}
for fn in [trailing_return_matrix,return_requires_real_evidence,raw1000_four_channels,opposite_departure]:
 try:cases[fn.__name__]=fn();print(fn.__name__+': PASS',flush=True)
 except AssertionError as e:failures.append(dict(case=fn.__name__,error=str(e)));print(fn.__name__+': FAIL '+str(e),flush=True)
before='--before' in sys.argv
files=['User/App_ThiefStrategy.c','Algorithm/ThiefStrategy/FrontLaserGate.c','Algorithm/ThiefStrategy/ThiefStrategyConfig.h','Algorithm/ThiefStrategy/ThiefMapPlanner.c','Hardware/LaserRx.c','Control/TravelFrame.c','Control/TravelFrame.h']
base=ROOT/'Tests/before_documents123154' if before else ROOT
r=dict(checks=checks,failures=failures,cases=cases,hardware_tested=False,firmware_sha256=hashlib.sha256(axf.read_bytes()).hexdigest(),app_source_sha256=hashlib.sha256((base/'User/App_ThiefStrategy.c').read_bytes()).hexdigest(),source_sha256={f:hashlib.sha256((base/f).read_bytes()).hexdigest() for f in files},test_script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),helper_sha256={f:hashlib.sha256((ROOT/'Tests'/f).read_bytes()).hexdigest() for f in ['test_documents15.py','test_app_runtime.py','test_escape_completion.py','test_documents13.py']})
out=ROOT/'Tests'/('before_documents123154/regression_before.json' if before else 'results_documents123154.json');out.write_text(json.dumps(r,indent=2)+'\n',encoding='utf-8');print(json.dumps(r,indent=2));sys.exit(bool(failures))
