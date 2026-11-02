"""Documents29: actual speed-control enable state, synthetic hardware only."""
from pathlib import Path
import sys,json,hashlib
src=Path(__file__).with_name('test_documents15.py').read_text(encoding='utf-8').split('parking_pwm=[]')[0]
src=src.replace("if '--before' in sys.argv:","if False:")
if '--before' in sys.argv:
    src=src.replace('exec(compile(runtime,',"runtime=runtime.replace(\"axf=ROOT/'MDK-ARM'/ROOT.name/(ROOT.name+'.axf')\",\"axf=ROOT/'Tests/before_documents29/original.axf'\")\nexec(compile(runtime,")
exec(compile(src,str(Path(__file__).with_name('test_documents15.py')),'exec'))
# Exercise actual linked reset/brake/enable and wheel-PI enable bookkeeping.
# Chassis hardware and speed feedback remain stubs; no motor-response claim.
for n in ['Route_ResetControl','Route_BrakeControl','ChassisSpeedControl_Brake','ChassisSpeedControl_Enable']:
    if n in hooks:u.hook_del(hooks.pop(n))
rear_raw=0x18
def trailing():
    record(u.reg_read(UC_ARM_REG_R0),'RouteGraySample_t',dict(rear_raw12=rear_raw,rear_error_x100=300))
    return rear_valid
hook('Route_ReadRear',trailing)
def reverse_entry(node=40,mode=1,progress=50):
    global clock,odom,gray_raw,gray_state,rear_valid,rear_raw
    ret('Route_AllWheelsStopped',1);rear_valid=1;rear_raw=0x18
    heading=3 if node==40 else 1;target=39 if node==40 else 40
    fresh(node,(heading+2*mode)%4);setstep(node,target,heading,mode,300)
    call('Route_Start');tick();odom=progress
    call('Route_BrakeControl');check(not var('speed_control_enabled'),'real interception brake disables speed control')
    var('route_state',12);var('route_state_tick',clock-100);var('turn_stop_stable_count',2);var('replan_after_leg',1)
    gray_raw=0;gray_state=2;clock+=20;targets.clear();call('Thief_InterceptStopped',clock)
    check(var('route_active') and var('map_gray_active'),'missing new leading board enters mapped recovery')
    check(var('speed_control_enabled')==1,'recovery commands must re-enable actual linked speed control after braking')
    check(targets and sum(targets[-1])>0,'real trailing line supports directed return output')
    check(var('map_progress_offset_mm')==300-progress,'retain actual short return distance')

def reverse_matrix():
    count=0
    for node in [39,40]:
      for mode in [0,1]:
        for mm in [1,24,27,50,90]:reverse_entry(node,mode,mm);count+=1
    return count
def road_recovery():
    global odom,gray_raw,gray_state,rear_valid,rear_raw
    ret('Route_AllWheelsStopped',1);fresh(36,3);rear_valid=1;rear_raw=0x18
    setstep(36,35,3,0,600);call('Route_Start');tick();odom=8;gray_raw=0;gray_state=2;tick()
def supported_over_one_second():
    global gray_raw,gray_state
    road_recovery()
    for _ in range(80):tick()
    check(var('route_active') and var('speed_control_enabled'),'rear018 prevents fixed1000ms lock while recovery is enabled')
    gray_raw=0x60;gray_state=1
    for _ in range(5):tick()
    check(var('route_state')==2,'actual leading line restores normal follow')
    return 1
def continuous_both_missing():
    global rear_valid,rear_raw,clock
    road_recovery();rear_valid=2;rear_raw=0;tick()
    start=clock;tick(1500)
    check(var('route_active'),'both missing1500ms must not latch the old1000ms fault')
    rear_valid=1;rear_raw=0x18;tick();check(var('route_active'),'a real rear line resets continuous-loss timer')
    rear_valid=2;rear_raw=0;tick();start=clock
    tick(1999);check(var('route_active'),'continuous both missing1999ms has not expired')
    clock=start+2020;call('App_ThiefStrategy_Task')
    check(not var('route_active') and reason()=='MAP_GRAY_REACQUIRE_FAILED','continuous both missing>=2000ms ends recovery at control sampling resolution')
    return 1
def after_turn_reverse():
    global yaw,odom,clock,gray_raw,gray_state,rear_valid,rear_raw
    ret('Route_AllWheelsStopped',1);fresh(46,2);rear_valid=1;rear_raw=0x18
    member('thief_localization','yaw_cw_sign',-1)
    setstep(46,40,0,1,300,action=-1);setstep(40,39,3,1,300,index=1);record(route,'MapRoute_t',dict(count=2));call('Route_Start')
    odom=300;clock+=600;call('Route_HandleJunction',clock,20,4,0xfe0)
    tick(180);tick(20);tick(80);yaw=6615;tick(440)
    yaw=9000
    for _ in range(12):tick()
    check(member('map_nav','step')==1,'real N046->N040 turn commits N040->N039')
    odom=50;call('Route_BrakeControl');var('route_state',12);var('route_state_tick',clock-100);var('turn_stop_stable_count',2);var('replan_after_leg',1)
    gray_raw=0;gray_state=2;clock+=20;call('Thief_InterceptStopped',clock)
    check(var('speed_control_enabled') and var('map_gray_active'),'50mm interception immediately after an actual turn can drive recovery')
    # Synthetic encoder/gray response to enabled targets; no fabricated node fix.
    odom=8;gray_raw=0x60;gray_state=1
    for _ in range(6):tick()
    result=complete_route()
    check(40 in result['fixes'] and result['node'] in [11,26,35,38,63],'return N040 and finish actual selected next refuge route')
    return result
failures=[];cases={}
for fn in [reverse_matrix,supported_over_one_second,continuous_both_missing,after_turn_reverse]:
    try:cases[fn.__name__]=fn();print(fn.__name__+': PASS',flush=True)
    except AssertionError as e:failures.append(dict(case=fn.__name__,error=str(e)));print(fn.__name__+': FAIL '+str(e),flush=True)
before='--before' in sys.argv
result=dict(checks=checks,failures=failures,cases=cases,hardware_tested=False,actual_speed_enable_bookkeeping=True,
 firmware_sha256=hashlib.sha256(axf.read_bytes()).hexdigest(),
 app_source_sha256=hashlib.sha256((ROOT/('Tests/before_documents29/User/App_ThiefStrategy.c' if before else 'User/App_ThiefStrategy.c')).read_bytes()).hexdigest(),
 test_script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
 helper_sha256={n:hashlib.sha256((ROOT/'Tests'/n).read_bytes()).hexdigest() for n in ['test_documents15.py','test_app_runtime.py','test_escape_completion.py','test_documents13.py']})
out=ROOT/'Tests'/('before_documents29/regression_before.json' if before else 'results_documents29.json')
out.write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8');print(json.dumps(result,indent=2));sys.exit(bool(failures))
