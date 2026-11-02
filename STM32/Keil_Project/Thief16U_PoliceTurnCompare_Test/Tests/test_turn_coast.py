"""030845: terminal turn PWM/brake handoff and >10degree road recovery."""
from pathlib import Path
import sys,json,hashlib
src=Path(__file__).with_name('test_documents15.py').read_text(encoding='utf-8').split('parking_pwm=[]')[0]
src=src.replace('Tests/before_documents15/original.axf','Tests/before_turn_coast/original.axf')
exec(compile(src,str(Path(__file__).with_name('test_documents15.py')),'exec'))
pwm=[]
def capture():
    v=u.reg_read(UC_ARM_REG_R1);pwm.append(v-2**32 if v>=2**31 else v);return 0
hook('Chassis_SetWheelLogicalPwm',capture)
for n in ['Route_DriveLeftTurn','Route_DriveRightTurn']:
    if n in hooks:u.hook_del(hooks.pop(n))

def pending(node=6,mode=0,sign=-1,quarter=-1):
    global odom,clock,yaw,gray_raw,gray_state,rear_valid
    source=(15 if node==6 else 14) if quarter<0 else (4 if node==6 else 3)
    target=(4 if node==6 else 3) if quarter<0 else (15 if node==6 else 14)
    incoming=0 if quarter<0 else 1;outgoing=3 if quarter<0 else 2
    fresh(source,(incoming+2*mode)%4);member('thief_localization','yaw_cw_sign',sign)
    setstep(source,node,incoming,mode,300,action=quarter);setstep(node,target,outgoing,mode,300,index=1)
    if node==6:record(steps+(16 if quarter<0 else 0),'MapRouteStep_t',dict(segment_count=2))
    record(route,'MapRoute_t',dict(count=2));call('Route_Start');odom=300;clock+=500
    call('Route_HandleJunction',clock,20,4 if quarter<0 else 5,0xc60 if quarter<0 else 0x63)
    var('turn_start_angle_x100',18000 if mode else 0)
    gray_raw=0x60;gray_state=1;rear_valid=1

def terminal_pwm():
    global yaw
    for mode in [0,1]:
      for sign in [-1,1]:
       for quarter in [-1,1]:
        pending(mode=mode,sign=sign,quarter=quarter);yaw=quarter*sign*6000;pwm.clear()
        call('Route_DriveLeftTurn' if quarter<0 else 'Route_DriveRightTurn',20)
        check(len(pwm)==4 and all(abs(v)==550 for v in pwm),'large angle retains normal turn PWM')
        yaw=quarter*sign*7500;pwm.clear();call('Route_DriveLeftTurn' if quarter<0 else 'Route_DriveRightTurn',20)
        check(len(pwm)==4 and all(0<abs(v)<550 for v in pwm),'terminal angle reduces actual four-wheel PWM')
    return 8

def measured_brake_handoff():
    global clock,yaw
    for node in [4,6]:
      for mode in [0,1]:
       for sign in [-1,1]:
        pending(node,mode,sign);yaw=-sign*8058;events.clear()
        call('Route_EnterAfterTurnForward',clock,20,0x2000ce00,yaw)
        check('Route_DriveStraight' not in events,'80.58degree sample cannot translate with residual wheel rotation')
        ret('Route_AllWheelsStopped',0);events.clear()
        for _ in range(5):tick()
        check('Route_DriveStraight' not in events and var('route_state')==7,'measured rotating wheels keep the one-shot brake')
        ret('Route_AllWheelsStopped',1);yaw=-sign*8800
        frozen=var('turn_coast_frame');hook('JY61P_GetFrameCount',lambda:frozen)
        for _ in range(3):tick()
        check(var('route_state')==7,'measured stop still needs a post-brake gyro frame')
        hook('JY61P_GetFrameCount',lambda:clock)
        for _ in range(8):tick()
        check(var('route_state')!=7,'one brake releases promptly without stationary angle fine trim')
        check(var('route_active'),'handoff remains runnable')
    return 8

def road_heading_context():
    global yaw,clock,gray_raw,gray_state,rear_valid,odom
    for mode in [0,1]:
      for sign in [-1,1]:
        fresh(4,(3+2*mode)%4);member('thief_localization','yaw_cw_sign',sign)
        setstep(4,3,3,mode,300);call('Route_Start');tick()
        yaw=sign*1217;member('thief_localization','heading_error_x100',1217)
        gray_raw=0;gray_state=2;rear_valid=2;odom=142
        var('map_gray_active',1);var('route_state',8);var('front_resume_state',2)
        var('front_state',2);var('map_gray_start_mm',46);var('map_gray_tick',clock-210)
        var('map_gray_missing_active',1);var('map_gray_missing_tick',clock-210)
        sample=allocate('RouteGraySample_t',0x2000cd00)
        call('Route_MapGrayRecover',clock,20,sample)
        check(var('route_active') and var('thief_strategy_state')!=4,'12.17degree yaw cannot become CONTEXT fault after210ms')
        check('turn_exit_active' in symbols and var('turn_exit_active'),'large yaw uses measured heading recovery')
        check(var('map_gray_missing_tick')==clock-210,'recovery transfer preserves the continuous white timer')
        # A continuous two-board white interval still expires, not renewed by transfer.
        var('map_gray_missing_active',1);var('map_gray_missing_tick',clock-2000)
        clock+=20;call('Route_TurnExitRecover',clock,20,sample)
        check(var('route_state')==10,'continuous both-board loss still stops after2seconds')
    return 4

failures=[];cases={}
for fn in [terminal_pwm,measured_brake_handoff,road_heading_context]:
    try:cases[fn.__name__]=fn()
    except Exception as e:failures.append(dict(case=fn.__name__,error=str(e)))
axf=ROOT/'MDK-ARM'/ROOT.name/(ROOT.name+'.axf')
if '--before' in sys.argv:axf=ROOT/'Tests/before_turn_coast/original.axf'
r=dict(cases=cases,failures=failures,axf_sha256=hashlib.sha256(axf.read_bytes()).hexdigest(),
       test_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),hardware_tested=False)
out=ROOT/('Tests/before_turn_coast/regression_before.json' if '--before' in sys.argv else 'Tests/results_turn_coast.json')
out.write_text(json.dumps(r,indent=2),encoding='utf-8');print(json.dumps(r,indent=2));sys.exit(bool(failures))
