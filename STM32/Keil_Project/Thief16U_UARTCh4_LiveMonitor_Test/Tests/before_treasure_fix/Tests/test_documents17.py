"""Documents17 actual ARM routines; synthetic hardware, no serial/motors."""
from pathlib import Path
import sys,json,hashlib
src=Path(__file__).with_name('test_documents15.py').read_text(encoding='utf-8').split('def wrap(a):')[0]
src=src.replace("if '--before' in sys.argv:","if False:")
if '--before' in sys.argv:
    src=src.replace('exec(compile(runtime,',"runtime=runtime.replace(\"axf=ROOT/'MDK-ARM'/ROOT.name/(ROOT.name+'.axf')\",\"axf=ROOT/'Tests/before_documents17/original.axf'\")\nexec(compile(runtime,")
exec(compile(src,str(Path(__file__).with_name('test_documents15.py')),'exec'))
fixture=Path(__file__).with_name('test_documents16.py').read_text(encoding='utf-8')
fixture=fixture[fixture.index('rx_enabled=1;rx_calls=[]'):fixture.index('def plateau():')]
exec(compile(fixture,str(Path(__file__).with_name('test_documents16.py')),'exec'))
pwm=[0]*4;pwm_calls=[];enable_calls=[]
def setpwm():
    wheel=u.reg_read(UC_ARM_REG_R0);value=u.reg_read(UC_ARM_REG_R1)
    value=value-2**32 if value>=2**31 else value
    pwm[wheel]=value;pwm_calls.append((clock,wheel,value));return 0
hook('Chassis_SetWheelLogicalPwm',setpwm)
def brake():
    pwm[:]=[0]*4;events.append('Route_BrakeControl');return 0
hook('Route_BrakeControl',brake);hook('Chassis_Brake',brake)
def resetcontrol():
    enable_calls.append(u.reg_read(UC_ARM_REG_R0));return 0
hook('Route_ResetControl',resetcontrol)

def parking_pulses():
    global yaw,clock
    count=0
    for node,axes in [(11,[1,3]),(38,[1,3]),(26,[0,2]),(35,[1,3]),(63,[0,2])]:
      for body in axes:
       for mode in [0,1]:
        for sign in [-1,1]:
            parking(node,body,mode,360)
            member('thief_localization','yaw_cw_sign',sign)
            pwm_calls.clear();targets.clear();pwm[:]=[0]*4
            # Real gyro cadence was about100ms, not a fresh frame each20ms.
            hook('JY61P_GetLastUpdateMs',lambda:(clock//100)*100)
            for _ in range(8):
                tick()
                if any(pwm):break
            check(any(pwm),'parking uses a four-wheel breakaway pulse, not tiny independent PI targets')
            check(pwm==[550,-550,550,-550] if sign<0 else pwm==[-550,550,-550,550],
                  'all four receive equal existing turn PWM in physical CW/CCW direction')
            check(not targets,'parking pulse does not run four low-RPM PI loops')
            tick()
            check(pwm==[0]*4,'pulse ends with all-wheel brake by next20ms control tick')
            calls=len(pwm_calls)
            for _ in range(3):tick()
            check(len(pwm_calls)==calls,'do not re-drive during80ms settle wait')
            yaw=100
            for _ in range(30):tick()
            check(var('thief_strategy_state')==2 and pwm==[0]*4,'fresh aligned yaw leaves genuinely quiet hiding')
            # Actual stable hiding never re-applies any nonzero command.
            calls=len(pwm_calls)
            for _ in range(10):tick()
            check(len(pwm_calls)==calls,'no residual wheel drive after HIDE_READY')
            hook('JY61P_GetLastUpdateMs',lambda:clock)
            count+=1
    return count

def turn_visible_resume():
    global yaw,clock,gray_raw,gray_state
    count=0
    for body in range(4):
      for mode in [0,1]:
       for sign in [-1,1]:
        for quarter in [-1,1]:
            fresh(11,body);member('thief_localization','yaw_cw_sign',sign)
            h=(body+quarter+2*mode)%4
            check(call('ThiefMapPlanner_HasRoad',11,h,0x2000c400,0x2000c402)==1,'test root exit exists')
            target,length=get(0x2000c400,2),get(0x2000c402,2)
            setstep(11,target,h,mode,length);call('Route_Start')
            tick(20);tick(60);yaw=sign*quarter*8064;tick(340);tick(20)
            gray_raw=0;gray_state=2
            call('Route_EnterFrontRecovery',clock,2)
            gray_raw=0x20;gray_state=1;targets.clear();events.clear();enable_calls.clear()
            tick()
            check(var('route_state')==2,'real leading line and outgoing yaw within10deg resume without three stopped gyro frames')
            check('Route_BrakeControl' not in events,'valid exit is not re-braked')
            check(targets and sum(targets[-1][:2])//2>=2000,'resume16T turn-exit speed immediately')
            check(1 in enable_calls,'resume explicitly re-enables PI after entry brake')
            count+=1
    return count

def visible_road_speed():
    global odom,gray_raw,gray_state,rear_valid,yaw
    count=0
    for mode in [0,1]:
      for sign in [-1,1]:
        start(38,1,37 if mode else 39,3 if mode else 1,mode)
        member('thief_localization','yaw_cw_sign',sign)
        odom=150;gray_raw=0;gray_state=2;rear_valid=1;targets.clear();tick()
        check(var('map_gray_active')==1,'corroborated map recovery entered')
        check(sum(targets[-1][:2])//2<=500,'missing front still uses bounded recovery speed')
        gray_raw=0x10;gray_state=1;targets.clear();events.clear()
        tick()
        check(targets and sum(targets[-1][:2])//2>=2000,'first real restored front sample uses normal following, no extra50RPM dip')
        check(var('map_gray_active')==1,'retain three-frame validity confirmation rather than fake success')
        tick();tick()
        check(var('route_state')==2 and not var('map_gray_active'),'third real frame completes recovery')
        check('Route_BrakeControl' not in events,'restored road does not stop for redundant confirmation')
        count+=1
    return count

def pulse_boundaries():
    control=allocate('RoadHeadingControl_t',0x2000c800)
    inp=allocate('RoadHeadingInput_t',0x2000c840);out=allocate('RoadHeadingOutput_t',0x2000c870)
    def update(now,stamp,error=360,stopped=1,direction=1,turn=False):
        record(inp,'RoadHeadingInput_t',dict(cw_error_x100=error,yaw_tick=stamp,stopped=stopped,direction_known=direction))
        call('RoadHeading_UpdateTurn' if turn else 'RoadHeading_UpdatePulse',control,inp,now,1000 if turn else 200,out)
        return {k:at(out,'RoadHeadingOutput_t',k) for k in ['brake','ready','failed','turn_x10']}
    call('RoadHeading_Reset',control,1000)
    check(update(1020,1000)['brake'],'pre-stop gyro frame cannot initiate another pulse')
    check(update(1080,1080)['brake'],'need two measured standstill samples')
    check(not update(1100,1100)['brake'],'fresh observed yaw and stop release one pulse')
    check(not update(1119,1100)['brake'],'pulse remains bounded below20ms')
    check(update(1120,1100)['brake'],'20ms ends pulse even without another gyro frame')
    for now in [1200,1220,1260]:
        check(update(now,1100)['brake'],'old gyro cannot cause repeated unsensed pulses')
    check(update(1300,1300)['brake'],'new pulse still waits two stopped samples')
    check(not update(1320,1300)['brake'],'next pulse uses a post-brake gyro observation')
    check(update(1330,1330,-360,0)['brake'],'sign reversal brakes immediately rather than counter-driving wheels')
    check(update(1350,1350,-360)['brake'],'sign reversal retains80ms measured brake gap')
    check(not update(1410,1410,-360)['brake'],'opposite pulse starts only after stop and fresh yaw')
    # The turn controller may finish with one fresh stopped frame, never while moving.
    call('RoadHeading_Reset',control,2000)
    check(not update(2020,2020,500,turn=True)['ready'],'turn settling not skipped at20ms')
    check(update(2100,2100,500,turn=True)['ready'],'one fresh settled frame permits road-exit handoff')
    check(not update(2120,2120,500,stopped=0,turn=True)['ready'],'moving wheels invalidate stopped confirmation')
    call('RoadHeading_Reset',control,2200)
    update(2220,2220,500,turn=True);update(2300,2300,500,turn=True)
    state=update(2320,2320,2000,direction=0,turn=True)
    check(state['failed'] and not state['ready'],'out-of-range unknown direction can never reuse old ready evidence')

failures=[];passed=[];counts={}
for fn in [parking_pulses,turn_visible_resume,visible_road_speed]+([] if '--before' in sys.argv else [pulse_boundaries]):
    try:counts[fn.__name__]=fn();passed.append(fn.__name__);print(fn.__name__+': PASS',flush=True)
    except AssertionError as e:failures.append(dict(case=fn.__name__,error=str(e)));print(fn.__name__+': FAIL '+str(e),flush=True)
before='--before' in sys.argv
result=dict(checks=checks,failures=failures,passed=passed,cases=counts,hardware_tested=False,
 firmware_sha256=hashlib.sha256(axf.read_bytes()).hexdigest(),
 app_source_sha256=hashlib.sha256((ROOT/('Tests/before_documents17/User/App_ThiefStrategy.c' if before else 'User/App_ThiefStrategy.c')).read_bytes()).hexdigest(),
 test_script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
 helper_sha256={n:hashlib.sha256((ROOT/'Tests'/n).read_bytes()).hexdigest() for n in ['test_documents15.py','test_documents16.py','test_app_runtime.py','test_escape_completion.py','test_documents13.py']})
out=ROOT/'Tests'/('before_documents17/regression_before.json' if before else 'results_documents17.json')
out.write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8');print(json.dumps(result,indent=2));sys.exit(bool(failures))
