"""Actual linked firmware regressions for parking posture and pending turns.
Synthetic hardware inputs: no physical motor or center-position validation.
"""
from pathlib import Path
import sys,json
runtime=Path(__file__).with_name('test_app_runtime.py').read_text(encoding='utf-8').split('def documents8_cases():')[0]
if '--before' in sys.argv:
    runtime=runtime.replace("axf=ROOT/'MDK-ARM'/ROOT.name/(ROOT.name+'.axf')", "axf=ROOT/'Tests/before_documents15/original.axf'")
exec(compile(runtime,str(Path(__file__).with_name('test_app_runtime.py')),'exec'))
fixture=Path(__file__).with_name('test_escape_completion.py').read_text(encoding='utf-8-sig').split('# Log regression:')[0]
exec(compile(fixture[fixture.index('for n in ['):],str(Path(__file__).with_name('test_escape_completion.py')),'exec'))
fixture=Path(__file__).with_name('test_documents13.py').read_text(encoding='utf-8').split('failures=[];passed=[]')[0]
exec(compile(fixture[fixture.index('rear_valid=1;rear_error=0;targets=[]'):],str(Path(__file__).with_name('test_documents13.py')),'exec'))

parking_pwm=[]
def capture_parking_pwm():
    value=u.reg_read(UC_ARM_REG_R1);value=value-2**32 if value>=2**31 else value
    parking_pwm.append((u.reg_read(UC_ARM_REG_R0),value));return 0
hook('Chassis_SetWheelLogicalPwm',capture_parking_pwm)

def wrap(a):return (a+18000)%36000-18000
def signed16(a):return a-65536 if a>=32768 else a
def road(node,h):
    check(call('ThiefMapPlanner_HasRoad',node,h,0x2000c400,0x2000c402)==1,'test uses actual map road')
    return get(0x2000c400,2),get(0x2000c402,2)

def park(node,body,mode,sign,error):
    global yaw,odom,clock,gray_raw,gray_state
    travel=(body+2*mode)%4;source,length=road(node,(travel+2)%4)
    fresh(source,body);member('thief_localization','yaw_cw_sign',sign)
    setstep(source,node,travel,mode,length);call('Route_Start')
    yaw=error;odom=length;clock+=400;targets.clear()
    raw=0x60
    for h,mask in [((travel+3)%4,0xc00),((travel+1)%4,3)]:
        if call('ThiefMapPlanner_HasRoad',node,h,0,0):raw|=mask
    call('Route_HandleJunction',clock,20,3,raw)
    check(var('route_state')==13,'fixed refuge enters original180ms approach')
    check(targets and sum(targets[-1][:2])==2000,'approach mean remains100RPM')
    if error:
        correction=targets[-1][0]-targets[-1][1]
        check(correction*error*sign<0,'approach steers against observed heading error')
    tick(179);check(var('route_state')==13,'do not shorten user180ms approach')
    tick(1)
    check(var('node_centered')==0 and var('thief_strategy_state')==5,
          'timed approach alone cannot certify aligned parking')
    check(member('thief_pose','valid')==1,'confirmed refuge pose remains available during heading trim')
    ret('Route_AllWheelsStopped',0);targets.clear()
    for _ in range(2):tick()
    check(not targets,'must measure stopped wheels before parking rotation')
    ret('Route_AllWheelsStopped',1)
    parking_pwm.clear()
    for _ in range(4):tick()
    if error:
        check(len(parking_pwm)>=4 and [w for w,v in parking_pwm[-4:]]==[0,1,2,3] and sum(v for w,v in parking_pwm[-4:])==0 and all(abs(v)==550 for w,v in parking_pwm[-4:]),'parking uses all four equal-PWM opposite-side pulse commands')
        check(var('thief_strategy_state')==5,'skewed vehicle is not yet hiding')
    # These are synthetic observed gyro corrections, not invented node fixes.
    yaw=100 if error>=0 else -100
    call('Thief_UpdatePose')
    for _ in range(10):tick()
    check(var('thief_strategy_state')==2 and var('node_centered')==1,'fresh aligned yaw + standstill allow hiding')
    check(member('thief_localization','confirmed_node')==node,'parking never changes the fixed node')
    check(abs(signed16(member('thief_localization','heading_error_x100')))<=200,'parking acceptance within2degrees')

def parking_cases():
    park(38,1,0,-1,1392)
    count=1
    for node,axes in [(11,[1,3]),(38,[1,3]),(26,[0,2]),(35,[1,3]),(63,[0,2])]:
      for body in axes:
       for mode in [0,1]:
        for sign in [-1,1]:
            park(node,body,mode,sign,800 if mode else -800);count+=1
    return count

def pending_root(body,mode,sign,quarter):
    global yaw,odom,clock,gray_raw,gray_state,rear_valid
    physical_sign=sign or -1
    fresh(11,body);member('thief_localization','yaw_cw_sign',sign)
    heading=(body+quarter+2*mode)%4;target,length=road(11,heading)
    setstep(11,target,heading,mode,length);call('Route_Start')
    tick(20);tick(60)
    yaw=wrap(physical_sign*quarter*5166);tick(340)
    if 'turn_axis_active' in symbols:
        check(member('thief_localization','phase')==2,'half turn cannot commit a departure')
        yaw=wrap(physical_sign*quarter*9000)
        for _ in range(30):
            tick()
            if var('route_state')==7 and not var('turn_axis_active'):break
        check(var('route_state')==7 and not var('turn_axis_active'),'real shared axis stage completes before missing-exit replay')
    else:check(var('route_state')==7,'original45degree spin handoff preserved')
    tick(20);gray_raw=0;gray_state=2;rear_valid=1
    for _ in range(15):tick()
    check('turn_exit_active' in symbols and var('turn_exit_active'),
          'pending quarter-turn bypasses old200ms no-line lock')
    # Respond to actual commanded in-place rotation. No map/pose state is set.
    for _ in range(110):
        if var('turn_exit_active') and member('turn_exit_heading','spinning'):
            turn=signed16(member('turn_exit_heading','last_turn'))
            yaw=wrap(yaw+physical_sign*round(turn/5))
        if var('turn_exit_active') and var('turn_exit_line_phase'):
            gray_raw=0x60;gray_state=1
        tick()
        if member('thief_localization','phase')==1:break
    check(member('thief_localization','phase')==1 and not var('departure_turn'),'actual exit line confirms the first leg')
    check(member('thief_localization','body_heading')==(body+quarter)%4,'confirmed outgoing body orientation')
    check(call('TravelFrame_IsReverse')==mode,'outgoing reverse frame retained')

def reverse_n078():
    global yaw,odom,clock,gray_raw,gray_state,rear_valid
    fresh(63,0);member('thief_localization','yaw_cw_sign',-1)
    setstep(63,78,2,1,600,action=1);setstep(78,77,3,1,300,index=1)
    record(route,'MapRoute_t',dict(count=2));call('Route_Start')
    odom=600;clock+=600;call('Route_HandleJunction',clock,20,5,0x83f)
    tick(180);tick(20);tick(80)
    yaw=-5166;tick(340);tick(20)
    if 'turn_axis_active' in symbols:
        yaw=-9000
        for _ in range(30):
            tick()
            if var('route_state')==7 and not var('turn_axis_active'):break
        check(var('route_state')==7 and not var('turn_axis_active'),'N078 completes shared axis before exit recovery replay')
    gray_raw=0;gray_state=2;rear_valid=1
    for _ in range(15):tick()
    check('turn_exit_active' in symbols and var('turn_exit_active'),'Documents15 N078 reverse uses outgoing turn recovery')
    for _ in range(110):
        if var('turn_exit_active') and member('turn_exit_heading','spinning'):
            yaw=wrap(yaw-round(signed16(member('turn_exit_heading','last_turn'))/5))
        if var('turn_exit_active') and var('turn_exit_line_phase'):
            gray_raw=0x60;gray_state=1
        tick()
        if member('map_nav','step')==1:break
    check(member('map_nav','step')==1 and member('thief_localization','phase')==1,'N078 commits actual N078->N077 exit')
    check(member('thief_localization','confirmed_node')==78,'do not claim arrival at N077 during recovery')

def turns():
    reverse_n078();count=1
    for body in range(4):
      for mode in [0,1]:
       for sign in [-1,1]:
        for quarter in [-1,1]:pending_root(body,mode,sign,quarter);count+=1
    pending_root(1,0,0,1);count+=1
    return count

def control_boundaries():
    s=allocate('RoadHeadingControl_t',0x2000c800)
    i=allocate('RoadHeadingInput_t',0x2000c840);o=allocate('RoadHeadingOutput_t',0x2000c870)
    def update(now,error,stamp,stopped=1):
        record(i,'RoadHeadingInput_t',dict(cw_error_x100=error,yaw_tick=stamp,stopped=stopped,direction_known=1))
        call('RoadHeading_Update',s,i,now,200,o)
        return {k:at(o,'RoadHeadingOutput_t',k) for k in ['turn_x10','brake','ready']}
    call('RoadHeading_Reset',s,1000)
    for now in [1020,1080,1100,1120]:
        check(not update(now,100,900)['ready'],'pre-brake gyro sample cannot certify fresh alignment')
    check(at(s,'RoadHeadingControl_t','stable_count')==0,'old timestamps do not accumulate')
    call('RoadHeading_Reset',s,1000)
    check(update(1020,1392,1020)['brake'],'brake before trim')
    check(not update(1080,1392,1080)['brake'],'measured standstill releases trim')
    check(update(1100,-400,1100,0)['brake'],'stop before correction polarity reversal')
    for now in [1120,1140,1160]:check(update(now,-400,now)['brake'],'reversal waits full brake interval')
    check(not update(1180,-400,1180)['brake'],'reversal allowed after standstill')
    check(update(1200,100,1200,0)['brake'],'within tolerance stops first')
    for now in [1220,1240,1260,1280,1300]:
        check(not update(now,100,1200)['ready'],'re-reading one gyro frame never certifies stable parking')
    for n,now in enumerate([1320,1340,1360]):
        check(bool(update(now,100,now)['ready'])==(n==2),'three distinct gyro stamps needed')
    for origin in [-17000,17000]:
      for origin_heading in range(4):
       for body in range(4):
        for sign in [-1,1]:
            current=wrap(origin+(body-origin_heading)*9000*sign+1392)
            result=call('RoadHeading_AxisError',origin & 0xffffffff,origin_heading,body,sign & 0xffffffff,current & 0xffffffff)
            check(signed16(result & 65535)==1392,'fresh axis error matches origin, body, sign and angle wrap')
    for sign in [-1,0,1]:
      for quarter in [-1,1]:
        actual_sign=sign or -1;start_yaw=17000
        current=wrap(start_yaw+actual_sign*quarter*5166)
        check(call('RoadHeading_TurnError',start_yaw & 0xffffffff,current & 0xffffffff,quarter & 0xffffffff,sign & 0xffffffff,0x2000c880)==1,
              'turn error supports wrap, learned sign and first-turn sign inference')
        check(signed16(get(0x2000c880,2))==quarter*(5166-9000),'relative outgoing yaw error correct')

failures=[];passed=[];counts={}
tests=[parking_cases,turns] if '--before' in sys.argv else [parking_cases,turns,control_boundaries]
for fn in tests:
    try:counts[fn.__name__]=fn();passed.append(fn.__name__);print(fn.__name__+': PASS',flush=True)
    except AssertionError as e:failures.append(dict(case=fn.__name__,error=str(e)));print(fn.__name__+': FAIL '+str(e),flush=True)
result=dict(checks=checks,failures=failures,passed=passed,cases=counts,hardware_tested=False,
 firmware_sha256=hashlib.sha256(axf.read_bytes()).hexdigest(),
 app_source_sha256=hashlib.sha256((ROOT/('Tests/before_documents15/User/App_ThiefStrategy.c' if '--before' in sys.argv else 'User/App_ThiefStrategy.c')).read_bytes()).hexdigest(),
 test_script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
 helper_sha256={n:hashlib.sha256((ROOT/'Tests'/n).read_bytes()).hexdigest() for n in
    ['test_app_runtime.py','test_escape_completion.py','test_documents13.py']})
out=ROOT/'Tests'/('before_documents15/regression_before.json' if '--before' in sys.argv else 'results_documents15.json')
out.write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8');print(json.dumps(result,indent=2))
sys.exit(bool(failures))
