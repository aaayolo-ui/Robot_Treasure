"""Documents19: execute actual linked ARM with explicitly synthetic hardware."""
from pathlib import Path
import sys,json,hashlib
src=Path(__file__).with_name('test_documents15.py').read_text(encoding='utf-8').split('parking_pwm=[]')[0]
src=src.replace("if '--before' in sys.argv:","if False:")
if '--before' in sys.argv:
    src=src.replace('exec(compile(runtime,',"runtime=runtime.replace(\"axf=ROOT/'MDK-ARM'/ROOT.name/(ROOT.name+'.axf')\",\"axf=ROOT/'Tests/before_documents19/original.axf'\")\nexec(compile(runtime,")
exec(compile(src,str(Path(__file__).with_name('test_documents15.py')),'exec'))

def ordinary_turn():
    global clock,odom,gray_raw,gray_state,yaw
    # Actual log N063->N052, north arrival, right/east exit, left-only FC0.
    fresh(63,0);setstep(63,52,0,0,300,action=1)
    call('ThiefMapPlanner_HasRoad',52,1,0x2000c400,0x2000c402)
    dest=get(0x2000c400,2);length=get(0x2000c402,2)
    setstep(52,dest,1,0,length,index=1);record(route,'MapRoute_t',dict(count=2))
    call('Route_Start');tick();odom=300;clock+=300
    call('Route_HandleJunction',clock,20,4,0xfc0)
    check(member('thief_localization','confirmed_node')==52,'real gray+distance arrival confirms N052')
    gray_raw=0xfc0;gray_state=4
    for _ in range(22):tick()
    check(var('route_state')==5,'map-confirmed ordinary right turn starts with opposite-only branch')
    return 1

def reverse_first_follow():
    global odom,clock,gray_raw,gray_state,rear_error
    fresh(38,1);setstep(38,39,1,0,300);call('Route_Start');tick()
    odom=140;clock+=20;call('Thief_UpdatePose');call('Thief_UpdateLaserMask',clock)
    call('Thief_OnLaserFrame',0,150,100,1,clock);tick()
    check(var('route_state')==12,'real oncoming laser enters measured brake')
    # Inspect calls rather than pretend that target values prove physical travel.
    events.clear();targets.clear();gray_raw=0x40;gray_state=1;rear_error=-100
    for _ in range(8):
        tick()
        if var('retreat_active'):break
    check(var('retreat_active')==1 and call('TravelFrame_IsReverse')==1,'retreat swaps frame after measured stop')
    check(targets and 'Route_DriveStraight' not in events,'first reverse command uses actual leading gray closed loop, not blind straight seed')
    return 1

def approach_corridor():
    paths=[]
    for body in [1,3]:
        fresh(38,body)
        check(call('ThiefMapPlanner_PlanEscape',38,body,1,0,clock,plan)==1,'north threat still has an escape')
        path=[];bad=[]
        for i in range(at(route,'MapRoute_t','count')):
            st=steps+i*size(types['MapRouteStep_t']);node=at(st,'MapRouteStep_t','to_node')
            xy=call('ThiefMapPlanner_Node',node);x=at(xy,'MapRouteNode_t','x_half')*150;y0=at(xy,'MapRouteNode_t','y_half')*150
            path.append(node)
            # North approach corridor one metre ahead, two grid cells wide.
            if 200<=y0<1200 and abs(x-300)<=600:bad.append(node)
        check(not bad,'N038 north threat must use available south detour, route='+str(path)+' conflict='+str(bad))
        paths.append(path)
    return paths


def reverse_junction_transient():
    global clock,odom,gray_raw,gray_state
    fresh(38,1);setstep(38,37,3,1,300,action=4);setstep(37,38,1,0,300,index=1)
    record(route,'MapRoute_t',dict(count=2));call('Route_Start');tick()
    odom=300;clock+=300;call('Route_HandleJunction',clock,20,3,0xfff)
    check(var('reverse_pending')==1,'test performs a real map reversal')
    # A single missing sample at the stopped frame swap is recoverable.
    gray_raw=0;gray_state=2
    for _ in range(22):
        tick()
        if not var('reverse_pending'):break
    check(var('route_state')==8 and var('route_active'),'missing new leading sample enters retry instead of one-shot FAULT')
    gray_raw=0x60;gray_state=1
    for _ in range(6):tick()
    check(var('route_state')==2,'real line resumes the same pending exit')
    return 1

def ordinary_turn_matrix():
    global clock,odom,gray_raw,gray_state
    ncase=0
    for n in range(1,89):
      if at(call('ThiefMapPlanner_Node',n),'MapRouteNode_t','kind')==0:continue
      for incoming in range(4):
        if not call('ThiefMapPlanner_HasRoad',n,(incoming+2)%4,0x2000c400,0x2000c402):continue
        source=get(0x2000c400,2);length=get(0x2000c402,2)
        if at(call('ThiefMapPlanner_Node',source),'MapRouteNode_t','kind')==0:continue
        for q in [-1,1]:
          # Opposite-only observation is supported only where that real road exists.
          if not call('ThiefMapPlanner_HasRoad',n,(incoming-q)%4,0,0):continue
          if not call('ThiefMapPlanner_HasRoad',n,(incoming+q)%4,0x2000c400,0x2000c402):continue
          dest=get(0x2000c400,2);after=get(0x2000c402,2)
          if at(call('ThiefMapPlanner_Node',dest),'MapRouteNode_t','kind')==0:continue
          for mode in [0,1]:
           for sign in [-1,1]:
            fresh(source,(incoming+2*mode)%4);member('thief_localization','yaw_cw_sign',sign)
            setstep(source,n,incoming,mode,length,action=q);setstep(n,dest,(incoming+q)%4,mode,after,index=1)
            record(route,'MapRoute_t',dict(count=2));call('Route_Start');tick()
            odom=length;clock+=300;raw=0xfc0 if q==1 else 0x3f
            call('Route_HandleJunction',clock,20,4 if q==1 else 5,raw)
            gray_raw=raw;gray_state=4 if q==1 else 5
            for _ in range(22):tick()
            check(var('route_state')==(5 if q==1 else 6),'opposite-only mapped turn starts n=%d h=%d mode=%d sign=%d'%(n,incoming,mode,sign))
            check(member('map_nav','step')==0 and member('map_nav','arrived'),'no exit committed before actual outgoing gray/yaw')
            ncase+=1
    return ncase

def dual_gray_frame_symmetry():
    global clock
    # Real GrayPosition, mount conversion, dual-board PID, smoothing and
    # TravelFrame run unhooked. Only physical sensor reads/wheel outputs stubbed.
    removed={}
    for name in ['Route_ReadFront','Route_ReadRear','Route_UpdateFollowMode','TravelFrame_SetWheelTargets']:
        if name in hooks:removed[name]=hooks.pop(name);u.hook_del(removed[name])
    physical=[0x60,0x60];wheel=[]
    def read0():put(u.reg_read(UC_ARM_REG_R0),2,physical[0]);return 0
    def read1():put(u.reg_read(UC_ARM_REG_R0),2,physical[1]);return 0
    hook('NCHD12_ReadRaw16',read0);hook('NCHD12Rear_ReadRaw16',read1)
    def capture():
        wheel.append(tuple((u.reg_read(r)+2**31)%2**32-2**31 for r in [UC_ARM_REG_R0,UC_ARM_REG_R1,UC_ARM_REG_R2,UC_ARM_REG_R3]));return 0
    hook('ChassisSpeedControl_SetWheelTargets',capture)
    sample=0x2000c500;curves=[]
    try:
      for mode in [0,1]:
        fresh(38,1);call('TravelFrame_SetReverse',mode)
        record(symbols['line_follow_pid_parameters'],'LineFollowPID_Parameters_t',dict(kp_x1000=65,ki_x1000=8,kd_x1000=9,kh_x1000=5))
        call('LineFollowPID_Reset',symbols['line_follow_pid']);call('FollowSmoothing_Reset',symbols['follow_smoothing'])
        member('follow_smoothing','base_rpm_x10',2000);wheel.clear()
        for lead,trail in [(0x60,0x60),(0x40,0x40),(0x80,0x20),(0xc0,0x30),(0x20,0x80),(0x10,0x100),(0x60,0x60)]:
          physical[:]=[lead,trail] if mode==0 else [trail,lead]
          call('Route_ResetSample',sample);state=call('Route_ReadFront',sample)
          check(state==1,'actual leading board must be valid for frame parity case')
          var('front_state',state);call('Route_RunFollow',sample,20);clock+=20
          fe=at(sample,'RouteGraySample_t','front_error_x100');re=at(sample,'RouteGraySample_t','rear_error_x100')
          if var('follow_mode')==1:
            check((at(sample,'RouteGraySample_t','heading_error_x100')%65536)==(fe-re)%65536,'reference front-minus-rear heading rule retained')
        curves.append(wheel[:])
      check(any(x[0]!=x[1] for x in curves[0]),'nonzero observed offsets must cause real steering, not uninitialized zero gains')
      check(len(curves[0])==len(curves[1])==7,'every sample yields one closed-loop target')
      for f,r in zip(*curves):check(r==tuple(-v for v in f[::-1]),'reverse physical outputs are exact rotated forward control')
    finally:
      for name in ['NCHD12_ReadRaw16','NCHD12Rear_ReadRaw16','ChassisSpeedControl_SetWheelTargets']:
        if name in hooks:u.hook_del(hooks.pop(name))
      hook('Route_ReadFront',readgray);hook('Route_ReadRear',rear_read)
      ret('Route_UpdateFollowMode');hook('TravelFrame_SetWheelTargets',target_capture)
    return dict(samples=14,forward=curves[0],reverse=curves[1])

failures=[];passed=[];cases={}
for fn in [ordinary_turn,reverse_first_follow,approach_corridor]+([] if '--before' in sys.argv else [reverse_junction_transient,ordinary_turn_matrix,dual_gray_frame_symmetry]):
    try:cases[fn.__name__]=fn();passed.append(fn.__name__);print(fn.__name__+': PASS',flush=True)
    except AssertionError as e:failures.append(dict(case=fn.__name__,error=str(e)));print(fn.__name__+': FAIL '+str(e),flush=True)
before='--before' in sys.argv
result=dict(checks=checks,failures=failures,passed=passed,cases=cases,hardware_tested=False,
 firmware_sha256=hashlib.sha256(axf.read_bytes()).hexdigest(),
 app_source_sha256=hashlib.sha256((ROOT/('Tests/before_documents19/User/App_ThiefStrategy.c' if before else 'User/App_ThiefStrategy.c')).read_bytes()).hexdigest(),
 test_script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
 helper_sha256={n:hashlib.sha256((ROOT/'Tests'/n).read_bytes()).hexdigest() for n in ['test_documents15.py','test_app_runtime.py','test_escape_completion.py','test_documents13.py']})
out=ROOT/'Tests'/('before_documents19/regression_before.json' if before else 'results_documents19.json')
out.write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8');print(json.dumps(result,indent=2));sys.exit(bool(failures))
