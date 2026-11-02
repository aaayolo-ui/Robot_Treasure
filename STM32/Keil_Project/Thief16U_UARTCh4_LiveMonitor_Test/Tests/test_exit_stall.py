"""214751 stagnant yaw/side gray and214323 blind probe; simulated hardware."""
from pathlib import Path
import sys,json,hashlib
src=Path(__file__).with_name('test_documents15.py').read_text(encoding='utf-8').split('parking_pwm=[]')[0]
src=src.replace('Tests/before_documents15/original.axf','Tests/before_exit_stall/original.axf')
exec(compile(src,str(Path(__file__).with_name('test_documents15.py')),'exec'))
src=Path(__file__).with_name('test_documents24.py').read_text(encoding='utf-8')
exec(src[src.index('def pending('):src.index('def slow_correction(')])
pwm=[]
def capture_pwm():
    v=u.reg_read(UC_ARM_REG_R1);v=v-2**32 if v>=2**31 else v
    pwm.append((u.reg_read(UC_ARM_REG_R0),v));return 0
hook('Chassis_SetWheelLogicalPwm',capture_pwm)
cue_error=0
def cue_read():
    result=readgray()
    record(u.reg_read(UC_ARM_REG_R0),'RouteGraySample_t',dict(front_error_x100=cue_error))
    return result
hook('Route_ReadFront',cue_read)

def stagnant_yaw():
    global yaw,gray_raw,gray_state,rear_valid,clock
    for mode in [0,1]:
      for sign in [-1,1]:
        pending(mode,sign);yaw=sign*q*(9000-1546);gray_raw=0xc00;gray_state=4;rear_valid=1;pwm.clear()
        for _ in range(35):tick()
        check(pwm and len(pwm)%4==0,'stagnant15.46degree yaw must escalate to four-wheel measured PWM pulses')
        check(all(abs(v)==550 for _,v in pwm),'fallback reuses verified normal-turn PWM550')
        check(sum(v for _,v in pwm[-4:])==0,'boost is rotation, not blind translation outside10degrees')
        yaw=sign*q*(9000-500)
        for _ in range(20):tick()
        check(var('turn_exit_line_phase')==1,'fresh observed correction may enter original10degree search')
    return 4

def side_guidance():
    global yaw,gray_raw,gray_state,rear_valid,targets,cue_error
    for mode in [0,1]:
     for raw,error in [(0xc00,-500),(0x003,500),(0x800,-550),(0x001,550)]:
      pending(mode,1);yaw=q*(9000-100);rear_valid=2
      for _ in range(10):tick()
      gray_raw=raw;gray_state=1 if raw in (0x800,1) else 4;cue_error=error;targets.clear()
      for _ in range(8):tick()
      check(any(l+r>0 for l,r,*_ in targets),'side-only leading black must guide bounded search instead of stationary sweep')
      check(any((l-r)*error>0 for l,r,*_ in targets),'actual side gray must steer toward observed side, not stale/default bias')
      check(member('thief_localization','phase')==2,'side fragment cannot certify outgoing edge')
      cue_error=0
    return 8

def boost_stop_and_fresh_yaw():
    global yaw,gray_raw,gray_state,rear_valid,clock
    pending();yaw=q*(9000-1546);gray_raw=0xc00;gray_state=4
    ret('Route_AllWheelsStopped',0);pwm.clear()
    for _ in range(25):
        tick()
        if 'turn_exit_boost' in symbols and var('turn_exit_boost'):break
    stamp=clock
    for _ in range(2):tick()
    check(not pwm,'boost must measure standstill before first PWM pulse')
    ret('Route_AllWheelsStopped',1)
    hook('JY61P_GetLastUpdateMs',lambda:stamp)
    for _ in range(3):tick()
    check(not pwm,'old yaw cannot authorize a new boost pulse')
    hook('JY61P_GetLastUpdateMs',lambda:clock)
    for _ in range(8):tick()
    check(pwm,'standstill and fresh yaw authorize measured recovery pulse')
    return 1

def failed_probe_return():
    global yaw,gray_raw,gray_state,rear_valid,clock,odom
    for mode in [0,1]:
      for sign in [-1,1]:
        pending(mode,sign);yaw=sign*q*(9000-100);rear_valid=2
        for _ in range(10):tick()
        odom=100;tick();targets.clear()
        ret('Route_AllWheelsStopped',0)
        for _ in range(18):tick()
        check(not any(l+r<0 for l,r,*_ in targets),'return cannot reverse moving wheels')
        ret('Route_AllWheelsStopped',1)
        for _ in range(8):tick()
        check(any(l+r<0 for l,r,*_ in targets),'failed100mm probe must retrace toward its measured origin')
        check(member('thief_localization','phase')==2,'probe return does not move confirmed node')
        gray_raw=0x60;gray_state=1;targets.clear();tick()
        check(not targets,'line caught while rewinding must brake before forward follow')
        for _ in range(18):tick()
        check(member('thief_localization','phase')==1,'real recovered line permits measured-stop exit handoff')
    return 4

def return_origin_bound():
    global yaw,gray_raw,gray_state,rear_valid,clock,odom
    pending();yaw=q*(9000-100);rear_valid=2
    for _ in range(10):tick()
    odom=100
    for _ in range(25):tick()
    check(var('turn_exit_line_phase')==3,'return stage really reached')
    odom=5;targets.clear();tick()
    check(var('turn_exit_line_phase')==2 and not targets,'measured origin stops reverse probing')
    odom=100;targets.clear()
    for _ in range(20):tick()
    check(not any(l+r<0 for l,r,*_ in targets),'one failed probe must not rewind repeatedly')
    check(member('thief_localization','confirmed_node')==37,'return envelope cannot fabricate node')
    return 1

def continuous_loss_still_stops():
    global yaw,gray_raw,gray_state,rear_valid,clock,odom
    pending();yaw=q*(9000-100);rear_valid=2
    for _ in range(10):tick()
    odom=100
    for _ in range(110):
        clock+=20;call('App_ThiefStrategy_Task')
    check(not var('route_active') and reason()=='GRAY_BOTH_NO_LINE','return/scan must not renew continuous2second loss clock')
    return 1

failures=[];cases={}
for fn in [stagnant_yaw,side_guidance,boost_stop_and_fresh_yaw,failed_probe_return,return_origin_bound,continuous_loss_still_stops]:
    try:cases[fn.__name__]=fn();print(fn.__name__+': PASS',flush=True)
    except AssertionError as exc:failures.append(dict(case=fn.__name__,error=str(exc)));print(fn.__name__+': FAIL '+str(exc),flush=True)
    finally:
        cue_error=0;ret('Route_AllWheelsStopped',1);hook('JY61P_GetLastUpdateMs',lambda:clock)
result=dict(checks=checks,failures=failures,cases=cases,hardware_tested=False,
 axf_sha256=hashlib.sha256(axf.read_bytes()).hexdigest(),test_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
out=ROOT/'Tests'/('before_exit_stall/regression_before.json' if '--before' in sys.argv else 'results_exit_stall.json')
out.write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8');print(json.dumps(result,indent=2));sys.exit(bool(failures))
