"""042252: no routine +/-2degree pulses; actual narrow gray drives the PID."""
from pathlib import Path
import re,sys,json,hashlib
src=Path(__file__).with_name('test_documents15.py').read_text(encoding='utf-8').split('parking_pwm=[]')[0]
src=src.replace('Tests/before_documents15/original.axf','Tests/before_line_handoff/original.axf')
exec(compile(src,str(Path(__file__).with_name('test_documents15.py')),'exec'))
helpers=Path(__file__).with_name('test_general_turns.py').read_text(encoding='utf-8')
exec(helpers[helpers.index('data=(ROOT'):helpers.index('def all_map_quarter_turns(')])
pwm=[]
def capture():
    v=u.reg_read(UC_ARM_REG_R1);pwm.append(v-2**32 if v>=2**31 else v);return 0
hook('Chassis_SetWheelLogicalPwm',capture)

def five_degree_handoff():
    global yaw,clock
    for mode in [0,1]:
      for sign in [-1,1]:
        setup_turn(78,1,-1,mode,sign)
        yaw=-sign*8500;ret('Route_AllWheelsStopped',0);pwm.clear()
        for _ in range(3):clock+=20;call('Route_AlignTurnAxis',clock,20)
        check(var('turn_axis_active'),'moving wheels cannot hand off')
        ret('Route_AllWheelsStopped',1)
        for _ in range(4):clock+=20;call('Route_AlignTurnAxis',clock,20)
        check(not var('turn_axis_active'),'5degree residual must hand to real gray rather than stationary2degree trim')
        check(not pwm,'normal handoff cannot emit fine-trim rotation pulses')
        check(member('thief_localization','confirmed_node')==78 and member('thief_localization','phase')==2,'heading alone does not confirm another node or exit')
    return 4

rear_raw=0x60;rear_offset=0
def real_rear_sample():
    record(u.reg_read(UC_ARM_REG_R0),'RouteGraySample_t',dict(rear_raw12=rear_raw,rear_error_x100=rear_offset))
    return 1
hook('Route_ReadRear',real_rear_sample)

def rear_width_follow():
    global rear_raw,rear_offset
    if 'Route_UpdateFollowMode' in hooks:u.hook_del(hooks.pop('Route_UpdateFollowMode'))
    for mode in [0,1]:
        fresh(4,(3+2*mode)%4);setstep(4,3,3,mode,300);call('Route_Start');tick()
        sample=allocate('RouteGraySample_t',0x2000cd00)
        record(sample,'RouteGraySample_t',dict(front_raw12=0x60,front_error_x100=0))
        rear_raw=0x60;rear_offset=0
        for _ in range(4):call('Route_UpdateFollowMode',sample)
        rear_raw=0x0ff;rear_offset=300
        call('Route_UpdateFollowMode',sample)
        check(at(sample,'RouteGraySample_t','heading_error_x100')==0,'wide rear branch must not become dual-board heading error')
        check(at(sample,'RouteGraySample_t','rear_raw12')==0x0ff,'raw wide branch remains available for node evidence')
        rear_raw=0x60;rear_offset=100
        for _ in range(4):call('Route_UpdateFollowMode',sample)
        check(at(sample,'RouteGraySample_t','heading_error_x100')!=0,'a narrow rear line still contributes actual heading correction')
    return 2

seen=[]
def observe_probe(uc,address,length,data):
    p=uc.reg_read(UC_ARM_REG_R1)
    seen.append(at(p,'RestartLineInput','trailing_valid'))
if 'RestartLineSearch_Probe' in symbols:
    u.hook_add(UC_HOOK_CODE,observe_probe,begin=symbols['RestartLineSearch_Probe']&~1,end=symbols['RestartLineSearch_Probe']&~1)

def rear_width_recovery():
    global rear_raw,rear_offset,gray_raw,gray_state,odom
    for raw,expected in [(0xff,0),(0xff0,0),(0x60,1),(0xc0,1)]:
        fresh(4,3);setstep(4,3,3,0,300);call('Route_Start');tick()
        gray_raw=0;gray_state=2;var('front_state',2);odom=30
        rear_raw=raw;rear_offset=300;seen.clear()
        var('map_gray_active',1);var('route_state',8)
        sample=allocate('RouteGraySample_t',0x2000cd00)
        call('Route_MapGrayRecover',clock,20,sample)
        check(seen and seen[-1]==expected,'wide rear cannot steer recovery; narrow rear can')
        check(var('route_active'),'rear-width classification must not introduce a parking fault')
    return 4

def first_unknown_sign_turn():
    global clock,yaw
    fresh(11,1);member('thief_localization','yaw_cw_sign',0)
    setstep(11,22,2,0,300);call('Route_Start');tick(20);tick(60)
    for _ in range(20):
        tick()
        if var('route_state') in (5,6):break
    check(var('route_state')==5,'actual unknown-sign first right turn has begun')
    yaw=4500;tick()
    check(var('route_state')==5,'removing fine trim must not leave the first unknown-sign turn at45degrees')
    yaw=7500;tick()
    check(var('route_state')==7 and var('turn_axis_active'),'unknown-sign first turn reaches75degree coarse target')
    return 1

failures=[];cases={}
for fn in [five_degree_handoff,rear_width_follow,rear_width_recovery,first_unknown_sign_turn]:
    try:cases[fn.__name__]=fn()
    except Exception as e:failures.append(dict(case=fn.__name__,error=str(e)))
axf=ROOT/'MDK-ARM'/ROOT.name/(ROOT.name+'.axf')
if '--before' in sys.argv:axf=ROOT/'Tests/before_line_handoff/original.axf'
r=dict(checks=checks,cases=cases,failures=failures,axf_sha256=hashlib.sha256(axf.read_bytes()).hexdigest(),
       test_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),hardware_tested=False)
out=ROOT/('Tests/before_line_handoff/regression_before.json' if '--before' in sys.argv else 'Tests/results_line_handoff.json')
out.write_text(json.dumps(r,indent=2),encoding='utf-8');print(json.dumps(r,indent=2));sys.exit(bool(failures))
