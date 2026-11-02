"""CH3 restored by user; rear-leading escape, actual ARM; synthetic hardware."""
from pathlib import Path
import sys,json,hashlib
src=Path(__file__).with_name('test_documents15.py').read_text(encoding='utf-8').split('parking_pwm=[]')[0]
src=src.replace("if '--before' in sys.argv:","if False:")
if '--before' in sys.argv:
    src=src.replace('exec(compile(runtime,',"runtime=runtime.replace(\"axf=ROOT/'MDK-ARM'/ROOT.name/(ROOT.name+'.axf')\",\"axf=ROOT/'Tests/before_documents18/original.axf'\")\nexec(compile(runtime,")
exec(compile(src,str(Path(__file__).with_name('test_documents15.py')),'exec'))

def channel_off():
    # Historical name retained for before-RUN10 evidence; current user restored CH3.
    global clock
    fresh(38,1)
    check(var('thief_laser_active_mask')&4,'restored CH3 must be present in detection mask')
    for _ in range(3):
        clock+=20;call('Thief_OnLaserFrame',2,150,100,1,clock)
    check(var('thief_escape_requested') and get(symbols['thief_laser_seen']+2,1),'new right probe can trigger a confirmed escape')

def boot_three_channels():
    global clock
    fresh(38,1)
    sent=[]
    for _ in range(8):
      for ch in [0,1,2,3]:
        raw=bytes([0xaa,5,104,0,150,0,100]);raw+=bytes([sum(raw)&255])
        sent.extend(f'{ch+1}#'.encode()+raw)
    pos=0;fallback=[]
    def hal_tick():
        global clock
        clock+=1;return clock
    hook('HAL_GetTick',hal_tick)
    ret('UartCh4_OpenPassive',1);ret('UartCh4_SetReceiving',1);ret('UartCh4_TakeLost',0)
    ret('UartCh4_Ready',0)
    hook('UartCh4_Init',lambda:fallback.append(1) or 0)
    def poll(uc,address,length,data):
        nonlocal pos
        if pos>=len(sent):
            uc.reg_write(UC_ARM_REG_PC,uc.reg_read(UC_ARM_REG_LR));return
        callback=uc.reg_read(UC_ARM_REG_R0)
        uc.reg_write(UC_ARM_REG_R0,sent[pos]);uc.reg_write(UC_ARM_REG_R1,clock);pos+=1
        uc.reg_write(UC_ARM_REG_PC,callback)
    handle=u.hook_add(UC_HOOK_CODE,poll,begin=symbols['UartCh4_Poll']&~1,end=symbols['UartCh4_Poll']&~1)
    try:
        call('LaserRx_Init')
        check(call('LaserRx_LinkReady')==1 and not fallback,'all four valid parsed channels establish boot link')
        check(var('data_link_ready')==1,'boot proof comes from real parsed required channels')
    finally:u.hook_del(handle);hook('HAL_GetTick',lambda:clock)

def rear_first_plans():
    count=0
    for node,axes in [(11,[1,3]),(38,[1,3]),(26,[0,2]),(35,[1,3]),(63,[0,2])]:
      for body in axes:
        fresh(node,body)
        check(call('ThiefMapPlanner_PlanEscape',node,body,1<<body,0,clock,plan)==1,'front-threat escape remains feasible')
        step=steps
        check(at(step,'MapRouteStep_t','drive_mode')==1,'prefer rear-leading departure instead of front-leading pivot')
        heading=at(step,'MapRouteStep_t','heading');mode=at(step,'MapRouteStep_t','drive_mode')
        check(heading!=body,'MONITOR6 full-route search cannot depart into actual front threat')
        out_body=(heading+2*mode)%4
        check((out_body-body)%4!=2,'no unnecessary180degree body pivot')
        call('Route_Start')
        check(bool(var('departure_turn'))==(out_body!=body),'actual chosen route determines whether a real quarter-turn is needed')
        if out_body==body:check(call('TravelFrame_IsReverse')==1,'straight-back chosen route uses rear gray without body pivot')
        goal=member('thief_plan','goal_node');result=complete_route()
        check(result['node']==goal and goal!=node,'rear-first complete route reaches selected refuge')
        count+=1
    # N036 front west blocked: prefer a rear-leading side turn when no rear road.
    fresh(36,3)
    check(call('ThiefMapPlanner_PlanEscape',36,3,8,0,clock,plan)==1,'Documents18 N036 has an escape')
    check(at(steps,'MapRouteStep_t','drive_mode')==1,'N036 root turn uses tail as leading end')
    call('Route_Start')
    check(var('departure_turn')==1,'N036 performs a real rear-leading side turn when rear straight road is absent')
    goal=member('thief_plan','goal_node');result=complete_route()
    check(result['node']==goal,'N036 rear-leading turn completes at the selected refuge')
    return count

def side_replay_after_wait():
    global clock
    fresh(38,1);setstep(38,39,1,0,300);call('Route_Start')
    put(symbols['departure_known_clearance'],2,480)
    clock+=700;call('Thief_UpdatePose');call('Thief_UpdateLaserMask',clock)
    clock+=1;call('Thief_OnLaserFrame',1,380,100,1,clock);tick()
    check(var('route_state')!=12 and var('route_active'),'already-planned side echo cannot cancel zero-motion departure merely after500ms')
    clock+=20;call('Thief_OnLaserFrame',1,200,100,1,clock);tick()
    check(var('route_state')==12,'substantially closer side obstacle still brakes immediately')

def oncoming_during_cooldown():
    global clock
    fresh(38,1);setstep(38,39,1,0,300);call('Route_Start')
    var('intercept_has_reversed',1);var('intercept_reverse_tick',clock)
    call('Thief_UpdatePose');call('Thief_UpdateLaserMask',clock)
    clock+=1;call('Thief_OnLaserFrame',0,150,100,1,clock);tick()
    check(var('route_state')==12,'actual new oncoming obstacle brakes even during reversal cooldown')

def latched_distance():
    global clock
    fresh(38,1)
    var('thief_strategy_state',5);var('hide_alignment_pending',0)
    var('route_state',9);var('route_active',0);var('route_state_tick',clock)
    call('Thief_UpdatePose');call('Thief_UpdateLaserMask',clock)
    for _ in range(3):
        clock+=20;call('Thief_OnLaserFrame',1,380,100,1,clock)
    check(var('thief_escape_requested'),'three stationary frames confirm side threat')
    clock+=20;call('Thief_OnLaserFrame',1,0,0,1,clock)
    for _ in range(4):tick()
    check(var('route_active'),'confirmed request survives invalid later frame during settling')
    check(get(symbols['departure_known_clearance'],2)==480,'preserve confirmed physical clearance across settle/replan')

failures=[];passed=[];counts={}
for fn in [channel_off,boot_three_channels,rear_first_plans,side_replay_after_wait,latched_distance,oncoming_during_cooldown]:
    try:counts[fn.__name__]=fn();passed.append(fn.__name__);print(fn.__name__+': PASS',flush=True)
    except AssertionError as e:failures.append(dict(case=fn.__name__,error=str(e)));print(fn.__name__+': FAIL '+str(e),flush=True)
before='--before' in sys.argv
result=dict(checks=checks,failures=failures,passed=passed,cases=counts,hardware_tested=False,
 firmware_sha256=hashlib.sha256(axf.read_bytes()).hexdigest(),
 app_source_sha256=hashlib.sha256((ROOT/('Tests/before_documents18/User/App_ThiefStrategy.c' if before else 'User/App_ThiefStrategy.c')).read_bytes()).hexdigest(),
 test_script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
 helper_sha256={n:hashlib.sha256((ROOT/'Tests'/n).read_bytes()).hexdigest() for n in ['test_documents15.py','test_app_runtime.py','test_escape_completion.py','test_documents13.py']})
out=ROOT/'Tests'/('before_documents18/regression_before.json' if before else 'results_documents18.json')
out.write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8');print(json.dumps(result,indent=2));sys.exit(bool(failures))
