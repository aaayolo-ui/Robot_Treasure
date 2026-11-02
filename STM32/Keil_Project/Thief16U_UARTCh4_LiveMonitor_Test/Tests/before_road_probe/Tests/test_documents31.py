"""Documents31 actual ARM firmware; hardware inputs are synthetic."""
from pathlib import Path
import sys,json,hashlib
src=Path(__file__).with_name('test_documents15.py').read_text(encoding='utf-8').split('parking_pwm=[]')[0]
src=src.replace("if '--before' in sys.argv:","if False:")
if '--before' in sys.argv:
    src=src.replace('exec(compile(runtime,',"runtime=runtime.replace(\"axf=ROOT/'MDK-ARM'/ROOT.name/(ROOT.name+'.axf')\",\"axf=ROOT/'Tests/before_documents31/original.axf'\")\nexec(compile(runtime,")
exec(compile(src,str(Path(__file__).with_name('test_documents15.py')),'exec'))
# Keep actual PI enable bookkeeping for the retry-after-brake case.
for n in ['Route_ResetControl','Route_BrakeControl','ChassisSpeedControl_Brake','ChassisSpeedControl_Enable']:
    if n in hooks:u.hook_del(hooks.pop(n))

def finish_and_escape(node,body):
    first=complete_route()
    check(first['node']==node,'real node-stop completion matches confirmed junction')
    check(call('ThiefMapPlanner_PlanEscape',node,body,0,0,clock,plan),'confirmed junction has a complete onward refuge route')
    call('Route_Start');result=complete_route()
    check(result['node'] in [11,26,35,38,63],'recovery continues all the way to an allowed refuge')
    return result

def mismatch_retry():
    global odom,clock,gray_raw,gray_state
    paths=[]
    for mode in [0,1]:
        fresh(68,2*mode);setstep(68,58,0,mode,300);call('Route_Start');tick()
        odom=211;clock+=400;gray_raw=0xc60;gray_state=4
        count=var('junction_count');confirmed=member('thief_localization','confirmations')
        call('Route_BrakeControl')
        for raw in [0xc60,0xc00,0xce0]:
            clock+=20;gray_raw=raw;call('Route_HandleJunction',clock,20,4,gray_raw)
            check(var('route_active') and member('thief_localization','error')==0,'wrong-side single frame must not latch LOC_GRAY_MISMATCH')
            check(not member('map_nav','arrived') and var('junction_count')==count,'rejected frame must not mutate navigator or junction count')
            check(member('thief_localization','confirmations')==confirmed,'no invented node fix on mismatched gray')
            check(var('speed_control_enabled') and targets,'actual brake-to-retry restores linked PI enable state and drive output')
        gray_raw=0x063;gray_state=4;odom=280;clock+=100
        call('Route_HandleJunction',clock,20,4,gray_raw)
        check(member('thief_localization','confirmed_node')==58,'subsequent actual matching branch confirms N058')
        paths.append(finish_and_escape(58,2*mode))
    # A watch callback can arrive every5ms while PI runs at20ms. Rejected
    # frames must not integrate the same elapsed interval repeatedly.
    fresh(68,0);setstep(68,58,0,0,300);call('Route_Start');tick();odom=211
    targets.clear();base=var('last_control_tick')
    for dt in [5,10,15]:
        clock=base+dt;call('Route_HandleJunction',clock,dt,4,0xc60)
        check(not targets,'5ms watch retries do not run wheel PI before20ms')
    clock=base+20;call('Route_HandleJunction',clock,20,4,0xc60)
    check(len(targets)==1 and var('last_control_tick')==clock,'one actual20ms retry consumes the control interval once')
    return paths

def full_black_next_junction():
    global clock,yaw,odom,gray_raw,gray_state,rear_valid
    cases=[]
    for mode in [0,1]:
      for sign in [-1,1]:
       for through in [False,True]:
        fresh(58,(2+2*mode)%4);member('thief_localization','yaw_cw_sign',sign)
        setstep(58,68,2,mode,300,action=0 if through else 2)
        if through:
            setstep(68,84,2,mode,600,index=1);record(route,'MapRoute_t',dict(count=2));member('thief_plan','goal_node',84)
        call('Route_Start');tick();rear_valid=1
        gray_raw=0;gray_state=2;yaw=sign*1800;odom=20;clock+=20
        call('Thief_UpdatePose');call('Route_EnterFrontRecovery',clock,2)
        check(var('turn_exit_active'),'actual18degree loss enters heading/exit recovery')
        yaw=sign*700;gray_raw=0xfff;gray_state=3;odom=20;tick()
        check(member('thief_localization','confirmed_node')==58,'early full black cannot fabricate next junction')
        odom=243
        for _ in range(6):
            tick()
            if not var('turn_exit_active'):break
        check(member('thief_localization','confirmed_node')==68 and not var('turn_exit_active'),'actual mapped full-black next junction must hand off instead of scanning until timeout')
        if through:check(var('speed_control_enabled'),'recovery-to-straight-junction handoff re-enables real PI')
        cases.append(finish_and_escape(84 if through else 68,(2+2*mode)%4))
    return cases

def emergency_clear_departure():
    # Isolate fallback policy: mark every edge temporarily blocked, so ordinary
    # search cannot succeed; emergency must still try a clear root before a threat.
    fresh(78,0)
    # Force normal search failure without fabricating a physical sensor reading.
    ret('ThiefIntercept_EdgeBlocked',1)
    check(call('ThiefMapPlanner_PlanEscape',78,0,9,63,clock,plan),'fallback produces complete refuge route')
    heading=at(steps,'MapRouteStep_t','heading')
    check(heading!=0,'with no complete clear-root route, emergency must avoid the actual front threat before relaxing side threats')
    ret('ThiefIntercept_EdgeBlocked',0)
    first=at(steps,'MapRouteStep_t','to_node');goal=member('thief_plan','goal_node')
    call('Route_Start');result=complete_route()
    check(result['node']==goal and result['fixes'][0]==77,'N078 fallback completes the actual selected route via N077')
    return dict(first=first,heading=heading,goal=goal,completion=result)

def scan_junction_brake():
    global clock,yaw,odom,gray_raw,gray_state,rear_valid
    fresh(58,2);setstep(58,68,2,0,300);call('Route_Start');tick();rear_valid=1
    yaw=1800;gray_raw=0;gray_state=2;odom=20;clock+=20
    call('Thief_UpdatePose');call('Route_EnterFrontRecovery',clock,2)
    for _ in range(8):tick()
    check(var('turn_exit_trim_started'),'real missing-line correction actually starts rotation')
    yaw=700
    for _ in range(12):
        tick()
        if var('turn_exit_line_phase')==1:break
    check(var('turn_exit_line_phase')==1,'measured small-error correction enters translation search')
    odom=100;tick();check(var('turn_exit_line_phase')==2,'actual100mm translation starts stationary scan')
    targets.clear()
    for _ in range(8):tick()
    check(targets and any(sum(t)==0 and any(t) for t in targets),'actual scan issued rotation before next junction frame')
    odom=243;gray_raw=0xfff;gray_state=3;ret('Route_AllWheelsStopped',0);targets.clear();tick()
    brake_tick=clock
    check(var('turn_exit_active') and member('thief_localization','confirmed_node')==58 and not targets,'new junction during spin first brakes without committing/translation')
    ret('Route_AllWheelsStopped',1);hook('JY61P_GetLastUpdateMs',lambda:brake_tick);tick()
    check(var('turn_exit_active') and member('thief_localization','confirmed_node')==58,'pre-brake yaw cannot authorize spin-to-junction handoff')
    hook('JY61P_GetLastUpdateMs',lambda:clock);tick()
    check(not var('turn_exit_active') and member('thief_localization','confirmed_node')==68,'stopped wheels and genuinely new yaw allow actual junction handoff')
    return finish_and_escape(68,2)

failures=[];cases={}
for fn in [mismatch_retry,full_black_next_junction,emergency_clear_departure,scan_junction_brake]:
    try:cases[fn.__name__]=fn();print(fn.__name__+': PASS',flush=True)
    except AssertionError as e:failures.append(dict(case=fn.__name__,error=str(e)));print(fn.__name__+': FAIL '+str(e),flush=True)
before='--before' in sys.argv
result=dict(checks=checks,failures=failures,cases=cases,hardware_tested=False,
 firmware_sha256=hashlib.sha256(axf.read_bytes()).hexdigest(),
 app_source_sha256=hashlib.sha256((ROOT/('Tests/before_documents31/User/App_ThiefStrategy.c' if before else 'User/App_ThiefStrategy.c')).read_bytes()).hexdigest(),
 test_script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
 helper_sha256={n:hashlib.sha256((ROOT/'Tests'/n).read_bytes()).hexdigest() for n in ['test_documents15.py','test_app_runtime.py','test_escape_completion.py','test_documents13.py']})
out=ROOT/'Tests'/('before_documents31/regression_before.json' if before else 'results_documents31.json')
out.write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8');print(json.dumps(result,indent=2));sys.exit(bool(failures))
