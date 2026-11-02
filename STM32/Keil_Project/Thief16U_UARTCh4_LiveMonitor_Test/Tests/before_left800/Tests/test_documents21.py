"""RUN12 unified gray recovery and actual route history; synthetic hardware only."""
from pathlib import Path
import sys,json,hashlib
src=Path(__file__).with_name('test_documents15.py').read_text(encoding='utf-8').split('parking_pwm=[]')[0]
src=src.replace("if '--before' in sys.argv:","if False:")
if '--before' in sys.argv:
    src=src.replace('exec(compile(runtime,',"runtime=runtime.replace(\"axf=ROOT/'MDK-ARM'/ROOT.name/(ROOT.name+'.axf')\",\"axf=ROOT/'Tests/before_documents21/original.axf'\")\nexec(compile(runtime,")
exec(compile(src,str(Path(__file__).with_name('test_documents15.py')),'exec'))


def overrun_return():
    global clock,odom,gray_raw,gray_state,rear_valid
    fresh(4,3);rear_valid=1;setstep(4,6,1,1,300)
    record(steps,'MapRouteStep_t',dict(segment_count=2));call('Route_Start');tick()
    # Actual post-brake handler context in Documents21: progress exceeds300.
    odom=485;var('route_state',12);var('replan_after_leg',1)
    var('route_state_tick',clock-100);var('turn_stop_stable_count',2)
    clock+=20;call('Thief_InterceptStopped',clock)
    check(var('route_active') and var('route_state')!=10,'485mm post-brake distance can return along a mapped straight continuation')
    check(call('Route_LegProgress')==(-185)%2**32,'retain signed185mm overrun; never clamp to endpoint')
    tick()
    check(member('thief_localization','confirmed_node')==4,'N006 has never been gray-confirmed')
    check(member('thief_pose','x_mm')==1985,'reverse pose retains actual overrun coordinate')
    odom=185;gray_raw=0xfc0;gray_state=4;tick()
    check(not member('map_nav','arrived') and member('thief_localization','confirmed_node')==4,'crossing unconfirmed N006 cannot become the return endpoint')
    gray_raw=0x60;gray_state=1
    result=complete_route()
    check(4 in result['fixes'] and result['node'] in [11,26,35,38,63],'actual return toN004 then replan completes')
    return result

def known_side_whole_leg():
    global clock,odom
    fresh(38,1);setstep(38,39,1,0,300);call('Route_Start');tick();odom=150
    put(symbols['departure_known_clearance'],2,475)
    call('Thief_UpdatePose');call('Thief_UpdateLaserMask',clock)
    # Isolate the already map-gated world-N observation; no fake gray/pose fix.
    ret('Thief_ThreatMask',1);var('intercept_pending',1)
    gate=symbols['thief_laser']+size(types['FrontLaserGate_t'])
    for _ in range(5):
        record(gate,'FrontLaserGate_t',dict(status=7,distance_mm=354,confidence=100))
        put(symbols['thief_laser_seen']+1,1,1);put(symbols['thief_last_laser_tick']+4,4,clock)
        var('intercept_pending',1);tick()
        check(var('route_state')!=12 and not var('retreat_active'),'same planned side threat cannot flip escape after first100mm')
    record(gate,'FrontLaserGate_t',dict(status=7,distance_mm=150,confidence=100))
    put(symbols['thief_last_laser_tick']+4,4,clock);var('intercept_pending',1);tick()
    check(var('route_state')==12,'substantially closer side target still preempts planned departure')
    if 'Thief_ThreatMask' in hooks:u.hook_del(hooks.pop('Thief_ThreatMask'))
    return 1

def retreat_side_continuity():
    global clock,odom
    fresh(38,1);setstep(38,39,1,0,300);call('Route_Start');tick();odom=120
    var('route_state',12);var('route_state_tick',clock-100);var('turn_stop_stable_count',2)
    clock+=20;call('Thief_InterceptStopped',clock)
    check(var('retreat_active'),'test begins a real partial reversal')
    ret('Thief_ThreatMask',1);initial=var('intercept_continues')
    for _ in range(22):var('intercept_pending',1);tick()
    check(var('route_state')!=12 and var('intercept_continues')==initial,'side evidence during ongoing retreat does not repeatedly truncate or flip motion')
    ret('Thief_ThreatMask',8);var('intercept_pending',1);tick()
    check(var('route_state')==12,'actual travel-front threat still brakes during ongoing retreat')
    if 'Thief_ThreatMask' in hooks:u.hook_del(hooks.pop('Thief_ThreatMask'))
    return 1


def signed_localization_matrix():
    count=0;s0=0x2000c500;r0=0x2000c600
    for node,h in [(4,1),(6,3),(11,1),(12,3),(38,1),(39,3)]:
     for mode in [0,1]:
      for sign in [-1,1]:
       for extra_mm in [1,50,185]:
        fresh(node,(h+2*mode)%4);member('thief_localization','yaw_cw_sign',sign)
        check(call('ThiefMapPlanner_HasRoad',node,h,0x2000c400,0x2000c402),'real road')
        end=get(0x2000c400,2);length=get(0x2000c402,2);parts=1
        while at(call('ThiefMapPlanner_Node',end),'MapRouteNode_t','kind')==0:
            call('ThiefMapPlanner_HasRoad',end,h,0x2000c400,0x2000c402)
            end=get(0x2000c400,2);length+=get(0x2000c402,2);parts+=1
        target=call('ThiefMapPlanner_Node',end);source=call('ThiefMapPlanner_Node',node)
        orig=dict(from_node=node,to_node=end,length_mm=length,segment_count=parts,heading=h,
          expected_ports=at(target,'MapRouteNode_t','degree'),target_kind=at(target,'MapRouteNode_t','kind'),action=2,drive_mode=mode)
        rev=dict(orig,from_node=end,to_node=node,heading=(h+2)%4,drive_mode=mode^1,
          expected_ports=at(source,'MapRouteNode_t','degree'),target_kind=at(source,'MapRouteNode_t','kind'))
        record(s0,'MapRouteStep_t',orig);record(r0,'MapRouteStep_t',rev)
        record(y,'ThiefLocYaw_t',dict(yaw_x100=0,stamp_ms=clock,valid=1))
        check(call('ThiefLocalization_BeginStep',loc,s0,y,clock),'begin actual leg')
        check(call('ThiefLocalization_ReverseLeg',loc,r0,length+extra_mm,y,clock),'bounded straight continuation supports real overrun')
        check(call('ThiefLocalization_Update',loc,(-extra_mm)%2**32,y,clock),'negative entry is a supported measured position')
        ox=at(source,'MapRouteNode_t','x_half')*150;oy=at(source,'MapRouteNode_t','y_half')*150
        expected=(ox+(length+extra_mm)*[0,1,0,-1][h],oy+(length+extra_mm)*[-1,0,1,0][h])
        off,_=fields(variables['thief_localization'])['pose'];pa=loc+off
        check((at(pa,'ThiefMapPose_t','x_mm'),at(pa,'ThiefMapPose_t','y_mm'))==expected,'signed pose matches independent geometry')
        check(member('thief_localization','confirmed_node')==node,'unseen endpoint is never promoted')
        check(call('ThiefLocalization_Update',loc,(-extra_mm+1)%2**32,y,clock),'move one real millimetre back')
        before=(at(pa,'ThiefMapPose_t','x_mm'),at(pa,'ThiefMapPose_t','y_mm'))
        check(call('ThiefLocalization_ReverseLeg',loc,s0,(-extra_mm+1)%2**32,y,clock),'fresh travel-front interception can reverse again inside overrun entry')
        check((at(pa,'ThiefMapPose_t','x_mm'),at(pa,'ThiefMapPose_t','y_mm'))==before,'second reversal retains XY continuity')
        count+=1
    # North of top boundary is not a mapped continuation, so do not invent one.
    fresh(11,0);call('ThiefMapPlanner_HasRoad',11,0,0x2000c400,0x2000c402)
    end=get(0x2000c400,2);length=get(0x2000c402,2)
    record(s0,'MapRouteStep_t',dict(from_node=11,to_node=end,length_mm=length,segment_count=1,heading=0,drive_mode=0,action=2))
    record(r0,'MapRouteStep_t',dict(from_node=end,to_node=11,length_mm=length,segment_count=1,heading=2,drive_mode=1,action=2))
    record(y,'ThiefLocYaw_t',dict(yaw_x100=0,stamp_ms=clock,valid=1))
    check(call('ThiefLocalization_BeginStep',loc,s0,y,clock),'boundary test has valid initial pose')
    # Latest user rule: encoder estimate alone cannot veto returning on the
    # same known road. Preserve the ledger but do not project an invented road.
    check(call('ThiefLocalization_ReverseLeg',loc,r0,length+1,y,clock),'auxiliary out-of-map encoder estimate does not veto a valid return edge')
    off,_=fields(variables['thief_localization'])['pose'];pa=loc+off
    check(at(pa,'ThiefMapPose_t','y_mm')>=0 and member('thief_localization','confirmed_node')==11,'unmapped extension stays clamped and never creates a node fix')
    return count

failures=[];passed=[];cases={}
for fn in [overrun_return,known_side_whole_leg,retreat_side_continuity]+([] if '--before' in sys.argv else [signed_localization_matrix]):
    try:cases[fn.__name__]=fn();passed.append(fn.__name__);print(fn.__name__+': PASS',flush=True)
    except AssertionError as e:failures.append(dict(case=fn.__name__,error=str(e)));print(fn.__name__+': FAIL '+str(e),flush=True)
    finally:
        if 'Thief_ThreatMask' in hooks:u.hook_del(hooks.pop('Thief_ThreatMask'))
before='--before' in sys.argv
result=dict(checks=checks,failures=failures,passed=passed,cases=cases,hardware_tested=False,
 firmware_sha256=hashlib.sha256(axf.read_bytes()).hexdigest(),
 app_source_sha256=hashlib.sha256((ROOT/('Tests/before_documents21/User/App_ThiefStrategy.c' if before else 'User/App_ThiefStrategy.c')).read_bytes()).hexdigest(),
 test_script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
 helper_sha256={n:hashlib.sha256((ROOT/'Tests'/n).read_bytes()).hexdigest() for n in ['test_documents15.py','test_app_runtime.py','test_escape_completion.py','test_documents13.py']})
out=ROOT/'Tests'/('before_documents21/regression_before.json' if before else 'results_documents21.json')
out.write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8');print(json.dumps(result,indent=2));sys.exit(bool(failures))
