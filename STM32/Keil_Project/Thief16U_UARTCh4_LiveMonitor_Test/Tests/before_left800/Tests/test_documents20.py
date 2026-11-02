"""RUN12 unified gray recovery and actual route history; synthetic hardware only."""
from pathlib import Path
import sys,json,hashlib
src=Path(__file__).with_name('test_documents15.py').read_text(encoding='utf-8').split('parking_pwm=[]')[0]
src=src.replace("if '--before' in sys.argv:","if False:")
if '--before' in sys.argv:
    src=src.replace('exec(compile(runtime,',"runtime=runtime.replace(\"axf=ROOT/'MDK-ARM'/ROOT.name/(ROOT.name+'.axf')\",\"axf=ROOT/'Tests/before_documents20/original.axf'\")\nexec(compile(runtime,")
exec(compile(src,str(Path(__file__).with_name('test_documents15.py')),'exec'))

def signed16(v):return v-65536 if v>=32768 else v

def straight_exit_gap():
    global odom,clock,gray_raw,gray_state,rear_valid,rear_error,yaw
    count=0
    for mode in [0,1]:
     for sign in [-1,1]:
      for skew in [0,-1152,1152]:
        fresh(36,(2+2*mode)%4);rear_valid=1;rear_error=-40
        member('thief_localization','yaw_cw_sign',sign)
        setstep(36,44,2,mode,300,action=0);setstep(44,53,2,mode,300,index=1)
        record(route,'MapRoute_t',dict(count=2));call('Route_Start');tick()
        odom=300;clock+=300;yaw=skew;call('Route_HandleJunction',clock,20,4,0xfc0)
        check(var('route_state')==11 and member('thief_localization','confirmed_node')==44,'replay actual gray-confirmed N044 pass')
        gray_raw=0;gray_state=2;targets.clear();events.clear()
        for _ in range(24):
            if 'turn_exit_heading' in symbols and var('turn_exit_active') and member('turn_exit_heading','spinning'):
                yaw=(yaw+sign*round(signed16(member('turn_exit_heading','last_turn'))/5)+18000)%36000-18000
            tick()
        check(var('route_active') and member('map_nav','step')==0,'gap longer than200ms neither locks nor fabricates next leg')
        check(targets,'actual trailing line or heading correction drives controlled recovery')
        gray_raw=0x60;gray_state=1
        for _ in range(12):tick()
        check(member('map_nav','step')==1 and member('thief_localization','phase')==1,'actual leading line + observed yaw commit exit')
        count+=1
    return count

def middle_both_missing():
    global odom,clock,gray_raw,gray_state,rear_valid,yaw
    fresh(38,1);rear_valid=1;setstep(38,39,1,0,300);call('Route_Start');tick()
    odom=150;gray_raw=0;gray_state=2;rear_valid=2;targets.clear();events.clear()
    tick();check('Route_BrakeControl' in events and not targets,'two missing boards first brake in road middle')
    for _ in range(5):tick()
    check(targets and all(sum(t)==0 for t in targets),'mapped middle-road loss scans after measured stop, no blind translation')
    gray_raw=0x60;gray_state=1;rear_valid=1
    for _ in range(9):tick()
    check(var('route_state')==2,'actual line ends scan')
    return 1

def history_available():
    check('RouteHistory_Record' in symbols and 'RouteHistory_PlanReturn' in symbols,'firmware records completed route and can build inverse route')
    return 1


def complete_history_return():
    global clock,odom,gray_raw,gray_state,rear_valid
    fresh(37,2);rear_valid=1
    setstep(37,55,2,0,600,action=-1);setstep(55,56,1,0,300,index=1)
    record(route,'MapRoute_t',dict(count=2));call('Route_Start')
    check(call('RouteHistory_Count')==0,'planned future steps must not become history')
    outbound=complete_route();check(outbound['node']==56,'real outbound reaches N056')
    check(call('RouteHistory_Count')==2 and call('RouteHistory_LastNode')==56,'record two real completed legs across a turn')
    check(call('RouteHistory_PlanReturn',56,1,8,clock,plan)==0,'known threat on inverse first road forbids history return')
    check(call('RouteHistory_Count')==2,'a plan or rejected plan must not consume actual history')
    call('ThiefIntercept_BlockLeg',56,55,clock)
    check(call('RouteHistory_PlanReturn',56,1,0,clock,plan)==0,'newly blocked return edge overrides old route memory')
    call('ThiefIntercept_Reset')
    check(call('RouteHistory_PlanReturn',56,1,0,clock,plan)==1,'inverse route exists')
    check(at(route,'MapRoute_t','count')==2 and member('thief_plan','goal_node')==37,'history returns through corner to earlier branch')
    check(at(steps,'MapRouteStep_t','drive_mode')==1,'rear becomes leading board')
    check(at(steps,'MapRouteStep_t','to_node')==55 and at(steps,'MapRouteStep_t','action')==1,'actual outbound left turn is inverted to right at N055')
    call('Route_Start');back=complete_route()
    check(back['node']==37 and back['fixes']==[55,37],'real gray arrivals, yaw and reversed turns return to N037')
    check(call('RouteHistory_Count')==0,'consume history only on real inverse arrivals')
    return dict(outbound=outbound,returned=back)

def bounded_history():
    global clock
    fresh(38,1);tmp=0x2000c600
    # Discontinuous entries cannot pretend to be a contiguous return path.
    record(tmp,'MapRouteStep_t',dict(from_node=38,to_node=39,length_mm=300,heading=1,drive_mode=0))
    call('RouteHistory_Record',tmp,0,0);check(call('RouteHistory_Count')==0,'missing gray cannot create history')
    for i in range(40):
        # A contiguous cycle of real map edges, not an expanding fake node list.
        a,b,h=[(38,39,1),(39,57,2),(57,56,3),(56,55,3),(55,37,0),(37,38,1)][i%6]
        check(call('ThiefMapPlanner_HasRoad',a,h,0x2000c400,0x2000c402) and get(0x2000c400,2)==b,'capacity fixture follows actual map edges')
        record(tmp,'MapRouteStep_t',dict(from_node=a,to_node=b,length_mm=get(0x2000c402,2),heading=h,drive_mode=0))
        call('RouteHistory_Record',tmp,0xfff,0)
        check(call('RouteHistory_Count')<=32,'recent-history capacity never locks vehicle')
    check(call('RouteHistory_Count')==32,'drop oldest entry at capacity')
    return 40

def startup_and_road_scopes():
    global clock,odom,gray_raw,gray_state,rear_valid,rear_error,yaw
    count=0
    for state in [1,2,7]:
     for mode in [0,1]:
      for sign in [-1,1]:
        fresh(38,1);rear_valid=1;rear_error=70
        heading=3 if mode else 1;goal=37 if mode else 39
        setstep(38,goal,heading,mode,300);call('Route_Start');tick()
        var('route_state',state);var('departure_reacquire',0);member('thief_localization','yaw_cw_sign',sign)
        odom=20;gray_raw=0;gray_state=2;targets.clear();events.clear()
        for _ in range(15):odom+=1;tick()
        check(targets and 'Route_BrakeControl' not in events,'single board absence uses mapped rear+yaw without repeated brake, state='+str(state))
        gray_raw=0x60;gray_state=1
        for _ in range(4):tick()
        check(var('route_state') in [1,2,7],'actual line restores travel phase')
        count+=1
    return count

def gray_failure_backtrack(cause="gray"):
    global clock,odom,gray_raw,gray_state,rear_valid,rear_error
    # Build genuine history first; then lose leading line on the next leg.
    fresh(37,2);rear_valid=1;setstep(37,55,2,0,600);call('Route_Start')
    check(complete_route()['node']==55,'history fallback fixture reaches observed N055')
    check(call('RouteHistory_Count')==1,'real incoming leg is available')
    setstep(55,56,1,0,300);call('Route_Start')
    # Root turn is completed by the real task/model before we inject the gap.
    for _ in range(100):
        if member('thief_localization','phase')==1:break
        # Same physical yaw response used by the existing route completion model.
        global yaw
        if var('route_state') in [5,6,7,2]:
            target=-9000
            diff=(target-yaw+18000)%36000-18000;yaw=(yaw+max(-600,min(600,diff))+18000)%36000-18000
        tick()
    check(member('thief_localization','phase')==1,'outgoing east leg committed before loss')
    # Continue observed yaw convergence to the road axis. The moving laser
    # geometry intentionally rejects a 6-degree residual (its existing limit is5).
    while yaw!=-9000:
        diff=(-9000-yaw+18000)%36000-18000
        yaw=(yaw+max(-600,min(600,diff))+18000)%36000-18000
        tick()
    odom=100;rear_valid=1;rear_error=0
    if cause=="gray":
        gray_raw=0;gray_state=2;tick()
        check(var('map_gray_active'),'on-leg trailing-supported recovery begins')
        odom=181;call('App_ThiefStrategy_Task')  #80mm search budget may request known-history return; fixed1s is no longer a fault.
    else:
        call('Thief_UpdatePose');call('Thief_UpdateLaserMask',clock)
        clock+=1;call('Thief_OnLaserFrame',0,150,100,1,clock);tick()
    check(var('route_state')==12 and var('route_active'),'bounded failed search requests measured-stop return rather than permanent fault')
    check(member('thief_localization','confirmed_node')==55,'return request never fabricates a node')
    check(not call('TravelFrame_IsReverse'),'must not reverse before measured brake')
    gray_raw=0x60;gray_state=1
    ret('Route_AllWheelsStopped',0)
    for _ in range(3):tick()
    check(not call('TravelFrame_IsReverse'),'moving wheels hold reverse request')
    ret('Route_AllWheelsStopped',1)
    for _ in range(4):tick()
    check(call('TravelFrame_IsReverse')==1 and var('retreat_active'),'after actual stop use old line backward')
    back=complete_route()
    check(55 in back['fixes'],'gray confirms original node during backtrack')
    check(back['node'] in [11,26,35,38,63],'return then history/corridor handling reaches an allowed refuge')
    return back

def obstacle_history_return():
    return gray_failure_backtrack("laser")

failures=[];passed=[];cases={}
for fn in [straight_exit_gap,middle_both_missing,history_available]+([] if '--before' in sys.argv else [complete_history_return,bounded_history,startup_and_road_scopes,gray_failure_backtrack,obstacle_history_return]):
    try:cases[fn.__name__]=fn();passed.append(fn.__name__);print(fn.__name__+': PASS',flush=True)
    except AssertionError as e:failures.append(dict(case=fn.__name__,error=str(e)));print(fn.__name__+': FAIL '+str(e),flush=True)
before='--before' in sys.argv
result=dict(checks=checks,failures=failures,passed=passed,cases=cases,hardware_tested=False,
 firmware_sha256=hashlib.sha256(axf.read_bytes()).hexdigest(),
 app_source_sha256=hashlib.sha256((ROOT/('Tests/before_documents20/User/App_ThiefStrategy.c' if before else 'User/App_ThiefStrategy.c')).read_bytes()).hexdigest(),
 test_script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
 helper_sha256={n:hashlib.sha256((ROOT/'Tests'/n).read_bytes()).hexdigest() for n in ['test_documents15.py','test_app_runtime.py','test_escape_completion.py','test_documents13.py']})
out=ROOT/'Tests'/('before_documents20/regression_before.json' if before else 'results_documents20.json')
out.write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8');print(json.dumps(result,indent=2));sys.exit(bool(failures))
