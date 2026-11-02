"""Actual 16U AXF state-machine regression. Synthetic sensor/encoder inputs;
no motors, serial ports or hardware execution. Reuse only emulator setup.
"""
from pathlib import Path
setup = Path(__file__).with_name('test_app_runtime.py').read_text(encoding='utf-8')
exec(compile(setup.split('def documents8_cases():')[0], str(Path(__file__).with_name('test_app_runtime.py')), 'exec'))

for n in ['Key_GetEvent','Key_IsPressed','JY61P_Task','TreasureMotorSnapshot_Probe',
          'TreasureWheelLog_Task','LaserRx_Poll','LaserRx_BootLogTask',
          'StartupStatus_Task','WheelMapLog_Task','Thief_LaserLog','Thief_LocalizationLog']:
    ret(n)
pristine = bytes(u.mem_read(0x20000000, 0x10000))
loc=symbols['thief_localization'];plan=symbols['thief_plan'];nav=symbols['map_nav']
po=fields(variables['thief_plan']);steps=plan+po['steps'][0];route=plan+po['route'][0]
y=0x2000c200
odom=0
hook('MapRoutePosition_DistanceMm',lambda:odom)
def resetcounts():
    global odom
    odom=0
    return 1000
hook('MapRoutePosition_ForwardCounts',resetcounts)

def at(addr,typ,key):
    off,t=fields(types[typ])[key];return get(addr+off,size(t))
def current():return steps+member('map_nav','step')*size(types['MapRouteStep_t'])
def reason():
    addr=get(symbols['route_fault_reason'])
    return bytes(u.mem_read(addr,80)).split(b'\0')[0].decode() if addr else 'NONE'
def fresh(node,body,centered=1):
    global clock,yaw,gray_raw,gray_state,odom
    u.mem_write(0x20000000,pristine);clock=10000;yaw=0;odom=0
    gray_raw=0x60;gray_state=1;events.clear()
    call('ThiefMapPlanner_Init')
    record(y,'ThiefLocYaw_t',dict(yaw_x100=0,stamp_ms=clock,valid=1))
    check(call('ThiefLocalization_Start',loc,node,body,y,clock)==1,'initialize confirmed hide')
    member('thief_localization','yaw_cw_sign',1)
    var('thief_strategy_state',2);var('thief_current_node',node);var('thief_body_heading',body)
    var('thief_treasure_passed',1);var('rear_hw_ready',1);var('node_centered',centered)
    call('Thief_UpdatePose');call('Thief_UpdateLaserMask',clock)

def tick(dt=20):
    global clock
    clock+=dt;call('App_ThiefStrategy_Task')
    check(var('route_state')!=10 and var('thief_strategy_state')!=4,
          f'no latched fault at N{member("thief_localization","confirmed_node"):03}: {reason()}')

def setstep(a,b,heading,mode,length,action=2,index=0):
    end=call('ThiefMapPlanner_Node',b)
    record(steps+index*size(types['MapRouteStep_t']),'MapRouteStep_t',dict(from_node=a,to_node=b,
      length_mm=length,segment_count=1,heading=heading,expected_ports=at(end,'MapRouteNode_t','degree'),
      target_kind=at(end,'MapRouteNode_t','kind'),action=action,drive_mode=mode))
    if index==0:
        record(route,'MapRoute_t',dict(id=1601,count=1,start_node=a,heading=heading,drive_mode=mode))
        member('thief_plan','valid',1);member('thief_plan','goal_node',b)

def complete_route(limit=3500,intercept=None,intercepted_before=False):
    """Feed synthetic physical inputs to the real task until final settled hide.
    No task/nav/pose state is forced. Targets assume ideal yaw/line response.
    Optional one-shot obstacle is injected at 120 mm on a committed first leg.
    """
    global odom,yaw,gray_raw,gray_state,clock
    injected=intercepted_before;states=set();fixes=[];lastfix=member('thief_localization','confirmations')
    for frame in range(limit):
        state=var('route_state');states.add(state)
        if not var('route_active') and var('thief_strategy_state')==2 and not var('replan_after_leg'):
            return dict(frames=frame,node=var('thief_current_node'),body=var('thief_body_heading'),
                        fixes=fixes,states=sorted(states),intercepted=injected)
        gray_raw=0x60;gray_state=1
        step=current();phase=member('thief_localization','phase')
        # Integrate the actual new parking rotation command. The old toy model
        # only changed yaw during route turns and could not respond at a hide.
        if 'hide_alignment_pending' in symbols and var('hide_alignment_pending') and member('hide_heading','spinning'):
            turn=member('hide_heading','last_turn');turn=turn-65536 if turn>=32768 else turn
            sign=member('thief_localization','yaw_cw_sign');sign=sign-256 if sign>=128 else sign
            yaw=(yaw+sign*round(turn/5)+18000)%36000-18000
        if phase==2 and state in [5,6,7,2]:
            nxt=step if var('departure_turn') else step+size(types['MapRouteStep_t'])
            targetbody=(at(nxt,'MapRouteStep_t','heading')+2*at(nxt,'MapRouteStep_t','drive_mode'))%4
            sign=member('thief_localization','yaw_cw_sign');sign=sign-256 if sign>=128 else sign
            target=(targetbody-member('thief_localization','origin_heading'))*9000*(sign or 1)
            diff=(target-yaw+18000)%36000-18000
            yaw=(yaw+max(-600,min(600,diff))+18000)%36000-18000
        if phase==1 and state in [1,2] and not member('map_nav','arrived'):
            length=at(step,'MapRouteStep_t','length_mm')
            odom=min(odom+10,length-var('map_progress_offset_mm'))
            progress=call('Route_LegProgress')
            if intercept is not None and not injected and progress>=120 and progress<length-30:
                call('Thief_UpdatePose');call('Thief_UpdateLaserMask',clock)
                clock+=1
                # Direction is relative to the physical body, not travel frame.
                call('Thief_OnLaserFrame',intercept,150,100,1,clock)
                if var('intercept_pending'):
                    injected=True
            if progress>=length and var('junction_armed'):
                gray_raw=0xfff;gray_state=3
        tick()
        count=member('thief_localization','confirmations')
        if count!=lastfix:fixes.append(member('thief_localization','confirmed_node'));lastfix=count
    raise AssertionError(f'completion timeout state={var("route_state")} node={member("thief_localization","confirmed_node")}')

# Log regression: N063 -> N078, one-sided DF4 evidence, then west departure.
# Run through actual arrival producer and root turn; RUN1 must fail here.
fresh(63,2,0)
setstep(63,78,2,0,600);call('Route_Start');var('replan_after_leg',1)
odom=600;clock+=600;call('Route_HandleJunction',clock,20,4,0xdf4)
check(var('branch_node')==78 and var('branch_headings')==2,'Documents11 retains only east branch')
tick(40);tick(60)
setstep(78,77,3,0,300);gray_raw=0x40;gray_state=1
call('Route_Start')
check(var('departure_turn')==1 and member('map_nav','seen_ports')==1,'Documents11 west exit with east-only evidence')
tick(180);tick(20);tick(60)
check(var('route_state')==5 and var('route_active')==1,'Documents11 right turn must start')
log_result=complete_route()
check(log_result['node']==77,'Documents11 reaches N077 after the formerly blocked turn')
print('Documents11 root-turn regression passed',flush=True)
if '--documents11-only' in sys.argv:
    print(json.dumps(dict(checks=checks,failures=0,log_regression=log_result)))
    sys.exit(0)

results=[];ignored=[]
for node in [11,38,26,35,63]:
    for body in ([1,3] if node in [11,38,35] else [0,2]):
        for direction,label in enumerate(['FRONT','LEFT','RIGHT','REAR']):
            fresh(node,body)
            for _ in range(3):
                clock+=20;call('Thief_OnLaserFrame',direction,150,100,1,clock)
            if not var('thief_escape_requested'):
                # A nonexistent road/off-map ray is intentionally not an opponent.
                sensor=(body+[0,3,1,2][direction])%4
                check(not (call('MapLaserMask_DetectionMask',symbols['thief_pose'])&(1<<direction)),
                      f'valid in-map direction must trigger {node=} {body=} {label=}')
                ignored.append(dict(node=node,body=body,laser=label,reason='outward map-edge ray'))
                continue
            tick();goal=member('thief_plan','goal_node')
            check(var('route_active') and goal!=node,'laser starts a route to another refuge')
            result=complete_route()
            check(result['node']==goal and result['node'] in [11,38,26,35,63], 'arrives at chosen refuge')
            check((result['body']%2)==(1 if goal in [11,38,35] else 0),'destination hide axis')
            results.append(dict(start=node,start_body=body,laser=label,goal=goal,**result))
            print(f'escape N{node:03} body={body} {label} -> N{goal:03} passed',flush=True)

# End-to-end one-shot obstacle while already escaping, all body-relative lasers.
interceptions=[]
for direction,label in [(0,'FRONT'),(3,'REAR')]:
    fresh(38,1)
    for _ in range(3):
        clock+=20;call('Thief_OnLaserFrame',3,150,100,1,clock)
    tick()
    result=complete_route(intercept=direction)
    check(result['intercepted'],'partial-leg laser must actually trigger')
    check(result['node'] in [11,38,26,35,63],'intercepted escape completes at a permitted hide')
    interceptions.append(dict(laser=label,**result))

# Side rays only project onto a road while still at the cross junction.
# Trigger after Route_Start but before translation: exercise real cancellation,
# settle, replan, turn and complete route, rather than forcing a threat mask.
for direction,label in [(1,'LEFT'),(2,'RIGHT')]:
    fresh(38,1)
    for _ in range(3):
        clock+=20;call('Thief_OnLaserFrame',3,150,100,1,clock)
    tick()
    call('Thief_UpdatePose');call('Thief_UpdateLaserMask',clock)
    clock+=1;call('Thief_OnLaserFrame',direction,150,100,1,clock)
    check(var('intercept_pending')==1,'side threat at departure crosses the real laser gate')
    tick()
    check(var('route_state')==12,'side threat immediately brakes departure')
    result=complete_route(intercepted_before=True)
    check(result['node'] in [11,38,26,35,63],'side interception replans and completes at a hide')
    interceptions.append(dict(laser=label,trigger='departure before translation',**result))

# Map authority is restricted to a genuinely confirmed, correctly located root.
# Wrong node / midpoint and a nonexistent edge remain rejected before motion.
for invalid,expected in [('wrong_node','ROOT_TURN_POSE'),('on_leg','ROOT_TURN_POSE'),
                         ('invalid_edge','MAP_RUNTIME_PLAN_INVALID')]:
    fresh(78,2,0);setstep(78,77,3,0,300)
    if invalid=='wrong_node':member('thief_localization','confirmed_node',79)
    elif invalid=='on_leg':member('thief_localization','phase',1)
    else:record(steps,'MapRouteStep_t',dict(to_node=62))
    call('Route_Start')
    check(var('route_active')==0 and reason()==expected,'invalid root remains rejected: '+invalid)

result=dict(checks=checks,failures=0,log_regression=log_result,escapes=results,ignored=ignored,
    interceptions=interceptions,scope='Linked ARM firmware task + real planner, synthetic gray/yaw/encoder; no physical motion proof',
    firmware_sha256=hashlib.sha256(axf.read_bytes()).hexdigest(),
    app_source_sha256=hashlib.sha256((ROOT/'User/App_ThiefStrategy.c').read_bytes()).hexdigest(),
    test_script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
(ROOT/'Tests/results_escape_completion.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
print(json.dumps(dict(checks=checks,escapes=len(results),ignored=len(ignored),interceptions=interceptions),indent=2))
