"""Linked ARM rules and parking lifecycle; synthetic sensors, no real motors.
--before checks the same new requirements against preserved STABLE3.
"""
from pathlib import Path
import sys
setup=Path(__file__).with_name('test_escape_completion.py').read_text(encoding='utf-8').split('# Log regression:')[0]
before='--before' in sys.argv
if before:
    setup=setup.replace('exec(compile(setup.split(',
        'setup=setup.replace("axf=ROOT/\'MDK-ARM\'/ROOT.name/(ROOT.name+\'.axf\')", "axf=ROOT/\'Reference/STABLE3_before_hide_routes/MDK-ARM\'/ROOT.name/(ROOT.name+\'.axf\')")\nexec(compile(setup.split(',1)
exec(compile(setup,str(Path(__file__).with_name('test_escape_completion.py')),'exec'))
assertions=0;failures=[];lifecycles=[]
fixed=[12,23,31,32,48,49,61,62,63]
def expect(value,message):
    global assertions
    assertions+=1
    if not value:failures.append(message)
def path():
    return [at(steps+i*size(types['MapRouteStep_t']),'MapRouteStep_t','to_node')
            for i in range(at(route,'MapRoute_t','count'))]
def signed(v):return v-0x100000000 if v>=0x80000000 else v
expect(call('ThiefIntercept_HideAxis',38)==5,'N038 parks north/south')

for body in range(4):
    for mask in [1,4,8]:
        fresh(11,body)
        ok=call('ThiefMapPlanner_PlanEscape',11,body,mask,0,clock,plan)
        expect(ok,'N011 requested route exists')
        if ok:
            expect(path()==fixed,f'full corridor retains N023 and N062: body={body} mask={mask} {path()}')
            expect(member('thief_plan','goal_node')==63,'fixed route ends at N063')
            expect(member('thief_plan','final_body_heading')==0,'final desired parking is north, not horizontal arrival')
    for mask in [1,2,8]:
        fresh(38,body)
        ok=call('ThiefMapPlanner_PlanEscape',38,body,mask,0,clock,plan)
        expect(ok,'N038 requested exit exists')
        if ok:
            if mask==1:expect(path()[:2]==[39,40],'north enemy -> N039-N040')
            else:
                expect(path()[0]==21,'either side enemy -> north N021')
                node=call('ThiefMapPlanner_Node',member('thief_plan','goal_node'))
                expect(at(node,'MapRouteNode_t','y_half')<8,'side enemy chooses an upper hide')

# The first hide must also obey the new vertical axis; fixed treasure prefix
# and its observable intermediate junctions remain intact.
fresh(54,3);ok=call('ThiefMapPlanner_PlanInitial',plan)
expect(ok,'initial treasure route still builds')
if ok:
    expect(path()[:10]==[53,44,36,35,34,33,25,15,6,4],'initial treasure prefix unchanged')
    expect(member('thief_plan','goal_node')==38 and member('thief_plan','final_body_heading') in [0,2],
           'first N038 hide is now reached vertically')
    if member('thief_plan','final_body_heading') in [0,2]:
        call('Route_Start');result=complete_route()
        expect(result['node']==38 and result['body'] in [0,2],'actual initial task ends vertically at N038')
        lifecycles.append(dict(kind='initial',result=result))

# Actual laser -> specified complete route -> measured stop -> parking turn.
# Both forward and reverse traversal, both gyro signs, all three trigger sides.
combinations=[(body,sign,0) for body in [1,3] for sign in [-1,1]]+[(1,1,h) for h in [2,3]]
for body,sign,heading in combinations:
    fresh(11,body);member('thief_localization','yaw_cw_sign',sign)
    channel=[(body+d)%4 for d in [0,3,1,2]].index(heading)
    for _ in range(3):
        clock+=20;call('Thief_OnLaserFrame',channel,150,100,1,clock)
    tick()
    expect(path()==fixed,f'actual gate selects exact corridor, body={body} gyro={sign} heading={heading}')
    if path()!=fixed:continue
    result=complete_route()
    expect(result['node']==63 and result['body']==0,'full task completes authorized north parking')
    expect(var('node_centered')==1,'parking occurs after normal center approach')
    expect(not var('park_turn_pending') and not var('route_active'),'parking completes without another travel leg')
    lifecycles.append(dict(kind='fixed',body=body,gyro_sign=sign,world_heading=heading,result=result))

# Boundary fixtures at actual centered N063 arrival, separate from end-to-end
# tests above: prove braking, +/-180 wrap, angular-only output and late heading
# publication. Only test setup seeds arrival state; readiness is never forced.
if 'park_turn_pending' in symbols:
    turn_targets=[]
    def capture_targets():
        turn_targets.append(tuple(signed(u.reg_read(r)) for r in [UC_ARM_REG_R0,UC_ARM_REG_R1,UC_ARM_REG_R2,UC_ARM_REG_R3]))
        return 0
    hook('TravelFrame_SetWheelTargets',capture_targets)
    for body in [1,3]:
      for sign in [-1,1]:
        fresh(62,body);member('thief_localization','yaw_cw_sign',sign)
        setstep(62,63,1,0 if body==1 else 1,600)
        member('thief_plan','goal_node',63);member('thief_plan','final_body_heading',0)
        call('Route_Start');yaw=17000
        var('route_state',13);call('Route_FinishNodeStop',clock,63,body,1)
        expect(var('park_turn_pending')==1 and not var('hide_alignment_pending'),'centered arrival queues special parking only')
        ret('Route_AllWheelsStopped',0)
        for _ in range(3):tick()
        expect(var('park_turn_pending')==1,'cannot rotate before measured standstill')
        ret('Route_AllWheelsStopped',1)
        for _ in range(5):
            if var('park_turn_pending')==2:break
            tick()
        expect(var('park_turn_pending')==2,'standstill starts main parking rotation')
        expect(call('TravelFrame_IsReverse')==0,'parking wheel commands use physical body frame after stopped swap')
        call('Thief_UpdatePose');expect(member('thief_pose','valid')==0,'rotating pose cannot project laser directions')
        start=var('park_start_yaw');start=start-65536 if start>=32768 else start
        quarter=-1 if body==1 else 1
        yaw=(start+sign*quarter*6999+18000)%36000-18000;tick()
        expect(var('park_turn_pending')==2,'ordinary 45-degree cutoff cannot end the parking quarter-turn')
        yaw=(start+sign*quarter*7200+18000)%36000-18000;turn_targets.clear();tick()
        expect(var('park_turn_pending')==3,'continuous wheel finish begins after main rotation')
        expect(turn_targets and all(a+b+c+d==0 and a==c and b==d for a,b,c,d in turn_targets),
               'parking commands rotate in place, never translate')
        expect(var('thief_body_heading')==body,'do not publish north while still rotating')
        old_stamp=clock
        for _ in range(3):
            clock+=1;call('Thief_OnLaserFrame',0,150,100,1,clock)
        expect(not var('thief_escape_requested'),'angled laser samples cannot abort midway and drive off line')
        yaw=(start+sign*quarter*9000+18000)%36000-18000
        ret('Route_AllWheelsStopped',0);tick(100)
        expect(var('park_turn_pending')==3 and var('thief_body_heading')==body,'target yaw alone cannot skip final measured brake')
        ret('Route_AllWheelsStopped',1)
        for _ in range(10):
            if not var('park_turn_pending'):break
            tick()
        expect(not var('park_turn_pending') and var('thief_body_heading')==0,'fresh gyro and standstill complete north parking')
        call('Thief_UpdatePose');call('Thief_UpdateLaserMask',clock)
        call('Thief_OnLaserFrame',0,150,100,1,old_stamp)
        expect(not var('thief_escape_requested'),'queued rotation-era frame rejected after north pose is published')

    # A confirmed real threat while still braked can preempt parking; a
    # rotation already underway completes before directional replanning.
    fresh(62,1);setstep(62,63,1,0,600)
    member('thief_plan','goal_node',63);member('thief_plan','final_body_heading',0)
    call('Route_Start');var('route_state',13);call('Route_FinishNodeStop',clock,63,1,1)
    call('Thief_UpdatePose');call('Thief_UpdateLaserMask',clock)
    for _ in range(3):
        clock+=20;call('Thief_OnLaserFrame',3,150,100,1,clock);call('App_ThiefStrategy_Task')
    for _ in range(5):
        if var('route_active'):break
        tick()
    expect(not var('park_turn_pending') and var('route_active') and member('thief_plan','goal_node')==78,
           'confirmed west threat before rotation takes the established N078 escape')
else:
    expect(False,'old firmware has no authorized 90-degree final parking lifecycle')

report=dict(firmware='UARTCH4_16V_STABLE3' if before else 'UARTCH4_16V_STABLE5',
    checks=assertions,failures=failures,lifecycles=lifecycles,expected_before_failure=before,
    axf_sha256=hashlib.sha256(axf.read_bytes()).hexdigest(),
    test_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),hardware_tested=False)
target=ROOT/('Reference/STABLE3_before_hide_routes/results_hide_routes.json' if before else 'Tests/results_hide_routes_stable4.json')
target.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print(json.dumps(report,ensure_ascii=False,indent=2))
assert bool(failures)==before,failures
