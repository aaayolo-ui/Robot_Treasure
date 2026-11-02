"""Actual linked firmware, synthetic sensors; N063 -> N078 wait -> split.
--before runs the same rule expectations against preserved STABLE2.
"""
from pathlib import Path
import sys
setup=Path(__file__).with_name('test_escape_completion.py').read_text(encoding='utf-8').split('# Log regression:')[0]
before='--before' in sys.argv
if before:
    setup=setup.replace('exec(compile(setup.split(',
        'setup=setup.replace("axf=ROOT/\'MDK-ARM\'/ROOT.name/(ROOT.name+\'.axf\')", "axf=ROOT/\'Reference/STABLE2_before_temp78/MDK-ARM\'/ROOT.name/(ROOT.name+\'.axf\')")\nexec(compile(setup.split(',1)
exec(compile(setup,str(Path(__file__).with_name('test_escape_completion.py')),'exec'))
assertions=0;failures=[];lifecycles=[]
def expect(v,msg):
    global assertions
    assertions+=1
    if not v:failures.append(msg)
def path():
    return [at(steps+i*size(types['MapRouteStep_t']),'MapRouteStep_t','to_node')
            for i in range(at(route,'MapRoute_t','count'))]
def unhook(n):
    if n in hooks:u.hook_del(hooks.pop(n))

for body in range(4):
    fresh(63,body)
    ok=call('ThiefMapPlanner_PlanEscape',63,body,8,0,clock,plan)
    expect(ok,'N063 west threat has a route')
    if ok:
        expect(path()==[78] and member('thief_plan','goal_node')==78,'N063 west -> only N078, no automatic next leg')
        expect(member('thief_plan','final_body_heading') in [0,2],'temporary parking is vertical')
        if body==0:expect(at(route,'MapRoute_t','drive_mode')==1,'north body reverses south without turning chassis')
expect(call('ThiefRiskSearch_HideSafe',78)==0,'N078 is not a generic sixth refuge')
expect(call('ThiefIntercept_HideAxis',78)==5,'N078 uses normal N/S parking centering')

for body in [0,2]:
    for mask,xy,prefix in [(1,(2700,1800),[77,76,75]),(8,(2100,2400),[63,52])]:
        fresh(78,body)
        call('ThiefMapPlanner_ObserveThreat',78,mask.bit_length()-1,*xy,clock)
        ok=call('ThiefMapPlanner_PlanEscape',78,body,mask,63,clock,plan)
        expect(ok,f'N078 departure body={body} mask={mask}')
        if ok:
            expect(path()[:len(prefix)]==prefix,f'requested corridor, not just first compass bit: {path()}')
            expect(member('thief_plan','goal_node') in [11,26,35,38,63],'departure ends at one of five original hides')
            expect(member('thief_plan','goal_node')!=75,'N075 is a waypoint, not a waiting hide')
            if mask==8:
                goal=call('ThiefMapPlanner_Node',member('thief_plan','goal_node'))
                expect(at(goal,'MapRouteNode_t','y_half')<12,'west threat continues to hide above N063, not stop at N063')

# Exact false-side cancellation from the 21:22 log: enemy near N061 is
# 600mm above the next N078-N077 road. An actual fresh located hit is supplied.
for hit,blocked in [((1800,1800),False),((2400,2400),True)]:
    fresh(63,0)
    setstep(63,78,2,1,600,action=1);setstep(78,77,3,1,300,index=1)
    record(route,'MapRoute_t',dict(count=2));call('Route_Start')
    g=symbols['thief_laser']+size(types['FrontLaserGate_t'])
    record(g,'FrontLaserGate_t',dict(status=7,hit_x_mm=hit[0],hit_y_mm=hit[1]))
    put(symbols['thief_laser_seen']+1,1,1);put(symbols['thief_last_laser_tick']+4,4,clock)
    ret('Thief_ThreatMask',8);var('intercept_pending',1);call('Thief_InterceptReact',clock)
    expect(bool(var('replan_after_leg'))==blocked,f'next-road geometry hit={hit}, blocked={blocked}')
    expect(at(route,'MapRoute_t','count')==(1 if blocked else 2),'only a hit on next road truncates the route')
unhook('Thief_ThreatMask')

# End-to-end: actual gate at N063, normal reverse line follow, real gray
# arrival, 180ms center, settle, quiet wait, and NEW gate at N078.
for threat_heading,expected_prefix in [(0,[77,76,75]),(3,[63,52])]:
    fresh(63,0)
    for _ in range(3):
        clock+=20;call('Thief_OnLaserFrame',1,797,100,1,clock)
    old_tick=clock;tick()
    expect(member('thief_plan','goal_node')==78,'actual west laser chooses temporary N078')
    if member('thief_plan','goal_node')!=78:continue
    arrived=complete_route()
    expect(arrived['node']==78 and arrived['body'] in [0,2],'actual task reaches and vertically settles at N078')
    expect(var('node_centered')==1,'temporary wait uses normal 180ms centering')
    for _ in range(60):tick(100)
    expect(not var('route_active') and var('thief_current_node')==78,'quiet N078 waits over 6 seconds, no timed return')
    expect(not var('replan_after_leg') and not var('intercept_pending'),'old replan latch cleared at temporary wait')
    expect(call('ThiefThreatModel_Prepare',clock)==0,'old N063 threat model cleared at N078')
    call('Thief_OnLaserFrame',1,797,100,1,old_tick);tick()
    expect(not var('route_active'),'queued old-position frame cannot restart escape')
    body=var('thief_body_heading')
    channel=[(body+d)%4 for d in [0,3,1,2]].index(threat_heading)
    for _ in range(2):
        clock+=20;call('Thief_OnLaserFrame',channel,450,100,1,clock);tick()
        expect(not var('route_active'),'new wait threat still requires stable multi-frame confirmation')
    clock+=20;call('Thief_OnLaserFrame',channel,450,100,1,clock);tick()
    expect(var('route_active'),'fresh confirmed threat at N078 starts escape')
    expect(path()[:len(expected_prefix)]==expected_prefix,'actual task chooses requested corridor')
    planned=member('thief_plan','goal_node')
    completed=complete_route()
    expect(completed['node']==planned,'full actual task completes post-wait transfer')
    if threat_heading==3:expect(completed['node'] in [11,26,35,38],'west at N078 ends in upper hide')
    lifecycles.append(dict(threat_heading=threat_heading,wait=arrived,escape=completed))

report=dict(firmware='UARTCH4_16V_STABLE2' if before else 'UARTCH4_16U_ROUTE1_RUN8',
    checks=assertions,failures=failures,lifecycles=lifecycles,expected_before_failure=before,
    axf_sha256=hashlib.sha256(axf.read_bytes()).hexdigest(),
    test_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),hardware_tested=False)
target=ROOT/('Reference/STABLE2_before_temp78/results_temp78.json' if before else 'Tests/results_temp78_stable3.json')
target.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print(json.dumps(report,ensure_ascii=False,indent=2))
assert bool(failures)==before,failures
