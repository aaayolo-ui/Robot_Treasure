"""Actual linked ARM trail + full replay, synthetic motion/sensors only."""
from pathlib import Path
import json,hashlib
setup=Path(__file__).with_name('test_escape_completion.py').read_text(encoding='utf-8').split('# Log regression:')[0]
# This test's bounded yaw model must account for nonzero map-yaw origins.
setup=setup.replace("target=(targetbody-member('thief_localization','origin_heading'))*9000*(sign or 1)","target=member('thief_localization','origin_yaw_x100')+(targetbody-member('thief_localization','origin_heading'))*9000*(sign or 1)")
exec(compile(setup,str(Path(__file__).with_name('test_escape_completion.py')),'exec'))
trail=symbols['route_trail'];entrysize=size(types['ThiefTrailEntry']);results=[]
def tm(n):return at(trail,'ThiefRouteTrail',n)
def entry(i,n):return at(trail+fields(types['ThiefRouteTrail'])['entries'][0]+i*entrysize,'ThiefTrailEntry',n)
def path():return [at(steps+i*size(types['MapRouteStep_t']),'MapRouteStep_t','to_node') for i in range(at(route,'MapRoute_t','count'))]

def gate_channel(world):return [(var('route_body_heading')+d)%4 for d in [0,3,1,2]].index(world)

# A staged turn is not a completed edge; failed/white exit must not commit it.
fresh(11,1);call('ThiefRouteTrail_Begin',trail,11,1)
setstep(11,12,1,0,300,action=1)
call('ThiefRouteTrail_Stage',trail,steps,0xC60,0x063,17990,287)
check(tm('count')==0 and tm('pending_valid')==1,'candidate gray arrival alone cannot enter executed history')
check(entry(0,'from')==0,'uncommitted entry remains absent')
check(call('ThiefRouteTrail_Commit',trail)==1 and tm('count')==1,'exit confirmation commits staged edge once')
check(call('ThiefRouteTrail_Commit',trail)==0 and tm('count')==1,'duplicate commit cannot add a road twice')
check(entry(0,'leading_raw')==0xC60 and entry(0,'trailing_raw')==0x063 and entry(0,'measured_mm')==287,'actual sensor/translation evidence retained')
check(entry(0,'yaw_x100')==17990,'wrapped gyro evidence retained')

# Capacity is bounded; reserve the last runtime step for the partial return.
fresh(11,1);call('ThiefRouteTrail_Begin',trail,11,1)
for i in range(47):
    a,b,h,mode=(11,12,1,0) if i%2==0 else (12,11,3,1)
    setstep(a,b,h,mode,300)
    check(call('ThiefRouteTrail_Stage',trail,steps,0xfff,0x60,0,300)==1,'bounded stage')
    check(call('ThiefRouteTrail_Commit',trail)==1,'bounded commit')
setstep(12,11,3,1,300)
# Copy partial input: production does this after measuring stop displacement.
scratch=0x2000c900;u.mem_write(scratch,bytes(u.mem_read(steps,size(types['MapRouteStep_t']))))
check(call('ThiefRouteTrail_BuildReturn',trail,scratch,plan)==1,'47 actual edges + one partial return fit exactly')
check(at(route,'MapRoute_t','count')==48 and path()[-1]==11,'bounded reverse plan ends at remembered home')
call('MapRouteNavigator_StartRuntime',nav,route,steps)
check(not member('map_nav','fault'),'return plan passes actual map connectivity, headings and mode checks')

ordinary_tick=tick
for home,body,sign,world,step_index,hit_mm,origin in [
    (11,1,1,3,0,10,0),(11,3,-1,3,0,44,17950),
    (11,1,1,3,1,120,0),(11,3,-1,3,2,44,0),
    (11,1,-1,3,3,120,17950),(11,3,1,3,6,120,0),
    (38,0,1,0,0,44,0),(38,2,-1,0,1,120,17950),
    (63,0,-1,3,0,44,0),(63,2,1,0,2,120,0),
    (35,1,1,0,0,44,0),(26,0,-1,1,1,120,0),
    (35,1,1,None,0,44,0),(35,3,-1,None,0,44,17950),
    (26,0,1,None,0,44,0),(26,2,-1,None,0,44,17950)]:
    fresh(home,body);member('thief_localization','yaw_cw_sign',sign)
    member('thief_localization','origin_yaw_x100',origin);yaw=origin
    if world is None:
        if home==35:setstep(35,26,0,0,300)
        else:setstep(26,27,1,0,300)
        call('Route_Start')
    else:
        ch=[(body+d)%4 for d in [0,3,1,2]].index(world)
        for _ in range(3):clock+=20;call('Thief_OnLaserFrame',ch,150,100,1,clock)
    tick();check(var('route_active'),'start actual home escape')
    # Some angularly constrained hides have short plans; intercept their last leg.
    wanted=min(step_index,at(route,'MapRoute_t','count')-1)
    injected=False;captured=False;expected=[];return_nodes=[];last_step=-1;previous_count=0
    def trace_tick(dt=20):
        global clock,injected,captured,expected,last_step,previous_count
        if var('route_active') and not tm('returning') and not injected and member('map_nav','step')==wanted and var('route_state') in [1,2] and not member('map_nav','arrived') and call('Route_LegProgress')>=hit_mm:
            check(tm('enabled') and not tm('pending_valid'),'history committed only through completed real exits')
            expected=[at(current(),'MapRouteStep_t','from_node')]+[entry(i,'from') for i in reversed(range(tm('count')))]
            call('Thief_UpdatePose');call('Thief_UpdateLaserMask',clock)
            front=gate_channel(at(current(),'MapRouteStep_t','heading'))
            for i in range(3):
                clock+=20;call('Thief_OnLaserFrame',front,150,100,1,clock)
                if i<2:check(not var('intercept_pending'),'moving front also requires consecutive confirmation')
            call('Thief_InterceptReact',clock)
            check(var('route_state')==12,'confirmed actual travel-front hit brakes')
            injected=True
        ordinary_tick(dt)
        if tm('returning'):
            if not captured:
                check(path()==expected,'return follows actual executed history, excludes future planned nodes')
                check(member('thief_plan','goal_node')==home,'return target is original settled home')
                previous_count=tm('count');captured=True
            check(tm('count')<=previous_count,'return history cannot grow while replaying')
            previous_count=tm('count')
            idx=member('map_nav','step')
            if idx!=last_step:return_nodes.append(at(current(),'MapRouteStep_t','to_node'));last_step=idx
            # The original pursuer is now behind travel: it must not cause oscillation.
            for ch in range(4):
                w=(var('route_body_heading')+[0,3,1,2][ch])%4
                if w!=at(current(),'MapRouteStep_t','heading'):call('Thief_OnLaserFrame',ch,150,100,1,clock)
            check(not var('intercept_pending'),'rear/side pursuit cannot cancel replay')
    tick=trace_tick
    result=complete_route(limit=6000)
    tick=ordinary_tick
    check(injected and captured,'actual mid-leg interception enters replay')
    check(result['node']==home and result['body']%2==body%2,'returns and restores home parking axis')
    check(not tm('returning') and tm('return_left')==0 and not tm('enabled'),'home consumes every returned edge and releases replay')
    check(not var('replan_after_leg') and not var('n011_escape_locked') and not var('thief_escape_requested'),'old threat cannot auto-launch from returned home')
    for _ in range(20):tick()
    check(not var('route_active'),'quiet returned home remains waiting')
    results.append(dict(home=home,body=body,sign=sign,requested_step=wanted,hit_mm=hit_mm,origin=origin,expected=expected,visited=return_nodes,result=result))
report=dict(checks=checks,failures=[],lifecycles=results,trace_bytes=size(types['ThiefRouteTrail']),axf_sha256=hashlib.sha256(axf.read_bytes()).hexdigest(),hardware_tested=False)
(ROOT/'Tests/results_trace_return.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
print(json.dumps(report,indent=2))
