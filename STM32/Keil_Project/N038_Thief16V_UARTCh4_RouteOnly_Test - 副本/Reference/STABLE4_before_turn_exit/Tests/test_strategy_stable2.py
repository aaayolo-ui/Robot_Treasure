"""Linked ARM strategy regression; synthetic world masks, no physical motors.

--before runs identical expectations against preserved STABLE1 and records
the actual regression failures. Mask direction is always world N/E/S/W.
"""
from pathlib import Path
import sys
setup = Path(__file__).with_name('test_escape_completion.py').read_text(encoding='utf-8').split('# Log regression:')[0]
before = '--before' in sys.argv
if before:
    setup = setup.replace("exec(compile(setup.split(",
        "setup=setup.replace(\"axf=ROOT/'MDK-ARM'/ROOT.name/(ROOT.name+'.axf')\", \"axf=ROOT/'Reference/STABLE1_before_route_fix/MDK-ARM'/ROOT.name/(ROOT.name+'.axf')\")\nexec(compile(setup.split(", 1)
exec(compile(setup, str(Path(__file__).with_name('test_escape_completion.py')), 'exec'))

assertions = 0
failures = []
def expect(value, message):
    global assertions
    assertions += 1
    if not value: failures.append(message)

def nodes_in_plan():
    return [at(steps+i*size(types['MapRouteStep_t']), 'MapRouteStep_t', 'to_node')
            for i in range(at(route, 'MapRoute_t', 'count'))]

# Pure policy: side/rear never reverse; front cannot be waived by cooldown.
for h in range(4):
    for mask in range(16):
        for cooldown in [0,1]:
            got = call('ThiefIntercept_ShouldReverse', h, mask, 0, cooldown)
            expect(got == bool(mask & (1 << h)), f'reversal travel={h} mask={mask} cooldown={cooldown}')

# N038 requests hold for both chassis axes and all four physical headings.
hits = {1:(300,600), 2:(900,1200), 4:(300,1800), 8:(0,1200)}
for body in range(4):
    for mask, xy in hits.items():
        fresh(38, body)
        call('ThiefMapPlanner_ObserveThreat',38,mask.bit_length()-1,*xy,clock)
        ok = call('ThiefMapPlanner_PlanEscape',38,body,mask,0,clock,plan)
        expect(ok, f'N038 route exists body={body} mask={mask}')
        if ok:
            path=nodes_in_plan()
            expect(path[0] == (21 if mask&10 else 39), f'N038 requested exit body={body} mask={mask}: {path}')
            if not mask&10:expect(path[:2] == [39,40], f'N038 east must go through N040: {path}')
            expect(not set(path)&{55,56,57}, f'escape cannot enter closed bottom loop: {path}')

# N011 now follows the user's full corridor for a north pursuer.
for body in range(4):
    fresh(11,body)
    call('ThiefMapPlanner_ObserveThreat',11,0,600,0,clock)
    ok=call('ThiefMapPlanner_PlanEscape',11,body,1,0,clock,plan)
    expect(ok, 'N011 north route exists')
    if ok:
        path=nodes_in_plan()
        expect(path==[12,23,31,32,48,49,61,62,63], f'N011 north uses requested corridor: {path}')
        expect(not set(path)&{55,56,57}, f'N011 avoids the closed lower pocket: {path}')

# Exercise actual reaction with repeated side/rear reports for >350ms.
# These masks are explicit sensor substitutes; no route state is forced to pass.
for mode in [0,1]:
    for mask in [1,4,8,5]:
        fresh(75,(1+2*mode)%4)
        setstep(75,76,1,mode,600,action=0)
        setstep(76,77,1,mode,600,index=1)
        record(route,'MapRoute_t',dict(count=2))
        call('Route_Start')
        check(var('route_active'), 'actual route starts')
        ret('Thief_ThreatMask',mask)
        for _ in range(45):
            clock+=20;var('intercept_pending',1)
            call('Thief_InterceptReact',clock)
        expect(var('route_state')!=12, f'N076 side/rear cannot brake mode={mode} mask={mask}')
        expect(var('intercept_reversals')==0 and not var('replan_after_leg'), f'no side oscillation mode={mode} mask={mask}')
        expect(at(route,'MapRoute_t','count')==2, 'existing N075-N076-N077 route preserved')

# Repeated real gate frames at N038 while an east departure is committed.
# Unhook the explicit world-mask stub used above and run the actual pose gate.
u.hook_del(hooks.pop('Thief_ThreatMask'))
fresh(38,1)
check(call('ThiefMapPlanner_PlanEscape',38,1,1,0,clock,plan), 'create eastward N038 escape from a north threat')
call('Route_Start')
accepted=0
for _ in range(60):
    clock+=20
    call('Thief_UpdatePose');call('Thief_UpdateLaserMask',clock)
    call('Thief_OnLaserFrame',1,450,100,1,clock)
    accepted+=bool(var('intercept_pending'))
    call('Thief_InterceptReact',clock)
expect(accepted>0, 'actual left CH2 frames pass the moving gate as a north-side threat')
expect(var('intercept_reversals')==0 and not var('replan_after_leg'), '60 actual gate frames cannot cancel a safe east departure')

# Side threat matters if the NEXT planned turn points into it: finish this
# real leg then replan at its gray-confirmed node, never reverse for side alone.
fresh(75,1)
setstep(75,76,1,0,600,action=-1);setstep(76,70,0,0,300,index=1)
record(route,'MapRoute_t',dict(count=2));call('Route_Start')
# This target is actually ON the next N076-N070 road. A direction-only
# mask without a located hit no longer pretends that a parallel road is blocked.
g=symbols['thief_laser']+size(types['FrontLaserGate_t'])
record(g,'FrontLaserGate_t',dict(status=7,hit_x_mm=1800,hit_y_mm=2250))
put(symbols['thief_laser_seen']+1,1,1)
put(symbols['thief_last_laser_tick']+4,4,clock)
ret('Thief_ThreatMask',1);var('intercept_pending',1);call('Thief_InterceptReact',clock)
expect(var('route_state')!=12 and var('intercept_reversals')==0, 'side conflict cannot abruptly swap travel frame')
expect(var('replan_after_leg') and at(route,'MapRoute_t','count')==1, 'unsafe next turn deferred to real junction')
expect(at(steps,'MapRouteStep_t','action')==2, 'current leg ends with node stop, not the threatened turn')

# Real brake/swap: the same front obstacle becomes rear after retreat and
# cannot cause another reversal; a genuinely new travel-front hit still stops.
fresh(38,1);setstep(38,39,1,0,300);call('Route_Start');odom=120
ret('Thief_ThreatMask',2);var('intercept_pending',1);call('Thief_InterceptReact',clock)
expect(var('route_state')==12, 'front threat immediately brakes')
for _ in range(12):
    clock+=20
    if var('route_state')!=12:break
    call('Thief_InterceptStopped',clock)
expect(at(current(),'MapRouteStep_t','heading')==3 and var('retreat_active'), 'actual measured brake switches to west retreat')
for _ in range(45):
    clock+=20;var('intercept_pending',1);call('Thief_InterceptReact',clock)
expect(var('route_state')!=12 and var('intercept_reversals')==1, 'same obstacle now behind never flips retreat again')
ret('Thief_ThreatMask',8);var('intercept_pending',1);call('Thief_InterceptReact',clock)
expect(var('route_state')==12, 'a new obstacle ahead of reverse travel still brakes')

report=dict(firmware='UARTCH4_16V_STABLE1' if before else 'UARTCH4_16V_STABLE4', checks=assertions, failures=failures,
            expected_before_failure=before, axf_sha256=hashlib.sha256(axf.read_bytes()).hexdigest(),
            test_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(), hardware_tested=False)
target=ROOT/('Reference/STABLE1_before_route_fix/results_strategy.json' if before else 'Tests/results_strategy_stable2.json')
target.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print(json.dumps(report,ensure_ascii=False,indent=2))
if before:
    assert failures, 'before image must reproduce the requested strategy defects'
else:
    assert not failures, failures
