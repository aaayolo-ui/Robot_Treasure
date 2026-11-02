"""Replay edge cases in actual ARM controller; peripherals simulated."""
from pathlib import Path
import json,hashlib
setup=Path(__file__).with_name('test_documents15.py').read_text(encoding='utf-8').split('parking_pwm=[]')[0]
exec(compile(setup,str(Path(__file__).with_name('test_documents15.py')),'exec'))
trail=symbols['route_trail'];results=[]
def tm(n):return at(trail,'ThiefRouteTrail',n)
def front_channel():return [(var('route_body_heading')+d)%4 for d in [0,3,1,2]].index(at(current(),'MapRouteStep_t','heading'))
def pulse(ch):
    global clock
    call('Thief_UpdatePose');call('Thief_UpdateLaserMask',clock)
    for _ in range(3):clock+=20;call('Thief_OnLaserFrame',ch,150,100,1,clock)
    call('Thief_InterceptReact',clock)

class Ready(Exception):pass
ordinary=tick

def prepare(wanted=3):
    global tick,clock,odom,gray_raw,gray_state,rear_valid,rear_error
    rear_valid=1;rear_error=0
    fresh(11,1)
    for _ in range(3):clock+=20;call('Thief_OnLaserFrame',3,150,100,1,clock)
    ordinary();fired=False
    def watcher(dt=20):
        nonlocal fired
        global odom
        if not fired and member('map_nav','step')==wanted and var('route_state') in [1,2] and not member('map_nav','arrived') and call('Route_LegProgress')>=120:
            pulse(front_channel());check(var('route_state')==12,'actual front confirmation starts braking');odom+=17;fired=True
        ordinary(dt)
        if fired and tm('returning') and var('route_state') in [1,2]:raise Ready()
    tick=watcher
    try:complete_route()
    except Ready:pass
    finally:tick=ordinary
    check(tm('returning') and tm('count')==wanted,'recorded only real edges before reversing partial leg')
    check(var('map_progress_offset_mm')==at(current(),'MapRouteStep_t','length_mm')-137,'partial reverse includes17mm of measured braking travel')

prepare()
length=at(current(),'MapRouteStep_t','length_mm');odom=length-var('map_progress_offset_mm')
old_count=tm('count');old_step=member('map_nav','step');gray_raw=0xC60;gray_state=4
for _ in range(4):ordinary()
check(tm('count')==old_count and member('map_nav','step')==old_step and not tm('return_pending'),'wrong-side gray cannot consume expected return turn')
gray_raw=0x063;gray_state=5
ordinary();check(tm('return_pending') and tm('count')==old_count,'correct junction stages, never pops before completing turn')
# Approach and main spin retain their actual original state machine.
for _ in range(60):
    st=var('route_state')
    if st in [5,6]:
        sign=member('thief_localization','yaw_cw_sign');sign=sign-256 if sign>=128 else sign
        yaw=(yaw+(1 if st==5 else -1)*sign*600+18000)%36000-18000
        gray_raw=0;gray_state=2
    ordinary()
    if var('route_state')==8:break
check(var('route_state')==8 and tm('count')==old_count,'missing exit line enters original recovery without consuming history')
for _ in range(6):ordinary()
check(tm('count')==old_count and member('map_nav','step')==old_step,'recovery cannot fabricate an exited node')
check(not targets or all(sum(t[:2])==0 for t in targets[-3:]),'unfinished replay turn never drives ahead from rear-only line')
# Drive a bounded yaw plant from the actual sweep targets. The front board
# sees the exit only near 90 degrees; the trailing old stripe stays visible.
targets.clear()
for _ in range(160):
    if targets and sum(targets[-1][:2])==0:
        yaw=(yaw+(targets[-1][0]-targets[-1][1])//10+18000)%36000-18000
    visible=abs((yaw-9000+18000)%36000-18000)<=400
    gray_raw=0x60 if visible else 0;gray_state=1 if visible else 2
    if not visible:check(tm('count')==old_count,'no old rear stripe can falsely complete the turn')
    ordinary()
    if member('map_nav','step')==old_step+1:break
check(member('map_nav','step')==old_step+1 and tm('return_left')==3,'real recovered line commits exactly one return edge')
result=complete_route();check(result['node']==11,'recovered replay still returns home')
results.append(dict(case='wrong_branch_and_missing_exit',result=result))

prepare();prior=tm('count');pulse(front_channel());check(var('route_state')==12,'new actual obstacle ahead of return still brakes')
for _ in range(12):
    ordinary()
    if var('route_state')!=12:break
check(not tm('returning') and not tm('enabled') and not var('n011_escape_locked'),'blocked return cancels replay before base node retreat/replan')
check(tm('count')==prior,'abort keeps historical records for diagnostics')
results.append(dict(case='new_return_front',fallback_route=at(route,'MapRoute_t','id')))

prepare();prior=tm('count');ptr=0x2000c500;u.mem_write(ptr,b'KEY_STOP\0');call('Route_StopFault',ptr)
check(not var('route_active') and not tm('enabled') and not tm('returning'),'manual stop remains authoritative during replay')
check(tm('count')==prior,'manual stop does not pretend return was completed')
results.append(dict(case='manual_stop'))

prepare();prior=tm('count');rear_valid=2;gray_raw=0;gray_state=2
for _ in range(120):
    clock+=20;call('App_ThiefStrategy_Task')
    if not var('route_active'):break
check(not var('route_active') and reason()=='GRAY_BOTH_NO_LINE','continuous both-board white still stops after two seconds')
check(tm('count')==prior and not tm('enabled'),'two-white stop does not consume a nonexistent road')
results.append(dict(case='two_seconds_both_white',reason=reason()))

fresh(11,1)
for _ in range(3):clock+=20;call('Thief_OnLaserFrame',1,150,100,1,clock)
ordinary();check(tm('enabled') and tm('count')==0,'known parked home starts empty history')
pulse(front_channel())
for _ in range(12):
    ordinary()
    if not var('route_active'):break
check(not var('route_active') and var('thief_current_node')==11 and not var('replan_after_leg'),'zero displacement returns to known home without traversing virtual full edge')
results.append(dict(case='zero_displacement'))
# A straight return junction preserves travel during line clearing; no new
# leg origin can be created at the later exit-line sample.
prepare(2)
counts=var('map_leg_start_counts')+at(current(),'MapRouteStep_t','length_mm')-var('map_progress_offset_mm')
hook('MapRoutePosition_ForwardCounts',lambda:counts)
hook('MapRoutePosition_DistanceMm',lambda:counts-u.reg_read(UC_ARM_REG_R0))
gray_raw=0xfff;gray_state=3
ordinary();check(var('route_state')==11 and var('trace_pass_pending'),'actual straight return junction records encoder anchor')
anchor=counts;left=tm('return_left');gray_raw=0x60;gray_state=1
for _ in range(3):counts+=20;ordinary()
check(member('map_nav','step')==1 and var('map_leg_start_counts')==anchor,'straight return keeps node detection origin')
check(call('Route_LegProgress')==60 and tm('return_left')==left-1,'60mm passing travel retained; only one returned edge consumed')
results.append(dict(case='straight_return_distance_credit',credit_mm=call('Route_LegProgress')))

report=dict(checks=checks,failures=[],cases=results,axf_sha256=hashlib.sha256(axf.read_bytes()).hexdigest(),hardware_tested=False)
(ROOT/'Tests/results_trace_recovery.json').write_text(json.dumps(report,indent=2),encoding='utf-8');print(json.dumps(report,indent=2))

