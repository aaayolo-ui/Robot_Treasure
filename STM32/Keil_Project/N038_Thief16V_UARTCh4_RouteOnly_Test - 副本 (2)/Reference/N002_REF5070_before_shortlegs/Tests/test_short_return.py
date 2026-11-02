"""152637 short reversal through N007: actual ARM, synthetic sensors/wheels.

Build the eight already-executed edges, then trigger the real three-frame
rear laser gate 89mm into 7->17. The virtual node17 must never be committed.
"""
from pathlib import Path
import sys,json,hashlib
ROOT=Path(__file__).absolute().parents[1]
setup=(ROOT/'Tests/test_n068_recovery.py').read_text(encoding='utf-8').split('failures=[]')[0]
setup=setup.replace('ROUTE3_before_recovery_fix','ROUTE5_before_short_return')
exec(compile(setup,str(ROOT/'Tests/test_n068_recovery.py'),'exec'))
failures=[];cases=[]

def prepare_short(settle=True):
    global front_raw,back_raw,odom,clock,physical_rpm
    front_raw=back_raw=0x60;fresh(7,0)
    setstep(7,17,2,1,300);call('Route_Start')
    call('ThiefRouteTrail_Begin',trail,38,2)
    for a,b,h,length in [(38,21,0,600),(21,22,1,300),(22,11,0,300),(11,2,0,300),
                         (2,3,1,600),(3,4,1,300),(4,6,1,300),(6,7,1,600)]:
        setstep(a,b,h,1,length)
        if (a,b)==(4,6):record(steps,'MapRouteStep_t',dict(segment_count=2))
        call('ThiefRouteTrail_Stage',trail,steps,0xFFF,0x60,0,length)
        check(call('ThiefRouteTrail_Commit',trail)==1,'build actual committed route to N007')
    setstep(7,17,2,1,300)
    for _ in range(8):tick()
    odom=89
    call('Thief_UpdatePose');call('Thief_UpdateLaserMask',clock)
    for _ in range(3):
        clock+=20;call('Thief_OnLaserFrame',3,387,100,1,clock)
    call('Thief_InterceptReact',clock)
    check(var('route_state')==12,'real travel-front rear laser must trigger intercept brake')
    front_raw=0;back_raw=0x100
    for _ in range(12):
        tick()
        if tm('returning') and var('route_state')==8:break
    check(tm('returning') and tm('first_back') and member('map_nav','step')==0,
          'real braking and sensor swap begin partial return without inventing N017')
    check(at(current(),'MapRouteStep_t','to_node')==7 and tm('count')==8,
          'nearest real return node is N007; eight actual edges retained')
    check(var('map_progress_offset_mm')==211,'partial return credits original 89mm rather than full300mm')
    # Facing north close to the top edge: wall returns must remain filtered
    # even though a real south obstacle just triggered this short reversal.
    for _ in range(3):
        clock+=20;call('Thief_OnLaserFrame',0,340,100,1,clock)
    check(not var('intercept_pending') and tm('returning'), 'off-field northern ray cannot abort the return')
    for _ in range(12 if settle else 1):tick()
    commands.clear()

def finish():
    global tick
    ordinary=tick
    def sensors(dt=20):
        global front_raw,back_raw
        front_raw=gray_raw;back_raw=0x60;ordinary(dt)
    tick=sensors
    try:return complete_route()
    finally:tick=ordinary

def logged_skew():
    global front_raw,back_raw
    for early,front,back in [(early,f,b) for early in [True,False] for f,b in
                            [(0x040,0x100),(0x060,0x180),(0x078,0x180)]]:
        prepare_short(not early)
        front_raw=front;back_raw=back
        for _ in range(12):tick()
        check(var('route_state')==8 and all(l+r<=0 for tag,l,r in commands if tag=='drive'),
              'logged skewed two-board stripe cannot release forward high-speed follow')
        check(tm('count')==8 and not member('map_nav','arrived'),'skew recovery cannot commit virtual N017 or N007')
        front_raw=back_raw=0x60;commands.clear()
        for _ in range(10):
            tick()
            if var('route_state')==2:break
        check(var('route_state')==2 and var('route_active'),'stable aligned gray returns to original PID')
        drives=[x for x in commands if x[0]=='drive' and x[1]+x[2]>0]
        check(drives and (drives[0][1]+drives[0][2])//2<900,
              'handoff begins near50RPM instead of the saved full-speed seed')
        result=finish()
        check(result['node']==38 and result['body']%2==0,'actual return task completes at vertical N038')
        cases.append(dict(front=front,rear=back,during_brake=early,home=result))

try:logged_skew();print('logged_skew: PASS')
except AssertionError as e:failures.append(dict(case='logged_skew',error=str(e)));print('logged_skew: FAIL',str(e))
report=dict(checks=checks,failures=failures,cases=cases,axf_sha256=hashlib.sha256(axf.read_bytes()).hexdigest(),hardware_tested=False)
(ROOT/'Tests'/('short_return_before.json' if '--before' in sys.argv else 'results_short_return.json')).write_text(json.dumps(report,indent=2),encoding='utf-8')
print(json.dumps(report,indent=2));sys.exit(bool(failures))
