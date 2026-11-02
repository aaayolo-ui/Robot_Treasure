"""Actual ARM turn observation and pass-through refuge lifecycle.
Peripherals/physical response are synthetic. --before runs saved ROUTE6_SL1.
"""
from pathlib import Path
import sys,json,hashlib
ROOT=Path(__file__).absolute().parents[1]
setup=(ROOT/'Tests/test_n068_recovery.py').read_text(encoding='utf-8').split('failures=[]')[0]
setup=setup.replace('ROUTE3_before_recovery_fix','ROUTE6_SL1_before_pursuit_pass')
exec(compile(setup,str(ROOT/'Tests/test_n068_recovery.py'),'exec'))
cases=[];failures=[]

class Turning(Exception):pass

def approach52(mode,sign=1,main_spin=False):
    global front_raw,back_raw,tick,yaw
    front_raw=back_raw=0x60;fresh(35,(1+2*mode)%4)
    member('thief_localization','yaw_cw_sign',sign)
    path=[(35,36,1,600,1),(36,44,2,300,0),(44,53,2,300,1),
          (53,52,3,300,-1),(52,63,2,300,2)]
    for i,(a,b,h,length,action) in enumerate(path):setstep(a,b,h,mode,length,action,i)
    record(route,'MapRoute_t',dict(count=len(path)));member('thief_plan','goal_node',63)
    call('Route_Start')
    ordinary=tick
    def entering(dt=20):
        global front_raw,back_raw
        front_raw=gray_raw;back_raw=0x60
        at52=at(current(),'MapRouteStep_t','to_node')==52 and member('map_nav','arrived')
        if at52 and var('route_state') in [7,2]:front_raw=0x007
        ordinary(dt)
        if main_spin and at52 and var('route_state') in [5,6]:raise Turning()
        if at52 and var('route_state')==2:raise Turning()
    tick=entering
    try:complete_route()
    except Turning:pass
    finally:tick=ordinary
    check(at(current(),'MapRouteStep_t','to_node')==52 and member('map_nav','arrived'),
          'actual turn+gray-acquisition at N052 precedes N063')
    bearing=24000 if main_spin else (19400 if mode==0 else 34600)
    origin=member('thief_localization','origin_heading')*9000
    yaw=((bearing-origin)*sign+18000)%36000-18000
    call('Thief_UpdatePose');call('Thief_UpdateLaserMask',clock)

def feed(mode,count=3,raw=468):
    global clock
    for _ in range(count):
        clock+=20;call('Thief_OnLaserFrame',3 if mode==0 else 0,raw,100,1,clock)

def finish_watch():
    global tick
    ordinary=tick;passed=[]
    def sensors(dt=20):
        global front_raw,back_raw
        front_raw=gray_raw;back_raw=0x60;ordinary(dt)
        if at(current(),'MapRouteStep_t','to_node')==63 and member('map_nav','arrived'):
            passed.append(var('route_state'))
            check(var('route_active') and var('route_state')==11,
                  'N063 real gray event passes straight, without hide approach/settle/parking')
    tick=sensors
    try:result=complete_route()
    finally:tick=ordinary
    return result,passed

def pursued():
    global front_raw,back_raw,yaw
    for mode in [0,1]:
        for sign in [-1,1]:
            approach52(mode,sign)
            before=member('thief_plan','goal_node');state=var('route_state')
            # Route locking must not erase rear pursuit observations; it still
            # prevents side/rear callbacks from interrupting the current turn.
            var('n011_escape_locked',1)
            feed(mode)
            check(member('thief_plan','goal_node')==before and var('route_state')==state,
                  'UART turn observation cannot replace a plan or interrupt motors')
            front_raw=back_raw=0x60
            for _ in range(6):tick()
            check(member('thief_plan','goal_node')!=63 and var('route_active'),
                  'confirmed rotating rear pursuit extends the hide before reaching N063')
            check(at(steps+4*size(types['MapRouteStep_t']),'MapRouteStep_t','action')==0,
                  'old N063 STOP becomes actual straight pass')
            prefix=[at(steps+i*size(types['MapRouteStep_t']),'MapRouteStep_t','to_node') for i in range(4,8)]
            check(prefix==[63,78,77,76], 'north pursuer uses existing N063 south/west corridor')
            result,passed=finish_watch()
            check(passed and result['node'] in [11,38] and result['node']==member('thief_plan','goal_node'),
                  'continuous N063 escape reaches the new selected refuge')
            cases.append(dict(case='pursued',mode=mode,gyro_sign=sign,passed_states=passed,result=result))

def no_pursuit():
    global front_raw,back_raw,clock
    for kind in ['quiet','one_frame','expired']:
        approach52(0)
        if kind=='one_frame':feed(0,1)
        if kind=='expired':feed(0);clock+=1600
        front_raw=back_raw=0x60
        for _ in range(6):tick()
        check(member('thief_plan','goal_node')==63,kind+' cannot create a pursuit extension')
        result,_=finish_watch_quiet()
        check(result['node']==63,kind+' preserves original N063 hide')
        cases.append(dict(case=kind,result=result))

def finish_watch_quiet():
    global tick
    ordinary=tick
    def sensors(dt=20):
        global front_raw,back_raw
        front_raw=gray_raw;back_raw=0x60;ordinary(dt)
    tick=sensors
    try:return complete_route(),[]
    finally:tick=ordinary

def return_home_pass():
    global front_raw,back_raw,tick
    approach52(0)
    # Controlled final-return cursor: home N063 is the next actual gray node.
    record(symbols['route_trail'],'ThiefRouteTrail',dict(home=63,home_body=0,count=0,
        enabled=1,returning=1,return_left=1,return_pending=0,first_back=1,
        expected_return_node=63,pending_valid=0))
    feed(0);front_raw=back_raw=0x60
    for _ in range(6):tick()
    check(member('thief_plan','goal_node')!=63,'return-home pursuit extends rather than parking')
    ordinary=tick;seen=[]
    def sensors(dt=20):
        global front_raw,back_raw
        front_raw=gray_raw;back_raw=0x60;ordinary(dt)
        if member('map_nav','step')>=5 and var('route_active'):
            check(not member('route_trail','returning'),'passed home releases exhausted reverse cursor')
            check(member('route_trail','enabled'),'appended real legs retain actual-history recording')
            seen.append(member('route_trail','count'))
    tick=sensors
    try:result=complete_route()
    finally:tick=ordinary
    check(seen and max(seen)>0,'actual exits after old home are committed again')
    cases.append(dict(case='return_home_pass',max_new_edges=max(seen),result=result))

def front_priority():
    global front_raw,back_raw,yaw,clock
    approach52(0);feed(0);front_raw=back_raw=0x60
    for _ in range(6):tick()
    check(member('thief_plan','goal_node')!=63,'pursuit already prepared the new escape route')
    yaw=18000-member('thief_localization','origin_heading')*9000
    call('Thief_UpdatePose');call('Thief_UpdateLaserMask',clock)
    for i in range(3):
        clock+=20;call('Thief_OnLaserFrame',0,150,100,1,clock)
    call('Thief_InterceptReact',clock)
    check(var('route_state')==12,'real travel-front obstacle still enters original immediate brake')
    cases.append(dict(case='front_priority',state=var('route_state')))

def main_spin_capture():
    approach52(0,main_spin=True)
    state=var('route_state');feed(0)
    check(var('route_state')==state and member('thief_plan','goal_node')==63,
          'actual main-spin rear frames are retained without interrupting rotation')
    check(member('pursuit','stamp')!=0,'main-spin physical ray produces confirmed pursuit evidence')
    result,passed=finish_watch()
    check(passed and result['node']!=63,'main-spin observation prevents later parking at N063')
    cases.append(dict(case='main_spin_capture',passed_states=passed,result=result))

functions=[pursued,no_pursuit] if '--before' in sys.argv else [pursued,no_pursuit,return_home_pass,front_priority,main_spin_capture]
for fn in functions:
    try:fn();print(fn.__name__+': PASS',flush=True)
    except AssertionError as e:
        failures.append(dict(case=fn.__name__,error=str(e)));print(fn.__name__+': FAIL '+str(e),flush=True)
report=dict(checks=checks,failures=failures,cases=cases,axf_sha256=hashlib.sha256(axf.read_bytes()).hexdigest(),test_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),hardware_tested=False)
(ROOT/'Tests'/('pursuit_pass_before.json' if '--before' in sys.argv else 'results_pursuit_pass.json')).write_text(json.dumps(report,indent=2),encoding='utf-8')
print(json.dumps(report,indent=2));sys.exit(bool(failures))
