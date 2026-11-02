"""Real linked ARM gray/stop logic; synthetic board reads and wheel physics.
--before checks saved SL1 with the same fixtures. No serial/flash/motors.
"""
from pathlib import Path
import sys,json,hashlib
ROOT=Path(__file__).absolute().parents[1];before='--before' in sys.argv
setup=(ROOT/'Tests/test_n068_recovery.py').read_text(encoding='utf-8').split('failures=[]')[0]
setup=setup.replace('ROUTE3_before_recovery_fix','ROUTE6_SL1_before_t_junction')
exec(compile(setup,str(ROOT/'Tests/test_n068_recovery.py'),'exec'))
cases=[];failures=[]
def step(dt=20):
    global clock
    clock+=dt;call('App_ThiefStrategy_Task')
def prepare(mode=0,sign=1,action=-1,node=35):
    global front_raw,back_raw,odom
    front_raw=back_raw=0x60
    start=26 if node==35 else 17
    fresh(start,(2+2*mode)%4);member('thief_localization','yaw_cw_sign',sign)
    end=(36 if action==-1 else 34) if node==35 else 27
    h=1 if action==-1 else 3
    setstep(start,node,2,mode,300,action)
    setstep(node,end,h,mode,600 if end==36 else 300,index=1)
    record(route,'MapRoute_t',dict(count=2));member('thief_plan','goal_node',end)
    call('Route_Start')
    for _ in range(8):tick()
    check(var('route_state')==2 and var('junction_armed'),'actual line follow arms the incoming leg')
    odom=300;front_raw=0x063 if action==-1 else 0xc60
    step(5)
    check(member('map_nav','arrived') and var('route_state')==3,'opposite-side gray actually latches arrival and starts normal approach')
    front_raw=0
    for _ in range(20):
        if var('route_state')==4:break
        step()
    check(var('route_state')==4,'normal 180ms approach enters measured brake')

def finish():
    global tick
    ordinary=tick
    def sensors(dt=20):
        global front_raw,back_raw
        front_raw=gray_raw;back_raw=0x60;ordinary(dt)
    tick=sensors
    try:return complete_route(limit=6000)
    finally:tick=ordinary

def preserved_ports():
    for mode in [0,1]:
        fresh(26,(2+2*mode)%4)
        setstep(26,35,2,mode,300,-1);setstep(35,36,1,mode,600,index=1)
        record(route,'MapRoute_t',dict(count=2))
        call('MapRouteNavigator_StartRuntime',nav,route,steps)
        call('MapRouteNavigator_Observe',nav,0xc60)
        call('MapRouteNavigator_Arrive',nav,0x063,300)
        check(member('map_nav','seen_ports')==3 and call('MapRouteNavigator_ConfirmTurn',nav),
              'arrival preserves the earlier requested branch across a later asymmetric sample')
        call('MapRouteNavigator_Advance',nav)
        check(member('map_nav','seen_ports')==0,'next leg clears old branch evidence')
    cases.append(dict(case='preserved_ports'))

def centered_t():
    for mode in [0,1]:
        for sign in [-1,1]:
            for action in [-1,1]:
                prepare(mode,sign,action)
                original_seen=member('map_nav','seen_ports')
                for _ in range(12):
                    if var('route_state') in [5,6,10]:break
                    step()
                check(var('route_state') in [5,6],
                      f'centered T rotates instead of locking: mode={mode} sign={sign} action={action} reason={reason()}')
                check(member('map_nav','step')==0 and member('map_nav','seen_ports')==original_seen,
                      'T evidence cannot invent a side observation or commit an outgoing node')
                result=finish()
                check(result['node']==(36 if action==-1 else 34),'ordinary gray/yaw turn exit and following still complete')
                cases.append(dict(case='centered_t',mode=mode,sign=sign,action=action,result=result))

def invalid_t():
    global back_raw,front_raw,yaw
    for kind in ['both_white','wide_trailing','offset_trailing','leading_on_line','wrong_yaw','unknown_sign','open_junction','stale_yaw','broken_pair']:
        prepare(node=26 if kind=='open_junction' else 35)
        if kind=='both_white':back_raw=0
        if kind=='wide_trailing':back_raw=0xfff
        if kind=='offset_trailing':back_raw=1
        if kind=='leading_on_line':front_raw=0x60
        if kind=='wrong_yaw':yaw=2000
        if kind=='unknown_sign':member('thief_localization','yaw_cw_sign',0)
        if kind=='stale_yaw':hook('JY61P_GetLastUpdateMs',lambda:clock-200)
        if kind=='broken_pair':
            step();back_raw=0
        for _ in range(12):
            if var('route_state') in [5,6,10]:break
            step()
        hook('JY61P_GetLastUpdateMs',lambda:clock)
        check((var('route_state') not in [5,6] if kind=='stale_yaw' else
               var('route_state')==10 and reason()=='MAP_REQUIRED_BRANCH_NOT_SEEN'),
              kind+' cannot authorize an unsupported turn')
        cases.append(dict(case=kind,reason=reason()))

def full_n007_corridor():
    global front_raw,back_raw,tick
    for mode in [0,1]:
        front_raw=back_raw=0x60;fresh(6,(1+2*mode)%4)
        path=[(6,7,1,600,1),(7,17,2,300,0),(17,26,2,300,0),(26,35,2,300,-1),
              (35,36,1,600,1),(36,44,2,300,0),(44,53,2,300,1),(53,52,3,300,-1),(52,63,2,300,2)]
        for i,(a,b,h,mm,action) in enumerate(path):setstep(a,b,h,mode,mm,action,i)
        record(route,'MapRoute_t',dict(count=len(path)));member('thief_plan','goal_node',63)
        call('Route_Start');ordinary=tick;seen=[]
        def sensors(dt=20):
            global front_raw,back_raw
            at35=at(current(),'MapRouteStep_t','to_node')==35
            front_raw=gray_raw;back_raw=0x60
            if at35:
                if member('map_nav','arrived') and var('route_state') in [3,4]:front_raw=0
                elif not member('map_nav','arrived') and gray_state in [3,4,5]:front_raw=0x063
            ordinary(dt)
            if at35:seen.append(var('route_state'))
        tick=sensors
        try:result=complete_route(limit=6000)
        finally:tick=ordinary
        check(result['node']==63 and result['body']%2==0,'N007 corridor completes N035 turn then reaches and vertically parks N063')
        check(4 in seen and 6 in seen,'real N035 stop/left-spin stages executed')
        cases.append(dict(case='full_n007_corridor',mode=mode,result=result))

for fn in [preserved_ports,centered_t,invalid_t]+([] if before else [full_n007_corridor]):
    try:fn();print(fn.__name__+': PASS',flush=True)
    except AssertionError as e:failures.append(dict(case=fn.__name__,error=str(e)));print(fn.__name__+': FAIL '+str(e),flush=True)
out=dict(checks=checks,failures=failures,cases=cases,axf_sha256=hashlib.sha256(axf.read_bytes()).hexdigest(),
         test_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),hardware_tested=False)
(ROOT/'Tests'/('t_junction_before.json' if before else 'results_t_junction.json')).write_text(json.dumps(out,indent=2),encoding='utf-8')
print(json.dumps(out,indent=2));sys.exit(bool(failures) if not before else not bool(failures))
