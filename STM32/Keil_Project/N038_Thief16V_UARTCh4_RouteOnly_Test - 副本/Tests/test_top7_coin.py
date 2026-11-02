"""Actual ARM task with synthetic gray, odometry and wheels; no real hardware."""
from pathlib import Path
import json,hashlib
ROOT=Path(__file__).absolute().parents[1]
setup=(ROOT/'Tests/test_n068_recovery.py').read_text(encoding='utf-8').split('failures=[]')[0]
exec(compile(setup,str(ROOT/'Tests/test_n068_recovery.py'),'exec'))

def cycle(dt=20):
    global clock
    clock+=dt;call('App_ThiefStrategy_Task')

def coin_start(a=6,b=4,h=3,mode=0,rid=1600):
    global front_raw,back_raw,odom
    front_raw=back_raw=0x60;fresh(a,(h+2*mode)%4)
    setstep(a,b,h,mode,600 if a==75 else 300)
    if (a,b) in [(6,4),(4,6)]:record(steps,'MapRouteStep_t',dict(segment_count=2))
    record(route,'MapRoute_t',dict(id=rid,count=1,start_node=a,heading=h,drive_mode=mode))
    call('Route_Start')
    for _ in range(8):cycle()
    check(var('route_state')==2,'actual stripe enters FOLLOW')
    odom=170;commands.clear()

failures=[];cases=[]
try:
    fresh(54,3);check(call('ThiefMapPlanner_PlanInitial',plan)==1,'initial route builds')
    nodes=[at(steps+i*size(types['MapRouteStep_t']),'MapRouteStep_t','to_node') for i in range(at(route,'MapRoute_t','count'))]
    check(nodes==[53,44,36,35,26,17,7,6,4,3,2,11],'complete requested map route, N005 remains an internal mark')
    check(member('thief_plan','goal_node')==11 and member('thief_plan','final_body_heading')==3,'first hide remains N011 west parked')
    cases.append(dict(case='route',nodes=nodes))
    for raw,entry_speed in [(0x60,1200),(0x30,2200),(0xC0,2600)]:
        coin_start();front_raw=0;back_raw=raw
        member('follow_smoothing','base_rpm_x10',entry_speed)
        for _ in range(3):
            cycle();odom+=8
            check(var('route_state')==2 and not member('map_nav','arrived'),'rear stripe passes short gap without claiming N004')
        drive=[x for x in commands if x[0]=='drive']
        check(drive and all(x[1]>0 and x[2]>0 and x[1]+x[2]==2*entry_speed and abs(x[1]-x[2])<=400 for x in drive),'rear correction preserves actual entry speed without a 50RPM dip')
        front_raw=back_raw=0x60;cycle()
        check(var('route_state')==2 and not var('treasure_rear_bridge'),'front stripe restores normal follow')
        cases.append(dict(case='coin',raw=raw,entry_rpm_x10=entry_speed,commands=drive))
    coin_start();front_raw=0;cycle();odom=225;cycle()
    check(var('route_state')==8,'50mm front gap falls back to existing recovery')
    for args in [(6,4,3,0,1601),(4,6,1,1,1600),(75,59,0,0,1600)]:
        coin_start(*args);front_raw=0;cycle()
        check(var('route_state')==8 and not var('treasure_rear_bridge'),'unrelated route/frame cannot use exception')
    coin_start();front_raw=back_raw=0;cycle()
    check(not var('treasure_rear_bridge') and var('route_state')==8,'both-white never authorizes forward bridge')
    for _ in range(110):cycle()
    check(var('route_state')==10 and reason()=='GRAY_BOTH_NO_LINE','continuous both-white two seconds stops')
    cases.append(dict(case='distance_and_white_limits'))
except Exception as e:failures.append(str(e))
out=dict(checks=checks,failures=failures,cases=cases,axf_sha256=hashlib.sha256(axf.read_bytes()).hexdigest(),hardware_tested=False)
(ROOT/'Tests/results_top7_coin.json').write_text(json.dumps(out,indent=2),encoding='utf-8')
print(json.dumps(out,indent=2));assert not failures,failures
