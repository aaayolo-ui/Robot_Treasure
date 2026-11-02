"""Linked ARM pursuit geometry, bounds, append validation and PA3 MMIO.
All input/peripheral memory is synthetic; no serial, flashing or motors.
"""
from pathlib import Path
import json,hashlib,math
ROOT=Path(__file__).absolute().parents[1]
setup=(ROOT/'Tests/test_escape_completion.py').read_text(encoding='utf-8').split('def complete_route(')[0]
exec(compile(setup,str(ROOT/'Tests/test_escape_completion.py'),'exec'))
p=0x2000c200;pose=0x2000c300;goal=0x2000c400
cases=[]
def observe(ch=3,raw=450,conf=100,angle=18000,x=2700,y=1200,times=(100,120,140)):
    call('ThiefPursuit_Reset',p)
    record(pose,'ThiefPursuitPose',dict(x=x,y=y,body_cw_x100=angle,travel_heading=2))
    for t in times:call('ThiefPursuit_Observe',p,ch,raw,conf,pose,t)
    return at(p,'ThiefPursuit','stamp')

# Same world north ray, every physical probe and multiple angle wraps.
for ch,offset in enumerate([0,3,1,2]):
    for wrap in [-72000,-36000,0,36000,72000]:
        raw=500 if ch in [1,2] else 450
        check(observe(ch,raw,angle=-offset*9000+wrap)==140,'physical ray cardinal projection')
        check(at(p,'ThiefPursuit','x')==2700 and at(p,'ThiefPursuit','y')==600,
              'every channel maps to the same real northern road')
        record(goal,'MapRouteStep_t',dict(to_node=63,heading=2))
        check(call('ThiefPursuit_GoalMask',p,goal,140)==1,'north pursuit mask')
        check(call('ThiefPursuit_GoalMask',p,goal,1641)==0,'1500 ms evidence expiry')
for angle in [16600,19400]:
    check(observe(angle=angle,raw=468)==140,'turn-time ray reaches real northern road')
    a=math.radians(angle/100+180)
    ex=2700+618*math.sin(a);ey=1200-618*math.cos(a)
    check(abs(at(p,'ThiefPursuit','x')-ex)<2 and abs(at(p,'ThiefPursuit','y')-ey)<2,
          'integer interpolation stays within 2 mm of floating reference')

for label,args in [
    ('low confidence',dict(conf=0)),('too near',dict(raw=49)),
    ('front beyond max',dict(ch=0,raw=1001,angle=0)),
    ('left beyond max',dict(ch=1,raw=901,angle=9000)),
    ('right beyond max',dict(ch=2,raw=901,angle=-9000)),
    ('rear beyond max',dict(raw=1001)),('single frame',dict(times=(100,))),
    ('duplicate frames',dict(times=(100,100,100))),('spaced frames',dict(times=(100,220,340))),
    ('outside north edge',dict(y=300)),('ahead of travel',dict(angle=0)),
    ('off-road gap',dict(x=2250,y=2100))]:
    check(observe(**args)==0,label+' cannot confirm pursuit');cases.append(label)
check(observe(times=(0xffffffe0,0xfffffff4,8))==8,'uint32 timer wrap preserves real frames')

# Build actual planner tails, reconcile both incoming motion frames, and ask
# the real runtime navigator to validate every joined route.
tail=symbols['pursuit_tail']
for mode in [0,1]:
    for mask in [1,2,8]:
        fresh(52,(2+2*mode)%4);setstep(52,63,2,mode,300)
        check(call('ThiefMapPlanner_PlanEscape',63,(2+2*mode)%4,mask,52,10000,tail)==1,
              'existing planner supplies a connected escape tail')
        check(call('ThiefPursuit_Append',plan,tail)==1,'valid route tail appends')
        call('MapRouteNavigator_StartRuntime',nav,route,steps)
        check(not member('map_nav','fault'),'joined actions/modes pass actual navigator checks')
        check(at(steps,'MapRouteStep_t','action')!=2,'old hide no longer has STOP')
fresh(52,2);setstep(52,63,2,0,300)
call('ThiefMapPlanner_PlanEscape',63,2,1,52,10000,tail)
record(route,'MapRoute_t',dict(count=48))
before=bytes(u.mem_read(plan,size(types['ThiefRoutePlan_t'])))
check(call('ThiefPursuit_Append',plan,tail)==0 and before==bytes(u.mem_read(plan,len(before))),
      'capacity failure leaves existing route byte-for-byte unchanged')

# Preserve the other chat's continuous PA3 sound/light initialization.
put(0x40010800,4,0x44444444);put(0x40021018,4,0x100)
call('PowerSoundLight_Init')
check(get(0x40021018)==0x104,'PA3 GPIOA clock enable preserves other clocks')
check(get(0x40010800)==0x44442444,'PA3 output mode preserves adjacent pins')
check(get(0x40010810)==(8<<16),'PA3 active-low command reaches HAL BSRR')
out=dict(checks=checks,failures=[],negative_cases=cases,axf_sha256=hashlib.sha256(axf.read_bytes()).hexdigest(),hardware_tested=False)
(ROOT/'Tests/results_pursuit_geometry.json').write_text(json.dumps(out,indent=2),encoding='utf-8')
print(json.dumps(out,indent=2))
