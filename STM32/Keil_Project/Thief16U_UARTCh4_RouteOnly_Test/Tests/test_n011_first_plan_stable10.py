"""Linked ARM first-confirmed N011 result and transfer lock.
Sensors/motion are simulated; --before applies assertions to STABLE9.
"""
from pathlib import Path
import sys,json,hashlib
setup=Path(__file__).with_name('test_escape_completion.py').read_text(encoding='utf-8').split('# Log regression:')[0]
before='--before' in sys.argv
if before:
    setup=setup.replace('exec(compile(setup.split(',
        'setup=setup.replace("axf=ROOT/\'MDK-ARM\'/ROOT.name/(ROOT.name+\'.axf\')", "axf=ROOT/\'Reference/STABLE9_before_n011_first_plan/MDK-ARM\'/ROOT.name/(ROOT.name+\'.axf\')")\nexec(compile(setup.split(',1)
exec(compile(setup,str(Path(__file__).with_name('test_escape_completion.py')),'exec'))
checks=0;failures=[];lifecycles=[];cases={}
west=[10,20,37,38,39,40];east=[12,13,3,4,6,7]
def expect(v,msg):
    global checks
    checks+=1
    if not v:failures.append(msg)
def flag(n):return var(n) if n in symbols else 0
def path():return [at(steps+i*size(types['MapRouteStep_t']),'MapRouteStep_t','to_node') for i in range(at(route,'MapRoute_t','count'))]
def channel(body,world):return [(body+d)%4 for d in [0,3,1,2]].index(world)
def frames(ch,n=3):
    global clock
    for _ in range(n):clock+=20;call('Thief_OnLaserFrame',ch,150,100,1,clock)
def verify_route(world):
    expect(path()[:6]==(west if world in [0,1] else east),f'world direction {world} selects exact six-node corridor: {path()}')
    expect(member('thief_plan','goal_node') in [26,35,63],'destination is an eastern refuge')

for body in range(4):
  for world in range(4):
    fresh(11,body)
    frames(channel(body,world),2)
    expect(not flag('n011_first_mask'),'single/two unconfirmed observations do not latch an opponent')
    frames(channel(body,world),1)
    expect(flag('n011_first_mask')==1<<world,'first confirmed result uses world direction')
    # Another direction arrives before the main task gets to plan.
    frames(channel(body,(world+2)%4))
    expect(flag('n011_first_mask')==1<<world,'later closer/different direction cannot replace first confirmation')
    tick();verify_route(world)
    expect(flag('n011_escape_locked')==1,'successful N011 plan activates transfer lock')
cases['body_world_first_result']=16

ordinary_tick=tick
for body,sign,world in [(1,1,0),(3,-1,0),(1,-1,2),(3,1,2)]:
    fresh(11,body);member('thief_localization','yaw_cw_sign',sign)
    frames(channel(body,world));tick();verify_route(world)
    if not flag('n011_escape_locked'):
        expect(False,'old image lacks transfer lock');continue
    expected_goal=member('thief_plan','goal_node')
    frozen_plan=bytes(u.mem_read(plan,size(variables['thief_plan'])))
    injected=0;frames_run=0
    def noisy_tick(dt=20):
        global injected,frames_run
        frames_run+=1
        if flag('n011_escape_locked') and var('route_active') and frames_run%10==0:
            for ch in range(4):call('Thief_OnLaserFrame',ch,120,100,1,clock)
            injected+=4
            expect(not var('intercept_pending') and not var('replan_after_leg'),
                'all moving laser channels cannot request retreat/replan while locked')
            expect(bytes(u.mem_read(plan,size(variables['thief_plan'])))==frozen_plan,
                'original goal/steps/drive modes stay unchanged through new sightings')
        ordinary_tick(dt)
    tick=noisy_tick
    result=complete_route()
    tick=ordinary_tick
    expect(injected>0 and result['node']==expected_goal,'noisy full transfer reaches the originally selected refuge')
    expect(not flag('n011_escape_locked') and not flag('n011_first_mask'),
        'actual arrival releases lock and previous direction')
    # Prove reception and planning restart at the new hide, with fresh frames.
    call('Thief_UpdatePose');call('Thief_UpdateLaserMask',clock)
    active=var('thief_laser_active_mask')
    ch=next(i for i in range(4) if active&(1<<i))
    frames(ch)
    expect(var('thief_escape_requested'),'new hide again detects a freshly confirmed opponent')
    tick()
    expect(var('route_active') and not flag('n011_escape_locked'),
        'new hide resumes its own ordinary escape policy')
    lifecycles.append(dict(body=body,gyro_sign=sign,world=world,injected_frames=injected,result=result))
cases['noisy_full_transfers']=len(lifecycles)

fresh(11,1);frames(channel(1,0));tick()
if flag('n011_escape_locked'):
    ptr=0x2000c500;u.mem_write(ptr,b'KEY_STOP\0');call('Route_StopFault',ptr)
    expect(not var('route_active') and not flag('n011_escape_locked') and not flag('n011_first_mask'),
        'manual/real fault stopping clears transfer latch instead of defeating stop')
cases['manual_stop']=1
source=(ROOT/'User/App_ThiefStrategy.c').read_text(encoding='utf-8')
expect('else if(n011_escape_locked) status="PLAN_LOCK";' in source,
    'RUN8 diagnostic labels route lock; raw-live-monitor migration intentionally excluded')
report=dict(checks=checks,cases=cases,failures=failures,lifecycles=lifecycles,expected_before_failure=before,
    axf_sha256=hashlib.sha256(axf.read_bytes()).hexdigest(),test_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),hardware_tested=False)
dest=ROOT/('Reference/STABLE9_before_n011_first_plan/first_plan_before.json' if before else 'Tests/results_n011_first_plan_stable10.json')
dest.write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
print(json.dumps(dict(checks=checks,cases=cases,failures=failures,axf_sha256=report['axf_sha256'],hardware_tested=False),indent=2))
assert failures if before else not failures,failures
