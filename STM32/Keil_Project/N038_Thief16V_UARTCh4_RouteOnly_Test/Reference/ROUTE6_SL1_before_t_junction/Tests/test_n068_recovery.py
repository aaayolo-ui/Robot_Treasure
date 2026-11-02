"""Logged N068 stop: actual ARM gray classification and stopped-wheel predicate.

Only sensor reads and wheel physics are simulated. A brake stops the synthetic
wheels but does NOT refresh their cached feedback; the application must do so.
No task state is forced to a successful exit. --before runs the saved ROUTE3.
"""
from pathlib import Path
import sys, json, hashlib

ROOT = Path(__file__).absolute().parents[1]
runtime = (ROOT/'Tests/test_app_runtime.py').read_text(encoding='utf-8').split('def documents8_cases():')[0]
if '--before' in sys.argv:
    runtime = runtime.replace("axf=ROOT/'MDK-ARM'/ROOT.name/(ROOT.name+'.axf')",
        "axf=ROOT/'Reference/ROUTE3_before_recovery_fix/MDK-ARM'/ROOT.name/(ROOT.name+'.axf')")
exec(compile(runtime, str(ROOT/'Tests/test_app_runtime.py'), 'exec'))
fixture = (ROOT/'Tests/test_escape_completion.py').read_text(encoding='utf-8-sig').split('# Log regression:')[0]
exec(compile(fixture[fixture.index('for n in ['):], str(ROOT/'Tests/test_escape_completion.py'), 'exec'))

# Exercise real Route_ReadFront/Rear, ClassifyPosition, gray extraction, and
# Route_AllWheelsStopped. The old fixture stubbed all of these as ideal success.
for n in ['Route_ReadFront', 'Route_ReadRear', 'Route_AllWheelsStopped', 'Route_BrakeControl']:
    if n in hooks:
        u.hook_del(hooks.pop(n))
front_raw = 0x60
back_raw = 0x60
physical_rpm = 0
refreshes = 0
commands = []
feedback = 0x2000c600
stride = size(types['ChassisWheelFeedback_t'])

def gray_read(raw):
    put(u.reg_read(UC_ARM_REG_R0), 2, raw)
    return 0

hook('TravelFrame_ReadLeading', lambda: gray_read(front_raw))
hook('TravelFrame_ReadTrailing', lambda: gray_read(back_raw))
hook('Chassis_GetWheelFeedback', lambda: feedback + u.reg_read(UC_ARM_REG_R0)*stride)

def cache(rpm):
    for wheel in range(4):
        record(feedback + wheel*stride, 'ChassisWheelFeedback_t', dict(rpm_x10=rpm))

def refresh():
    global refreshes
    refreshes += 1
    cache(physical_rpm)
    return 0

def brake():
    global physical_rpm
    physical_rpm = 0
    commands.append(('brake', 0, 0))
    return 0

def targets():
    global physical_rpm
    values = [u.reg_read(r) for r in [UC_ARM_REG_R0, UC_ARM_REG_R1]]
    left, right = [v-2**32 if v >= 2**31 else v for v in values]
    physical_rpm = max(abs(left), abs(right))
    commands.append(('drive', left, right))
    return 0

hook('Chassis_UpdateFeedback', refresh)
hook('ChassisSpeedControl_Update', refresh)
hook('ChassisSpeedControl_Brake', brake)
hook('Chassis_Brake', brake)
hook('TravelFrame_SetWheelTargets', targets)
ret('ChassisSpeedControl_IsEnabled', 1)
trail = symbols['route_trail']

def tm(key):
    return at(trail, 'ThiefRouteTrail', key)

def prepare(edge=0x007, mode=0):
    global front_raw, back_raw, clock, odom, refreshes, physical_rpm
    front_raw = back_raw = 0x60
    fresh(84, 2*mode)
    call('ThiefRouteTrail_Begin', trail, 38, 0)
    # Actual connected outbound edges, not future planned nodes. Returning
    # from the last partial edge reaches N068 then continues straight to N058.
    for a, b, h, length in [(38,39,1,300), (39,40,1,300), (40,46,2,300),
                            (46,58,2,300), (58,68,2,300)]:
        setstep(a,b,h,mode ^ 1,length)
        call('ThiefRouteTrail_Stage',trail,steps,0x63,0x60,0,length)
        check(call('ThiefRouteTrail_Commit',trail)==1, 'build confirmed actual history')
    setstep(68,84,2,mode ^ 1,600)
    partial=0x2000c300
    u.mem_write(partial,bytes(u.mem_read(steps,size(types['MapRouteStep_t']))))
    check(call('ThiefRouteTrail_BuildReturn',trail,partial,plan)==1, 'reverse real recorded route')
    var('thief_strategy_state',3)
    call('Route_Start')
    for _ in range(8): tick()
    check(var('route_state')==2 and var('junction_armed'), 'real centered line arms return leg')
    odom=600
    front_raw=edge
    call('Route_HandleJunction',clock,20,5 if edge==7 else 4,edge)
    check(var('route_state')==11 and tm('return_pending'), 'real gray N068 stages straight return')
    check(member('map_nav','step')==0 and tm('return_left')==6, 'N068 not committed at first detection')
    back_raw=0x8C0
    physical_rpm=900
    cache(900)
    commands.clear()
    refreshes=0

failures=[]
cases=[]

def finish_home():
    global tick
    ordinary=tick
    def sensors(dt=20):
        global front_raw,back_raw
        front_raw=gray_raw
        back_raw=0x60
        ordinary(dt)
    tick=sensors
    try:
        result=complete_route()
    finally:
        tick=ordinary
    check(result['node']==38 and result['body']%2==0, 'recovered N068 completes full recorded return to vertical N038')
    return result

def edge_line():
    global front_raw, back_raw
    for mode in [0,1]:
        for edge in [0x007,0xE00]:
            prepare(edge,mode)
            for _ in range(12):
                previous=refreshes
                tick()
                check(refreshes-previous<=1, 'one encoder feedback sample per recovery cycle; do not drain encoder delta twice')
            moving=[x for x in commands if x[0]=='drive' and x[1]+x[2]>0]
            check(moving, 'N068 compact edge line must produce recovery motion, not permanent zero')
            check(all(0 < (l+r)//2 <= 500 for _,l,r in moving), 'reuse original 50RPM gray recovery')
            check(all((l-r)>0 if edge==7 else (l-r)<0 for _,l,r in moving), 'edge error steers toward actual black line')
            check(member('map_nav','step')==0 and tm('return_left')==6,
                  'edge stripe alone cannot commit N068 or invent next node')
            check(var('trace_pass_pending')==1, 'retain original junction encoder anchor')
            front_raw=back_raw=0x60
            for _ in range(8): tick()
            check(member('map_nav','step')==1 and tm('return_left')==5,
                  'actual centered samples commit exactly N068 then resume N058')
            check(not tm('return_pending') and not var('trace_pass_pending'), 'consume pending node and anchor once')
            cases.append(dict(case='edge_line',mode=mode,raw=edge,motion_commands=len(moving),return_home=finish_home()))

def brake_feedback():
    global front_raw, back_raw
    prepare()
    front_raw=0xFFF
    # A broad intersection is not a line-following stripe. With no reliable
    # rear stripe the controller must brake, measure stopped, then sweep only.
    for _ in range(9): tick()
    check(refreshes>0, 'recovery must refresh cached wheel speed while braking')
    check(call('Route_AllWheelsStopped') in [0,1], 'real stopped predicate remains callable')
    sweep=[x for x in commands if x[0]=='drive']
    check(sweep and any(l==-r and l!=0 for _,l,r in sweep), 'measured standstill releases real stationary sweep')
    check(all(l+r==0 for _,l,r in sweep), 'wide front and mixed rear cannot authorize blind forward drive')
    check(member('map_nav','step')==0 and tm('return_left')==6, 'sweep never consumes history')
    front_raw=0x007
    for _ in range(10): tick()
    check(any(x[0]=='drive' and x[1]+x[2]>0 for x in commands), 'real edge line after sweep resumes gray recovery')
    front_raw=back_raw=0x60
    for _ in range(8): tick()
    check(member('map_nav','step')==1 and tm('return_left')==5, 'sweep recovery returns to pending straight-pass owner')
    cases.append(dict(case='brake_feedback',refreshes=refreshes))

for fn in [edge_line,brake_feedback]:
    try:
        fn()
        print(fn.__name__+': PASS',flush=True)
    except AssertionError as e:
        failures.append(dict(case=fn.__name__,error=str(e)))
        print(fn.__name__+': FAIL '+str(e),flush=True)

out=ROOT/'Tests'/('n068_before.json' if '--before' in sys.argv else 'results_n068_recovery.json')
report=dict(checks=checks,failures=failures,cases=cases,axf_sha256=hashlib.sha256(axf.read_bytes()).hexdigest(),
            real_gray_classification=True,real_stopped_predicate=True,hardware_tested=False)
out.write_text(json.dumps(report,indent=2),encoding='utf-8')
print(json.dumps(report,indent=2))
sys.exit(bool(failures))
