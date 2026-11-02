"""Actual ARM turn-exit/recovery regression; explicit synthetic hardware.
Actual log YAW samples are replayed with line-like and word-like gray fixtures;
the log does not contain every raw gray sample, so this is not a full sensor replay.
--before uses preserved STABLE4 with identical desired behavior.
"""
from pathlib import Path
import sys,re
setup=Path(__file__).with_name('test_escape_completion.py').read_text(encoding='utf-8').split('# Log regression:')[0]
before='--before' in sys.argv
if before:
    setup=setup.replace('exec(compile(setup.split(',
        'setup=setup.replace("axf=ROOT/\'MDK-ARM\'/ROOT.name/(ROOT.name+\'.axf\')", "axf=ROOT/\'Reference/STABLE4_before_turn_exit/MDK-ARM\'/ROOT.name/(ROOT.name+\'.axf\')")\nexec(compile(setup.split(',1)
exec(compile(setup,str(Path(__file__).with_name('test_escape_completion.py')),'exec'))
assertions=0;failures=[];commands=[]
def expect(v,msg):
    global assertions
    assertions+=1
    if not v:failures.append(msg)
def signed(v,bits=32):return v-(1<<bits) if v>=(1<<(bits-1)) else v
def capture():
    commands.append(tuple(signed(u.reg_read(r)) for r in [UC_ARM_REG_R0,UC_ARM_REG_R1,UC_ARM_REG_R2,UC_ARM_REG_R3]))
    return 0
hook('TravelFrame_SetWheelTargets',capture)
def step(dt=20):
    global clock
    clock+=dt;call('App_ThiefStrategy_Task')
def enter_corner(mode=0,sign=-1,right=False,start=-3015):
    global yaw,odom,gray_raw,gray_state,rear_raw,rear_gray
    fresh(15,2*mode);member('thief_localization','yaw_cw_sign',sign)
    yaw=start;gray_raw=0x60;gray_state=1;rear_raw=0x60;rear_gray=1
    setstep(15,6,0,mode,300,action=1 if right else -1)
    setstep(6,7 if right else 4,1 if right else 3,mode,600 if right else 300,index=1)
    if not right:record(steps+size(types['MapRouteStep_t']),'MapRouteStep_t',dict(segment_count=2))
    record(route,'MapRoute_t',dict(count=2));call('Route_Start')
    check(var('route_active'),f'fixture route must be structurally valid: {reason()}')
    odom=300
    call('Route_HandleJunction',clock,20,5 if right else 4,0x073 if right else 0xce0)
    for _ in range(35):
        if var('route_state') in [5,6]:break
        step()
    check(var('route_state') in [5,6], f'fixture corner state={var("route_state")} reason={reason()} seen={member("map_nav","seen_ports")} arrived={member("map_nav","arrived")}')
    quarter=-1 if var('route_state')==6 else 1
    commands.clear();events.clear()
    yaw=(start+sign*quarter*4647+18000)%36000-18000;step()
    return quarter

# Reproduce the observed stale-angle handoff: 46.47 degrees remained unchanged
# while three front-valid samples used to advance N006->N004.
log=Path('C:/Users/Administrator/Desktop/小车串口_20261028_230118.txt').read_text(encoding='utf-8-sig')
actual_start=int(re.findall(r'STATE=TURN_LEFT JUNCTION=11 YAW_START=(-?\d+)',log)[-1])
samples={}
for m in re.finditer(r'MT3,8,\d+,[^\r\n*]+\*[0-9A-Fa-f]{4}',log):
    f=m[0].split('*')[0].split(',');samples[int(f[2])]=(int(f[3]),int(f[11]))
expect(samples[133][1]-actual_start==4647,'log main-spin delta is 46.47 degrees')
enter_corner(start=actual_start)
expect('Route_DriveStraight' not in events,'main-spin exit cannot emit the previous unconditional forward burst')
previous=samples[133][0]
for seq in [134,136,138,139,140]:
    stamp,yaw=samples[seq];step(max(20,stamp-previous));previous=stamp
    expect(member('map_nav','step')==0 and var('route_state')==7,
           f'actual YAW seq={seq} cannot prematurely publish N006->N004')
expect(commands and all(a+b+c+d==0 for a,b,c,d in commands),'observed under-turn sequence commands rotation only')
call('Thief_UpdatePose');expect(member('thief_pose','valid')==0,'angle acquisition cannot project a completed exit pose')

# Left/right, forward/reverse, either gyro sign, including +/-180-degree wrap.
for mode in [0,1]:
  for sign in [-1,1]:
    for right in [False,True]:
      start=17000;quarter=enter_corner(mode,sign,right,start)
      for progress,front,rear,state in [(7559,0x60,0x60,1),(9000,1,0x60,1),
          (9000,0xf40,0x60,4),(9000,0x050,0x60,1),(9000,0x60,1,1)]:
        gray_raw=front;gray_state=state;rear_raw=rear;rear_gray=1
        yaw=(start+sign*quarter*progress+18000)%36000-18000;commands.clear()
        for _ in range(4):step()
        expect(var('route_state')==7 and member('map_nav','step')==0,
               f'weak/unpaired/under-angle fixture cannot resume: mode={mode} sign={sign} right={right} raw={front:03x},{rear:03x}')
        expect(all(a+b+c+d==0 for a,b,c,d in commands),'no translational search during exit acquisition')
      gray_raw=rear_raw=0x60;gray_state=rear_gray=1
      step();step()
      gray_raw=0x050;step()
      expect(var('route_state')==7,'fragmented third sample resets consecutive confirmation')
      gray_raw=0x60
      for _ in range(3):step()
      expect(var('route_state')==2,'three compatible line samples near 90 degrees restore original PID')
      for _ in range(3):step()
      expect(member('map_nav','step')==1,'normal subsequent line confirmation advances the route')

# Preserve the user's "PID handles the remaining offset" requirement.
# At 90 degrees, compact but incompatible board positions remain uncertain;
# a consistent off-center line is accepted without demanding zero error.
enter_corner();yaw=(actual_start+9000+18000)%36000-18000
gray_raw=0x008;gray_state=1;rear_raw=0x100;rear_gray=1
offset_rear_error=250
def offset_front():
    record(u.reg_read(UC_ARM_REG_R0),'RouteGraySample_t',dict(front_raw12=gray_raw,
        front_error_x100=-250,position_error_x100=-250,heading_error_x100=0))
    return gray_state
def offset_rear():
    record(u.reg_read(UC_ARM_REG_R0),'RouteGraySample_t',dict(rear_raw12=rear_raw,rear_error_x100=offset_rear_error))
    return rear_gray
hook('Route_ReadFront',offset_front);hook('Route_ReadRear',offset_rear)
for _ in range(4):step()
expect(var('route_state')==7,'opposed board errors cannot claim compatible exit line')
offset_rear_error=-250
for _ in range(3):step()
expect(var('route_state')==2,'consistent -250 offset returns to original PID instead of requiring perfect centering')
hook('Route_ReadFront',readgray);hook('Route_ReadRear',read_rear)

# A weak single rear edge cannot propel the car deeper into printed text.
fresh(38,0);setstep(38,21,0,0,600);call('Route_Start')
gray_raw=0;gray_state=2;rear_raw=1;rear_gray=1
call('Route_EnterFrontRecovery',clock,2);commands.clear()
for _ in range(6):step()
expect(commands and all(a+b+c+d==0 for a,b,c,d in commands),'rear 001 alone cannot authorize forward recovery')
gray_raw=1;gray_state=1
for _ in range(4):step()
expect(var('route_state')==8,'leading 001 cannot falsely re-enter normal follow')
gray_raw=0x60;gray_state=1;rear_raw=0;rear_gray=2
step();step()
expect(var('route_state')==2,'real compact leading line still recovers with temporarily missing rear')

# Keep the user-defined two-second BOTH-white stop and real I2C fault handling.
enter_corner();gray_raw=rear_raw=0;gray_state=rear_gray=2;step()
clock+=2000;call('App_ThiefStrategy_Task')
expect(not var('route_active') and reason()=='GRAY_BOTH_NO_LINE','continuous both-white exit acquisition still stops after two seconds')
enter_corner();gray_state=7;step();step(200)
expect(not var('route_active') and reason()=='GRAY_I2C','persistent physical gray I2C failure still brakes/faults')

report=dict(firmware='UARTCH4_16V_STABLE4' if before else 'UARTCH4_16V_STABLE5',
    checks=assertions,failures=failures,expected_before_failure=before,
    actual_log_yaw={str(k):samples[k] for k in [133,134,136,138,139,140]},
    axf_sha256=hashlib.sha256(axf.read_bytes()).hexdigest(),
    test_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
    log_sha256=hashlib.sha256(Path('C:/Users/Administrator/Desktop/小车串口_20261028_230118.txt').read_bytes()).hexdigest(),
    hardware_tested=False)
target=ROOT/('Reference/STABLE4_before_turn_exit/results_turn_line.json' if before else 'Tests/results_turn_line_stable5.json')
target.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print(json.dumps(report,ensure_ascii=False,indent=2))
assert bool(failures)==before,failures
