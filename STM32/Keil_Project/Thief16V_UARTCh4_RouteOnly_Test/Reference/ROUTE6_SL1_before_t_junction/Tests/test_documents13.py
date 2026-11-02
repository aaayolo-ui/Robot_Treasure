"""Actual ARM application, synthetic hardware. No physical line-loss claim."""
from pathlib import Path
import sys,json
setup=Path(__file__).with_name('test_escape_completion.py').read_text(encoding='utf-8-sig')
exec(compile(setup.split('# Log regression:')[0],str(Path(__file__).with_name('test_escape_completion.py')),'exec'))
rear_valid=1;rear_error=0;targets=[]
def rear_read():
    record(u.reg_read(UC_ARM_REG_R0),'RouteGraySample_t',dict(rear_raw12=0x60 if rear_valid==1 else 0,
         rear_error_x100=rear_error));return rear_valid
hook('Route_ReadRear',rear_read)
def target_capture():
    targets.append(tuple(int.from_bytes(u.reg_read(r).to_bytes(4,'little'),'little',signed=True)
        for r in [UC_ARM_REG_R0,UC_ARM_REG_R1,UC_ARM_REG_R2,UC_ARM_REG_R3]));return 0
hook('TravelFrame_SetWheelTargets',target_capture)

def start(node=36,body=3,goal=35,heading=3,mode=0):
    global rear_valid,rear_error
    rear_valid=1;rear_error=0;targets.clear();fresh(node,body)
    call('ThiefMapPlanner_HasRoad',node,heading,0x2000c400,0x2000c402)
    length=get(0x2000c402,2)
    setstep(node,goal,heading,mode,length);call('Route_Start');tick()

def straight_continuity():
    global odom,gray_raw,gray_state
    fresh(38,1)
    setstep(38,39,1,0,300,action=0);setstep(39,40,1,0,300,index=1)
    record(route,'MapRoute_t',dict(count=2))
    call('Route_Start');tick()
    odom=250
    for _ in range(12):tick()
    before=sum(targets[-1][:2])//2
    check(before>1500,'establish cruising before straight junction')
    odom=300;call('Route_HandleJunction',clock,20,5,0x77)
    targets.clear();gray_raw=0x60;gray_state=1
    for _ in range(4):tick()
    check(member('map_nav','step')==1,'straight pass starts next real leg')
    check(min(sum(t[:2])//2 for t in targets)>1000,'straight next leg does not drop to 100 RPM')

def corroborated_recovery():
    global odom,gray_raw,gray_state,rear_error
    for node,body,goal,h,mode in [(36,3,35,3,0),(38,1,37,3,1)]:
        start(node,body,goal,h,mode);odom=8;gray_raw=0;gray_state=2;rear_error=-40
        targets.clear();events.clear()
        for _ in range(15):
            odom+=2;tick()
            check(member('thief_localization','confirmed_node')==node,'no fabricated node during missing front')
        check(targets and 'Route_BrakeControl' not in events,'rear + mapped exit permits controlled motion beyond 200ms')
        check(all(0<sum(t[:2])//2<=500 for t in targets),'recovery stays at <=50 RPM')
        gray_raw=0x60;gray_state=1
        for _ in range(3):tick()
        check(var('route_state')==2,'three actual front samples restore follow')

def both_missing_scan():
    global odom,gray_raw,gray_state,rear_valid
    start();odom=8;rear_valid=2;gray_raw=0;gray_state=2;events.clear();targets.clear()
    tick();check(targets and sum(targets[-1][:2])>0,'MONITOR4 mapped short loss permits bounded forward probing')
    for _ in range(3):tick()
    check(targets and all(0<sum(t[:2])<=1000 for t in targets),'blind probe remains at50RPM mean before100mm cap')
    gray_raw=0x60;gray_state=1
    for _ in range(7):tick()
    check(var('route_state')==2,'real front readings end stationary scan and restore follow')

def unsupported_context():
    global odom,gray_raw,gray_state,rear_valid
    for case in ['mid_no_rear','i2c','unknown_yaw']:
        start();odom=150 if case=='mid_no_rear' else 8
        rear_valid=2 if case=='mid_no_rear' else 1
        if case=='unknown_yaw':member('thief_localization','yaw_cw_sign',0)
        gray_raw=0;gray_state=7 if case=='i2c' else 2;events.clear()
        tick()
        if case=='mid_no_rear':
            check(var('map_gray_active') and targets and sum(targets[-1][:2])>0,'MONITOR4 known road permits brief middle-of-road probing without inventing nodes')
        else:check('Route_BrakeControl' in events,case+' cannot rely on uncorroborated coordinates')

def bounded_recovery():
    global odom,gray_raw,gray_state,clock,rear_valid
    # User changed the rule: distance/time alone cannot lock rear-supported tracking.
    start();odom=8;gray_raw=0;gray_state=2;tick();odom=89;tick(1001)
    check(var('route_active')==1,'rear-visible recovery survives former80mm and1000ms stop bounds')
    start();odom=8;gray_raw=0;gray_state=2;rear_valid=2;tick()
    clock+=2001;call('App_ThiefStrategy_Task')
    check(var('route_active')==0 and reason()=='MAP_GRAY_REACQUIRE_FAILED','continuous both-board loss still stops after2000ms')


def obstacle_priority():
    global odom,gray_raw,gray_state,clock
    start();odom=8;gray_raw=0;gray_state=2;tick()
    check('map_gray_active' in symbols and var('map_gray_active')==1,'map recovery entered')
    call('Thief_UpdatePose');call('Thief_UpdateLaserMask',clock)
    clock+=1;call('Thief_OnLaserFrame',0,150,100,1,clock);tick()
    check(var('route_state')==12 and not var('map_gray_active'),'new laser obstacle preempts map recovery')

failures=[];passed=[]
for fn in [straight_continuity,corroborated_recovery,both_missing_scan,unsupported_context,bounded_recovery,obstacle_priority]:
    try:fn();passed.append(fn.__name__);print(fn.__name__+': PASS',flush=True)
    except AssertionError as e:failures.append(dict(case=fn.__name__,error=str(e)));print(fn.__name__+': FAIL '+str(e),flush=True)
result=dict(checks=checks,failures=failures,passed=passed,hardware_tested=False,
 firmware_sha256=hashlib.sha256(axf.read_bytes()).hexdigest(),
 app_source_sha256=hashlib.sha256((ROOT/'User/App_ThiefStrategy.c').read_bytes()).hexdigest(),
 test_script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
 helper_sha256={n:hashlib.sha256((ROOT/'Tests'/n).read_bytes()).hexdigest() for n in ['test_app_runtime.py','test_escape_completion.py']})
out=ROOT/'Tests'/('before_documents13/regression_before.json' if '--before' in sys.argv else 'results_documents13.json')
out.write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8');print(json.dumps(result,indent=2))
sys.exit(bool(failures))
