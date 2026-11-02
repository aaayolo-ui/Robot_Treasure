"""Documents16: real linked ARM code; sensor/vehicle inputs are synthetic."""
from pathlib import Path
import sys,json,hashlib
source=Path(__file__).with_name('test_documents15.py').read_text(encoding='utf-8').split('def wrap(a):')[0]
source=source.replace("if '--before' in sys.argv:","if False:")
if '--before' in sys.argv:
    source=source.replace("exec(compile(runtime,", "runtime=runtime.replace(\"axf=ROOT/'MDK-ARM'/ROOT.name/(ROOT.name+'.axf')\",\"axf=ROOT/'Tests/before_documents16/original.axf'\")\nexec(compile(runtime,")
exec(compile(source,str(Path(__file__).with_name('test_documents15.py')),'exec'))

rx_enabled=1;rx_calls=[]
def rxset():
    global rx_enabled
    rx_enabled=u.reg_read(UC_ARM_REG_R0);rx_calls.append(rx_enabled);return 1
hook('LaserRx_SetEnabled',rxset);hook('LaserRx_IsEnabled',lambda:rx_enabled)

def parking(node=38,body=1,mode=0,error=293):
    global yaw,odom,clock,gray_raw,gray_state,rear_valid,rx_enabled
    rx_calls.clear();rx_enabled=1;rear_valid=1
    ret('Route_AllWheelsStopped',1)
    travel=(body+2*mode)%4
    check(call('ThiefMapPlanner_HasRoad',node,(travel+2)%4,0x2000c400,0x2000c402)==1,'incoming road exists')
    source_node=get(0x2000c400,2);length=get(0x2000c402,2)
    fresh(source_node,body);member('thief_localization','yaw_cw_sign',-1)
    setstep(source_node,node,travel,mode,length);call('Route_Start')
    yaw=error;odom=length;clock+=400
    call('Route_HandleJunction',clock,20,3,0xfff)
    check(var('route_state')==13,'same nominal hide approach')
    tick(179);check(var('route_state')==13,'full180ms retained')
    tick(1);check(var('hide_alignment_pending')==1,'parking trim started')

def plateau():
    global yaw
    parking()
    # Real log plateaus at2.93deg despite low-speed trim commands.
    for _ in range(145):tick()
    check(var('thief_strategy_state')==2,'2.93deg plateau must finish as hiding, never permanent FAULT')
    check(rx_enabled and 0 not in rx_calls,'ordinary parking never disables laser RX')
    check(member('thief_localization','heading_error_x100')==293,'never fake an aligned gyro')
    check(not var('hide_alignment_pending'),'bounded trim ends instead of hanging')
    clock_before=clock
    for _ in range(4):tick()
    check(not var('route_active'),'timeout alone cannot invent a threat')

def threat_matrix():
    global clock,yaw
    completed=[];ignored=[]
    for node,axes in [(11,[1,3]),(38,[1,3]),(26,[0,2]),(35,[1,3]),(63,[0,2])]:
      for body in axes:
       for mode in [0,1]:
        for direction in range(4):
            parking(node,body,mode,800)
            clock+=1;call('Thief_UpdatePose');call('Thief_UpdateLaserMask',clock)
            bit=1<<direction
            expected=call('ThiefMapPlanner_HasRoad',node,(body+[0,3,1,2][direction])%4,0,0)
            if not expected:
                for _ in range(3):
                    clock+=20;call('Thief_OnLaserFrame',direction,150,100,1,clock)
                check(not var('thief_escape_requested'),'outward/no-road ray remains ignored during trim')
                ignored.append([node,body,mode,direction]);continue
            check(var('thief_laser_active_mask')&bit,'real connected laser ray must be active during parking trim')
            check(member('thief_pose','valid')==1,'confirmed hide pose remains available while correcting')
            ret('Route_AllWheelsStopped',0)
            for n in range(3):
                clock+=20;call('Thief_OnLaserFrame',direction,150,100,1,clock)
                check(bool(var('thief_escape_requested'))==(n==2),'same three-frame hidden threat confirmation')
            tick()
            check(not var('hide_alignment_pending'),'confirmed threat preempts trim at8deg')
            check(not var('route_active'),'no departure before measured standstill')
            check('Route_BrakeControl' in events,'threat first brakes rotation')
            # Obstacle disappears while braking: preserve already-confirmed request.
            for _ in range(20):tick()
            check(not var('route_active'),'long brake wait does not cause uncontrolled restart')
            ret('Route_AllWheelsStopped',1)
            tick();tick()
            result=complete_route()
            check(result['node']!=node,'confirmed threat reaches a different refuge')
            check(rx_enabled and 0 not in rx_calls,'escape path never switches off laser reception')
            completed.append([node,body,mode,direction,result['node']])
    return dict(completed=completed,ignored=ignored)

def noise_and_boundary():
    global clock
    for distance,confidence in [(150,0),(0,100),(1100,100)]:
        parking();clock+=1;call('Thief_UpdatePose');call('Thief_UpdateLaserMask',clock)
        for _ in range(6):
            clock+=20;call('Thief_OnLaserFrame',0,distance,confidence,1,clock)
        check(not var('thief_escape_requested'),'quality/range filters preserved')
    parking();clock+=1;call('Thief_UpdatePose');call('Thief_UpdateLaserMask',clock)
    for distance in [150,0,150,0,150]:
        clock+=20;call('Thief_OnLaserFrame',0,distance,100,1,clock)
    check(not var('thief_escape_requested'),'isolated returns cannot preempt trim')
    # A connected outward ray can still hit beyond field bounds.
    parking(11,1);clock+=1;call('Thief_UpdatePose');call('Thief_UpdateLaserMask',clock)
    for _ in range(3):
        clock+=20;call('Thief_OnLaserFrame',3,900,100,1,clock)
    check(not var('thief_escape_requested'),'outside-field hit remains ignored')

failures=[];passed=[];matrix={}
for fn in [plateau,threat_matrix,noise_and_boundary]:
    try:
        value=fn()
        if value:matrix=value
        passed.append(fn.__name__);print(fn.__name__+': PASS',flush=True)
    except AssertionError as e:
        failures.append(dict(case=fn.__name__,error=str(e)));print(fn.__name__+': FAIL '+str(e),flush=True)
before='--before' in sys.argv
result=dict(checks=checks,failures=failures,passed=passed,matrix=matrix,hardware_tested=False,
 firmware_sha256=hashlib.sha256(axf.read_bytes()).hexdigest(),
 app_source_sha256=hashlib.sha256((ROOT/('Tests/before_documents16/User/App_ThiefStrategy.c' if before else 'User/App_ThiefStrategy.c')).read_bytes()).hexdigest(),
 test_script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
 helper_sha256={n:hashlib.sha256((ROOT/'Tests'/n).read_bytes()).hexdigest() for n in
 ['test_documents15.py','test_app_runtime.py','test_escape_completion.py','test_documents13.py']})
out=ROOT/'Tests'/('before_documents16/regression_before.json' if before else 'results_documents16.json')
out.write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
print(json.dumps({k:v for k,v in result.items() if k!='matrix'},indent=2))
sys.exit(bool(failures))
