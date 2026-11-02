"""RUN12 unified gray recovery and actual route history; synthetic hardware only."""
from pathlib import Path
import sys,json,hashlib
src=Path(__file__).with_name('test_documents15.py').read_text(encoding='utf-8').split('parking_pwm=[]')[0]
src=src.replace("if '--before' in sys.argv:","if False:")
if '--before' in sys.argv:
    src=src.replace('exec(compile(runtime,',"runtime=runtime.replace(\"axf=ROOT/'MDK-ARM'/ROOT.name/(ROOT.name+'.axf')\",\"axf=ROOT/'Tests/before_documents22/original.axf'\")\nexec(compile(runtime,")
exec(compile(src,str(Path(__file__).with_name('test_documents15.py')),'exec'))


def n038_east_first():
    paths=[]
    for body in [1,3]:
        fresh(38,body)
        check(call('ThiefMapPlanner_PlanEscape',38,body,1,0,clock,plan),'north threat has an escape')
        check(at(steps,'MapRouteStep_t','to_node')==39 and at(steps+16,'MapRouteStep_t','to_node')==40,'MONITOR2 north threat selects actual east runway')
        check(at(steps,'MapRouteStep_t','drive_mode')==int(body==3),'east runway preserves the actual body frame without an unnecessary pivot')
        goal=member('thief_plan','goal_node');call('Route_Start');result=complete_route()
        check(result['node']==goal and result['fixes'][:2]==[39,40],'east runway completes actual selected refuge route')
        paths.append(result)
    return paths

def exit_distance_scan(mode=1,sign=1,entry_error=0,persistent=False):
    global clock,yaw,odom,gray_raw,gray_state,rear_valid,rear_error
    fresh(37,3);rear_valid=1;rear_error=-60
    member('thief_localization','yaw_cw_sign',sign);yaw=entry_error
    q=1 if mode==0 else -1
    setstep(37,20,0,mode,600);call('Route_Start');tick(20);tick(60)
    yaw=entry_error+sign*q*5639;tick(380)
    check(var('route_state')==7,'replay N037 rear-leading left turn handoff at56.39degrees')
    gray_raw=0;gray_state=2;tick()
    for _ in range(100):
        if var('turn_exit_line_phase'):break
        if member('turn_exit_heading','spinning'):
            v=member('turn_exit_heading','last_turn');v=v-65536 if v>=32768 else v
            yaw=(yaw+sign*round(v/5)+18000)%36000-18000
        tick()
    check(var('turn_exit_line_phase')==1,'observed gyro correction enters line search')
    check(abs((yaw-sign*q*9000+18000)%36000-18000)<=1000,'MONITOR4 missing exit may probe within user10degree road axis tolerance')
    rear_valid=2 #100mm cap applies to blind probing, not real trailing-line guidance.
    # User requested100mm:80/99 continue translation,100 begins measured-stop scan.
    for distance in [80,99]:
        odom=distance;tick()
        check(var('turn_exit_line_phase')==1,'below100mm remains translation search')
    odom=100;targets.clear();events.clear();tick()
    check(var('turn_exit_active') and var('turn_exit_line_phase')==2,'100mm limit starts stationary scan instead of permanent fault')
    check('Route_BrakeControl' in events and not targets,'scan begins by braking all wheels')
    ret('Route_AllWheelsStopped',0)
    for _ in range(3):tick()
    check(not targets,'scan does not turn while wheels still moving')
    ret('Route_AllWheelsStopped',1)
    for _ in range(5):tick()
    check(targets and all(sum(t)==0 for t in targets),'blind distance-limit recovery rotates with zero translation')
    if persistent:
        targets.clear();clock+=2501;call('App_ThiefStrategy_Task')
        check(not var('route_active') and reason()=='GRAY_BOTH_NO_LINE','persistent BOTH-board loss stops after2000ms')
        check(not targets,'expired scan cannot issue another drive command')
        return 1
    gray_raw=0x60;gray_state=1
    for _ in range(10):tick()
    check(not var('turn_exit_active') and member('thief_localization','phase')==1,'actual outgoing gray and yaw resume committed travel')
    return 1


def scan_matrix():
    count=0
    for mode in [0,1]:
     for sign in [-1,1]:
      for error in [-700,0,700]:
        exit_distance_scan(mode,sign,error);count+=1
    exit_distance_scan(persistent=True)
    return count

def east_obstacle_override():
    for body in [1,3]:
      for blocked in [False,True]:
        fresh(38,body)
        if blocked:call('ThiefIntercept_BlockLeg',38,39,clock)
        mask=1 if blocked else 3
        check(call('ThiefMapPlanner_PlanEscape',38,body,mask,0,clock,plan),'east blocked still finds another refuge route')
        check(at(steps,'MapRouteStep_t','to_node')!=39,'new obstacle overrides direct N039 preference')
    return 4

failures=[];passed=[];cases={}
for fn in [n038_east_first,exit_distance_scan]+([] if '--before' in sys.argv else [scan_matrix,east_obstacle_override]):
    try:cases[fn.__name__]=fn();passed.append(fn.__name__);print(fn.__name__+': PASS',flush=True)
    except AssertionError as e:failures.append(dict(case=fn.__name__,error=str(e)));print(fn.__name__+': FAIL '+str(e),flush=True)
before='--before' in sys.argv
result=dict(checks=checks,failures=failures,passed=passed,cases=cases,hardware_tested=False,
 firmware_sha256=hashlib.sha256(axf.read_bytes()).hexdigest(),
 app_source_sha256=hashlib.sha256((ROOT/('Tests/before_documents22/User/App_ThiefStrategy.c' if before else 'User/App_ThiefStrategy.c')).read_bytes()).hexdigest(),
 test_script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
 helper_sha256={n:hashlib.sha256((ROOT/'Tests'/n).read_bytes()).hexdigest() for n in ['test_documents15.py','test_app_runtime.py','test_escape_completion.py','test_documents13.py']})
out=ROOT/'Tests'/('before_documents22/regression_before.json' if before else 'results_documents22.json')
out.write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8');print(json.dumps(result,indent=2));sys.exit(bool(failures))
