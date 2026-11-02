"""Documents24: evidence-first exit handoff and progress-based timeout; synthetic hardware."""
from pathlib import Path
import sys,json,hashlib
src=Path(__file__).with_name('test_documents15.py').read_text(encoding='utf-8').split('parking_pwm=[]')[0]
src=src.replace("if '--before' in sys.argv:","if False:")
if '--before' in sys.argv:
    src=src.replace('exec(compile(runtime,',"runtime=runtime.replace(\"axf=ROOT/'MDK-ARM'/ROOT.name/(ROOT.name+'.axf')\",\"axf=ROOT/'Tests/before_documents24/original.axf'\")\nexec(compile(runtime,")
exec(compile(src,str(Path(__file__).with_name('test_documents15.py')),'exec'))
def pending(mode=1,sign=1):
    global yaw,gray_raw,gray_state,rear_valid,q
    ret('Route_AllWheelsStopped',1);fresh(37,3);rear_valid=1;member('thief_localization','yaw_cw_sign',sign)
    q=1 if mode==0 else -1
    setstep(37,20,0,mode,600);call('Route_Start');tick(20);tick(60)
    yaw=sign*q*5639;tick(380)
    if 'turn_axis_active' in symbols:
        # Complete the shared turn stage first, then reproduce a disturbed
        # pending exit. Never bypass/force the new angle gate in the fixture.
        yaw=sign*q*9000
        for _ in range(30):
            tick()
            if var('route_state')==7 and not var('turn_axis_active'):break
        check(var('route_state')==7 and not var('turn_axis_active'),'shared axis stage really completes before recovery replay')
        yaw=sign*q*5639
    gray_raw=0;gray_state=2;tick()
    for _ in range(6):tick()
    check(var('turn_exit_trim_started'),'replay begins actual missing-line rotation')
def slow_correction():
    global yaw,gray_raw,gray_state
    count=0
    for mode in [0,1]:
      for sign in [-1,1]:
        pending(mode,sign)
        start=clock
        for angle in range(5739,7730,100):
            yaw=sign*q*angle;tick(200)
        check(clock-start>2500 and var('route_active'),'fresh correction progress may take longer than2500ms')
        yaw=sign*q*(9000-271);gray_raw=0x60;gray_state=1
        ret('Route_AllWheelsStopped',0);targets.clear();tick()
        check(var('turn_exit_active') and not targets,'real line at2.71deg brakes before leaving actual trim')
        ret('Route_AllWheelsStopped',1)
        for _ in range(12):tick()
        check(not var('turn_exit_active') and member('thief_localization','phase')==1,'real line and2.71deg exit resume committed leg')
        count+=1
    return count
def deadline_line():
    global yaw,gray_raw,gray_state,clock
    pending();yaw=q*(9000-271);gray_raw=0x60;gray_state=1;clock+=2600
    for _ in range(12):tick()
    check(var('route_active') and not var('turn_exit_active'),'credible recovered line wins over elapsed deadline')
    return 1
def no_progress():
    global clock,rear_valid
    pending();clock+=2600;call('App_ThiefStrategy_Task')
    check(var('route_active'),'MONITOR4 actual trailing line keeps correction alive beyond2500ms')
    rear_valid=2;clock+=20;call('App_ThiefStrategy_Task');clock+=2000;call('App_ThiefStrategy_Task')
    check(not var('route_active') and reason()=='GRAY_BOTH_NO_LINE','continuous BOTH-board loss terminates after2000ms')
    return 1
def stalled_small_error():
    global yaw,clock
    pending();yaw=q*(9000-271);tick();targets.clear();ret('Route_AllWheelsStopped',0);clock+=2600;tick()
    check(var('route_active') and var('turn_exit_line_phase')==1,'stalled trim inside10deg changes to bounded search rather than permanent fault')
    for _ in range(3):tick()
    check(not targets,'stalled trim fallback must brake and measure stopped wheels before translation')
    ret('Route_AllWheelsStopped',1)
    for _ in range(5):tick()
    check(targets,'measured standstill permits existing rear-supported search')
    return 1
def n010_log_replay():
    global yaw,odom,clock,gray_raw,gray_state,rear_valid
    ret('Route_AllWheelsStopped',1);fresh(20,2);member('thief_localization','yaw_cw_sign',-1);rear_valid=1
    setstep(20,10,0,1,300,action=1);setstep(10,11,1,1,600,index=1)
    record(route,'MapRoute_t',dict(count=2));call('Route_Start')
    odom=300;clock+=600;call('Route_HandleJunction',clock,20,5,0x03f)
    tick(180);tick(20);tick(80)
    yaw=-4571;tick(340);gray_raw=0;gray_state=2;tick()
    for angle in range(4671,8730,100):
        yaw=-angle;tick(100)
    yaw=-9000;gray_raw=0x60;gray_state=1
    for _ in range(12):tick()
    check(var('route_active') and member('map_nav','step')==1,'actual N020->N010 right turn resumes N010->N011')
    check(member('thief_localization','confirmed_node')==10 and member('thief_localization','phase')==1,'real exit commits next edge without fabricating N011 arrival')
    return 1
failures=[];cases={}
for fn in [slow_correction,deadline_line,no_progress,stalled_small_error,n010_log_replay]:
    try:cases[fn.__name__]=fn();print(fn.__name__+': PASS',flush=True)
    except AssertionError as e:failures.append(dict(case=fn.__name__,error=str(e)));print(fn.__name__+': FAIL '+str(e),flush=True)
before='--before' in sys.argv
result=dict(checks=checks,failures=failures,cases=cases,hardware_tested=False,
 firmware_sha256=hashlib.sha256(axf.read_bytes()).hexdigest(),
 app_source_sha256=hashlib.sha256((ROOT/('Tests/before_documents24/User/App_ThiefStrategy.c' if before else 'User/App_ThiefStrategy.c')).read_bytes()).hexdigest(),
 test_script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
 helper_sha256={n:hashlib.sha256((ROOT/'Tests'/n).read_bytes()).hexdigest() for n in ['test_documents15.py','test_app_runtime.py','test_escape_completion.py','test_documents13.py']})
out=ROOT/'Tests'/('before_documents24/regression_before.json' if before else 'results_documents24.json')
out.write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8');print(json.dumps(result,indent=2));sys.exit(bool(failures))
