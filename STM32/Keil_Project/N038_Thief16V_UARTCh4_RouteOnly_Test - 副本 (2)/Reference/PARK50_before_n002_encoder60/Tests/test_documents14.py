"""Documents14: preserve actual visible gray during normal yaw variation.
Real linked ARM code with synthetic sensor/encoder inputs; no physical test.
"""
from pathlib import Path
import sys,json
setup=Path(__file__).with_name('test_documents13.py').read_text(encoding='utf-8')
exec(compile(setup.split('failures=[];passed=[]')[0],str(Path(__file__).with_name('test_documents13.py')),'exec'))

def log_replay():
    global clock,odom,gray_raw,gray_state,rear_valid,yaw
    start(40,2,46,2);member('thief_localization','yaw_cw_sign',-1)
    ret('Route_AllWheelsStopped',0)
    clock=54425;odom=14;gray_raw=0;gray_state=2;rear_valid=2;yaw=-81
    call('Thief_UpdatePose');tick(20)
    check(var('route_state')==8 and var('map_gray_active')==1,'Documents14 begins exit-line recovery')
    odom=38;gray_raw=0x10;gray_state=1;rear_valid=1;yaw=-466
    call('Thief_UpdatePose');tick(105)
    check(var('route_active')==1,'visible front at4.66 degrees remains active')
    odom=41;yaw=-651
    call('Thief_UpdatePose');tick(60)
    check(var('route_active')==1 and var('map_gray_active')==1,
          'Documents14 actual front line must survive6.51-degree yaw without fault')
    ret('Route_AllWheelsStopped',1)
    for _ in range(6):tick()
    check(var('route_state')==2 and not var('map_gray_active'),'real line plus measured stop resumes normal follow')
    check(member('thief_localization','confirmed_node')==40,'recovery never invents arrival at N046')
    check(sum(targets[-1][:2])//2>=2000,'16T normal speed retained after recovery')

def heading_sweep():
    global odom,gray_raw,gray_state,rear_valid,yaw
    count=0
    for mode in [0,1]:
      for sign in [-1,1]:
       for angle in [-1000,-999,-800,-651,-501,0,501,651,800,999,1000]:
        start(38,1,37 if mode else 39,3 if mode else 1,mode)
        member('thief_localization','yaw_cw_sign',sign)
        ret('Route_AllWheelsStopped',1)
        odom=20;gray_raw=0;gray_state=2;rear_valid=1;yaw=angle
        call('Thief_UpdatePose');tick()
        check(var('map_gray_active')==1,'localized road remains eligible inside user-selected10deg')
        gray_raw=0x10;gray_state=1
        for _ in range(3):tick()
        check(var('route_state')==2 and member('thief_localization','error')==0,
              f'real gray inside10deg resumes {mode=} {sign=} {angle=}')
        count+=1
    return count

def recovery_limit():
    global odom,gray_raw,gray_state,yaw,clock
    for angle in [-1001,1001]:
        start();odom=20;gray_raw=0;gray_state=2;tick()
        check(var('map_gray_active')==1,'recovery begins within yaw limit')
        yaw=angle;call('Thief_UpdatePose');clock+=20;call('App_ThiefStrategy_Task')
        check(var('route_active')==0 and reason()=='MAP_GRAY_REACQUIRE_FAILED',
              'recovery respects requested10deg limit')
        check(member('thief_localization','error')==0,'normal localization tolerance is unchanged')

def original_rejections():
    global odom,gray_raw,gray_state,rear_valid,yaw,clock
    for angle in [-2501,2501]:
        start();odom=20;gray_raw=0x10;gray_state=1;yaw=angle
        call('App_ThiefStrategy_Task')
        check(var('route_active')==0 and reason()=='LOC_HEADING_MISMATCH',
              'original localization still rejects an actual out-of-tolerance heading')
    start();gray_raw=0;gray_state=2;odom=20
    hook('JY61P_GetLastUpdateMs',lambda:clock-200)
    call('App_ThiefStrategy_Task')
    check(var('route_active')==0 and reason()=='LOC_GYRO_STALE','stale gyro is still rejected')
    hook('JY61P_GetLastUpdateMs',lambda:clock)

failures=[];passed=[];sweep_cases=0
for fn in [log_replay,heading_sweep,recovery_limit,original_rejections]:
    try:
        result=fn()
        if fn==heading_sweep:sweep_cases=result
        passed.append(fn.__name__);print(fn.__name__+': PASS',flush=True)
    except AssertionError as e:failures.append(dict(case=fn.__name__,error=str(e)));print(fn.__name__+': FAIL '+str(e),flush=True)
result=dict(checks=checks,failures=failures,passed=passed,heading_sweep_cases=sweep_cases,
 firmware_sha256=hashlib.sha256(axf.read_bytes()).hexdigest(),hardware_tested=False,
 app_source_sha256=hashlib.sha256((ROOT/'User/App_ThiefStrategy.c').read_bytes()).hexdigest(),
 test_script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
 helper_sha256={n:hashlib.sha256((ROOT/'Tests'/n).read_bytes()).hexdigest() for n in
    ['test_app_runtime.py','test_escape_completion.py','test_documents13.py']})
out=ROOT/'Tests'/('before_documents14/regression_before.json' if '--before' in sys.argv else 'results_documents14.json')
out.write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8');print(json.dumps(result,indent=2))
sys.exit(bool(failures))
