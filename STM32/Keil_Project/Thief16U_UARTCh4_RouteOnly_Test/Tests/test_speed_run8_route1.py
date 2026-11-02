"""Compare actual ARM wheel target curves to the user's 16T reference."""
from pathlib import Path
import sys,re,json
setup=Path(__file__).with_name('test_documents13.py').read_text(encoding='utf-8')
exec(compile(setup.split('failures=[];passed=[]')[0],str(Path(__file__).with_name('test_documents13.py')),'exec'))
ref=ROOT/'Reference/RUN8'
source=(ROOT/'User/App_ThiefStrategy.c').read_text(encoding='utf-8')
original=(ref/'User/App_ThiefStrategy.c').read_text(encoding='utf-8')
params=['ROUTE_LEAVE_INITIAL_RPM_X10','ROUTE_LEAVE_START_RPM_X10','ROUTE_AFTER_TURN_START_RPM_X10',
 'ROUTE_LEAVE_RAMP_STEP_RPM_X10','ROUTE_TURN_APPROACH_RPM_X10','ROUTE_TURN_APPROACH_DELAY_MS',
 'ROUTE_TURN_EQUAL_PWM','ROUTE_TURN_TARGET_ANGLE_X100','ROUTE_PID_CENTER_GAIN_PCT',
 'ROUTE_PID_SIDE_GAIN_PCT','ROUTE_PID_CENTER_EDGE_GAIN_PCT']
values={}
for key in params:
    a=re.search(r'^#define\s+'+key+r'\s+(\w+)',source,re.M)[1]
    b=re.search(r'^#define\s+'+key+r'\s+(\w+)',original,re.M)[1]
    check(a==b,'speed/turn/gray gain equals16T: '+key);values[key]=a
check('FollowSmoothing_LimitBase' not in source,'no extra normal-follow speed cap')
curves={}
for mode,label in [(0,'forward'),(1,'reverse')]:
    start(38,1,37 if mode else 39,3 if mode else 1,mode)
    targets.clear()
    for _ in range(4):tick()
    curves[label]=targets[:]
    check(len(targets)==4 and all(sum(t[:2])//2>=2000 for t in targets),'normal launch retains16T speed: '+label)
fresh(11,1);setstep(11,22,2,0,300);call('Route_Start')
tick(20);tick(60);yaw=9000;tick(200)
check(var('route_state')==7,'actual turn hands off to exit-line following')
check(var('leave_target_rpm_x10')==2000,'post-turn seed is200RPM')
targets.clear()
for _ in range(4):tick()
curves['turn_exit']=targets[:]
check(targets and all(sum(t[:2])//2>=2000 for t in targets),'turn exit has no added100RPM cap')
if '--baseline-16t' not in sys.argv:
    start();odom=8;gray_raw=0;gray_state=2
    for _ in range(4):tick()
    gray_raw=0x60;gray_state=1;targets.clear()
    for _ in range(3):tick()
    check(var('route_state')==2 and sum(targets[-1][:2])//2>=2000,
          'real leading confirmation restores pre-anomaly speed without a new50RPM normal seed')
result=dict(checks=checks,failures=0,curves=curves,parameters=values,
 firmware_sha256=hashlib.sha256(axf.read_bytes()).hexdigest(),hardware_tested=False,
 app_source_sha256=hashlib.sha256(((ref if '--baseline-16t' in sys.argv else ROOT)/'User/App_ThiefStrategy.c').read_bytes()).hexdigest(),
 test_script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
 reference_source_sha256=hashlib.sha256((ref/'User/App_ThiefStrategy.c').read_bytes()).hexdigest())
out=ROOT/'Tests'/('results_speed16t_reference.json' if '--baseline-16t' in sys.argv else 'results_speed16t.json')
out.write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8');print(json.dumps(result,indent=2))
