"""033134: known map-axis recovery beyond110relative degrees, actual turn PWM."""
from pathlib import Path
import sys,json,hashlib
src=Path(__file__).with_name('test_documents15.py').read_text(encoding='utf-8').split('parking_pwm=[]')[0]
src=src.replace('Tests/before_documents15/original.axf','Tests/before_monitor7_restore/original.axf')
exec(compile(src,str(Path(__file__).with_name('test_documents15.py')),'exec'))
pwm=[]
def capture():
    v=u.reg_read(UC_ARM_REG_R1);pwm.append(v-2**32 if v>=2**31 else v);return 0
hook('Chassis_SetWheelLogicalPwm',capture)

def overshoot_axis():
    global clock,yaw,odom,gray_raw,gray_state,rear_valid
    check('Route_AlignTurnAxis' in symbols,'saved MONITOR7 axis trim restored')
    for mode in [0,1]:
      for sign in [-1,1]:
        fresh(15,2*mode);member('thief_localization','yaw_cw_sign',sign)
        setstep(15,6,0,mode,300,action=-1);setstep(6,4,3,mode,300,index=1)
        record(steps+16,'MapRouteStep_t',dict(segment_count=2));record(route,'MapRoute_t',dict(count=2))
        call('Route_Start');odom=300;clock+=500;call('Route_HandleJunction',clock,20,4,0xc60)
        start=sign*1850
        var('turn_start_angle_x100',(start+(18000 if mode else 0)+18000)%36000-18000)
        yaw=-sign*11200;gray_raw=0x60;gray_state=1;rear_valid=1
        call('Route_EnterAfterTurnForward',clock,20,0x2000ce00,yaw)
        # Relative movement is130.5degrees, but actual map axis is still known.
        error=0x2000cd00;obs=allocate('ThiefLocYaw_t',0x2000cd10)
        check(call('Route_TurnExitContext',error,obs),'known axis remains available beyond110relative degrees')
        pwm.clear()
        for _ in range(10):clock+=20;call('Route_AlignTurnAxis',clock,20)
        check(pwm and len(pwm)%4==0,'actual four-wheel trim output exists after overshoot')
        check(pwm[-4]>0 and pwm[-3]<0,'overshoot correction turns back toward the known exit')
        yaw=-sign*9000
        for _ in range(20):clock+=20;call('Route_AlignTurnAxis',clock,20)
        check(not var('turn_axis_active'),'fresh centered stopped axis completes original MONITOR7 trim')
    return 4

failures=[];cases={}
try:cases['overshoot_axis']=overshoot_axis()
except Exception as e:failures.append(dict(case='overshoot_axis',error=str(e)))
axf=ROOT/'MDK-ARM'/ROOT.name/(ROOT.name+'.axf')
if '--before' in sys.argv:axf=ROOT/'Tests/before_monitor7_restore/original.axf'
r=dict(cases=cases,failures=failures,axf_sha256=hashlib.sha256(axf.read_bytes()).hexdigest(),
       test_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),hardware_tested=False)
dest=ROOT/('Tests/before_monitor7_restore/regression_before.json' if '--before' in sys.argv else 'Tests/results_monitor7_restore.json')
dest.write_text(json.dumps(r,indent=2),encoding='utf-8');print(json.dumps(r,indent=2));sys.exit(bool(failures))
