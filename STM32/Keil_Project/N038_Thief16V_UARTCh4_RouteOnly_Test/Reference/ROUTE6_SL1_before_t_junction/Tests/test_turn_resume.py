"""025736: real speed PI/enable state, synthetic hardware feedback and PWM sinks."""
from pathlib import Path
import sys,json,hashlib
src=Path(__file__).with_name('test_documents15.py').read_text(encoding='utf-8').split('parking_pwm=[]')[0]
src=src.replace('Tests/before_documents15/original.axf','Tests/before_turn_resume/original.axf')
exec(compile(src,str(Path(__file__).with_name('test_documents15.py')),'exec'))
for n in ['Route_ResetControl','Route_BrakeControl','ChassisSpeedControl_Enable',
          'ChassisSpeedControl_Disable','ChassisSpeedControl_Brake','ChassisSpeedControl_Update',
          'ChassisSpeedControl_ResetPiState','TravelFrame_SetWheelTargets']:
    if n in hooks:u.hook_del(hooks.pop(n))
ret('Chassis_Init')
event('Chassis_Disable')
pwm=[]
def capture():
    value=u.reg_read(UC_ARM_REG_R1)
    pwm.append((u.reg_read(UC_ARM_REG_R0),value-2**32 if value>=2**31 else value));return 0
hook('Chassis_SetWheelLogicalPwm',capture)

def resume_after_brake():
    for mode in [0,1]:
        fresh(37,2*mode);setstep(37,20,0,mode,600);call('Route_Start')
        call('ChassisSpeedControl_Init');call('ChassisSpeedControl_Enable')
        call('Route_BrakeControl')
        check(not call('ChassisSpeedControl_IsEnabled'),'actual brake disables PI')
        call('Route_EnterFollow',clock,1,800,1)
        check(call('ChassisSpeedControl_IsEnabled'),'keep-loop request must restore an actually disabled speed loop')
        pwm.clear();call('TravelFrame_SetWheelTargets',800,800,800,800)
        call('ChassisSpeedControl_Update',20)
        check(len(pwm)==4 and all(v!=0 for _,v in pwm),'all four actual PI controllers reach PWM output after brake')
        check(not call('ChassisSpeedControl_HasStall'),'resume is not reported as a stall')
    return 2

def continuous_loop_kept():
    fresh(37,0);setstep(37,20,0,0,600);call('Route_Start')
    call('ChassisSpeedControl_Init');call('ChassisSpeedControl_Enable')
    call('TravelFrame_SetWheelTargets',800,800,800,800);call('ChassisSpeedControl_Update',20)
    # An already-running loop must retain the actual wheel targets/PI state.
    controller=call('ChassisSpeedControl_GetController',0)
    prior=bytes(u.mem_read(controller,size(types['WheelSpeedPi_t'])))
    call('Route_EnterFollow',clock,1,800,1)
    check(bytes(u.mem_read(controller,size(types['WheelSpeedPi_t'])))==prior,'continuous transfer retains active wheel PI')
    return 1

failures=[];cases={}
for fn in [resume_after_brake,continuous_loop_kept]:
    try:cases[fn.__name__]=fn()
    except Exception as e:failures.append(dict(case=fn.__name__,error=str(e)))
axf=ROOT/'MDK-ARM'/ROOT.name/(ROOT.name+'.axf')
if '--before' in sys.argv:axf=ROOT/'Tests/before_turn_resume/original.axf'
r=dict(cases=cases,failures=failures,axf_sha256=hashlib.sha256(axf.read_bytes()).hexdigest(),
       test_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),hardware_tested=False)
out=ROOT/('Tests/before_turn_resume/regression_before.json' if '--before' in sys.argv else 'Tests/results_turn_resume.json')
out.write_text(json.dumps(r,indent=2),encoding='utf-8');print(json.dumps(r,indent=2));sys.exit(bool(failures))
