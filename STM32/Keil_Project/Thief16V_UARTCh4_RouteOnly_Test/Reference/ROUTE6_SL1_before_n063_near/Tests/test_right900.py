"""Channel caps: left/right800, front/rear1000. Linked ARM, mocked HW."""
from pathlib import Path
import json,sys,hashlib
src=Path(__file__).with_name('test_documents15.py').read_text(encoding='utf-8').split('parking_pwm=[]')[0]
src=src.replace("if '--before' in sys.argv:","if False:")
if '--before' in sys.argv:
 src=src.replace('exec(compile(runtime,',"runtime=runtime.replace(\"axf=ROOT/'MDK-ARM'/ROOT.name/(ROOT.name+'.axf')\",\"axf=ROOT/'Tests/before_right900/original.axf'\")\nexec(compile(runtime,")
exec(compile(src,str(Path(__file__).with_name('test_documents15.py')),'exec'))
failures=[];cases={}
def boundary_matrix():
 global clock
 total=0
 for ch in range(4):
  limit=800 if ch in [1,2] else 1000
  for distance in [49,50,799,800,801,899,900,901,999,1000,1001]:
   fresh(49,1);clock+=1
   for _ in range(8):clock+=20;call('Thief_OnLaserFrame',ch,distance,100,1,clock)
   accepted=50<=distance<=limit
   check(bool(var('thief_escape_requested'))==accepted,f'physical CH{ch+1} raw{distance} acceptance must match limit{limit}')
   if distance>limit:
    gate=symbols['thief_laser']+ch*size(types['FrontLaserGate_t'])
    check(at(gate,'FrontLaserGate_t','status')==3,f'CH{ch+1} over-range reports TOO_FAR')
   total+=1
 return total
def right_cap_independent_of_heading():
 global clock
 total=0
 for body in range(4):
  for distance in [800,801,900,901,1000]:
   fresh(49,body)
   # Interior synthetic map pose isolates channel limit from map-edge rejection.
   member('thief_pose','x_mm',1500);member('thief_pose','y_mm',1400);clock+=1
   for _ in range(8):clock+=20;call('Thief_OnLaserFrame',2,distance,100,1,clock)
   check(bool(var('thief_escape_requested'))==(distance==800),'physical CH3 800mm cap does not rotate with chassis heading')
   total+=1
 return total
for fn in [boundary_matrix,right_cap_independent_of_heading]:
 try:cases[fn.__name__]=fn();print(fn.__name__+': PASS',flush=True)
 except AssertionError as e:failures.append(dict(case=fn.__name__,error=str(e)));print(fn.__name__+': FAIL '+str(e),flush=True)
sha=lambda f:hashlib.sha256(f.read_bytes()).hexdigest()
before='--before' in sys.argv;base=ROOT/'Tests/before_right900' if before else ROOT
r=dict(checks=checks,failures=failures,cases=cases,hardware_tested=False,firmware_sha256=sha(axf),test_sha256=sha(Path(__file__)),source_sha256={n:sha(base/n) for n in ['User/App_ThiefStrategy.c','Algorithm/ThiefStrategy/FrontLaserGate.c','Algorithm/ThiefStrategy/ThiefStrategyConfig.h']},helper_sha256={n:sha(ROOT/'Tests'/n) for n in ['test_documents15.py','test_app_runtime.py','test_escape_completion.py','test_documents13.py']})
(ROOT/'Tests'/('before_right900/regression_before.json' if before else 'results_right900.json')).write_text(json.dumps(r,indent=2),encoding='utf-8');print(json.dumps(r,indent=2));sys.exit(bool(failures))
