from pathlib import Path
import json,hashlib
ROOT=Path(__file__).absolute().parents[1]
setup=(ROOT/'Tests/test_quick350.py').read_text(encoding='utf-8').split('try:')[0]
exec(compile(setup,str(ROOT/'Tests/test_quick350.py'),'exec'))
try:
 for node,threshold in [(38,6000),(63,6000),(11,7000)]:
  for quarter in [-1,1]:
   for sign in [-1,1]:
    for progress in [threshold-100,threshold]:
     fresh(node,0);member('thief_localization','yaw_cw_sign',sign)
     var('park_start_yaw',0);var('park_turn_quarter',quarter)
     var('park_turn_pending',2);var('park_target_heading',1)
     front_raw=back_raw=0x60;physical_rpm=0;cache(0);commands.clear()
     yaw=progress*quarter*sign;clock+=20
     call('Route_RunParkingTurn',clock,20)
     check(var('park_turn_pending')==(2 if progress<threshold else 3),f'node {node} threshold {threshold} progress {progress} quarter {quarter} sign {sign}')
     check(var('park_turn_pending')!=0,'incoming stripe cannot prematurely finish at main cutoff')
 fresh(38,0);member('thief_localization','yaw_cw_sign',1)
 var('park_start_yaw',0);var('park_turn_quarter',1);var('park_turn_pending',2);var('park_target_heading',1)
 front_raw=back_raw=0x60;physical_rpm=0;cache(0);commands.clear();yaw=6000;clock+=20
 call('Route_RunParkingTurn',clock,20)
 check(var('park_turn_pending')==3,'N038 enters fine phase at 60 degrees')
 yaw=8200
 for _ in range(18):
  clock+=20;call('Route_RunParkingTurn',clock,20)
  if not var('park_turn_pending'):break
 check(not var('park_turn_pending'),'real aligned stripes finish N038 after reaching exit window')
except Exception as e:failures.append(str(e))
out=dict(checks=checks,failures=failures,axf_sha256=hashlib.sha256(axf.read_bytes()).hexdigest(),hardware_tested=False)
(ROOT/'Tests/results_n038_main60.json').write_text(json.dumps(out,indent=2),encoding='utf-8')
print(json.dumps(out,indent=2));assert not failures,failures
