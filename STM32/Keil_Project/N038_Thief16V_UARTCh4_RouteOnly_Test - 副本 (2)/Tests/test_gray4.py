from pathlib import Path
import json,hashlib
ROOT=Path(__file__).absolute().parents[1]
setup=(ROOT/'Tests/test_arrival_gate.py').read_text(encoding='utf-8').split('try:')[0]
exec(compile(setup,str(ROOT/'Tests/test_arrival_gate.py'),'exec'))
failures=[];watch=[]
try:
 begin();odom=180;front_raw=back_raw=0x60
 var('last_front_watch_tick',clock);var('last_control_tick',clock);commands.clear();start=clock;last=clock
 for _ in range(40):
  tick(1);nowwatch=var('last_front_watch_tick')
  if nowwatch!=last:watch.append(clock-start);last=nowwatch
 check(watch==list(range(4,41,4)),f'ideal scheduler uses fixed 4ms fast watch: {watch}')
 check(len([x for x in commands if x[0]=='drive'])==2,'20ms PID remains at 20ms and 40ms')
 begin();odom=180;front_raw=0x60;var('last_front_watch_tick',clock);var('last_control_tick',clock)
 for _ in range(3):tick(1)
 check(not member('map_nav','arrived'),'before first watch interval no new arrival')
 front_raw=0x07F;tick(1)
 check(member('map_nav','arrived') and var('route_state')==3,'right branch at 4ms accepted before PID update')
 # No alternation state remains.
 begin();check('front_watch_extra_ms' not in symbols,'alternating phase state removed')
except Exception as e:failures.append(str(e))
out=dict(checks=checks,failures=failures,watch_offsets_ms=watch,axf_sha256=hashlib.sha256(axf.read_bytes()).hexdigest(),hardware_tested=False,ideal_scheduler=True)
(ROOT/'Tests/results_gray4.json').write_text(json.dumps(out,indent=2),encoding='utf-8')
print(json.dumps(out,indent=2));assert not failures,failures
