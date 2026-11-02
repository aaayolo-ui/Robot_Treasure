from pathlib import Path
import json,hashlib
ROOT=Path(__file__).absolute().parents[1]
setup=(ROOT/'Tests/test_arrival_gate.py').read_text(encoding='utf-8').split('try:')[0]
exec(compile(setup,str(ROOT/'Tests/test_arrival_gate.py'),'exec'))
failures=[];reads=[]
def sample_front():
 reads.append(clock)
 return gray_read(front_raw)
hook('TravelFrame_ReadLeading',sample_front)
try:
 begin();odom=180;front_raw=back_raw=0x60
 var('last_front_watch_tick',clock);var('last_control_tick',clock);reads.clear();commands.clear();start=clock
 for _ in range(20):tick(1)
 offsets=[v-start for v in reads]
 check(offsets==list(range(2,21,2)),f'actual ARM task reads leading board every 2ms under ideal 1ms task scheduling: {offsets}')
 check(len([x for x in commands if x[0]=='drive'])==1,'20ms PID still updates once during ten fast gray reads')
 begin();odom=180;front_raw=0x60;var('last_front_watch_tick',clock);var('last_control_tick',clock)
 tick(1);check(not member('map_nav','arrived'),'no sampling before 2ms watch interval')
 front_raw=0x07F;tick(1)
 check(member('map_nav','arrived') and var('route_state')==3,'short right branch at 2ms is accepted before next PID cycle')
except Exception as e:failures.append(str(e))
out=dict(checks=checks,failures=failures,sample_offsets_ms=offsets,axf_sha256=hashlib.sha256(axf.read_bytes()).hexdigest(),hardware_tested=False,ideal_scheduler=True)
(ROOT/'Tests/results_gray500.json').write_text(json.dumps(out,indent=2),encoding='utf-8')
print(json.dumps(out,indent=2));assert not failures,failures
