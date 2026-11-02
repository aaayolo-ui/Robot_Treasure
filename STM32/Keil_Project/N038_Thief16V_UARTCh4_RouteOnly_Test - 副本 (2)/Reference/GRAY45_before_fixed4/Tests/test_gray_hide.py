from pathlib import Path
import json,hashlib
ROOT=Path(__file__).absolute().parents[1]
setup=(ROOT/'Tests/test_n068_recovery.py').read_text(encoding='utf-8').split('failures=[]')[0]
exec(compile(setup,str(ROOT/'Tests/test_n068_recovery.py'),'exec'))
ptr=0x2000ca00
cases=[];failures=[]
def run_align(n=14):
 global clock
 for _ in range(n):
  clock+=20;call('Route_HideAlign',clock,20)
def start_align():
 global front_raw,back_raw,physical_rpm
 fresh(38,0);front_raw=back_raw=0x60;physical_rpm=0;cache(0);commands.clear()
 var('hide_alignment_pending',1);var('hide_align_tick',clock);call('RoadHeading_Reset',symbols['hide_heading'],clock)
try:
 for a,b,aligned in [(0x60,0x60,1),(0x18,0x180,1),(0x180,0x18,1),(0x18,0x18,0),(0x60,0xC,0)]:
  front_raw=a;back_raw=b
  good=call('Route_HideGrayError',ptr);err=get(ptr,2);err=err-65536 if err>=32768 else err
  check(good==1,'two compact real stripes accepted')
  check((abs(err)<=50)==bool(aligned),'mirrored P positions produce expected alignment')
  cases.append(dict(front=a,rear=b,error=err))
 for a,b in [(0,0x60),(0x60,0),(0xFFF,0x60),(0x60,0xFFF),(0x8C0,0x60)]:
  front_raw=a;back_raw=b;check(call('Route_HideGrayError',ptr)==0,'missing/wide/multiple clusters do not prove alignment')
 start_align();yaw=1200;run_align()
 check(not var('hide_alignment_pending'),'centered gray ends trim despite 12-degree map YAW error')
 check(not any(x[0]=='drive' for x in commands),'aligned gray does not command another rotation')
 start_align();front_raw=back_raw=0x18;yaw=0;run_align(8)
 check(var('hide_alignment_pending') and any(x[0]=='drive' and x[1]*x[2]<0 for x in commands),'gray skew commands rotation even at zero YAW error')
 back_raw=0x180;run_align()
 check(not var('hide_alignment_pending'),'parallel off-center stripes finish after brake confirmation')
 fresh(63,0);member('thief_localization','yaw_cw_sign',1)
 var('park_start_yaw',0);var('park_turn_quarter',1);var('park_turn_pending',2);var('park_target_heading',1)
 front_raw=back_raw=0x60;physical_rpm=0;cache(0);commands.clear();yaw=7500;clock+=20
 call('Route_RunParkingTurn',clock,20)
 check(var('park_turn_pending')!=0,'75-degree old line cannot complete parking turn')
 yaw=8200
 for _ in range(18):
  clock+=20;call('Route_RunParkingTurn',clock,20)
  if not var('park_turn_pending'):break
 check(not var('park_turn_pending'),'aligned gray can end parking before forced exact 90 degrees')
except Exception as e:failures.append(str(e))
out=dict(checks=checks,failures=failures,cases=cases,axf_sha256=hashlib.sha256(axf.read_bytes()).hexdigest(),hardware_tested=False)
(ROOT/'Tests/results_gray_hide.json').write_text(json.dumps(out,indent=2),encoding='utf-8')
print(json.dumps(out,indent=2));assert not failures,failures
