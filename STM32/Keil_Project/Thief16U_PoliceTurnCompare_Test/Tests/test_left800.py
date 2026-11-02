"""Physical CH2/CH3 raw800 acceptance against the real linked ARM gate."""
from pathlib import Path
import sys,json,hashlib
src=Path(__file__).with_name('test_right900.py').read_text(encoding='utf-8').split('failures=[];cases={}')[0]
src=src.replace('before_right900/original.axf','before_left800/original.axf')
exec(compile(src,str(Path(__file__).with_name('test_right900.py')),'exec'))
if '--before' in sys.argv:
    failures=[]
    for ch in [1,2]:
        fresh(49,1);clock+=1
        for _ in range(8):clock+=20;call('Thief_OnLaserFrame',ch,801,100,1,clock)
        if var('thief_escape_requested'):failures.append('CH%d still accepts raw801'%(ch+1))
    result=dict(expected_before_failure=len(failures)==2,failures=failures,axf_sha256=hashlib.sha256(axf.read_bytes()).hexdigest(),test_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),hardware_tested=False)
    (ROOT/'Tests/before_left800/regression_before.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    print(json.dumps(result,indent=2));sys.exit(0 if len(failures)==2 else 1)
failure=None
try:
    for ch,limit in enumerate([1000,800,800,1000]):
      for raw in [49,50,799,800,801,899,900,901,999,1000,1001]:
        fresh(49,1);clock+=1
        for _ in range(8):clock+=20;call('Thief_OnLaserFrame',ch,raw,100,1,clock)
        check(bool(var('thief_escape_requested'))==(50<=raw<=limit),'static physical CH%d raw%d must respect %dmm'%(ch+1,raw,limit))
        for heading in range(4):
          member('thief_pose','x_mm',1500);member('thief_pose','y_mm',1400)
          member('thief_pose','body_heading',heading);member('thief_pose','valid',1)
          gate=0x2000c800;u.mem_write(gate,bytes(size(types['FrontLaserGate_t'])))
          accepted=call('FrontLaserGate_Observe',gate,symbols['thief_pose'],1,raw,100,1,ch)
          check(bool(accepted)==(50<=raw<=limit),'immediate gate channel cap must not rotate with chassis')
          check(at(gate,'FrontLaserGate_t','distance_mm')==raw,'over-limit raw telemetry remains unchanged')
    check(call('FrontLaserGate_CenterDistanceMm',800,1)==900,'CH2 center projection still adds100mm without changing raw threshold')
    check(call('FrontLaserGate_CenterDistanceMm',800,2)==900,'CH3 center projection still adds100mm without changing raw threshold')
except AssertionError as e:failure=str(e)
before='--before' in sys.argv
result=dict(checks=checks,failure=failure,expected_before_failure=before and bool(failure),axf_sha256=hashlib.sha256(axf.read_bytes()).hexdigest(),test_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),hardware_tested=False)
path=ROOT/'Tests'/('before_left800/regression_before.json' if before else 'results_left800.json')
path.write_text(json.dumps(result,indent=2),encoding='utf-8');print(json.dumps(result,indent=2))
sys.exit(0 if before and failure else bool(failure))
