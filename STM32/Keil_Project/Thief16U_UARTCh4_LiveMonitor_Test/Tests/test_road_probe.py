"""MCU linked recovery, actual outputs with simulated gray/yaw/encoders."""
from pathlib import Path
import sys,json,hashlib
src=Path(__file__).with_name('test_documents15.py').read_text(encoding='utf-8').split('parking_pwm=[]')[0]
src=src.replace('Tests/before_documents15/original.axf','Tests/before_road_probe/original.axf')
exec(compile(src,str(Path(__file__).with_name('test_documents15.py')),'exec'))
src=Path(__file__).with_name('test_documents24.py').read_text(encoding='utf-8')
exec(src[src.index('def pending('):src.index('def slow_correction(')])

def supported_exit():
    global yaw,gray_raw,gray_state,rear_valid,clock
    for mode in [0,1]:
      for sign in [-1,1]:
        pending(mode,sign)
        yaw=sign*q*(9000-277);rear_valid=1
        targets.clear()
        for _ in range(20):tick()
        check(var('turn_exit_line_phase')==1,'2.77degree residual must promptly enter search without waiting2500ms')
        for _ in range(300):tick()
        check(var('route_active'),'rear line must prevent2500ms exit fault: '+reason())
        check(var('turn_exit_line_phase')==1,'2.77degree residual must enter search without waiting2degrees')
        check(any(l+r>0 for l,r,*_ in targets),'rear-supported search must translate')
        check(member('thief_localization','phase')==2,'rear-only line never commits outgoing edge')
    return 4

def missing_probe():
    global yaw,gray_raw,gray_state,rear_valid,clock
    for mode in [0,1]:
      pending(mode,1);yaw=q*(9000-150);rear_valid=2;targets.clear()
      for _ in range(20):tick()
      check(var('route_active'),'short both-board gap remains recoverable')
      check(any(l+r>0 for l,r,*_ in targets),'both missing must briefly probe likely mapped line')
      check(member('thief_localization','phase')==2,'blind movement cannot confirm road exit')
      for _ in range(110):
        clock+=20;call('App_ThiefStrategy_Task')
      check(not var('route_active') and reason()=='GRAY_BOTH_NO_LINE','both boards continuously missing2seconds stops')
    return 2

def loss_reset_and_real_exit():
    global yaw,gray_raw,gray_state,rear_valid,clock
    for mode in [0,1]:
      pending(mode,1);yaw=q*(9000-150);rear_valid=2
      for _ in range(60):tick()
      rear_valid=1;tick();rear_valid=2
      for _ in range(60):tick()
      check(var('route_active'),'separate missing intervals must not accumulate')
      gray_raw=0x60;gray_state=1
      for _ in range(15):tick()
      check(member('thief_localization','phase')==1,'fresh real line commits actual outgoing edge')
      check(member('thief_localization','confirmed_node')==37,'finding line must not invent next node')
    return 2

def road_probe_and_cap():
    global yaw,gray_raw,gray_state,rear_valid,clock,odom
    for mode in [0,1]:
      fresh(38,1 if mode else 3);rear_valid=2
      setstep(38,37,3,mode,300);call('Route_Start');tick()
      odom=20;gray_raw=0;gray_state=2;targets.clear()
      for _ in range(5):tick()
      check(any(l+r>0 for l,r,*_ in targets),'mapped road loss probes rather than spinning only')
      odom=125;targets.clear()
      for _ in range(7):tick()
      check(all(l+r==0 for l,r,*_ in targets),'blind probe stops translating outside100mm envelope')
      check(member('thief_localization','confirmed_node')==38,'100mm alone cannot advance node')
      rear_valid=1
      for _ in range(20):tick()
      check(var('route_active'),'real trailing line permits tracking beyond blind envelope')
    return 2

failures=[];cases={}
for fn in [supported_exit,missing_probe,loss_reset_and_real_exit,road_probe_and_cap]:
    try:cases[fn.__name__]=fn();print(fn.__name__+': PASS',flush=True)
    except AssertionError as exc:failures.append(dict(case=fn.__name__,error=str(exc)));print(fn.__name__+': FAIL '+str(exc),flush=True)
result=dict(checks=checks,failures=failures,cases=cases,hardware_tested=False,
 axf_sha256=hashlib.sha256(axf.read_bytes()).hexdigest(),test_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
out=ROOT/'Tests'/('before_road_probe/regression_before.json' if '--before' in sys.argv else 'results_road_probe.json')
out.write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8');print(json.dumps(result,indent=2));sys.exit(bool(failures))
