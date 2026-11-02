from pathlib import Path
import sys,json,hashlib
setup=Path(__file__).with_name('test_escape_completion.py').read_text(encoding='utf-8').split('# Log regression:')[0]
if '--before' in sys.argv:
    setup=setup.replace('exec(compile(setup.split(', 'setup=setup.replace("axf=ROOT/\'MDK-ARM\'/ROOT.name/(ROOT.name+\'.axf\')", "axf=ROOT/\'Reference/ROUTE2_before_trace_return/MDK-ARM\'/ROOT.name/(ROOT.name+\'.axf\')")\nexec(compile(setup.split(',1)
exec(compile(setup,str(Path(__file__).with_name('test_escape_completion.py')),'exec'))
fresh(11,1)
for _ in range(3):clock+=20;call('Thief_OnLaserFrame',1,150,100,1,clock)
tick();odom=120
call('Thief_UpdatePose');call('Thief_UpdateLaserMask',clock)
front_channel=3 if call('TravelFrame_IsReverse') else 0
for _ in range(3):
    clock+=20;call('Thief_OnLaserFrame',front_channel,150,100,1,clock)
call('Thief_InterceptReact',clock)
failures=[]
if var('route_state')!=12:failures.append('locked N011 transfer ignores confirmed travel-front interception')
if not failures:
    for _ in range(12):
        tick()
        if var('route_state')!=12:break
    if member('thief_plan','goal_node')!=11:failures.append('does not return to original N011')
    result=complete_route()
    if result['node']!=11:failures.append('does not settle at original N011')
image=ROOT/'MDK-ARM'/ROOT.name/(ROOT.name+'.axf')
if '--before' in sys.argv:image=ROOT/'Reference/ROUTE2_before_trace_return/MDK-ARM'/ROOT.name/(ROOT.name+'.axf')
report=dict(failures=failures,axf_sha256=hashlib.sha256(image.read_bytes()).hexdigest(),hardware_tested=False)
(ROOT/'Tests'/('trace_front_before.json' if '--before' in sys.argv else 'trace_front_after.json')).write_text(json.dumps(report,indent=2),encoding='utf-8')
print(json.dumps(report,indent=2));sys.exit(bool(failures))
