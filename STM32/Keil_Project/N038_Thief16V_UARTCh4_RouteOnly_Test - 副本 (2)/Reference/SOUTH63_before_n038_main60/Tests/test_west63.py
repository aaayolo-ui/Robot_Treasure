from pathlib import Path
import sys,json,hashlib
setup=Path(__file__).with_name('test_escape_completion.py').read_text(encoding='utf-8').split('# Log regression:')[0]
if '--before' in sys.argv:
    setup=setup.replace('exec(compile(setup.split(', 'setup=setup.replace("axf=ROOT/\'MDK-ARM\'/ROOT.name/(ROOT.name+\'.axf\')", "axf=ROOT/\'Reference/ROUTE1_before_west63/MDK-ARM\'/ROOT.name/(ROOT.name+\'.axf\')")\nexec(compile(setup.split(',1)
exec(compile(setup,str(Path(__file__).with_name('test_escape_completion.py')),'exec'))
failures=[]
expected=[12,23,31,32,48,49,61,62,63]
for body in range(4):
    fresh(11,body)
    assert call('ThiefMapPlanner_PlanEscape',11,body,8,0,clock,plan)
    path=[at(steps+i*size(types['MapRouteStep_t']),'MapRouteStep_t','to_node') for i in range(at(route,'MapRoute_t','count'))]
    if path!=expected:failures.append(dict(body=body,path=path))
    if member('thief_plan','goal_node')!=63 or member('thief_plan','final_body_heading')!=0:failures.append(dict(body=body,issue='not north parked N063'))
image=ROOT/'MDK-ARM'/ROOT.name/(ROOT.name+'.axf')
if '--before' in sys.argv:image=ROOT/'Reference/ROUTE1_before_west63/MDK-ARM'/ROOT.name/(ROOT.name+'.axf')
report=dict(failures=failures,axf_sha256=hashlib.sha256(image.read_bytes()).hexdigest(),hardware_tested=False)
(ROOT/'Tests'/('west63_before.json' if '--before' in sys.argv else 'west63_after.json')).write_text(json.dumps(report,indent=2),encoding='utf-8')
print(json.dumps(report,indent=2));sys.exit(bool(failures))
