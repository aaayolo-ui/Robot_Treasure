"""Linked ARM N063 near/far route split; synthetic sensors/motion only."""
from pathlib import Path
import sys,json,hashlib
ROOT=Path(__file__).absolute().parents[1];before='--before' in sys.argv
model=(ROOT/'Tests/test_escape_completion.py').read_text(encoding='utf-8').split('# Log regression:')[0]
app=(ROOT/'Tests/test_app_runtime.py').read_text(encoding='utf-8').split('def documents8_cases():')[0]
if before:app=app.replace("axf=ROOT/'MDK-ARM'/ROOT.name/(ROOT.name+'.axf')","axf=ROOT/'Reference/ROUTE6_SL1_before_n063_near/MDK-ARM'/ROOT.name/(ROOT.name+'.axf')")
exec(compile(app,str(ROOT/'Tests/test_app_runtime.py'),'exec'))
exec(compile(model[model.index('for n in ['):],str(ROOT/'Tests/test_escape_completion.py'),'exec'))
assertions=0;failures=[];cases=[]
expected=[62,61,49,33,25,15,6,4,3,13,12,11]
def expect(ok,message):
    global assertions
    assertions+=1
    if not ok:failures.append(message)
def path():return [at(steps+i*size(types['MapRouteStep_t']),'MapRouteStep_t','to_node') for i in range(at(route,'MapRoute_t','count'))]

for body in range(4):
    for kind,xy,age,near in [('N052',(2700,1500),0,True),('toward_N063',(2700,1650),0,True),
        ('N035_N036',(2700,900),0,False),('midpoint',(2700,1200),0,False),
        ('stale_N052',(2700,1500),251,False),('no_location',None,0,False),
        ('near_range_error',(2700,1400),0,True),('other_column',(2400,1500),0,False),
        ('other_ray_report',(2700,1500),0,True)]:
        fresh(63,body)
        if xy:call('ThiefMapPlanner_ObserveThreat',63,0,*xy,clock)
        if kind=='other_ray_report':call('ThiefMapPlanner_ObserveThreat',63,3,1800,1800,clock)
        ok=call('ThiefMapPlanner_PlanEscape',63,body,1,0,clock+age,plan)
        expect(ok,'plan exists: '+kind)
        nodes=path()
        expect(nodes==expected if near else nodes[:3]==[78,77,76],f'{kind} body={body} corridor: {nodes}')
        if near and nodes==expected:
            expect(member('thief_plan','goal_node')==11 and member('thief_plan','final_body_heading')%2==1,'N011 arrival stays horizontal')
            six=nodes.index(6);step=steps+six*size(types['MapRouteStep_t'])
            expect(at(step,'MapRouteStep_t','action')==255,'N006 north-to-west uses an ordinary left turn (signed int8 -1)')
            merged=steps+(six+1)*size(types['MapRouteStep_t'])
            expect(at(merged,'MapRouteStep_t','segment_count')==2,'N005 remains an internal treasure segment, not a fake junction')
        call('MapRouteNavigator_StartRuntime',nav,route,steps)
        expect(not member('map_nav','fault'),'actual navigator validates joined map path/modes')
        cases.append(dict(case=kind,body=body,path=nodes))

lifecycles=[]
if not before:
    for body in [0,2]:
        for raw,near in [(150,True),(250,True),(750,False)]:
            fresh(63,body)
            channel=[(body+d)%4 for d in [0,3,1,2]].index(0)
            for _ in range(3):clock+=20;call('Thief_OnLaserFrame',channel,raw,100,1,clock)
            tick();nodes=path();goal=member('thief_plan','goal_node')
            expect(var('route_active') and (nodes==expected if near else nodes[:3]==[78,77,76]),'actual gate selects correct near/far corridor')
            result=complete_route(limit=6000)
            expect(result['node']==goal,'actual task completes selected escape at its final refuge')
            if near:expect(result['node']==11 and result['body']%2==1,'N052 escape reaches N011 and parks horizontally')
            lifecycles.append(dict(body=body,physical_channel=channel,raw_mm=raw,path=nodes,result=result))
out=dict(checks=assertions+checks,failures=failures,cases=cases,lifecycles=lifecycles,
         axf_sha256=hashlib.sha256(axf.read_bytes()).hexdigest(),test_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),hardware_tested=False)
(ROOT/'Tests'/('n063_near_before.json' if before else 'results_n063_near.json')).write_text(json.dumps(out,indent=2),encoding='utf-8')
print(json.dumps(out,indent=2));assert bool(failures)==before,failures
