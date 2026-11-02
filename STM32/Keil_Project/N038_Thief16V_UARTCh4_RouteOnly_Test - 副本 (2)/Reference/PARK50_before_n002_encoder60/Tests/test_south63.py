from pathlib import Path
import json,hashlib
ROOT=Path(__file__).absolute().parents[1]
setup=(ROOT/'Tests/test_escape_completion.py').read_text(encoding='utf-8').split('# Log regression:')[0]
exec(compile(setup,str(ROOT/'Tests/test_escape_completion.py'),'exec'))
cases=[];failures=[]
try:
 for body in [0,2]:
  for sign in [-1,1]:
   fresh(63,body);member('thief_localization','yaw_cw_sign',sign)
   ch=[(body+d)%4 for d in [0,3,1,2]].index(2)
   for _ in range(3):clock+=20;call('Thief_OnLaserFrame',ch,150,100,1,clock)
   tick()
   path=[at(steps+i*size(types['MapRouteStep_t']),'MapRouteStep_t','to_node') for i in range(at(route,'MapRoute_t','count'))]
   check(path==[62,61,49,33,25,15,6,4,3,2,11],'south threat selects exact N061-N006-left refuge corridor')
   check(member('thief_plan','goal_node')==11 and member('thief_plan','final_body_heading')==3,'goal N011 west parked')
   result=complete_route()
   check(result['node']==11 and result['body']==3,'complete actual task settles horizontally at N011')
   cases.append(dict(body=body,sign=sign,path=path,result=result))
except Exception as e:failures.append(str(e))
out=dict(checks=checks,failures=failures,cases=cases,axf_sha256=hashlib.sha256(axf.read_bytes()).hexdigest(),hardware_tested=False)
(ROOT/'Tests/results_south63.json').write_text(json.dumps(out,indent=2),encoding='utf-8')
print(json.dumps(out,indent=2));assert not failures,failures
