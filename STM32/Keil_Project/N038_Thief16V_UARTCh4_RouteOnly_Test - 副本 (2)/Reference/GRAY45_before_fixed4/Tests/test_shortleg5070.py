from pathlib import Path
import json,hashlib
ROOT=Path(__file__).absolute().parents[1]
setup=(ROOT/'Tests/test_n068_recovery.py').read_text(encoding='utf-8').split('failures=[]')[0]
exec(compile(setup,str(ROOT/'Tests/test_n068_recovery.py'),'exec'))
failures=[];cases=[]
def setup_leg(a,b,h,length=300,action=2):
 fresh(a,h);setstep(a,b,h,0,length);call('MapRouteNavigator_StartRuntime',nav,route,steps)
 check(not member('map_nav','fault'),'real connected test leg validates')
 record(steps,'MapRouteStep_t',dict(action=action))
try:
 for a,b,h,action in [(11,2,0,1),(54,53,3,1),(44,36,0,-1),(26,17,0,0),(21,22,1,0),(39,38,3,2)]:
  setup_leg(a,b,h,action=action)
  for distance in [149,150,210,211,300,1000]:
   odom=distance
   for raw in [0xC40,0x07F,0xFFF]:
    left=bool(raw&0xC00==0xC00);right=bool(raw&3==3)
    if action==-1:shape=left
    elif action==1:shape=right
    elif action==0:
     shape=(left and call('ThiefMapPlanner_HasRoad',b,(h+3)%4,0,0)) or (right and call('ThiefMapPlanner_HasRoad',b,(h+1)%4,0,0))
    else:shape=True
    expected=bool(distance>=150 and shape)
    check(bool(call('Route_ArrivalPlausible',raw))==expected,f'{a}->{b} action {action} raw {raw:X} distance {distance}')
  cases.append(dict(start=a,end=b,action=action))
 setup_leg(63,78,2,600);odom=240
 check(call('Route_ArrivalPlausible',0xC40)==1,'600mm normal leg retains original 40-percent floor')
 # Returning and partial retreat retain original arrival-window behavior.
 for returning,retreat in [(1,0),(0,1)]:
  setup_leg(11,2,0,action=1);record(trail,'ThiefRouteTrail',dict(returning=returning));var('retreat_active',retreat);odom=80
  check(call('Route_ArrivalPlausible',0xC40)==call('MapRouteNavigator_InArrivalWindow',nav,odom),'short-return rules bypass new ordinary-leg floor and port rule')
except Exception as e:failures.append(str(e))
out=dict(checks=checks,failures=failures,cases=cases,axf_sha256=hashlib.sha256(axf.read_bytes()).hexdigest(),hardware_tested=False)
(ROOT/'Tests/results_shortleg5070.json').write_text(json.dumps(out,indent=2),encoding='utf-8')
print(json.dumps(out,indent=2));assert not failures,failures
