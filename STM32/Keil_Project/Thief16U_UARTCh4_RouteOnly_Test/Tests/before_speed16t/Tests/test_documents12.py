"""Documents12 and repeated-stop regressions against actual linked ARM code.
Sensor, encoder and motor interfaces are stubs; this is not a hardware test.
"""
from pathlib import Path
import sys,json
setup=Path(__file__).with_name('test_escape_completion.py').read_text(encoding='utf-8-sig')
exec(compile(setup.split('# Log regression:')[0],str(Path(__file__).with_name('test_escape_completion.py')),'exec'))

def frames(direction,distance=150,confidence=100,count=3):
    global clock
    for _ in range(count):
        clock+=20;call('Thief_OnLaserFrame',direction,distance,confidence,1,clock)

def early_junction():
    global odom,gray_raw,gray_state
    fresh(22,0);setstep(22,11,0,0,300);call('Route_Start')
    for _ in range(4):tick()
    odom=43;gray_raw=0xc60;gray_state=4
    tick()
    check(member('thief_localization','confirmed_node')==22 and not member('map_nav','arrived'),
          '43mm branch-shaped reading is not N011')
    check(var('junction_count')==0,'early reading cannot advance junction index')
    result=complete_route()
    check(result['node']==11,'ignored early branch does not prevent real N011 arrival')

def noisy_hide():
    global clock
    fresh(38,1)
    for distance,confidence in [(391,0),(380,60),(422,0),(387,20),(395,0)]:
        frames(2,distance,confidence,1)
        check(not var('thief_escape_requested'),'isolated Documents12 right-laser frame must not wake hide')
        tick();check(var('thief_strategy_state')==2,'no fresh confirmation means keep hiding')
    frames(2,380,60,2)
    check(not var('thief_escape_requested'),'two frames are not enough at rest')
    frames(2,385,60,1)
    check(var('thief_escape_requested')==1,'third consecutive valid frame wakes hide')
    tick();check(var('route_active')==1,'stable confirmed detection starts escape')

def repeated_side():
    fresh(38,1);frames(2,380,60);tick()
    check(at(current(),'MapRouteStep_t','heading')==1,'reported departure goes east away from south hit')
    for _ in range(4):
        frames(2,380,60,1);tick()
        check(var('route_active') and var('route_state')!=12,'known side hit does not cancel unmoved departure')
    frames(0,150,100,1);tick()
    check(var('route_state')==12,'new obstacle in the actual travel direction brakes immediately')
    result=complete_route();check(result['node'] in [11,38,26,35,63],'new interception still completes refuge route')

def rear_pursuit():
    fresh(38,1);setstep(38,39,1,0,300,action=0)
    setstep(39,40,1,0,300,index=1);record(route,'MapRoute_t',dict(count=2))
    call('Route_Start');call('Thief_UpdatePose');call('Thief_UpdateLaserMask',clock)
    for _ in range(5):frames(3,150,100,1);tick()
    check(at(route,'MapRoute_t','count')==2 and not var('replan_after_leg'),
          'rear-only pursuit keeps the complete route instead of stopping at N039')

def brake_credit():
    global odom,clock,gray_raw,gray_state
    for slide in [0,30,70]:
        fresh(38,1,0);setstep(38,37,3,1,300);call('Route_Start')
        var('replan_after_leg',1);odom=300;clock+=400
        call('Route_HandleJunction',clock,20,5,0xbff)
        tick(40);tick(60)
        odom=slide
        setstep(37,20,0,0,600);call('Route_Start')
        check(var('route_state')==(4 if slide==70 else 3),
              f'N037 root approach credits braking travel {slide=}')
        if 'departure_approach_ms' in symbols:
            expected=180-min(180,slide*600000000//(1000*204204))
            check(var('departure_approach_ms')==expected,'remaining approach matches existing wheel scale')
        # Turn/start code and real navigator must still complete the outgoing edge.
        result=complete_route();check(result['node']==20,'N037 credit does not skip outgoing edge')

def launch_speed():
    global odom
    targets=[]
    def capture():
        targets.append(tuple(int.from_bytes(u.reg_read(r).to_bytes(4,'little'),'little',signed=True)
                       for r in [UC_ARM_REG_R0,UC_ARM_REG_R1,UC_ARM_REG_R2,UC_ARM_REG_R3]));return 0
    hook('TravelFrame_SetWheelTargets',capture)
    fresh(38,1);setstep(38,37,3,1,300);call('Route_Start')
    for _ in range(6):tick()
    check(targets and max((a+b)//2 for a,b,_,_ in targets)<=1000,'reverse startup acquires line at <=100 RPM')
    odom=150;targets.clear()
    for _ in range(5):tick()
    check(any((a+b)//2>1000 for a,b,_,_ in targets),'normal cruise resumes after departure zone')
    event('TravelFrame_SetWheelTargets')

def brake_credit_matrix():
    global odom,clock
    cases=0
    def road(node,h):
        if not call('ThiefMapPlanner_HasRoad',node,h,0x2000c400,0x2000c402):return None
        target,length=get(0x2000c400,2),get(0x2000c402,2)
        if not at(call('ThiefMapPlanner_Node',target),'MapRouteNode_t','kind'):return None
        return target,length
    for node in [11,38,26,35,63,37,78]:
      for entry_road in range(4):
        incoming=road(node,entry_road)
        if incoming is None:continue
        source,length=incoming;heading=(entry_road+2)%4
        for mode in [0,1]:
          body=(heading+2*mode)%4
          for out in [(body+1)%4,(body+3)%4]:
            outgoing=road(node,out)
            if outgoing is None:continue
            target,next_length=outgoing
            for slide in [0,30,70]:
                fresh(source,body,0);setstep(source,node,heading,mode,length);call('Route_Start')
                var('replan_after_leg',1);odom=length;clock+=600
                call('Route_HandleJunction',clock,20,3,0xfff)
                check(var('thief_current_node')==node,'matrix arrival reaches expected real node')
                tick(40);tick(60);odom=slide
                setstep(node,target,out,0,next_length);call('Route_Start')
                expected=180-min(180,slide*600000000//(1000*204204))
                check(var('departure_approach_ms')==expected and var('route_state')==(4 if not expected else 3),
                      f'braking credit applies to every physical entry frame {node=} {mode=} {body=} {out=} {slide=}')
                check(call('TravelFrame_IsReverse')==mode,'credited approach preserves actual incoming frame')
                cases+=1
    return cases

matrix_cases=0
failures=[];passed=[]
for fn in [early_junction,noisy_hide,repeated_side,rear_pursuit,brake_credit,launch_speed,brake_credit_matrix]:
    try:
        value=fn()
        if fn==brake_credit_matrix:matrix_cases=value
        passed.append(fn.__name__);print(fn.__name__+': PASS',flush=True)
    except AssertionError as e:failures.append(dict(case=fn.__name__,error=str(e)));print(fn.__name__+': FAIL '+str(e),flush=True)
result=dict(checks=checks,failures=failures,passed=passed,hardware_tested=False,brake_credit_matrix_cases=matrix_cases,
 helper_sha256={name:hashlib.sha256((ROOT/'Tests'/name).read_bytes()).hexdigest() for name in ['test_app_runtime.py','test_escape_completion.py']},
 firmware_sha256=hashlib.sha256(axf.read_bytes()).hexdigest(),
 app_source_sha256=hashlib.sha256((ROOT/'User/App_ThiefStrategy.c').read_bytes()).hexdigest(),
 test_script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
out=ROOT/'Tests'/('before_documents12/regression_before.json' if '--before' in sys.argv else 'results_documents12.json')
out.write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
print(json.dumps(result,indent=2))
sys.exit(bool(failures))
