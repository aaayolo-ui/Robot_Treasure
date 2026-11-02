"""185614 text interference and N038 north-threat regressions, linked ARM."""
from pathlib import Path
import hashlib,json,sys
src=Path(__file__).with_name('test_documents15.py').read_text(encoding='utf-8').split('parking_pwm=[]')[0]
src=src.replace("if '--before' in sys.argv:","if False:")
if '--before' in sys.argv:
    src=src.replace('exec(compile(runtime,',"runtime=runtime.replace(\"axf=ROOT/'MDK-ARM'/ROOT.name/(ROOT.name+'.axf')\",\"axf=ROOT/'Tests/before_treasure_fix/original.axf'\")\nexec(compile(runtime,")
exec(compile(src,str(Path(__file__).with_name('test_documents15.py')),'exec'))

def text_mask_matrix():
    total=0
    for a,b,h in [(3,4,1),(4,6,1),(6,7,1),(7,6,3),(6,4,3),(4,3,3)]:
      for mode in [0,1]:
        fresh(a,(h+2*mode)%4);setstep(a,b,h,mode,300)
        for leading in [0,1]:
          mask=0xf00 if (h==1 and leading) or (h==3 and not leading) else 0x00f
          for raw in [0,0x60,0xf60,0x06f,0xfff,0xc00,0x003,0xc63]:
            got=call('TreasureGrayMask_Apply',steps,1,leading,raw)
            check(got==raw&~mask,'north-side text excluded; real center and south side preserved')
            total+=1
    fresh(38,1);setstep(38,39,1,0,300)
    check(call('TreasureGrayMask_Apply',steps,1,1,0xfff)==0xfff,'ordinary roads are not masked')
    return total

def side_fragments_are_not_nodes():
    total=0
    for a,b,h in [(3,4,1),(4,6,1),(6,15,2),(15,25,2)]:
      fresh(a,h);setstep(a,b,h,0,300)
      for raw in [0x003,0x007,0x00f,0xc00,0xe00,0xf00]:
        check(call('ThiefLocalization_GrayMatchesStep',steps,raw)==0,'isolated edge fragment cannot confirm a mapped node')
        total+=1
      raw=0x60
      if call('ThiefMapPlanner_HasRoad',b,(h+3)%4,0,0):raw|=0xc00
      if call('ThiefMapPlanner_HasRoad',b,(h+1)%4,0,0):raw|=3
      check(call('ThiefLocalization_GrayMatchesStep',steps,raw)==1,'real center plus mapped branch still confirms')
    return total

def eighteen_ms_restart():
    global clock,odom,gray_raw,gray_state
    fresh(4,3);setstep(4,6,1,1,300);record(steps,'MapRouteStep_t',dict(segment_count=2));call('Route_Start')
    check(var('route_active') and member('thief_localization','phase')==1,'real compressed N004-N006 leg starts successfully')
    for _ in range(3):clock+=5;call('Route_UpdateJunctionArm',1)
    clock+=3;odom=13;fixes=member('thief_localization','confirmations')
    call('Route_HandleJunction',clock,20,5,0x63)
    check(member('thief_localization','confirmed_node')==4 and member('thief_localization','confirmations')==fixes,'18ms/13mm restart cannot become next node even with center black')
    check(var('route_active') and not member('map_nav','arrived'),'early candidate keeps driving rather than latching a fault')
    clock+=400;odom=300
    call('Route_HandleJunction',clock,20,5,0x63)
    check(member('thief_localization','confirmed_node')==6,'later true N006 branch is accepted')
    return 1

def n007_turn_phase():
    global clock,odom,yaw
    count=0
    for mode in [0,1]:
      for sign in [-1,1]:
        fresh(17,2*mode);member('thief_localization','yaw_cw_sign',sign)
        setstep(17,7,0,mode,300,action=-1)
        setstep(7,6,3,mode,600,index=1);record(route,'MapRoute_t',dict(count=2))
        call('Route_Start');clock+=500;odom=300
        call('Route_HandleJunction',clock,20,4,0xc60)
        check(member('map_nav','arrived'),'actual N007 arrival precedes text-side turn filtering')
        yaw=sign*800
        check(call('Route_FilterTreasureGray',0xfff,1)==0xfff,'N007 initial turn including8deg yaw remains unaffected')
        yaw=-sign*6000
        check(call('Route_FilterTreasureGray',0xfff,1)==0xff0,'after turning toward the text road, mask the actual north side')
        check(call('Route_FilterTreasureGray',0xfff,0)==0x0ff,'trailing board uses its real mirrored installation')
        count+=1
    return count

def fragment_retry_then_complete():
    global odom,gray_raw,gray_state
    fresh(4,3)
    call('ThiefIntercept_BlockLeg',4,3,clock) #Pin this historical road replay, not the global planning policy.
    check(call('ThiefMapPlanner_PlanEscape',4,3,4,0,clock,plan),'actual N004 escape route exists')
    check(at(steps,'MapRouteStep_t','to_node')==6,'replay starts on actual compressed N004-N006 leg')
    call('Route_Start')
    for _ in range(5):tick()
    fixes=member('thief_localization','confirmations');gray_raw=7;gray_state=5
    for progress in [13,20,30,50,70,90]:
        odom=progress;tick()
        check(member('thief_localization','confirmed_node')==4 and member('thief_localization','confirmations')==fixes,'repeated007 edge fragments keep driving without confirming N006')
    gray_raw=0x60;gray_state=1
    for _ in range(4):tick()
    odom=900;gray_raw=0x63;gray_state=5;tick()
    check(member('thief_localization','confirmed_node')==6,'real centered branch confirms N006 even with late auxiliary encoder')
    result=complete_route()
    check(result['node'] in [11,26,35,38,63],'after rejecting fragments, real turns complete the full hide transfer')
    return result

def n038_north_runway():
    results=[]
    for body in [1,3]:
      fresh(38,body)
      check(call('ThiefMapPlanner_PlanEscape',38,body,1,0,clock,plan),'north-threat complete escape route exists')
      check(at(steps,'MapRouteStep_t','to_node')==39 and at(steps+16,'MapRouteStep_t','to_node')==40,'use actual east runway N038-N039-N040 before generic opposite direction')
      call('Route_Start');result=complete_route();check(result['node'] in [11,26,35,38,63],'east runway reaches an actual legal hide')
      results.append(result)
    fresh(38,1);call('ThiefIntercept_BlockLeg',38,39,clock)
    check(call('ThiefMapPlanner_PlanEscape',38,1,1,0,clock,plan),'blocked east departure retains a legal fallback')
    check(at(steps,'MapRouteStep_t','to_node')!=39,'never force east into a blocked edge')
    fresh(38,1)
    check(call('ThiefMapPlanner_PlanEscape',38,1,3,0,clock,plan),'multiple threat fallback exists')
    check(at(steps,'MapRouteStep_t','to_node')!=39,'fresh east threat overrides north-to-east preference')
    fresh(38,1);call('ThiefIntercept_BlockLeg',39,40,clock)
    check(call('ThiefMapPlanner_PlanEscape',38,1,1,0,clock,plan),'blocked second east edge retains a full alternative route')
    for i in range(at(route,'MapRoute_t','count')):
      s=steps+i*16
      check(not(at(s,'MapRouteStep_t','from_node')==39 and at(s,'MapRouteStep_t','to_node')==40),'second runway edge is never forced through an actual block')
    return results

failures=[];cases={}
for fn in [text_mask_matrix,side_fragments_are_not_nodes,eighteen_ms_restart,n007_turn_phase,n038_north_runway,fragment_retry_then_complete]:
    try:cases[fn.__name__]=fn();print(fn.__name__+': PASS',flush=True)
    except AssertionError as e:failures.append(dict(case=fn.__name__,error=str(e)));print(fn.__name__+': FAIL '+str(e),flush=True)
before='--before' in sys.argv
result=dict(checks=checks,failures=failures,cases=cases,axf_sha256=hashlib.sha256(axf.read_bytes()).hexdigest(),test_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),hardware_tested=False)
path=ROOT/'Tests'/('before_treasure_fix/regression_before.json' if before else 'results_treasure_fix.json')
path.write_text(json.dumps(result,indent=2),encoding='utf-8');print(json.dumps(result,indent=2),flush=True)
sys.exit(0 if before and len(failures)==6 else bool(failures))
