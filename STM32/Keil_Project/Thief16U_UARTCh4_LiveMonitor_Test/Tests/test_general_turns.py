"""Whole-map turn-entry audit and N011 world-direction exit policy; synthetic hardware."""
from pathlib import Path
import re,sys,json,hashlib
src=Path(__file__).with_name('test_documents15.py').read_text(encoding='utf-8').split('parking_pwm=[]')[0]
src=src.replace('Tests/before_documents15/original.axf','Tests/before_general_turns/original.axf')
exec(compile(src,str(Path(__file__).with_name('test_documents15.py')),'exec'))
data=(ROOT/'Algorithm/MapRoute/MapRoutePlan.c').read_text(encoding='utf-8')
node_text=data.split('const MapRouteNode_t map_route_nodes[] = {')[1].split('};')[0]
nodes={int(a):(int(kind),int(degree)) for a,x,y0,kind,degree in re.findall(r'\{(\d+),\s*(\d+),\s*(\d+),\s*(\d+),\s*(\d+)\}',node_text)}

def neighbor(node,heading):
    total=0;segments=0;current=node
    for _ in range(4):
        if not call('ThiefMapPlanner_HasRoad',current,heading,0x2000c400,0x2000c402):return None
        current=get(0x2000c400,2);total+=get(0x2000c402,2);segments+=1
        if nodes[current][0]!=0:return current,total,segments
    return None

def setup_turn(node,incoming,quarter,mode,sign):
    global clock,odom,yaw,gray_raw,gray_state,rear_valid
    source,length,segments=neighbor(node,(incoming+2)%4)
    target,out_length,out_segments=neighbor(node,(incoming+quarter)%4)
    fresh(source,(incoming+2*mode)%4);member('thief_localization','yaw_cw_sign',sign)
    setstep(source,node,incoming,mode,length,action=quarter)
    record(steps,'MapRouteStep_t',dict(segment_count=segments))
    setstep(node,target,(incoming+quarter)%4,mode,out_length,index=1)
    record(steps+16,'MapRouteStep_t',dict(segment_count=out_segments));record(route,'MapRoute_t',dict(count=2))
    call('Route_Start');check(var('route_active'),'actual map road starts '+reason())
    clock+=500;odom=length
    left=neighbor(node,(incoming+3)%4);right=neighbor(node,(incoming+1)%4)
    raw=0x60|(0xc00 if left else 0)|(3 if right else 0)
    call('Route_HandleJunction',clock,20,3 if left and right else 4 if left else 5,raw)
    check(member('thief_localization','confirmed_node')==node and member('thief_localization','phase')==2,'actual gray anchors incoming node')
    var('turn_start_angle_x100',18000 if mode else 0)
    yaw=sign*quarter*8000;gray_raw=0x60;gray_state=1;rear_valid=1;events.clear()
    call('Route_EnterAfterTurnForward',clock,20,0x2000ce00,yaw)

def all_map_quarter_turns():
    global clock,yaw
    check('turn_axis_active' in symbols and 'Route_AlignTurnAxis' in symbols,'one shared axis stage exists for all mapped turns')
    count=0;covered=set()
    for node,(kind,degree) in nodes.items():
      if not kind or degree<2:continue
      for incoming in range(4):
       for quarter in [-1,1]:
        if not neighbor(node,(incoming+2)%4) or not neighbor(node,(incoming+quarter)%4):continue
        for mode in [0,1]:
         for sign in [-1,1]:
          setup_turn(node,incoming,quarter,mode,sign)
          check(var('turn_axis_active') and 'Route_DriveStraight' not in events,'no map node bypasses axis alignment')
          yaw=sign*quarter*9000
          for _ in range(12):clock+=20;call('Route_AlignTurnAxis',clock,20)
          check(not var('turn_axis_active'),'shared map axis reaches ready without a fabricated node')
          check(member('thief_localization','confirmed_node')==node,'turn alignment cannot certify the next node')
          count+=1;covered.add(node)
    check(78 in covered,'N078 included in the whole-map matrix')
    return dict(combinations=count,nodes=sorted(covered))

def n078_original_log():
    global yaw,clock
    check('turn_axis_active' in symbols,'N078 shared axis stage')
    for mode in [0,1]:
      for sign in [-1,1]:
        setup_turn(78,1,-1,mode,sign)
        yaw=-sign*5281;ret('Route_AllWheelsStopped',0)
        for _ in range(8):clock+=20;call('Route_AlignTurnAxis',clock,20)
        check(var('turn_axis_active') and member('thief_localization','phase')==2,'52.81degree log sample cannot start the outgoing road')
        ret('Route_AllWheelsStopped',1);yaw=-sign*9000
        for _ in range(20):clock+=20;call('Route_AlignTurnAxis',clock,20)
        check(not var('turn_axis_active'),'N078 can finish the outgoing axis')
    return 4

def n011_exit_policy():
    global clock
    for body in range(4):
      for threat,expected in [(4,12),(1,10)]:
        fresh(11,body);call('Thief_UpdateLaserMask',clock)
        world=2 if threat==4 else 0
        channel={0:0,1:2,2:3,3:1}[(world-body)%4]
        for _ in range(3):clock+=20;call('Thief_OnLaserFrame',channel,300,100,1,clock)
        check(var('hide_escape_mask')&threat,'physical laser maps to the requested world direction')
        check(call('ThiefMapPlanner_PlanEscape',11,body,threat,0,clock,plan),'preferred corridor has a complete hide route')
        check(at(steps,'MapRouteStep_t','to_node')==expected,'N011 world-direction priority selects correct first exit')
        if threat==4:check(at(steps+16,'MapRouteStep_t','to_node')==13,'south pursuer leaves through N012 then N013')
    for a,b,mask in [(11,12,4),(12,13,4),(11,10,1)]:
        fresh(11,3);call('ThiefIntercept_BlockLeg',a,b,clock)
        check(call('ThiefMapPlanner_PlanEscape',11,3,mask,0,clock,plan),'blocked preferred corridor retains fallback')
        count=at(route,'MapRoute_t','count')
        check(all({at(steps+16*i,'MapRouteStep_t','from_node'),at(steps+16*i,'MapRouteStep_t','to_node')}!={a,b} for i in range(count)),'fallback never crosses known blocked preferred edge')
    fresh(11,3)
    check(call('ThiefMapPlanner_PlanEscape',11,3,6,0,clock,plan),'south plus east threat has a legal alternative')
    check(at(steps,'MapRouteStep_t','to_node')!=12,'east threat overrides south-to-east preference')
    fresh(11,3)
    check(call('ThiefMapPlanner_PlanEscape',11,3,5,0,clock,plan),'opposed north/south threats retain complete planning')
    check(at(steps,'MapRouteStep_t','to_node') in (10,12),'north/south conflict uses an actual clear lateral exit')
    fresh(11,1)
    check(call('ThiefMapPlanner_PlanEscape',11,1,9,0,clock,plan),'north plus west threat overrides the west preference')
    check(at(steps,'MapRouteStep_t','to_node')!=10,'immediate west threat prevents preferred west departure')
    fresh(38,1)
    check(call('ThiefMapPlanner_PlanEscape',38,1,1,0,clock,plan),'other node remains plannable after N011')
    check(at(steps,'MapRouteStep_t','to_node')==39,'N011 search constraints do not leak to N038')
    return dict(body_directions=4,world_threats=2,blocked_cases=3)

failures=[];cases={}
for fn in [all_map_quarter_turns,n078_original_log,n011_exit_policy]:
    try:cases[fn.__name__]=fn()
    except Exception as e:failures.append(dict(case=fn.__name__,error=str(e)))
axf=ROOT/'MDK-ARM'/ROOT.name/(ROOT.name+'.axf')
if '--before' in sys.argv:axf=ROOT/'Tests/before_general_turns/original.axf'
r=dict(checks=checks,cases=cases,failures=failures,axf_sha256=hashlib.sha256(axf.read_bytes()).hexdigest(),
       test_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),hardware_tested=False)
dest=ROOT/('Tests/before_general_turns/regression_before.json' if '--before' in sys.argv else 'Tests/results_general_turns.json')
dest.write_text(json.dumps(r,indent=2),encoding='utf-8');print(json.dumps(r,indent=2));sys.exit(bool(failures))
