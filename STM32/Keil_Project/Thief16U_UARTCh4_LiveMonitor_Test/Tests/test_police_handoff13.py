"""COMPARE1 actual ARM control handoff; synthetic hardware, no physical proof."""
from pathlib import Path
import sys,json,hashlib,re
src=Path(__file__).with_name('test_documents15.py').read_text(encoding='utf-8').split('parking_pwm=[]')[0]
if '--baseline' in sys.argv:
    src=src.replace("exec(compile(runtime,", "runtime=runtime.replace(\"axf=ROOT/'MDK-ARM'/ROOT.name/(ROOT.name+'.axf')\",\"axf=ROOT/'Tests/reference/MONITOR12.axf'\")\nexec(compile(runtime,")
exec(compile(src,str(Path(__file__).with_name('test_documents15.py')),'exec'))
helpers=Path(__file__).with_name('test_general_turns.py').read_text(encoding='utf-8')
exec(helpers[helpers.index('data=(ROOT'):helpers.index('def all_map_quarter_turns(')])
exec(helpers[helpers.index('def n011_exit_policy():'):helpers.index('failures=[];cases={}')])

def relative_threshold():
    global yaw,clock
    count=0
    for mode in [0,1]:
      for sign in [-1,1]:
        for direction,target,heading in [(1,22,2),(-1,2,0)]:
          fresh(11,(1+2*mode)%4)
          member('thief_localization','yaw_cw_sign',sign)
          setstep(11,target,heading,mode,300);call('Route_Start');tick(20);tick(60)
          for _ in range(20):
              if var('route_state') in [5,6]:break
              tick()
          check(var('route_state') in [5,6],'real turn state for relative threshold')
          snapshot=bytes(u.mem_read(0x20000000,0x10000));saved_clock=clock
          for start in [0,17000,-17000]:
            for amount,expected in [(4499,0),(4500,1),(6142,1)]:
              u.mem_write(0x20000000,snapshot);clock=saved_clock
              var('turn_start_angle_x100',start)
              yaw=(start+direction*sign*amount-18000*mode+18000)%36000-18000
              tick()
              check((var('route_state')==7)==bool(expected),'actual task relative45degree threshold including wrap/sign/reverse')
              count+=1
    return count

def whole_map_handoff():
    count=0;covered=set()
    for node,(kind,degree) in nodes.items():
      if not kind or degree<2:continue
      for incoming in range(4):
       for quarter in [-1,1]:
        if not neighbor(node,(incoming+2)%4) or not neighbor(node,(incoming+quarter)%4):continue
        for mode in [0,1]:
         for sign in [-1,1]:
          setup_turn(node,incoming,quarter,mode,sign)
          check('Route_DriveStraight' in events,'police immediate handoff used at every mapped turn')
          check(var('route_state')==7,'actual after-turn state entered')
          check(member('thief_localization','confirmed_node')==node and member('thief_localization','phase')==2,'handoff does not fabricate next node or road commit')
          count+=1;covered.add(node)
    check({4,6,7,78}.issubset(covered),'treasure turns and N078 included')
    return dict(combinations=count,nodes=sorted(covered))

def root_gray_branches():
    global yaw,gray_state,gray_raw
    for mode in [0,1]:
      for turn,target,heading in [(1,22,2),(-1,2,0)]:
        fresh(11,(1+2*mode)%4);setstep(11,target,heading,mode,300)
        call('Route_Start');tick(20);tick(60)
        for _ in range(20):
            if var('route_state') in [5,6]:break
            tick()
        check(var('route_state') in [5,6],'real root turn begins')
        check(var('departure_turn')==1,'root departure context retained')
        yaw=turn*6142;tick()
        check(var('route_state')==7,'root turn hands to after-turn state')
        snapshot=bytes(u.mem_read(0x20000000,0x10000))
        gray_state=1;gray_raw=0x60;tick()
        check(var('route_state')==2,'real visible line enters FOLLOW')
        u.mem_write(0x20000000,snapshot)
        gray_state=2;gray_raw=0;tick()
        check(var('route_state')==8 and var('route_active'),'no line enters existing recovery rather than claiming FOLLOW')
    return 8

def white_at_coarse_handoff():
    global gray_raw,gray_state
    for mode in [0,1]:
      for sign in [-1,1]:
        setup_turn(78,1,-1,mode,sign)
        gray_raw=0;gray_state=2;events.clear()
        call('Route_EnterAfterTurnForward',clock,20,0x2000ce00,sign*-8000)
        check('Route_DriveStraight' not in events and 'Route_BrakeControl' in events,
              'coarse turn never launches one blind forward cycle over actual white')
    return 4

failures=[];cases={}
for fn in [relative_threshold,whole_map_handoff,root_gray_branches,white_at_coarse_handoff]:
    try:cases[fn.__name__]=fn()
    except Exception as e:failures.append(dict(case=fn.__name__,error=str(e)))
axf=ROOT/('Tests/reference/MONITOR12.axf' if '--baseline' in sys.argv else 'MDK-ARM/'+ROOT.name+'/'+ROOT.name+'.axf')
r=dict(checks=checks,cases=cases,failures=failures,axf_sha256=hashlib.sha256(axf.read_bytes()).hexdigest(),test_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),hardware_tested=False)
dest=ROOT/('Tests/results_compare_baseline.json' if '--baseline' in sys.argv else 'Tests/results_police_handoff13.json')
dest.write_text(json.dumps(r,indent=2),encoding='utf-8');print(json.dumps(r,indent=2));sys.exit(bool(failures))
