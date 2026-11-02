"""Actual ARM MONITOR13, synthetic sensors. No motor or physical localization proof."""
from pathlib import Path
import json,hashlib,sys
setup=Path(__file__).with_name('test_documents15.py').read_text(encoding='utf-8').split('parking_pwm=[]')[0]
if '--before13' in sys.argv:
    setup=setup.replace("runtime=Path", "runtime=Path",1)
    # Load only runtime fixture, replacing the image before its ELF is opened.
    setup=Path(__file__).with_name('test_app_runtime.py').read_text(encoding='utf-8').split('def documents8_cases():')[0]
    setup=setup.replace("axf=ROOT/'MDK-ARM'/ROOT.name/(ROOT.name+'.axf')", "axf=ROOT/'Tests/before_topology_recovery/original.axf'")
exec(compile(setup,str(Path(__file__).with_name('test_app_runtime.py')),'exec'))

def signed(x):return x-2**32 if x>=2**31 else x
def run_before():
    s=allocate('RestartLineSearch',0x2000c400)
    inp=allocate('RestartLineInput',0x2000c440);out=allocate('RestartLineOutput',0x2000c480)
    record(inp,'RestartLineInput',dict(yaw_known=1,leading_valid=0,trailing_valid=0,stopped=1))
    call('RestartLineSearch_Reset',s,0);call('RestartLineSearch_Probe',s,inp,1,out)
    a=at(out,'RestartLineOutput','left_x10') if 'at' in globals() else get(out,2)
    b=get(out+2,2)
    assert a+b>0,'baseline should reproduce blind forward motion'
    (ROOT/'Tests/before_topology_recovery/regression_before.json').write_text(json.dumps(dict(
        expected_old_rule_violation=True,left_x10=a,right_x10=b,
        axf_sha256=hashlib.sha256(axf.read_bytes()).hexdigest(),hardware_tested=False),indent=2),encoding='utf-8')
    print('Baseline reproduced: both gray boards white still commanded positive translation')
    return

if '--before13' in sys.argv:
    run_before();sys.exit(0)

roadbuf=0x2000c500;lenbuf=roadbuf+4
def road(n,h):
    if call('ThiefMapPlanner_HasRoad',n,h,roadbuf,lenbuf):return get(roadbuf,2),get(lenbuf,2)
    return None
def ports(n,h):
    return (1 if road(n,(h+3)%4) else 0)|(2 if road(n,(h+1)%4) else 0)
def topo_items(b):
    off,t=fields(types['ThiefTopoBelief'])['item'];stride=size(types['ThiefTopoCandidate'])
    return [(at(b+off+i*stride,'ThiefTopoCandidate','node'),at(b+off+i*stride,'ThiefTopoCandidate','cost'))
            for i in range(at(b,'ThiefTopoBelief','count'))]

results={};b=allocate('ThiefTopoBelief',0x2000c600)
check(size(types['ThiefTopoBelief'])<=40,'fixed candidate storage <=40 bytes')
count=0
for n in range(1,89):
    node=call('ThiefMapPlanner_Node',n)
    if not at(node,'MapRouteNode_t','kind'):continue
    for h in range(4):
        prev=road(n,(h+2)%4)
        if not prev:continue
        call('ThiefTopo_Seed',b,prev[0],h,prev[1],ports(n,h),int(bool(road(n,h))))
        items=topo_items(b)
        check(0<len(items)<=8,'bounded candidates on each real incoming edge')
        check(all(road(i,(h+2)%4) for i,c in items),'all candidates have an actual incoming road')
        check(all((ports(n,h)&~ports(i,h))==0 for i,c in items),'seen side branches exist in each candidate')
        check(call('ThiefTopo_Resolve',b)==0,'a single junction never certifies a guessed location')
        count+=1
results['seed_incoming_edges']=count

# Follow actual directed map sequences (including turns); a resolved answer
# must match the synthetic physical node, not merely a manually assigned score.
graph={(n,h):road(n,h) for n in range(1,89) for h in range(4)}
kinds={n:at(call('ThiefMapPlanner_Node',n),'MapRouteNode_t','kind') for n in range(1,89)}
def next_junction(n,h):
    distance=0
    for _ in range(88):
        edge=graph[n,h]
        if not edge:return None
        n,d=edge;distance+=d
        if kinds[n]:return n,distance
    raise AssertionError('marker cycle')
sequences=resolved=0
for n in range(1,89):
  if not kinds[n]:continue
  for heading in range(4):
    prev=graph[n,(heading+2)%4]
    if not prev or not kinds[prev[0]]:continue
    call('ThiefTopo_Seed',b,prev[0],heading,prev[1],ports(n,heading),int(bool(graph[n,heading])))
    actual=n;h=heading
    for step_index in range(5):
      choices=[q for q in [(h+1)%4,h,(h+3)%4] if graph[actual,q]]
      if not choices:break
      h=choices[(n+step_index)%len(choices)]
      actual,length=next_junction(actual,h)
      seen=ports(actual,h);straight=int(bool(graph[actual,h]))
      call('ThiefTopo_Observe',b,h,length,seen,straight)
      fix=call('ThiefTopo_Resolve',b)
      check(not fix or fix==actual,'real topology sequence must never resolve to another node')
      if fix:resolved+=1
    sequences+=1
check(resolved>0,'actual map sequences can recover a position')
results['actual_map_sequences']=dict(paths=sequences,resolved_observations=resolved)

# N005 must never be a fix; odometry cannot force an impossible shape.
call('ThiefTopo_Seed',b,4,1,150,3,-1)
check(all(n!=5 for n,c in topo_items(b)),'treasure marker is not an observed junction')
old=bytes(u.mem_read(b,size(types['ThiefTopoBelief'])))
call('ThiefTopo_Observe',b,1,100,255,0)
check(bytes(u.mem_read(b,len(old)))==old,'contradictory branch cannot erase history or fabricate a fix')

# Real update/resolve rule: same-shaped candidates persist, a distinct sequence may resolve.
record(b,'ThiefTopoBelief',dict(count=2,heading=1,events=2,omitted_cost=65535))
off,_=fields(types['ThiefTopoBelief'])['item'];stride=size(types['ThiefTopoCandidate'])
record(b+off,'ThiefTopoCandidate',dict(node=6,cost=3))
record(b+off+stride,'ThiefTopoCandidate',dict(node=7,cost=4))
check(call('ThiefTopo_Resolve',b)==0,'near tied candidates remain uncertain')
record(b+off+stride,'ThiefTopoCandidate',dict(node=7,cost=14))
check(call('ThiefTopo_Resolve',b)==6,'two-event score gap can resolve without pretending percent confidence')
record(b,'ThiefTopoBelief',dict(omitted_cost=5))
check(call('ThiefTopo_Resolve',b)==0,'pruned equal-quality candidates prevent false confidence')

# The new no-blind-translation rule applies in either travel frame.
s=allocate('RestartLineSearch',0x2000c700);inp=allocate('RestartLineInput',0x2000c740)
out=allocate('RestartLineOutput',0x2000c780)
for mode in [0,1]:
  call('TravelFrame_SetReverse',mode)
  for err in [-900,-100,0,100,900]:
    record(inp,'RestartLineInput',dict(yaw_known=1,yaw_cw_x100=err,leading_valid=0,trailing_valid=0,stopped=1))
    call('RestartLineSearch_Reset',s,0)
    for _ in range(7):
      call('RestartLineSearch_Probe',s,inp,1,out)
      a=at(out,'RestartLineOutput','left_x10');a=a-65536 if a>=32768 else a
      d=at(out,'RestartLineOutput','right_x10');d=d-65536 if d>=32768 else d
      check(a+d==0,'both white permits brake/rotation, never translation')

# Local corner control uses real yaw sign, reverse sensor frame and actual center line.
raw_rear=0x60
def read_rear_local():
    record(u.reg_read(UC_ARM_REG_R0),'RouteGraySample_t',dict(rear_raw12=raw_rear,rear_error_x100=0))
    return 1 if raw_rear else 2
hook('Route_ReadRear',read_rear_local)
hook('MapRoutePosition_ForwardCounts',lambda:odom)
hook('MapRoutePosition_DistanceMm',lambda:odom-u.reg_read(UC_ARM_REG_R0))
for mode in [0,1]:
  for sign in [-1,1]:
    fresh(53,(0+2*mode)%4);member('thief_localization','yaw_cw_sign',sign)
    setstep(53,44,0,mode,300);call('Route_Start')
    check(var('route_active') and member('thief_localization','phase')==1 and not member('map_nav','arrived'),
          'actual Route_Start establishes the incoming leg, without forced success flags')
    yaw=0
    # N036 has a real west branch when arriving northward from N044.
    # Use its actual L-shaped topology, not an invented two-sided T.
    gray_raw=0;gray_state=2;raw_rear=0xff;odom=50
    var('local_branch_counts',0)
    check(call('Route_BeginLocal',clock,0xfc0,1)==1,'rear-supported incompatible corner enters line-first localization')
    check(member('thief_localization','confirmed_node')==53,'entry cannot change confirmed node')
    check(var('local_phase')==1,'observer entry has no fake turn completion')
    for _ in range(3):clock+=20;call('Route_LocalTask',clock,20)
    check(var('local_phase')==2,'three real front-white/rear-black frames recognize no straight continuation')
    clock+=200;call('Route_LocalTask',clock,20)
    clock+=20;call('Route_LocalTask',clock,20)
    check(var('local_phase')==4,'actual stop and fresh yaw precede local turn')
    direction=var('local_turn');q=1 if direction==1 else -1
    yaw=sign*q*5200
    clock+=20;call('Route_LocalTask',clock,20)
    check(var('local_phase')==4,'52deg is not a completed gray-confirmed quarter turn')
    var('local_phase',6);yaw=sign*q*6900
    clock+=20;call('Route_LocalTask',clock,20)
    check(var('local_phase')==8,'angular scan endpoint brakes rather than latching a permanent stop')
    clock+=20;call('Route_LocalTask',clock,20)
    check(var('local_phase')==4,'fresh stopped feedback re-arms angular acquisition without blind translation')
    yaw=sign*q*9000;gray_raw=0x60;gray_state=1
    for _ in range(2):clock+=20;call('Route_LocalTask',clock,20)
    check(var('local_phase')==5,'actual centered leading gray and turn angle end rotation')
    clock+=20;call('Route_LocalTask',clock,20)
    check(var('local_phase')==1 and var('local_heading')==direction,'line-follow resumes on actually observed branch')
    check(member('thief_localization','confirmed_node')==53,'turn alone is not a global position fix')
    if mode==0 and sign==1:
      # Follow real N036 westward junctions to a global reanchor. Sensors and
      # odometry are synthetic, but the entire application observer is executed.
      saved=bytes(u.mem_read(0x20000000,0x10000));saved_vars=(clock,odom,gray_raw,gray_state,raw_rear,yaw)
      actual=36;h=3;fixes=[]
      for event_index in range(5):
        gray_raw=0x60;gray_state=1;raw_rear=0x60;odom+=35
        clock+=20;call('Route_LocalTask',clock,20)
        edge=next_junction(actual,h)
        if not edge:break
        actual,length=edge;odom+=max(0,length-35)
        seen=ports(actual,h);gray_raw=0x60|(0xc00 if seen&1 else 0)|(3 if seen&2 else 0)
        gray_state=3 if seen==3 else 4 if seen&1 else 5
        for _ in range(3):clock+=20;call('Route_LocalTask',clock,20)
        odom+=35
        straight=bool(graph[actual,h]);gray_raw=0x60 if straight else 0
        gray_state=1 if straight else 2;raw_rear=0x60
        for _ in range(3):clock+=20;call('Route_LocalTask',clock,20)
        if var('local_phase')==2:
          clock+=200;call('Route_LocalTask',clock,20)
          clock+=20;call('Route_LocalTask',clock,20)
        if not var('local_phase'):
          check(member('thief_localization','confirmed_node')==actual,'application reanchors at the actual observed physical node')
          fixes.append(actual);break
        if var('local_phase')==4:
          q=1 if var('local_turn')==1 else -1;h=(h+q)%4
          check(graph[actual,h] is not None,'local turn follows an actual observed road')
          yaw=(yaw+q*9000+18000)%36000-18000;gray_raw=0x60;gray_state=1
          for _ in range(3):clock+=20;call('Route_LocalTask',clock,20)
      check(fixes,'real application sequence must reach a confirmed reanchor, not just a pure-algorithm score')
      results['application_reanchor']=fixes
      u.mem_write(0x20000000,saved);clock,odom,gray_raw,gray_state,raw_rear,yaw=saved_vars
    direction=var('local_heading');old_mode=call('TravelFrame_IsReverse')
    # Fresh obstacle in the current travel front: measured brake precedes frame swap.
    clock+=1;call('Thief_OnLaserFrame',3 if old_mode else 0,150,100,1,clock)
    check(var('local_reverse_pending')==1,'localization uncertainty does not disable front obstacle detection')
    clock+=20;call('Route_LocalTask',clock,20)
    check(var('local_phase')==7 and call('TravelFrame_IsReverse')==old_mode,'obstacle first brakes, never swaps a moving frame')
    clock+=20;call('Route_LocalTask',clock,20)
    check(call('TravelFrame_IsReverse')==(old_mode^1) and var('local_heading')==(direction+2)%4,'measured stop swaps front/rear and travel heading once')
results['local_turn_frames']=4

# Continuous two-board loss survives phase changes, rather than restarting a timer.
gray_raw=0;gray_state=2;raw_rear=0;var('local_phase',1);var('map_gray_missing_active',1)
var('map_gray_missing_tick',clock-2000);call('Route_LocalTask',clock,20)
check(var('route_active')==0 and reason()=='GRAY_BOTH_NO_LINE','local line search retains the continuous two-second loss stop')

# N011 is always E/W, including all threat masks and emergency tier paths.
for body in range(4):
  for mask in range(16):
    fresh(11,body)
    if call('ThiefMapPlanner_PlanEscape',11,body,mask,0,clock,plan):
      check(at(steps,'MapRouteStep_t','to_node') in [10,12],'N011 cannot depart toward N002/N022')
      if at(steps,'MapRouteStep_t','to_node')==12:
        check(at(steps+16,'MapRouteStep_t','to_node')==13,'east exit continues to N013')
      if mask==4:check(at(steps,'MapRouteStep_t','to_node')==12,'south threat prefers N013 corridor')
      if mask==1:check(at(steps,'MapRouteStep_t','to_node')==10,'north threat prefers N010 corridor')
results['n011_body_masks']=64
report=dict(firmware='MONITOR13',checks=checks,cases=results,axf_sha256=hashlib.sha256(axf.read_bytes()).hexdigest(),
            hardware_tested=False,candidate_bytes=size(types['ThiefTopoBelief']))
(ROOT/'Tests/results_topology_linefirst.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps(report,ensure_ascii=False),flush=True)
