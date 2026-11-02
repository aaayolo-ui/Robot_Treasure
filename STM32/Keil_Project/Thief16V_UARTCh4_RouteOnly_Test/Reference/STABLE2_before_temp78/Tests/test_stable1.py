"""Execute linked STABLE1 with synthetic hardware; compare the actual RUN8 base."""
from pathlib import Path
import re,json,hashlib,sys
setup=Path(__file__).with_name('test_documents15.py').read_text(encoding='utf-8').split('parking_pwm=[]')[0]
if '--baseline' in sys.argv:
    setup=setup.replace('exec(compile(runtime,','runtime=runtime.replace("axf=ROOT/\'MDK-ARM\'/ROOT.name/(ROOT.name+\'.axf\')", "axf=ROOT/\'Reference/RUN8.axf\'")\nexec(compile(runtime,',1)
exec(compile(setup,str(Path(__file__).with_name('test_documents15.py')),'exec'))
def unhook(n):
    if n in hooks:u.hook_del(hooks.pop(n))
def fn(text,name):
    m=re.search(r'static [^;\n]+\b'+name+r'\([^;]*?\)\s*\{',text);assert m,name
    a=m.start();j=text.index('{',a);d=1;k=j+1
    while d:d+=(text[k]=='{')-(text[k]=='}');k+=1
    return text[a:k]

cases={};before='--baseline' in sys.argv
source=(ROOT/'User/App_ThiefStrategy.c').read_text(encoding='utf-8')
original=(ROOT/'Reference/App_RUN8.c').read_text(encoding='utf-8')
if not before:
    base_hashes=json.loads((ROOT/'Reference/RUN8_sources.json').read_text(encoding='utf-8'))
    matched=[]
    for rel_path,digest in base_hashes.items():
        if rel_path.startswith('Control/'):
            check(hashlib.sha256((ROOT/rel_path).read_bytes()).hexdigest()==digest,'complete original control module '+rel_path)
            matched.append(rel_path)
    cases['unchanged_control_files']=len(matched)
    functions=['Route_ReadFront','Route_ReadRear','Route_UpdateErrors','Route_UpdateFollowMode',
        'Route_ClassifyPosition','Route_DriveStraight','Route_DriveUnlocatedFollow','Route_DriveTurnApproach',
        'Route_DriveLeftTurn','Route_DriveRightTurn','Route_HasReachedTurnAngle',
        'Route_EnterTurnApproach','Route_EnterTurnStop']
    for motion_name in functions:check(fn(source,motion_name)==fn(original,motion_name),'complete baseline motion function '+motion_name)
    # Route_RunFollow differs only after sending the real PID/wheel targets.
    check(fn(source,'Route_RunFollow').split('  Route_UpdateFollowMode(sample);')[1].split('  Route_UpdateJunctionArm(front_state);')[0]==
          fn(original,'Route_RunFollow').split('  Route_UpdateFollowMode(sample);')[1].split('  Route_UpdateJunctionArm(front_state);')[0],
          'actual PID/heading/zone gains/smoothing/wheel update chain byte-identical to RUN8')
    cases['motion_functions']=len(functions)
    for key in ['ROUTE_TURN_TARGET_ANGLE_X100','ROUTE_TURN_EQUAL_PWM','ROUTE_TURN_APPROACH_RPM_X10',
        'ROUTE_TURN_APPROACH_DELAY_MS','ROUTE_AFTER_TURN_START_RPM_X10']:
        a=re.search(r'^#define\s+'+key+r'\s+(\w+)',source,re.M)[1]
        d=re.search(r'^#define\s+'+key+r'\s+(\w+)',original,re.M)[1]
        check(a==d,'baseline action parameter '+key)
    for removed_name in ['Route_TurnExitRecover','Route_MapGrayRecover','Route_AlignTurnAxis','Route_LocalTask']:
        check(removed_name not in source,'removed map motion takeover '+removed_name)

# Exact pre-change failure: observer rejects a sample and cancels normal motion.
fresh(38,1);setstep(38,39,1,0,300);call('Route_Start')
ret('ThiefLocalization_Update',0);member('thief_localization','error',5)
events.clear();call('Thief_UpdatePose')
if before:
    check(not var('route_active'),'RUN8 reproduces localization rejection stopping the car')
    result=dict(expected_rule_violation=True,reason=reason(),axf_sha256=hashlib.sha256(axf.read_bytes()).hexdigest())
    (ROOT/'Reference/observer_before.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    print(json.dumps(result));sys.exit(0)
check(var('route_active') and var('route_state')!=10,'position rejection cannot stop the stable motion controller')
check(not any(n in events for n in ['Route_BrakeControl','Route_DriveLeftTurn','Route_DriveRightTurn']),
      'observer has no actuator command on rejection')
check(var('observer_ok')==0 and member('thief_pose','valid'),'strategy keeps explicit route estimate while observer reports uncertainty')
unhook('ThiefLocalization_Update');cases['shadow_no_actuation']=1

fresh(38,1);setstep(38,39,1,0,300);call('Route_Start')
odom=2000;gray_raw=0x60;gray_state=1;tick()
check(var('route_active'),'odometry overrun with real line cannot trigger a hidden map-position stop')
cases['no_hidden_map_veto']=1

# Route execution proceeds using real gray even if the independent observer rejects it.
fresh(38,1);setstep(38,39,1,0,300,action=0)
setstep(39,40,1,0,300,index=1);record(route,'MapRoute_t',dict(count=2))
call('Route_Start');odom=250;clock+=500
ret('ThiefLocalization_Arrive',0);member('thief_localization','error',5)
call('Route_HandleJunction',clock,20,5,0x063)
check(var('route_active') and member('map_nav','arrived'),'real junction executes despite observer rejection')
ret('ThiefLocalization_ExitReady',0);ret('ThiefLocalization_CommitExit',0)
gray_raw=0x60;gray_state=1
for _ in range(3):tick()
check(member('map_nav','step')==1 and var('route_active'),'three actual leading samples advance the route without observer veto')
for api_name in ['ThiefLocalization_Arrive','ThiefLocalization_ExitReady','ThiefLocalization_CommitExit']:unhook(api_name)
cases['shadow_route_execution']=1

# The one recovery controller never translates with BOTH real boards white.
for mode in [0,1]:
    fresh(38,1);setstep(38,37 if mode else 39,3 if mode else 1,mode,300)
    call('Route_Start');check(var('route_active'),'real forward/reverse route starts in the same physical body heading')
    gray_raw=0;gray_state=2;rear_valid=2;targets.clear()
    call('Route_EnterFrontRecovery',clock,2)
    for _ in range(6):tick()
    check(all(a+b==0 for a,b,*_ in targets),'both-white search cannot issue blind forward/back translation')
    gray_raw=0x60;gray_state=1;rear_valid=1
    for _ in range(2):tick()
    check(var('route_active') and var('route_state')==2,
          f'actual leading line returns directly to baseline PID mode={mode} state={var("route_state")} count={var("front_recovery_valid_count")} reason={reason()}')
cases['single_gray_recovery']=2

fresh(38,1);setstep(38,39,1,0,300);call('Route_Start')
gray_raw=0;gray_state=2;rear_valid=2
call('Route_EnterFrontRecovery',clock,2);tick()
clock+=2000;call('App_ThiefStrategy_Task')
check(not var('route_active') and reason()=='GRAY_BOTH_NO_LINE','continuous actual two-board loss still stops after two seconds')
cases['both_white_deadline']=1

# Learn only the physical gyro sign from a real root turn, not position acceptance.
for mode in [0,1]:
  for sign in [-1,1]:
    for quarter,target,h in [(1,22,2),(-1,2,0)]:
      fresh(11,(1+2*mode)%4);member('thief_localization','yaw_cw_sign',0)
      setstep(11,target,h,mode,300);call('Route_Start')
      for _ in range(20):
        if var('route_state') in [5,6]:break
        tick()
      check(var('route_state') in [5,6],'root enters actual rotation before gyro sign learning')
      yaw=sign*quarter*5000;gray_raw=0;gray_state=2;events.clear();tick()
      expected=sign if sign>0 else 255
      check(member('thief_localization','yaw_cw_sign')==expected,'physical right/left PWM turn learns gyro sign independently')
      check('Route_DriveStraight' not in events,'coarse rotation over real white cannot launch blind forward burst')
cases['gyro_sign_root_turns']=8

for body in range(4):
  for mask in range(16):
    fresh(11,body)
    check(call('ThiefMapPlanner_PlanEscape',11,body,mask,0,clock,plan),'complete escape route exists at N011')
    first=at(steps,'MapRouteStep_t','to_node')
    check(first in [10,12],'N011 only west/east')
    if first==12:check(at(steps+16,'MapRouteStep_t','to_node')==13,'east path continues N012-N013')
    if mask==4:check(first==12,'south pursuer chooses east')
    if mask==1:check(first==10,'north pursuer chooses west')
cases['n011_body_masks']=64

r=dict(firmware='UARTCH4_16V_STABLE2',checks=checks,cases=cases,failures=[],
       axf_sha256=hashlib.sha256(axf.read_bytes()).hexdigest(),hardware_tested=False)
(ROOT/'Tests/results_stable1.json').write_text(json.dumps(r,indent=2),encoding='utf-8')
print(json.dumps(r,indent=2),flush=True)
