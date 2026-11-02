"""001220 complete-route choice and ambiguous N006/N007; synthetic hardware."""
from pathlib import Path
import sys,json,hashlib
src=Path(__file__).with_name('test_documents15.py').read_text(encoding='utf-8').split('parking_pwm=[]')[0]
src=src.replace('Tests/before_documents15/original.axf','Tests/before_route_identity/original.axf')
exec(compile(src,str(Path(__file__).with_name('test_documents15.py')),'exec'))

def n011_departure():
    global clock
    fresh(11,3);call('Thief_UpdateLaserMask',clock)
    for _ in range(3):
        clock+=20;call('Thief_OnLaserFrame',1,217,100,1,clock)
    check(call('ThiefMapPlanner_PlanEscape',11,3,4,0,clock,plan),'actual south threat has full escape')
    first=at(steps,'MapRouteStep_t','to_node')
    print('N011 first',first,'goal',member('thief_plan','goal_node'),flush=True)
    check(first in (10,12),'full-route score beats forced north pivot atN011')
    check(at(steps,'MapRouteStep_t','heading')%2==1,'nearby south pursuer is escaped without initial body pivot')
    call('ThiefIntercept_BlockLeg',11,10,clock)
    check(call('ThiefMapPlanner_PlanEscape',11,3,4,0,clock,plan),'east route remains when west unsafe')
    print('west blocked path',[at(steps+i*16,'MapRouteStep_t','to_node') for i in range(at(route,'MapRoute_t','count'))],flush=True)
    check(at(steps,'MapRouteStep_t','to_node')==12 and at(steps+16,'MapRouteStep_t','to_node')==13,'complete actual N012-N013 route is evaluated')
    return member('thief_plan','goal_node')

def ambiguous_n006(dt=20,mode=1,sign=1):
    global odom,clock,gray_raw,gray_state,rear_valid
    fresh(4,(1+2*mode)%4);rear_valid=1;member('thief_localization','yaw_cw_sign',sign)
    setstep(4,6,1,mode,300);record(steps,'MapRouteStep_t',dict(segment_count=2));call('Route_Start');tick()
    var('retreat_active',1);member('thief_localization','reversal_entry_mm',196)
    odom=728;clock+=600;gray_raw=0x07f;gray_state=5
    if dt<20:var('last_control_tick',clock-dt)
    call('Route_HandleJunction',clock,dt,5,gray_raw)
    check(member('thief_localization','confirmed_node')==4,'late similar gray cannot immediately certifyN006')
    check(var('route_active'),'ambiguity must continue observing rather than distance-stop')
    check('route_identity_phase' in symbols and var('route_identity_phase')==1,'candidate observer active')
    return 1

def n007_observed_reanchor():
    global odom,clock,gray_raw,gray_state,rear_valid
    for mode in [0,1]:
      for sign in [-1,1]:
        ambiguous_n006(mode=mode,sign=sign);gray_raw=0;gray_state=2;rear_valid=1;odom=40
        for _ in range(2):
            clock+=20;call('Route_ObserveIdentity',clock,20)
            check(member('thief_localization','confirmed_node')==4,'one/two white frames cannot relocate')
        clock+=20;call('Route_ObserveIdentity',clock,20)
        check(var('route_identity_node')==7,'observed no-continuation distinguishes actual N007 corner')
        check(member('thief_localization','confirmed_node')==4,'candidate alone is not centered confirmed pose')
        ret('Route_AllWheelsStopped',0)
        for _ in range(10):clock+=20;call('Route_ObserveIdentity',clock,20)
        check(member('thief_localization','confirmed_node')==4,'moving wheels cannot authorize relocalization')
        ret('Route_AllWheelsStopped',1)
        for _ in range(3):clock+=20;call('Route_ObserveIdentity',clock,20)
        check(member('thief_localization','confirmed_node')==7 and not var('route_identity_phase'),'real corner and centered standstill reanchor N007')
        check(call('RouteHistory_Count')==0,'skipped/unobserved edge cannot become reverse history')
    return 4

def exact_log_reversal():
    global odom,clock,gray_raw,gray_state,rear_valid
    fresh(6,3);rear_valid=1;member('thief_localization','yaw_cw_sign',-1)
    setstep(6,4,3,0,300);record(steps,'MapRouteStep_t',dict(segment_count=2));call('Route_Start');tick()
    odom=104;call('Route_BrakeControl');var('route_state',12)
    var('route_state_tick',clock-100);var('turn_stop_stable_count',2);var('replan_after_leg',1)
    clock+=20;call('Thief_InterceptStopped',clock)
    check(member('thief_localization','reversal_entry_mm')==196,'actual104mm reversal produces196mm signed entry')
    fixes=member('thief_localization','confirmations')
    odom=532;clock+=600;gray_raw=0x7f;gray_state=5
    call('Route_HandleJunction',clock,20,5,gray_raw)
    check('route_identity_phase' in symbols and var('route_identity_phase')==1 and member('thief_localization','confirmations')==fixes,'actual728mm observation does not falsely reconfirm N006')
    return 1

def candidates_follow_shape():
    check('ThiefLocalization_CandidatesBegin' in symbols,'candidate API exists')
    fresh(4,3);setstep(4,6,1,1,300);record(steps,'MapRouteStep_t',dict(segment_count=2));call('Route_Start');tick()
    c=allocate('ThiefLocCandidates',0x2000ca00)
    check(call('ThiefLocalization_CandidatesBegin',c,steps,0x07f)==2,'same raw admits N006 and N007')
    for _ in range(2):check(call('ThiefLocalization_CandidatesObserve',c,0x60,0x60,40)==0,'two continuing-line frames insufficient')
    check(call('ThiefLocalization_CandidatesObserve',c,0x60,0x60,40)==6,'real continuation chooses N006 even with suspect odometry')
    # Sequence of two actual junction observations shifts a corridor hypothesis.
    setstep(7,6,3,0,600)
    check(call('ThiefLocalization_CandidatesBegin',c,steps,0xff0)>=2,'reverse corridor has multiple aliases')
    for _ in range(3):call('ThiefLocalization_CandidatesObserve',c,0x60,0x60,40)
    for _ in range(3):call('ThiefLocalization_CandidatesObserve',c,0xff0,0x60,300)
    found=0
    for _ in range(3):found=call('ThiefLocalization_CandidatesObserve',c,0,0x60,40) or found
    check(found==1,'observed next branch then absent continuation resolves actual N001')
    return 2

def ambiguity_laser_priority():
    global clock
    ambiguous_n006();call('Thief_UpdatePose');call('Thief_UpdateLaserMask',clock)
    clock+=1;call('Thief_OnLaserFrame',3,150,100,1,clock)
    call('Thief_InterceptReact',clock)
    check(var('route_state')==12 and not var('route_identity_phase'),'new travel-front laser cancels observer and brakes immediately')
    check(member('thief_localization','confirmed_node')==4,'laser preemption does not invent candidate node')
    return 1

def ambiguity_loss_deadline():
    global clock,gray_raw,gray_state,rear_valid,odom
    ambiguous_n006();gray_raw=0;gray_state=2;rear_valid=2;odom=40
    clock+=20;call('Route_ObserveIdentity',clock,20)
    clock+=2000;call('Route_ObserveIdentity',clock,20)
    check(not var('route_active') and reason()=='GRAY_BOTH_NO_LINE','uncertainty cannot bypass continuous2second gray-loss rule')
    return 1

def observer_control_cadence():
    global clock,gray_raw,gray_state
    paid=[]
    hook('ChassisSpeedControl_Update',lambda:(paid.append(u.reg_read(UC_ARM_REG_R0))
        if 'route_identity_phase' in symbols and var('route_identity_phase') else None) or 0)
    ambiguous_n006(5)
    check(not paid,'5ms branch watcher must not integrate an extra speed-control update')
    # LINE_FIRST requires an actual leading line to pay forward wheel PI.
    gray_raw=0x60;gray_state=1
    clock+=15;call('App_ThiefStrategy_Task')
    check(paid==[20],'observer runs real wheel PI once on20ms schedule')
    ret('ChassisSpeedControl_Update')
    return 1

failures=[];cases={}
for fn in [n011_departure,ambiguous_n006,n007_observed_reanchor,exact_log_reversal,candidates_follow_shape,ambiguity_laser_priority,ambiguity_loss_deadline,observer_control_cadence]:
    try:cases[fn.__name__]=fn();print(fn.__name__+': PASS',flush=True)
    except AssertionError as exc:failures.append(dict(case=fn.__name__,error=str(exc)));print(fn.__name__+': FAIL '+str(exc),flush=True)
result=dict(checks=checks,failures=failures,cases=cases,hardware_tested=False,
 axf_sha256=hashlib.sha256(axf.read_bytes()).hexdigest(),test_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
out=ROOT/'Tests'/('before_route_identity/regression_before.json' if '--before' in sys.argv else 'results_route_identity.json')
out.write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8');print(json.dumps(result,indent=2));sys.exit(bool(failures))
