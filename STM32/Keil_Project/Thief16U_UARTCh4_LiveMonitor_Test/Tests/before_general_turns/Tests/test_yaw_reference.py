"""013314 incomplete turn and straight-road yaw reference; synthetic hardware."""
from pathlib import Path
import sys,json,hashlib
src=Path(__file__).with_name('test_documents15.py').read_text(encoding='utf-8').split('parking_pwm=[]')[0]
src=src.replace('Tests/before_documents15/original.axf','Tests/before_yaw_reference/original.axf')
exec(compile(src,str(Path(__file__).with_name('test_documents15.py')),'exec'))
def signed16(v):return v-65536 if v>=32768 else v

def straight_reference():
    global clock,yaw,odom
    check('Route_UpdateYawReference' in symbols,'straight reference learning exists')
    fresh(37,0);setstep(37,20,0,0,600);call('Route_Start');tick()
    sample=allocate('RouteGraySample_t',0x2000cd00)
    record(sample,'RouteGraySample_t',dict(front_raw12=0x60,rear_raw12=0x60))
    var('front_state',1);var('rear_state',1);var('route_state',2)
    yaw=589;odom=150
    for i in range(20):
        clock+=100;odom=150+i*8;call('Route_UpdateYawReference',sample,0)
    learned=signed16(member('thief_localization','origin_yaw_x100'))
    check(0<learned<589,'gradual reference correction, never snap current yaw to zero')
    check(yaw==589 and member('thief_localization','confirmed_node')==37,'raw yaw and node untouched')
    for case in ['text','side','skew','saturation','identity','stale']:
        old=member('thief_localization','origin_yaw_x100')
        if case=='text':record(steps,'MapRouteStep_t',dict(from_node=4,to_node=6))
        if case=='side':record(sample,'RouteGraySample_t',dict(front_raw12=0xc60))
        if case=='skew':record(sample,'RouteGraySample_t',dict(rear_error_x100=150))
        if case=='identity':var('route_identity_phase',1)
        if case=='stale':hook('JY61P_GetLastUpdateMs',lambda:clock-500)
        for _ in range(12):
            clock+=100;call('Route_UpdateYawReference',sample,800 if case=='saturation' else 0)
        check(member('thief_localization','origin_yaw_x100')==old,'no learning during '+case)
        record(steps,'MapRouteStep_t',dict(from_node=37,to_node=20))
        record(sample,'RouteGraySample_t',dict(front_raw12=0x60,rear_error_x100=0))
        var('route_identity_phase',0);hook('JY61P_GetLastUpdateMs',lambda:clock)
    return learned

def treasure_turn_handoff():
    global clock,yaw,gray_raw,gray_state,rear_valid
    for node,source,incoming,outgoing,target in [(6,15,0,3,4),(4,14,0,3,3)]:
      for mode in [0,1]:
       for sign in [-1,1]:
        body=(incoming+2*mode)%4
        fresh(source,body);member('thief_localization','yaw_cw_sign',sign)
        setstep(source,node,incoming,mode,300,action=-1)
        setstep(node,target,outgoing,mode,300,index=1);record(route,'MapRoute_t',dict(count=2))
        if node==6:record(steps+16,'MapRouteStep_t',dict(segment_count=2))
        call('Route_Start');odom_save=300
        check(var('route_active'),'valid physical map route starts: '+reason())
        # Real arrival anchors the turn; no fabricated outgoing leg.
        globals()['odom']=odom_save;clock+=500
        call('Route_HandleJunction',clock,20,4,0xc60)
        # An incoming5.89degree skew must not become the new turn target.
        var('turn_start_angle_x100',((589+(18000 if mode else 0)+18000)%36000)-18000)
        yaw=(589-sign*4657+18000)%36000-18000
        gray_raw=0x60;gray_state=1;rear_valid=1;targets.clear();events.clear()
        if 'Route_HasReachedTurnAngle' in symbols:
            check(call('Route_HasReachedTurnAngle',call('TravelFrame_Yaw',yaw))==0,'46.57degree rotation still has a large map-axis error')
        call('Route_EnterAfterTurnForward',clock,20,0x2000ce00,yaw)
        check('treasure_turn_align_active' in symbols and var('treasure_turn_align_active'),'N004/N006 46.57deg cannot hand off to PID')
        check('Route_DriveStraight' not in events,'no translational burst before heading alignment')
        ret('Route_AllWheelsStopped',0)
        yaw=(-sign*9000+18000)%36000-18000
        for _ in range(6):clock+=20;call('Route_AlignTreasureTurn',clock,20)
        check(var('treasure_turn_align_active'),'aligned angle without measured stop cannot hand off')
        ret('Route_AllWheelsStopped',1)
        for _ in range(20):clock+=20;call('Route_AlignTreasureTurn',clock,20)
        check(not var('treasure_turn_align_active'),f'fresh stable stopped axis releases gray acquisition node={node} mode={mode} sign={sign} phase={member("thief_localization","phase")} arrived={member("map_nav","arrived")} error={member("thief_localization","error")} stable={member("treasure_turn_heading","stable_count")}')
        check(member('thief_localization','phase')==2,'alignment alone never confirms outgoing leg')
    return 8

def reference_wrap_reverse():
    global clock,yaw,odom
    check('Route_UpdateYawReference' in symbols,'reference learner exists')
    for mode in [0,1]:
      for sign in [-1,1]:
        fresh(37,2*mode);member('thief_localization','yaw_cw_sign',sign)
        setstep(37,20,0,mode,600);call('Route_Start');tick()
        sample=allocate('RouteGraySample_t',0x2000cd00)
        record(sample,'RouteGraySample_t',dict(front_raw12=0x60,rear_raw12=0x60))
        var('front_state',1);var('rear_state',1);var('route_state',2)
        member('thief_localization','origin_yaw_x100',17950);yaw=-17800
        for i in range(30):
            clock+=100;odom=150+i*8;call('Route_UpdateYawReference',sample,0)
        learned=signed16(member('thief_localization','origin_yaw_x100'))
        error=(yaw-learned+18000)%36000-18000
        check(0<error<250,'wrapped yaw and reverse frame learn the small physical drift')
        check(yaw==-17800 and member('thief_localization','body_heading')==2*mode,'no yaw rewrite or frame flip')
    return 4

failures=[];cases={}
for fn in [straight_reference,treasure_turn_handoff,reference_wrap_reverse]:
    try:cases[fn.__name__]=fn()
    except Exception as e:failures.append(dict(case=fn.__name__,error=str(e)))
firmware=ROOT/'MDK-ARM'/ROOT.name/(ROOT.name+'.axf')
if '--before' in sys.argv:firmware=ROOT/'Tests/before_yaw_reference/original.axf'
report=dict(cases=cases,failures=failures,axf_sha256=hashlib.sha256(firmware.read_bytes()).hexdigest(),
            test_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),hardware_tested=False)
dest=ROOT/('Tests/before_yaw_reference/regression_before.json' if '--before' in sys.argv else 'Tests/results_yaw_reference.json')
dest.write_text(json.dumps(report,indent=2),encoding='utf-8');print(json.dumps(report,indent=2))
sys.exit(bool(failures))
