"""Real linked ARM recovery; synthetic sensor reads and measured wheel response.

The sensor sequence reproduces the logged front=000/rear=010 or030 case.
--before uses the immutable ROUTE4 image. No physical motors or serial port.
"""
from pathlib import Path
import sys, json, hashlib

ROOT = Path(__file__).absolute().parents[1]
setup = (ROOT/'Tests/test_n068_recovery.py').read_text(encoding='utf-8').split('failures=[]')[0]
setup = setup.replace('ROUTE3_before_recovery_fix', 'ROUTE4_before_rear_retrace')
exec(compile(setup, str(ROOT/'Tests/test_n068_recovery.py'), 'exec'))

failures = []
cases = []

def prepare39(mode=0, rear=0x010, pending=False):
    global front_raw, back_raw, odom, physical_rpm, refreshes
    front_raw = back_raw = 0x60
    fresh(38, (1+2*mode)%4)
    setstep(38,39,1,mode,300,0)
    setstep(39,40,1,mode,300,2,index=1)
    record(route,'MapRoute_t',dict(count=2))
    member('thief_plan','goal_node',40)
    call('Route_Start')
    for _ in range(8): tick()
    odom=300
    front_raw=0x8F7
    call('Route_HandleJunction',clock,20,5,front_raw)
    check(var('route_state')==11, 'real N039 straight junction starts pending pass')
    if not pending:
        front_raw=0x60
        for _ in range(3): tick()
        check(member('map_nav','step')==1, 'three real line samples start N039 to N040')
        for _ in range(4):tick()
        check(var('junction_armed'), 'ordinary on-leg line is armed before loss')
    front_raw=0;back_raw=rear;odom=20
    cache(900);physical_rpm=900;refreshes=0;commands.clear()
    tick()
    check(var('route_state')==8, 'logged leading-white enters real recovery')

def rear_only():
    global front_raw, back_raw
    for mode in [0,1]:
        for rear in [0x010,0x030,0x080,0x0C0]:
            prepare39(mode,rear)
            saved_count=tm('count')
            for _ in range(12):
                previous=refreshes;tick()
                check(refreshes-previous<=1, 'sample each wheel encoder once per task')
                check(member('map_nav','step')==1 and tm('count')==saved_count,
                      'rear-only recovery never advances a route or records a fabricated junction')
                check(call('TravelFrame_IsReverse')==mode, 'retrace does not exchange physical sensor frames')
            drives=[x for x in commands if x[0]=='drive']
            check(drives and all(l+r<0 for _,l,r in drives),
                  'rear-only stripe must backtrack along visible line instead of continuing forward')
            check(all(l<0 and r<0 for _,l,r in drives), 'all wheels reverse; no stationary diagonal lock')
            check(all(l-r>0 if rear in [0x010,0x030] else l-r<0 for _,l,r in drives),
                  'reverse steering uses the trailing board as the leading end')
            check(commands[0][0]=='brake' and refreshes>0, 'measured stop precedes reverse targets')
            commands.clear();front_raw=0x60
            tick()
            check(var('route_state')==8 and commands and commands[-1][0]=='brake',
                  'first regained front line brakes backward motion; cannot jump to forward PID')
            # One transient stripe cannot finish the backtrack or commit a node.
            front_raw=0
            for _ in range(4): tick()
            check(not any(x[0]=='drive' and x[1]+x[2]>0 for x in commands),
                  'front chatter cannot reverse wheel direction before standstill')
            front_raw=back_raw=0x60
            for _ in range(12): tick()
            check(var('route_state')==2 and var('route_active'), 'stable real line resumes original follow')
            check(member('map_nav','step')==1 and tm('count')==saved_count,
                  'resuming follow cannot count recovery as arrival at N040')
            check(any(x[0]=='drive' and x[1]+x[2]>0 for x in commands), 'forward movement resumes after measured stop')
            cases.append(dict(case='rear_only',mode=mode,rear=rear))

def freeze_fault_pose():
    global odom
    prepare39()
    odom=123;call('Thief_UpdatePose')
    pose=symbols['thief_pose'];length=size(variables['thief_pose'])
    keys=['node_id','x_mm','y_mm','body_heading','travel_heading','drive_mode']
    saved=[at(pose,'ThiefMapPose_t',k) for k in keys]
    msg=0x2000C800;u.mem_write(msg,b'KEY_STOP\0');call('Route_StopFault',msg)
    for _ in range(5):call('Thief_UpdatePose')
    check(at(pose,'ThiefMapPose_t','node_id')==39, 'fault display retains current estimated leg instead of old N038 refuge')
    check([at(pose,'ThiefMapPose_t',k) for k in keys]==saved, 'fault does not replace last estimated coordinates or travel frame')
    check(at(pose,'ThiefMapPose_t','valid')==0, 'frozen coordinates remain explicitly uncertain')
    cases.append(dict(case='freeze_fault_pose'))

def both_white():
    global front_raw,back_raw,clock
    prepare39();front_raw=back_raw=0;commands.clear()
    for _ in range(120):
        clock+=20;call('App_ThiefStrategy_Task')
        if not var('route_active'):break
    check(reason()=='GRAY_BOTH_NO_LINE', 'continuous two-board white still stops after two seconds')
    check(not any(x[0]=='drive' and x[1]+x[2]!=0 for x in commands),
          'both white never authorizes blind forward or backward translation')
    cases.append(dict(case='both_white'))

def pending_pass():
    global front_raw,back_raw
    for mode in [0,1]:
        prepare39(mode,pending=True)
        for _ in range(12):tick()
        check(member('map_nav','step')==0 and tm('count')==0,
              'rear-only recovery cannot consume pending N039 pass')
        front_raw=back_raw=0x60
        for _ in range(16):tick()
        check(member('map_nav','step')==1 and tm('count')==1 and var('route_state')==2,
              'pending straight owner advances N039 exactly once after recovery')
        cases.append(dict(case='pending_pass',mode=mode))

def measured_brake():
    global physical_rpm,front_raw,back_raw
    prepare39()
    def delayed():
        commands.append(('brake',0,0))
        return 0
    hook('ChassisSpeedControl_Brake',delayed);hook('Chassis_Brake',delayed)
    physical_rpm=900;cache(900);commands.clear()
    try:
        for _ in range(8):tick()
        check(all(x[0]=='brake' for x in commands), 'moving measured wheels cannot reverse merely because brake was requested')
        physical_rpm=0
        for _ in range(8):tick()
        check(any(x[0]=='drive' and x[1]+x[2]<0 for x in commands), 'actual stopped feedback releases backward recovery')
        front_raw=back_raw=0x60;physical_rpm=500;cache(500);commands.clear()
        for _ in range(8):tick()
        check(all(x[0]=='brake' for x in commands), 'front line cannot bypass still-moving backward wheels')
        physical_rpm=0
        for _ in range(12):tick()
        check(var('route_state')==2, 'actual second standstill releases normal line following')
    finally:
        hook('ChassisSpeedControl_Brake',brake);hook('Chassis_Brake',brake)
    cases.append(dict(case='measured_brake'))

def edge_not_node():
    global front_raw,back_raw
    for mode in [0,1]:
        for raw in [0x007,0xE00,0xFFF]:
            prepare39(mode)
            count=tm('count')
            for _ in range(12):tick()
            front_raw=raw;tick()
            check(member('map_nav','step')==1 and not member('map_nav','arrived') and tm('count')==count,
                  'edge stripe encountered while backing is not arrival at N040')
            check(var('route_state')==8, 'reacquired edge remains owned by recovery, not hide arrival')
            front_raw=back_raw=0x60
            for _ in range(14):tick()
            check(var('route_state')==2 and member('map_nav','step')==1,
                  'edge-to-centered transition resumes same road without a false junction')
            cases.append(dict(case='edge_not_node',mode=mode,raw=raw))

def geometry():
    import math
    addr=0x2000CA00;inp=addr+64;out=addr+128
    for span in [160,220,280]:
        for sign in [-1,1]:
            # Hypothetical board spacing/width, NOT measured vehicle geometry.
            theta=sign*math.atan2(80,span);entry=theta;x=sign*30.0
            call('RestartLineSearch_Reset',addr,-sign*400)
            backwards=False;ready=False
            for frame in range(250):
                lead=-(x+span/2*math.sin(theta))/math.cos(theta)
                rear=-(x-span/2*math.sin(theta))/math.cos(theta)
                lv=abs(lead)<=44;rv=abs(rear)<=44
                record(inp,'RestartLineInput',dict(leading_valid=lv,trailing_valid=rv,
                    leading_error=round(lead/8*100),trailing_error=round(rear/8*100),
                    yaw_known=1,yaw_cw_x100=round((theta-entry)*18000/math.pi),stopped=1))
                call('RestartLineSearch_Update',addr,inp,out)
                l=at(out,'RestartLineOutput','left_x10');rr=at(out,'RestartLineOutput','right_x10')
                l=l-65536 if l>=32768 else l;rr=rr-65536 if rr>=32768 else rr
                backwards |= l+rr<0
                check(lv or rv or l+rr==0, 'geometry plant cannot translate without a visible stripe')
                v=(l+rr)*210/1200;omega=(l-rr)*210/600/160
                x+=v*math.sin(theta)*0.02;theta+=omega*0.02
                if at(out,'RestartLineOutput','ready'):
                    ready=True;break
            check(backwards and ready, 'rear-leading kinematic model must reacquire front line')
            cases.append(dict(case='geometry',board_span_mm=span,side=sign,frames=frame+1))

def unfinished_turn():
    global front_raw,back_raw,odom,yaw
    for mode in [0,1]:
        front_raw=back_raw=0x60;fresh(39,(1+2*mode)%4)
        setstep(39,40,1,mode,300,1)
        setstep(40,46,2,mode,300,2,index=1)
        record(route,'MapRoute_t',dict(count=2));member('thief_plan','goal_node',46)
        call('Route_Start')
        for _ in range(8):tick()
        odom=300;front_raw=0x007
        call('Route_HandleJunction',clock,20,5,front_raw)
        # Preserve the baseline after-turn acquisition interval, then test
        # recovery ownership; do not force the application into that state.
        for _ in range(180):
            if var('route_state') in [5,6]:
                yaw=(yaw+(600 if var('route_state')==5 else -600)+18000)%36000-18000
                front_raw=0
            tick()
            if var('route_state')==8:break
        check(var('route_state')==8, f'real ordinary turn with missing exit enters recovery: mode={mode} state={var("route_state")} yaw={yaw} fault={reason()}')
        commands.clear()
        for _ in range(12):tick()
        check(all(l+rr==0 for tag,l,rr in commands if tag=='drive'),
              'trailing old line cannot drive forward or reverse an unfinished ordinary turn')
        check(member('map_nav','step')==0, 'unfinished turn does not commit its node')
        cases.append(dict(case='unfinished_turn',mode=mode))

for fn in [rear_only,freeze_fault_pose,both_white,pending_pass,measured_brake,edge_not_node,geometry,unfinished_turn]:
    try:fn();print(fn.__name__+': PASS',flush=True)
    except AssertionError as e:
        failures.append(dict(case=fn.__name__,error=str(e)))
        print(fn.__name__+': FAIL '+str(e),flush=True)

report=dict(checks=checks,failures=failures,cases=cases,
            axf_sha256=hashlib.sha256(axf.read_bytes()).hexdigest(),hardware_tested=False)
out=ROOT/'Tests'/('rear_retrace_before.json' if '--before' in sys.argv else 'results_rear_retrace.json')
out.write_text(json.dumps(report,indent=2),encoding='utf-8')
print(json.dumps(report,indent=2))
sys.exit(bool(failures))
