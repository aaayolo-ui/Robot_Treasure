"""Execute actual linked application routines. Hardware calls are explicit stubs.
This proves software state transitions only, never physical motor response.
"""
from pathlib import Path
import sys,json,hashlib
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'Tests/build/deps'))
from elftools.elf.elffile import ELFFile
from elftools.dwarf.dwarf_expr import DWARFExprParser
from unicorn import Uc,UC_ARCH_ARM,UC_MODE_THUMB,UC_MODE_MCLASS,UC_HOOK_CODE
from unicorn.arm_const import *
axf=ROOT/'MDK-ARM'/ROOT.name/(ROOT.name+'.axf')
f=axf.open('rb');elf=ELFFile(f)
symbols={s.name:s['st_value'] for s in elf.get_section_by_name('.symtab').iter_symbols()}
dwarf=elf.get_dwarf_info();expr=DWARFExprParser(dwarf.structs)
types={};variables={}
def name(d):
    a=d.attributes.get('DW_AT_name');return a.value.decode() if a else ''
for cu in dwarf.iter_CUs():
    for d in cu.iter_DIEs():
        if d.tag=='DW_TAG_variable' and name(d) in symbols and 'DW_AT_type' in d.attributes:
            variables[name(d)]=d.get_DIE_from_attribute('DW_AT_type')
        if d.tag=='DW_TAG_typedef':types[name(d)]=d.get_DIE_from_attribute('DW_AT_type')
def base(d):
    while d.tag in ('DW_TAG_typedef','DW_TAG_const_type','DW_TAG_volatile_type'):
        d=d.get_DIE_from_attribute('DW_AT_type')
    return d
def size(d):return base(d).attributes['DW_AT_byte_size'].value
def fields(d):
    out={}
    for m in base(d).iter_children():
        if m.tag!='DW_TAG_member':continue
        v=m.attributes['DW_AT_data_member_location'].value
        if not isinstance(v,int):v=expr.parse_expr(v)[0].args[0]
        out[name(m)]=(v,m.get_DIE_from_attribute('DW_AT_type'))
    return out
u=Uc(UC_ARCH_ARM,UC_MODE_THUMB|UC_MODE_MCLASS);u.ctl_set_cpu_model(UC_CPU_ARM_CORTEX_M3)
for addr,length in [(0x08000000,0x100000),(0x20000000,0x10000),(0x40000000,0x30000),(0xe000e000,0x2000)]:u.mem_map(addr,length)
for s in elf.iter_sections():
    if s['sh_flags']&2 and s['sh_type']!='SHT_NOBITS' and s['sh_size']:u.mem_write(s['sh_addr'],s.data())
def put(addr,n,value):u.mem_write(addr,int(value%(1<<(n*8))).to_bytes(n,'little'))
def get(addr,n=4):return int.from_bytes(u.mem_read(addr,n),'little')
def var(n,value=None):
    length=size(variables[n]);addr=symbols[n]
    if value is not None:put(addr,length,value)
    return get(addr,length)
def member(n,field,value=None):
    off,t=fields(variables[n])[field];addr=symbols[n]+off
    if value is not None:put(addr,size(t),value)
    return get(addr,size(t))
def record(addr,typ,values):
    for k,v in values.items():
        off,t=fields(types[typ])[k];put(addr+off,size(t),v)
def allocate(typ,addr):u.mem_write(addr,bytes(size(types[typ])));return addr
clock=1000;events=[];hooks={}
def hook(n,fn):
    if n not in symbols:return
    if n in hooks:u.hook_del(hooks[n])
    def dispatch(uc,address,length,data):
        result=fn()
        if result is not None:uc.reg_write(UC_ARM_REG_R0,result)
        uc.reg_write(UC_ARM_REG_PC,uc.reg_read(UC_ARM_REG_LR))
    hooks[n]=u.hook_add(UC_HOOK_CODE,dispatch,begin=symbols[n]&~1,end=symbols[n]&~1)
def ret(n,value=0):hook(n,lambda:value)
def event(n,value=0):
    def fn():events.append(n);return value
    hook(n,fn)
for n in ['UartDebug_Printf','UartDebug_SendString','UartDebug_Task','MapTimingLog_Task',
          'TreasureWheelLog_Begin','TreasureWheelLog_Event','TreasureWheelLog_GrayContext',
          'Chassis_UpdateFeedback','ChassisSpeedControl_Update','ChassisSpeedControl_ResetPiState']:
    ret(n)
for n in ['Route_BrakeControl','Route_ResetControl','Route_DriveStraight','Route_DriveLeftTurn',
          'Route_DriveRightTurn','Chassis_Brake','ChassisSpeedControl_Brake',
          'ChassisSpeedControl_Disable','ChassisSpeedControl_Enable','TravelFrame_SetWheelTargets']:
    event(n)
ret('Route_AllWheelsStopped',1);ret('Route_InitializeGray',1);ret('LaserRx_SetEnabled',1)
ret('ChassisSpeedControl_HasStall',0)
hook('SystemTime_GetMs',lambda:clock);hook('JY61P_GetLastUpdateMs',lambda:clock)
yaw=0
def getyaw():put(u.reg_read(UC_ARM_REG_R0),2,yaw);return 1
hook('JY61P_GetYawX100',getyaw)
hook('JY61P_GetFrameCount',lambda:clock)
gray_raw=0x60;gray_state=1
def readgray():
    # A normal valid centered line in both logical direction mappings.
    a=u.reg_read(UC_ARM_REG_R0)
    record(a,'RouteGraySample_t',{'front_raw12':gray_raw,'rear_raw12':0x60,
      'front_error_x100':0,'rear_error_x100':0,'position_error_x100':0,'heading_error_x100':0})
    return gray_state
hook('Route_ReadFront',readgray)
ret('Route_UpdateFollowMode')
stop=0x080ff000
u.hook_add(UC_HOOK_CODE,lambda uc,a,s,d:uc.emu_stop(),begin=stop,end=stop)
def call(n,*args):
    for r,v in zip([UC_ARM_REG_R0,UC_ARM_REG_R1,UC_ARM_REG_R2,UC_ARM_REG_R3],args):u.reg_write(r,v)
    u.reg_write(UC_ARM_REG_SP,0x2000bfc0);u.reg_write(UC_ARM_REG_LR,stop|1)
    for i,v in enumerate(args[4:]):put(0x2000bfc0+i*4,4,v)
    u.emu_start(symbols[n]|1,0,count=50_000_000)
    assert u.reg_read(UC_ARM_REG_PC)==stop,('did not return',n,hex(u.reg_read(UC_ARM_REG_PC)))
    return u.reg_read(UC_ARM_REG_R0)
checks=0
def check(x,message):
    global checks
    checks+=1
    assert x,message
# Scratch data lies above declared 48 KiB RAM, separate from real globals/stack.
s=allocate('MapRouteStep_t',0x2000c100);y=allocate('ThiefLocYaw_t',0x2000c200)
record(s,'MapRouteStep_t',dict(from_node=11,to_node=12,length_mm=300,segment_count=1,
 heading=1,expected_ports=3,target_kind=2,action=2,drive_mode=0))
record(y,'ThiefLocYaw_t',dict(yaw_x100=0,stamp_ms=clock,valid=1))
loc=symbols['thief_localization'];plan=symbols['thief_plan'];nav=symbols['map_nav']
po=fields(variables['thief_plan']);steps=plan+po['steps'][0];route=plan+po['route'][0]
u.mem_write(steps,bytes(u.mem_read(s,16)))
record(route,'MapRoute_t',dict(id=1601,count=1,start_node=11,heading=1,drive_mode=0))
member('thief_plan','valid',1);member('thief_plan','goal_node',12)
check(call('ThiefLocalization_Start',loc,11,1,y,clock)==1,'start localization')
check(call('ThiefLocalization_BeginStep',loc,s,y,clock)==1,'begin leg')
call('MapRouteNavigator_StartRuntime',nav,route,steps)
odom=180
hook('MapRoutePosition_DistanceMm',lambda:odom)
def resetcounts():
    global odom
    odom=0
    return 1000
hook('MapRoutePosition_ForwardCounts',resetcounts)
var('route_active',1);var('route_state',2);var('thief_strategy_state',3)
var('thief_treasure_passed',1);var('rear_hw_ready',1)
call('Thief_UpdatePose');call('Thief_UpdateLaserMask',clock)
check(member('thief_pose','x_mm')==780,'moving XY before interception')
clock+=1 # Frames from the enable millisecond are deliberately rejected.
for _ in range(2):call('Thief_OnLaserFrame',0,400,30,1,clock)
if not var('intercept_pending'):
    print('LASER_DIAG',var('thief_laser_active_mask'),
          bytes(u.mem_read(symbols['thief_pose'],size(variables['thief_pose']))).hex(),
          bytes(u.mem_read(symbols['thief_laser'],40)).hex(),flush=True)
check(var('intercept_pending')==1,'valid forward frames request interception')
events.clear();call('Thief_InterceptReact',clock)
check(events==['Route_BrakeControl'],'brake immediately before any motion or planner')
check(var('route_state')==12 and var('replan_after_leg')==1,'interception enters braking')
check(call('TravelFrame_IsReverse')==0,'no premature polarity switch')
clock+=20;call('Thief_InterceptStopped',clock)
check(call('TravelFrame_IsReverse')==0,'standstill needs two samples and minimum time')
# First post-swap sample can fail without cancelling the whole escape.
saved_ram=bytes(u.mem_read(0x20000000,0xc000));saved_clock=clock
for n in ['Key_GetEvent','Key_IsPressed','JY61P_Task','TreasureMotorSnapshot_Probe',
          'TreasureWheelLog_Task','LaserRx_Poll','LaserRx_BootLogTask',
          'StartupStatus_Task','WheelMapLog_Task','Thief_LaserLog','Thief_LocalizationLog']:
    ret(n)
clock+=60;gray_raw=0;gray_state=2
call('Thief_InterceptStopped',clock)
check(var('route_state')==8 and var('route_active')==1,'post-swap missing gray enters recovery')
check(call('TravelFrame_IsReverse')==1,'post-swap recovery retains correct reverse frame')
gray_raw=0x60;gray_state=1
clock+=20;call('App_ThiefStrategy_Task')
clock+=20;call('App_ThiefStrategy_Task')
check(var('route_active')==1 and var('thief_strategy_state')!=4,'post-swap recovery resumes retreat')
u.mem_write(0x20000000,saved_ram);clock=saved_clock;odom=180
clock+=60;call('Thief_InterceptStopped',clock)
check(call('TravelFrame_IsReverse')==1,'switch after confirmed stop')
check(var('map_progress_offset_mm')==120,'partial-leg offset retained')
check(member('thief_localization','confirmed_node')==11,'no phantom node fix at turnback')
check(var('retreat_active') and var('route_active'),'retreat starts driving')
check(var('route_state')==1,'retreat leaves using normal gray controller')
check(member('map_nav','fault')==0,'runtime reversed route valid')
call('Thief_UpdatePose')
check(member('thief_pose','x_mm')==780,'reversal keeps XY continuous')
check(member('thief_pose','body_heading')==1,'reversal does not rotate body')
odom=180;clock+=200
call('Thief_UpdatePose')
check(member('thief_pose','x_mm')==600,'retreat reaches original junction')
call('Route_HandleJunction',clock,20,4,0xc63)
check(var('route_active')==0 and var('thief_strategy_state')==5,'return node enters settling')
check(var('replan_after_leg')==1,'return node must replan, not become a hide')
check(var('thief_current_node')==11,'correct replan node')
for n in ['Key_GetEvent','Key_IsPressed','JY61P_Task','TreasureMotorSnapshot_Probe',
          'TreasureWheelLog_Task','LaserRx_Poll','LaserRx_BootLogTask',
          'StartupStatus_Task','WheelMapLog_Task','Thief_LaserLog','Thief_LocalizationLog']:
    ret(n)
var('route_key_release_pending',0)
clock+=40;call('App_ThiefStrategy_Task')
clock+=40;call('App_ThiefStrategy_Task')
check(var('thief_strategy_state')==2,'full task completes stopped-node settling')
check(var('replan_after_leg')==1,'forced replan survives sensor evidence reset')
clock+=1
def slow_planner(uc,address,length,data):
    global clock
    clock+=150 # Slow but bounded computation while already stopped.
delay_hook=u.hook_add(UC_HOOK_CODE,slow_planner,
    begin=symbols['ThiefMapPlanner_PlanEscape']&~1,end=symbols['ThiefMapPlanner_PlanEscape']&~1)
call('App_ThiefStrategy_Task');u.hook_del(delay_hook)
check(var('route_active')==1,'full task replans and restarts even without a new sighting')
check(var('thief_strategy_state')!=4,'150ms valid planning result does not permanently fault')
check(var('replan_after_leg')==0,'replan latch consumed exactly once')
check(member('thief_plan','goal_node') in [11,38,26,35,63],'full task chooses allowed hide')
# Exercise actual departure setup and actual follow/gray commit for a root turn.
clock+=100;yaw=0
record(y,'ThiefLocYaw_t',dict(yaw_x100=yaw,stamp_ms=clock,valid=1))
call('ThiefLocalization_Start',loc,11,1,y,clock)
record(steps,'MapRouteStep_t',dict(from_node=11,to_node=22,length_mm=300,
 segment_count=1,heading=2,expected_ports=3,target_kind=2,action=2,drive_mode=0))
record(route,'MapRoute_t',dict(id=1601,count=1,start_node=11,heading=2,drive_mode=0))
member('thief_plan','valid',1);var('thief_strategy_state',2)
call('TravelFrame_SetReverse',1) # Arrived backwards: approach must stay backwards.
events.clear();call('Route_Start')
check(var('departure_turn')==1,'root turn dispatched instead of failing BeginStep')
check(var('route_state')==3,'root turn approaches junction before braking')
check(call('TravelFrame_IsReverse')==1,'root approach preserves incoming reverse mode')
check(member('thief_localization','phase')==2,'root turn pose transitional')
check(var('route_active')==1,'root departure remains active')
# Enter brake at the approach deadline, accumulate branch evidence in the
# incoming frame, then switch travel frame only after measured standstill.
gray_raw=0xc60;gray_state=4
clock+=180;call('App_ThiefStrategy_Task')
check(var('route_state')==4,'approach deadline brakes before rotation')
clock+=20;call('App_ThiefStrategy_Task')
check(call('TravelFrame_IsReverse')==1,'incoming frame retained while braking')
clock+=60;call('App_ThiefStrategy_Task')
check(var('route_state')==5,'reverse approach left branch permits physical right turn')
check(call('TravelFrame_IsReverse')==0,'new forward frame selected after standstill')
gray_raw=0x60;gray_state=1
# RunFollow must commit the current first step, not skip to step 1.
yaw=9000;clock+=200;var('route_state',2);var('front_state',1)
sample=allocate('RouteGraySample_t',0x2000c300)
record(sample,'RouteGraySample_t',dict(front_raw12=0x60,rear_raw12=0x60))
ret('MapTimingLog_FollowSample',1)
call('Route_RunFollow',sample,20)
check(var('departure_turn')==0,'root turn consumed after confirmed yaw and line')
check(member('map_nav','step')==0,'root turn did not skip its first edge')
check(member('thief_localization','phase')==1,'root turn enters first leg')
check(member('thief_localization','body_heading')==2,'new body heading committed')
check(var('thief_strategy_state')!=4,'no false localization fault')
# A single bad gray read while passing a straight junction must recover.
clock+=100;yaw=0;gray_raw=0x60;gray_state=1
record(y,'ThiefLocYaw_t',dict(yaw_x100=yaw,stamp_ms=clock,valid=1))
call('ThiefLocalization_Start',loc,11,1,y,clock)
record(steps,'MapRouteStep_t',dict(from_node=11,to_node=12,length_mm=300,
 segment_count=1,heading=1,expected_ports=3,target_kind=2,action=0,drive_mode=0))
record(steps+16,'MapRouteStep_t',dict(from_node=12,to_node=13,length_mm=300,
 segment_count=1,heading=1,expected_ports=3,target_kind=2,action=2,drive_mode=0))
record(route,'MapRoute_t',dict(id=1601,count=2,start_node=11,heading=1,drive_mode=0))
member('thief_plan','valid',1);var('thief_strategy_state',2)
call('Route_Start');odom=300
call('Route_HandleJunction',clock,20,5,0x63)
check(var('route_state')==11,'straight junction enters pass state')
gray_raw=0;gray_state=2;clock+=20
call('App_ThiefStrategy_Task')
check(var('route_state')==8 and var('route_active')==1,'one missing gray read enters recovery')
gray_raw=0x60;gray_state=1
clock+=20;call('App_ThiefStrategy_Task')
clock+=20;call('App_ThiefStrategy_Task')
check(var('route_state')==11,'valid gray samples resume same pass state')
for _ in range(3):clock+=20;call('App_ThiefStrategy_Task')
check(member('map_nav','step')==1,'recovered pass advances only after line confirmation')
check(var('route_active')==1 and var('thief_strategy_state')!=4,'no nuisance pass fault')
# Retained manual stop must still override a running escape.
ret('Key_IsPressed',1);clock+=20;call('App_ThiefStrategy_Task')
check(var('route_active')==0 and var('thief_strategy_state')==4,'manual stop remains authoritative')
ret('Key_IsPressed',0);var('route_key_release_pending',0)
# Exhaust every real junction quarter-turn with each possible incoming
# forward/reverse mode. Walk over non-observable N005 instead of inventing a fix.
def field_at(addr,typ,k):
    off,t=fields(types[typ])[k];return get(addr+off,size(t))
def road(node,heading):
    if not call('ThiefMapPlanner_HasRoad',node,heading,0x2000c400,0x2000c402):return None
    return get(0x2000c400,2),get(0x2000c402,2)
hide_arrival_cases=0
for node,axes in [(11,[1,3]),(38,[1,3]),(26,[0,2]),(35,[1,3]),(63,[0,2])]:
    for body in axes:
        for mode in [0,1]:
            travel=(body+2*mode)%4
            start,length=road(node,(travel+2)%4)
            clock+=1000;yaw=0;odom=0;gray_raw=0x60;gray_state=1
            record(y,'ThiefLocYaw_t',dict(yaw_x100=0,stamp_ms=clock,valid=1))
            call('ThiefLocalization_Start',loc,start,body,y,clock)
            end=call('ThiefMapPlanner_Node',node)
            record(steps,'MapRouteStep_t',dict(from_node=start,to_node=node,length_mm=length,
              segment_count=1,heading=travel,expected_ports=field_at(end,'MapRouteNode_t','degree'),
              target_kind=field_at(end,'MapRouteNode_t','kind'),action=2,drive_mode=mode))
            record(route,'MapRoute_t',dict(id=1601,count=1,start_node=start,heading=travel,drive_mode=mode))
            member('thief_plan','valid',1);var('thief_strategy_state',2);var('node_centered',0)
            call('Route_Start');odom=length
            raw=0x60
            if road(node,(travel+3)%4):raw|=0xc00
            if road(node,(travel+1)%4):raw|=3
            call('Route_HandleJunction',clock,20,4,raw)
            check(var('route_state')==13 and var('route_active')==1,'hide arrival keeps moving to center')
            check(call('TravelFrame_IsReverse')==mode,'hide approach keeps forward/reverse arrival frame')
            call('Thief_UpdatePose')
            check(member('thief_pose','valid')==0,'no settled-pose sensor projection while centering')
            clock+=179;call('App_ThiefStrategy_Task')
            check(var('route_active')==1 and var('route_state')==13,'hide does not brake before 180ms')
            clock+=1;call('App_ThiefStrategy_Task')
            check(var('route_active')==0 and var('thief_strategy_state')==5,'hide brakes at 180ms')
            check(var('node_centered')==1 and var('thief_current_node')==node,'centered arrival recorded')
            clock+=20;call('App_ThiefStrategy_Task')
            clock+=60;call('App_ThiefStrategy_Task')
            check(var('thief_strategy_state')==2,'centered hide settles normally')
            # A later side exit must not repeat the same 180ms translation.
            out=(body+1)%4
            if not road(node,out):out=(body+3)%4
            target,distance=road(node,out);end=call('ThiefMapPlanner_Node',target)
            record(steps,'MapRouteStep_t',dict(from_node=node,to_node=target,length_mm=distance,
              segment_count=1,heading=out,expected_ports=field_at(end,'MapRouteNode_t','degree'),
              target_kind=field_at(end,'MapRouteNode_t','kind'),action=2,drive_mode=0))
            record(route,'MapRoute_t',dict(id=1601,count=1,start_node=node,heading=out,drive_mode=0))
            call('Route_Start')
            check(var('departure_centered')==1 and var('route_state')==4,'centered hide does not approach twice')
            clock+=20;call('App_ThiefStrategy_Task')
            clock+=60;call('App_ThiefStrategy_Task')
            check(var('route_state') in [5,6],'centered departure turns without requiring a second branch crossing')
            hide_arrival_cases+=1
root_turn_cases=0
for node in range(1,89):
    p=call('ThiefMapPlanner_Node',node)
    if not field_at(p,'MapRouteNode_t','kind'):continue
    for body in range(4):
        for incoming_mode in range(2):
            incoming=(body+2*incoming_mode)%4
            if not road(node,(incoming+2)%4):continue
            for delta in [1,3]:
                outgoing=(body+delta)%4
                edge=road(node,outgoing)
                if edge is None:continue
                target,length=edge;parts=1
                while not field_at(call('ThiefMapPlanner_Node',target),'MapRouteNode_t','kind'):
                    target,extra=road(target,outgoing);length+=extra;parts+=1
                end=call('ThiefMapPlanner_Node',target)
                clock+=1000;yaw=0;odom=0
                record(y,'ThiefLocYaw_t',dict(yaw_x100=0,stamp_ms=clock,valid=1))
                call('ThiefLocalization_Start',loc,node,body,y,clock)
                record(steps,'MapRouteStep_t',dict(from_node=node,to_node=target,length_mm=length,
                  segment_count=parts,heading=outgoing,expected_ports=field_at(end,'MapRouteNode_t','degree'),
                  target_kind=field_at(end,'MapRouteNode_t','kind'),action=2,drive_mode=0))
                record(route,'MapRoute_t',dict(id=1601,count=1,start_node=node,heading=outgoing,drive_mode=0))
                member('thief_plan','valid',1);var('thief_strategy_state',2);var('node_centered',0)
                call('TravelFrame_SetReverse',incoming_mode)
                relative=(outgoing-incoming)%4
                gray_raw,gray_state=(0x63,5) if relative==1 else (0xc60,4)
                call('Route_Start')
                check(var('route_state')==3,'all roots begin aligned approach')
                check(call('TravelFrame_IsReverse')==incoming_mode,'all roots preserve entry mode')
                clock+=180;call('App_ThiefStrategy_Task')
                clock+=20;call('App_ThiefStrategy_Task')
                clock+=60;call('App_ThiefStrategy_Task')
                check(var('route_state')==(5 if delta==1 else 6),
                      f'root turn {node=} {body=} {incoming_mode=} {delta=}')
                check(call('TravelFrame_IsReverse')==0,'all roots change frame only before spin')
                yaw=9000 if delta==1 else -9000;clock+=200
                gray_raw=0x60;gray_state=1;var('route_state',2);var('front_state',1)
                call('Route_RunFollow',sample,20)
                check(var('departure_turn')==0,'all roots commit outgoing axis')
                check(member('map_nav','step')==0,'all roots preserve first leg')
                check(member('thief_localization','body_heading')==outgoing,'all roots commit correct body')
                check(var('route_active')==1 and var('thief_strategy_state')!=4,'all roots remain active')
                root_turn_cases+=1
# Exercise recovery through the actual application, including the reported
# N038->N037 reverse route, both leading-board confirmations and laser priority.
trailing_state=1;trailing_error=-50
logical_targets=[]
def read_trailing():
    record(u.reg_read(UC_ARM_REG_R0),'RouteGraySample_t',
      dict(rear_raw12=0x40 if trailing_state==1 else 0,rear_error_x100=trailing_error))
    return trailing_state
hook('Route_ReadRear',read_trailing)
def target_capture():
    logical_targets.append(tuple(u.reg_read(reg) for reg in [UC_ARM_REG_R0,UC_ARM_REG_R1,UC_ARM_REG_R2,UC_ARM_REG_R3]))
    events.append('TravelFrame_SetWheelTargets');return 0
hook('TravelFrame_SetWheelTargets',target_capture)
def start_search(node,body,mode):
    global clock,yaw,gray_state,gray_raw,odom
    clock+=1000;yaw=0;gray_state=2;gray_raw=0;odom=0
    record(y,'ThiefLocYaw_t',dict(yaw_x100=0,stamp_ms=clock,valid=1))
    call('ThiefLocalization_Start',loc,node,body,y,clock)
    member('thief_localization','yaw_cw_sign',1)
    heading=(body+2*mode)%4
    target,length=road(node,heading)
    record(steps,'MapRouteStep_t',dict(from_node=node,to_node=target,length_mm=length,
      segment_count=1,heading=heading,expected_ports=3,target_kind=2,action=2,drive_mode=mode))
    record(route,'MapRoute_t',dict(id=1601,count=1,start_node=node,heading=heading,drive_mode=mode))
    var('thief_strategy_state',2);var('node_centered',0)
    call('Route_Start')
    return target,length,heading
for node,body,mode in [(11,1,0),(11,1,1),(26,0,1),(38,1,1)]:
    trailing_state=1;trailing_error=-50
    target,length,heading=start_search(node,body,mode)
    if node==38:check(target==37,'reproduce N038 to N037 reverse')
    clock+=301;odom=68;logical_targets.clear();call('App_ThiefStrategy_Task')
    check(var('route_active')==1 and var('route_state')==1,'301ms/68mm still searches')
    check(logical_targets[-1][0]<logical_targets[-1][1],'trailing-left error steers left rather than straight')
    check(sum(logical_targets[-1][:2])==1000,'50 RPM mean recovery speed')
    check(call('TravelFrame_IsReverse')==mode,'recovery retains travel frame')
    gray_state=1;gray_raw=0x60
    for k in range(3):
        clock+=20;call('App_ThiefStrategy_Task')
        check(var('route_state')==(2 if k==2 else 1),'three leading samples before follow')
    check(var('route_active')==1 and var('restart_search_active')==0,'confirmed leading line resumes')
    # Losing both boards immediately brakes; no continued translational search.
    start_search(node,body,mode);trailing_state=2;clock+=20;events.clear();logical_targets.clear()
    call('App_ThiefStrategy_Task')
    check('Route_BrakeControl' in events and not logical_targets,'both missing brakes first')
    # After both stop samples the speed loop must be re-enabled before spin.
    for _ in range(2):clock+=20;call('App_ThiefStrategy_Task')
    events.clear();clock+=20;call('App_ThiefStrategy_Task')
    check('Route_ResetControl' in events,'scan re-enables speed loop after brake')
    check(events.index('Route_ResetControl')<events.index('TravelFrame_SetWheelTargets'),
          'enable occurs before scan targets')
    # Force a confirmed forward threat while search is braking: actual reaction wins.
    var('intercept_pending',1);var('intercept_has_reversed',0)
    ret('Thief_ThreatMask',1<<heading);events.clear();call('Thief_InterceptReact',clock)
    check(var('route_state')==12 and 'Route_BrakeControl' in events,'laser interrupts recovery')
    u.hook_del(hooks.pop('Thief_ThreatMask'))
    # Distance is still bounded even when trailing board remains valid.
    start_search(node,body,mode);trailing_state=1;clock+=600;odom=180
    call('App_ThiefStrategy_Task')
    check(var('route_active')==0,'failed leading capture is distance bounded')
# Validate the REAL travel transform, not only logical outputs.
u.hook_del(hooks.pop('TravelFrame_SetWheelTargets'))
physical=[]
hook('ChassisSpeedControl_SetWheelTargets',lambda:physical.append(tuple(
  int.from_bytes(u.reg_read(reg).to_bytes(4,'little'),'little',signed=True)
  for reg in [UC_ARM_REG_R0,UC_ARM_REG_R1,UC_ARM_REG_R2,UC_ARM_REG_R3])) or 0)
for mode in [0,1]:
    call('TravelFrame_SetReverse',mode);call('TravelFrame_SetWheelTargets',400,600,400,600)
    check(physical[-1]==((400,600,400,600) if not mode else (-600,-400,-600,-400)),
          'real reverse differential mapping')
    call('TravelFrame_SetWheelTargets',200,-200,200,-200)
    check(physical[-1]==(200,-200,200,-200),'scan rotation has same physical CW sign in reverse')
hook('TravelFrame_SetWheelTargets',target_capture)
# Run the linked controller in a simple differential-drive model. This checks
# feedback direction/convergence, not real sensor pitch, friction or wheel slip.
import math
cs=allocate('RestartLineSearch',0x2000c600)
ci=allocate('RestartLineInput',0x2000c650)
co=allocate('RestartLineOutput',0x2000c680)
def controller(lead,trail,le,te,yaw_cw,stopped=1):
    record(ci,'RestartLineInput',dict(leading_valid=lead,trailing_valid=trail,
      leading_error=le,trailing_error=te,yaw_cw_x100=yaw_cw,stopped=stopped,yaw_known=1))
    call('RestartLineSearch_Update',cs,ci,co)
    def value(k):
        off,typ=fields(types['RestartLineOutput'])[k]
        return int.from_bytes(u.mem_read(co+off,size(typ)),'little',signed=k.endswith('x10'))
    return {k:value(k) for k in ['left_x10','right_x10','brake','ready','failed']}
# Scan cannot start or reverse its spin before two measured-stop samples.
call('RestartLineSearch_Reset',cs,100)
check(controller(0,0,0,0,0)['brake'],'scan first brakes')
for _ in range(4):check(controller(0,0,0,0,0,0)['brake'],'moving wheels block scan')
check(controller(0,0,0,0,0)['brake'],'first stopped frame')
check(controller(0,0,0,0,0)['brake'],'second stopped frame')
o=controller(0,0,0,0,0)
check(o['left_x10']>0 and o['right_x10']<0,'scan rotates without translation')
check(controller(0,0,0,0,400)['brake'],'brake before sweep reversal')
for _ in range(2):check(controller(0,0,0,0,400)['brake'],'measured stop before reverse sweep')
o=controller(0,0,0,0,400)
check(o['left_x10']<0 and o['right_x10']>0,'second scan goes opposite way')
check(controller(0,0,0,0,-400)['brake'],'stop after both scan endpoints')
controller(0,0,0,0,-400);check(controller(0,0,0,0,-400)['failed'],'bounded scan fails if no line')
model_results=[]
for initial_y,initial_deg in [(-40,-8),(40,8),(-47,0),(47,0)]:
    lateral=float(initial_y);theta=math.radians(initial_deg);ready=False;travel=0.0
    call('RestartLineSearch_Reset',cs,-1 if lateral>0 else 1)
    for frame in range(200):
        lead=(-lateral-80*math.sin(theta))/math.cos(theta)
        trail=(-lateral+80*math.sin(theta))/math.cos(theta)
        o=controller(abs(lead)<=44,abs(trail)<=44,round(lead*12.5),round(trail*12.5),round(math.degrees(theta)*100))
        check(not o['failed'],'model did not exhaust scan before capture')
        if o['ready']:ready=True;break
        if o['brake']:continue
        vl=o['left_x10']*204.204/600.0;vr=o['right_x10']*204.204/600.0
        v=(vl+vr)/2;theta+=(vl-vr)/200.0*0.02
        lateral+=v*math.sin(theta)*0.02;travel+=abs(v)*0.02
    check(ready and travel<180,'feedback captures leading board before distance limit')
    model_results.append(dict(initial_y=initial_y,initial_deg=initial_deg,frames=frame,travel_mm=round(travel,2)))
# Unknown initial pose must not acquire an invented location from empty gray.
member('thief_localization','initialized',0);var('thief_strategy_state',0)
call('Route_Start')
check(var('route_active')==0 and var('route_state')==10,'initial no-line start remains rejected')
# Telemetry runs during motion, one channel every 25ms; queue pressure cannot brake.
u.hook_del(hooks.pop('Thief_LaserLog'))
lines=[]
def capture_line():
    addr=u.reg_read(UC_ARM_REG_R0);data=bytes(u.mem_read(addr,256)).split(b'\0',1)[0]
    lines.append(data.decode());return 1
hook('UartDebug_TrySendString',capture_line)
var('route_active',1);var('thief_strategy_state',3)
clock+=1000;call('Thief_LaserLog',clock);lines.clear()
for _ in range(1000):
    clock+=1;call('Thief_LaserLog',clock)
check(len(lines)==40,f'40 total lines per second during motion, got {len(lines)}')
for direction in ['FRONT','LEFT','RIGHT','REAR']:
    check(sum(('DIR='+direction+' ') in line for line in lines)==10,'10 Hz per channel '+direction)
ret('UartDebug_TrySendString',0);events.clear();clock+=25;call('Thief_LaserLog',clock)
check(var('route_active')==1 and 'Route_BrakeControl' not in events,'full telemetry queue cannot stop vehicle')
result=dict(checks=checks,failures=0,scope='Actual application functions from linked ARMCC5 firmware; hardware functions stubbed',
 firmware_sha256=hashlib.sha256(axf.read_bytes()).hexdigest(),
 app_source_sha256=hashlib.sha256((ROOT/'User/App_ThiefStrategy.c').read_bytes()).hexdigest(),
 test_script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
 root_turn_cases=root_turn_cases,
 hide_arrival_cases=hide_arrival_cases,
 recovery_model_cases=model_results,
 scenarios=['two-frame interception and immediate brake','measured-stop-gated reversal',
 'partial-leg coordinate continuity','full-task return, settle, replan and restart',
 'root quarter-turn and first-step commit','slow valid planner keeps running',
 'transient junction gray failure recovers','post-reverse gray failure recovers','manual stop preserved'])
(ROOT/'Tests/results_app_runtime.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
print(json.dumps(result,indent=2))
