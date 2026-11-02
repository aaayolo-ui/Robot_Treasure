"""One-time rebuild from desktop RUN8; does not flash or open serial ports."""
from pathlib import Path
import re,json,hashlib,shutil
P=Path(__file__).resolve().parents[1]
B=Path('C:/Users/Administrator/Desktop/Thief16U_UARTCh4_ImmediateRetreat700_Test')
M=P.parent/'Thief16U_UARTCh4_LiveMonitor_Test'
for name in ['README.md','AGENTS.md','Docs/修改规范与回归清单.md']:(P/name).read_text(encoding='utf-8')
def write(path,text):
    with path.open('w',encoding='utf-8',newline='\n') as f:f.write(text)
def function(text,name):
    m=re.search(r'static [^;\n]+\b'+name+r'\([^;]*?\)\s*\{',text);assert m,name
    start=m.start();j=text.index('{',start);depth=1;k=j+1
    while depth:
        depth+=(text[k]=='{')-(text[k]=='}');k+=1
    return text[start:k]
s=(B/'User/App_ThiefStrategy.c').read_text(encoding='utf-8')
def replace_func(name,body):
    global s
    s=s.replace(function(s,name),body.strip())
# Remove map-driven motion recovery wholesale rather than leaving dormant paths.
for name in ['Route_TurnExitContext','Route_TurnExitRecover','Route_MapGrayContext','Route_MapGrayRecover']:
    s=s.replace(function(s,name),'')
s=re.sub(r'^static (?:uint8_t Route_TurnExitContext\([^;]+|void Route_(?:TurnExitRecover|MapGrayRecover)\([^;]+);\n','',s,flags=re.M)
start=s.index('static uint8_t turn_exit_active,')
end=s.index('static uint8_t route_key_release_pending;',start)
s=s[:start]+'''static RestartLineSearch gray_search;
static uint8_t gray_missing_active;
static uint32_t gray_missing_tick,gray_log_tick;
static int16_t gray_entry_yaw;
static uint8_t observer_ok;
static uint8_t route_body_heading;
static MapRouteStep_t departure_step;
'''+s[end:]
s=s.replace('static uint8_t map_gray_active,map_gray_braked;','')
# Delete all obsolete reset-only references.
s=s.replace('map_gray_active=0U;turn_exit_active=0U;','')
s=s.replace('restart_search_active=0U;','restart_search_active=0U;')
s=s.replace('thief_localization.body_heading','route_body_heading')
s=s.replace('  if (keep_speed_loop != 0U)','  if ((keep_speed_loop != 0U) && ChassisSpeedControl_IsEnabled())')
s=s.replace('&thief_localization.step :','&departure_step :')
replace_func('Route_MapBeginLeg',r'''
static uint8_t Route_MapBeginLeg(uint32_t now)
{
  ThiefLocYaw_t yaw;const MapRouteStep_t *step=MapRouteNavigator_Current(&map_nav);
  Thief_LocalYaw(&yaw);
  observer_ok=ThiefLocalization_BeginStep(&thief_localization,step,&yaw,now);
  route_body_heading=(uint8_t)((step->heading+2U*step->drive_mode)%4U);
  Route_MapContext();map_leg_start_counts=MapRoutePosition_ForwardCounts();
  map_progress_offset_mm=0L;node_centered=0U;node_stop_counts_valid=0U;
  junction_armed=0U;junction_latched=0U;front_was_valid=0U;rearm_valid_count=0U;
  branch_node=0U;branch_headings=0U;thief_laser_pose_since=now;Thief_ResetLaserEvidence();
  MapTimingLog_Event(MAP_TIMING_LEG_BEGIN,junction_count,now,0L);
  return 1U;
}''')
replace_func('Thief_UpdatePose',r'''
static void Thief_UpdatePose(void)
{
  const MapRouteStep_t *step=MapRouteNavigator_Current(&map_nav);
  ThiefLocYaw_t yaw;Thief_LocalYaw(&yaw);
  if(thief_localization.initialized)
    observer_ok=ThiefLocalization_Update(&thief_localization,route_active?Route_LegProgress():0L,&yaw,SystemTime_GetMs());
  /* Navigation estimate is separate from the diagnostic observer. Neither
   * observation rejection nor LOC_DISTANCE may call a motor operation here. */
  if(route_active && step) {
    if(!map_nav.arrived && !departure_turn)ThiefMapPlanner_PoseOnStep(step,Route_LegProgress(),&thief_pose);
    else ThiefMapPlanner_PoseAtNode(departure_turn?map_nav.route->start_node:step->to_node,
        route_body_heading,&thief_pose);
  } else ThiefMapPlanner_PoseAtNode(thief_current_node,thief_body_heading,&thief_pose);
  if(route_state==ROUTE_STATE_TURN_APPROACH || route_state==ROUTE_STATE_TURN_STOP ||
      route_state==ROUTE_STATE_TURN_LEFT || route_state==ROUTE_STATE_TURN_RIGHT ||
      route_state==ROUTE_STATE_HIDE_APPROACH || thief_strategy_state==THIEF_STRATEGY_FAULT)thief_pose.valid=0U;
}''')
# Label route-estimated coordinates explicitly; keep MT3 route events unchanged.
s=s.replace('  if(route_active || thief_strategy_state==THIEF_STRATEGY_SETTLING || (uint32_t)(now-last)<500UL) return;',
            '  if((uint32_t)(now-last)<500UL) return;')
s=s.replace('STATUS=%s\\r\\n",','STATUS=%s OBSERVER_ONLY=1\\r\\n",',1)
s=s.replace('(unsigned)thief_pose.valid,(unsigned)thief_localization.phase,',
            '0U,(unsigned)thief_localization.phase,',1)
s=s.replace('ThiefLocalization_ErrorText(thief_localization.error));\n}',
            '"ESTIMATED");\n}',1)
s=s.replace('      thief_localization.error != THIEF_LOC_OK ||\n','')
# Observe exit validation, but never veto actual gray-controlled route execution.
s=s.replace('    if(!ThiefLocalization_ExitReady(&thief_localization,',
            '    observer_ok=ThiefLocalization_ExitReady(&thief_localization,')
s=s.replace('        map_stable,&yaw,SystemTime_GetMs())) map_stable=0U;\n    if(thief_localization.error!=THIEF_LOC_OK) {\n      Route_StopFault(ThiefLocalization_ErrorText(thief_localization.error));\n      return 0U;\n    }',
            '        map_stable,&yaw,SystemTime_GetMs());')
s=s.replace('      if(!ThiefLocalization_CommitExit(&thief_localization,\n          &map_nav.steps[map_nav.step+(departure_turn?0U:1U)],\n          &yaw,SystemTime_GetMs()) || (!departure_turn && !MapRouteNavigator_Advance(&map_nav)) ||',
'''      observer_ok=ThiefLocalization_CommitExit(&thief_localization,
          &map_nav.steps[map_nav.step+(departure_turn?0U:1U)],&yaw,SystemTime_GetMs());
      if((!departure_turn && !MapRouteNavigator_Advance(&map_nav)) ||''')
s=s.replace('    if(!ThiefLocalization_Arrive(&thief_localization,MapRouteNavigator_Current(&map_nav),\n        raw12,Route_LegProgress(),&yaw,SystemTime_GetMs())) {\n      Route_StopFault(ThiefLocalization_ErrorText(thief_localization.error));\n      return;\n    }',
'''    observer_ok=ThiefLocalization_Arrive(&thief_localization,MapRouteNavigator_Current(&map_nav),
        raw12,Route_LegProgress(),&yaw,SystemTime_GetMs());''')
# Execution frame belongs to the route, not an observer that may reject a sample.
s=s.replace('  map_pass_valid_count = 0U;', '  map_pass_valid_count = 0U;')
s=s.replace('  uint8_t turn_delta=0U;\n','  uint8_t turn_delta=0U;\n  route_body_heading=thief_body_heading;\n')
s=s.replace('departure_reacquire=(uint8_t)(thief_localization.initialized &&',
            'departure_reacquire=(uint8_t)(thief_localization.initialized &&')
s=s.replace('    if(!ThiefLocalization_PrepareDeparture(&thief_localization,&thief_plan.steps[0],&yaw,now)) {\n      Route_StopFault("ROOT_TURN_POSE");return;\n    }',
'''    observer_ok=ThiefLocalization_PrepareDeparture(&thief_localization,&thief_plan.steps[0],&yaw,now);
    departure_step=thief_plan.steps[0];
    departure_step.action=turn_delta==1U?MAP_ACTION_RIGHT:MAP_ACTION_LEFT;''')
s=s.replace('!s || departure_turn ||\n      map_nav.arrived || thief_localization.phase!=THIEF_LOC_ON_LEG ||','!s || departure_turn ||\n      map_nav.arrived ||')
s=s.replace('!(route_state==ROUTE_STATE_FRONT_RECOVERY && (map_gray_active || turn_exit_active))',
            'route_state!=ROUTE_STATE_FRONT_RECOVERY')
for call,reason in [('CancelDeparture(&thief_localization,progress,&yaw,now)','INTERCEPT_START_POSE'),
                    ('ReverseLeg(&thief_localization,&back,progress,&yaw,now)','INTERCEPT_REVERSE_POSE')]:
    old='if(!ThiefLocalization_'+call+') {\n      Route_StopFault("'+reason+'");return;\n    }'
    # The reverse block has a different indentation.
    if old not in s:old=old.replace('\n      Route_StopFault','\n    Route_StopFault').replace('\n    }','\n  }')
    assert old in s,reason
    s=s.replace(old,'observer_ok=ThiefLocalization_'+call+';')
# A confirmed navigation node allows its planned branch, not observer tolerance.
s=s.replace('!(map_nav.arrived && thief_localization.phase==THIEF_LOC_AT_JUNCTION &&\n              thief_localization.error==THIEF_LOC_OK &&\n              thief_localization.confirmed_node==MapRouteNavigator_Current(&map_nav)->to_node)',
            '!map_nav.arrived')
# One sensor-based recovery; no map yaw fine trim, probe/return or candidate states.
replace_func('Route_EnterFrontRecovery',r'''
static void Route_EnterFrontRecovery(uint32_t now,RouteGrayState_t cause)
{
  ThiefLocYaw_t yaw;Thief_LocalYaw(&yaw);
  front_resume_state=route_state;front_resume_tick=route_state_tick;
  front_resume_base_rpm_x10=follow_smoothing.base_rpm_x10;
  front_recovery_valid_count=0U;gray_missing_active=0U;
  gray_entry_yaw=yaw.yaw_x100;gray_log_tick=now;
  RestartLineSearch_Reset(&gray_search,last_leading_error);
  route_state=ROUTE_STATE_FRONT_RECOVERY;route_state_tick=now;
  Route_BrakeControl();
  MapTimingLog_Event(MAP_TIMING_FRONT_WAIT,junction_count,now,(int32_t)cause);
}''')
start=s.index('    case ROUTE_STATE_FRONT_RECOVERY:')
end=s.index('    case ROUTE_STATE_',start+10)
s=s[:start]+r'''
    case ROUTE_STATE_FRONT_RECOVERY:
      {
        RestartLineInput input;RestartLineOutput output;ThiefLocYaw_t yaw;
        Route_ResetSample(&sample);front_state=Route_ReadLeadingOnce(&sample,&leading_cache);
        rear_state=Route_ReadRear(&sample);Thief_LocalYaw(&yaw);
        if(front_state==ROUTE_GRAY_I2C_ERROR || rear_state==ROUTE_GRAY_I2C_ERROR) {
          Route_BrakeControl();
          if((uint32_t)(now-route_state_tick)>=ROUTE_FRONT_RECOVERY_MS)Route_StopFault("GRAY_I2C");
          break;
        }
        if(!sample.front_raw12 && !sample.rear_raw12) {
          if(!gray_missing_active){gray_missing_active=1U;gray_missing_tick=now;}
          if((uint32_t)(now-gray_missing_tick)>=2000U){Route_StopFault("GRAY_BOTH_NO_LINE");break;}
        } else gray_missing_active=0U;
        if(!map_nav.arrived && Route_IsJunctionCandidate(front_state) && junction_armed && !junction_latched) {
          if(!ChassisSpeedControl_IsEnabled())Route_ResetControl(1U);
          Route_HandleJunction(now,elapsed_ms,front_state,sample.front_raw12);break;
        }
        input.leading_valid=(uint8_t)(front_state==ROUTE_GRAY_VALID ||
            front_state==ROUTE_GRAY_LEFT_BLACK || front_state==ROUTE_GRAY_RIGHT_BLACK);
        input.trailing_valid=(uint8_t)(rear_state==ROUTE_GRAY_VALID);
        if(input.leading_valid) {
          if(++front_recovery_valid_count>=ROUTE_FRONT_RECOVERY_VALID_SAMPLES) {
            Route_EnterFollow(now,1U,front_resume_base_rpm_x10,0U);
            (void)Route_RunFollow(&sample,elapsed_ms);break;
          }
        } else front_recovery_valid_count=0U;
        input.leading_error=sample.front_error_x100;input.trailing_error=sample.rear_error_x100;
        input.yaw_known=(uint8_t)(yaw.valid && (uint32_t)(now-yaw.stamp_ms)<=ROUTE_JY61P_STALE_MS);
        input.yaw_cw_x100=(int16_t)(Route_TurnAngleDeltaX100(yaw.yaw_x100,gray_entry_yaw)*thief_localization.yaw_cw_sign);
        input.stopped=Route_AllWheelsStopped();
        RestartLineSearch_Update(&gray_search,&input,&output);
        if(output.brake || output.failed) {
          Route_BrakeControl();
          if(output.failed)RestartLineSearch_Reset(&gray_search,last_leading_error);
        } else {
          if(!ChassisSpeedControl_IsEnabled())Route_ResetControl(1U);
          TravelFrame_SetWheelTargets(output.left_x10,output.right_x10,output.left_x10,output.right_x10);
          ChassisSpeedControl_Update(elapsed_ms);
        }
        if((uint32_t)(now-gray_log_tick)>=200U) {
          gray_log_tick=now;
          UartDebug_Printf("GRAY_RECOVER RAW=%03X,%03X LR=%d,%d LOST_MS=%lu\r\n",
              (unsigned)sample.front_raw12,(unsigned)sample.rear_raw12,
              (int)output.left_x10,(int)output.right_x10,
              (unsigned long)(gray_missing_active?now-gray_missing_tick:0U));
        }
        if(ChassisSpeedControl_HasStall())Route_StopFault("STALL");
      }
      break;

'''+s[end:]
# Pure raw telemetry from the monitor version is independent of motor control.
replace_func('Thief_LaserLog',function((M/'User/App_ThiefStrategy.c').read_text(encoding='utf-8'),'Thief_LaserLog'))
s=s.replace('UARTCH4_16U_RUN8','UARTCH4_16V_STABLE1')
s=s.replace('Thief16U_UARTCh4_ImmediateRetreat700_Test','Thief16V_UARTCh4_StableBase_Test')
# Initialize execution heading using the user-known startup orientation.
s=s.replace('thief_body_heading = THIEF_START_BODY_HEADING;',
            'thief_body_heading = THIEF_START_BODY_HEADING;route_body_heading=THIEF_START_BODY_HEADING;')
write(P/'User/App_ThiefStrategy.c',s)
# Keep complete proven control files. Only receiver/telemetry and raw-range gate
# come from the latest known four-channel interface, never the MONITOR app.
ported=['Hardware/LaserRx.c','Hardware/LaserRx.h','Algorithm/ThiefStrategy/FrontLaserGate.c',
        'Algorithm/ThiefStrategy/FrontLaserGate.h','Algorithm/ThiefStrategy/ThiefStrategyConfig.h']
for name in ported:shutil.copy2(M/name,P/name)
# Project names and explicit stack provision; do not copy volatile debugger files.
f=P/'MDK-ARM'/('Thief16V_UARTCh4_StableBase_Test.uvprojx')
write(f,f.read_text(encoding='utf-8').replace('Thief16U_UARTCh4_ImmediateRetreat700_Test',P.name))
f=P/'MDK-ARM/startup_stm32f103xe.s';write(f,f.read_text(encoding='utf-8').replace('EQU     0x400','EQU     0xC00'))
manifest={str(f.relative_to(B)).replace('\\','/'):hashlib.sha256(f.read_bytes()).hexdigest()
    for d in ['User','Control','Algorithm','Hardware','System','Core'] for f in (B/d).rglob('*')
    if f.is_file() and f.suffix.lower() in ['.c','.h','.s']}
write(P/'Reference/RUN8_sources.json',json.dumps(manifest,indent=2))
write(P/'Reference/ported_interfaces.json',json.dumps({n:hashlib.sha256((M/n).read_bytes()).hexdigest() for n in ported},indent=2))
print('Stable baseline source rebuilt; inspect, compile and test before delivery.')
