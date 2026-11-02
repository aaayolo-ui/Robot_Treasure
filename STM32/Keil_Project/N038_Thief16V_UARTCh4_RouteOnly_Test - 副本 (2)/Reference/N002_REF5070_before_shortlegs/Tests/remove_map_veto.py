from pathlib import Path
P=Path(__file__).resolve().parents[1]
for n in ['README.md','Docs/修改规范与回归清单.md']:(P/n).read_text(encoding='utf-8')
f=P/'User/App_ThiefStrategy.c';s=f.read_text(encoding='utf-8')
a=s.index('  if ((route_active != 0U) && !departure_turn && (map_nav.arrived == 0U) &&')
b=s.index('#if ROUTE_TURN_APPROACH_DELAY_MS > 0U',a)
s=s[:a]+s[b:]
# Relative spin controls the motor; model-angle acceptance is observation only.
a=s.index('  if ((map_nav.arrived || departure_turn) && (map_step != NULL) &&')
b=s.index('  if((map_nav.arrived || departure_turn) && map_step!=NULL &&',a)
s=s[:a]+s[b:]
s=s.replace('''          if (front_state != ROUTE_GRAY_VALID) {
            Route_StopFault("REVERSE_START_REQUIRES_LEADING_LINE");
            break;
          }
''','')
needle='''          last_front_watch_tick = now;
          (void)Route_RunFollow(&sample, elapsed_ms);
          break;'''
assert needle in s
s=s.replace(needle,'''          last_front_watch_tick = now;
          if(front_state==ROUTE_GRAY_VALID || front_state==ROUTE_GRAY_LEFT_BLACK || front_state==ROUTE_GRAY_RIGHT_BLACK)
            (void)Route_RunFollow(&sample,elapsed_ms);
          else Route_EnterFrontRecovery(now,front_state);
          break;''')
s=s.replace('Route_StopFault("MAP_STRAIGHT_PASS_TIMEOUT");','Route_EnterFrontRecovery(now,front_state);')
s=s.replace('Route_StopFault("AFTER_TURN_NO_LINE");','Route_EnterFrontRecovery(now,ROUTE_GRAY_NO_LINE);')
s=s.replace('Route_StopFault("LEAVE_TIMEOUT");','Route_EnterFrontRecovery(now,front_state);')
s=s.replace('Route_StopFault("LOC_GYRO_STALE");','Route_StopFault("JY61P_STALE");')
# Calibrate the gyro sign from an executed, fresh physical PWM turn, independent
# of whether the position observer accepts its node/angle model.
needle='''  MapTimingLog_Event(MAP_TIMING_SPIN_END, junction_count, now,'''
assert needle in s
s=s.replace(needle,'''  if(!thief_localization.yaw_cw_sign && finish_reason[0]=='Y' && delta) {
    int8_t measured=(int8_t)(delta>0?1:-1);
    thief_localization.yaw_cw_sign=(int8_t)(measured*(route_state==ROUTE_STATE_TURN_RIGHT?1:-1));
  }
'''+needle,1)
old='''  if(route_active &&
      (thief_localization.heading_error_x100>THIEF_LASER_MOVING_YAW_TOL_X100 ||
       thief_localization.heading_error_x100< -THIEF_LASER_MOVING_YAW_TOL_X100)) allow=0U;'''
assert old in s
s=s.replace(old,'''  if(route_active) {
    ThiefLocYaw_t yaw;int16_t error;Thief_LocalYaw(&yaw);
    error=RoadHeading_AxisError(thief_localization.origin_yaw_x100,
        thief_localization.origin_heading,route_body_heading,thief_localization.yaw_cw_sign,yaw.yaw_x100);
    if(!yaw.valid || error>THIEF_LASER_MOVING_YAW_TOL_X100 ||
        error< -THIEF_LASER_MOVING_YAW_TOL_X100)allow=0U;
  }''')
with f.open('w',encoding='utf-8',newline='\n') as out:out.write(s)
