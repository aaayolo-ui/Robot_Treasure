"""Restore the saved MONITOR7 turn implementation, retaining identified fixes."""
from pathlib import Path
import re
root=Path(__file__).resolve().parents[1]
current=(root/'Tests/before_monitor7_restore/User/App_ThiefStrategy.c').read_text(encoding='utf-8')
restored=(root/'Tests/before_turn_resume/User/App_ThiefStrategy.c').read_text(encoding='utf-8')
def function(src,name):
    match=re.search(r'^static [^;{}]*\b'+name+r'\([^;{}]*\)\s*\{',src,re.M)
    assert match,name
    end=src.index('\n}',match.end())+2
    return src[match.start():end]
restored=restored.replace(function(restored,'Route_MapGrayContext'),
    function(current,'Route_MapRoadIdentity')+'\n\n'+function(current,'Route_MapGrayContext'))
restored=restored.replace(function(restored,'Route_MapGrayRecover'),function(current,'Route_MapGrayRecover'))
old='if (keep_speed_loop != 0U)'
assert restored.count(old)==1
restored=restored.replace(old,'if ((keep_speed_loop != 0U) && ChassisSpeedControl_IsEnabled())')
old='''      if(!RoadHeading_TurnError(turn_start_angle_x100,TravelFrame_Yaw(yaw->yaw_x100),
          (int8_t)(step->action==MAP_ACTION_RIGHT?1:-1),thief_localization.yaw_cw_sign,error))return 0U;
      if(!thief_localization.yaw_cw_sign)return 1U;'''
new='''      /* A known map axis remains usable after inertial overshoot beyond
       *110relative degrees. Only unknown gyro direction needs turn inference. */
      if(!thief_localization.yaw_cw_sign ||
          ((departure_turn?step->from_node:step->to_node)!=4U &&
           (departure_turn?step->from_node:step->to_node)!=6U)) {
        if(!RoadHeading_TurnError(turn_start_angle_x100,TravelFrame_Yaw(yaw->yaw_x100),
            (int8_t)(step->action==MAP_ACTION_RIGHT?1:-1),thief_localization.yaw_cw_sign,error))return 0U;
        if(!thief_localization.yaw_cw_sign)return 1U;
      }'''
assert restored.count(old)==1
restored=restored.replace(old,new).replace('UARTCH4_16U_MONITOR7','UARTCH4_16U_MONITOR10')
assert 'turn_coast_braking' not in restored and 'Route_TurnPwm' not in restored
assert 'Route_AlignTreasureTurn' in restored
(root/'User/App_ThiefStrategy.c').write_text(restored,encoding='utf-8')
(root/'Tests/test_yaw_reference.py').write_bytes((root/'Tests/before_turn_resume/Tests/test_yaw_reference.py').read_bytes())
print('MONITOR7 turn/trim restored with PI-enable, road-yaw recovery and known-axis overshoot fixes.')
