from pathlib import Path
import re
P=Path(__file__).resolve().parents[1]
for name in ['README.md','Docs/修改规范与回归清单.md']:(P/name).read_text(encoding='utf-8')
def save(p,s):
    with p.open('w',encoding='utf-8',newline='\n') as f:f.write(s)
f=P/'User/App_ThiefStrategy.c';s=f.read_text(encoding='utf-8')
# Collapse the second, odometry-limited startup search into the same recovery.
a=s.index('      else if (departure_reacquire &&\n')
b=s.index('      else if (front_state == ROUTE_GRAY_VALID)',a)
s=s[:a]+'''      else if (departure_reacquire && front_state==ROUTE_GRAY_NO_LINE)
      {
        Route_EnterFrontRecovery(now,front_state);
      }
'''+s[b:]
for name in ['restart_search_active','restart_search_braked','restart_log_tick']:
    s=re.sub(r'\b'+name+r'=0U;','',s)
s=re.sub(r'^static [^;\n]*\brestart_search(?:_active|_braked)?\b[^;\n]*;\n','',s,flags=re.M)
s=re.sub(r'^static uint32_t restart_log_tick;\n','',s,flags=re.M)
s=s.replace('  departure_reacquire=(uint8_t)(thief_localization.initialized &&\n      thief_localization.phase==THIEF_LOC_AT_NODE &&\n      thief_localization.confirmed_node==thief_plan.route.start_node);',
'''  departure_reacquire=(uint8_t)(thief_strategy_state!=THIEF_STRATEGY_READY &&
      thief_current_node==thief_plan.route.start_node);''')
s=s.replace('OBSERVER_ONLY=1\\r\\n",','OBSERVER_ONLY=1 OBS_OK=%u\\r\\n",')
s=s.replace('"ESTIMATED");\n}', '"ESTIMATED",(unsigned)observer_ok);\n}',1)
# A line-empty transition must not issue an unconditional forward burst.
needle='''                   (int)current_angle_x100, (int)delta);
  Route_DriveStraight(elapsed_ms);'''
assert needle in s
s=s.replace(needle,'''                   (int)current_angle_x100, (int)delta);
  {
    RouteGraySample_t sample;Route_ResetSample(&sample);
    front_state=Route_ReadFront(&sample);
    if(front_state==ROUTE_GRAY_NO_LINE || front_state==ROUTE_GRAY_I2C_ERROR)Route_BrakeControl();
    else Route_DriveStraight(elapsed_ms);
  }''')
s=s.replace('RestartLineSearch_Reset(&gray_search,last_leading_error);',
    'RestartLineSearch_Reset(&gray_search,last_leading_mode==TravelFrame_IsReverse()?last_leading_error:(int16_t)-last_leading_error);')
save(f,s)
# N011 first-exit policy is deliberately isolated from motor/gray functions.
f=P/'Algorithm/ThiefStrategy/ThiefRiskSearch.h';s=f.read_text(encoding='utf-8')
s=s.replace('void ThiefRiskSearch_Init(void);','void ThiefRiskSearch_Init(void);\nvoid ThiefRiskSearch_SetDepartureMask(uint8_t mask);')
save(f,s)
f=P/'Algorithm/ThiefStrategy/ThiefRiskSearch.c';s=f.read_text(encoding='utf-8')
s=s.replace('static uint8_t emergency;', 'static uint8_t emergency;\nstatic uint8_t departure_mask=15U;\nvoid ThiefRiskSearch_SetDepartureMask(uint8_t mask){departure_mask=(uint8_t)(mask&15U);}')
s=s.replace('  emergency=0U;', '  emergency=0U;departure_mask=15U;')
needle='      if(pick==root && !emergency && (threat_mask&(1U<<out))) continue;'
assert needle in s
s=s.replace(needle,'''      if(pick==root && !(departure_mask&(1U<<out)))continue;
      if(start==11U && parent[pick]==root && from==12U && out!=1U)continue;
'''+needle)
save(f,s)
f=P/'Algorithm/ThiefStrategy/ThiefMapPlanner.c';s=f.read_text(encoding='utf-8')
a=s.index('uint8_t ThiefMapPlanner_PlanEscape(');b=s.index('\nvoid ThiefMapPlanner_PoseAtNode',a)
s=s[:a]+'''uint8_t ThiefMapPlanner_PlanEscape(uint16_t current_node,
    uint8_t body_heading,uint8_t threat_mask,uint16_t previous_hide_node,
    uint32_t now,ThiefRoutePlan_t *plan)
{
  uint16_t count;uint8_t mode,pass,found=0U,allowed,preferred;
  if(!plan || body_heading>3U)return 0U;
  plan->valid=0U;allowed=current_node==11U?10U:15U;
  preferred=allowed;
  if(current_node==11U) {
    if(threat_mask&4U)preferred=2U;
    else if(threat_mask&1U)preferred=8U;
  }
  for(pass=0U;pass<3U && !found;pass++) {
    ThiefRiskSearch_SetEmergency((uint8_t)(pass==2U));
    ThiefRiskSearch_SetDepartureMask(pass==0U?preferred:allowed);
    found=ThiefRiskSearch_Select(current_node,body_heading,0U,1U,threat_mask,
        0U,previous_hide_node,now,departure,&count,&mode,&decision);
  }
  ThiefRiskSearch_SetEmergency(0U);ThiefRiskSearch_SetDepartureMask(15U);
  return found?BuildRoute(departure,count,mode,ESCAPE_ID,plan):0U;
}
'''+s[b:]
save(f,s)
# Gray-triggered runtime navigation keeps odometry as observation, not a veto.
f=P/'Algorithm/MapRoute/MapRouteNavigator.c';s=f.read_text(encoding='utf-8')
s=s.replace('return (step!=NULL && !nav->fault && !nav->arrived &&\n        travel_mm>=(int32_t)step->length_mm*4L/10L-50L &&\n        !MapRouteNavigator_TooFar(nav,travel_mm)) ? 1U : 0U;',
'''if(nav->runtime)return (uint8_t)(step && !nav->fault && !nav->arrived);
  return (step!=NULL && !nav->fault && !nav->arrived &&
        travel_mm>=(int32_t)step->length_mm*4L/10L-50L &&
        !MapRouteNavigator_TooFar(nav,travel_mm)) ? 1U : 0U;''')
s=s.replace('if ((travel_mm < minimum) || (MapRouteNavigator_TooFar(nav, travel_mm) != 0U)) {',
            'if (!nav->runtime && ((travel_mm < minimum) || (MapRouteNavigator_TooFar(nav, travel_mm) != 0U))) {')
save(f,s)
print('Single gray recovery, shadow localization and isolated N011 policy written.')
