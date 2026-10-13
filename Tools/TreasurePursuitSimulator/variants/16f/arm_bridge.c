/* Offline adapter only; not part of the STM32 Keil project. */
#include "ThiefMapPlanner.h"
#include "FrontLaserGate.h"
#include "ThiefStrategyConfig.h"
volatile unsigned bridge_command, bridge_tick, bridge_node, bridge_body;
volatile unsigned bridge_fresh, bridge_distance[4], bridge_confidence[4];
volatile unsigned bridge_previous, bridge_requested, bridge_ok, bridge_mask;
volatile unsigned bridge_goal, bridge_count, bridge_mode, bridge_heading;
volatile unsigned bridge_time_ms, bridge_states, bridge_threats, bridge_degraded;
volatile unsigned bridge_reused;
volatile int bridge_margin;
volatile unsigned bridge_from[48],bridge_to[48],bridge_length[48];
volatile unsigned bridge_step_heading[48],bridge_step_mode[48],bridge_action[48];
volatile unsigned bridge_status[4];
static ThiefRoutePlan_t plan;
static FrontLaserGate_t gate[4];
static unsigned last_tick[4], seen[4];

static void Export(void)
{
  unsigned i;
  const ThiefDecision_t *d=ThiefMapPlanner_Decision();
  bridge_goal=plan.goal_node;bridge_count=plan.route.count;
  bridge_mode=plan.route.drive_mode;bridge_heading=plan.route.heading;
  bridge_time_ms=d->travel_ms;bridge_margin=d->worst_margin_ms;
  bridge_states=d->states_expanded;bridge_threats=d->threat_count;
  bridge_degraded=d->degraded;
#ifdef THIEF_STRATEGY_VERSION_16E
  bridge_reused=d->reused_hide;
#else
  bridge_reused=0;
#endif
  for(i=0;i<plan.route.count;i++) {
    bridge_from[i]=plan.steps[i].from_node;bridge_to[i]=plan.steps[i].to_node;
    bridge_length[i]=plan.steps[i].length_mm;
    bridge_step_heading[i]=plan.steps[i].heading;
    bridge_step_mode[i]=plan.steps[i].drive_mode;
    bridge_action[i]=(unsigned)(int)plan.steps[i].action;
  }
}
void bridge_execute(void)
{
  static const unsigned offsets[4]={0,3,1,2};
  unsigned i;
  ThiefMapPose_t pose;
  bridge_requested=0;bridge_mask=0;bridge_ok=0;
  if(bridge_command==0) {
    ThiefMapPlanner_Init();
    for(i=0;i<4;i++) {FrontLaserGate_Init(&gate[i]);seen[i]=0;last_tick[i]=0;}
    bridge_ok=ThiefMapPlanner_PlanInitial(&plan);
    if(bridge_ok) Export();
  } else if(bridge_command==1) {
    /* Match the firmware's clean evidence window after arriving/settling. */
    for(i=0;i<4;i++) {FrontLaserGate_ResetEvidence(&gate[i]);seen[i]=0;}
  } else if(bridge_command==2) {
    ThiefMapPlanner_PoseAtNode((unsigned short)bridge_node,(unsigned char)bridge_body,&pose);
    for(i=0;i<4;i++) {
      ThiefMapPose_t ray=pose;
      ray.body_heading=(unsigned char)((pose.body_heading+offsets[i])%4);
      if(!seen[i] || (unsigned)(bridge_tick-last_tick[i])>250)
        FrontLaserGate_ResetEvidence(&gate[i]);
      if(bridge_fresh&(1U<<i)) {
        if(!bridge_distance[i]) FrontLaserGate_ResetEvidence(&gate[i]);
        last_tick[i]=bridge_tick;seen[i]=1;
        if(FrontLaserGate_Observe(&gate[i],&ray,1,(unsigned short)bridge_distance[i],
                                  (unsigned char)bridge_confidence[i],123)) {
          ThiefMapPlanner_ObserveThreat((unsigned short)bridge_node,ray.body_heading,
            gate[i].hit_x_mm,gate[i].hit_y_mm,bridge_tick);
          bridge_requested=1;
        }
      }
      bridge_status[i]=gate[i].status;
      if(seen[i] && (unsigned)(bridge_tick-last_tick[i])<=250 && gate[i].status==FRONT_LASER_OPPONENT)
        bridge_mask|=1U<<ray.body_heading;
    }
    if(bridge_requested && bridge_mask) {
      bridge_ok=ThiefMapPlanner_PlanEscape((unsigned short)bridge_node,(unsigned char)bridge_body,
        (unsigned char)bridge_mask,(unsigned short)bridge_previous,bridge_tick,&plan);
      if(bridge_ok) Export();
    }
  }
}
