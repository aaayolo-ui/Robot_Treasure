/* Offline adapter only; not part of the STM32 Keil project. */
#include "ThiefMapPlanner.h"
#include "FrontLaserGate.h"
#include "ThiefStrategyConfig.h"
#ifdef THIEF_STRATEGY_VERSION_16I
#include "ThiefRollout.h"
#endif
volatile unsigned bridge_command, bridge_tick, bridge_node, bridge_body;
volatile unsigned bridge_fresh, bridge_distance[4], bridge_confidence[4];
volatile unsigned bridge_previous, bridge_requested, bridge_ok, bridge_mask;
volatile unsigned bridge_goal, bridge_count, bridge_mode, bridge_heading;
volatile unsigned bridge_time_ms, bridge_states, bridge_threats, bridge_degraded;
volatile unsigned bridge_reused;
volatile unsigned bridge_exits,bridge_branch_ms,bridge_parking;
volatile unsigned bridge_pause_risk,bridge_approach;
volatile unsigned bridge_version;
volatile unsigned bridge_refuge,bridge_fast_exits,bridge_patrol_delay;
volatile unsigned bridge_late_checked,bridge_late_known,bridge_late_clear,bridge_late_wait;
volatile unsigned bridge_late_candidates,bridge_late_steps,bridge_emergency;
volatile int bridge_late_reserve;
volatile unsigned bridge_coverage,bridge_risk_band,bridge_runway;
volatile unsigned bridge_candidates,bridge_rollouts,bridge_hit,bridge_algorithm;
volatile int bridge_terminal;
volatile unsigned bridge_model_steps,bridge_clear,bridge_entry;
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
#ifdef THIEF_STRATEGY_VERSION_16L
  bridge_version=1612U;
#elif defined(THIEF_STRATEGY_VERSION_16K)
  bridge_version=1611U;
#elif defined(THIEF_STRATEGY_VERSION_16J)
  bridge_version=1610U;
#elif defined(THIEF_STRATEGY_VERSION_16I)
  bridge_version=1609U;
#elif defined(THIEF_STRATEGY_VERSION_16H)
  bridge_version=1608U;
#elif defined(THIEF_STRATEGY_VERSION_16G)
  bridge_version=1607U;
#else
  bridge_version=1605U;
#endif
#if defined(THIEF_STRATEGY_VERSION_16L) || defined(THIEF_STRATEGY_VERSION_16K)
  bridge_refuge=d->refuge_class;bridge_fast_exits=d->fast_escape_exits;
  bridge_patrol_delay=d->patrol_detection_ms;
#else
  bridge_refuge=0U;bridge_fast_exits=0U;bridge_patrol_delay=0U;
#endif
#ifdef THIEF_STRATEGY_VERSION_16L
  bridge_late_checked=d->terminal_checked;bridge_late_known=d->terminal_known_models;
  bridge_late_clear=d->terminal_clear_models;bridge_late_wait=d->terminal_wait_ms;
  bridge_late_reserve=d->terminal_reserve_ms;bridge_late_candidates=d->terminal_candidates;
  bridge_late_steps=d->terminal_steps;bridge_emergency=d->emergency_hide;
#else
  bridge_late_checked=bridge_late_known=bridge_late_clear=bridge_late_wait=0U;
  bridge_late_reserve=0;bridge_late_candidates=bridge_late_steps=bridge_emergency=0U;
#endif
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
#ifdef THIEF_STRATEGY_VERSION_16G
  bridge_exits=d->escape_exits;bridge_branch_ms=d->exit_branch_ms;
  bridge_parking=d->parking_heading;
#else
  bridge_exits=0;bridge_branch_ms=0;bridge_parking=0;
#endif
#ifdef THIEF_STRATEGY_VERSION_16H
  bridge_pause_risk=d->first_pause_risk_ms;bridge_approach=d->approach_mask;
#else
  bridge_pause_risk=0;bridge_approach=0;
#endif
#ifdef THIEF_STRATEGY_VERSION_16I
  bridge_candidates=d->candidates_evaluated;bridge_rollouts=d->model_rollouts;
  bridge_hit=d->forecast_contact_ms;bridge_terminal=d->terminal_escape_ms;bridge_algorithm=d->planner_kind;
  bridge_model_steps=d->model_steps;bridge_clear=d->forecast_clear;bridge_entry=d->entry_inferred;
#else
  bridge_candidates=0;bridge_rollouts=0;bridge_hit=0;bridge_terminal=0;bridge_algorithm=0;
  bridge_model_steps=0;bridge_clear=0;bridge_entry=0;
#endif
#ifdef THIEF_STRATEGY_VERSION_16J
  bridge_coverage=d->surviving_models;bridge_risk_band=d->risk_band_ms;bridge_runway=d->recovery_runway_ms;
#else
  bridge_coverage=0;bridge_risk_band=0;bridge_runway=0;
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
#ifdef THIEF_STRATEGY_VERSION_16I
    ThiefRollout_ResetEvidence();
#endif
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
#ifdef THIEF_STRATEGY_VERSION_16I
        ThiefRollout_Evidence((uint16_t)bridge_node,ray.body_heading,
            (uint16_t)bridge_distance[i],(uint8_t)bridge_confidence[i],bridge_tick);
#endif
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
