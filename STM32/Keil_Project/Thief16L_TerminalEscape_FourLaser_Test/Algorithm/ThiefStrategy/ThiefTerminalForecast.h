#ifndef THIEF_TERMINAL_FORECAST_H
#define THIEF_TERMINAL_FORECAST_H
#include <stdint.h>

#define THIEF_TERMINAL_NODE_COUNT 88U
#define THIEF_TERMINAL_WAIT_MAX_MS 6000U
#define THIEF_TERMINAL_EXIT_MAX_MS 6000U
#define THIEF_TERMINAL_PATH_MAX_EDGES 12U
#define THIEF_TERMINAL_MAX_INTERVALS 96U

/* Same committed-motion state used by the rollout. phase: 0 ready at node,
 * 1 stationary action wait, 2 translating, 3 stationary hold. */
typedef struct {
  uint16_t node,to,total,elapsed,length,goal,last;
  uint8_t h,mode,phase,turning,index,first,last_seen_h;
} ThiefTerminalActor_t;
typedef struct {int32_t x,y;int16_t hx,hy;} ThiefTerminalBox_t;

/* Pure callbacks to existing rollout primitives; no duplicate map tables.
 * contact_fraction returns -1 for no contact, otherwise fraction 0..1024.
 * branch(node,h) works at ANY node: return a useful non-dead/non-pocket branch
 * reachable through arm h, or 0. It must not return a fake physical junction.
 * node_kind is 0 for a straight task marker where turning is forbidden.
 * A phase-3 police actor is a genuinely stationary hypothesis and remains so.
 */
typedef struct {
  void *context;
  ThiefTerminalBox_t (*pose)(void *,const ThiefTerminalActor_t *);
  uint8_t (*visible)(void *,const ThiefTerminalBox_t *,const ThiefTerminalBox_t *);
  uint8_t (*visible_during)(void *,const ThiefTerminalBox_t *,const ThiefTerminalBox_t *,
                          const ThiefTerminalBox_t *,const ThiefTerminalBox_t *);
  int32_t (*contact_fraction)(void *,const ThiefTerminalBox_t *,const ThiefTerminalBox_t *,
                            const ThiefTerminalBox_t *,const ThiefTerminalBox_t *);
  void (*command)(void *,ThiefTerminalActor_t *,uint8_t,uint32_t,uint32_t);
  void (*advance)(void *,ThiefTerminalActor_t *,uint16_t,uint32_t);
  uint8_t (*best_police)(void *,uint16_t,uint8_t,uint8_t,uint16_t,uint32_t);
  uint16_t (*next)(void *,uint16_t,uint8_t);
  uint16_t (*length)(void *,uint16_t,uint8_t);
  uint16_t (*branch)(void *,uint16_t,uint8_t);
  int32_t (*reach)(void *,const ThiefTerminalActor_t *,uint16_t,uint32_t);
  uint8_t (*node_kind)(void *,uint16_t);
} ThiefTerminalOps_t;

typedef struct {
  uint32_t forward_speed_mm_s;
  uint16_t turn_ms,reaction_ms,timing_guard_ms,contact_mm;
  uint16_t wait_limit_ms,exit_limit_ms;
  uint8_t endpoint_budget,max_edges;
} ThiefTerminalConfig_t;
typedef struct {
  int32_t reserve_ms;
  uint16_t wait_ms,escape_ms,steps;
  uint8_t known,clear,sensed,exit_heading;
} ThiefTerminalScore_t;

/* Snapshot inputs AT goal arrival, before the old terminal hold is advanced.
 * Only fore/aft geographic exit headings are supported. Reaction includes
 * startup from rest; it does not add a moving reverse-switch penalty.
 * The escape must pass through a genuine quarter turn before the useful
 * terminal branch. Unknown/budget-exhausted results never claim safe reserve.
 * Caller evaluates <=2 finalists x 6 hypotheses x 2 exits, and preserves the
 * legacy result whenever a required forecast is unknown. This is not a bound
 * on measured STM32 wall-clock planning time. Non-reentrant: call sequentially
 * from the stopped main-loop planner, never an ISR. Recursive calls return
 * unknown; a small static workspace is copied/reset for each invocation. */
ThiefTerminalScore_t ThiefTerminalForecast_Run(const ThiefTerminalConfig_t *,
    const ThiefTerminalOps_t *,const ThiefTerminalActor_t *stopped_thief,
    const ThiefTerminalActor_t *pursuer,const uint8_t visits[88],
    uint32_t police_speed_mm_s,uint8_t branch_bias,uint8_t exit_heading);
uint32_t ThiefTerminalForecast_WorkBytes(void);
#endif
