#ifndef THIEF_STRATEGY_CONFIG_H
#define THIEF_STRATEGY_CONFIG_H
#define THIEF_STRATEGY_VERSION_16E 1
#define THIEF_STRATEGY_VERSION_16G 1

/* Score actual cycle exits and the slower fore/aft route to the next useful
 * junction. Integer quality weights; not experimentally measured constants. */
#ifndef THIEF_HIDE_EXIT_WEIGHT
#define THIEF_HIDE_EXIT_WEIGHT 1800L
#endif
#ifndef THIEF_HIDE_AXIS_WEIGHT
#define THIEF_HIDE_AXIS_WEIGHT 2L
#endif
#ifndef THIEF_HIDE_FORWARD_WEIGHT
#define THIEF_HIDE_FORWARD_WEIGHT 1L
#endif
#ifndef THIEF_HIDE_ALIGNED_PENALTY_MS
#define THIEF_HIDE_ALIGNED_PENALTY_MS 0L /* Trial prior disabled: speed cases did not improve together */
#endif

/* Planning estimates, not newly measured vehicle data. All costs are integer ms.
 * Recalibrate after changing motor speed or turn/reverse control parameters. */
#define THIEF_FORWARD_MM_S       674UL
#define THIEF_REVERSE_MM_S       679UL
#define THIEF_POLICE_MAX_MM_S    815UL /* Assumed 1.2 x max reference FWD/REV speed */
#define THIEF_TURN_EXTRA_MS      768UL
#define THIEF_SWITCH_TO_REV_MS   282UL
#define THIEF_SWITCH_TO_FWD_MS   274UL
#define THIEF_STARTUP_MS          80UL /* Starting from an already stopped hide */
#define THIEF_CONTACT_MM         180UL
#define THIEF_TIMING_GUARD_MS    120UL
#define THIEF_PLAN_BUDGET_MS     100UL
#define THIEF_THREAT_MEMORY_MS  4000UL
#define THIEF_RISK_HORIZON_MS   3000UL
#define THIEF_HIDE_COOLDOWN_MS 30000UL
#define THIEF_MIN_RELOCATE_MM    600UL
#define THIEF_STATE_COUNT (MAP_ROUTE_NODE_COUNT * 8U)
#define THIEF_COST_INF 0x3FFFFFFFUL

/* Independent ablations may override these at compile time. The production
 * defaults are selected only after the replay comparison. */
#ifndef THIEF_EMERGENCY_ESCAPE
#define THIEF_EMERGENCY_ESCAPE 1
#endif
#ifndef THIEF_REACTION_HIDE_SCORE
#define THIEF_REACTION_HIDE_SCORE 0 /* Did not improve the compared cases */
#endif
#ifndef THIEF_FAST_NEAR_CONFIRM
#define THIEF_FAST_NEAR_CONFIRM 1
#endif
#define THIEF_NEAR_CONFIRM_MM 400U
#define THIEF_NEAR_CONFIDENCE 20U
#define THIEF_CONFIRM_FRAME_MS 20UL /* Estimate; actual UART frame cadence matters */
#ifndef THIEF_HIDE_REACTION_WEIGHT
#define THIEF_HIDE_REACTION_WEIGHT 40L
#endif
#ifndef THIEF_TURN_REACH_MODEL
#define THIEF_TURN_REACH_MODEL 0
#endif
/* Trial assumptions about the opponent, not measured specifications. If its
 * minimum turn/switch time is unknown, zero retains the optimistic lower bound. */
#ifndef THIEF_POLICE_MIN_TURN_MS
#define THIEF_POLICE_MIN_TURN_MS 500UL
#endif
#define THIEF_POLICE_MIN_SWITCH_MS 120UL
#ifndef THIEF_INITIAL_HIDE_NODE
#define THIEF_INITIAL_HIDE_NODE 38U /* Fixed-map first hide; scored arrival axis, later hides selected online. */
#endif
#ifndef THIEF_BRANCH_JUNCTION_HIDES
#define THIEF_BRANCH_JUNCTION_HIDES 0
#endif

#endif
