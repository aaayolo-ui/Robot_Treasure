#ifndef THIEF_STRATEGY_CONFIG_H
#define THIEF_STRATEGY_CONFIG_H
#define THIEF_STRATEGY_VERSION_16E 1
#define THIEF_STRATEGY_VERSION_16G 1
#define THIEF_STRATEGY_VERSION_16H 1
#define THIEF_STRATEGY_VERSION_16I 1
#define THIEF_STRATEGY_VERSION_16J 1
#define THIEF_STRATEGY_VERSION_16M 1
/* Software acceptance range, not the sensor's physical ranging limit. */
#define THIEF_LASER_MAX_MM 1000U
#define THIEF_LASER_TRIGGER_MM 600U /* Two 300 mm cells, measured from car center. */
#define THIEF_LASER_SIDE_OFFSET_MM 100U
#define THIEF_LASER_REAR_OFFSET_MM 150U
/* User calibration point: raw 1000 mm ~= physical 900 mm. Single-point
 * proportional estimate for map projection, not a measured calibration curve. */
#define THIEF_LASER_SCALE_NUM 9UL
#define THIEF_LASER_SCALE_DEN 10UL
#define THIEF_INTERCEPT_COOLDOWN_MS 350UL
#define THIEF_LASER_MOVING_FRAME_MAX_MS 25UL
#define THIEF_LASER_MOVING_YAW_TOL_X100 500L /* 5 degrees for cardinal ray projection. */
#ifndef THIEF_ROLLOUT_ENABLED
#define THIEF_ROLLOUT_ENABLED 0 /* 16M searches the live graph and five fixed axes. */
#endif
#ifndef THIEF_ROLLOUT_RISK_TIERS
#define THIEF_ROLLOUT_RISK_TIERS 1
#endif
#ifndef THIEF_ROLLOUT_TIME_BUCKET_MS
#define THIEF_ROLLOUT_TIME_BUCKET_MS 120U /* same as timing guard; a planning assumption, not measurement precision */
#endif

#if THIEF_ROLLOUT_TIME_BUCKET_MS < 1
#error Contact time bucket must be positive
#endif

/* Rollout advances at committed-motion phase endpoints, with interval checks. */
#define THIEF_ROLLOUT_HOLD_MS 340UL
#define THIEF_ROLLOUT_MAX_MS 20000UL
#define THIEF_ROLLOUT_POLICE_TURN_MS 768UL /* reference model, not a measured opponent bound */
#ifndef THIEF_ENTRY_EVIDENCE
#define THIEF_ENTRY_EVIDENCE 1 /* recent clear ray + endpoint-junction entry */
#endif
#ifndef THIEF_ROLLOUT_PRUNE
#define THIEF_ROLLOUT_PRUNE 1 /* exact optimistic-bound pruning, no candidate truncation */
#endif
#ifndef THIEF_ROLLOUT_MODELS
#define THIEF_ROLLOUT_MODELS 6U /* two speeds x follow, alternate search, branch interception */
#endif

/* Nominal 190 x 198 mm bodies: ceil(sum of their half diagonals)) = 275.
 * Planning inflation only; the referee still uses actual rectangle contact.
 * Not measured braking/acceleration or a guarantee against an unseen car. */
#define THIEF_TURN_CLEAR_MM 275UL
#define THIEF_HIDE_REACTION_MS 340UL /* settle 80 + confirm 80 + plan 100 + start 80 */
#ifndef THIEF_TURN_SAFETY
#define THIEF_TURN_SAFETY 1
#endif
#ifndef THIEF_HIDE_RUNWAY_WEIGHT
#define THIEF_HIDE_RUNWAY_WEIGHT 8L
#endif
#ifndef THIEF_AWAY_TURN_PENALTY
#define THIEF_AWAY_TURN_PENALTY 20000UL
#endif
#ifndef THIEF_APPROACH_AXIS_PENALTY
#define THIEF_APPROACH_AXIS_PENALTY 12000L
#endif

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
#ifndef THIEF_CONTACT_MM
#define THIEF_CONTACT_MM         275UL
#endif
#define THIEF_TIMING_GUARD_MS    120UL
#define THIEF_PLAN_BUDGET_MS     100UL
#define THIEF_THREAT_MEMORY_MS  4000UL
#ifndef THIEF_RISK_HORIZON_MS
#define THIEF_RISK_HORIZON_MS   6000UL
#endif
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
#define THIEF_NEAR_CONFIRM_MM THIEF_LASER_TRIGGER_MM /* Two good consecutive frames trigger reaction. */
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
