#ifndef THIEF_STRATEGY_CONFIG_H
#define THIEF_STRATEGY_CONFIG_H

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

#endif
