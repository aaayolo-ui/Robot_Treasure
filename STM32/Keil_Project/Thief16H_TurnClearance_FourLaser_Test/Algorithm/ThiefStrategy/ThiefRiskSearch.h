#ifndef THIEF_RISK_SEARCH_H
#define THIEF_RISK_SEARCH_H
#include "MapRoutePlan.h"
#include <stdint.h>

int32_t ThiefRiskSearch_TurnMargin(uint16_t from,uint16_t to,uint16_t length,
                                  uint32_t before,uint32_t wait,uint8_t mode);

typedef struct {
  uint32_t travel_ms;
  int32_t worst_margin_ms;
  uint16_t states_expanded, goal_node;
  uint8_t threat_count, degraded, reused_hide;
  uint16_t exit_branch_ms;
  uint8_t escape_exits, parking_heading;
  uint16_t first_pause_risk_ms;
  uint8_t approach_mask;
} ThiefDecision_t;
void ThiefRiskSearch_Init(void);
void ThiefRiskSearch_MarkExposed(uint16_t node,uint32_t now);
uint8_t ThiefRiskSearch_Select(uint16_t start,uint8_t heading,uint8_t mode,
    uint8_t axis_only,uint8_t threat_mask,uint16_t blocked,uint16_t previous,
    uint32_t now,uint16_t *path,uint16_t *count,uint8_t *first_mode,
    ThiefDecision_t *decision);
uint32_t ThiefRiskSearch_LegTime(uint16_t length,uint8_t mode,
                                uint8_t turn,uint8_t change);
uint32_t ThiefRiskSearch_WorkBytes(void);
uint8_t ThiefRiskSearch_HideSafe(uint16_t node);
#endif
