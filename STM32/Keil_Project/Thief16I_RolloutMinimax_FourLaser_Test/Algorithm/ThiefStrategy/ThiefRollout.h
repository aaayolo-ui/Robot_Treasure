#ifndef THIEF_ROLLOUT_H
#define THIEF_ROLLOUT_H
#include "ThiefRiskSearch.h"
#include <stdint.h>
void ThiefRollout_Init(void);
void ThiefRollout_ResetEvidence(void);
void ThiefRollout_Evidence(uint16_t own_node,uint8_t world_heading,
    uint16_t distance,uint8_t confidence,uint32_t tick);
void ThiefRollout_MarkExposed(uint16_t node,uint32_t now);
uint8_t ThiefRollout_Select(uint16_t node,uint8_t body,uint8_t mask,
    uint16_t previous,uint32_t now,uint16_t *path,uint16_t *count,
    uint8_t *mode,ThiefDecision_t *decision);
uint32_t ThiefRollout_WorkBytes(void);
#ifdef THIEF_ROLLOUT_TEST
uint8_t ThiefRollout_TestView(int32_t x0,int32_t y0,int32_t x1,int32_t y1);
int32_t ThiefRollout_TestSweep(int32_t x0,int32_t y0,int32_t x1,int32_t y1,uint8_t rotating);
#endif
#endif
