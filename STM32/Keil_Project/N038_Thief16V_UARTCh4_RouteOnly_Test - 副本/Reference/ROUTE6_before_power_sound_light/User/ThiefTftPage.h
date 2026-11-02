#ifndef THIEF_TFT_PAGE_H
#define THIEF_TFT_PAGE_H

#include "FrontLaserGate.h"
#include "ThiefMapPlanner.h"
#include <stdint.h>

void ThiefTftPage_Init(void);
void ThiefTftPage_Task(const char *motion_state, const char *fault_text,
                       const char *strategy_state,
                       const ThiefMapPose_t *pose,
                       const FrontLaserGate_t *laser,
                       uint16_t goal_node);

#endif

