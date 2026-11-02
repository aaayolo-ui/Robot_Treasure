#ifndef ROUTE_HISTORY_H
#define ROUTE_HISTORY_H
#include "ThiefMapPlanner.h"
#define ROUTE_HISTORY_CAPACITY 32U
void RouteHistory_Reset(void);
uint16_t RouteHistory_Count(void);
uint32_t RouteHistory_WorkBytes(void);
uint16_t RouteHistory_LastNode(void);
/* Only call after a real gray/distance/yaw-confirmed complete leg. */
void RouteHistory_Record(const MapRouteStep_t *step,uint16_t raw,int16_t yaw);
/* Reverse confirmed legs to the nearest earlier branch. Does not consume them;
 * inverse arrivals consume history only after actual gray confirmation. */
uint8_t RouteHistory_PlanReturn(uint16_t node,uint8_t body,uint8_t threats,
    uint32_t now,ThiefRoutePlan_t *plan);
#endif
