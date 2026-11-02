#ifndef MAP_ROUTE_POSITION_H
#define MAP_ROUTE_POSITION_H
#include <stdint.h>
/* Nominal 65 mm tire from repository hardware list, not a calibrated scale. */
#define MAP_ROUTE_WHEEL_CIRCUMFERENCE_UM 204204UL
int32_t MapRoutePosition_ForwardCounts(void);
int32_t MapRoutePosition_DistanceMm(int32_t start_counts);
uint32_t MapRoutePosition_RemainingApproachMs(uint32_t requested_ms, int32_t traveled_mm,
                                             int32_t rpm_x10);
#endif
