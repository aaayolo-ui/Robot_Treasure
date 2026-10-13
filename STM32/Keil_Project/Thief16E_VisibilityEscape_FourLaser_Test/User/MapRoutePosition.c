#include "MapRoutePosition.h"
#include "Chassis.h"
#include "Encoder.h"
#include "TravelFrame.h"
#include "NewFrontDriveMap.h"
#include <stddef.h>

int32_t MapRoutePosition_ForwardCounts(void)
{
  ChassisWheelId_t wheel;
  int64_t sum = 0;
  for (wheel = CHASSIS_WHEEL_FRONT_LEFT; wheel < CHASSIS_WHEEL_COUNT; wheel++) {
    const NewFrontDriveMap_t *map = NewFrontDriveMap_Get(wheel);
    if (map != NULL) sum += (int64_t)Encoder_GetTotal(map->encoder) * map->encoder_logical_sign;
  }
  return (int32_t)(sum / 4L);
}

int32_t MapRoutePosition_DistanceMm(int32_t start_counts)
{
  int32_t counts = (int32_t)((uint32_t)MapRoutePosition_ForwardCounts() - (uint32_t)start_counts);
  if (TravelFrame_IsReverse() != 0U) counts = -counts;
  return (int32_t)(((int64_t)counts * MAP_ROUTE_WHEEL_CIRCUMFERENCE_UM) /
                   ((int64_t)MOTOR_ENCODER_COUNTS_PER_REV * 1000L));
}
