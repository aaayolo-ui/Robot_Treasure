#ifndef MAP_TIMING_LOG_H
#define MAP_TIMING_LOG_H

#include <stdint.h>

typedef enum
{
  MAP_TIMING_BOOT = 1,
  MAP_TIMING_START,
  MAP_TIMING_JUNCTION,
  MAP_TIMING_APPROACH,
  MAP_TIMING_BRAKE,
  MAP_TIMING_ROTATE,
  MAP_TIMING_SPIN_END,
  MAP_TIMING_FOLLOW,
  MAP_TIMING_LINE_STABLE,
  MAP_TIMING_GYRO_WAIT,
  MAP_TIMING_GYRO_RECOVER,
  MAP_TIMING_FRONT_WAIT,
  MAP_TIMING_END,
  MAP_TIMING_SAMPLE,
  MAP_TIMING_NODE_ARRIVE,
  MAP_TIMING_LEG_BEGIN,
  MAP_TIMING_PASS_BEGIN
} MapTimingEvent_t;

void MapTimingLog_Init(void);
void MapTimingLog_SetRoute(uint16_t route, uint16_t step,
                          uint16_t from_node, uint16_t to_node);
void MapTimingLog_Begin(uint32_t now);
void MapTimingLog_Event(MapTimingEvent_t event, uint8_t junction,
                        uint32_t now, int32_t argument);
uint8_t MapTimingLog_FollowSample(uint32_t now, uint8_t junction,
                                  uint8_t front_valid);
void MapTimingLog_Task(void);

#endif /* MAP_TIMING_LOG_H */
