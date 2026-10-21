#ifndef AI1811_H
#define AI1811_H

#include <stdint.h>

#define AI1811_DISTANCE_MIN_MM 50U
#define AI1811_DISTANCE_MAX_MM 4000U
#define AI1811_OBSTACLE_MAX_MM 300U

typedef struct
{
  uint16_t distance_mm;
  uint8_t confidence;
  uint16_t firmware;
  uint32_t timestamp_ms;
  uint32_t sequence;
  uint8_t has_frame;
  uint8_t measurement_valid;
  uint8_t obstacle_streak;
} AI1811_Status_t;

void AI1811_Init(void);
void AI1811_GetStatus(AI1811_Status_t *status);
void AI1811_ResetObstacleStreak(void);

/* Called by the USART2 RX interrupt callbacks in AI1811.c. */
void AI1811_OnRxComplete(void);
void AI1811_OnRxError(void);

#endif /* AI1811_H */
