#ifndef ROAD_HEADING_CONTROL_H
#define ROAD_HEADING_CONTROL_H
#include <stdint.h>
typedef struct {
  uint32_t brake_tick,last_yaw_tick;
  int16_t last_turn;
  uint8_t spinning,stopped_count,stable_count;
} RoadHeadingControl_t;
typedef struct {
  int16_t cw_error_x100;
  uint32_t yaw_tick;
  uint8_t stopped,direction_known;
} RoadHeadingInput_t;
typedef struct {int16_t turn_x10;uint8_t brake,ready,failed;} RoadHeadingOutput_t;
int16_t RoadHeading_AxisError(int16_t origin,uint8_t origin_heading,
    uint8_t body,int8_t sign,int16_t current);
int16_t RoadHeading_Correction(int16_t cw_error_x100,int16_t bound);
/* quarter=+1 right/-1 left; zero known_sign is inferred only from an
 * already-observed planned turn, never from a stationary gyro sample. */
uint8_t RoadHeading_TurnError(int16_t start,int16_t current,int8_t quarter,
    int8_t known_sign,int16_t *cw_error);
void RoadHeading_Reset(RoadHeadingControl_t *s,uint32_t now);
void RoadHeading_Update(RoadHeadingControl_t *s,const RoadHeadingInput_t *in,
    uint32_t now,int16_t tolerance,RoadHeadingOutput_t *out);
#endif
