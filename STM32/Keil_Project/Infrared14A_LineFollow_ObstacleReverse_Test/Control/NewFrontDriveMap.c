#include "NewFrontDriveMap.h"
#include <stddef.h>

/* Vehicle coordinates: FL=B, FR=A, RL=D, RR=C. */
static const NewFrontDriveMap_t drive_map[CHASSIS_WHEEL_COUNT] =
{
  {MOTOR_ID_B, ENCODER_ID_B, MOTOR_DIRECTION_REVERSE,  1},
  {MOTOR_ID_A, ENCODER_ID_A, MOTOR_DIRECTION_REVERSE, -1},
  {MOTOR_ID_D, ENCODER_ID_D, MOTOR_DIRECTION_FORWARD,  1},
  {MOTOR_ID_C, ENCODER_ID_C, MOTOR_DIRECTION_FORWARD, -1}
};

const NewFrontDriveMap_t *NewFrontDriveMap_Get(ChassisWheelId_t wheel)
{
  if ((uint32_t)wheel >= (uint32_t)CHASSIS_WHEEL_COUNT)
  {
    return NULL;
  }
  return &drive_map[wheel];
}
