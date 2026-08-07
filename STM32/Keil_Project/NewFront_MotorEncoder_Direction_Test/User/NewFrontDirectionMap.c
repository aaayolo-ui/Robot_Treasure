#include "NewFrontDirectionMap.h"

typedef struct
{
  MotorDirection_t vehicle_forward_electrical_direction;
  int8_t encoder_logical_sign;
} NewFrontDirectionMap_Entry_t;

static const NewFrontDirectionMap_Entry_t new_front_direction_map[MOTOR_ID_COUNT] =
{
  {MOTOR_DIRECTION_REVERSE, -1}, /* A = FR */
  {MOTOR_DIRECTION_REVERSE,  1}, /* B = FL */
  {MOTOR_DIRECTION_FORWARD, -1}, /* C = RR */
  {MOTOR_DIRECTION_FORWARD,  1}  /* D = RL */
};

uint8_t NewFrontDirectionMap_GetElectricalDirection(
  MotorId_t motor,
  NewFrontVehicleDirection_t vehicle_direction,
  MotorDirection_t *electrical_direction)
{
  MotorDirection_t forward_direction;

  if (((uint32_t)motor >= (uint32_t)MOTOR_ID_COUNT) ||
      (electrical_direction == 0) ||
      ((vehicle_direction != NEW_FRONT_VEHICLE_DIRECTION_FORWARD) &&
       (vehicle_direction != NEW_FRONT_VEHICLE_DIRECTION_REVERSE)))
  {
    return 0U;
  }

  forward_direction = new_front_direction_map[motor].vehicle_forward_electrical_direction;
  *electrical_direction =
    (vehicle_direction == NEW_FRONT_VEHICLE_DIRECTION_FORWARD) ? forward_direction :
    ((forward_direction == MOTOR_DIRECTION_FORWARD) ?
     MOTOR_DIRECTION_REVERSE : MOTOR_DIRECTION_FORWARD);
  return 1U;
}

int8_t NewFrontDirectionMap_GetEncoderLogicalSign(MotorId_t motor)
{
  return ((uint32_t)motor < (uint32_t)MOTOR_ID_COUNT) ?
         new_front_direction_map[motor].encoder_logical_sign : 0;
}

uint8_t NewFrontDirectionMap_RunSelfTest(void)
{
  MotorDirection_t direction;

  return ((NewFrontDirectionMap_GetElectricalDirection(
             MOTOR_ID_B, NEW_FRONT_VEHICLE_DIRECTION_FORWARD, &direction) != 0U) &&
          (direction == MOTOR_DIRECTION_REVERSE) &&
          (NewFrontDirectionMap_GetElectricalDirection(
             MOTOR_ID_A, NEW_FRONT_VEHICLE_DIRECTION_FORWARD, &direction) != 0U) &&
          (direction == MOTOR_DIRECTION_REVERSE) &&
          (NewFrontDirectionMap_GetElectricalDirection(
             MOTOR_ID_D, NEW_FRONT_VEHICLE_DIRECTION_FORWARD, &direction) != 0U) &&
          (direction == MOTOR_DIRECTION_FORWARD) &&
          (NewFrontDirectionMap_GetElectricalDirection(
             MOTOR_ID_C, NEW_FRONT_VEHICLE_DIRECTION_FORWARD, &direction) != 0U) &&
          (direction == MOTOR_DIRECTION_FORWARD) &&
          (NewFrontDirectionMap_GetEncoderLogicalSign(MOTOR_ID_B) == 1) &&
          (NewFrontDirectionMap_GetEncoderLogicalSign(MOTOR_ID_A) == -1) &&
          (NewFrontDirectionMap_GetEncoderLogicalSign(MOTOR_ID_D) == 1) &&
          (NewFrontDirectionMap_GetEncoderLogicalSign(MOTOR_ID_C) == -1)) ? 1U : 0U;
}
