#ifndef NEW_FRONT_DIRECTION_MAP_H
#define NEW_FRONT_DIRECTION_MAP_H

#include <stdint.h>
#include "MotorDriver.h"

typedef enum
{
  NEW_FRONT_VEHICLE_DIRECTION_FORWARD = 0,
  NEW_FRONT_VEHICLE_DIRECTION_REVERSE
} NewFrontVehicleDirection_t;

uint8_t NewFrontDirectionMap_GetElectricalDirection(
  MotorId_t motor,
  NewFrontVehicleDirection_t vehicle_direction,
  MotorDirection_t *electrical_direction);
int8_t NewFrontDirectionMap_GetEncoderLogicalSign(MotorId_t motor);
uint8_t NewFrontDirectionMap_RunSelfTest(void);

#endif /* NEW_FRONT_DIRECTION_MAP_H */
