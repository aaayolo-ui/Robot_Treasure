#ifndef NEW_FRONT_DRIVE_MAP_H
#define NEW_FRONT_DRIVE_MAP_H

#include "Chassis.h"
#include "Encoder.h"
#include "MotorDriver.h"

typedef struct
{
  MotorId_t motor;
  EncoderId_t encoder;
  MotorDirection_t vehicle_forward_electrical_direction;
  int8_t encoder_logical_sign;
} NewFrontDriveMap_t;

const NewFrontDriveMap_t *NewFrontDriveMap_Get(ChassisWheelId_t wheel);

#endif /* NEW_FRONT_DRIVE_MAP_H */
