#ifndef CHASSIS_H
#define CHASSIS_H

#include <stdint.h>

typedef enum
{
  CHASSIS_WHEEL_FRONT_LEFT = 0,
  CHASSIS_WHEEL_FRONT_RIGHT,
  CHASSIS_WHEEL_REAR_LEFT,
  CHASSIS_WHEEL_REAR_RIGHT,
  CHASSIS_WHEEL_COUNT
} ChassisWheelId_t;

typedef enum
{
  CHASSIS_MOTION_STOP = 0,
  CHASSIS_MOTION_FORWARD,
  CHASSIS_MOTION_BACKWARD,
  CHASSIS_MOTION_TURN_LEFT,
  CHASSIS_MOTION_TURN_RIGHT
} ChassisMotion_t;

typedef struct
{
  int16_t delta_count;
  int32_t total_count;
  int32_t rpm_x10;
} ChassisWheelFeedback_t;

void Chassis_Init(void);

void Chassis_Enable(void);
void Chassis_Disable(void);
uint8_t Chassis_IsEnabled(void);

void Chassis_Forward(uint16_t pwm);
void Chassis_Backward(uint16_t pwm);
void Chassis_TurnLeft(uint16_t pwm);
void Chassis_TurnRight(uint16_t pwm);
void Chassis_Stop(void);

/* logical_pwm is positive for vehicle-forward wheel rotation. */
void Chassis_SetWheelLogicalPwm(ChassisWheelId_t wheel, int32_t logical_pwm);

ChassisMotion_t Chassis_GetMotion(void);
uint16_t Chassis_GetPwm(void);

void Chassis_UpdateFeedback(uint32_t elapsed_ms);

const ChassisWheelFeedback_t *
Chassis_GetWheelFeedback(ChassisWheelId_t wheel);

int32_t Chassis_GetLeftAverageRpmX10(void);
int32_t Chassis_GetRightAverageRpmX10(void);

#endif /* CHASSIS_H */
