#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <stdint.h>

#define MOTOR_PWM_MAX 999U

typedef enum
{
  MOTOR_ID_A = 0,
  MOTOR_ID_B,
  MOTOR_ID_C,
  MOTOR_ID_D,
  MOTOR_ID_COUNT
} MotorId_t;

typedef enum
{
  MOTOR_DIRECTION_STOP = 0,
  MOTOR_DIRECTION_FORWARD,
  MOTOR_DIRECTION_REVERSE
} MotorDirection_t;

void MotorDriver_Init(void);

void MotorDriver_Enable(void);
void MotorDriver_Disable(void);
uint8_t MotorDriver_IsEnabled(void);

void Motor_SetPwm(MotorId_t motor, uint16_t pwm);
void Motor_Forward(MotorId_t motor, uint16_t pwm);
void Motor_Reverse(MotorId_t motor, uint16_t pwm);
void Motor_Stop(MotorId_t motor);
void Motor_StopAll(void);

uint16_t Motor_GetPwm(MotorId_t motor);
MotorDirection_t Motor_GetDirection(MotorId_t motor);

#endif /* MOTOR_DRIVER_H */
