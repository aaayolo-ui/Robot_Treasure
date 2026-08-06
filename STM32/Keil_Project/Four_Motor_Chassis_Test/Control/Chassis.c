#include "Chassis.h"
#include "Encoder.h"
#include "MotorDriver.h"
#include <limits.h>
#include <stddef.h>

#define ENCODER_A_DIRECTION_SIGN    -1
#define ENCODER_B_DIRECTION_SIGN     1
#define ENCODER_C_DIRECTION_SIGN    -1
#define ENCODER_D_DIRECTION_SIGN     1

typedef struct
{
  MotorId_t motor;
  EncoderId_t encoder;
  int8_t direction_sign;
} ChassisWheelConfig_t;

static const ChassisWheelConfig_t wheel_config[CHASSIS_WHEEL_COUNT] =
{
  {MOTOR_ID_D, ENCODER_ID_D, ENCODER_D_DIRECTION_SIGN},
  {MOTOR_ID_C, ENCODER_ID_C, ENCODER_C_DIRECTION_SIGN},
  {MOTOR_ID_B, ENCODER_ID_B, ENCODER_B_DIRECTION_SIGN},
  {MOTOR_ID_A, ENCODER_ID_A, ENCODER_A_DIRECTION_SIGN}
};

static ChassisWheelFeedback_t wheel_feedback[CHASSIS_WHEEL_COUNT];
static ChassisMotion_t chassis_motion;
static uint16_t chassis_pwm;

static int32_t Chassis_ApplySignInt32(int32_t value, int8_t sign)
{
  int64_t result;

  result = (int64_t)value * (int64_t)sign;
  if (result > (int64_t)INT32_MAX)
  {
    return INT32_MAX;
  }
  if (result < (int64_t)INT32_MIN)
  {
    return INT32_MIN;
  }
  return (int32_t)result;
}

static int16_t Chassis_ApplySignInt16(int16_t value, int8_t sign)
{
  int32_t result;

  result = (int32_t)value * (int32_t)sign;
  if (result > (int32_t)INT16_MAX)
  {
    return INT16_MAX;
  }
  if (result < (int32_t)INT16_MIN)
  {
    return INT16_MIN;
  }
  return (int16_t)result;
}

static uint8_t Chassis_CanStartMotion(void)
{
  return (chassis_motion == CHASSIS_MOTION_STOP) ? 1U : 0U;
}

static void Chassis_EnableIfNeeded(void)
{
  if (MotorDriver_IsEnabled() == 0U)
  {
    MotorDriver_Enable();
  }
}

void Chassis_Init(void)
{
  ChassisWheelId_t wheel;

  MotorDriver_Init();
  Encoder_InitAll();
  MotorDriver_Disable();
  chassis_motion = CHASSIS_MOTION_STOP;
  chassis_pwm = 0U;
  for (wheel = CHASSIS_WHEEL_FRONT_LEFT; wheel < CHASSIS_WHEEL_COUNT; wheel++)
  {
    wheel_feedback[wheel].delta_count = 0;
    wheel_feedback[wheel].total_count = 0;
    wheel_feedback[wheel].rpm_x10 = 0;
  }
}

void Chassis_Enable(void)
{
  Chassis_EnableIfNeeded();
}

void Chassis_Disable(void)
{
  MotorDriver_Disable();
  chassis_motion = CHASSIS_MOTION_STOP;
  chassis_pwm = 0U;
}

uint8_t Chassis_IsEnabled(void)
{
  return MotorDriver_IsEnabled();
}

void Chassis_Forward(uint16_t pwm)
{
  if (Chassis_CanStartMotion() == 0U)
  {
    return;
  }
  if (pwm == 0U)
  {
    Chassis_Stop();
    return;
  }

  Chassis_EnableIfNeeded();
  Motor_Forward(MOTOR_ID_D, pwm);
  Motor_Forward(MOTOR_ID_C, pwm);
  Motor_Reverse(MOTOR_ID_B, pwm);
  Motor_Reverse(MOTOR_ID_A, pwm);
  chassis_motion = CHASSIS_MOTION_FORWARD;
  chassis_pwm = Motor_GetPwm(MOTOR_ID_A);
}

void Chassis_Backward(uint16_t pwm)
{
  if (Chassis_CanStartMotion() == 0U)
  {
    return;
  }
  if (pwm == 0U)
  {
    Chassis_Stop();
    return;
  }

  Chassis_EnableIfNeeded();
  Motor_Reverse(MOTOR_ID_D, pwm);
  Motor_Reverse(MOTOR_ID_C, pwm);
  Motor_Forward(MOTOR_ID_B, pwm);
  Motor_Forward(MOTOR_ID_A, pwm);
  chassis_motion = CHASSIS_MOTION_BACKWARD;
  chassis_pwm = Motor_GetPwm(MOTOR_ID_A);
}

void Chassis_TurnLeft(uint16_t pwm)
{
  if (Chassis_CanStartMotion() == 0U)
  {
    return;
  }
  if (pwm == 0U)
  {
    Chassis_Stop();
    return;
  }

  Chassis_EnableIfNeeded();
  Motor_Reverse(MOTOR_ID_D, pwm);
  Motor_Forward(MOTOR_ID_B, pwm);
  Motor_Forward(MOTOR_ID_C, pwm);
  Motor_Reverse(MOTOR_ID_A, pwm);
  chassis_motion = CHASSIS_MOTION_TURN_LEFT;
  chassis_pwm = Motor_GetPwm(MOTOR_ID_A);
}

void Chassis_TurnRight(uint16_t pwm)
{
  if (Chassis_CanStartMotion() == 0U)
  {
    return;
  }
  if (pwm == 0U)
  {
    Chassis_Stop();
    return;
  }

  Chassis_EnableIfNeeded();
  Motor_Forward(MOTOR_ID_D, pwm);
  Motor_Reverse(MOTOR_ID_B, pwm);
  Motor_Reverse(MOTOR_ID_C, pwm);
  Motor_Forward(MOTOR_ID_A, pwm);
  chassis_motion = CHASSIS_MOTION_TURN_RIGHT;
  chassis_pwm = Motor_GetPwm(MOTOR_ID_A);
}

void Chassis_Stop(void)
{
  Motor_StopAll();
  chassis_motion = CHASSIS_MOTION_STOP;
  chassis_pwm = 0U;
}

ChassisMotion_t Chassis_GetMotion(void)
{
  return chassis_motion;
}

uint16_t Chassis_GetPwm(void)
{
  return chassis_pwm;
}

void Chassis_UpdateFeedback(uint32_t elapsed_ms)
{
  int16_t raw_delta[ENCODER_ID_COUNT];
  int32_t raw_total[ENCODER_ID_COUNT];
  int32_t raw_rpm_x10[ENCODER_ID_COUNT];
  ChassisWheelId_t wheel;

  raw_delta[ENCODER_ID_A] = Encoder_GetDelta(ENCODER_ID_A);
  raw_delta[ENCODER_ID_B] = Encoder_GetDelta(ENCODER_ID_B);
  raw_delta[ENCODER_ID_C] = Encoder_GetDelta(ENCODER_ID_C);
  raw_delta[ENCODER_ID_D] = Encoder_GetDelta(ENCODER_ID_D);
  raw_total[ENCODER_ID_A] = Encoder_GetTotal(ENCODER_ID_A);
  raw_total[ENCODER_ID_B] = Encoder_GetTotal(ENCODER_ID_B);
  raw_total[ENCODER_ID_C] = Encoder_GetTotal(ENCODER_ID_C);
  raw_total[ENCODER_ID_D] = Encoder_GetTotal(ENCODER_ID_D);
  raw_rpm_x10[ENCODER_ID_A] = Encoder_CalculateRpmX10(ENCODER_ID_A,
                                                        raw_delta[ENCODER_ID_A],
                                                        elapsed_ms);
  raw_rpm_x10[ENCODER_ID_B] = Encoder_CalculateRpmX10(ENCODER_ID_B,
                                                        raw_delta[ENCODER_ID_B],
                                                        elapsed_ms);
  raw_rpm_x10[ENCODER_ID_C] = Encoder_CalculateRpmX10(ENCODER_ID_C,
                                                        raw_delta[ENCODER_ID_C],
                                                        elapsed_ms);
  raw_rpm_x10[ENCODER_ID_D] = Encoder_CalculateRpmX10(ENCODER_ID_D,
                                                        raw_delta[ENCODER_ID_D],
                                                        elapsed_ms);

  for (wheel = CHASSIS_WHEEL_FRONT_LEFT; wheel < CHASSIS_WHEEL_COUNT; wheel++)
  {
    EncoderId_t encoder = wheel_config[wheel].encoder;
    int8_t sign = wheel_config[wheel].direction_sign;

    wheel_feedback[wheel].delta_count = Chassis_ApplySignInt16(raw_delta[encoder], sign);
    wheel_feedback[wheel].total_count = Chassis_ApplySignInt32(raw_total[encoder], sign);
    wheel_feedback[wheel].rpm_x10 = Chassis_ApplySignInt32(raw_rpm_x10[encoder], sign);
  }
}

const ChassisWheelFeedback_t *
Chassis_GetWheelFeedback(ChassisWheelId_t wheel)
{
  if ((uint32_t)wheel >= (uint32_t)CHASSIS_WHEEL_COUNT)
  {
    return NULL;
  }
  return &wheel_feedback[wheel];
}

int32_t Chassis_GetLeftAverageRpmX10(void)
{
  int64_t sum;

  sum = (int64_t)wheel_feedback[CHASSIS_WHEEL_FRONT_LEFT].rpm_x10 +
        (int64_t)wheel_feedback[CHASSIS_WHEEL_REAR_LEFT].rpm_x10;
  return (int32_t)(sum / 2LL);
}

int32_t Chassis_GetRightAverageRpmX10(void)
{
  int64_t sum;

  sum = (int64_t)wheel_feedback[CHASSIS_WHEEL_FRONT_RIGHT].rpm_x10 +
        (int64_t)wheel_feedback[CHASSIS_WHEEL_REAR_RIGHT].rpm_x10;
  return (int32_t)(sum / 2LL);
}
