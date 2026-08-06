#include "ChassisSpeedControl.h"
#include <stddef.h>

#define CHASSIS_SPEED_STALL_TARGET_RPM_X10 300L
#define CHASSIS_SPEED_STALL_RPM_X10         20L
#define CHASSIS_SPEED_STALL_TIMEOUT_MS      1500U

static WheelSpeedPi_t controllers[CHASSIS_WHEEL_COUNT];
static uint32_t stall_elapsed_ms[CHASSIS_WHEEL_COUNT];
static uint8_t speed_control_enabled;
static uint8_t speed_control_disabled;
static uint8_t stall_detected;
static ChassisWheelId_t stall_wheel;

static int64_t ChassisSpeedControl_Absolute(int32_t value)
{
  return (value < 0) ? -(int64_t)value : (int64_t)value;
}

static void ChassisSpeedControl_ResetAll(void)
{
  ChassisWheelId_t wheel;

  for (wheel = CHASSIS_WHEEL_FRONT_LEFT; wheel < CHASSIS_WHEEL_COUNT; wheel++)
  {
    WheelSpeedPi_Reset(&controllers[wheel]);
    stall_elapsed_ms[wheel] = 0U;
  }
}

void ChassisSpeedControl_Init(void)
{
  ChassisWheelId_t wheel;

  Chassis_Init();
  for (wheel = CHASSIS_WHEEL_FRONT_LEFT; wheel < CHASSIS_WHEEL_COUNT; wheel++)
  {
    WheelSpeedPi_Init(&controllers[wheel]);
    stall_elapsed_ms[wheel] = 0U;
  }
  speed_control_enabled = 0U;
  speed_control_disabled = 1U;
  stall_detected = 0U;
  stall_wheel = CHASSIS_WHEEL_COUNT;
}

void ChassisSpeedControl_Enable(void)
{
  ChassisWheelId_t wheel;

  ChassisSpeedControl_ResetAll();
  for (wheel = CHASSIS_WHEEL_FRONT_LEFT; wheel < CHASSIS_WHEEL_COUNT; wheel++)
  {
    WheelSpeedPi_SetEnabled(&controllers[wheel], 1U);
  }
  Chassis_Enable();
  speed_control_enabled = 1U;
  speed_control_disabled = 0U;
  stall_detected = 0U;
  stall_wheel = CHASSIS_WHEEL_COUNT;
}

void ChassisSpeedControl_Disable(void)
{
  ChassisWheelId_t wheel;

  ChassisSpeedControl_ResetAll();
  for (wheel = CHASSIS_WHEEL_FRONT_LEFT; wheel < CHASSIS_WHEEL_COUNT; wheel++)
  {
    WheelSpeedPi_SetEnabled(&controllers[wheel], 0U);
  }
  Chassis_Disable();
  speed_control_enabled = 0U;
  speed_control_disabled = 1U;
}

void ChassisSpeedControl_Stop(void)
{
  ChassisWheelId_t wheel;

  ChassisSpeedControl_ResetAll();
  for (wheel = CHASSIS_WHEEL_FRONT_LEFT; wheel < CHASSIS_WHEEL_COUNT; wheel++)
  {
    WheelSpeedPi_SetEnabled(&controllers[wheel], 0U);
  }
  Chassis_Stop();
  speed_control_enabled = 0U;
  speed_control_disabled = 0U;
}

void ChassisSpeedControl_SetWheelTargets(int32_t fl_rpm_x10,
                                          int32_t fr_rpm_x10,
                                          int32_t rl_rpm_x10,
                                          int32_t rr_rpm_x10)
{
  WheelSpeedPi_SetTarget(&controllers[CHASSIS_WHEEL_FRONT_LEFT], fl_rpm_x10);
  WheelSpeedPi_SetTarget(&controllers[CHASSIS_WHEEL_FRONT_RIGHT], fr_rpm_x10);
  WheelSpeedPi_SetTarget(&controllers[CHASSIS_WHEEL_REAR_LEFT], rl_rpm_x10);
  WheelSpeedPi_SetTarget(&controllers[CHASSIS_WHEEL_REAR_RIGHT], rr_rpm_x10);
}

void ChassisSpeedControl_SetMotionTarget(ChassisMotion_t motion,
                                         int32_t target_rpm_x10)
{
  int64_t magnitude = ChassisSpeedControl_Absolute(target_rpm_x10);
  int32_t target;

  if (magnitude > 2147483647LL)
  {
    target = 2147483647L;
  }
  else
  {
    target = (int32_t)magnitude;
  }

  switch (motion)
  {
    case CHASSIS_MOTION_FORWARD:
      ChassisSpeedControl_SetWheelTargets(target, target, target, target);
      break;
    case CHASSIS_MOTION_BACKWARD:
      ChassisSpeedControl_SetWheelTargets(-target, -target, -target, -target);
      break;
    case CHASSIS_MOTION_TURN_LEFT:
      ChassisSpeedControl_SetWheelTargets(-target, target, -target, target);
      break;
    case CHASSIS_MOTION_TURN_RIGHT:
      ChassisSpeedControl_SetWheelTargets(target, -target, target, -target);
      break;
    case CHASSIS_MOTION_STOP:
    default:
      ChassisSpeedControl_Stop();
      break;
  }
}

void ChassisSpeedControl_Update(uint32_t elapsed_ms)
{
  ChassisWheelId_t wheel;
  const ChassisWheelFeedback_t *feedback;
  int32_t output;

  if (elapsed_ms == 0U)
  {
    return;
  }

  Chassis_UpdateFeedback(elapsed_ms);
  for (wheel = CHASSIS_WHEEL_FRONT_LEFT; wheel < CHASSIS_WHEEL_COUNT; wheel++)
  {
    feedback = Chassis_GetWheelFeedback(wheel);
    if (speed_control_enabled == 0U)
    {
      /* Stopped/disabled: sample feedback, but do not run PI or drive PWM. */
      WheelSpeedPi_UpdateFeedback(&controllers[wheel], feedback->rpm_x10);
      continue;
    }
    output = WheelSpeedPi_Update(&controllers[wheel], feedback->rpm_x10, elapsed_ms);
    Chassis_SetWheelLogicalPwm(wheel, output);

    if ((ChassisSpeedControl_Absolute(WheelSpeedPi_GetTarget(&controllers[wheel])) >=
         (int64_t)CHASSIS_SPEED_STALL_TARGET_RPM_X10) &&
        (ChassisSpeedControl_Absolute(output) >= (int64_t)WHEEL_SPEED_PI_PWM_MAX) &&
        (ChassisSpeedControl_Absolute(feedback->rpm_x10) <
         (int64_t)CHASSIS_SPEED_STALL_RPM_X10))
    {
      stall_elapsed_ms[wheel] += elapsed_ms;
      if (stall_elapsed_ms[wheel] >= CHASSIS_SPEED_STALL_TIMEOUT_MS)
      {
        stall_wheel = wheel;
        stall_detected = 1U;
        ChassisSpeedControl_Stop();
        return;
      }
    }
    else
    {
      stall_elapsed_ms[wheel] = 0U;
    }
  }
}

const WheelSpeedPi_t *ChassisSpeedControl_GetController(ChassisWheelId_t wheel)
{
  if ((uint32_t)wheel >= (uint32_t)CHASSIS_WHEEL_COUNT)
  {
    return NULL;
  }
  return &controllers[wheel];
}

uint8_t ChassisSpeedControl_IsEnabled(void)
{
  return speed_control_enabled;
}

uint8_t ChassisSpeedControl_IsDisabled(void)
{
  return speed_control_disabled;
}

uint8_t ChassisSpeedControl_HasStall(void)
{
  return stall_detected;
}

ChassisWheelId_t ChassisSpeedControl_GetStallWheel(void)
{
  return stall_wheel;
}
