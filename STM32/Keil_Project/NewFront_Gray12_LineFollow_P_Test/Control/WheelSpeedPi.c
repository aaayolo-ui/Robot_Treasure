#include "WheelSpeedPi.h"
#include <limits.h>
#include <stddef.h>

static int64_t WheelSpeedPi_Absolute(int32_t value)
{
  return (value < 0) ? -(int64_t)value : (int64_t)value;
}

static int64_t WheelSpeedPi_LimitIntegral(int64_t integral)
{
  int64_t limit;

  limit = ((int64_t)WHEEL_SPEED_PI_I_TERM_LIMIT_PWM * 1000000LL) /
          (int64_t)WHEEL_SPEED_PI_KI_X100;
  if (integral > limit)
  {
    return limit;
  }
  if (integral < -limit)
  {
    return -limit;
  }
  return integral;
}

void WheelSpeedPi_Init(WheelSpeedPi_t *controller)
{
  if (controller == NULL)
  {
    return;
  }
  controller->enabled = 0U;
  WheelSpeedPi_Reset(controller);
}

void WheelSpeedPi_Reset(WheelSpeedPi_t *controller)
{
  if (controller == NULL)
  {
    return;
  }
  controller->target_rpm_x10 = 0;
  controller->measured_rpm_x10 = 0;
  controller->error_rpm_x10 = 0;
  controller->integral = 0;
  controller->output_pwm = 0;
  controller->saturated = 0U;
}

void WheelSpeedPi_SetEnabled(WheelSpeedPi_t *controller, uint8_t enabled)
{
  if (controller == NULL)
  {
    return;
  }
  controller->enabled = (enabled != 0U) ? 1U : 0U;
  if (controller->enabled == 0U)
  {
    WheelSpeedPi_Reset(controller);
  }
}

void WheelSpeedPi_SetTarget(WheelSpeedPi_t *controller, int32_t target_rpm_x10)
{
  if (controller == NULL)
  {
    return;
  }
  controller->target_rpm_x10 = target_rpm_x10;
  if (target_rpm_x10 == 0)
  {
    WheelSpeedPi_Reset(controller);
  }
}

void WheelSpeedPi_UpdateFeedback(WheelSpeedPi_t *controller,
                                 int32_t measured_rpm_x10)
{
  if (controller == NULL)
  {
    return;
  }

  controller->measured_rpm_x10 = measured_rpm_x10;
  controller->error_rpm_x10 = controller->target_rpm_x10 - measured_rpm_x10;
  controller->output_pwm = 0;
  controller->saturated = 0U;
}

int32_t WheelSpeedPi_Update(WheelSpeedPi_t *controller, int32_t measured_rpm_x10,
                            uint32_t elapsed_ms)
{
  int64_t target_magnitude;
  int64_t measured_in_direction;
  int64_t control_error;
  int64_t integral;
  int64_t output;
  int64_t candidate_integral;
  int64_t pwm_magnitude;
  int64_t direction;
  uint8_t integrate;

  if (controller == NULL)
  {
    return 0;
  }

  WheelSpeedPi_UpdateFeedback(controller, measured_rpm_x10);
  if ((controller->enabled == 0U) || (controller->target_rpm_x10 == 0))
  {
    WheelSpeedPi_Reset(controller);
    return 0;
  }

  direction = (controller->target_rpm_x10 < 0) ? -1LL : 1LL;
  target_magnitude = WheelSpeedPi_Absolute(controller->target_rpm_x10);
  measured_in_direction = (int64_t)measured_rpm_x10 * direction;
  control_error = target_magnitude - measured_in_direction;

  output = ((target_magnitude * 276LL + 500LL) / 1000LL) + 14LL +
           ((control_error * (int64_t)WHEEL_SPEED_PI_KP_X100) / 1000LL) +
           (((int64_t)controller->integral * (int64_t)WHEEL_SPEED_PI_KI_X100) /
            1000000LL);
  integrate = (elapsed_ms > 0U) ? 1U : 0U;
  if (((output >= (int64_t)WHEEL_SPEED_PI_PWM_MAX) && (control_error > 0LL)) ||
      ((output <= 0LL) && (control_error < 0LL)))
  {
    integrate = 0U;
  }
  if (integrate != 0U)
  {
    candidate_integral = (int64_t)controller->integral +
                         control_error * (int64_t)elapsed_ms;
    integral = WheelSpeedPi_LimitIntegral(candidate_integral);
    if (integral > (int64_t)INT32_MAX)
    {
      controller->integral = INT32_MAX;
    }
    else if (integral < (int64_t)INT32_MIN)
    {
      controller->integral = INT32_MIN;
    }
    else
    {
      controller->integral = (int32_t)integral;
    }
  }

  output = ((target_magnitude * 276LL + 500LL) / 1000LL) + 14LL +
           ((control_error * (int64_t)WHEEL_SPEED_PI_KP_X100) / 1000LL) +
           (((int64_t)controller->integral * (int64_t)WHEEL_SPEED_PI_KI_X100) /
            1000000LL);
  if (output <= 0LL)
  {
    pwm_magnitude = 0LL;
    controller->saturated = 1U;
  }
  else if (output >= (int64_t)WHEEL_SPEED_PI_PWM_MAX)
  {
    pwm_magnitude = (int64_t)WHEEL_SPEED_PI_PWM_MAX;
    controller->saturated = 1U;
  }
  else
  {
    pwm_magnitude = output;
  }

  if (pwm_magnitude > 0LL)
  {
    if (WheelSpeedPi_Absolute(measured_rpm_x10) <
        (int64_t)WHEEL_SPEED_PI_RUNNING_RPM_X10)
    {
      if (pwm_magnitude < (int64_t)WHEEL_SPEED_PI_PWM_START_MIN)
      {
        pwm_magnitude = (int64_t)WHEEL_SPEED_PI_PWM_START_MIN;
      }
    }
    else if (pwm_magnitude < (int64_t)WHEEL_SPEED_PI_PWM_RUNNING_MIN)
    {
      pwm_magnitude = (int64_t)WHEEL_SPEED_PI_PWM_RUNNING_MIN;
    }
  }

  controller->output_pwm = (int32_t)(direction * pwm_magnitude);
  return controller->output_pwm;
}

int32_t WheelSpeedPi_GetTarget(const WheelSpeedPi_t *controller)
{
  return (controller != NULL) ? controller->target_rpm_x10 : 0;
}

int32_t WheelSpeedPi_GetMeasured(const WheelSpeedPi_t *controller)
{
  return (controller != NULL) ? controller->measured_rpm_x10 : 0;
}

int32_t WheelSpeedPi_GetError(const WheelSpeedPi_t *controller)
{
  return (controller != NULL) ? controller->error_rpm_x10 : 0;
}

int32_t WheelSpeedPi_GetOutput(const WheelSpeedPi_t *controller)
{
  return (controller != NULL) ? controller->output_pwm : 0;
}
