#include "SpeedPid.h"

#define SPEED_PID_I_TERM_LIMIT_PWM 150L

static int64_t integral_error_x10_ms;
static int32_t previous_error_x10;
static int32_t error_x10;
static int32_t feedforward_pwm;
static int32_t p_term_pwm;
static int32_t i_term_pwm;
static int32_t d_term_pwm;

static int64_t SpeedPid_LimitIntegral(int64_t integral)
{
  int64_t integral_limit;

  if (SPEED_PID_KI_X100 == 0L)
  {
    return 0;
  }

  integral_limit = ((int64_t)SPEED_PID_I_TERM_LIMIT_PWM * 1000000LL) /
                   (int64_t)SPEED_PID_KI_X100;

  if (integral > integral_limit)
  {
    return integral_limit;
  }
  if (integral < -integral_limit)
  {
    return -integral_limit;
  }

  return integral;
}

static int32_t SpeedPid_CalculateITerm(int64_t integral)
{
  return (int32_t)((integral * (int64_t)SPEED_PID_KI_X100) / 1000000LL);
}

static uint16_t SpeedPid_LimitOutput(int64_t output)
{
  if (output <= 0LL)
  {
    return 0U;
  }
  if (output >= (int64_t)SPEED_PID_PWM_MAX)
  {
    return SPEED_PID_PWM_MAX;
  }

  return (uint16_t)output;
}

void SpeedPid_Init(void)
{
  SpeedPid_Reset();
}

void SpeedPid_Reset(void)
{
  integral_error_x10_ms = 0;
  previous_error_x10 = 0;
  error_x10 = 0;
  feedforward_pwm = 0;
  p_term_pwm = 0;
  i_term_pwm = 0;
  d_term_pwm = 0;
}

uint16_t SpeedPid_Update(int32_t target_rpm_x10,
                         int32_t measured_rpm_x10,
                         uint32_t elapsed_ms)
{
  int64_t output_before_integration;
  int64_t output;
  int64_t candidate_integral;
  int64_t measured_abs;
  uint16_t pwm;
  uint8_t integrate;

  if (target_rpm_x10 <= 0)
  {
    SpeedPid_Reset();
    return 0U;
  }

  error_x10 = target_rpm_x10 - measured_rpm_x10;
  feedforward_pwm = (int32_t)(((int64_t)target_rpm_x10 * 276LL + 500LL) /
                               1000LL + 14LL);
  p_term_pwm = (int32_t)(((int64_t)error_x10 *
                           (int64_t)SPEED_PID_KP_X100) / 1000LL);

  if (elapsed_ms > 0U)
  {
    d_term_pwm = (int32_t)(((int64_t)(error_x10 - previous_error_x10) *
                             (int64_t)SPEED_PID_KD_X100) /
                            (int64_t)elapsed_ms);
  }
  else
  {
    d_term_pwm = 0;
  }

  i_term_pwm = SpeedPid_CalculateITerm(integral_error_x10_ms);
  output_before_integration = (int64_t)feedforward_pwm +
                              (int64_t)p_term_pwm +
                              (int64_t)i_term_pwm +
                              (int64_t)d_term_pwm;
  integrate = (elapsed_ms > 0U) ? 1U : 0U;

  if (((output_before_integration >= (int64_t)SPEED_PID_PWM_MAX) &&
       (error_x10 > 0)) ||
      ((output_before_integration <= 0LL) && (error_x10 < 0)))
  {
    integrate = 0U;
  }

  if (integrate != 0U)
  {
    candidate_integral = integral_error_x10_ms +
                         (int64_t)error_x10 * (int64_t)elapsed_ms;
    integral_error_x10_ms = SpeedPid_LimitIntegral(candidate_integral);
  }

  i_term_pwm = SpeedPid_CalculateITerm(integral_error_x10_ms);
  output = (int64_t)feedforward_pwm + (int64_t)p_term_pwm +
           (int64_t)i_term_pwm + (int64_t)d_term_pwm;
  pwm = SpeedPid_LimitOutput(output);

  measured_abs = (measured_rpm_x10 < 0) ? -(int64_t)measured_rpm_x10 :
                                           (int64_t)measured_rpm_x10;
  if (measured_abs < (int64_t)SPEED_PID_RUNNING_RPM_X10)
  {
    if (pwm < SPEED_PID_PWM_START_MIN)
    {
      pwm = SPEED_PID_PWM_START_MIN;
    }
  }
  else if ((pwm > 0U) && (pwm < SPEED_PID_PWM_RUNNING_MIN))
  {
    pwm = SPEED_PID_PWM_RUNNING_MIN;
  }

  previous_error_x10 = error_x10;
  return pwm;
}

int32_t SpeedPid_GetErrorX10(void)
{
  return error_x10;
}

int32_t SpeedPid_GetFeedforwardPwm(void)
{
  return feedforward_pwm;
}

int32_t SpeedPid_GetPTermPwm(void)
{
  return p_term_pwm;
}

int32_t SpeedPid_GetITermPwm(void)
{
  return i_term_pwm;
}

int32_t SpeedPid_GetDTermPwm(void)
{
  return d_term_pwm;
}
