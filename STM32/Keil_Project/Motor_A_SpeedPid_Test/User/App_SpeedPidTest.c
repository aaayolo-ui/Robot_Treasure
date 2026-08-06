#include "App_SpeedPidTest.h"
#include "Encoder.h"
#include "Key.h"
#include "Motor.h"
#include "SpeedPid.h"
#include "UartDebug.h"
#include "main.h"

#define SPEED_TARGET_MIN_RPM_X10       0L
#define SPEED_TARGET_MAX_RPM_X10       1500L
#define SPEED_TARGET_STEP_RPM_X10      300L

#define SPEED_CONTROL_PERIOD_MS        100U
#define SPEED_REPORT_PERIOD_MS         500U

#define SPEED_STALL_PWM_MIN            100U
#define SPEED_STALL_RPM_X10            20L
#define SPEED_STALL_TIMEOUT_MS         1500U

static int32_t target_rpm_x10;
static int32_t measured_rpm_x10;
static int16_t latest_delta;
static int32_t latest_total;
static uint16_t current_pwm;
static uint32_t last_control_tick;
static uint32_t last_report_tick;
static uint32_t latest_control_elapsed_ms;
static uint32_t stall_elapsed_ms;

static void App_SplitRpmX10(int32_t rpm_x10,
                            const char **sign,
                            long *integer,
                            long *decimal)
{
  int64_t value;
  int64_t absolute_value;

  value = (int64_t)rpm_x10;
  absolute_value = (value < 0) ? -value : value;
  *sign = (value < 0) ? "-" : "";
  *integer = (long)(absolute_value / 10LL);
  *decimal = (long)(absolute_value % 10LL);
}

static void App_PrintTargetMessage(const char *message)
{
  const char *sign;
  long integer;
  long decimal;

  App_SplitRpmX10(target_rpm_x10, &sign, &integer, &decimal);
  UartDebug_Printf("%s: %s%ld.%ld RPM\r\n", message, sign, integer, decimal);
}

static void App_PrintReport(void)
{
  const char *target_sign;
  const char *actual_sign;
  const char *error_sign;
  long target_integer;
  long target_decimal;
  long actual_integer;
  long actual_decimal;
  long error_integer;
  long error_decimal;

  App_SplitRpmX10(target_rpm_x10, &target_sign, &target_integer, &target_decimal);
  App_SplitRpmX10(measured_rpm_x10, &actual_sign, &actual_integer, &actual_decimal);
  App_SplitRpmX10(SpeedPid_GetErrorX10(), &error_sign, &error_integer, &error_decimal);

  UartDebug_Printf("Target: %s%ld.%ld RPM, Actual: %s%ld.%ld RPM, Error: %s%ld.%ld RPM, PWM: %u, FF: %ld, P: %ld, I: %ld, dt: %lu ms, delta: %d, total: %ld\r\n",
                   target_sign, target_integer, target_decimal,
                   actual_sign, actual_integer, actual_decimal,
                   error_sign, error_integer, error_decimal,
                   (unsigned int)current_pwm,
                   (long)SpeedPid_GetFeedforwardPwm(),
                   (long)SpeedPid_GetPTermPwm(),
                   (long)SpeedPid_GetITermPwm(),
                   (unsigned long)latest_control_elapsed_ms,
                   (int)latest_delta,
                   (long)latest_total);
}

static uint8_t App_IsRpmBelow(int32_t rpm_x10, int32_t limit_x10)
{
  int64_t absolute_rpm;

  absolute_rpm = (rpm_x10 < 0) ? -(int64_t)rpm_x10 : (int64_t)rpm_x10;
  return (absolute_rpm < (int64_t)limit_x10) ? 1U : 0U;
}

static void App_StopForStall(void)
{
  target_rpm_x10 = SPEED_TARGET_MIN_RPM_X10;
  current_pwm = 0U;
  SpeedPid_Reset();
  MotorA_Stop();
  stall_elapsed_ms = 0U;
  UartDebug_SendString("Stall or encoder fault detected. Motor stopped.\r\n");
}

static void App_ProcessKeys(void)
{
  switch (Key_GetEvent())
  {
    case KEY_EVENT_KEY1:
      if (target_rpm_x10 < SPEED_TARGET_MAX_RPM_X10)
      {
        if (target_rpm_x10 == SPEED_TARGET_MIN_RPM_X10)
        {
          SpeedPid_Reset();
          stall_elapsed_ms = 0U;
        }
        target_rpm_x10 += SPEED_TARGET_STEP_RPM_X10;
        App_PrintTargetMessage("Target speed increased");
      }
      else
      {
        UartDebug_SendString("Target speed already at maximum: 150.0 RPM\r\n");
      }
      break;

    case KEY_EVENT_KEY2:
      if (target_rpm_x10 > SPEED_TARGET_MIN_RPM_X10)
      {
        target_rpm_x10 -= SPEED_TARGET_STEP_RPM_X10;
        if (target_rpm_x10 == SPEED_TARGET_MIN_RPM_X10)
        {
          current_pwm = 0U;
          SpeedPid_Reset();
          MotorA_Stop();
          stall_elapsed_ms = 0U;
        }
        App_PrintTargetMessage("Target speed decreased");
      }
      else
      {
        UartDebug_SendString("Target speed already zero.\r\n");
      }
      break;

    case KEY_EVENT_WK_UP:
      target_rpm_x10 = SPEED_TARGET_MIN_RPM_X10;
      current_pwm = 0U;
      SpeedPid_Reset();
      MotorA_Stop();
      stall_elapsed_ms = 0U;
      UartDebug_SendString("Emergency stop. Target speed reset to 0.0 RPM.\r\n");
      break;

    case KEY_EVENT_NONE:
    default:
      break;
  }
}

void App_SpeedPidTest_Init(void)
{
  uint32_t now;

  Key_Init();
  MotorA_Init();
  EncoderA_Init();
  SpeedPid_Init();

  target_rpm_x10 = SPEED_TARGET_MIN_RPM_X10;
  measured_rpm_x10 = 0;
  latest_delta = 0;
  latest_total = 0;
  current_pwm = 0U;
  latest_control_elapsed_ms = 0U;
  stall_elapsed_ms = 0U;
  now = HAL_GetTick();
  last_control_tick = now;
  last_report_tick = now;

  UartDebug_SendString("\r\n");
  UartDebug_SendString("Motor_A_SpeedPid_Test started.\r\n");
  UartDebug_SendString("A motor closed-loop speed PID test.\r\n");
  UartDebug_SendString("Encoder calibration: 1560 counts/rev\r\n");
  UartDebug_SendString("Control period: approximately 100 ms\r\n");
  UartDebug_SendString("Report period: approximately 500 ms\r\n");
  UartDebug_SendString("Target range: 0 to 150 RPM\r\n");
  UartDebug_SendString("Target step: 30 RPM\r\n");
  UartDebug_SendString("PWM limit: 0 to 600\r\n");
  UartDebug_SendString("Reliable start PWM: 100\r\n");
  UartDebug_SendString("Minimum running PWM: 50\r\n");
  UartDebug_SendString("Initial Kp: 0.80\r\n");
  UartDebug_SendString("Initial Ki: 0.25\r\n");
  UartDebug_SendString("Initial Kd: 0.00\r\n");
  UartDebug_SendString("KEY1: increase target speed by 30 RPM\r\n");
  UartDebug_SendString("KEY2: decrease target speed by 30 RPM\r\n");
  UartDebug_SendString("WK_UP: emergency stop\r\n");
  UartDebug_SendString("Wheels must remain lifted.\r\n");
}

void App_SpeedPidTest_Task(void)
{
  uint32_t now;

  App_ProcessKeys();
  now = HAL_GetTick();

  if ((uint32_t)(now - last_control_tick) >= SPEED_CONTROL_PERIOD_MS)
  {
    latest_control_elapsed_ms = (uint32_t)(now - last_control_tick);
    last_control_tick = now;
    latest_delta = EncoderA_GetDelta();
    measured_rpm_x10 = EncoderA_CalculateRpmX10(latest_delta,
                                                  latest_control_elapsed_ms);
    latest_total = EncoderA_GetTotal();

    if (target_rpm_x10 > SPEED_TARGET_MIN_RPM_X10)
    {
      current_pwm = SpeedPid_Update(target_rpm_x10,
                                    measured_rpm_x10,
                                    latest_control_elapsed_ms);
      MotorA_SetForwardPwm(current_pwm);

      if ((current_pwm >= SPEED_STALL_PWM_MIN) &&
          (App_IsRpmBelow(measured_rpm_x10, SPEED_STALL_RPM_X10) != 0U))
      {
        stall_elapsed_ms += latest_control_elapsed_ms;
        if (stall_elapsed_ms >= SPEED_STALL_TIMEOUT_MS)
        {
          App_StopForStall();
        }
      }
      else
      {
        stall_elapsed_ms = 0U;
      }
    }
    else
    {
      SpeedPid_Reset();
      MotorA_Stop();
      current_pwm = 0U;
      stall_elapsed_ms = 0U;
    }
  }

  if ((uint32_t)(now - last_report_tick) >= SPEED_REPORT_PERIOD_MS)
  {
    last_report_tick = now;
    App_PrintReport();
  }
}
