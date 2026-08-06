#include "App_PwmRpmTest.h"
#include "Encoder.h"
#include "Key.h"
#include "Motor.h"
#include "UartDebug.h"
#include "main.h"

#define PWM_TEST_MIN 0U
#define PWM_TEST_MAX 600U
#define PWM_TEST_STEP 50U
#define RPM_SAMPLE_MS 500U
#define PWM_TIMER_MAX 999U

static uint16_t test_pwm;
static uint32_t last_sample_tick;

static void App_PrintTestData(uint16_t pwm,
                              uint32_t elapsed_ms,
                              int16_t delta,
                              int32_t total,
                              int32_t rpm_x10)
{
  uint32_t pwm_percent_x10;
  int64_t rpm_value;
  int64_t rpm_abs;
  unsigned long pwm_percent_integer;
  unsigned long pwm_percent_decimal;
  long rpm_integer;
  long rpm_decimal;

  pwm_percent_x10 = ((uint32_t)pwm * 1000U + 499U) / PWM_TIMER_MAX;
  pwm_percent_integer = (unsigned long)(pwm_percent_x10 / 10U);
  pwm_percent_decimal = (unsigned long)(pwm_percent_x10 % 10U);
  rpm_value = (int64_t)rpm_x10;
  rpm_abs = (rpm_value < 0) ? -rpm_value : rpm_value;
  rpm_integer = (long)(rpm_abs / 10LL);
  rpm_decimal = (long)(rpm_abs % 10LL);

  if (rpm_value < 0)
  {
    UartDebug_Printf("PWM: %u/999 (%lu.%lu%%), dt: %lu ms, delta: %d, total: %ld, RPM: -%ld.%ld\r\n",
                     (unsigned int)pwm, pwm_percent_integer, pwm_percent_decimal,
                     (unsigned long)elapsed_ms, (int)delta, (long)total,
                     rpm_integer, rpm_decimal);
  }
  else
  {
    UartDebug_Printf("PWM: %u/999 (%lu.%lu%%), dt: %lu ms, delta: %d, total: %ld, RPM: %ld.%ld\r\n",
                     (unsigned int)pwm, pwm_percent_integer, pwm_percent_decimal,
                     (unsigned long)elapsed_ms, (int)delta, (long)total,
                     rpm_integer, rpm_decimal);
  }
}

void App_PwmRpmTest_Init(void)
{
  Key_Init();
  MotorA_Init();
  EncoderA_Init();
  test_pwm = PWM_TEST_MIN;
  last_sample_tick = HAL_GetTick();

  UartDebug_SendString("\r\n");
  UartDebug_SendString("Motor_A_PwmRpm_Test started.\r\n");
  UartDebug_SendString("A motor PWM-RPM manual test.\r\n");
  UartDebug_SendString("Encoder calibration: 1560 counts/rev\r\n");
  UartDebug_SendString("PWM range: 0 to 600\r\n");
  UartDebug_SendString("PWM step: 50\r\n");
  UartDebug_SendString("KEY1: increase PWM by 50 and run forward\r\n");
  UartDebug_SendString("KEY2: decrease PWM by 50\r\n");
  UartDebug_SendString("WK_UP: emergency stop and reset PWM to 0\r\n");
  UartDebug_SendString("Wait at least 3 seconds after each PWM change.\r\n");
  UartDebug_SendString("Record the last 3 stable RPM samples.\r\n");
  UartDebug_SendString("Wheels must remain lifted.\r\n");
}

void App_PwmRpmTest_Task(void)
{
  uint32_t now;
  uint32_t elapsed_ms;
  int16_t delta;
  int32_t total;
  int32_t rpm_x10;

  switch (Key_GetEvent())
  {
    case KEY_EVENT_KEY1:
      if (test_pwm < PWM_TEST_MAX)
      {
        test_pwm += PWM_TEST_STEP;
        MotorA_Forward(test_pwm);
        UartDebug_Printf("PWM increased: %u / 999\r\n", (unsigned int)test_pwm);
        UartDebug_SendString("Motor A forward.\r\n");
      }
      else
      {
        UartDebug_SendString("PWM maximum reached: 600 / 999\r\n");
      }
      break;
    case KEY_EVENT_KEY2:
      if (test_pwm > PWM_TEST_MIN)
      {
        test_pwm -= PWM_TEST_STEP;
        if (test_pwm > PWM_TEST_MIN)
        {
          MotorA_Forward(test_pwm);
          UartDebug_Printf("PWM decreased: %u / 999\r\n", (unsigned int)test_pwm);
          UartDebug_SendString("Motor A forward.\r\n");
        }
        else
        {
          MotorA_Stop();
          UartDebug_SendString("PWM decreased: 0 / 999\r\n");
        }
      }
      else
      {
        UartDebug_SendString("PWM already zero.\r\n");
      }
      break;
    case KEY_EVENT_WK_UP:
      MotorA_Stop();
      test_pwm = PWM_TEST_MIN;
      UartDebug_SendString("Emergency stop. PWM reset to 0.\r\n");
      break;
    case KEY_EVENT_NONE:
    default:
      break;
  }

  now = HAL_GetTick();
  if ((uint32_t)(now - last_sample_tick) >= RPM_SAMPLE_MS)
  {
    elapsed_ms = (uint32_t)(now - last_sample_tick);
    last_sample_tick = now;
    delta = EncoderA_GetDelta();
    total = EncoderA_GetTotal();
    rpm_x10 = EncoderA_CalculateRpmX10(delta, elapsed_ms);
    App_PrintTestData(test_pwm, elapsed_ms, delta, total, rpm_x10);
  }

  HAL_Delay(1U);
}
