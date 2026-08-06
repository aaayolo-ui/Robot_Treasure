#include "App_MotorA_MigrationTest.h"
#include "Encoder.h"
#include "Key.h"
#include "MotorDriver.h"
#include "UartDebug.h"
#include "main.h"

#define MIGRATION_TEST_PWM_MAX       200U
#define MIGRATION_TEST_PWM_STEP       50U
#define MIGRATION_REPORT_PERIOD_MS   500U

static uint16_t test_pwm;
static uint32_t last_report_tick;

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

static void App_ProcessKeys(void)
{
  switch (Key_GetEvent())
  {
    case KEY_EVENT_KEY1:
      if (test_pwm < MIGRATION_TEST_PWM_MAX)
      {
        if (test_pwm == 0U)
        {
          MotorDriver_Enable();
        }
        test_pwm += MIGRATION_TEST_PWM_STEP;
        Motor_Forward(MOTOR_ID_A, test_pwm);
        UartDebug_Printf("Motor A PWM increased: %u\r\n", (unsigned int)test_pwm);
      }
      else
      {
        UartDebug_SendString("Motor A PWM already at maximum: 200\r\n");
      }
      break;

    case KEY_EVENT_KEY2:
      if (test_pwm > 0U)
      {
        test_pwm -= MIGRATION_TEST_PWM_STEP;
        if (test_pwm == 0U)
        {
          Motor_Stop(MOTOR_ID_A);
        }
        else
        {
          Motor_Forward(MOTOR_ID_A, test_pwm);
        }
        UartDebug_Printf("Motor A PWM decreased: %u\r\n", (unsigned int)test_pwm);
      }
      else
      {
        UartDebug_SendString("Motor A PWM already zero.\r\n");
      }
      break;

    case KEY_EVENT_WK_UP:
      test_pwm = 0U;
      MotorDriver_Disable();
      UartDebug_SendString("Emergency stop. Motor driver disabled.\r\n");
      break;

    case KEY_EVENT_NONE:
    default:
      break;
  }
}

static void App_Report(void)
{
  uint32_t now;
  uint32_t elapsed_ms;
  int16_t delta;
  int32_t total;
  int32_t rpm_x10;
  const char *sign;
  long rpm_integer;
  long rpm_decimal;
  const char *driver_state;

  now = HAL_GetTick();
  if ((uint32_t)(now - last_report_tick) < MIGRATION_REPORT_PERIOD_MS)
  {
    return;
  }

  elapsed_ms = (uint32_t)(now - last_report_tick);
  last_report_tick = now;
  delta = Encoder_GetDelta(ENCODER_ID_A);
  total = Encoder_GetTotal(ENCODER_ID_A);
  rpm_x10 = Encoder_CalculateRpmX10(ENCODER_ID_A, delta, elapsed_ms);
  App_SplitRpmX10(rpm_x10, &sign, &rpm_integer, &rpm_decimal);
  driver_state = (MotorDriver_IsEnabled() != 0U) ? "ENABLED" : "DISABLED";

  UartDebug_Printf("Driver: %s, Motor A PWM: %u, dt: %lu ms, delta: %d, total: %ld, RPM: %s%ld.%ld\r\n",
                   driver_state,
                   (unsigned int)Motor_GetPwm(MOTOR_ID_A),
                   (unsigned long)elapsed_ms,
                   (int)delta,
                   (long)total,
                   sign,
                   rpm_integer,
                   rpm_decimal);
}

void App_MotorA_MigrationTest_Init(void)
{
  Key_Init();
  MotorDriver_Init();
  Encoder_InitAll();
  test_pwm = 0U;
  last_report_tick = HAL_GetTick();

  UartDebug_SendString("\r\n");
  UartDebug_SendString("Four_Motor_Base started.\r\n");
  UartDebug_SendString("Motor A migration test.\r\n");
  UartDebug_SendString("Debug UART: USART3 on PB10/PB11\r\n");
  UartDebug_SendString("Motor A PWM: TIM8_CH1 on PC6\r\n");
  UartDebug_SendString("Motor A encoder: TIM3 on PA6/PA7\r\n");
  UartDebug_SendString("Shared STBY: PC2\r\n");
  UartDebug_SendString("Encoder calibration: 1560 counts/rev\r\n");
  UartDebug_SendString("KEY1: increase Motor A PWM by 50\r\n");
  UartDebug_SendString("KEY2: decrease Motor A PWM by 50\r\n");
  UartDebug_SendString("WK_UP: disable the whole motor driver\r\n");
  UartDebug_SendString("PWM test limit: 200\r\n");
  UartDebug_SendString("Wheels must remain lifted.\r\n");
  UartDebug_SendString("Motor driver is disabled at startup.\r\n");
}

void App_MotorA_MigrationTest_Task(void)
{
  App_ProcessKeys();
  App_Report();
}
