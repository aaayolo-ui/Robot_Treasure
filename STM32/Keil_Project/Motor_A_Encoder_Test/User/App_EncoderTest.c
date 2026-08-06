#include "App_EncoderTest.h"
#include "Encoder.h"
#include "Key.h"
#include "Motor.h"
#include "UartDebug.h"
#include "main.h"

#define ENCODER_REPORT_PERIOD_MS 500U

static uint32_t encoder_last_report_tick;

static void App_PrintEncoderData(uint32_t elapsed_ms,
                                 int16_t delta,
                                 int32_t total,
                                 int32_t rpm_x10)
{
  int64_t rpm_value;
  int64_t rpm_abs;
  long rpm_integer;
  long rpm_decimal;

  rpm_value = (int64_t)rpm_x10;
  rpm_abs = (rpm_value < 0) ? -rpm_value : rpm_value;
  rpm_integer = (long)(rpm_abs / 10LL);
  rpm_decimal = (long)(rpm_abs % 10LL);

  if (rpm_value < 0)
  {
    UartDebug_Printf("Encoder dt: %lu ms, delta: %d, total: %ld, RPM: -%ld.%ld\r\n",
                     (unsigned long)elapsed_ms, (int)delta, (long)total,
                     rpm_integer, rpm_decimal);
  }
  else
  {
    UartDebug_Printf("Encoder dt: %lu ms, delta: %d, total: %ld, RPM: %ld.%ld\r\n",
                     (unsigned long)elapsed_ms, (int)delta, (long)total,
                     rpm_integer, rpm_decimal);
  }
}

void App_EncoderTest_Init(void)
{
  Key_Init();
  MotorA_Init();
  EncoderA_Init();
  encoder_last_report_tick = HAL_GetTick();

  UartDebug_SendString("\r\n");
  UartDebug_SendString("Motor_A_Encoder_Test started.\r\n");
  UartDebug_SendString("SYSCLK: 72 MHz\r\n");
  UartDebug_SendString("TIM1 CH1 PWM: 1 kHz\r\n");
  UartDebug_SendString("TIM3 Encoder: PA6 CH1, PA7 CH2\r\n");
  UartDebug_SendString("KEY1: forward 20 percent\r\n");
  UartDebug_SendString("KEY2: reverse 20 percent\r\n");
  UartDebug_SendString("WK_UP: stop\r\n");
  UartDebug_SendString("Encoder raw delta is printed every 500 ms.\r\n");
  UartDebug_SendString("Encoder calibration: 1560 counts/rev\r\n");
  UartDebug_SendString("RPM sample period: approximately 500 ms\r\n");
  UartDebug_SendString("RPM is calculated from the actual elapsed time.\r\n");
  UartDebug_SendString("RPM unit resolution: 0.1 RPM\r\n");
}

void App_EncoderTest_Task(void)
{
  uint32_t now;
  uint32_t elapsed_ms;
  int16_t delta;
  int32_t total;
  int32_t rpm_x10;

  switch (Key_GetEvent())
  {
    case KEY_EVENT_KEY1:
      MotorA_Forward(200U);
      UartDebug_SendString("Motor A forward: 20 percent\r\n");
      break;
    case KEY_EVENT_KEY2:
      MotorA_Reverse(200U);
      UartDebug_SendString("Motor A reverse: 20 percent\r\n");
      break;
    case KEY_EVENT_WK_UP:
      MotorA_Stop();
      UartDebug_SendString("Motor A stopped\r\n");
      break;
    case KEY_EVENT_NONE:
    default:
      break;
  }

  now = HAL_GetTick();
  if ((uint32_t)(now - encoder_last_report_tick) >= ENCODER_REPORT_PERIOD_MS)
  {
    elapsed_ms = (uint32_t)(now - encoder_last_report_tick);
    encoder_last_report_tick = now;
    delta = EncoderA_GetDelta();
    total = EncoderA_GetTotal();
    rpm_x10 = EncoderA_CalculateRpmX10(delta, elapsed_ms);
    App_PrintEncoderData(elapsed_ms, delta, total, rpm_x10);
  }

  HAL_Delay(1U);
}
