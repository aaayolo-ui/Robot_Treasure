#include "App_EncoderTest.h"
#include "Encoder.h"
#include "Key.h"
#include "Motor.h"
#include "UartDebug.h"
#include "main.h"

#define ENCODER_REPORT_PERIOD_MS 500U

static uint32_t encoder_last_report_tick;

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
}

void App_EncoderTest_Task(void)
{
  int16_t delta;

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

  if ((HAL_GetTick() - encoder_last_report_tick) >= ENCODER_REPORT_PERIOD_MS)
  {
    encoder_last_report_tick = HAL_GetTick();
    delta = EncoderA_GetDelta();
    UartDebug_Printf("Encoder delta/500ms: %d, total: %ld\r\n",
                     (int)delta, (long)EncoderA_GetTotal());
  }

  HAL_Delay(1U);
}
