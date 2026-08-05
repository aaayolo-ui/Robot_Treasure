#include "App_MotorTest.h"
#include "main.h"
#include "Key.h"
#include "Motor.h"
#include "UartDebug.h"

void App_MotorTest_Init(void)
{
  Key_Init();
  MotorA_Init();

  UartDebug_SendString("\r\n");
  UartDebug_SendString("Motor_A_Test started.\r\n");
  UartDebug_SendString("SYSCLK: 72 MHz\r\n");
  UartDebug_SendString("TIM1 CH1 PWM: 1 kHz\r\n");
  UartDebug_SendString("KEY1: forward 20 percent\r\n");
  UartDebug_SendString("KEY2: reverse 20 percent\r\n");
  UartDebug_SendString("WK_UP: stop\r\n");
}

void App_MotorTest_Task(void)
{
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

  HAL_Delay(5U);
}
