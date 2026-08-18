#include "App_InfraredRawTest.h"
#include "UartDebug.h"
#include "main.h"

#define IR_RAW_REPORT_PERIOD_MS 200U

static uint32_t last_report_ms;

void App_InfraredRawTest_Init(void)
{
  last_report_ms = HAL_GetTick();
}

void App_InfraredRawTest_Task(void)
{
  GPIO_PinState raw_level;
  uint32_t now = HAL_GetTick();

  if ((uint32_t)(now - last_report_ms) < IR_RAW_REPORT_PERIOD_MS)
  {
    return;
  }

  last_report_ms = now;
  raw_level = HAL_GPIO_ReadPin(IR_OUT_GPIO_Port, IR_OUT_Pin);

  if (raw_level == GPIO_PIN_SET)
  {
    UartDebug_SendString("IR_RAW=1\r\n");
  }
  else
  {
    UartDebug_SendString("IR_RAW=0\r\n");
  }
}
