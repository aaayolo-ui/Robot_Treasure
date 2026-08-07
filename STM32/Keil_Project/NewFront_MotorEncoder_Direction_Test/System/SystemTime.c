#include "SystemTime.h"
#include "stm32f1xx_hal.h"

void SystemTime_Init(void)
{
}

uint32_t SystemTime_GetMs(void)
{
  return HAL_GetTick();
}
