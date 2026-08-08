#include "SystemTime.h"
#include "main.h"

uint32_t SystemTime_GetMs(void)
{
  return HAL_GetTick();
}
