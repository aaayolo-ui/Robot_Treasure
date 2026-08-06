#include "SystemTime.h"
#include "stm32f1xx_hal.h"

uint32_t SystemTime_GetMs(void)
{
    return HAL_GetTick();
}
