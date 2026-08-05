#include "UartDebug.h"
#include "main.h"
#include <string.h>

extern UART_HandleTypeDef huart1;

void UartDebug_SendString(const char *str)
{
  if (str != NULL)
  {
    (void)HAL_UART_Transmit(&huart1, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);
  }
}
