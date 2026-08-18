#include "UartDebug.h"
#include "main.h"
#include <string.h>

extern UART_HandleTypeDef huart3;

void UartDebug_SendString(const char *text)
{
  if (text != 0)
  {
    (void)HAL_UART_Transmit(&huart3, (uint8_t *)text,
                            (uint16_t)strlen(text), HAL_MAX_DELAY);
  }
}
