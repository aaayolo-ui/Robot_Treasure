#include "UartDebug.h"
#include "main.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

extern UART_HandleTypeDef huart1;

void UartDebug_SendString(const char *str)
{
  if (str != NULL)
  {
    (void)HAL_UART_Transmit(&huart1, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);
  }
}

void UartDebug_Printf(const char *format, ...)
{
  char buffer[192];
  va_list arguments;
  int length;

  if (format == NULL)
  {
    return;
  }

  va_start(arguments, format);
  length = vsnprintf(buffer, sizeof(buffer), format, arguments);
  va_end(arguments);

  if (length < 0)
  {
    return;
  }

  buffer[sizeof(buffer) - 1U] = '\0';
  UartDebug_SendString(buffer);
}
