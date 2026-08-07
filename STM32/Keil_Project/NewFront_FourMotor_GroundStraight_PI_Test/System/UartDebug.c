#include "UartDebug.h"
#include "main.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

extern UART_HandleTypeDef huart3;

void UartDebug_SendString(const char *text)
{
  if (text != NULL)
  {
    (void)HAL_UART_Transmit(&huart3,
                            (uint8_t *)text,
                            strlen(text),
                            HAL_MAX_DELAY);
  }
}

void UartDebug_Printf(const char *format, ...)
{
  char buffer[128];
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
