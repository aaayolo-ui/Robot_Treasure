#include "UartDebug.h"
#include "main.h"
#include <stdarg.h>
#include <stdio.h>

#define UART_DEBUG_TX_BUFFER_SIZE 1024U

extern UART_HandleTypeDef huart3;

static uint8_t uart_debug_tx_buffer[UART_DEBUG_TX_BUFFER_SIZE];
static uint16_t uart_debug_tx_head;
static uint16_t uart_debug_tx_tail;

static void UartDebug_QueueByte(uint8_t value)
{
  uint16_t next = (uint16_t)(uart_debug_tx_head + 1U);

  if (next >= UART_DEBUG_TX_BUFFER_SIZE)
  {
    next = 0U;
  }
  if (next == uart_debug_tx_tail)
  {
    /* Debug text must never delay or stop vehicle control. Drop new bytes if
     * the optional diagnostic channel cannot keep up. */
    return;
  }
  uart_debug_tx_buffer[uart_debug_tx_head] = value;
  uart_debug_tx_head = next;
}

void UartDebug_SendString(const char *text)
{
  if (text != NULL)
  {
    while (*text != '\0')
    {
      UartDebug_QueueByte((uint8_t)*text);
      text++;
    }
  }
}

void UartDebug_Task(void)
{
  if ((uart_debug_tx_tail == uart_debug_tx_head) ||
      (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_TXE) == RESET))
  {
    return;
  }

  huart3.Instance->DR = uart_debug_tx_buffer[uart_debug_tx_tail];
  uart_debug_tx_tail++;
  if (uart_debug_tx_tail >= UART_DEBUG_TX_BUFFER_SIZE)
  {
    uart_debug_tx_tail = 0U;
  }
}

void UartDebug_Printf(const char *format, ...)
{
  char buffer[256];
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

  if ((size_t)length >= sizeof(buffer))
  {
    buffer[sizeof(buffer) - 3U] = '\r';
    buffer[sizeof(buffer) - 2U] = '\n';
  }
  buffer[sizeof(buffer) - 1U] = '\0';
  UartDebug_SendString(buffer);
}
