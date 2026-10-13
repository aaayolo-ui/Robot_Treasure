#include "main.h"
#include "JY61P.h"
#include "LaserRx.h"

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == 0) return;
  if (huart->Instance == USART2) LaserRx_UartRxComplete(huart);
  else if (huart->Instance == UART5) JY61P_UartRxComplete(huart);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart == 0) return;
  if (huart->Instance == USART2) LaserRx_UartError(huart);
  else if (huart->Instance == UART5) JY61P_UartError(huart);
}

