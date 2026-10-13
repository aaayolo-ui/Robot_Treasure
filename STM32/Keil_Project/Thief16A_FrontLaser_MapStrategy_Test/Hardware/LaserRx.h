#ifndef LASER_RX_H
#define LASER_RX_H
#include <stdint.h>
#include "stm32f1xx_hal.h"
typedef void (*LaserRx_Handler)(uint16_t distance, uint8_t confidence, uint16_t firmware);
void LaserRx_Init(void);
/* USART2 is allowed only while stationary: PA2 is also TIM5_CH3 (motor D).
 * Call from the main loop before enabling chassis motion / after settling. */
uint8_t LaserRx_SetEnabled(uint8_t enabled);
uint8_t LaserRx_IsEnabled(void);
void LaserRx_Poll(LaserRx_Handler handler);
void LaserRx_Counters(uint32_t *bad, uint32_t *dropped, uint32_t *errors);
void LaserRx_UartRxComplete(UART_HandleTypeDef *huart);
void LaserRx_UartError(UART_HandleTypeDef *huart);
#endif
