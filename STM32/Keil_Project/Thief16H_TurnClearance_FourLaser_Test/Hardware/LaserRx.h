#ifndef LASER_RX_H
#define LASER_RX_H
#include <stdint.h>
#include "stm32f1xx_hal.h"
typedef enum { LASER_FRONT=0, LASER_LEFT, LASER_RIGHT, LASER_REAR, LASER_COUNT } LaserDirection_t;
typedef void (*LaserRx_Handler)(LaserDirection_t direction, uint16_t distance,
    uint8_t confidence, uint16_t firmware, uint32_t arrival_tick);
void LaserRx_Init(void);
uint8_t LaserRx_SetEnabled(uint8_t enabled);
uint8_t LaserRx_IsEnabled(void);
void LaserRx_Poll(LaserRx_Handler handler);
void LaserRx_Counters(LaserDirection_t direction, uint32_t *bad, uint32_t *dropped, uint32_t *errors);
void LaserRx_UartRxComplete(UART_HandleTypeDef *huart);
void LaserRx_UartError(UART_HandleTypeDef *huart);
#endif
