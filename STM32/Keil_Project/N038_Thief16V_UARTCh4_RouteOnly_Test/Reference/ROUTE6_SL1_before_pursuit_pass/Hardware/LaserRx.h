#ifndef LASER_RX_H
#define LASER_RX_H
#include <stdint.h>
#include "stm32f1xx_hal.h"
#define LASER_RX_POLL_BYTES_PER_CHANNEL 32U /* Bounded work after motion control. */
typedef enum { LASER_FRONT=0, LASER_LEFT, LASER_RIGHT, LASER_REAR, LASER_COUNT } LaserDirection_t;
typedef struct {
  uint16_t distance_mm, firmware;
  uint8_t confidence, received;
  uint32_t arrival_tick;
} LaserRx_Snapshot;
typedef void (*LaserRx_Handler)(LaserDirection_t direction, uint16_t distance,
    uint8_t confidence, uint16_t firmware, uint32_t arrival_tick);
void LaserRx_Init(void);
uint8_t LaserRx_LinkReady(void);
const char *LaserRx_LinkSource(void);
uint8_t LaserRx_InitFault(void);
void LaserRx_BootLogTask(void); /* Call only while parked in READY/FAULT. */
uint8_t LaserRx_SetEnabled(uint8_t enabled);
uint8_t LaserRx_IsEnabled(void);
void LaserRx_Poll(LaserRx_Handler handler);
/* Raw, checksum-verified data; does not imply a strategy obstacle. */
uint8_t LaserRx_GetSnapshot(LaserDirection_t direction,LaserRx_Snapshot *snapshot);
void LaserRx_Counters(LaserDirection_t direction, uint32_t *bad, uint32_t *dropped, uint32_t *errors);
void LaserRx_UartRxComplete(UART_HandleTypeDef *huart);
void LaserRx_UartError(UART_HandleTypeDef *huart);
#endif
