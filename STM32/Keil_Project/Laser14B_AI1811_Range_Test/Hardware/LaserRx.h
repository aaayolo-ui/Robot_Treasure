#ifndef LASER_RX_H
#define LASER_RX_H
#include <stdint.h>
typedef void (*LaserRx_Handler)(uint16_t distance, uint8_t confidence, uint16_t firmware);
void LaserRx_Init(void);
void LaserRx_Poll(LaserRx_Handler handler);
void LaserRx_Counters(uint32_t *bad, uint32_t *dropped, uint32_t *errors);
#endif
