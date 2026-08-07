#ifndef __NCHD12_H
#define __NCHD12_H

#include "stm32f1xx_hal.h"

#define NCHD12_CHANNEL_COUNT        12U
#define NCHD12_GRAY12_MASK          0x0FFFU

typedef enum
{
    NCHD12_STATUS_OK = 0,
    NCHD12_STATUS_NO_DEVICE,
    NCHD12_STATUS_NOT_COMPATIBLE,
    NCHD12_STATUS_I2C_ERROR,
    NCHD12_STATUS_INVALID_ARGUMENT
} NCHD12_Status_t;

void NCHD12_Init(void);
NCHD12_Status_t NCHD12_ScanBus(void);
uint8_t NCHD12_GetDeviceCount(void);
uint8_t NCHD12_GetFoundAddress(uint8_t index, uint8_t *address7);
uint8_t NCHD12_HasSelectedDevice(void);
uint8_t NCHD12_GetSelectedAddress(void);

NCHD12_Status_t NCHD12_ConfigureInputs(void);
NCHD12_Status_t NCHD12_ReadRaw16(uint16_t *raw16);

uint16_t NCHD12_Extract12(uint16_t raw16);
uint8_t NCHD12_GetChannel(uint16_t gray12, uint8_t channel);
uint8_t NCHD12_CountBlack(uint16_t gray12);

NCHD12_Status_t NCHD12_GetLastStatus(void);
uint32_t NCHD12_GetLastUpdateMs(void);
uint32_t NCHD12_GetLastErrorMs(void);

#endif
