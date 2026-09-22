#ifndef SOFT_I2C_H
#define SOFT_I2C_H

#include <stdint.h>
#include "stm32f1xx_hal.h"

typedef struct
{
  GPIO_TypeDef *port;
  uint16_t scl_pin;
  uint16_t sda_pin;
} SoftI2C_Bus_t;

void SoftI2C_Init(const SoftI2C_Bus_t *bus);
uint8_t SoftI2C_IsDeviceReady(const SoftI2C_Bus_t *bus,
                              uint8_t address7,
                              uint8_t trials);
uint8_t SoftI2C_WriteRegister(const SoftI2C_Bus_t *bus,
                              uint8_t address7,
                              uint8_t reg,
                              uint8_t value);
uint8_t SoftI2C_ReadRegister16(const SoftI2C_Bus_t *bus,
                               uint8_t address7,
                               uint8_t reg,
                               uint16_t *value);

#endif
