#ifndef NCHD12_REAR_H
#define NCHD12_REAR_H

#include <stdint.h>
#include "nchd12.h"

void NCHD12Rear_Init(void);
NCHD12_Status_t NCHD12Rear_ConfigureInputs(void);
NCHD12_Status_t NCHD12Rear_ReadRaw16(uint16_t *raw16);

#endif
