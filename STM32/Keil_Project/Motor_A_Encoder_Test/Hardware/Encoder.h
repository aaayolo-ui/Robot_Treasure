#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

void EncoderA_Init(void);
int16_t EncoderA_GetDelta(void);
int32_t EncoderA_GetTotal(void);
void EncoderA_Reset(void);

#endif /* ENCODER_H */
