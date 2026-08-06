#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

#define MOTOR_ENCODER_COUNTS_PER_REV 1560L

typedef enum
{
  ENCODER_ID_A = 0,
  ENCODER_ID_B,
  ENCODER_ID_C,
  ENCODER_ID_D,
  ENCODER_ID_COUNT
} EncoderId_t;

void Encoder_InitAll(void);
void Encoder_Reset(EncoderId_t encoder);
void Encoder_ResetAll(void);

int16_t Encoder_GetDelta(EncoderId_t encoder);
int32_t Encoder_GetTotal(EncoderId_t encoder);

int32_t Encoder_CalculateRpmX10(EncoderId_t encoder,
                                int16_t delta_count,
                                uint32_t elapsed_ms);

#endif /* ENCODER_H */
