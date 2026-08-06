#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

/*
 * Wheel output shaft calibration: 15584 counts over 10 forward revolutions
 * and 15566 counts over 10 reverse revolutions. The current calibration is
 * 1560 counts/rev. RPM calculation results use 0.1 RPM units.
 */
#define ENCODER_A_COUNTS_PER_REV 1560L

void EncoderA_Init(void);
int16_t EncoderA_GetDelta(void);
int32_t EncoderA_GetTotal(void);
void EncoderA_Reset(void);
int32_t EncoderA_CalculateRpmX10(int16_t delta_count, uint32_t elapsed_ms);

#endif /* ENCODER_H */
