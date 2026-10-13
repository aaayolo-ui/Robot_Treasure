#ifndef DRIVE_OUTPUT_SNAPSHOT_H
#define DRIVE_OUTPUT_SNAPSHOT_H

#include <stdint.h>

typedef struct
{
  uint32_t tim5_cr1, tim5_ccer, tim5_ccmr2, tim5_ccr3, tim5_arr;
  uint32_t usart2_cr1, afio_mapr, apb1_enr;
  uint8_t pa2_config, d_in1, d_in2, stby;
} DriveOutputSnapshot_t;

/* Passive register read. Does not alter pins, timers, encoders or motors. */
void DriveOutputSnapshot_Read(DriveOutputSnapshot_t *snapshot);

#endif
