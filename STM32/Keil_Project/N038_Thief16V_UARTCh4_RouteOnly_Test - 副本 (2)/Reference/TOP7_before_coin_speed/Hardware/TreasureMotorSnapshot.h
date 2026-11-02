#ifndef TREASURE_MOTOR_SNAPSHOT_H
#define TREASURE_MOTOR_SNAPSHOT_H

#include <stdint.h>

/* The STM32F1 timer fields below are 16-bit hardware fields. Keeping only
 * their defined bits bounds a 96-record diagnostic buffer without clipping
 * the four signed RPM values. GPIO configuration nibbles are raw CRL/CRH.
 * Pins: RR/C PWM PA1, IN1 PC5, IN2 PB0, encoder PA15/PB3; RL/D PWM PA2.
 * outputs bits 0..4 = C_IN1,C_IN2,D_IN1,D_IN2,STBY.
 * inputs bits 0..1 = PA15,PB3; bits 4..6 = USART1/USART2/UART4 clock enable.
 * No actuator or counter writes occur here. */
typedef struct
{
  uint32_t apb1_enr;
  uint16_t tim5_ccr2, tim5_ccr3, tim5_arr, tim5_cr1, tim5_ccer;
  uint16_t tim5_ccmr1, tim5_ccmr2;
  uint16_t tim2_cnt, tim2_smcr, tim2_cr1, tim2_ccer;
  uint16_t usart1_cr1, usart2_cr1, uart4_cr1;
  uint8_t pa1_config, pa2_config, pa15_config, pb3_config;
  uint8_t pb7_config, pc11_config, outputs, inputs;
  uint16_t probe_samples, c_high, d_high, c_checked, c_bad, d_checked, d_bad;
  uint16_t phase_mask;
  uint8_t actual_outputs;
} TreasureMotorSnapshot_t;

typedef struct
{
  int32_t raw_a, raw_b, raw_c, raw_d;
  uint16_t pwm_a, pwm_b, pwm_c, pwm_d;
  uint8_t stby;
} TreasureEncoderMap_t;

typedef struct
{
  uint8_t pa10_config, pb15_config, din2_level, stby;
} TreasureDIn2PinMap_t;

/* Boot-only readback for this PA10-to-PB15 wiring comparison; no writes. */
void TreasureMotorSnapshot_ReadDIn2PinMap(TreasureDIn2PinMap_t *snapshot);

/* Main-loop polling only: no GPIO/timer configuration or actuator writes.
 * Checked PWM samples exclude edges and two milliseconds after CCR changes.
 * This is a digital readback probe, not an oscilloscope or a duty-cycle meter. */
void TreasureMotorSnapshot_Init(void);
void TreasureMotorSnapshot_Probe(uint32_t now);
void TreasureMotorSnapshot_ReadEncoderMap(TreasureEncoderMap_t *snapshot);

/* Passive main-loop register snapshot, not a simultaneous logic-analyser
 * sample. A gated UART clock is reported as CR1=0 and distinguished by the
 * saved clock bits (USART1 belongs to APB2, the other two belong to APB1). */
void TreasureMotorSnapshot_Read(TreasureMotorSnapshot_t *snapshot);

#endif
