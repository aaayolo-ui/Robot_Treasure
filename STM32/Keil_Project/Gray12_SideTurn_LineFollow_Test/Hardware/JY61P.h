#ifndef JY61P_H
#define JY61P_H

#include <stdint.h>

typedef struct
{
  int16_t roll_x100;
  int16_t pitch_x100;
  int16_t yaw_x100;
  uint32_t frame_count;
  uint32_t checksum_error_count;
  uint32_t uart_error_count;
  uint32_t last_update_ms;
  uint8_t valid;
} JY61P_Data_t;

/*
 * Adapted for this car's STM32F103RCT6 HAL project:
 *   UART5_TX = PC12 -> JY61P RX
 *   UART5_RX = PD2  <- JY61P TX
 *   9600 baud, 8 data bits, no parity, 1 stop bit
 *
 * UART5 and its GPIO pins are initialized by main.c/HAL MSP. This driver only
 * owns reception, frame parsing and decoded JY61P data.
 */
void JY61P_Init(void);
void JY61P_Task(void);
void JY61P_ReceiveByte(uint8_t rx_data);
void JY61P_GetData(JY61P_Data_t *out);
uint8_t JY61P_IsValid(void);
uint32_t JY61P_GetFrameCount(void);
uint32_t JY61P_GetChecksumErrorCount(void);
uint32_t JY61P_GetUartErrorCount(void);

/* Compatibility APIs used by the current TFT and 90-degree turn controller. */
uint8_t JY61P_GetPitchX100(int16_t *pitch_x100);
uint8_t JY61P_GetYawX100(int16_t *yaw_x100);
uint32_t JY61P_GetLastUpdateMs(void);

#endif /* JY61P_H */
