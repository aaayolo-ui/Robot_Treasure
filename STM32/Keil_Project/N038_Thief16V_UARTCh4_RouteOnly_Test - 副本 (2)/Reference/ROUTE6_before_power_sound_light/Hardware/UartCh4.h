#ifndef UART_CH4_H
#define UART_CH4_H
#include <stdint.h>
#include "stm32f1xx_hal.h"
#define UART_CH4_FIFO_SIZE 512U
#define UART_CH4_POLL_BUDGET 128U
#define UART_CH4_BOOT_TRACES 32U
#define UART_CH4_BOOT_SAMPLE 24U
typedef struct {
  uint32_t host_baud,elapsed_ms,rx_bytes,uart_errors;
  volatile uint32_t error_mask;
  uint8_t channel,crlf,tx_ok,ack,sample_size;
  uint8_t sample[UART_CH4_BOOT_SAMPLE];
} UartCh4_BootTrace;
typedef void (*UartCh4_ByteHandler)(uint8_t byte,uint32_t tick);
/* Boot-only, nonblocking service. Channel 255 means the power-on wait. */
typedef void (*UartCh4_BootService)(uint32_t host_baud,uint8_t channel);
/* Blocking configuration is boot-only, before manual start and motor enable. */
uint8_t UartCh4_Init(UartCh4_BootService service);
/* Open 115200 RX without claiming readiness or sending configuration. */
uint8_t UartCh4_OpenPassive(void);
uint8_t UartCh4_Ready(void);
uint8_t UartCh4_Fault(void);
uint8_t UartCh4_BootTraceCount(void);
const UartCh4_BootTrace *UartCh4_BootTraceAt(uint8_t index);
uint8_t UartCh4_SetReceiving(uint8_t value);
void UartCh4_Poll(UartCh4_ByteHandler handler);
void UartCh4_RxComplete(UART_HandleTypeDef *uart);
void UartCh4_RxError(UART_HandleTypeDef *uart);
uint8_t UartCh4_TakeLost(void);
void UartCh4_Counters(uint32_t *dropped,uint32_t *errors);
#endif
