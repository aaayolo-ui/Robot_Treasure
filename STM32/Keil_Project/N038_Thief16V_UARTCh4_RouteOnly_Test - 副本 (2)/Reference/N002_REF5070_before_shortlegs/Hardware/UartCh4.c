#include "UartCh4.h"
#include "main.h"
#include <string.h>
extern UART_HandleTypeDef huart1;
static volatile uint8_t bytes[UART_CH4_FIFO_SIZE];
static volatile uint32_t ticks[UART_CH4_FIFO_SIZE];
static volatile uint16_t head,tail;
static volatile uint32_t dropped,errors;
static volatile uint8_t receiving,lost,needs_arm;
static uint8_t rx,ready,fault;
static uint32_t command_tick,retry_tick;
static UartCh4_BootService boot_service;
static uint32_t boot_baud;
static uint8_t boot_channel;
static UartCh4_BootTrace boot_traces[UART_CH4_BOOT_TRACES];
static uint8_t boot_trace_count;
static volatile uint8_t active_trace=255U;
static const uint32_t bauds[]={115200U,9600U,19200U,14400U,4800U,2400U,1200U};
static const char *const commands[]={"CH1=115200","CH2=115200","CH3=115200","CH4=115200","CH0=115200"};

static void BootService(void)
{
  if(boot_service) boot_service(boot_baud,boot_channel);
}
static void BootWait(uint32_t duration)
{
  uint32_t start=HAL_GetTick();
  while((uint32_t)(HAL_GetTick()-start)<duration) BootService();
}

static void Flush(void)
{
  uint32_t irq=__get_PRIMASK();__disable_irq();
  tail=head;lost=0U;__set_PRIMASK(irq);
}
static uint8_t Arm(void)
{
  if(!receiving) return 1U;
  if(HAL_UART_Receive_IT(&huart1,&rx,1U)==HAL_OK) {needs_arm=0U;return 1U;}
  errors++;lost=needs_arm=1U;return 0U;
}
uint8_t UartCh4_SetReceiving(uint8_t value)
{
  receiving=needs_arm=0U;
  (void)HAL_UART_AbortReceive(&huart1);
  Flush();__HAL_UART_CLEAR_OREFLAG(&huart1);
  if(!value) return 1U;
  receiving=1U;
  if(!Arm()) {receiving=0U;return 0U;}
  return 1U;
}
static uint8_t HostBaud(uint32_t baud)
{
  boot_baud=baud;
  (void)UartCh4_SetReceiving(0U);
  if(HAL_UART_DeInit(&huart1)!=HAL_OK) return 0U;
  huart1.Instance=USART1;
  huart1.Init.BaudRate=baud;huart1.Init.WordLength=UART_WORDLENGTH_8B;
  huart1.Init.StopBits=UART_STOPBITS_1;huart1.Init.Parity=UART_PARITY_NONE;
  huart1.Init.Mode=UART_MODE_TX_RX;huart1.Init.HwFlowCtl=UART_HWCONTROL_NONE;
  huart1.Init.OverSampling=UART_OVERSAMPLING_16;
  if(HAL_UART_Init(&huart1)!=HAL_OK) return 0U;
  return UartCh4_SetReceiving(1U);
}
void UartCh4_RxComplete(UART_HandleTypeDef *uart)
{
  uint16_t next;
  if(uart!=&huart1 || !receiving) return;
  if(active_trace<UART_CH4_BOOT_TRACES) {
    boot_traces[active_trace].rx_bytes++;
    boot_traces[active_trace].error_mask|=uart->ErrorCode;
  }
  /* HAL calls byte-complete before its error callback. Rearming here on an
   * erroneous byte clears ErrorCode before HAL can handle ORE/FE/NE. Leave
   * recovery to RxError and never use such a byte as configuration evidence. */
  if(uart->ErrorCode!=HAL_UART_ERROR_NONE) return;
  next=(uint16_t)((head+1U)%UART_CH4_FIFO_SIZE);
  if(next==tail) {dropped++;lost=1U;}
  else {bytes[head]=rx;ticks[head]=HAL_GetTick();head=next;}
  (void)Arm();
}
void UartCh4_RxError(UART_HandleTypeDef *uart)
{
  if(uart!=&huart1 || !receiving) return;
  if(active_trace<UART_CH4_BOOT_TRACES)
    boot_traces[active_trace].error_mask|=uart->ErrorCode;
  errors++;lost=1U;
  (void)HAL_UART_AbortReceive(uart);__HAL_UART_CLEAR_OREFLAG(uart);(void)Arm();
}
static uint8_t Pop(uint8_t *b,uint32_t *tick)
{
  if(tail==head) return 0U;
  *b=bytes[tail];*tick=ticks[tail];tail=(uint16_t)((tail+1U)%UART_CH4_FIFO_SIZE);
  return 1U;
}
static uint8_t Send(uint8_t command,uint8_t crlf)
{
  char text[16];uint16_t n=0U,length=(uint16_t)strlen(commands[command]);
  boot_channel=(command==4U)?0U:(uint8_t)(command+1U);
  BootService();
  Flush();command_tick=HAL_GetTick();
  /* Clear an unterminated attempt before the line-delimited fallback. */
  if(crlf) {text[n++]='\r';text[n++]='\n';}
  memcpy(text+n,commands[command],length);n=(uint16_t)(n+length);
  if(crlf) {text[n++]='\r';text[n++]='\n';}
  if(HAL_UART_Transmit(&huart1,(uint8_t *)text,n,200U)!=HAL_OK) return 0U;
  if(active_trace<UART_CH4_BOOT_TRACES) boot_traces[active_trace].tx_ok=1U;
  return 1U;
}
static uint8_t Ack(uint32_t timeout)
{
  static const char token[]="OK,115200";
  uint32_t start=HAL_GetTick(),tick;uint8_t b,match=0U,budget;
  while((uint32_t)(HAL_GetTick()-start)<timeout) {
    BootService();
    budget=32U;
    while(budget-- && Pop(&b,&tick)) {
      if((int32_t)(tick-command_tick)<0) continue;
      if(active_trace<UART_CH4_BOOT_TRACES) {
        UartCh4_BootTrace *trace=&boot_traces[active_trace];
        if(trace->sample_size<UART_CH4_BOOT_SAMPLE) trace->sample[trace->sample_size++]=b;
      }
      match=(b==(uint8_t)token[match]) ? (uint8_t)(match+1U) : (b=='O'?1U:0U);
      if(match==sizeof(token)-1U) return 1U;
    }
  }
  return 0U;
}
static uint8_t Exchange(uint8_t command,uint8_t crlf,uint32_t timeout)
{
  uint8_t index=255U,ok;
  uint32_t start=HAL_GetTick(),before=errors;
  if(boot_trace_count<UART_CH4_BOOT_TRACES) {
    index=boot_trace_count++;
    memset(&boot_traces[index],0,sizeof(boot_traces[index]));
    boot_traces[index].host_baud=boot_baud;
    boot_traces[index].channel=(command==4U)?0U:(uint8_t)(command+1U);
    boot_traces[index].crlf=crlf;
  }
  active_trace=index;
  ok=(uint8_t)(Send(command,crlf) && Ack(timeout));
  active_trace=255U;
  if(index<UART_CH4_BOOT_TRACES) {
    boot_traces[index].ack=ok;
    boot_traces[index].uart_errors=errors-before;
    boot_traces[index].elapsed_ms=(uint32_t)(HAL_GetTick()-start);
  }
  return ok;
}
static uint8_t Configure(uint8_t command,uint8_t crlf)
{
  return Exchange(command,crlf,300U);
}
uint8_t UartCh4_OpenPassive(void)
{
  receiving=0U;
  head=tail=0U;dropped=errors=0U;lost=ready=fault=needs_arm=0U;
  retry_tick=command_tick=0U;
  active_trace=255U;boot_trace_count=0U;boot_service=0;
  boot_baud=115200U;boot_channel=254U;
  if(!HostBaud(115200U)) {fault=1U;return 0U;}
  return 1U;
}
uint8_t UartCh4_Init(UartCh4_BootService service)
{
  uint8_t i,e,found=0U,ending=0U,main_ack=0U;
  head=tail=0U;dropped=errors=0U;receiving=lost=ready=fault=needs_arm=0U;
  retry_tick=command_tick=0U;
  active_trace=255U;boot_trace_count=0U;
  boot_service=service;boot_baud=0U;boot_channel=255U;
  /* A bounded boot-only search accepts actual ACKs; successful transmission
   * alone is never treated as a configured expansion module. */
  BootWait(300U);
  for(i=0U;i<sizeof(bauds)/sizeof(bauds[0]) && !found;i++) {
    for(e=0U;e<2U;e++) {
      if(!HostBaud(bauds[i])) {fault=1U;goto done;}
      /* Raise the shared host link before enabling child sensor traffic.
       * An old-baud CH0 ACK is optional; a fresh CH1 ACK at 115200 proves
       * that the module actually switched, including new-baud ACK firmware. */
      BootWait(80U);
      if(Exchange(4U,e,120U)) main_ack=1U;
      if(!HostBaud(115200U)) {fault=1U;goto done;}
      BootWait(30U);
      if(Configure(0U,e)) {found=1U;ending=e;break;}
    }
  }
  if(!found) {fault=main_ack?7U:2U;goto done;}
  for(i=1U;i<4U;i++) {
    if(!Configure(i,ending) && !Configure(i,ending)) {fault=(uint8_t)(3U+i);goto done;}
  }
  ready=1U;
done:
  (void)UartCh4_SetReceiving(0U);
  BootService();boot_service=0;
  return ready;
}
uint8_t UartCh4_Ready(void) {return ready;}
uint8_t UartCh4_Fault(void) {return fault;}
uint8_t UartCh4_BootTraceCount(void) {return boot_trace_count;}
const UartCh4_BootTrace *UartCh4_BootTraceAt(uint8_t index)
{
  return index<boot_trace_count ? &boot_traces[index] : 0;
}
void UartCh4_Poll(UartCh4_ByteHandler handler)
{
  uint16_t budget=UART_CH4_POLL_BUDGET;uint8_t b;uint32_t tick;
  if(!receiving) return;
  if(needs_arm && (uint32_t)(HAL_GetTick()-retry_tick)>=100U) {
    retry_tick=HAL_GetTick();
    (void)HAL_UART_AbortReceive(&huart1);__HAL_UART_CLEAR_OREFLAG(&huart1);(void)Arm();
  }
  while(budget-- && Pop(&b,&tick)) {if(handler) handler(b,tick);}
}
uint8_t UartCh4_TakeLost(void)
{
  uint32_t irq;uint8_t value;
  irq=__get_PRIMASK();__disable_irq();value=lost;
  if(value) {tail=head;lost=0U;}
  __set_PRIMASK(irq);return value;
}
void UartCh4_Counters(uint32_t *drop,uint32_t *error)
{
  if(drop) *drop=dropped;if(error) *error=errors;
}
