#include "LaserRx.h"
#include "UartCh4.h"
#include "AI1811Mux.h"
#include "ThiefStrategyConfig.h"
#include "main.h"
#include "UartDebug.h"
#include <stdio.h>
#include <string.h>
static AI1811Mux mux;
static uint8_t enabled;
static LaserRx_Handler application_handler;
static uint32_t boot_report_tick;
static uint8_t boot_report_sent;
static uint8_t boot_trace_next;
static uint32_t boot_trace_tick;
static uint8_t data_link_ready;
static uint8_t proof_count[LASER_COUNT];
static uint32_t proof_first[LASER_COUNT],proof_last[LASER_COUNT];

static void ProofReset(void)
{
  memset(proof_count,0,sizeof(proof_count));
  AI1811Mux_ResetStream(&mux);
}
static void ProofFrame(void *context,uint8_t channel,uint16_t distance,
                       uint8_t confidence,uint16_t firmware,uint32_t tick)
{
  (void)context;(void)distance;(void)confidence;(void)firmware;
  if(channel>=LASER_COUNT || !(THIEF_LASER_CHANNEL_MASK&(1U<<channel))) return;
  if(!proof_count[channel] || (uint32_t)(tick-proof_last[channel])>AI1811_MUX_FRESH_MS) {
    proof_count[channel]=0U;proof_first[channel]=tick;
  }
  if(proof_count[channel]<255U) proof_count[channel]++;
  proof_last[channel]=tick;
}
static void ProofByte(uint8_t byte,uint32_t tick)
{
  if((uint32_t)(HAL_GetTick()-tick)>AI1811_MUX_FRESH_MS) {ProofReset();return;}
  AI1811Mux_Byte(&mux,byte,tick,ProofFrame,0);
}

static void BootService(uint32_t baud,uint8_t channel)
{
  uint32_t now=HAL_GetTick();
  /* Configuration runs with motors disabled. Keep the independent Bluetooth
   * output draining while waiting for the expansion module, never in an ISR. */
  UartDebug_Task();
  if(!boot_report_sent || (uint32_t)(now-boot_report_tick)>=500U) {
    boot_report_sent=1U;boot_report_tick=now;
    if(channel==255U) UartDebug_SendString("UARTCH4_CONFIG STAGE=POWER_WAIT\r\n");
    else if(channel==254U) UartDebug_SendString("UARTCH4_CONFIG STAGE=VERIFY_FOUR_CHANNEL_DATA BAUD=115200\r\n");
    else UartDebug_Printf("UARTCH4_CONFIG HOST_BAUD=%lu COMMAND_CH=%u WAIT=OK,115200\r\n",
        (unsigned long)baud,(unsigned)channel);
  }
}

void LaserRx_Init(void)
{
  uint32_t started,now;uint8_t i,complete;
  enabled=0U;application_handler=0;AI1811Mux_Init(&mux);
  data_link_ready=0U;ProofReset();
  boot_report_sent=0U;boot_report_tick=0U;
  boot_trace_next=0U;boot_trace_tick=0U;
  /* Real RUN3 bytes already contained four valid channels at 115200, without
   * any documented text ACK. Prove the receive service directly before
   * changing saved module settings. Boot frames never reach the strategy. */
  if(UartCh4_OpenPassive()) {
    started=HAL_GetTick();
    while((uint32_t)(HAL_GetTick()-started)<1200U) {
      BootService(115200U,254U);
      if(UartCh4_TakeLost()) ProofReset();
      UartCh4_Poll(ProofByte);
      now=HAL_GetTick();complete=1U;
      for(i=0U;i<LASER_COUNT;i++) {
        if(!(THIEF_LASER_CHANNEL_MASK&(1U<<i)))continue;
        if(proof_count[i]<3U || (uint32_t)(proof_last[i]-proof_first[i])<20U ||
            (uint32_t)(now-proof_last[i])>AI1811_MUX_FRESH_MS) complete=0U;
      }
      if(complete) {data_link_ready=1U;break;}
    }
  }
  (void)UartCh4_SetReceiving(0U);
  if(data_link_ready) {
    UartDebug_Printf("UARTCH4_RX_PROOF SOURCE=FRAMES3 BAUD=115200 REQUIRED_MASK=15 VALID=%u,%u,%u,%u COMMAND_ACK=UNVERIFIED\r\n",
        (unsigned)proof_count[0],(unsigned)proof_count[1],
        (unsigned)proof_count[2],(unsigned)proof_count[3]);
  } else (void)UartCh4_Init(BootService);
  AI1811Mux_Init(&mux);
}
void LaserRx_BootLogTask(void)
{
  static const char hex[]="0123456789ABCDEF";
  static char line[240];
  const UartCh4_BootTrace *trace;
  uint32_t now=HAL_GetTick();
  uint8_t i;int length;
  if((uint32_t)(now-boot_trace_tick)<50U) return;
  trace=UartCh4_BootTraceAt(boot_trace_next);
  if(!trace) {
    /* A Bluetooth link may reconnect after boot. Repeat failed-init evidence
     * at a bounded rate while parked, without reconfiguring the hardware. */
    if(!UartCh4_Ready() && UartCh4_BootTraceCount()!=0U &&
        (uint32_t)(now-boot_trace_tick)>=15000U) boot_trace_next=0U;
    return;
  }
  length=snprintf(line,sizeof(line),
      "UARTCH4_TRY I=%u BAUD=%lu CH=%u CRLF=%u TX=%u ACK=%u RX=%lu ERR=%lu MASK=%lu MS=%lu HEX=",
      (unsigned)boot_trace_next,(unsigned long)trace->host_baud,(unsigned)trace->channel,
      (unsigned)trace->crlf,(unsigned)trace->tx_ok,(unsigned)trace->ack,
      (unsigned long)trace->rx_bytes,(unsigned long)trace->uart_errors,
      (unsigned long)trace->error_mask,(unsigned long)trace->elapsed_ms);
  if(length<=0 || (unsigned)length+2U*trace->sample_size+3U>=sizeof(line)) return;
  for(i=0U;i<trace->sample_size;i++) {
    line[length++]=hex[trace->sample[i]>>4];
    line[length++]=hex[trace->sample[i]&15U];
  }
  line[length++]='\r';line[length++]='\n';line[length]=0;
  if(UartDebug_TrySendString(line)) {boot_trace_next++;boot_trace_tick=now;}
}
uint8_t LaserRx_LinkReady(void) {return (uint8_t)(data_link_ready || UartCh4_Ready());}
const char *LaserRx_LinkSource(void) {return data_link_ready?"FRAMES3":(UartCh4_Ready()?"ACK":"NONE");}
uint8_t LaserRx_InitFault(void) {return data_link_ready?0U:UartCh4_Fault();}
uint8_t LaserRx_IsEnabled(void) {return enabled;}
uint8_t LaserRx_SetEnabled(uint8_t value)
{
  if(value && enabled) return 1U;
  enabled=0U;AI1811Mux_ResetStream(&mux);
  if(value && !LaserRx_LinkReady()) return 0U;
  if(!UartCh4_SetReceiving(value)) return 0U;
  enabled=value?1U:0U;return 1U;
}
void LaserRx_UartRxComplete(UART_HandleTypeDef *uart) {UartCh4_RxComplete(uart);}
void LaserRx_UartError(UART_HandleTypeDef *uart) {UartCh4_RxError(uart);}
static void Frame(void *context,uint8_t channel,uint16_t distance,uint8_t confidence,
                  uint16_t firmware,uint32_t tick)
{
  (void)context;
  if(application_handler) application_handler((LaserDirection_t)channel,distance,confidence,firmware,tick);
}
static void Byte(uint8_t byte,uint32_t tick)
{
  if((uint32_t)(HAL_GetTick()-tick)>AI1811_MUX_FRESH_MS) {AI1811Mux_ResetStream(&mux);return;}
  AI1811Mux_Byte(&mux,byte,tick,Frame,0);
}
void LaserRx_Poll(LaserRx_Handler handler)
{
  uint8_t i;
  if(!enabled) return;
  application_handler=handler;
  if(UartCh4_TakeLost()) {
    AI1811Mux_ResetStream(&mux);
    if(handler) for(i=0U;i<LASER_COUNT;i++) handler((LaserDirection_t)i,0U,0U,0U,HAL_GetTick());
  }
  UartCh4_Poll(Byte);
  application_handler=0;
}
void LaserRx_Counters(LaserDirection_t direction,uint32_t *bad,uint32_t *drop,uint32_t *errors)
{
  if(direction>=LASER_COUNT) return;
  if(bad) *bad=mux.bad[direction];
  UartCh4_Counters(drop,errors);
}
