#include "LaserRx.h"
#include "main.h"
#include <string.h>
#define FIFO_SIZE 256U
#define FRESH_MS 250U
#define POLL_BYTES_PER_CHANNEL 32U
static const IRQn_Type laser_irq[LASER_COUNT]={USART2_IRQn,USART1_IRQn,UART4_IRQn,USART3_IRQn};
extern UART_HandleTypeDef huart1, huart2, huart3, huart4;
typedef struct {
  volatile uint8_t fifo[FIFO_SIZE];
  volatile uint32_t tick[FIFO_SIZE];
  volatile uint16_t head, tail;
  volatile uint32_t dropped, errors;
  volatile uint8_t lost;
  uint32_t bad, last_tick;
  uint8_t rx, frame[8], count;
} LaserChannel;
static LaserChannel channels[LASER_COUNT];
static UART_HandleTypeDef *const ports[LASER_COUNT]={&huart2,&huart1,&huart4,&huart3};
static volatile uint8_t enabled;
static int Channel(UART_HandleTypeDef *uart)
{
  int i;
  for(i=0;i<LASER_COUNT;i++) if(uart==ports[i]) return i;
  return -1;
}
static void Arm(int i)
{
  if(enabled && HAL_UART_Receive_IT(ports[i],&channels[i].rx,1U)!=HAL_OK) {
    channels[i].errors++; channels[i].lost=1U;
  }
}
void LaserRx_Init(void)
{
  memset(channels,0,sizeof(channels));
  enabled=0U;
  (void)LaserRx_SetEnabled(1U);
}
uint8_t LaserRx_IsEnabled(void) { return enabled; }
uint8_t LaserRx_IsMotionIsolated(void)
{
  int i;
  if(enabled || !__HAL_RCC_USART1_IS_CLK_DISABLED() ||
     !__HAL_RCC_USART2_IS_CLK_DISABLED() || !__HAL_RCC_UART4_IS_CLK_DISABLED()) return 0U;
  if((USART3->CR1 & (USART_CR1_RE|USART_CR1_RXNEIE|USART_CR1_PEIE|USART_CR1_IDLEIE)) ||
     (USART3->CR3 & (USART_CR3_EIE|USART_CR3_DMAR))) return 0U;
  for(i=0;i<LASER_COUNT;i++) if(NVIC_GetEnableIRQ(laser_irq[i])) return 0U;
  return 1U;
}
uint8_t LaserRx_SetEnabled(uint8_t value)
{
  int i;
  uint8_t stopped=1U;
  if(value && enabled) return 1U;
  enabled=0U;
  for(i=0;i<LASER_COUNT;i++) {
    HAL_NVIC_DisableIRQ(laser_irq[i]);
    HAL_NVIC_ClearPendingIRQ(laser_irq[i]);
  }
  /* All detection is stationary, as in 16A. Gate UART4 as well: PC10 is encoder A.
   * Keep USART3 clock and TX alive for logs; abort only its RX transaction. */
  for(i=0;i<LASER_COUNT;i++) {
    (void)HAL_UART_AbortReceive(ports[i]);
    if(i!=LASER_REAR && HAL_UART_DeInit(ports[i])!=HAL_OK) stopped=0U;
    channels[i].head=0U; channels[i].tail=0U;
    channels[i].count=0U; channels[i].lost=0U;
  }
  __HAL_RCC_USART2_FORCE_RESET();
  __HAL_RCC_USART2_RELEASE_RESET();
  __HAL_RCC_UART4_FORCE_RESET();
  __HAL_RCC_UART4_RELEASE_RESET();
  __HAL_RCC_USART1_FORCE_RESET();
  __HAL_RCC_USART1_RELEASE_RESET();
  /* AbortReceive alone leaves RE enabled. Keep USART3 TX, but silence RX in hardware. */
  CLEAR_BIT(USART3->CR1, USART_CR1_RE);
  __HAL_UART_CLEAR_OREFLAG(&huart3);
  if(!value || !stopped) return (uint8_t)(stopped && LaserRx_IsMotionIsolated());
  for(i=0;i<LASER_COUNT;i++) {
    if(i!=LASER_REAR && HAL_UART_Init(ports[i])!=HAL_OK) {
      (void)LaserRx_SetEnabled(0U); return 0U;
    }
    __HAL_UART_CLEAR_OREFLAG(ports[i]);
  }
  SET_BIT(USART3->CR1, USART_CR1_RE);
  enabled=1U;
  for(i=0;i<LASER_COUNT;i++) {
    if(HAL_UART_Receive_IT(ports[i],&channels[i].rx,1U)!=HAL_OK) {
      (void)LaserRx_SetEnabled(0U); return 0U;
    }
  }
  for(i=0;i<LASER_COUNT;i++) HAL_NVIC_EnableIRQ(laser_irq[i]);
  return 1U;
}
void LaserRx_UartRxComplete(UART_HandleTypeDef *uart)
{
  int i=Channel(uart);
  uint16_t next;
  LaserChannel *c;
  if(i<0 || !enabled) return;
  c=&channels[i]; next=(uint16_t)((c->head+1U)%FIFO_SIZE);
  if(next==c->tail) { c->dropped++; c->lost=1U; }
  else {
    c->fifo[c->head]=c->rx; c->tick[c->head]=HAL_GetTick(); c->head=next;
  }
  Arm(i);
}
void LaserRx_UartError(UART_HandleTypeDef *uart)
{
  int i=Channel(uart);
  if(i<0 || !enabled) return;
  channels[i].errors++; channels[i].lost=1U;
  (void)HAL_UART_AbortReceive(uart);
  __HAL_UART_CLEAR_OREFLAG(uart);
  Arm(i);
}
static void Byte(LaserChannel *c, uint8_t b, uint32_t tick,
                 LaserDirection_t direction, LaserRx_Handler handler)
{
  uint8_t sum=0U, j, k;
  if((uint32_t)(tick-c->last_tick)>FRESH_MS) c->count=0U;
  c->last_tick=tick;
  if(!c->count) { if(b==0xAAU) c->frame[c->count++]=b; return; }
  if(c->count==1U && b!=0x05U) { c->count=(b==0xAAU)?1U:0U; return; }
  c->frame[c->count++]=b;
  if(c->count<8U) return;
  for(j=0;j<7U;j++) sum=(uint8_t)(sum+c->frame[j]);
  if(sum==c->frame[7]) {
    if(handler) handler(direction,(uint16_t)(c->frame[4]|((uint16_t)c->frame[5]<<8)),
      c->frame[6],(uint16_t)(c->frame[2]|((uint16_t)c->frame[3]<<8)),tick);
    c->count=0U;
  } else {
    c->bad++; c->count=0U;
    /* Keep the first possible header in the failed frame tail. */
    for(j=1U;j<8U;j++) if(c->frame[j]==0xAAU && (j==7U || c->frame[j+1U]==0x05U)) {
      c->count=(uint8_t)(8U-j);
      for(k=0U;k<c->count;k++) c->frame[k]=c->frame[j+k];
      break;
    }
  }
}
void LaserRx_Poll(LaserRx_Handler handler)
{
  int i;
  uint8_t b;
  uint16_t budget;
  uint32_t tick, irq;
  LaserChannel *c;
  if(!enabled) return;
  for(i=0;i<LASER_COUNT;i++) {
    c=&channels[i];
    if(c->lost) {
      irq=__get_PRIMASK(); __disable_irq();
      c->tail=c->head; c->lost=0U; c->count=0U;
      __set_PRIMASK(irq);
      if(handler) handler((LaserDirection_t)i,0U,0U,0U,HAL_GetTick());
    }
    budget=POLL_BYTES_PER_CHANNEL;
    while(c->tail!=c->head && budget--) {
      b=c->fifo[c->tail]; tick=c->tick[c->tail];
      c->tail=(uint16_t)((c->tail+1U)%FIFO_SIZE);
      if((uint32_t)(HAL_GetTick()-tick)>FRESH_MS) { c->count=0U; continue; }
      Byte(c,b,tick,(LaserDirection_t)i,handler);
    }
  }
}
void LaserRx_Counters(LaserDirection_t direction,uint32_t *bad,uint32_t *dropped,uint32_t *errors)
{
  if(direction>=LASER_COUNT) return;
  *bad=channels[direction].bad; *dropped=channels[direction].dropped; *errors=channels[direction].errors;
}
