/* Computer-only fixtures for the documented module protocol; no real device. */
#include "UartCh4.h"
#include "LaserRx.h"
#include "AI1811Mux.h"
#include "main.h"
#include "UartDebug.h"
#include <string.h>
volatile unsigned test_checks,test_failures,test_first_line,test_done;
UART_HandleTypeDef huart1,huart3;
static uint32_t now,module_baud=9600U,rates[4];
static unsigned no_ack,require_crlf,new_baud_ack,fail_channel,fail_arm,tx_count;
static unsigned slow_host_busy;
static unsigned inject_bad_once;
static unsigned stream_mask,stream_bad,stream_limit,stream_groups,in_tick;
static uint32_t next_stream_tick;
static void PassiveStream(void);
static uint8_t *rx;
static unsigned received[4],invalid[4],last[4];
static unsigned boot_pumps,boot_lines;
#define CHECK(x) do {test_checks++;if(!(x)){test_failures++;if(!test_first_line)test_first_line=__LINE__;}} while(0)
uint32_t HAL_GetTick(void) {
  if(in_tick) return now; /* A short IRQ burst does not advance milliseconds per byte. */
  now++;
  if(!in_tick && stream_mask && rx && huart1.Init.BaudRate==115200U &&
      (int32_t)(now-next_stream_tick)>=0 && (!stream_limit || stream_groups<stream_limit)) {
    in_tick=1U;stream_groups++;next_stream_tick=now+50U;
    PassiveStream();in_tick=0U;
  }
  return now;
}
void UartDebug_Task(void) {boot_pumps++;}
void UartDebug_SendString(const char *text) {CHECK(text!=0);boot_lines++;}
void UartDebug_Printf(const char *format,...) {CHECK(format!=0);boot_lines++;}
uint8_t UartDebug_TrySendString(const char *text) {CHECK(text!=0);return 1U;}
void HAL_Delay(uint32_t ms) {now+=ms;}
HAL_StatusTypeDef HAL_UART_Init(UART_HandleTypeDef *u) {CHECK(u==&huart1);return HAL_OK;}
HAL_StatusTypeDef HAL_UART_DeInit(UART_HandleTypeDef *u) {CHECK(u==&huart1);return HAL_OK;}
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *u) {CHECK(u==&huart1);rx=0;return HAL_OK;}
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *u,uint8_t *p,uint16_t n) {
  CHECK(u==&huart1 && n==1U);
  if(fail_arm) {fail_arm=0;return HAL_ERROR;}
  u->ErrorCode=HAL_UART_ERROR_NONE; /* Match real HAL: a rearm clears errors. */
  rx=p;return HAL_OK;
}
static void Byte(uint8_t b) {CHECK(rx!=0);if(rx){*rx=b;UartCh4_RxComplete(&huart1);}}
static void PassiveStream(void) {
  /* Exact CRC-valid binary frames from the actual RUN3 boot trace samples. */
  static const uint8_t frames[4][8]={
    {0xAA,5,0x67,0,0x35,1,100,0xB0},
    {0xAA,5,0x68,0,0x7F,3,100,0xFD},
    {0xAA,5,0x68,0,0x49,1,100,0xC5},
    {0xAA,5,0x68,0,0x44,1,100,0xC0}};
  unsigned ch,i;
  for(ch=0U;ch<4U;ch++) if(stream_mask&(1U<<ch)) {
    Byte((uint8_t)('1'+ch));Byte('#');
    for(i=0U;i<8U;i++) Byte((uint8_t)(frames[ch][i]+((stream_bad && i==7U)?1U:0U)));
  }
}
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *u,const uint8_t *p,uint16_t n,uint32_t timeout) {
  char s[17];unsigned i,ch;uint32_t old=module_baud;
  CHECK(u==&huart1 && n<=16U && timeout>=100U);tx_count++;
  memcpy(s,p,n);s[n]=0;
  if(no_ack || u->Init.BaudRate!=module_baud) return HAL_OK;
  if(require_crlf && (n<12U || p[n-2U]!='\r' || p[n-1U]!='\n')) return HAL_OK;
  ch=0;
  for(i=0;i+10U<=n;i++) if(!memcmp(s+i,"CH",2U) && s[i+2U]>='0' && s[i+2U]<='4' && !memcmp(s+i+3U,"=115200",7U)) {ch=(unsigned)(s[i+2U]-'0');break;}
  CHECK(i+10U<=n);
  if(i+10U>n || (fail_channel && ch==fail_channel)) return HAL_OK;
  /* Simulate child-report traffic preventing an ACK over the slow host link. */
  if(slow_host_busy && ch && module_baud!=115200U) return HAL_OK;
  if(ch) rates[ch-1U]=115200U;
  else module_baud=115200U;
  if(inject_bad_once) {
    inject_bad_once=0U;u->ErrorCode=HAL_UART_ERROR_FE;
    Byte(0U);UartCh4_RxError(u);
  }
  if(ch || !new_baud_ack || old==module_baud) {
    static const char ack[]="OK,115200\r\n";
    for(i=0;i<sizeof(ack)-1U;i++) Byte((uint8_t)ack[i]);
  }
  return HAL_OK;
}
static void Handler(LaserDirection_t d,uint16_t mm,uint8_t confidence,uint16_t fw,uint32_t tick) {
  CHECK(d<LASER_COUNT && tick<=now);
  if(!mm) {invalid[d]++;return;}
  CHECK(confidence==50U && fw==0x2331U);
  received[d]++;last[d]=mm;
}
static void Frame(unsigned channel,unsigned mm,unsigned corrupt,unsigned ascii) {
  static const char hex[]="0123456789ABCDEF";
  uint8_t f[8]={0xAAU,5U,0x31U,0x23U,0U,0U,50U,0U};unsigned i,sum=0;
  f[4]=(uint8_t)mm;f[5]=(uint8_t)(mm>>8);
  for(i=0;i<7;i++)sum+=f[i];f[7]=(uint8_t)(sum+corrupt);
  Byte((uint8_t)('1'+channel));Byte('#');
  for(i=0;i<8;i++) {
    if(ascii) {Byte((uint8_t)hex[f[i]>>4]);Byte((uint8_t)hex[f[i]&15U]);Byte(' ');}
    else Byte(f[i]);
  }
}
void test_main(void) {
  unsigned i,before[4],total,pumps;uint32_t bad,drop,err;
  huart1.Instance=USART1;huart3.Instance=USART3;
  LaserRx_Init();CHECK(LaserRx_LinkReady() && !LaserRx_IsEnabled());
  CHECK(boot_pumps>0U && boot_lines>0U);pumps=boot_pumps;
  CHECK(module_baud==115200U && huart1.Init.BaudRate==115200U && huart1.Init.Mode==UART_MODE_TX_RX);
  for(i=0;i<4;i++)CHECK(rates[i]==115200U);
  CHECK(LaserRx_SetEnabled(1U));
  for(i=0;i<4;i++)Frame(i,300U+i,0U,0U);
  LaserRx_Poll(Handler);
  for(i=0;i<4;i++)CHECK(received[i]==1U && last[i]==300U+i);
  CHECK(boot_pumps==pumps); /* No boot logging/service in moving RX polling. */
  /* Literal 31 23 ('1#') in firmware bytes cannot switch a payload channel. */
  Frame(3,444U,0U,0U);LaserRx_Poll(Handler);CHECK(last[3]==444U && received[0]==1U);
  for(i=0;i<4;i++)Frame(i,500U+i,0U,1U);
  LaserRx_Poll(Handler);LaserRx_Poll(Handler);
  for(i=0;i<4;i++)CHECK(last[i]==500U+i);
  total=received[2];Frame(2,600U,1U,0U);LaserRx_Poll(Handler);
  CHECK(received[2]==total);LaserRx_Counters(LASER_RIGHT,&bad,&drop,&err);CHECK(bad==1U);
  Frame(2,601U,0U,0U);LaserRx_Poll(Handler);CHECK(last[2]==601U);
  total=received[1];Frame(1,650U,0U,0U);now+=251U;LaserRx_Poll(Handler);CHECK(received[1]==total);
  Byte('2');Byte('#');Byte(0xAA);Byte(5);CHECK(LaserRx_SetEnabled(1U));
  {uint8_t rest[]={0x31,0x23,0xBC,0x02,50,0};unsigned sum=0xAA+5+0x31+0x23+0xBC+2+50;
   rest[5]=(uint8_t)sum;for(i=0;i<6;i++)Byte(rest[i]);}
  LaserRx_Poll(Handler);CHECK(last[1]==700U);
  CHECK(LaserRx_SetEnabled(0U) && !LaserRx_IsEnabled());
  CHECK(LaserRx_SetEnabled(1U));
  for(i=0;i<4;i++)before[i]=received[i];
  for(i=0;i<20;i++)Frame(i%4U,800U+i,0U,0U);
  LaserRx_Poll(Handler);total=0;for(i=0;i<4;i++)total+=received[i]-before[i];CHECK(total==12U);
  LaserRx_Poll(Handler);total=0;for(i=0;i<4;i++)total+=received[i]-before[i];CHECK(total==20U);
  /* Shared overflow invalidates all directions and recovers at a new tag. */
  for(i=0;i<600;i++)Byte(0x55U);
  LaserRx_Poll(Handler);for(i=0;i<4;i++)CHECK(invalid[i]==1U);
  LaserRx_Counters(LASER_FRONT,&bad,&drop,&err);CHECK(drop>0U);
  Frame(0,999U,0U,0U);LaserRx_Poll(Handler);CHECK(last[0]==999U);
  huart1.ErrorCode=HAL_UART_ERROR_FE|HAL_UART_ERROR_ORE;
  Byte('O');
  CHECK(huart1.ErrorCode==(HAL_UART_ERROR_FE|HAL_UART_ERROR_ORE));
  UartCh4_RxError(&huart1);CHECK(huart1.ErrorCode==HAL_UART_ERROR_NONE);LaserRx_Poll(Handler);
  Frame(3,1000U,0U,0U);LaserRx_Poll(Handler);CHECK(last[3]==1000U);
  fail_arm=1U;Byte('4');LaserRx_Poll(Handler);now+=101U;LaserRx_Poll(Handler);
  Frame(3,1001U,0U,0U);LaserRx_Poll(Handler);CHECK(last[3]==1001U);
  /* Saved 115200 configuration and line-delimited firmware both boot. */
  CHECK(LaserRx_SetEnabled(0U));LaserRx_Init();CHECK(LaserRx_LinkReady());
  require_crlf=1U;module_baud=9600U;new_baud_ack=1U;
  LaserRx_Init();CHECK(LaserRx_LinkReady() && module_baud==115200U);
  module_baud=9600U;slow_host_busy=1U;
  LaserRx_Init();CHECK(LaserRx_LinkReady() && module_baud==115200U);
  CHECK(UartCh4_BootTraceCount()>0U && UartCh4_BootTraceCount()<=UART_CH4_BOOT_TRACES);
  {const UartCh4_BootTrace *t=UartCh4_BootTraceAt(0U);CHECK(t && t->channel==0U);}
  CHECK(UartCh4_BootTraceAt(UART_CH4_BOOT_TRACES)==0);
  LaserRx_BootLogTask();slow_host_busy=0U;
  inject_bad_once=1U;LaserRx_Init();CHECK(LaserRx_LinkReady());
  {unsigned saw=0U;
   for(i=0U;i<UartCh4_BootTraceCount();i++) {
     const UartCh4_BootTrace *t=UartCh4_BootTraceAt((uint8_t)i);
     if(t->error_mask&HAL_UART_ERROR_FE) {CHECK(t->uart_errors>0U && t->ack);saw=1U;}
   }
   CHECK(saw);}
  fail_channel=3U;LaserRx_Init();CHECK(!LaserRx_LinkReady() && LaserRx_InitFault()==5U);
  fail_channel=1U;LaserRx_Init();CHECK(!LaserRx_LinkReady() && LaserRx_InitFault()==7U);
  fail_channel=0U;no_ack=1U;total=now;pumps=boot_pumps;LaserRx_Init();
  CHECK(!LaserRx_LinkReady() && LaserRx_InitFault()==2U && now-total<10000U);
  CHECK(UartCh4_BootTraceCount()<=UART_CH4_BOOT_TRACES);
  {const UartCh4_BootTrace *t=UartCh4_BootTraceAt(0U);CHECK(t && t->tx_ok && !t->ack);}
  CHECK(boot_pumps>pumps+100U); /* The silent/no-ACK path still drains logs. */
  CHECK(!LaserRx_SetEnabled(1U));
  total=now;CHECK(!UartCh4_Init(0) && UartCh4_Fault()==2U && now-total<10000U);
  CHECK(tx_count>0U);
  /* No text ACK is needed when all four real packet formats are repeatedly
   * received and checked. No command may be sent on this passive path. */
  stream_mask=15U;stream_bad=stream_limit=stream_groups=0U;next_stream_tick=0U;
  total=tx_count;LaserRx_Init();
  CHECK(LaserRx_LinkReady() && LaserRx_InitFault()==0U && !LaserRx_IsEnabled());
  CHECK(strcmp(LaserRx_LinkSource(),"FRAMES4")==0 && tx_count==total);
  stream_mask=0U;CHECK(LaserRx_SetEnabled(1U));CHECK(LaserRx_SetEnabled(0U));
  stream_mask=7U;stream_groups=0U;next_stream_tick=0U;
  LaserRx_Init();CHECK(!LaserRx_LinkReady() && LaserRx_InitFault()==2U);
  stream_mask=15U;stream_bad=1U;stream_groups=0U;next_stream_tick=0U;
  LaserRx_Init();CHECK(!LaserRx_LinkReady());
  stream_bad=0U;stream_limit=1U;stream_groups=0U;next_stream_tick=0U;
  LaserRx_Init();CHECK(!LaserRx_LinkReady());
  stream_mask=0U;
  test_done=1U;while(1){}
}
