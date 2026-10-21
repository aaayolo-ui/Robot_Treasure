#include "LaserRx.h"
volatile unsigned test_checks,test_failures,test_done;
UART_HandleTypeDef huart1={(void*)1},huart2={(void*)2},huart3={(void*)3},huart4={(void*)4};
static UART_HandleTypeDef *ports[4]={&huart2,&huart1,&huart4,&huart3};
static uint8_t *buffers[5];
static unsigned now=100, received[4], last[4], deinit[5], initialized[5];
static unsigned id(UART_HandleTypeDef *p) {return (unsigned)p->Instance;}
HAL_StatusTypeDef HAL_UART_Init(UART_HandleTypeDef *p) {initialized[id(p)]++;return HAL_OK;}
HAL_StatusTypeDef HAL_UART_DeInit(UART_HandleTypeDef *p) {deinit[id(p)]++;return HAL_OK;}
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *p) {buffers[id(p)]=0;return HAL_OK;}
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *p,uint8_t *b,uint16_t n) {(void)n;buffers[id(p)]=b;return HAL_OK;}
uint32_t HAL_GetTick(void) {return now;}
static void check(unsigned ok) {test_checks++;if(!ok)test_failures++;}
static void handler(LaserDirection_t d,uint16_t distance,uint8_t confidence,uint16_t fw,uint32_t tick)
{
 if(!distance) return;
 check(confidence==20 && fw==123 && tick<=now);
 received[d]++;last[d]=distance;
}
static void byte(unsigned channel,unsigned value)
{
 UART_HandleTypeDef *p=ports[channel];
 check(buffers[id(p)]!=0);
 if(buffers[id(p)]) { *buffers[id(p)]=(uint8_t)value;LaserRx_UartRxComplete(p); }
}
static void frame(unsigned d,unsigned distance,unsigned corrupt)
{
 unsigned j,sum=0;
 unsigned char f[8]={0xAA,0x05,123,0,0,0,20,0};
 f[4]=distance&255;f[5]=distance>>8;
 for(j=0;j<7;j++)sum+=f[j];f[7]=(sum+corrupt)&255;
 for(j=0;j<8;j++)byte(d,f[j]);
}
void test_main(void)
{
 unsigned d,j,before;
 uint32_t bad,drop,errors;
 unsigned char frames[4][8];
 LaserRx_Init();check(!LaserRx_IsEnabled());
 for(d=0;d<4;d++) check(buffers[id(ports[d])]==0);
 check(LaserRx_SetEnabled(1));
 check(initialized[2]==0 && buffers[2]==0);
 for(d=1;d<4;d++) {
  unsigned sum=0;
  frames[d][0]=0xAA;frames[d][1]=5;frames[d][2]=123;frames[d][3]=0;
  frames[d][4]=100+d;frames[d][5]=0;frames[d][6]=20;
  for(j=0;j<7;j++)sum+=frames[d][j];frames[d][7]=sum&255;
 }
 for(j=0;j<8;j++)for(d=1;d<4;d++)byte(d,frames[d][j]);
 LaserRx_Poll(handler);
 for(d=1;d<4;d++)check(received[d]==1 && last[d]==100+d);
 frame(1,200,1);frame(1,201,0);frame(2,202,0);LaserRx_Poll(handler);
 check(received[1]==2 && last[1]==201);check(received[2]==2 && last[2]==202);
 LaserRx_Counters(LASER_LEFT,&bad,&drop,&errors);check(bad==1 && drop==0 && errors==0);
 /* Expired queued bytes cannot become a fresh observation. */
 before=received[1];frame(1,300,0);now+=251;LaserRx_Poll(handler);check(received[1]==before);
 frame(1,301,0);LaserRx_Poll(handler);check(last[1]==301);
 /* Overflow is isolated; subsequent clean frames recover. */
 for(j=0;j<300;j++)byte(3,0x12);
 frame(1,302,0);LaserRx_Poll(handler);check(last[1]==302);
 LaserRx_Counters(LASER_REAR,&bad,&drop,&errors);check(drop>0);
 frame(3,303,0);LaserRx_Poll(handler);check(last[3]==303);
 LaserRx_UartError(&huart4);LaserRx_Poll(handler);frame(2,304,0);LaserRx_Poll(handler);
 LaserRx_Counters(LASER_RIGHT,&bad,&drop,&errors);check(errors==1 && last[2]==304);
 /* Motion gating must never deinitialize USART3 (log TX). */
 byte(1,0xAA);check(LaserRx_SetEnabled(0));check(!LaserRx_IsEnabled());
 check(deinit[3]==0 && deinit[1]>0 && deinit[2]>0 && deinit[4]>0);
 check(LaserRx_SetEnabled(1));frame(1,305,0);LaserRx_Poll(handler);check(last[1]==305);
 /* Four simultaneous backlogs cannot make one poll monopolize control.
  * Ten frames=80bytes per channel. Each poll drains32bytes/four frames. */
 {
  unsigned baseline[4];
  check(LASER_RX_POLL_BYTES_PER_CHANNEL==32U);
  for(d=1;d<4;d++) {
   baseline[d]=received[d];
   for(j=0;j<10;j++) frame(d,400U+d,0);
  }
  LaserRx_Poll(handler);
  for(d=1;d<4;d++) check(received[d]==baseline[d]+4U);
  LaserRx_Poll(handler);
  for(d=1;d<4;d++) check(received[d]==baseline[d]+8U);
  LaserRx_Poll(handler);
  for(d=1;d<4;d++) check(received[d]==baseline[d]+10U && last[d]==400U+d);
  /* Enabled->enabled is a no-op: do not drop a partial frame on route start. */
  frame(1,450,0);check(LaserRx_SetEnabled(1));LaserRx_Poll(handler);check(last[1]==450);
 }
 /* Disabled front callbacks cannot arm RX or create observations. */
 LaserRx_UartRxComplete(&huart2);LaserRx_UartError(&huart2);
 LaserRx_Poll(handler);
 check(initialized[2]==0 && buffers[2]==0 && received[0]==0);
 check(initialized[1]>0 && initialized[4]>0 && deinit[3]==0);
 test_done=1;while(1){}
}
