/* CPU-only boundary test. All getters below are test doubles, never linked
 * into Keil firmware. The actual User logger and wheel map are compiled. */
#include "TreasureWheelLog.h"
#include "TreasureMotorSnapshot.h"
#include "ChassisSpeedControl.h"
#include "MotorDriver.h"
#include "NewFrontDriveMap.h"
#include "UartDebug.h"
#include <stdint.h>
#include <string.h>

volatile unsigned test_checks, test_failures, test_first_line, test_done;
volatile unsigned test_running, test_workspace_bytes, test_runs;
volatile unsigned test_uart_attempts, test_uart_accepted;
static WheelSpeedPi_t controllers[4];
static ChassisWheelFeedback_t feedback[4];
static uint32_t snapshot_reads, run_epoch;
static uint8_t stub_tag;
static uint32_t expected_tick[96];
static uint8_t expected_event[96], expected_tag[96];
static uint16_t expected_dt[96], current_dt;
static uint32_t expected_control_max;
static unsigned expected_count, run_accepted, record_cursor, line_cursor;
static unsigned retry_remaining, retry_seen, saw_end;
static char retry_line[2048];

#define check(x) check_at((x),__LINE__)
static void check_at(unsigned ok,unsigned line)
{
  test_checks++;
  if(!ok) { test_failures++;if(!test_first_line)test_first_line=line; }
}
static void feed(uint32_t tick,uint8_t tag)
{
  unsigned i;
  stub_tag=tag;
  TreasureWheelLog_GrayContext(tick,0x012U,0x345U,-220,430);
  for(i=0;i<4;i++) {
    controllers[i].target_rpm_x10=2000+(int32_t)i*100;
    feedback[i].rpm_x10=1000+(int32_t)i*100;
    feedback[i].delta_count=(int16_t)(i+1);
    feedback[i].total_count=(int32_t)(tick*10U+i);
  }
}
const WheelSpeedPi_t *ChassisSpeedControl_GetController(ChassisWheelId_t wheel)
{ check(wheel<4);return &controllers[(unsigned)wheel&3U]; }
const ChassisWheelFeedback_t *Chassis_GetWheelFeedback(ChassisWheelId_t wheel)
{ check(wheel<4);return &feedback[(unsigned)wheel&3U]; }
int32_t WheelSpeedPi_GetTarget(const WheelSpeedPi_t *pi)
{ return pi->target_rpm_x10; }
uint16_t Motor_GetPwm(MotorId_t motor)
{ check(motor<4);return (uint16_t)(70U+(unsigned)motor); }
MotorDirection_t Motor_GetDirection(MotorId_t motor)
{ check(motor<4);return (MotorDirection_t)(1U+((unsigned)motor&1U)); }
void TreasureMotorSnapshot_Read(TreasureMotorSnapshot_t *s)
{
  snapshot_reads++;
  memset(s,stub_tag,sizeof(*s));
}
static uint32_t number_base(const char *s,const char *key,unsigned base)
{
  const char *p=strstr(s,key);
  uint32_t value=0;
  unsigned digit;
  check(p!=0);
  if(!p)return 0xFFFFFFFFU;
  p+=strlen(key);
  while(*p) {
    if(*p>='0' && *p<='9')digit=(unsigned)(*p-'0');
    else if(*p>='A' && *p<='F')digit=(unsigned)(*p-'A'+10);
    else break;
    if(digit>=base)break;
    value=value*base+digit;p++;
  }
  return value;
}
static uint32_t number(const char *s,const char *key)
{ return number_base(s,key,10); }
static void expect(uint32_t tick,uint8_t event,uint8_t tag)
{
  check(expected_count<96U);
  if(expected_count<96U) {
    expected_tick[expected_count]=tick;
    expected_event[expected_count]=event;
    expected_tag[expected_count]=tag;
    expected_dt[expected_count]=current_dt;
    expected_count++;
  }
}
static void reset_output(void)
{
  expected_count=run_accepted=record_cursor=line_cursor=retry_seen=saw_end=0;
  retry_remaining=3;
  retry_line[0]=0;
  current_dt=0;expected_control_max=0;
}
static void control(uint32_t elapsed)
{
  TreasureWheelLog_ControlElapsed(elapsed);
  current_dt=(elapsed>65535U) ? 65535U : (uint16_t)elapsed;
  if(elapsed>expected_control_max)expected_control_max=elapsed;
}
static void accepted(const char *line)
{
  uint32_t tag;
  if(run_accepted==0U) {
    check(strncmp(line,"TW_BEGIN ",9)==0);
    check(number(line,"COUNT=")==expected_count);
    check(number(line,"RAM=")==test_workspace_bytes);
    check(number(line,"PRE_MAX=")==32 && number(line,"POST_MS=")==6000);
    check(number(line,"MAX_CONTROL_MS=")==expected_control_max);
  } else if(record_cursor==expected_count) {
    check(strcmp(line,"TW_END\r\n")==0);saw_end=1;
  } else {
    check(record_cursor<expected_count);
    check(number(line,"T=")==((uint32_t)(expected_tick[record_cursor]-run_epoch)));
    check(number(line,"EV=")==expected_event[record_cursor]);
    tag=expected_tag[record_cursor];
    if(line_cursor==0U) {
      check(strncmp(line,"TW ",3)==0);
      check(number(line,"S=")==2);
      check(number(line,"DT=")==expected_dt[record_cursor]);
      check(strstr(line,"FL=2000,1000,71,2,1 ")!=0);
      check(strstr(line,"FR=2100,1100,70,1,2 ")!=0);
      check(strstr(line,"RL=2200,1200,73,2,3 ")!=0);
      check(strstr(line,"RR=2300,1300,72,1,4\r\n")!=0);
    } else if(line_cursor==1U) {
      check(strncmp(line,"TW_PWM ",7)==0);
      check(number(line,"CCR2=")==tag*257U);
      check(number(line,"CCR3=")==tag*257U);
      check(number_base(line,"PA1=",16)==tag);
      check(number_base(line,"PA2=",16)==tag);
    } else if(line_cursor==2U) {
      check(strncmp(line,"TW_ENC ",7)==0);
      check(number(line,"CNT=")==tag*257U);
      check(number_base(line,"PA15=",16)==tag);
      check(number_base(line,"PB3=",16)==tag);
    } else if (line_cursor==3U) {
      check(strncmp(line,"TW_UART ",8)==0);
      check(number_base(line,"U1CR1=",16)==tag*257U);
      check(number_base(line,"U2CR1=",16)==tag*257U);
      check(number_base(line,"U4CR1=",16)==tag*257U);
      check(number_base(line,"APB1=",16)==tag*0x01010101U);
    } else {
      check(strncmp(line,"TW_PIN ",7)==0);
      check(number(line,"N=")==tag*257U);
      check(number(line,"CHECK_C=")==tag*257U);
      check(number(line,"BAD_D=")==tag*257U);
      check(strstr(line,"GRAY=012,345 POS=-220 HEAD=430 AGE=0\r\n")!=0);
    }
    line_cursor++;
    if(line_cursor==5U) {line_cursor=0;record_cursor++;}
  }
  run_accepted++;
}
uint8_t UartDebug_TrySendString(const char *line)
{
  unsigned n=(unsigned)strlen(line);
  test_uart_attempts++;
  check(!test_running);
  check(n>1 && n<sizeof(retry_line) && line[n-2]=='\r' && line[n-1]=='\n');
  if(retry_seen) check(strcmp(retry_line,line)==0);
  else { memcpy(retry_line,line,n+1U);retry_seen=1; }
  if(retry_remaining) { retry_remaining--;return 0; }
  accepted(line);
  test_uart_accepted++;
  retry_remaining=3;retry_seen=0;
  return 1;
}
static void task(uint32_t tick,uint8_t running,uint8_t tag)
{
  test_running=running;feed(tick,tag);
  TreasureWheelLog_Task(tick,2,running);
}
static void event(uint8_t kind,uint32_t tick,uint8_t tag)
{
  test_running=1;feed(tick,tag);
  TreasureWheelLog_Event(kind,tick,2);
}
static void drain(uint32_t tick,uint8_t tag)
{
  unsigned i;
  test_running=0;
  for(i=0;i<4096U && !saw_end;i++) task(tick+i,0,tag);
  check(saw_end);
  check(run_accepted==expected_count*5U+2U);
  check(record_cursor==expected_count);
  test_runs++;
}
void test_main(void)
{
  unsigned i,before;
  uint32_t epoch;
  TreasureWheelLog_Init();
  test_workspace_bytes=TreasureWheelLog_WorkBytes();
  check(test_workspace_bytes>0 && test_workspace_bytes<13U*1024U);
  /* No implicit run and no UART traffic at initialization. */
  task(100,0,0x44);check(test_uart_attempts==0 && snapshot_reads==0);

  /* Long approach: retain exactly the latest 32 pre-trigger samples, then
   * the actual before/after UART-enable snapshots and a full 6 s window. */
  reset_output();run_epoch=0;TreasureWheelLog_Begin(0);
  control(20);
  for(i=0;i<100U;i++)task(i*100U,1,0x10);
  for(i=68U;i<100U;i++)expect(i*100U,0,0x10);
  control(70000);event(1,10000,0x11);expect(10000,1,0x11);
  control(21);
  event(2,10001,0x22);expect(10001,2,0x22);
  for(i=101U;i<=160U;i++) {task(i*100U,1,0x20);expect(i*100U,0,0x20);}
  before=snapshot_reads;task(17000,1,0x77);
  check(snapshot_reads==before && test_uart_attempts==0);
  /* Starting again while pending must not replace the collected evidence. */
  TreasureWheelLog_Begin(17000);
  task(17010,0,0x44);expect(17010,4,0x44);
  drain(17011,0x55);

  /* Fault freezes the pre-brake state immediately. Do not overwrite it
   * with the subsequent stop, even if the fault occurs in the 6 s window. */
  reset_output();run_epoch=20000;TreasureWheelLog_Begin(20000);
  for(i=0;i<40U;i++)task(20000+i*100U,1,0x10);
  for(i=8;i<40U;i++)expect(20000+i*100U,0,0x10);
  event(1,24000,0x11);expect(24000,1,0x11);
  event(2,24001,0x22);expect(24001,2,0x22);
  task(24100,1,0x20);expect(24100,0,0x20);
  event(3,24150,0x33);expect(24150,3,0x33);
  before=snapshot_reads;task(24200,1,0x77);
  check(snapshot_reads==before);
  task(24300,0,0x44);expect(24300,4,0x44);
  drain(24301,0x55);

  /* Source-clock wrap must keep the whole elapsed-time window and markers. */
  reset_output();epoch=0xFFFFFF00U;run_epoch=epoch;TreasureWheelLog_Begin(epoch);
  for(i=0;i<4U;i++) {task(epoch+i*100U,1,0x10);expect(epoch+i*100U,0,0x10);}
  event(1,epoch+400U,0x11);expect(epoch+400U,1,0x11);
  event(2,epoch+401U,0x22);expect(epoch+401U,2,0x22);
  task(epoch+500U,1,0x20);expect(epoch+500U,0,0x20);
  task(epoch+600U,0,0x44);expect(epoch+600U,4,0x44);
  drain(epoch+601U,0x55);

  /* A route stopped without ever enabling lasers still retains a bounded
   * rolling history rather than only the first seconds of the drive. */
  reset_output();run_epoch=30000;TreasureWheelLog_Begin(30000);
  for(i=0;i<120U;i++)task(30000+i*100U,1,0x10);
  for(i=25U;i<120U;i++)expect(30000+i*100U,0,0x10);
  task(42000,0,0x44);expect(42000,4,0x44);
  drain(42001,0x55);
  before=test_uart_attempts;task(50000,0,0x55);
  check(test_uart_attempts==before);

  /* Even after the six-second sample window is frozen, the reserved fault
   * slot must retain the state before braking, followed by a distinct stop. */
  reset_output();run_epoch=60000;TreasureWheelLog_Begin(60000);
  for(i=0;i<40U;i++)task(60000+i*100U,1,0x10);
  for(i=8;i<40U;i++)expect(60000+i*100U,0,0x10);
  event(1,64000,0x11);expect(64000,1,0x11);
  event(2,64001,0x22);expect(64001,2,0x22);
  /* Duplicate events must neither restart the window nor consume slots. */
  before=snapshot_reads;
  event(1,64002,0x88);event(2,64003,0x88);event(0,64004,0x88);event(4,64005,0x88);
  check(snapshot_reads==before);
  for(i=641U;i<=700U;i++) {task(i*100U,1,0x20);expect(i*100U,0,0x20);}
  before=snapshot_reads;task(70100,1,0x77);check(snapshot_reads==before);
  event(3,70200,0x33);expect(70200,3,0x33);
  before=snapshot_reads;event(3,70210,0x88);task(70300,1,0x77);
  check(snapshot_reads==before);
  task(70400,0,0x44);expect(70400,4,0x44);
  drain(70401,0x55);
  test_running=0;test_done=1;while(1){}
}
