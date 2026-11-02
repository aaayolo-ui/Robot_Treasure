#include "App_LaserRangeTest.h"
#include "LaserRx.h"
#include "MotorDriver.h"
#include "UartDebug.h"
#include "main.h"
#include <string.h>

/* Actual ruler distances, FRONT/LEFT/RIGHT/REAR. Zero means not specified.
 * These references only calculate error; received distances are never scaled. */
static const uint16_t actual_mm[LASER_COUNT] = {0U, 0U, 0U, 0U};
#define REPORT_SLOT_MS 25U /* Four channels, each uploaded every 100 ms. */
#define STATS_PERIOD_MS 1000U
#define STALE_MS 500U

typedef struct {
  uint16_t raw, firmware, minimum, maximum;
  uint8_t confidence, seen;
  uint32_t tick, count, sum, rejected, frames, window_tick;
} RangeSample;
static RangeSample samples[LASER_COUNT];
static const char *const names[LASER_COUNT] = {"FRONT", "LEFT", "RIGHT", "REAR"};
static uint8_t next_channel, running;
static uint32_t last_report, last_retry;

static void RangeFrame(LaserDirection_t direction, uint16_t distance,
    uint8_t confidence, uint16_t firmware, uint32_t tick)
{
  RangeSample *s;
  if(direction>=LASER_COUNT) return;
  s=&samples[direction];
  s->raw=distance;s->confidence=confidence;s->firmware=firmware;
  s->tick=tick;s->seen=1U;
  /* FIFO loss is reported by LaserRx as a synthetic zero frame, not a
   * measurement. It invalidates the latest value but must not inflate Hz. */
  if(!firmware && !distance && !confidence) return;
  s->frames++;
  /* Keep zero/confidence-zero readings visible, but out of the average.
   * No strategy range threshold, road gate, or 0.9 correction is applied. */
  if(distance==0U || confidence==0U) {s->rejected++;return;}
  if(s->count==0U || distance<s->minimum) s->minimum=distance;
  if(s->count==0U || distance>s->maximum) s->maximum=distance;
  s->count++;s->sum+=distance;
}

static void StartReceiving(void)
{
  LaserRx_Init();
  running=LaserRx_LinkReady() && LaserRx_SetEnabled(1U);
  last_retry=HAL_GetTick();
  UartDebug_Printf("RANGE_BOOT READY=%u LINK=%s FAULT=%u MOTORS=OFF RAW_SCALE=1\r\n",
      (unsigned)running,LaserRx_LinkSource(),(unsigned)LaserRx_InitFault());
}

void App_LaserRangeTest_Init(void)
{
  uint8_t i;
  MotorDriver_Init();
  MotorDriver_Disable();
  memset(samples,0,sizeof(samples));
  next_channel=0U;running=0U;
  UartDebug_SendString("BOOT FW=LASER_RANGE_CAL_3 MOTORS=OFF LOG=PB10_115200 LASER=PB6_PB7 DISPLAY_HZ=10\r\n");
  StartReceiving();
  last_report=HAL_GetTick();
  for(i=0U;i<LASER_COUNT;i++) samples[i].window_tick=last_report;
}

void App_LaserRangeTest_Task(void)
{
  uint32_t now,age,bad=0U,drop=0U,errors=0U,mean,window,hz_x10;
  const char *status;
  RangeSample *s;
  UartDebug_Task();
  now=HAL_GetTick();
  if(!running) {
    LaserRx_BootLogTask();
    /* Parked retry also recovers an expansion board powered after the MCU. */
    if((uint32_t)(now-last_retry)>=3000U) StartReceiving();
  } else LaserRx_Poll(RangeFrame);
  now=HAL_GetTick();
  if((uint32_t)(now-last_report)<REPORT_SLOT_MS) return;
  last_report=now;s=&samples[next_channel];
  age=s->seen ? (uint32_t)(now-s->tick) : 0U;
  if(!running) status="LINK_ERROR";
  else if(!s->seen) status="WAIT_DATA";
  else if(age>STALE_MS) status="STALE";
  else if(!s->raw || !s->confidence) status="BAD_DATA";
  else status="RAW";
  LaserRx_Counters((LaserDirection_t)next_channel,&bad,&drop,&errors);
  /* Existing web dashboard accepts this line without firmware-specific gates. */
  UartDebug_Printf("LASER DIR=%s D=%u C=%u STATUS=%s AGE=%lu BAD=%lu DROP=%lu ERR=%lu STATE=RANGE_TEST FAULT=%s\r\n",
      names[next_channel],(unsigned)s->raw,(unsigned)s->confidence,status,
      (unsigned long)age,(unsigned long)bad,(unsigned long)drop,(unsigned long)errors,
      running?"NONE":"UARTCH4_CONFIG_FAILED");
  window=(uint32_t)(now-s->window_tick);
  if(window>=STATS_PERIOD_MS) {
    mean=s->count ? (s->sum+s->count/2U)/s->count : 0U;
    hz_x10=(s->frames*10000U+window/2U)/window;
    UartDebug_Printf("RANGE DIR=%s N=%lu MEAN_MM=%lu MIN_MM=%u MAX_MM=%u REJECT=%lu FW=%u REF_MM=%u FRAMES=%lu WINDOW_MS=%lu HZ_X10=%lu\r\n",
      names[next_channel],(unsigned long)s->count,(unsigned long)mean,
      (unsigned)(s->count?s->minimum:0U),(unsigned)(s->count?s->maximum:0U),
      (unsigned long)s->rejected,(unsigned)s->firmware,(unsigned)actual_mm[next_channel],
      (unsigned long)s->frames,(unsigned long)window,(unsigned long)hz_x10);
    if(actual_mm[next_channel] && s->count)
      UartDebug_Printf("RANGE_ERROR DIR=%s ERROR_MM=%ld\r\n",names[next_channel],
        (long)mean-(long)actual_mm[next_channel]);
    s->count=0U;s->sum=0U;s->rejected=0U;s->frames=0U;s->window_tick=now;
  }
  next_channel=(uint8_t)((next_channel+1U)%LASER_COUNT);
}
