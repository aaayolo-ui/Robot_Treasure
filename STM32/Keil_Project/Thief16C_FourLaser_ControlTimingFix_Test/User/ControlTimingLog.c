#include "ControlTimingLog.h"
#include "UartDebug.h"
#include <stdio.h>
static uint32_t last_loop, max_loop, max_control, late_count;
static uint8_t active, pending, max_state;
static char report[160];
void ControlTimingLog_Begin(uint32_t now)
{
  last_loop=now; max_loop=0U; max_control=0U; late_count=0U;
  max_state=0U; active=1U; pending=0U;
}
void ControlTimingLog_Loop(uint32_t now)
{
  uint32_t gap=now-last_loop;
  if(!active) return;
  last_loop=now;
  if(gap>max_loop) max_loop=gap;
}
void ControlTimingLog_Control(uint32_t elapsed,uint8_t state)
{
  if(!active) return;
  if(elapsed>max_control) {max_control=elapsed;max_state=state;}
  if(elapsed>25U) late_count++;
}
void ControlTimingLog_Task(uint8_t running)
{
  if(running) return;
  if(active) {
    active=0U;pending=1U;
    (void)snprintf(report,sizeof(report),
      "CONTROL_TIMING LOOP_MAX_MS=%lu CONTROL_MAX_MS=%lu OVER25=%lu MAX_STATE=%u\r\n",
      (unsigned long)max_loop,(unsigned long)max_control,(unsigned long)late_count,(unsigned)max_state);
  }
  if(pending && UartDebug_TrySendString(report)) pending=0U;
}
