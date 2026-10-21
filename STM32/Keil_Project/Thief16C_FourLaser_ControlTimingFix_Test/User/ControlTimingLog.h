#ifndef CONTROL_TIMING_LOG_H
#define CONTROL_TIMING_LOG_H
#include <stdint.h>
void ControlTimingLog_Begin(uint32_t now);
void ControlTimingLog_Loop(uint32_t now);
void ControlTimingLog_Control(uint32_t elapsed, uint8_t state);
void ControlTimingLog_Task(uint8_t running);
#endif
