#ifndef STARTUP_WHEEL_LOG_H
#define STARTUP_WHEEL_LOG_H

#include <stdint.h>

void StartupWheelLog_Begin(uint32_t now);
void StartupWheelLog_Task(uint32_t now, uint8_t state, uint8_t running);

#endif
