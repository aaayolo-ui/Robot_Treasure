#ifndef TREASURE_WHEEL_LOG_H
#define TREASURE_WHEEL_LOG_H

#include <stdint.h>

#define TREASURE_WHEEL_BEFORE_LASER 1U
#define TREASURE_WHEEL_AFTER_LASER 2U
#define TREASURE_WHEEL_BEFORE_FAULT 3U
#define TREASURE_WHEEL_AFTER_STOP 4U

/* Main-loop only. Init clears data and is intended once at app init.
 * Begin refuses to overwrite an unexported run. Task's running argument is
 * an export inhibit: pass 1 throughout routes, HIDING and SETTLING; pass 0
 * only after a terminal/manual stop or fault. No UART formatting occurs
 * while inhibited, even when the six-second capture window is finished. */
void TreasureWheelLog_Init(void);
void TreasureWheelLog_Begin(uint32_t now);
void TreasureWheelLog_Task(uint32_t now, uint8_t state, uint8_t running);
/* Immediate snapshots before/after laser setup and before braking. */
void TreasureWheelLog_Event(uint8_t kind, uint32_t now, uint8_t state);
/* Set once when a real control step is due; does not sample or format. */
void TreasureWheelLog_ControlElapsed(uint32_t elapsed_ms);
void TreasureWheelLog_GrayContext(uint32_t now, uint16_t front, uint16_t rear,
                                  int16_t position, int16_t heading);
uint32_t TreasureWheelLog_WorkBytes(void);

#endif
