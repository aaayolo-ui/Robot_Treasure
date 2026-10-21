#ifndef THIEF_THREAT_MODEL_H
#define THIEF_THREAT_MODEL_H
#include "MapRoutePlan.h"
#include <stdint.h>

void ThiefThreatModel_Init(void);
/* Record only map-gated, stable observations. Channel must be a world heading. */
void ThiefThreatModel_Record(uint8_t channel, int16_t x_mm, int16_t y_mm,
                             uint32_t tick);
uint8_t ThiefThreatModel_Prepare(uint32_t now);
/* Fresh, unambiguous latest observation of the single opponent. */
uint8_t ThiefThreatModel_Snapshot(uint32_t now,int16_t *x,int16_t *y);
/* Map-derived earliest incoming corridors, not an observed opponent heading. */
uint8_t ThiefThreatModel_ApproachMask(uint16_t node);
int32_t ThiefThreatModel_Margin(uint16_t from, uint16_t to,
                               uint16_t progress_mm, uint32_t own_time_ms);
uint32_t ThiefThreatModel_WorkBytes(void);
#endif
