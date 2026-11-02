#ifndef TRAVEL_FRAME_H
#define TRAVEL_FRAME_H

#include "nchd12.h"

/* Reverse travel is a 180-degree rotation of the chassis coordinates. */
void TravelFrame_SetReverse(uint8_t reverse);
uint8_t TravelFrame_IsReverse(void);
NCHD12_Status_t TravelFrame_ReadLeading(uint16_t *raw16);
NCHD12_Status_t TravelFrame_ReadTrailing(uint16_t *raw16);
void TravelFrame_SetWheelTargets(int32_t fl, int32_t fr, int32_t rl, int32_t rr);
int16_t TravelFrame_Yaw(int16_t chassis_yaw);

#endif
