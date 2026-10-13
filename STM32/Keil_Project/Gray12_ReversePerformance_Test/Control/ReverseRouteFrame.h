#ifndef REVERSE_ROUTE_FRAME_H
#define REVERSE_ROUTE_FRAME_H

#include <stdint.h>
#include "nchd12.h"

/* Travel coordinates are the physical chassis rotated by 180 degrees.
 * Leading = physical rear; trailing = physical front. */
void ReverseRouteFrame_LeadingInit(void);
NCHD12_Status_t ReverseRouteFrame_LeadingProbe(void);
NCHD12_Status_t ReverseRouteFrame_LeadingConfigure(void);
NCHD12_Status_t ReverseRouteFrame_LeadingRead(uint16_t *raw16);
void ReverseRouteFrame_TrailingInit(void);
NCHD12_Status_t ReverseRouteFrame_TrailingConfigure(void);
NCHD12_Status_t ReverseRouteFrame_TrailingRead(uint16_t *raw16);
void ReverseRouteFrame_SetWheelTargets(int32_t fl, int32_t fr,
                                      int32_t rl, int32_t rr);

#endif
