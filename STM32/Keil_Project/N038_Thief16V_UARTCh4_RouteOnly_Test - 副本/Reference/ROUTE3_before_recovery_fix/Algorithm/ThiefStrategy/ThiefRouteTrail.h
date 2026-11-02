#ifndef THIEF_ROUTE_TRAIL_H
#define THIEF_ROUTE_TRAIL_H
#include "ThiefMapPlanner.h"

/* Reserve one runtime step for the partially traversed interception leg. */
#define THIEF_TRAIL_CAPACITY (THIEF_ROUTE_MAX_STEPS-1U)
typedef struct {
  uint16_t from, to, length_mm, segments;
  uint16_t leading_raw, trailing_raw, measured_mm;
  int16_t yaw_x100;
  uint8_t heading, drive_mode;
} ThiefTrailEntry;
typedef struct {
  ThiefTrailEntry entries[THIEF_TRAIL_CAPACITY], pending;
  uint16_t home, expected_return_node;
  uint8_t home_body, count, enabled, pending_valid;
  uint8_t returning, return_left, return_pending, first_back;
} ThiefRouteTrail;

uint8_t ThiefRouteTrail_TrackLine(uint16_t raw);
void ThiefRouteTrail_Begin(ThiefRouteTrail *trail,uint16_t home,uint8_t body);
uint8_t ThiefRouteTrail_Stage(ThiefRouteTrail *trail,const MapRouteStep_t *step,
    uint16_t leading,uint16_t trailing,int16_t yaw,int32_t measured_mm);
/* Called after actual exit-line acquisition or a confirmed stopped endpoint. */
uint8_t ThiefRouteTrail_Commit(ThiefRouteTrail *trail);
uint8_t ThiefRouteTrail_BuildReturn(ThiefRouteTrail *trail,
    const MapRouteStep_t *partial,ThiefRoutePlan_t *plan);
#endif
