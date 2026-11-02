#include "FrontLaserGate.h"
#include "ThiefStrategyConfig.h"
#include <stddef.h>

#define LASER_MIN_MM 50U
#define LASER_MAX_MM THIEF_LASER_MAX_MM
#define LASER_MIN_CONFIDENCE 10U
#define FIELD_MIN_X_MM (-150)
#define FIELD_MAX_X_MM 3450
#define FIELD_MIN_Y_MM (-150)
#define FIELD_MAX_Y_MM 2850

/* Side/rear offsets use the user's 3-cell mounting measurements (about +/-50 mm).
 * Front retains the older proportional estimate until its probe is replaced. */
uint16_t FrontLaserGate_CenterDistanceMm(uint16_t raw_mm, uint8_t direction)
{
  uint32_t value;
  if (direction==1U || direction==2U) value=(uint32_t)raw_mm+THIEF_LASER_SIDE_OFFSET_MM;
  else if (direction==3U) value=(uint32_t)raw_mm+THIEF_LASER_REAR_OFFSET_MM;
  else value=(uint32_t)raw_mm*THIEF_LASER_SCALE_NUM/THIEF_LASER_SCALE_DEN;
  return (uint16_t)(value>65535UL?65535UL:value);
}

static uint8_t ProjectOnRoad(const ThiefMapPose_t *pose, uint16_t distance_mm,
                             int16_t *hit_x, int16_t *hit_y)
{
  uint16_t cursor = 0U, remaining = distance_mm, i;
  uint8_t heading = pose->body_heading;
  uint8_t vertical = (uint8_t)((heading & 1U) == 0U);
  int32_t origin = vertical ? pose->y_mm : pose->x_mm;
  int32_t side = vertical ? pose->x_mm : pose->y_mm;
  int32_t sign = (heading == 0U || heading == 3U) ? -1L : 1L;
  if (heading > 3U) return 0U;
  /* Start at the estimated moving XY, not the last gray-confirmed node.
   * Find the first real road piece in this cardinal ray, then walk graph
   * connections. Aligned but disconnected road pieces cannot bridge a gap. */
  for (i = 0U; i < MAP_ROUTE_SEGMENT_COUNT; i++) {
    const MapRouteSegment_t *s = &map_route_segments[i];
    const MapRouteNode_t *a = ThiefMapPlanner_Node(s->start);
    const MapRouteNode_t *b = ThiefMapPlanner_Node(s->end);
    int32_t av, bv, low, high, available;
    if (a == NULL || b == NULL) continue;
    if ((vertical ? a->x_half : a->y_half) * 150L != side ||
        (vertical ? b->x_half : b->y_half) * 150L != side) continue;
    av = (vertical ? a->y_half : a->x_half) * 150L;
    bv = (vertical ? b->y_half : b->x_half) * 150L;
    low = av < bv ? av : bv;
    high = av > bv ? av : bv;
    if (origin < low || origin > high) continue;
    available = sign > 0L ? high - origin : origin - low;
    if (available <= 0L) continue;
    cursor = (sign * av > sign * bv) ? s->start : s->end;
    if (remaining <= available) remaining = 0U;
    else remaining = (uint16_t)(remaining - available);
    break;
  }
  if (cursor == 0U) return 0U;
  while (remaining > 0U) {
    uint16_t next, length;
    if (ThiefMapPlanner_HasRoad(cursor, heading, &next, &length) == 0U) return 0U;
    if (length == 0U) return 0U;
    if (remaining <= length) break;
    remaining = (uint16_t)(remaining - length);
    cursor = next;
  }
  *hit_x = (int16_t)(pose->x_mm + (vertical ? 0L : sign * distance_mm));
  *hit_y = (int16_t)(pose->y_mm + (vertical ? sign * distance_mm : 0L));
  return 1U;
}

void FrontLaserGate_Init(FrontLaserGate_t *gate)
{
  if (gate == NULL) return;
  gate->distance_mm = 0U;
  gate->firmware = 0U;
  gate->confidence = 0U;
  gate->valid_history = 0U;
  gate->history_count = 0U;
  gate->accepted_count = 0U;
  gate->distance_index = 0U;
  gate->near_previous = 0U;
  for (gate->accepted_count = 0U; gate->accepted_count < 5U;
       gate->accepted_count++) gate->valid_distances[gate->accepted_count] = 0U;
  gate->accepted_count = 0U;
  gate->hit_x_mm = 0;
  gate->hit_y_mm = 0;
  gate->status = FRONT_LASER_NO_DATA;
}

void FrontLaserGate_ResetEvidence(FrontLaserGate_t *gate)
{
  if (gate == NULL) return;
  gate->valid_history = 0U;
  gate->history_count = 0U;
  gate->accepted_count = 0U;
  gate->distance_index = 0U;
  gate->near_previous = 0U;
  for (gate->accepted_count = 0U; gate->accepted_count < 5U;
       gate->accepted_count++) gate->valid_distances[gate->accepted_count] = 0U;
  gate->accepted_count = 0U;
  gate->status = FRONT_LASER_NO_DATA;
}

uint8_t FrontLaserGate_Observe(FrontLaserGate_t *gate,
                               const ThiefMapPose_t *pose,
                               uint8_t detection_enabled,
                               uint16_t distance_mm,
                               uint8_t confidence,
                               uint16_t firmware,
                               uint8_t direction)
{
  uint16_t center_mm=FrontLaserGate_CenterDistanceMm(distance_mm,direction);
  uint8_t geometric_valid = 0U;
  uint8_t i;
  uint16_t minimum = 0xFFFFU;
  uint16_t maximum = 0U;
  if (gate == NULL) return 0U;
  gate->distance_mm = distance_mm;
  gate->confidence = confidence;
  gate->firmware = firmware;
  if ((detection_enabled == 0U) || (pose == NULL) || (pose->valid == 0U)) {
    gate->status = FRONT_LASER_NOT_HIDING;
  } else if ((distance_mm < LASER_MIN_MM) || (confidence < LASER_MIN_CONFIDENCE)) {
    gate->status = FRONT_LASER_BAD_QUALITY;
  } else if (distance_mm > LASER_MAX_MM) {
    gate->status = FRONT_LASER_TOO_FAR;
  } else {
    /* Test the actual projected hit before road connectivity. A car near an
     * edge can see beyond the field even when it is not exactly on the edge. */
    gate->hit_x_mm=(int16_t)(pose->x_mm +
        (pose->body_heading==1U?center_mm:pose->body_heading==3U?-(int32_t)center_mm:0L));
    gate->hit_y_mm=(int16_t)(pose->y_mm +
        (pose->body_heading==2U?center_mm:pose->body_heading==0U?-(int32_t)center_mm:0L));
    if ((gate->hit_x_mm < FIELD_MIN_X_MM) || (gate->hit_x_mm > FIELD_MAX_X_MM) ||
        (gate->hit_y_mm < FIELD_MIN_Y_MM) || (gate->hit_y_mm > FIELD_MAX_Y_MM)) {
      gate->status=FRONT_LASER_OUTSIDE_FIELD;
    } else if (!ProjectOnRoad(pose,center_mm,&gate->hit_x_mm,&gate->hit_y_mm)) {
      gate->status=FRONT_LASER_NO_MAP_ROAD;
    } else if (center_mm>THIEF_LASER_TRIGGER_MM) {
      gate->status=FRONT_LASER_IN_RANGE;
    } else {
      gate->status=FRONT_LASER_UNSTABLE;
      geometric_valid=1U;
    }
  }

  gate->valid_history = (uint8_t)(((gate->valid_history << 1U) |
                                  geometric_valid) & 0x1FU);
  gate->valid_distances[gate->distance_index] =
      (geometric_valid != 0U) ? distance_mm : 0U;
  gate->distance_index = (uint8_t)((gate->distance_index + 1U) % 5U);
  if (gate->history_count < 5U) gate->history_count++;
  gate->accepted_count = 0U;
  for (i = 0U; i < gate->history_count; i++) {
    if ((gate->valid_history & (1U << i)) != 0U) gate->accepted_count++;
    if (gate->valid_distances[i] != 0U) {
      if (gate->valid_distances[i] < minimum) minimum = gate->valid_distances[i];
      if (gate->valid_distances[i] > maximum) maximum = gate->valid_distances[i];
    }
  }
  if ((geometric_valid != 0U) && (gate->history_count >= 5U) &&
      (gate->accepted_count >= 3U) &&
      ((uint16_t)(maximum - minimum) <= 120U)) {
    gate->status = FRONT_LASER_OPPONENT;
    return 1U;
  }
#if THIEF_FAST_NEAR_CONFIRM
  /* A close object has little remaining reaction time. Two consecutive
   * high-quality map-projected frames can trigger retreat. Never combine
   * different sensors, a missing/invalid frame, or off-map returns. This is
   * an obstacle response, not proof of the object's identity. */
  if (geometric_valid && center_mm <= THIEF_NEAR_CONFIRM_MM &&
      confidence >= THIEF_NEAR_CONFIDENCE &&
      (gate->valid_history & 3U) == 3U && gate->near_previous &&
      maximum - minimum <= 80U) {
    gate->near_previous = 1U;
    gate->status = FRONT_LASER_OPPONENT;
    return 1U;
  }
  gate->near_previous = (uint8_t)(geometric_valid &&
      center_mm <= THIEF_NEAR_CONFIRM_MM && confidence >= THIEF_NEAR_CONFIDENCE);
#endif
  return 0U;
}

const char *FrontLaserGate_StatusText(FrontLaserGateStatus_t status)
{
  static const char *const text[] = {
    "NO_DATA", "INACTIVE", "BAD_DATA", "TOO_FAR",
    "NO_ROAD", "OUT_FIELD", "UNSTABLE", "OPPONENT", "IN_RANGE"
  };
  return ((uint8_t)status < (uint8_t)(sizeof(text) / sizeof(text[0]))) ?
         text[(uint8_t)status] : "UNKNOWN";
}

