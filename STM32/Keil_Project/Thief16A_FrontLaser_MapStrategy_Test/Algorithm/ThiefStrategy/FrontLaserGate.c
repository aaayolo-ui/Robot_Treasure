#include "FrontLaserGate.h"
#include <stddef.h>

#define LASER_MIN_MM 50U
#define LASER_MAX_MM 600U
#define LASER_MIN_CONFIDENCE 10U
#define FIELD_MIN_X_MM (-150)
#define FIELD_MAX_X_MM 3450
#define FIELD_MIN_Y_MM (-150)
#define FIELD_MAX_Y_MM 2850

static uint8_t ProjectOnRoad(const ThiefMapPose_t *pose, uint16_t distance_mm,
                             int16_t *hit_x, int16_t *hit_y)
{
  uint16_t cursor = pose->node_id;
  uint16_t remaining = distance_mm;
  uint8_t heading = pose->body_heading;
  while (remaining > 0U) {
    uint16_t next, length;
    const MapRouteNode_t *node;
    if (ThiefMapPlanner_HasRoad(cursor, heading, &next, &length) == 0U) return 0U;
    node = ThiefMapPlanner_Node(cursor);
    if (node == NULL) return 0U;
    if (remaining <= length) {
      int16_t x = (int16_t)(node->x_half * 150);
      int16_t y = (int16_t)(node->y_half * 150);
      if (heading == 0U) y = (int16_t)(y - remaining);
      else if (heading == 1U) x = (int16_t)(x + remaining);
      else if (heading == 2U) y = (int16_t)(y + remaining);
      else x = (int16_t)(x - remaining);
      *hit_x = x;
      *hit_y = y;
      return 1U;
    }
    remaining = (uint16_t)(remaining - length);
    cursor = next;
  }
  return 0U;
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
  for (gate->accepted_count = 0U; gate->accepted_count < 5U;
       gate->accepted_count++) gate->valid_distances[gate->accepted_count] = 0U;
  gate->accepted_count = 0U;
  gate->status = FRONT_LASER_NO_DATA;
}

uint8_t FrontLaserGate_Observe(FrontLaserGate_t *gate,
                               const ThiefMapPose_t *pose,
                               uint8_t hiding,
                               uint16_t distance_mm,
                               uint8_t confidence,
                               uint16_t firmware)
{
  uint8_t geometric_valid = 0U;
  uint8_t i;
  uint16_t minimum = 0xFFFFU;
  uint16_t maximum = 0U;
  if (gate == NULL) return 0U;
  gate->distance_mm = distance_mm;
  gate->confidence = confidence;
  gate->firmware = firmware;
  if ((hiding == 0U) || (pose == NULL) || (pose->valid == 0U)) {
    gate->status = FRONT_LASER_NOT_HIDING;
  } else if ((distance_mm < LASER_MIN_MM) || (confidence < LASER_MIN_CONFIDENCE)) {
    gate->status = FRONT_LASER_BAD_QUALITY;
  } else if (distance_mm > LASER_MAX_MM) {
    gate->status = FRONT_LASER_TOO_FAR;
  } else if (ProjectOnRoad(pose, distance_mm,
                           &gate->hit_x_mm, &gate->hit_y_mm) == 0U) {
    gate->status = FRONT_LASER_NO_MAP_ROAD;
  } else if ((gate->hit_x_mm < FIELD_MIN_X_MM) || (gate->hit_x_mm > FIELD_MAX_X_MM) ||
             (gate->hit_y_mm < FIELD_MIN_Y_MM) || (gate->hit_y_mm > FIELD_MAX_Y_MM)) {
    gate->status = FRONT_LASER_OUTSIDE_FIELD;
  } else {
    gate->status = FRONT_LASER_UNSTABLE;
    geometric_valid = 1U;
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
  return 0U;
}

const char *FrontLaserGate_StatusText(FrontLaserGateStatus_t status)
{
  static const char *const text[] = {
    "NO_DATA", "MOVING", "BAD_DATA", "TOO_FAR",
    "NO_ROAD", "OUT_FIELD", "UNSTABLE", "OPPONENT"
  };
  return ((uint8_t)status < (uint8_t)(sizeof(text) / sizeof(text[0]))) ?
         text[(uint8_t)status] : "UNKNOWN";
}

