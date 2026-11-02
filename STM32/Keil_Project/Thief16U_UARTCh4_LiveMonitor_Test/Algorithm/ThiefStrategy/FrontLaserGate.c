#include "FrontLaserGate.h"
#include "ThiefStrategyConfig.h"
#include <stddef.h>

#define LASER_MIN_MM 50U
#define LASER_MIN_CONFIDENCE 10U
#define FIELD_MIN_X_MM (-150)
#define FIELD_MAX_X_MM 3450
#define FIELD_MIN_Y_MM (-150)
#define FIELD_MAX_Y_MM 2850

/* Front/rear use150mm and sides100mm only for map projection.
 * Raw acceptance limits are independent of these mounting estimates. */
uint16_t FrontLaserGate_CenterDistanceMm(uint16_t raw_mm, uint8_t direction)
{
  uint32_t value;
  if (direction==1U || direction==2U) value=(uint32_t)raw_mm+THIEF_LASER_SIDE_OFFSET_MM;
  else if (direction==3U) value=(uint32_t)raw_mm+THIEF_LASER_REAR_OFFSET_MM;
  else value=(uint32_t)raw_mm+THIEF_LASER_REAR_OFFSET_MM;
  return (uint16_t)(value>65535UL?65535UL:value);
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
  uint16_t maximum_mm=THIEF_LASER_CHANNEL_MAX_MM(direction);
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
  } else if (distance_mm > maximum_mm) {
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
    } else if (distance_mm>maximum_mm) {
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
  /* A close object has little remaining reaction time. One high-quality,
   * map-projected current frame triggers retreat. Missing, stale, low-quality
   * and off-map returns are still rejected before this point. */
  if (geometric_valid && distance_mm <= maximum_mm &&
      confidence >= THIEF_NEAR_CONFIDENCE) {
    gate->near_previous = 1U;
    gate->status = FRONT_LASER_OPPONENT;
    return 1U;
  }
  gate->near_previous = 0U;
#endif
  return 0U;
}

/* At rest, isolated reflections must not launch an escape. Moving obstacle
 * response still uses Observe's immediate current-frame result. */
uint8_t FrontLaserGate_HideConfirmed(const FrontLaserGate_t *gate)
{
  uint8_t i,index;
  uint16_t minimum=65535U,maximum=0U;
  if(gate==NULL || gate->status!=FRONT_LASER_OPPONENT ||
      (gate->valid_history&7U)!=7U || gate->confidence<THIEF_NEAR_CONFIDENCE) return 0U;
  for(i=1U;i<=3U;i++) {
    index=(uint8_t)((gate->distance_index+5U-i)%5U);
    if(gate->valid_distances[index]<minimum) minimum=gate->valid_distances[index];
    if(gate->valid_distances[index]>maximum) maximum=gate->valid_distances[index];
  }
  return maximum-minimum<=120U ? 1U : 0U;
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

