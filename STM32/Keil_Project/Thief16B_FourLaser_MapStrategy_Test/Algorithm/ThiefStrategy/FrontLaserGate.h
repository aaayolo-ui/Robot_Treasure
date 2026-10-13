#ifndef FRONT_LASER_GATE_H
#define FRONT_LASER_GATE_H

#include "ThiefMapPlanner.h"
#include <stdint.h>

typedef enum {
  FRONT_LASER_NO_DATA = 0,
  FRONT_LASER_NOT_HIDING,
  FRONT_LASER_BAD_QUALITY,
  FRONT_LASER_TOO_FAR,
  FRONT_LASER_NO_MAP_ROAD,
  FRONT_LASER_OUTSIDE_FIELD,
  FRONT_LASER_UNSTABLE,
  FRONT_LASER_OPPONENT
} FrontLaserGateStatus_t;

typedef struct {
  uint16_t distance_mm;
  uint16_t firmware;
  uint8_t confidence;
  uint8_t valid_history;
  uint8_t history_count;
  uint8_t accepted_count;
  uint8_t distance_index;
  uint16_t valid_distances[5];
  int16_t hit_x_mm;
  int16_t hit_y_mm;
  FrontLaserGateStatus_t status;
} FrontLaserGate_t;

void FrontLaserGate_Init(FrontLaserGate_t *gate);
void FrontLaserGate_ResetEvidence(FrontLaserGate_t *gate);
uint8_t FrontLaserGate_Observe(FrontLaserGate_t *gate,
                               const ThiefMapPose_t *pose,
                               uint8_t hiding,
                               uint16_t distance_mm,
                               uint8_t confidence,
                               uint16_t firmware);
const char *FrontLaserGate_StatusText(FrontLaserGateStatus_t status);

#endif

