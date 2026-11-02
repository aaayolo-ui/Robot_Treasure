#ifndef THIEF_LOCALIZATION_H
#define THIEF_LOCALIZATION_H

#include "ThiefMapPlanner.h"
#include <stdint.h>

/* Startup is a user-known N054/W pose, not an absolute IMU position fix.
 * Yaw tolerances/odometry scale need car validation. Angles are degree x100. */
#define THIEF_LOC_GYRO_STALE_MS 150UL
#define THIEF_LOC_HEADING_TOL_X100 2500L
#define THIEF_LOC_TURN_MIN_X100 7000L
#define THIEF_LOC_TURN_MAX_X100 11000L

typedef struct {
  int16_t yaw_x100;
  uint32_t stamp_ms;
  uint8_t valid;
} ThiefLocYaw_t;

typedef enum {
  THIEF_LOC_OK=0, THIEF_LOC_UNINITIALIZED, THIEF_LOC_GYRO_STALE,
  THIEF_LOC_HEADING_MISMATCH, THIEF_LOC_STEP_MISMATCH,
  THIEF_LOC_DISTANCE, THIEF_LOC_GRAY_MISMATCH, THIEF_LOC_TURN_DIRECTION
} ThiefLocError_t;
typedef enum { THIEF_LOC_AT_NODE=0, THIEF_LOC_ON_LEG, THIEF_LOC_AT_JUNCTION } ThiefLocPhase_t;

typedef struct {
  ThiefMapPose_t pose;
  MapRouteStep_t step;
  int32_t reversal_entry_mm; /* Signed position at reversal, never a node fix. */
  int16_t origin_yaw_x100, arrival_yaw_x100, heading_error_x100;
  uint16_t confirmed_node, gray_raw12;
  uint32_t gyro_age_ms, confirmations;
  int8_t yaw_cw_sign; /* Learned at the first gray-confirmed planned quarter turn. */
  uint8_t origin_heading, body_heading, initialized, phase, error;
} ThiefLocalization_t;

void ThiefLocalization_Reset(ThiefLocalization_t *loc);
uint8_t ThiefLocalization_Start(ThiefLocalization_t *loc, uint16_t node,
    uint8_t body, const ThiefLocYaw_t *yaw, uint32_t now);
uint8_t ThiefLocalization_BeginStep(ThiefLocalization_t *loc,
    const MapRouteStep_t *step, const ThiefLocYaw_t *yaw, uint32_t now);
uint8_t ThiefLocalization_Update(ThiefLocalization_t *loc, int32_t distance_mm,
    const ThiefLocYaw_t *yaw, uint32_t now);
/* Cancel an untravelled departure without claiming a new gray observation. */
uint8_t ThiefLocalization_CancelDeparture(ThiefLocalization_t *loc,
    int32_t distance_mm,const ThiefLocYaw_t *yaw,uint32_t now);
/* Reverse an unfinished straight leg without inventing an endpoint fix.
 * Caller uses length-old_progress as the new odometry offset. */
uint8_t ThiefLocalization_ReverseLeg(ThiefLocalization_t *loc,
    const MapRouteStep_t *reverse_step,int32_t old_progress,
    const ThiefLocYaw_t *yaw,uint32_t now);
uint8_t ThiefLocalization_PrepareDeparture(ThiefLocalization_t *loc,
    const MapRouteStep_t *next,const ThiefLocYaw_t *yaw,uint32_t now);
/* Pure shape check: does not advance or invalidate localization. */
uint8_t ThiefLocalization_GrayMatchesStep(const MapRouteStep_t *step,uint16_t leading_gray12);
uint8_t ThiefLocalization_Arrive(ThiefLocalization_t *loc,
    const MapRouteStep_t *step, uint16_t leading_gray12, int32_t distance_mm,
    const ThiefLocYaw_t *yaw, uint32_t now);
uint8_t ThiefLocalization_ExitReady(ThiefLocalization_t *loc,
    const MapRouteStep_t *next, uint8_t leading_line_valid,
    const ThiefLocYaw_t *yaw, uint32_t now);
uint8_t ThiefLocalization_CommitExit(ThiefLocalization_t *loc,
    const MapRouteStep_t *next, const ThiefLocYaw_t *yaw, uint32_t now);
const char *ThiefLocalization_ErrorText(uint8_t error);

/* Bounded topological sequence matcher. Costs are relative evidence scores,
 * NOT calibrated probabilities. No heap, floating point, or map-sized RAM. */
#define THIEF_TOPO_CAPACITY 8U
typedef struct { uint16_t node,cost; } ThiefTopoCandidate;
typedef struct {
  ThiefTopoCandidate item[THIEF_TOPO_CAPACITY];
  uint16_t omitted_cost;
  uint8_t count,heading,events;
} ThiefTopoBelief;
uint8_t ThiefTopo_Seed(ThiefTopoBelief *b,uint16_t last_node,uint8_t heading,
    int32_t travelled_mm,uint8_t ports,int8_t straight);
uint8_t ThiefTopo_Observe(ThiefTopoBelief *b,uint8_t heading,int32_t travelled_mm,
    uint8_t ports,int8_t straight);
uint16_t ThiefTopo_Resolve(const ThiefTopoBelief *b);
void ThiefTopo_Refine(ThiefTopoBelief *b,uint8_t ports,int8_t straight);
uint8_t ThiefLocalization_Reanchor(ThiefLocalization_t *l,uint16_t node,
    uint8_t travel,uint8_t mode,const ThiefLocYaw_t *y,uint32_t now);

/* Local straight-corridor hypotheses. No node is selected by odometry. */
typedef struct {
  uint16_t nodes[4],branch_raw;
  uint8_t count,heading,straight_frames,gap_frames,resolved,armed,branch_frames,new_branch;
} ThiefLocCandidates;
uint8_t ThiefLocalization_CandidatesBegin(ThiefLocCandidates *c,
    const MapRouteStep_t *s,uint16_t raw);
uint16_t ThiefLocalization_CandidatesObserve(ThiefLocCandidates *c,
    uint16_t leading,uint16_t trailing,int32_t moved_mm);
uint8_t ThiefLocalization_ConfirmCandidate(ThiefLocalization_t *l,
    const ThiefLocCandidates *c,const ThiefLocYaw_t *y,uint32_t now);

#endif
