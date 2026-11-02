#ifndef FOLLOW_SMOOTHING_H
#define FOLLOW_SMOOTHING_H

#include <stdint.h>
#include "LineFollowPID.h"

typedef struct
{
  int32_t base_rpm_x10;
  int32_t correction_rpm_x10;
} FollowSmoothing_t;

void FollowSmoothing_Reset(FollowSmoothing_t *smoothing);
void FollowSmoothing_Apply(FollowSmoothing_t *smoothing,
                           int16_t position_error_x100,
                           uint32_t elapsed_ms,
                           LineFollowPID_Output_t *output);

/* Scale both sides together to retain curvature while acquiring a line. */
void FollowSmoothing_LimitBase(FollowSmoothing_t *smoothing, int32_t limit,
                               LineFollowPID_Output_t *output);
#endif
