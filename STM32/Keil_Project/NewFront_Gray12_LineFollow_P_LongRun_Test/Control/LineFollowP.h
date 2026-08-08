#ifndef LINE_FOLLOW_P_H
#define LINE_FOLLOW_P_H

#include <stdint.h>

/* All RPM values exposed by this module use RPM x10 units. */
#define LINE_FOLLOW_P_CONTROL_PERIOD_MS        100U
#define LINE_FOLLOW_P_BASE_RPM_X10             400L
#define LINE_FOLLOW_P_GRAY_KP_NUM              15L
#define LINE_FOLLOW_P_GRAY_KP_DEN              1000L
#define LINE_FOLLOW_P_MAX_CORRECTION_RPM_X10   120L

typedef struct
{
  int32_t base_rpm_x10;
  int32_t correction_rpm_x10;
  int32_t left_target_rpm_x10;
  int32_t right_target_rpm_x10;
} LineFollowP_Output_t;

void LineFollowP_Calculate(int16_t vehicle_error_x100, LineFollowP_Output_t *output);

#endif
