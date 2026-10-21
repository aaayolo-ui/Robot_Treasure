#ifndef LINE_FOLLOW_PID_H
#define LINE_FOLLOW_PID_H

#include <stdint.h>

/* All RPM values exposed by this module use RPM x10 units. */
#define LINE_FOLLOW_PID_CONTROL_PERIOD_MS         50U
#define LINE_FOLLOW_PID_BASE_RPM_X10             400L
#define LINE_FOLLOW_PID_PARAMETER_SCALE         1000L
#define LINE_FOLLOW_PID_DEFAULT_KP_X1000          15L
#define LINE_FOLLOW_PID_DEFAULT_KI_X1000           0L
#define LINE_FOLLOW_PID_DEFAULT_KD_X1000           2L
#define LINE_FOLLOW_PID_DEFAULT_KH_X1000           5L
#define LINE_FOLLOW_PID_KP_STEP_X1000              1L
#define LINE_FOLLOW_PID_KI_STEP_X1000              1L
#define LINE_FOLLOW_PID_KD_STEP_X1000              1L
#define LINE_FOLLOW_PID_KH_STEP_X1000              1L
#define LINE_FOLLOW_PID_KP_MIN_X1000               0L
#define LINE_FOLLOW_PID_KP_MAX_X1000             100L
#define LINE_FOLLOW_PID_KI_MIN_X1000               0L
#define LINE_FOLLOW_PID_KI_MAX_X1000              50L
#define LINE_FOLLOW_PID_KD_MIN_X1000               0L
#define LINE_FOLLOW_PID_KD_MAX_X1000              50L
#define LINE_FOLLOW_PID_KH_MIN_X1000               0L
#define LINE_FOLLOW_PID_KH_MAX_X1000              50L
#define LINE_FOLLOW_PID_INTEGRAL_LIMIT           10000L
#define LINE_FOLLOW_PID_D_LIMIT_RPM_X10          40L
#define LINE_FOLLOW_PID_MAX_CORRECTION_RPM_X10   120L

typedef enum
{
  LINE_FOLLOW_PARAMETER_KP = 0,
  LINE_FOLLOW_PARAMETER_KI,
  LINE_FOLLOW_PARAMETER_KD,
  LINE_FOLLOW_PARAMETER_KH,
  LINE_FOLLOW_PARAMETER_COUNT
} LineFollowPID_ParameterId_t;

typedef struct
{
  int32_t kp_x1000;
  int32_t ki_x1000;
  int32_t kd_x1000;
  int32_t kh_x1000;
} LineFollowPID_Parameters_t;

typedef struct
{
  uint8_t initialized;
  int16_t previous_error_x100;
  int32_t filtered_delta_error_x100;
  int32_t integral_error;
} LineFollowPID_Controller_t;

typedef struct
{
  int32_t base_rpm_x10;
  int32_t delta_error_x100;
  int32_t filtered_delta_error_x100;
  int32_t p_term_rpm_x10;
  int32_t d_term_rpm_x10;
  int32_t position_correction_rpm_x10;
  int32_t heading_correction_rpm_x10;
  int32_t final_correction_rpm_x10;
  int32_t left_target_rpm_x10;
  int32_t right_target_rpm_x10;
  int32_t i_term_rpm_x10;
  int32_t integral_error;
} LineFollowPID_Output_t;

void LineFollowPID_Reset(LineFollowPID_Controller_t *controller);
const LineFollowPID_Parameters_t *LineFollowPID_GetParameters(void);
uint8_t LineFollowPID_AdjustParameter(LineFollowPID_ParameterId_t parameter,
                                       int32_t delta_x1000);
void LineFollowPID_Calculate(LineFollowPID_Controller_t *controller,
                             int16_t vehicle_error_x100,
                             uint32_t elapsed_ms,
                             LineFollowPID_Output_t *output);
void LineFollowPID_ApplyHeadingP(LineFollowPID_Output_t *output,
                                 int16_t heading_error_x100);

#endif
