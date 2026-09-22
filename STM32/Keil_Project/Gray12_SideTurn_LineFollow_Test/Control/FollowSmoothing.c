#include "FollowSmoothing.h"

/* TUNING AREA - outer-loop target shaping after gray PID, before wheel PI.
 * Inspired by the supplied LineFollow.c: base-speed ramp, slowing on large
 * error and steer slew limit. Initial values are not hardware-verified.
 * All speeds use RPM x10. */
#define SMOOTH_ENABLE_BASE_RAMP       1U
#define SMOOTH_ENABLE_CURVE_SLOWDOWN  1U
#define SMOOTH_ENABLE_STEER_SLEW      1U
#define SMOOTH_BASE_STEP_RPM_X10     20L
#define SMOOTH_STEER_STEP_RPM_X10    80L /* 8 RPM per 50 ms; faster P5/P8 response */
#define SMOOTH_CURVE_START_ERROR    100L
#define SMOOTH_CURVE_FULL_ERROR     300L
#define SMOOTH_CURVE_MIN_BASE_X10   500L /* 50.0 RPM at large error */

static int32_t FollowSmoothing_Approach(int32_t current, int32_t target,
                                         int32_t step)
{
  if (target > current + step) return current + step;
  if (target < current - step) return current - step;
  return target;
}

static int32_t FollowSmoothing_Step(int32_t step_per_period,
                                     uint32_t elapsed_ms)
{
  int64_t step = ((int64_t)step_per_period * (int64_t)elapsed_ms) /
                 LINE_FOLLOW_PID_CONTROL_PERIOD_MS;
  if (step < 1LL) step = 1LL;
  if (step > 2147483647LL) step = 2147483647LL;
  return (int32_t)step;
}

void FollowSmoothing_Reset(FollowSmoothing_t *smoothing)
{
  if (smoothing == 0) return;
  smoothing->base_rpm_x10 = 0;
  smoothing->correction_rpm_x10 = 0;
}

void FollowSmoothing_Apply(FollowSmoothing_t *smoothing,
                           int16_t position_error_x100,
                           uint32_t elapsed_ms,
                           LineFollowPID_Output_t *output)
{
  int32_t desired_base;
  int32_t desired_correction;
  int32_t error;

  if ((smoothing == 0) || (output == 0)) return;
  desired_base = output->base_rpm_x10;
  desired_correction = output->final_correction_rpm_x10;
  error = (position_error_x100 < 0) ?
          -(int32_t)position_error_x100 : (int32_t)position_error_x100;

#if SMOOTH_ENABLE_CURVE_SLOWDOWN
  if (error > SMOOTH_CURVE_START_ERROR)
  {
    int32_t bounded_error = (error > SMOOTH_CURVE_FULL_ERROR) ?
                            SMOOTH_CURVE_FULL_ERROR : error;
    int32_t reduction = ((desired_base - SMOOTH_CURVE_MIN_BASE_X10) *
                         (bounded_error - SMOOTH_CURVE_START_ERROR)) /
                        (SMOOTH_CURVE_FULL_ERROR - SMOOTH_CURVE_START_ERROR);
    desired_base -= reduction;
  }
#endif

#if SMOOTH_ENABLE_BASE_RAMP
  smoothing->base_rpm_x10 = FollowSmoothing_Approach(
      smoothing->base_rpm_x10, desired_base,
      FollowSmoothing_Step(SMOOTH_BASE_STEP_RPM_X10, elapsed_ms));
#else
  smoothing->base_rpm_x10 = desired_base;
#endif

#if SMOOTH_ENABLE_STEER_SLEW
  smoothing->correction_rpm_x10 = FollowSmoothing_Approach(
      smoothing->correction_rpm_x10, desired_correction,
      FollowSmoothing_Step(SMOOTH_STEER_STEP_RPM_X10, elapsed_ms));
#else
  smoothing->correction_rpm_x10 = desired_correction;
#endif

  output->base_rpm_x10 = smoothing->base_rpm_x10;
  output->final_correction_rpm_x10 = smoothing->correction_rpm_x10;
  output->left_target_rpm_x10 = smoothing->base_rpm_x10 +
                                 smoothing->correction_rpm_x10;
  output->right_target_rpm_x10 = smoothing->base_rpm_x10 -
                                  smoothing->correction_rpm_x10;
}
