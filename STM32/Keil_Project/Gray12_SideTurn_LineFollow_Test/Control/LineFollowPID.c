#include "LineFollowPID.h"

static LineFollowPID_Parameters_t line_follow_pid_parameters =
{
  LINE_FOLLOW_PID_DEFAULT_KP_X1000,
  LINE_FOLLOW_PID_DEFAULT_KI_X1000,
  LINE_FOLLOW_PID_DEFAULT_KD_X1000,
  LINE_FOLLOW_PID_DEFAULT_KH_X1000
};

static int32_t LineFollowPID_Clamp(int32_t value, int32_t limit)
{
  if (value > limit) return limit;
  if (value < -limit) return -limit;
  return value;
}

static int32_t LineFollowPID_CalculateITerm(int32_t integral_error)
{
  return (int32_t)(((int64_t)integral_error *
                    (int64_t)line_follow_pid_parameters.ki_x1000 *
                    (int64_t)LINE_FOLLOW_PID_CONTROL_PERIOD_MS * 10LL) /
                   ((int64_t)LINE_FOLLOW_PID_PARAMETER_SCALE *
                    (int64_t)LINE_FOLLOW_PID_PARAMETER_SCALE));
}

const LineFollowPID_Parameters_t *LineFollowPID_GetParameters(void)
{
  return &line_follow_pid_parameters;
}

uint8_t LineFollowPID_AdjustParameter(LineFollowPID_ParameterId_t parameter,
                                       int32_t delta_x1000)
{
  int32_t *value;
  int32_t minimum;
  int32_t maximum;
  int64_t proposed;

  switch (parameter)
  {
    case LINE_FOLLOW_PARAMETER_KP:
      value = &line_follow_pid_parameters.kp_x1000;
      minimum = LINE_FOLLOW_PID_KP_MIN_X1000;
      maximum = LINE_FOLLOW_PID_KP_MAX_X1000;
      break;
    case LINE_FOLLOW_PARAMETER_KI:
      value = &line_follow_pid_parameters.ki_x1000;
      minimum = LINE_FOLLOW_PID_KI_MIN_X1000;
      maximum = LINE_FOLLOW_PID_KI_MAX_X1000;
      break;
    case LINE_FOLLOW_PARAMETER_KD:
      value = &line_follow_pid_parameters.kd_x1000;
      minimum = LINE_FOLLOW_PID_KD_MIN_X1000;
      maximum = LINE_FOLLOW_PID_KD_MAX_X1000;
      break;
    case LINE_FOLLOW_PARAMETER_KH:
      value = &line_follow_pid_parameters.kh_x1000;
      minimum = LINE_FOLLOW_PID_KH_MIN_X1000;
      maximum = LINE_FOLLOW_PID_KH_MAX_X1000;
      break;
    default:
      return 0U;
  }

  proposed = (int64_t)(*value) + (int64_t)delta_x1000;
  if (proposed < (int64_t)minimum)
  {
    proposed = minimum;
  }
  if (proposed > (int64_t)maximum)
  {
    proposed = maximum;
  }
  *value = (int32_t)proposed;
  return 1U;
}

void LineFollowPID_Reset(LineFollowPID_Controller_t *controller)
{
  if (controller == 0) return;
  controller->initialized = 0U;
  controller->previous_error_x100 = 0;
  controller->filtered_delta_error_x100 = 0;
  controller->integral_error = 0;
}

void LineFollowPID_Calculate(LineFollowPID_Controller_t *controller,
                             int16_t vehicle_error_x100,
                             uint32_t elapsed_ms,
                             LineFollowPID_Output_t *output)
{
  int32_t delta_error_x100;
  int32_t proposed_integral_error;
  int32_t proposed_i_term_rpm_x10;
  int32_t unsaturated_correction_rpm_x10;
  int32_t p_term_rpm_x10;
  int32_t d_term_rpm_x10;
  int32_t i_term_rpm_x10;
  int32_t final_correction_rpm_x10;
  uint32_t d_period_ms;

  if ((controller == 0) || (output == 0))
  {
    return;
  }

  d_period_ms = (elapsed_ms == 0U) ? 1U : elapsed_ms;

  p_term_rpm_x10 = ((int32_t)vehicle_error_x100 *
                    line_follow_pid_parameters.kp_x1000 * 10L) /
                   LINE_FOLLOW_PID_PARAMETER_SCALE;

  if (controller->initialized == 0U)
  {
    delta_error_x100 = 0;
    controller->filtered_delta_error_x100 = 0;
    controller->initialized = 1U;
  }
  else
  {
    delta_error_x100 = (int32_t)vehicle_error_x100 -
                       (int32_t)controller->previous_error_x100;
    /* First-order IIR: 75% previous filtered delta + 25% new delta. */
    controller->filtered_delta_error_x100 =
        ((controller->filtered_delta_error_x100 * 3L) + delta_error_x100) / 4L;
  }
  controller->previous_error_x100 = vehicle_error_x100;

  /* Keep the full numerator until the final division:
   * D_X10 = filtered_delta * (1000 ms/s) * Kd * (10 x RPM)
   *         / (elapsed_ms * Kd_denominator). */
  d_term_rpm_x10 = (int32_t)(
      ((int64_t)controller->filtered_delta_error_x100 * 1000LL *
       (int64_t)line_follow_pid_parameters.kd_x1000 * 10LL) /
      ((int64_t)d_period_ms *
       (int64_t)LINE_FOLLOW_PID_PARAMETER_SCALE));
  d_term_rpm_x10 = LineFollowPID_Clamp(d_term_rpm_x10,
                                       LINE_FOLLOW_PID_D_LIMIT_RPM_X10);

  proposed_integral_error = controller->integral_error +
                             (int32_t)vehicle_error_x100;
  proposed_integral_error = LineFollowPID_Clamp(
      proposed_integral_error, LINE_FOLLOW_PID_INTEGRAL_LIMIT);
  proposed_i_term_rpm_x10 = LineFollowPID_CalculateITerm(proposed_integral_error);
  unsaturated_correction_rpm_x10 = p_term_rpm_x10 +
                                   proposed_i_term_rpm_x10 +
                                   d_term_rpm_x10;

  /* Conditional integration: do not keep integrating when the proposed
   * output is saturated in the same direction as the present error. */
  if ((unsaturated_correction_rpm_x10 > LINE_FOLLOW_PID_MAX_CORRECTION_RPM_X10 &&
       vehicle_error_x100 > 0) ||
      (unsaturated_correction_rpm_x10 < -LINE_FOLLOW_PID_MAX_CORRECTION_RPM_X10 &&
       vehicle_error_x100 < 0))
  {
    i_term_rpm_x10 = LineFollowPID_CalculateITerm(controller->integral_error);
  }
  else
  {
    controller->integral_error = proposed_integral_error;
    i_term_rpm_x10 = proposed_i_term_rpm_x10;
  }

  final_correction_rpm_x10 = LineFollowPID_Clamp(
      p_term_rpm_x10 + i_term_rpm_x10 + d_term_rpm_x10,
      LINE_FOLLOW_PID_MAX_CORRECTION_RPM_X10);

  output->base_rpm_x10 = LINE_FOLLOW_PID_BASE_RPM_X10;
  output->delta_error_x100 = delta_error_x100;
  output->filtered_delta_error_x100 = controller->filtered_delta_error_x100;
  output->p_term_rpm_x10 = p_term_rpm_x10;
  output->d_term_rpm_x10 = d_term_rpm_x10;
  output->i_term_rpm_x10 = i_term_rpm_x10;
  output->position_correction_rpm_x10 = final_correction_rpm_x10;
  output->heading_correction_rpm_x10 = 0;
  output->final_correction_rpm_x10 = final_correction_rpm_x10;
  output->left_target_rpm_x10 = LINE_FOLLOW_PID_BASE_RPM_X10 + final_correction_rpm_x10;
  output->right_target_rpm_x10 = LINE_FOLLOW_PID_BASE_RPM_X10 - final_correction_rpm_x10;
  output->integral_error = controller->integral_error;
}

void LineFollowPID_ApplyHeadingP(LineFollowPID_Output_t *output,
                                 int16_t heading_error_x100)
{
  int32_t heading_correction_rpm_x10;

  if (output == 0)
  {
    return;
  }

  /* Kh is RPM / ERR_X100. Keep x10 RPM scaling until the final division. */
  heading_correction_rpm_x10 = ((int32_t)heading_error_x100 *
                                line_follow_pid_parameters.kh_x1000 * 10L) /
                               LINE_FOLLOW_PID_PARAMETER_SCALE;

  output->position_correction_rpm_x10 = output->final_correction_rpm_x10;
  output->heading_correction_rpm_x10 = heading_correction_rpm_x10;
  output->final_correction_rpm_x10 = LineFollowPID_Clamp(
      output->position_correction_rpm_x10 + heading_correction_rpm_x10,
      LINE_FOLLOW_PID_MAX_CORRECTION_RPM_X10);
  output->left_target_rpm_x10 = LINE_FOLLOW_PID_BASE_RPM_X10 +
                                output->final_correction_rpm_x10;
  output->right_target_rpm_x10 = LINE_FOLLOW_PID_BASE_RPM_X10 -
                                 output->final_correction_rpm_x10;
}
