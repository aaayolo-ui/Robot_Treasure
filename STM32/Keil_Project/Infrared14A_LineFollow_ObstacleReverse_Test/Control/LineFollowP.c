#include "LineFollowP.h"

void LineFollowP_Calculate(int16_t vehicle_error_x100, LineFollowP_Output_t *output)
{
  int32_t correction_rpm_x10;

  if (output == 0)
  {
    return;
  }

  /* GRAY_KP=0.015 RPM per error_x100, converted here to RPM x10. */
  correction_rpm_x10 = ((int32_t)vehicle_error_x100 *
                        LINE_FOLLOW_P_GRAY_KP_NUM * 10L) /
                       LINE_FOLLOW_P_GRAY_KP_DEN;

  if (correction_rpm_x10 > LINE_FOLLOW_P_MAX_CORRECTION_RPM_X10)
  {
    correction_rpm_x10 = LINE_FOLLOW_P_MAX_CORRECTION_RPM_X10;
  }
  else if (correction_rpm_x10 < -LINE_FOLLOW_P_MAX_CORRECTION_RPM_X10)
  {
    correction_rpm_x10 = -LINE_FOLLOW_P_MAX_CORRECTION_RPM_X10;
  }

  output->base_rpm_x10 = LINE_FOLLOW_P_BASE_RPM_X10;
  output->correction_rpm_x10 = correction_rpm_x10;
  output->left_target_rpm_x10 = LINE_FOLLOW_P_BASE_RPM_X10 + correction_rpm_x10;
  output->right_target_rpm_x10 = LINE_FOLLOW_P_BASE_RPM_X10 - correction_rpm_x10;
}
