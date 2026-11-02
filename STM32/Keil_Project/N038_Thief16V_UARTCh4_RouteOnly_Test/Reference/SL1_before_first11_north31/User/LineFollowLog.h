#ifndef LINE_FOLLOW_LOG_H
#define LINE_FOLLOW_LOG_H

#include <stdint.h>

#define LINE_FOLLOW_LOG_CAPACITY 256U

typedef struct
{
  uint32_t elapsed_ms;
  int16_t front_error_x100;
  int16_t rear_error_x100;
  int16_t position_error_x100;
  int16_t heading_error_x100;
  int32_t p_term_rpm_x10;
  int32_t i_term_rpm_x10;
  int32_t d_term_rpm_x10;
  int32_t correction_rpm_x10;
  int32_t left_target_rpm_x10;
  int32_t right_target_rpm_x10;
  int32_t left_actual_rpm_x10;
  int32_t right_actual_rpm_x10;
  int32_t a_target_rpm_x10;
  int32_t a_actual_rpm_x10;
  int32_t a_pwm;
  int32_t b_target_rpm_x10;
  int32_t b_actual_rpm_x10;
  int32_t b_pwm;
  int32_t c_target_rpm_x10;
  int32_t c_actual_rpm_x10;
  int32_t c_pwm;
  int32_t d_target_rpm_x10;
  int32_t d_actual_rpm_x10;
  int32_t d_pwm;
  uint8_t follow_state;
  uint8_t front_state;
  uint8_t rear_state;
} LineFollowLog_Record_t;

void LineFollowLog_Reset(void);
void LineFollowLog_Append(const LineFollowLog_Record_t *record);
void LineFollowLog_Export(void);
uint16_t LineFollowLog_GetCount(void);

#endif /* LINE_FOLLOW_LOG_H */
