#include "LineFollowLog.h"
#include "UartDebug.h"

static LineFollowLog_Record_t records[LINE_FOLLOW_LOG_CAPACITY];
static uint16_t record_count;

static const char *LineFollowLog_FollowStateName(uint8_t state)
{
  static const char *const names[] = {"IDLE", "FRONT_ONLY", "DUAL"};
  return ((uint32_t)state < (sizeof(names) / sizeof(names[0]))) ?
         names[state] : "UNKNOWN";
}

static const char *LineFollowLog_GrayStateName(uint8_t state)
{
  static const char *const names[] = {
      "UNKNOWN", "VALID", "NO_LINE", "ALL_BLACK", "WIDE_BLACK",
      "I2C_ERROR", "UNAVAILABLE"};
  return ((uint32_t)state < (sizeof(names) / sizeof(names[0]))) ?
         names[state] : "UNKNOWN";
}

void LineFollowLog_Reset(void)
{
  record_count = 0U;
}

void LineFollowLog_Append(const LineFollowLog_Record_t *record)
{
  if ((record == 0) || (record_count >= LINE_FOLLOW_LOG_CAPACITY))
  {
    return;
  }
  records[record_count] = *record;
  record_count++;
}

uint16_t LineFollowLog_GetCount(void)
{
  return record_count;
}

void LineFollowLog_Export(void)
{
  uint16_t index;
  const LineFollowLog_Record_t *record;

  if (record_count == 0U)
  {
    UartDebug_SendString("LINE_LOG_EMPTY\r\n");
    return;
  }

  UartDebug_Printf("LINE_LOG_BEGIN COUNT=%u CAPACITY=%u\r\n",
                   (unsigned int)record_count,
                   (unsigned int)LINE_FOLLOW_LOG_CAPACITY);
  UartDebug_SendString(
      "LINE_LOG_FIELDS=INDEX TIME_MS STATE FRONT_STATE REAR_STATE "
      "front_error rear_error position_error heading_error P I D "
      "correction left_target right_target left_actual right_actual\r\n");
  UartDebug_SendString(
      "LINE_LOG_MOTOR_FIELDS=A_target A_actual A_pwm B_target B_actual B_pwm "
      "C_target C_actual C_pwm D_target D_actual D_pwm\r\n");
  for (index = 0U; index < record_count; index++)
  {
    record = &records[index];
    UartDebug_Printf(
        "LINE_LOG=%u TIME_MS=%lu STATE=%s FRONT_STATE=%s REAR_STATE=%s "
        "front_error=%d rear_error=%d position_error=%d heading_error=%d "
        "P=%ld I=%ld D=%ld correction=%ld left_target=%ld "
        "right_target=%ld left_actual=%ld right_actual=%ld\r\n",
        (unsigned int)index, (unsigned long)record->elapsed_ms,
        LineFollowLog_FollowStateName(record->follow_state),
        LineFollowLog_GrayStateName(record->front_state),
        LineFollowLog_GrayStateName(record->rear_state),
        (int)record->front_error_x100, (int)record->rear_error_x100,
        (int)record->position_error_x100, (int)record->heading_error_x100,
        (long)record->p_term_rpm_x10, (long)record->i_term_rpm_x10,
        (long)record->d_term_rpm_x10, (long)record->correction_rpm_x10,
        (long)record->left_target_rpm_x10,
        (long)record->right_target_rpm_x10,
        (long)record->left_actual_rpm_x10,
        (long)record->right_actual_rpm_x10);
    UartDebug_Printf(
        "MOTOR A=%ld,%ld,%ld B=%ld,%ld,%ld C=%ld,%ld,%ld "
        "D=%ld,%ld,%ld\r\n",
        (long)record->a_target_rpm_x10,
        (long)record->a_actual_rpm_x10,
        (long)record->a_pwm,
        (long)record->b_target_rpm_x10,
        (long)record->b_actual_rpm_x10,
        (long)record->b_pwm,
        (long)record->c_target_rpm_x10,
        (long)record->c_actual_rpm_x10,
        (long)record->c_pwm,
        (long)record->d_target_rpm_x10,
        (long)record->d_actual_rpm_x10,
        (long)record->d_pwm);
  }
  UartDebug_SendString("LINE_LOG_END\r\n");
}
