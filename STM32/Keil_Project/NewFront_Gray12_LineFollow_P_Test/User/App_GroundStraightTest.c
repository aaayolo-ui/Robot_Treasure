#include "App_GroundStraightTest.h"
#include "ChassisSpeedControl.h"
#include "Key.h"
#include "UartDebug.h"
#include "main.h"

#define SPEED_CONTROL_PERIOD_MS       100U
#define GROUND_REPORT_PERIOD_MS       100U
#define GROUND_TEST_TARGET_RPM_X10    800L
#define GROUND_TARGET_STEP_RPM_X10    40L
#define GROUND_TEST_MAX_RUN_MS        3000U
#define GROUND_RECORD_CAPACITY         32U
#define PI_LONG_SATURATION_SAMPLES      5U

typedef struct
{
  uint32_t elapsed_ms;
  int32_t target_rpm_x10;
  int32_t a_rpm_x10;
  int32_t b_rpm_x10;
  int32_t c_rpm_x10;
  int32_t d_rpm_x10;
  uint16_t a_pwm;
  uint16_t b_pwm;
  uint16_t c_pwm;
  uint16_t d_pwm;
  int32_t left_avg_rpm_x10;
  int32_t right_avg_rpm_x10;
  uint8_t a_saturated;
  uint8_t b_saturated;
  uint8_t c_saturated;
  uint8_t d_saturated;
} GroundTestRecord_t;

static int32_t active_target_rpm_x10;
static uint32_t run_start_tick;
static uint32_t last_control_tick;
static uint32_t last_report_tick;
static uint8_t stall_announced;
static GroundTestRecord_t ground_records[GROUND_RECORD_CAPACITY];
static uint8_t ground_record_count;
static uint8_t ground_record_valid;

static void App_SplitRpmX10(int32_t rpm_x10, const char **sign,
                            long *integer, long *decimal)
{
  int64_t value = (int64_t)rpm_x10;
  int64_t absolute_value = (value < 0) ? -value : value;

  *sign = (value < 0) ? "-" : "";
  *integer = (long)(absolute_value / 10LL);
  *decimal = (long)(absolute_value % 10LL);
}

static void App_PrintRpmValue(int32_t rpm_x10)
{
  const char *sign;
  long integer;
  long decimal;

  App_SplitRpmX10(rpm_x10, &sign, &integer, &decimal);
  UartDebug_Printf("%s%ld.%ld", sign, integer, decimal);
}

static void App_PrintChannel(const char *channel, ChassisWheelId_t wheel)
{
  const ChassisWheelFeedback_t *feedback = Chassis_GetWheelFeedback(wheel);
  const WheelSpeedPi_t *controller = ChassisSpeedControl_GetController(wheel);
  int32_t output = WheelSpeedPi_GetOutput(controller);
  long pwm = (long)((output < 0) ? -output : output);

  UartDebug_Printf("%s_RPM=", channel);
  App_PrintRpmValue(feedback->rpm_x10);
  UartDebug_Printf(" %s_PWM=%ld", channel, pwm);
}

static uint16_t App_AbsolutePwm(int32_t pwm)
{
  int64_t value = (int64_t)pwm;

  if (value < 0)
  {
    value = -value;
  }
  return (value > 65535LL) ? 65535U : (uint16_t)value;
}

static void App_ResetRecords(void)
{
  ground_record_count = 0U;
  ground_record_valid = 0U;
}

static void App_CaptureRecord(uint32_t now)
{
  GroundTestRecord_t *record;
  const ChassisWheelFeedback_t *a_feedback;
  const ChassisWheelFeedback_t *b_feedback;
  const ChassisWheelFeedback_t *c_feedback;
  const ChassisWheelFeedback_t *d_feedback;
  const WheelSpeedPi_t *a;
  const WheelSpeedPi_t *b;
  const WheelSpeedPi_t *c;
  const WheelSpeedPi_t *d;

  if (ground_record_count >= GROUND_RECORD_CAPACITY)
  {
    return;
  }

  a_feedback = Chassis_GetWheelFeedback(CHASSIS_WHEEL_FRONT_RIGHT);
  b_feedback = Chassis_GetWheelFeedback(CHASSIS_WHEEL_FRONT_LEFT);
  c_feedback = Chassis_GetWheelFeedback(CHASSIS_WHEEL_REAR_RIGHT);
  d_feedback = Chassis_GetWheelFeedback(CHASSIS_WHEEL_REAR_LEFT);
  a = ChassisSpeedControl_GetController(CHASSIS_WHEEL_FRONT_RIGHT);
  b = ChassisSpeedControl_GetController(CHASSIS_WHEEL_FRONT_LEFT);
  c = ChassisSpeedControl_GetController(CHASSIS_WHEEL_REAR_RIGHT);
  d = ChassisSpeedControl_GetController(CHASSIS_WHEEL_REAR_LEFT);
  record = &ground_records[ground_record_count];

  record->elapsed_ms = (uint32_t)(now - run_start_tick);
  record->target_rpm_x10 = active_target_rpm_x10;
  record->a_rpm_x10 = a_feedback->rpm_x10;
  record->b_rpm_x10 = b_feedback->rpm_x10;
  record->c_rpm_x10 = c_feedback->rpm_x10;
  record->d_rpm_x10 = d_feedback->rpm_x10;
  record->a_pwm = App_AbsolutePwm(WheelSpeedPi_GetOutput(a));
  record->b_pwm = App_AbsolutePwm(WheelSpeedPi_GetOutput(b));
  record->c_pwm = App_AbsolutePwm(WheelSpeedPi_GetOutput(c));
  record->d_pwm = App_AbsolutePwm(WheelSpeedPi_GetOutput(d));
  record->left_avg_rpm_x10 = Chassis_GetLeftAverageRpmX10();
  record->right_avg_rpm_x10 = Chassis_GetRightAverageRpmX10();
  record->a_saturated = a->saturated;
  record->b_saturated = b->saturated;
  record->c_saturated = c->saturated;
  record->d_saturated = d->saturated;
  ground_record_count++;
  ground_record_valid = 1U;
}

static void App_PrintRecord(const GroundTestRecord_t *record, uint8_t index)
{
  UartDebug_Printf("RECORD=%u T_MS=%lu TARGET_RPM=",
                   (unsigned int)index, (unsigned long)record->elapsed_ms);
  App_PrintRpmValue(record->target_rpm_x10);
  UartDebug_SendString("\r\nA_RPM=");
  App_PrintRpmValue(record->a_rpm_x10);
  UartDebug_Printf(" A_PWM=%u B_RPM=", (unsigned int)record->a_pwm);
  App_PrintRpmValue(record->b_rpm_x10);
  UartDebug_Printf(" B_PWM=%u C_RPM=", (unsigned int)record->b_pwm);
  App_PrintRpmValue(record->c_rpm_x10);
  UartDebug_Printf(" C_PWM=%u D_RPM=", (unsigned int)record->c_pwm);
  App_PrintRpmValue(record->d_rpm_x10);
  UartDebug_Printf(" D_PWM=%u\r\nLEFT_AVG_RPM=", (unsigned int)record->d_pwm);
  App_PrintRpmValue(record->left_avg_rpm_x10);
  UartDebug_SendString(" RIGHT_AVG_RPM=");
  App_PrintRpmValue(record->right_avg_rpm_x10);
  UartDebug_Printf(" PI_SATURATION: A=%u B=%u C=%u D=%u\r\n",
                   (unsigned int)record->a_saturated,
                   (unsigned int)record->b_saturated,
                   (unsigned int)record->c_saturated,
                   (unsigned int)record->d_saturated);
}

static uint8_t App_HasLongSaturation(uint8_t channel)
{
  uint8_t index;
  uint8_t consecutive = 0U;

  for (index = 0U; index < ground_record_count; index++)
  {
    const GroundTestRecord_t *record = &ground_records[index];
    uint8_t saturated = (channel == 0U) ? record->a_saturated :
                        (channel == 1U) ? record->b_saturated :
                        (channel == 2U) ? record->c_saturated : record->d_saturated;

    if (saturated != 0U)
    {
      consecutive++;
      if (consecutive >= PI_LONG_SATURATION_SAMPLES)
      {
        return 1U;
      }
    }
    else
    {
      consecutive = 0U;
    }
  }
  return 0U;
}

static void App_PrintSummary(void)
{
  uint8_t index;
  int64_t a_rpm_sum = 0;
  int64_t b_rpm_sum = 0;
  int64_t c_rpm_sum = 0;
  int64_t d_rpm_sum = 0;
  int64_t a_pwm_sum = 0;
  int64_t b_pwm_sum = 0;
  int64_t c_pwm_sum = 0;
  int64_t d_pwm_sum = 0;
  int64_t left_rpm_sum = 0;
  int64_t right_rpm_sum = 0;
  uint16_t b_max_pwm = 0U;
  uint8_t long_a;
  uint8_t long_b;
  uint8_t long_c;
  uint8_t long_d;

  if (ground_record_valid == 0U)
  {
    UartDebug_SendString("SUMMARY: NO_COMPLETED_TEST_RECORD\r\n");
    return;
  }

  for (index = 0U; index < ground_record_count; index++)
  {
    const GroundTestRecord_t *record = &ground_records[index];
    a_rpm_sum += record->a_rpm_x10;
    b_rpm_sum += record->b_rpm_x10;
    c_rpm_sum += record->c_rpm_x10;
    d_rpm_sum += record->d_rpm_x10;
    a_pwm_sum += record->a_pwm;
    b_pwm_sum += record->b_pwm;
    c_pwm_sum += record->c_pwm;
    d_pwm_sum += record->d_pwm;
    left_rpm_sum += record->left_avg_rpm_x10;
    right_rpm_sum += record->right_avg_rpm_x10;
    if (record->b_pwm > b_max_pwm)
    {
      b_max_pwm = record->b_pwm;
    }
  }

  long_a = App_HasLongSaturation(0U);
  long_b = App_HasLongSaturation(1U);
  long_c = App_HasLongSaturation(2U);
  long_d = App_HasLongSaturation(3U);
  UartDebug_Printf("SUMMARY: SAMPLES=%u\r\nA_AVG_RPM=", (unsigned int)ground_record_count);
  App_PrintRpmValue((int32_t)(a_rpm_sum / (int64_t)ground_record_count));
  UartDebug_SendString(" B_AVG_RPM=");
  App_PrintRpmValue((int32_t)(b_rpm_sum / (int64_t)ground_record_count));
  UartDebug_SendString(" C_AVG_RPM=");
  App_PrintRpmValue((int32_t)(c_rpm_sum / (int64_t)ground_record_count));
  UartDebug_SendString(" D_AVG_RPM=");
  App_PrintRpmValue((int32_t)(d_rpm_sum / (int64_t)ground_record_count));
  UartDebug_Printf("\r\nA_AVG_PWM=%ld B_AVG_PWM=%ld C_AVG_PWM=%ld D_AVG_PWM=%ld B_MAX_PWM=%u\r\n",
                   (long)(a_pwm_sum / (int64_t)ground_record_count),
                   (long)(b_pwm_sum / (int64_t)ground_record_count),
                   (long)(c_pwm_sum / (int64_t)ground_record_count),
                   (long)(d_pwm_sum / (int64_t)ground_record_count),
                   (unsigned int)b_max_pwm);
  UartDebug_SendString("LEFT_AVG_RPM=");
  App_PrintRpmValue((int32_t)(left_rpm_sum / (int64_t)ground_record_count));
  UartDebug_SendString(" RIGHT_AVG_RPM=");
  App_PrintRpmValue((int32_t)(right_rpm_sum / (int64_t)ground_record_count));
  UartDebug_SendString(" LEFT_RIGHT_DIFF_RPM=");
  App_PrintRpmValue((int32_t)((left_rpm_sum - right_rpm_sum) /
                               (int64_t)ground_record_count));
  UartDebug_Printf("\r\nPI_LONG_SATURATION: A=%u B=%u C=%u D=%u ANY=%u (>=500ms continuous)\r\n",
                   (unsigned int)long_a, (unsigned int)long_b,
                   (unsigned int)long_c, (unsigned int)long_d,
                   (unsigned int)((long_a != 0U) || (long_b != 0U) ||
                                  (long_c != 0U) || (long_d != 0U)));
}

static void App_PrintLastTestRecord(void)
{
  uint8_t index;

  if (ground_record_valid == 0U)
  {
    UartDebug_SendString("NO_PREVIOUS_TEST_RECORD\r\n");
    return;
  }

  UartDebug_Printf("LAST_TEST_RECORD_BEGIN: SAMPLES=%u\r\n",
                   (unsigned int)ground_record_count);
  for (index = 0U; index < ground_record_count; index++)
  {
    App_PrintRecord(&ground_records[index], index);
  }
  App_PrintSummary();
  UartDebug_SendString("LAST_TEST_RECORD_END\r\n");
}

static void App_StopGroundTest(const char *reason)
{
  active_target_rpm_x10 = 0;
  ChassisSpeedControl_Disable();
  UartDebug_SendString(reason);
}

static void App_StartGroundTest(void)
{
  App_ResetRecords();
  ChassisSpeedControl_Enable();
  active_target_rpm_x10 = 0;
  run_start_tick = HAL_GetTick();
  stall_announced = 0U;
  UartDebug_SendString("New-front ground straight PI test started.\r\n");
  UartDebug_SendString("FORWARD_ONLY, target=80.0 RPM, ramp=4.0 RPM/100ms.\r\n");
  UartDebug_SendString("Automatic stop after 3.0 seconds.\r\n");
}

static void App_UpdateTargetRamp(void)
{
  if (active_target_rpm_x10 < GROUND_TEST_TARGET_RPM_X10)
  {
    active_target_rpm_x10 += GROUND_TARGET_STEP_RPM_X10;
    if (active_target_rpm_x10 > GROUND_TEST_TARGET_RPM_X10)
    {
      active_target_rpm_x10 = GROUND_TEST_TARGET_RPM_X10;
    }
  }
  ChassisSpeedControl_SetWheelTargets(active_target_rpm_x10,
                                      active_target_rpm_x10,
                                      active_target_rpm_x10,
                                      active_target_rpm_x10);
}

static void App_ProcessKeys(void)
{
  KeyEvent_t event = Key_GetEvent();

  if (event == KEY_EVENT_KEY2)
  {
    if (ChassisSpeedControl_IsEnabled() == 0U)
    {
      App_StartGroundTest();
    }
    else
    {
      App_StopGroundTest("Ground straight PI test stopped by KEY2.\r\n");
    }
  }
  else if (event == KEY_EVENT_KEY1)
  {
    if (ChassisSpeedControl_IsEnabled() != 0U)
    {
      App_StopGroundTest("Ground straight PI test stopped by KEY1.\r\n");
    }
    else
    {
      App_PrintLastTestRecord();
    }
  }
  else if (event == KEY_EVENT_WK_UP)
  {
    App_StopGroundTest("Emergency stop by WK_UP. Motor driver disabled.\r\n");
  }
  else
  {
  }
}

static void App_Report(void)
{
  uint32_t now = HAL_GetTick();
  const WheelSpeedPi_t *a;
  const WheelSpeedPi_t *b;
  const WheelSpeedPi_t *c;
  const WheelSpeedPi_t *d;

  if ((uint32_t)(now - last_report_tick) < GROUND_REPORT_PERIOD_MS)
  {
    return;
  }
  last_report_tick = now;
  a = ChassisSpeedControl_GetController(CHASSIS_WHEEL_FRONT_RIGHT);
  b = ChassisSpeedControl_GetController(CHASSIS_WHEEL_FRONT_LEFT);
  c = ChassisSpeedControl_GetController(CHASSIS_WHEEL_REAR_RIGHT);
  d = ChassisSpeedControl_GetController(CHASSIS_WHEEL_REAR_LEFT);

  UartDebug_SendString("TARGET_RPM=");
  App_PrintRpmValue(active_target_rpm_x10);
  UartDebug_SendString("\r\n");
  App_PrintChannel("A", CHASSIS_WHEEL_FRONT_RIGHT);
  UartDebug_SendString(" / ");
  App_PrintChannel("B", CHASSIS_WHEEL_FRONT_LEFT);
  UartDebug_SendString(" / ");
  App_PrintChannel("C", CHASSIS_WHEEL_REAR_RIGHT);
  UartDebug_SendString(" / ");
  App_PrintChannel("D", CHASSIS_WHEEL_REAR_LEFT);
  UartDebug_SendString("\r\nLEFT_AVG_RPM=");
  App_PrintRpmValue(Chassis_GetLeftAverageRpmX10());
  UartDebug_SendString(" RIGHT_AVG_RPM=");
  App_PrintRpmValue(Chassis_GetRightAverageRpmX10());
  UartDebug_Printf("\r\nPI_SATURATION: A=%u B=%u C=%u D=%u\r\n",
                   (unsigned int)a->saturated, (unsigned int)b->saturated,
                   (unsigned int)c->saturated, (unsigned int)d->saturated);
}

void App_GroundStraightTest_Init(void)
{
  Key_Init();
  ChassisSpeedControl_Init();
  active_target_rpm_x10 = 0;
  run_start_tick = HAL_GetTick();
  last_control_tick = run_start_tick;
  last_report_tick = run_start_tick;
  stall_announced = 0U;
  App_ResetRecords();

  UartDebug_SendString("\r\nNewFront_FourMotor_GroundStraight_PI_Test started.\r\n");
  UartDebug_SendString("PHYSICAL_MAP: B=FL A=FR D=RL C=RR\r\n");
  UartDebug_SendString("FORWARD_MAP: B=REV A=REV D=FWD C=FWD\r\n");
  UartDebug_SendString("ENCODER_SIGN: B=+1 A=-1 D=+1 C=-1\r\n");
  UartDebug_SendString("B_FIXED_COMPENSATION=NONE\r\n");
  UartDebug_SendString("FOUR_INDEPENDENT_SPEED_PI=ENABLED\r\n");
  UartDebug_SendString("TARGET=80.0 RPM RAMP=4.0 RPM/100ms PWM_MAX=300 RUN_MAX=3.0s\r\n");
  UartDebug_SendString("KEY2=start/stop, KEY1=stop while running or print last record, WK_UP=emergency stop.\r\n");
  UartDebug_SendString("Motor driver is disabled at startup.\r\n");
}

void App_GroundStraightTest_Task(void)
{
  uint32_t now;
  uint32_t elapsed_ms;
  uint8_t capture_running;

  App_ProcessKeys();
  now = HAL_GetTick();
  if ((uint32_t)(now - last_control_tick) >= SPEED_CONTROL_PERIOD_MS)
  {
    capture_running = 0U;
    elapsed_ms = (uint32_t)(now - last_control_tick);
    last_control_tick = now;
    if (ChassisSpeedControl_IsEnabled() != 0U)
    {
      if ((uint32_t)(now - run_start_tick) >= GROUND_TEST_MAX_RUN_MS)
      {
        /* Preserve the final active PI state at the 3-second safety boundary. */
        App_CaptureRecord(now);
        App_StopGroundTest("Ground straight PI test automatically stopped after 3.0 seconds.\r\n");
      }
      else
      {
        App_UpdateTargetRamp();
        capture_running = 1U;
      }
    }
    ChassisSpeedControl_Update(elapsed_ms);
    if (capture_running != 0U)
    {
      App_CaptureRecord(now);
    }
  }
  if ((ChassisSpeedControl_HasStall() != 0U) && (stall_announced == 0U))
  {
    stall_announced = 1U;
    UartDebug_SendString("Wheel stall detected. All PWM cleared and motor driver disabled.\r\n");
  }
  App_Report();
}
