#include "App_LineFollowPID.h"
#include "ChassisSpeedControl.h"
#include "gray_coordinate.h"
#include "gray_position.h"
#include "Key.h"
#include "LineFollowPID.h"
#include "LineFollowLog.h"
#include "main.h"
#include "MotorDriver.h"
#include "nchd12.h"
#include "nchd12_rear.h"
#include "SystemTime.h"
#include "TftPidPage.h"
#include "UartDebug.h"

#define APP_MAX_RUN_MS          20000U
#define APP_REPORT_PERIOD_MS      200U
#define APP_LOG_PERIOD_MS         100U
#define APP_REAR_VALID_REQUIRED     3U

typedef enum
{
  APP_GRAY_UNKNOWN = 0,
  APP_GRAY_VALID,
  APP_GRAY_NO_LINE,
  APP_GRAY_ALL_BLACK,
  APP_GRAY_WIDE_BLACK,
  APP_GRAY_I2C_ERROR,
  APP_GRAY_UNAVAILABLE
} AppGrayState_t;

typedef enum
{
  APP_FOLLOW_IDLE = 0,
  APP_FOLLOW_FRONT_ONLY,
  APP_FOLLOW_DUAL
} AppFollowState_t;

typedef enum
{
  APP_STOP_KEY2 = 0,
  APP_STOP_KEY1,
  APP_STOP_WK_UP,
  APP_STOP_TIMEOUT,
  APP_STOP_FRONT_I2C,
  APP_STOP_FRONT_LINE,
  APP_STOP_STALL
} AppStopReason_t;

typedef struct
{
  uint16_t front_raw12;
  uint16_t rear_raw12;
  GrayPosition_Result_t front_position;
  GrayPosition_Result_t rear_position;
  int16_t front_error_x100;
  int16_t rear_error_x100;
  int16_t position_error_x100;
  int16_t heading_error_x100;
} DualGraySample_t;

static LineFollowPID_Controller_t line_follow_pid;
static uint8_t app_running;
static uint8_t rear_hw_ready;
static uint8_t rear_valid_count;
static AppFollowState_t follow_state;
static AppGrayState_t front_state;
static AppGrayState_t rear_state;
static uint32_t run_start_tick;
static uint32_t last_control_tick;
static uint32_t last_log_tick;
static uint32_t last_report_tick;
static LineFollowPID_ParameterId_t selected_parameter;
static uint8_t startup_motor_diagnostic_pending;
static const char *app_status_text;

static const char *App_StopName(AppStopReason_t reason)
{
  static const char *const names[] = {
      "KEY2", "KEY1", "WK_UP", "TIMEOUT", "FRONT_I2C",
      "FRONT_LINE", "STALL"};
  return ((uint32_t)reason < (sizeof(names) / sizeof(names[0]))) ?
         names[reason] : "UNKNOWN";
}

static const char *App_GrayStateName(AppGrayState_t state)
{
  static const char *const names[] = {
      "UNKNOWN", "VALID", "NO_LINE", "ALL_BLACK", "WIDE_BLACK",
      "I2C_ERROR", "UNAVAILABLE"};
  return ((uint32_t)state < (sizeof(names) / sizeof(names[0]))) ?
         names[state] : "UNKNOWN";
}

static const char *App_FrontStatusText(AppGrayState_t state)
{
  switch (state)
  {
    case APP_GRAY_VALID:
      return "OK";
    case APP_GRAY_NO_LINE:
      return "FRONT NO_LINE";
    case APP_GRAY_ALL_BLACK:
      return "FRONT ALL_BLACK";
    case APP_GRAY_WIDE_BLACK:
      return "FRONT WIDE_BLACK";
    case APP_GRAY_I2C_ERROR:
      return "FRONT I2C";
    default:
      return "FRONT ERROR";
  }
}

static const char *App_FollowStateName(AppFollowState_t state)
{
  static const char *const names[] = {"IDLE", "FRONT_ONLY", "DUAL"};
  return ((uint32_t)state < (sizeof(names) / sizeof(names[0]))) ?
         names[state] : "IDLE";
}

static const char *App_ParameterName(LineFollowPID_ParameterId_t parameter)
{
  static const char *const names[] = {"KP", "KI", "KD", "KH"};

  if ((uint32_t)parameter >= (uint32_t)LINE_FOLLOW_PARAMETER_COUNT)
  {
    return names[0];
  }
  return names[parameter];
}

static void App_ReportParameters(void)
{
  const LineFollowPID_Parameters_t *parameters =
      LineFollowPID_GetParameters();

  UartDebug_Printf(
      "PARAM KP=%ld.%03ld KI=%ld.%03ld KD=%ld.%03ld KH=%ld.%03ld SEL=%s\r\n",
      (long)(parameters->kp_x1000 / LINE_FOLLOW_PID_PARAMETER_SCALE),
      (long)(parameters->kp_x1000 % LINE_FOLLOW_PID_PARAMETER_SCALE),
      (long)(parameters->ki_x1000 / LINE_FOLLOW_PID_PARAMETER_SCALE),
      (long)(parameters->ki_x1000 % LINE_FOLLOW_PID_PARAMETER_SCALE),
      (long)(parameters->kd_x1000 / LINE_FOLLOW_PID_PARAMETER_SCALE),
      (long)(parameters->kd_x1000 % LINE_FOLLOW_PID_PARAMETER_SCALE),
      (long)(parameters->kh_x1000 / LINE_FOLLOW_PID_PARAMETER_SCALE),
      (long)(parameters->kh_x1000 % LINE_FOLLOW_PID_PARAMETER_SCALE),
      App_ParameterName(selected_parameter));
}

static AppGrayState_t App_ClassifyPosition(
    const GrayPosition_Result_t *position)
{
  if (position == 0)
  {
    return APP_GRAY_I2C_ERROR;
  }
  if (position->status == GRAY_POSITION_STATUS_NO_LINE)
  {
    return APP_GRAY_NO_LINE;
  }
  if (position->status == GRAY_POSITION_STATUS_ALL_BLACK)
  {
    return APP_GRAY_ALL_BLACK;
  }
  if ((position->valid == 0U) || (position->black_count > 4U))
  {
    return APP_GRAY_WIDE_BLACK;
  }
  return APP_GRAY_VALID;
}

static AppStopReason_t App_FrontStopReason(AppGrayState_t state)
{
  return (state == APP_GRAY_I2C_ERROR) ? APP_STOP_FRONT_I2C :
                                         APP_STOP_FRONT_LINE;
}

static void App_ResetSample(DualGraySample_t *sample)
{
  sample->front_raw12 = 0U;
  sample->rear_raw12 = 0U;
  sample->front_error_x100 = 0;
  sample->rear_error_x100 = 0;
  sample->position_error_x100 = 0;
  sample->heading_error_x100 = 0;
}

static void App_Stop(AppStopReason_t reason)
{
  if (reason == APP_STOP_FRONT_I2C)
  {
    app_status_text = "FRONT I2C";
  }
  else if (reason == APP_STOP_FRONT_LINE)
  {
    app_status_text = App_FrontStatusText(front_state);
  }
  else
  {
    app_status_text = "STOP";
  }
  ChassisSpeedControl_Disable();
  LineFollowPID_Reset(&line_follow_pid);
  app_running = 0U;
  follow_state = APP_FOLLOW_IDLE;
  rear_valid_count = 0U;
  startup_motor_diagnostic_pending = 0U;
  UartDebug_Printf("STOP=%s STATE=IDLE\r\n", App_StopName(reason));
}

static uint8_t App_InitializeGray(void)
{
  rear_hw_ready = 0U;
  NCHD12_Init();
  if ((NCHD12_ScanBus() != NCHD12_STATUS_OK) ||
      (NCHD12_ConfigureInputs() != NCHD12_STATUS_OK))
  {
    return 0U;
  }

  NCHD12Rear_Init();
  if (NCHD12Rear_ConfigureInputs() == NCHD12_STATUS_OK)
  {
    rear_hw_ready = 1U;
  }
  return 1U;
}

static AppGrayState_t App_ReadFront(DualGraySample_t *sample)
{
  uint16_t raw16;
  AppGrayState_t state;

  if (NCHD12_ReadRaw16(&raw16) != NCHD12_STATUS_OK)
  {
    return APP_GRAY_I2C_ERROR;
  }
  sample->front_raw12 = NCHD12_Extract12(raw16);
  GrayPosition_Calculate(sample->front_raw12, &sample->front_position);
  state = App_ClassifyPosition(&sample->front_position);
  if (state != APP_GRAY_VALID)
  {
    return state;
  }
  if (!GrayCoordinate_ToVehicleError(sample->front_position.position_x100,
                                     GRAY_MOUNT_P12_LEFT_P1_RIGHT,
                                     &sample->front_error_x100))
  {
    return APP_GRAY_I2C_ERROR;
  }
  return APP_GRAY_VALID;
}

static AppGrayState_t App_ReadRear(DualGraySample_t *sample)
{
  uint16_t raw16;
  AppGrayState_t state;

  if (rear_hw_ready == 0U)
  {
    if (NCHD12Rear_ConfigureInputs() != NCHD12_STATUS_OK)
    {
      return APP_GRAY_UNAVAILABLE;
    }
    rear_hw_ready = 1U;
  }
  if (NCHD12Rear_ReadRaw16(&raw16) != NCHD12_STATUS_OK)
  {
    rear_hw_ready = 0U;
    return APP_GRAY_I2C_ERROR;
  }
  sample->rear_raw12 = NCHD12_Extract12(raw16);
  GrayPosition_Calculate(sample->rear_raw12, &sample->rear_position);
  state = App_ClassifyPosition(&sample->rear_position);
  if (state != APP_GRAY_VALID)
  {
    return state;
  }
  if (!GrayCoordinate_ToVehicleError(sample->rear_position.position_x100,
                                     GRAY_MOUNT_P1_LEFT_P12_RIGHT,
                                     &sample->rear_error_x100))
  {
    return APP_GRAY_I2C_ERROR;
  }
  return APP_GRAY_VALID;
}

static void App_ReportFrontCheck(const DualGraySample_t *sample,
                                 AppGrayState_t state)
{
  UartDebug_Printf(
      "START_FRONT_CHECK I2C=%s FRONT_STATE=%s RAW12=%u ERROR=%d\r\n",
      (state == APP_GRAY_I2C_ERROR) ? "FAIL" : "OK",
      App_GrayStateName(state), (unsigned)sample->front_raw12,
      (int)sample->front_error_x100);
}

static void App_ReportMotorStartup(void)
{
  const WheelSpeedPi_t *a_controller;
  const WheelSpeedPi_t *b_controller;
  const WheelSpeedPi_t *c_controller;
  const WheelSpeedPi_t *d_controller;
  uint8_t stby;

  a_controller = ChassisSpeedControl_GetController(
      CHASSIS_WHEEL_FRONT_RIGHT);
  b_controller = ChassisSpeedControl_GetController(
      CHASSIS_WHEEL_FRONT_LEFT);
  c_controller = ChassisSpeedControl_GetController(
      CHASSIS_WHEEL_REAR_RIGHT);
  d_controller = ChassisSpeedControl_GetController(
      CHASSIS_WHEEL_REAR_LEFT);
  stby = (HAL_GPIO_ReadPin(MOTOR_STBY_GPIO_Port, MOTOR_STBY_Pin) ==
          GPIO_PIN_SET) ? 1U : 0U;
  UartDebug_Printf(
      "START_MOTOR STBY=%u ENABLE=%u A_TARGET=%ld A_PWM=%ld "
      "B_TARGET=%ld B_PWM=%ld C_TARGET=%ld C_PWM=%ld "
      "D_TARGET=%ld D_PWM=%ld\r\n",
      (unsigned)stby, (unsigned)MotorDriver_IsEnabled(),
      (long)WheelSpeedPi_GetTarget(a_controller),
      (long)WheelSpeedPi_GetOutput(a_controller),
      (long)WheelSpeedPi_GetTarget(b_controller),
      (long)WheelSpeedPi_GetOutput(b_controller),
      (long)WheelSpeedPi_GetTarget(c_controller),
      (long)WheelSpeedPi_GetOutput(c_controller),
      (long)WheelSpeedPi_GetTarget(d_controller),
      (long)WheelSpeedPi_GetOutput(d_controller));
}

static void App_UpdateErrors(DualGraySample_t *sample)
{
  if ((follow_state == APP_FOLLOW_DUAL) &&
      (rear_state == APP_GRAY_VALID))
  {
    sample->position_error_x100 = (int16_t)(
        ((int32_t)sample->front_error_x100 +
         (int32_t)sample->rear_error_x100) / 2L);
    sample->heading_error_x100 = (int16_t)(
        (int32_t)sample->front_error_x100 -
        (int32_t)sample->rear_error_x100);
  }
  else
  {
    sample->position_error_x100 = sample->front_error_x100;
    sample->heading_error_x100 = 0;
  }
}

static void App_Report(const DualGraySample_t *sample,
                       const LineFollowPID_Output_t *control)
{
  uint32_t now = SystemTime_GetMs();

  if ((uint32_t)(now - last_report_tick) < APP_REPORT_PERIOD_MS)
  {
    return;
  }
  last_report_tick = now;
  UartDebug_Printf(
      "STATE=%s FRONT=%s REAR=%s front_error=%d rear_error=%d position_error=%d heading_error=%d P=%ld I=%ld D=%ld correction=%ld left_target=%ld right_target=%ld\r\n",
      App_FollowStateName(follow_state), App_GrayStateName(front_state),
      App_GrayStateName(rear_state), (int)sample->front_error_x100,
      (int)sample->rear_error_x100, (int)sample->position_error_x100,
      (int)sample->heading_error_x100, (long)control->p_term_rpm_x10,
      (long)control->i_term_rpm_x10, (long)control->d_term_rpm_x10,
      (long)control->final_correction_rpm_x10,
      (long)control->left_target_rpm_x10, (long)control->right_target_rpm_x10);
}

static void App_StartReport(const char *result)
{
  UartDebug_Printf("%s FRONT=%s REAR=%s INITIAL_STATE=%s\r\n", result,
                   App_GrayStateName(front_state),
                   App_GrayStateName(rear_state),
                   App_FollowStateName(follow_state));
}

static void App_CaptureLog(uint32_t now,
                           const DualGraySample_t *sample,
                           const LineFollowPID_Output_t *control)
{
  LineFollowLog_Record_t record;
  const WheelSpeedPi_t *a_controller;
  const WheelSpeedPi_t *b_controller;
  const WheelSpeedPi_t *c_controller;
  const WheelSpeedPi_t *d_controller;
  const ChassisWheelFeedback_t *a_feedback;
  const ChassisWheelFeedback_t *b_feedback;
  const ChassisWheelFeedback_t *c_feedback;
  const ChassisWheelFeedback_t *d_feedback;

  record.elapsed_ms = (uint32_t)(now - run_start_tick);
  record.front_error_x100 = sample->front_error_x100;
  record.rear_error_x100 = sample->rear_error_x100;
  record.position_error_x100 = sample->position_error_x100;
  record.heading_error_x100 = sample->heading_error_x100;
  record.p_term_rpm_x10 = control->p_term_rpm_x10;
  record.i_term_rpm_x10 = control->i_term_rpm_x10;
  record.d_term_rpm_x10 = control->d_term_rpm_x10;
  record.correction_rpm_x10 = control->final_correction_rpm_x10;
  record.left_target_rpm_x10 = control->left_target_rpm_x10;
  record.right_target_rpm_x10 = control->right_target_rpm_x10;
  record.left_actual_rpm_x10 = Chassis_GetLeftAverageRpmX10();
  record.right_actual_rpm_x10 = Chassis_GetRightAverageRpmX10();

  /* Physical motor labels: A=FR, B=FL, C=RR, D=RL. */
  a_controller = ChassisSpeedControl_GetController(
      CHASSIS_WHEEL_FRONT_RIGHT);
  b_controller = ChassisSpeedControl_GetController(
      CHASSIS_WHEEL_FRONT_LEFT);
  c_controller = ChassisSpeedControl_GetController(
      CHASSIS_WHEEL_REAR_RIGHT);
  d_controller = ChassisSpeedControl_GetController(
      CHASSIS_WHEEL_REAR_LEFT);
  a_feedback = Chassis_GetWheelFeedback(CHASSIS_WHEEL_FRONT_RIGHT);
  b_feedback = Chassis_GetWheelFeedback(CHASSIS_WHEEL_FRONT_LEFT);
  c_feedback = Chassis_GetWheelFeedback(CHASSIS_WHEEL_REAR_RIGHT);
  d_feedback = Chassis_GetWheelFeedback(CHASSIS_WHEEL_REAR_LEFT);
  record.a_target_rpm_x10 = WheelSpeedPi_GetTarget(a_controller);
  record.a_actual_rpm_x10 = (a_feedback != 0) ? a_feedback->rpm_x10 : 0;
  record.a_pwm = WheelSpeedPi_GetOutput(a_controller);
  record.b_target_rpm_x10 = WheelSpeedPi_GetTarget(b_controller);
  record.b_actual_rpm_x10 = (b_feedback != 0) ? b_feedback->rpm_x10 : 0;
  record.b_pwm = WheelSpeedPi_GetOutput(b_controller);
  record.c_target_rpm_x10 = WheelSpeedPi_GetTarget(c_controller);
  record.c_actual_rpm_x10 = (c_feedback != 0) ? c_feedback->rpm_x10 : 0;
  record.c_pwm = WheelSpeedPi_GetOutput(c_controller);
  record.d_target_rpm_x10 = WheelSpeedPi_GetTarget(d_controller);
  record.d_actual_rpm_x10 = (d_feedback != 0) ? d_feedback->rpm_x10 : 0;
  record.d_pwm = WheelSpeedPi_GetOutput(d_controller);
  record.follow_state = (uint8_t)follow_state;
  record.front_state = (uint8_t)front_state;
  record.rear_state = (uint8_t)rear_state;
  LineFollowLog_Append(&record);
}

static void App_Start(void)
{
  DualGraySample_t sample;

  app_status_text = "CHECK";
  startup_motor_diagnostic_pending = 0U;
  LineFollowPID_Reset(&line_follow_pid);
  ChassisSpeedControl_Disable();
  follow_state = APP_FOLLOW_IDLE;
  rear_valid_count = 0U;
  front_state = APP_GRAY_UNKNOWN;
  rear_state = APP_GRAY_UNKNOWN;
  App_ResetSample(&sample);

  if (App_InitializeGray() == 0U)
  {
    front_state = APP_GRAY_I2C_ERROR;
    rear_state = APP_GRAY_UNAVAILABLE;
    app_status_text = App_FrontStatusText(front_state);
    App_ReportFrontCheck(&sample, front_state);
    App_StartReport("START_REJECTED=FRONT_I2C");
    return;
  }

  front_state = App_ReadFront(&sample);
  App_ReportFrontCheck(&sample, front_state);
  if (front_state != APP_GRAY_VALID)
  {
    rear_state = (rear_hw_ready != 0U) ? APP_GRAY_UNKNOWN :
                                        APP_GRAY_UNAVAILABLE;
    app_status_text = App_FrontStatusText(front_state);
    if (front_state == APP_GRAY_I2C_ERROR)
    {
      App_StartReport("START_REJECTED=FRONT_I2C");
    }
    else if (front_state == APP_GRAY_NO_LINE)
    {
      App_StartReport("START_REJECTED=FRONT_NO_LINE");
    }
    else if (front_state == APP_GRAY_ALL_BLACK)
    {
      App_StartReport("START_REJECTED=FRONT_ALL_BLACK");
    }
    else
    {
      App_StartReport("START_REJECTED=FRONT_WIDE_BLACK");
    }
    return;
  }

  if (rear_hw_ready != 0U)
  {
    rear_state = App_ReadRear(&sample);
  }
  else
  {
    rear_state = APP_GRAY_UNAVAILABLE;
  }
  if (rear_state == APP_GRAY_VALID)
  {
    follow_state = APP_FOLLOW_DUAL;
    rear_valid_count = APP_REAR_VALID_REQUIRED;
  }
  else
  {
    follow_state = APP_FOLLOW_FRONT_ONLY;
  }
  App_UpdateErrors(&sample);

  LineFollowLog_Reset();
  ChassisSpeedControl_Enable();
  run_start_tick = SystemTime_GetMs();
  last_control_tick = run_start_tick;
  last_log_tick = run_start_tick;
  app_running = 1U;
  app_status_text = "OK";
  startup_motor_diagnostic_pending = 1U;
  App_StartReport("START_OK=1");
}

static void App_ProcessIdleEvent(KeyEvent_t event)
{
  if (event == KEY_EVENT_KEY1_SHORT)
  {
    selected_parameter = (LineFollowPID_ParameterId_t)(
        ((uint32_t)selected_parameter + 1U) %
        (uint32_t)LINE_FOLLOW_PARAMETER_COUNT);
    App_ReportParameters();
  }
  else if (event == KEY_EVENT_KEY2_SHORT)
  {
    (void)LineFollowPID_AdjustParameter(
        selected_parameter,
        (selected_parameter == LINE_FOLLOW_PARAMETER_KP) ?
            LINE_FOLLOW_PID_KP_STEP_X1000 :
        (selected_parameter == LINE_FOLLOW_PARAMETER_KI) ?
            LINE_FOLLOW_PID_KI_STEP_X1000 :
        (selected_parameter == LINE_FOLLOW_PARAMETER_KD) ?
            LINE_FOLLOW_PID_KD_STEP_X1000 : LINE_FOLLOW_PID_KH_STEP_X1000);
    App_ReportParameters();
  }
  else if (event == KEY_EVENT_WK_UP_SHORT)
  {
    (void)LineFollowPID_AdjustParameter(
        selected_parameter,
        -((selected_parameter == LINE_FOLLOW_PARAMETER_KP) ?
            LINE_FOLLOW_PID_KP_STEP_X1000 :
          (selected_parameter == LINE_FOLLOW_PARAMETER_KI) ?
            LINE_FOLLOW_PID_KI_STEP_X1000 :
          (selected_parameter == LINE_FOLLOW_PARAMETER_KD) ?
            LINE_FOLLOW_PID_KD_STEP_X1000 : LINE_FOLLOW_PID_KH_STEP_X1000));
    App_ReportParameters();
  }
  else if (event == KEY_EVENT_KEY2_LONG)
  {
    App_Start();
  }
  else if (event == KEY_EVENT_WK_UP_LONG)
  {
    LineFollowLog_Export();
  }
}

void App_LineFollowPID_Init(void)
{
  Key_Init();
  ChassisSpeedControl_Init();
  LineFollowPID_Reset(&line_follow_pid);
  app_running = 0U;
  rear_hw_ready = 0U;
  rear_valid_count = 0U;
  follow_state = APP_FOLLOW_IDLE;
  front_state = APP_GRAY_UNKNOWN;
  rear_state = APP_GRAY_UNKNOWN;
  selected_parameter = LINE_FOLLOW_PARAMETER_KP;
  startup_motor_diagnostic_pending = 0U;
  app_status_text = "IDLE";
  run_start_tick = SystemTime_GetMs();
  last_control_tick = run_start_tick;
  last_log_tick = run_start_tick;
  last_report_tick = run_start_tick;
  UartDebug_SendString("NewFront_Gray12_LineFollow_PD_Test\r\n");
  UartDebug_SendString("START: FRONT_VALID -> FRONT_ONLY or DUAL\r\n");
  UartDebug_SendString("KEY1=select KEY2=+ WK_UP=- (IDLE)\r\n");
  UartDebug_SendString("KEY2_LONG=start WK_UP_LONG=export KEY1/WK_UP=stop\r\n");
  App_ReportParameters();
  TftPidPage_Init();
}

void App_LineFollowPID_Task(void)
{
  KeyEvent_t event = Key_GetEvent();
  uint32_t now = SystemTime_GetMs();
  DualGraySample_t sample;
  LineFollowPID_Output_t control;
  uint32_t elapsed_ms;
  uint32_t control_now;

  if ((app_running != 0U) &&
      (Key_IsPressed(KEY_ID_KEY1) != 0U))
  {
    App_Stop(APP_STOP_KEY1);
    goto task_end;
  }
  if ((app_running != 0U) &&
      (Key_IsPressed(KEY_ID_WK_UP) != 0U))
  {
    App_Stop(APP_STOP_WK_UP);
    goto task_end;
  }

  if (app_running != 0U)
  {
    if (event == KEY_EVENT_KEY2_SHORT)
    {
      App_Stop(APP_STOP_KEY2);
    }
    goto task_end;
  }

  if (event != KEY_EVENT_NONE)
  {
    App_ProcessIdleEvent(event);
    goto task_end;
  }

  if ((app_running == 0U) ||
      ((uint32_t)(now - last_control_tick) < LINE_FOLLOW_PID_CONTROL_PERIOD_MS))
  {
    goto task_end;
  }

  elapsed_ms = (uint32_t)(now - last_control_tick);
  last_control_tick = now;
  if ((uint32_t)(now - run_start_tick) >= APP_MAX_RUN_MS)
  {
    App_Stop(APP_STOP_TIMEOUT);
    goto task_end;
  }

  App_ResetSample(&sample);
  front_state = App_ReadFront(&sample);
  if (front_state != APP_GRAY_VALID)
  {
    App_Stop(App_FrontStopReason(front_state));
    goto task_end;
  }
  rear_state = App_ReadRear(&sample);
  if (rear_state == APP_GRAY_VALID)
  {
    if (rear_valid_count < APP_REAR_VALID_REQUIRED)
    {
      rear_valid_count++;
    }
    if ((follow_state == APP_FOLLOW_FRONT_ONLY) &&
        (rear_valid_count >= APP_REAR_VALID_REQUIRED))
    {
      follow_state = APP_FOLLOW_DUAL;
      LineFollowPID_Reset(&line_follow_pid);
      UartDebug_SendString("STATE_SWITCH=DUAL\r\n");
    }
  }
  else
  {
    rear_valid_count = 0U;
    if (follow_state == APP_FOLLOW_DUAL)
    {
      follow_state = APP_FOLLOW_FRONT_ONLY;
      LineFollowPID_Reset(&line_follow_pid);
      UartDebug_Printf("STATE_SWITCH=FRONT_ONLY REAR=%s\r\n",
                       App_GrayStateName(rear_state));
    }
  }
  App_UpdateErrors(&sample);

  LineFollowPID_Calculate(&line_follow_pid, sample.position_error_x100,
                          elapsed_ms, &control);
  if (follow_state == APP_FOLLOW_DUAL)
  {
    LineFollowPID_ApplyHeadingP(&control, sample.heading_error_x100);
  }
  ChassisSpeedControl_SetWheelTargets(control.left_target_rpm_x10,
                                      control.right_target_rpm_x10,
                                      control.left_target_rpm_x10,
                                      control.right_target_rpm_x10);
  ChassisSpeedControl_Update(elapsed_ms);
  if (startup_motor_diagnostic_pending != 0U)
  {
    App_ReportMotorStartup();
    startup_motor_diagnostic_pending = 0U;
  }
  control_now = SystemTime_GetMs();
  if ((uint32_t)(control_now - last_log_tick) >= APP_LOG_PERIOD_MS)
  {
    last_log_tick = control_now;
    App_CaptureLog(control_now, &sample, &control);
  }
  if (ChassisSpeedControl_HasStall() != 0U)
  {
    App_Stop(APP_STOP_STALL);
    goto task_end;
  }
  App_Report(&sample, &control);

task_end:
  TftPidPage_Task(LineFollowPID_GetParameters(), selected_parameter,
                  app_running, app_status_text);
}
