#include "App_InfraredLineFollowObstacleReverse.h"
#include "ChassisSpeedControl.h"
#include "gray_coordinate.h"
#include "gray_position.h"
#include "Infrared.h"
#include "Key.h"
#include "LineFollowP.h"
#include "nchd12.h"
#include "SystemTime.h"
#include "UartDebug.h"

#define APP_MAX_RUN_MS             10000U
#define APP_BRAKE_WAIT_MS            200U
#define APP_REVERSE_RUN_MS          1000U
#define APP_REVERSE_RPM_X10          300L
#define APP_REQUIRED_DETECT_COUNT      3U
#define APP_REPORT_PERIOD_MS         200U

typedef enum
{
  APP_STATE_IDLE = 0,
  APP_STATE_LINE_FOLLOW,
  APP_STATE_BRAKE_WAIT,
  APP_STATE_REVERSE,
  APP_STATE_DONE
} AppState_t;

typedef enum
{
  APP_STOP_KEY2 = 0,
  APP_STOP_KEY1,
  APP_STOP_WK_UP,
  APP_STOP_TIMEOUT,
  APP_STOP_NO_LINE,
  APP_STOP_ALL_BLACK,
  APP_STOP_WIDE_BLACK,
  APP_STOP_I2C,
  APP_STOP_STALL,
  APP_STOP_REVERSE_DONE
} AppStopReason_t;

static AppState_t app_state;
static uint32_t run_start_tick;
static uint32_t state_start_tick;
static uint32_t last_control_tick;
static uint32_t last_report_tick;
static uint8_t detect_count;
static uint8_t ir_detected;
static int32_t left_target_rpm_x10;
static int32_t right_target_rpm_x10;

static const char *App_StateName(void)
{
  static const char *const names[] = {"IDLE", "LINE_FOLLOW", "BRAKE_WAIT", "REVERSE", "DONE"};
  return ((uint32_t)app_state < (sizeof(names) / sizeof(names[0]))) ? names[app_state] : "UNKNOWN";
}

static const char *App_StopName(AppStopReason_t reason)
{
  static const char *const names[] = {"KEY2", "KEY1", "WK_UP", "TIMEOUT", "NO_LINE", "ALL_BLACK", "WIDE_BLACK", "I2C", "STALL", "REVERSE_DONE"};
  return ((uint32_t)reason < (sizeof(names) / sizeof(names[0]))) ? names[reason] : "UNKNOWN";
}

static void App_ClearTargets(void)
{
  left_target_rpm_x10 = 0L;
  right_target_rpm_x10 = 0L;
}

static void App_StopToIdle(AppStopReason_t reason)
{
  App_ClearTargets();
  ChassisSpeedControl_Disable();
  app_state = APP_STATE_IDLE;
  detect_count = 0U;
  UartDebug_Printf("STOP=%s\r\n", App_StopName(reason));
}

static void App_EnterDone(AppStopReason_t reason)
{
  App_ClearTargets();
  ChassisSpeedControl_Disable();
  app_state = APP_STATE_DONE;
  detect_count = 0U;
  UartDebug_Printf("DONE=%s\r\n", App_StopName(reason));
}

static uint8_t App_CountBlack(uint16_t raw12)
{
  uint8_t bit;
  uint8_t count = 0U;

  for (bit = 0U; bit < 12U; bit++)
  {
    if ((raw12 & ((uint16_t)1U << bit)) != 0U)
    {
      count++;
    }
  }
  return count;
}

static uint8_t App_ReadValidLine(int16_t *error_x100)
{
  uint16_t raw16;
  uint16_t raw12;
  uint8_t black_count;
  GrayPosition_Result_t position;

  if (NCHD12_ReadRaw16(&raw16) != NCHD12_STATUS_OK)
  {
    App_EnterDone(APP_STOP_I2C);
    return 0U;
  }

  raw12 = NCHD12_Extract12(raw16);
  GrayPosition_Calculate(raw12, &position);
  black_count = App_CountBlack(raw12);
  if ((black_count == 0U) || (position.status == GRAY_POSITION_STATUS_NO_LINE))
  {
    App_EnterDone(APP_STOP_NO_LINE);
    return 0U;
  }
  if ((black_count == 12U) || (position.status == GRAY_POSITION_STATUS_ALL_BLACK))
  {
    App_EnterDone(APP_STOP_ALL_BLACK);
    return 0U;
  }
  if (black_count > 4U)
  {
    App_EnterDone(APP_STOP_WIDE_BLACK);
    return 0U;
  }
  if (!GrayCoordinate_ToVehicleError(position.position_x100,
                                      GRAY_MOUNT_P12_LEFT_P1_RIGHT,
                                      error_x100))
  {
    App_EnterDone(APP_STOP_I2C);
    return 0U;
  }
  return 1U;
}

static uint8_t App_GrayReady(void)
{
  NCHD12_Init();
  if (NCHD12_ScanBus() != NCHD12_STATUS_OK)
  {
    return 0U;
  }
  return (NCHD12_ConfigureInputs() == NCHD12_STATUS_OK) ? 1U : 0U;
}

static void App_Start(void)
{
  int16_t error_x100;

  if ((!App_GrayReady()) || (!App_ReadValidLine(&error_x100)))
  {
    UartDebug_SendString("START_REJECTED\r\n");
    return;
  }
  ChassisSpeedControl_Enable();
  run_start_tick = SystemTime_GetMs();
  state_start_tick = run_start_tick;
  last_control_tick = run_start_tick;
  detect_count = 0U;
  App_ClearTargets();
  app_state = APP_STATE_LINE_FOLLOW;
  UartDebug_SendString("START_LINE_FOLLOW_P\r\n");
}

static void App_EnterBrakeWait(void)
{
  App_ClearTargets();
  ChassisSpeedControl_Stop();
  app_state = APP_STATE_BRAKE_WAIT;
  state_start_tick = SystemTime_GetMs();
  UartDebug_SendString("OBSTACLE_3X: STOP_WAIT_200MS\r\n");
}

static void App_EnterReverse(void)
{
  ChassisSpeedControl_Enable();
  ChassisSpeedControl_SetMotionTarget(CHASSIS_MOTION_BACKWARD, APP_REVERSE_RPM_X10);
  left_target_rpm_x10 = -APP_REVERSE_RPM_X10;
  right_target_rpm_x10 = -APP_REVERSE_RPM_X10;
  app_state = APP_STATE_REVERSE;
  state_start_tick = SystemTime_GetMs();
  UartDebug_SendString("REVERSE_30RPM_1S\r\n");
}

static void App_ProcessKeys(void)
{
  KeyEvent_t event = Key_GetEvent();

  if (event == KEY_EVENT_KEY2)
  {
    if ((app_state == APP_STATE_IDLE) || (app_state == APP_STATE_DONE))
    {
      App_Start();
    }
    else
    {
      App_StopToIdle(APP_STOP_KEY2);
    }
  }
  else if (event == KEY_EVENT_KEY1)
  {
    App_StopToIdle(APP_STOP_KEY1);
  }
  else if (event == KEY_EVENT_WK_UP)
  {
    App_StopToIdle(APP_STOP_WK_UP);
  }
}

static void App_Report(void)
{
  uint32_t now = SystemTime_GetMs();

  if ((uint32_t)(now - last_report_tick) < APP_REPORT_PERIOD_MS)
  {
    return;
  }
  last_report_tick = now;
  UartDebug_Printf("STATE=%s IR=%s DETECT_COUNT=%u LEFT_TARGET=%ld RIGHT_TARGET=%ld\r\n",
                   App_StateName(), (ir_detected != 0U) ? "DETECTED" : "CLEAR",
                   (unsigned int)detect_count, (long)left_target_rpm_x10,
                   (long)right_target_rpm_x10);
}

void App_InfraredLineFollowObstacleReverse_Init(void)
{
  Key_Init();
  Infrared_Init();
  ChassisSpeedControl_Init();
  app_state = APP_STATE_IDLE;
  run_start_tick = SystemTime_GetMs();
  state_start_tick = run_start_tick;
  last_control_tick = run_start_tick;
  last_report_tick = run_start_tick;
  detect_count = 0U;
  ir_detected = 0U;
  App_ClearTargets();
  UartDebug_SendString("Infrared14A_LineFollow_ObstacleReverse_Test\r\n");
  UartDebug_SendString("IR PB14 no-pull: 0=DETECTED, 1=CLEAR.\r\n");
}

void App_InfraredLineFollowObstacleReverse_Task(void)
{
  uint32_t now;
  uint32_t elapsed_ms;

  App_ProcessKeys();
  now = SystemTime_GetMs();
  if ((uint32_t)(now - last_control_tick) < LINE_FOLLOW_P_CONTROL_PERIOD_MS)
  {
    App_Report();
    return;
  }
  elapsed_ms = (uint32_t)(now - last_control_tick);
  last_control_tick = now;
  ir_detected = Infrared_IsDetected();

  if (((app_state == APP_STATE_LINE_FOLLOW) ||
       (app_state == APP_STATE_BRAKE_WAIT) ||
       (app_state == APP_STATE_REVERSE)) &&
      ((uint32_t)(now - run_start_tick) >= APP_MAX_RUN_MS))
  {
    App_EnterDone(APP_STOP_TIMEOUT);
  }
  else if (app_state == APP_STATE_LINE_FOLLOW)
  {
    if (ir_detected != 0U)
    {
      if (detect_count < APP_REQUIRED_DETECT_COUNT)
      {
        detect_count++;
      }
    }
    else
    {
      detect_count = 0U;
    }

    if (detect_count >= APP_REQUIRED_DETECT_COUNT)
    {
      App_EnterBrakeWait();
    }
    else
    {
      int16_t error_x100;
      LineFollowP_Output_t line_follow;

      if (App_ReadValidLine(&error_x100) != 0U)
      {
        LineFollowP_Calculate(error_x100, &line_follow);
        left_target_rpm_x10 = line_follow.left_target_rpm_x10;
        right_target_rpm_x10 = line_follow.right_target_rpm_x10;
        ChassisSpeedControl_SetWheelTargets(left_target_rpm_x10, right_target_rpm_x10,
                                            left_target_rpm_x10, right_target_rpm_x10);
        ChassisSpeedControl_Update(elapsed_ms);
      }
    }
  }
  else if (app_state == APP_STATE_BRAKE_WAIT)
  {
    if ((uint32_t)(now - state_start_tick) >= APP_BRAKE_WAIT_MS)
    {
      App_EnterReverse();
    }
    else
    {
      ChassisSpeedControl_Update(elapsed_ms);
    }
  }
  else if (app_state == APP_STATE_REVERSE)
  {
    if ((uint32_t)(now - state_start_tick) >= APP_REVERSE_RUN_MS)
    {
      App_EnterDone(APP_STOP_REVERSE_DONE);
    }
    else
    {
      ChassisSpeedControl_SetMotionTarget(CHASSIS_MOTION_BACKWARD, APP_REVERSE_RPM_X10);
      ChassisSpeedControl_Update(elapsed_ms);
    }
  }

  if ((ChassisSpeedControl_HasStall() != 0U) &&
      ((app_state == APP_STATE_LINE_FOLLOW) || (app_state == APP_STATE_REVERSE)))
  {
    App_EnterDone(APP_STOP_STALL);
  }
  App_Report();
}
