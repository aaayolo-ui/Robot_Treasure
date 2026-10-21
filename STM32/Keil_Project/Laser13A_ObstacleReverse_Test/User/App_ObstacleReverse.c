#include "App_ObstacleReverse.h"
#include "AI1811.h"
#include "ChassisSpeedControl.h"
#include "Key.h"
#include "UartDebug.h"
#include "main.h"

#define APP_CONTROL_PERIOD_MS       100U
#define APP_REPORT_PERIOD_MS        100U
#define APP_LASER_TIMEOUT_MS        500U
#define APP_BRAKE_WAIT_MS           200U
#define APP_REVERSE_RUN_MS          1000U
#define APP_MAX_RUN_MS              10000U
#define APP_TARGET_RPM_X10          300L
#define APP_REQUIRED_DETECT_COUNT   3U

typedef enum
{
  APP_STATE_IDLE = 0,
  APP_STATE_FORWARD,
  APP_STATE_BRAKE_WAIT,
  APP_STATE_REVERSE,
  APP_STATE_DONE,
  APP_STATE_FAULT
} AppState_t;

static AppState_t app_state;
static uint32_t state_start_tick;
static uint32_t run_start_tick;
static uint32_t last_control_tick;
static uint32_t last_report_tick;
static uint32_t last_frame_sequence;
static uint16_t last_distance_mm;
static uint8_t detect_count;
static int32_t left_target_rpm_x10;
static int32_t right_target_rpm_x10;

static const char *App_StateName(void)
{
  switch (app_state)
  {
    case APP_STATE_FORWARD:
      return "FORWARD";
    case APP_STATE_BRAKE_WAIT:
      return "BRAKE_WAIT";
    case APP_STATE_REVERSE:
      return "REVERSE";
    case APP_STATE_DONE:
      return "DONE";
    case APP_STATE_FAULT:
      return "FAULT";
    case APP_STATE_IDLE:
    default:
      return "IDLE";
  }
}

static void App_ClearTargets(void)
{
  left_target_rpm_x10 = 0L;
  right_target_rpm_x10 = 0L;
}

static void App_StopToIdle(void)
{
  App_ClearTargets();
  ChassisSpeedControl_Disable();
  app_state = APP_STATE_IDLE;
  state_start_tick = HAL_GetTick();
  detect_count = 0U;
  UartDebug_SendString("STOPPED_BY_KEY\r\n");
}

static void App_EnterFault(const char *reason)
{
  App_ClearTargets();
  ChassisSpeedControl_Disable();
  app_state = APP_STATE_FAULT;
  state_start_tick = HAL_GetTick();
  UartDebug_Printf("FAULT=%s\r\n", reason);
}

static void App_EnterBrakeWait(void)
{
  App_ClearTargets();
  ChassisSpeedControl_Stop();
  app_state = APP_STATE_BRAKE_WAIT;
  state_start_tick = HAL_GetTick();
  UartDebug_SendString("OBSTACLE_3_FRAMES: STOP_THEN_WAIT_200MS\r\n");
}

static void App_EnterReverse(void)
{
  ChassisSpeedControl_Enable();
  ChassisSpeedControl_SetMotionTarget(CHASSIS_MOTION_BACKWARD,
                                      APP_TARGET_RPM_X10);
  left_target_rpm_x10 = -APP_TARGET_RPM_X10;
  right_target_rpm_x10 = -APP_TARGET_RPM_X10;
  app_state = APP_STATE_REVERSE;
  state_start_tick = HAL_GetTick();
  UartDebug_SendString("REVERSE_START: TARGET=30.0RPM DURATION=1.0s\r\n");
}

static void App_EnterDone(void)
{
  App_ClearTargets();
  ChassisSpeedControl_Disable();
  app_state = APP_STATE_DONE;
  state_start_tick = HAL_GetTick();
  UartDebug_SendString("RUN_DONE: MOTOR_STOPPED\r\n");
}

static void App_StartRun(void)
{
  AI1811_Status_t sensor;

  AI1811_GetStatus(&sensor);
  ChassisSpeedControl_Enable();
  ChassisSpeedControl_SetMotionTarget(CHASSIS_MOTION_FORWARD,
                                      APP_TARGET_RPM_X10);
  left_target_rpm_x10 = APP_TARGET_RPM_X10;
  right_target_rpm_x10 = APP_TARGET_RPM_X10;
  detect_count = 0U;
  last_distance_mm = sensor.has_frame ? sensor.distance_mm : 0U;
  last_frame_sequence = sensor.sequence;
  AI1811_ResetObstacleStreak();
  run_start_tick = HAL_GetTick();
  state_start_tick = run_start_tick;
  app_state = APP_STATE_FORWARD;
  UartDebug_SendString("RUN_START: FORWARD TARGET=30.0RPM\r\n");
}

static void App_ProcessKeys(void)
{
  KeyEvent_t event = Key_GetEvent();

  if (event == KEY_EVENT_KEY2)
  {
    if ((app_state == APP_STATE_IDLE) || (app_state == APP_STATE_DONE) ||
        (app_state == APP_STATE_FAULT))
    {
      App_StartRun();
    }
    else
    {
      App_StopToIdle();
    }
  }
  else if (event == KEY_EVENT_KEY1)
  {
    if ((app_state == APP_STATE_FORWARD) ||
        (app_state == APP_STATE_BRAKE_WAIT) ||
        (app_state == APP_STATE_REVERSE))
    {
      App_StopToIdle();
    }
  }
  else if (event == KEY_EVENT_WK_UP)
  {
    App_EnterFault("EMERGENCY_KEY");
  }
}

static void App_ProcessLaser(uint32_t now)
{
  AI1811_Status_t sensor;

  AI1811_GetStatus(&sensor);
  if (sensor.has_frame == 0U)
  {
    if ((uint32_t)(now - run_start_tick) >= APP_LASER_TIMEOUT_MS)
    {
      App_EnterFault("LASER_NO_VALID_DATA");
    }
    return;
  }

  last_distance_mm = sensor.distance_mm;
  if ((uint32_t)(now - sensor.timestamp_ms) > APP_LASER_TIMEOUT_MS)
  {
    App_EnterFault("LASER_TIMEOUT");
    return;
  }

  if (sensor.sequence == last_frame_sequence)
  {
    return;
  }
  last_frame_sequence = sensor.sequence;

  if (sensor.measurement_valid == 0U)
  {
    App_EnterFault("LASER_ABNORMAL_DISTANCE");
    return;
  }

  detect_count = sensor.obstacle_streak;

  if (detect_count >= APP_REQUIRED_DETECT_COUNT)
  {
    App_EnterBrakeWait();
  }
}

static void App_Report(void)
{
  uint32_t now = HAL_GetTick();
  AI1811_Status_t sensor;

  if ((uint32_t)(now - last_report_tick) < APP_REPORT_PERIOD_MS)
  {
    return;
  }
  last_report_tick = now;
  AI1811_GetStatus(&sensor);
  if (sensor.has_frame != 0U)
  {
    last_distance_mm = sensor.distance_mm;
  }
  UartDebug_Printf("STATE=%s DIST_MM=%u DETECT_COUNT=%u LEFT_TARGET=%ld RIGHT_TARGET=%ld\r\n",
                   App_StateName(), (unsigned int)last_distance_mm,
                   (unsigned int)detect_count, (long)left_target_rpm_x10,
                   (long)right_target_rpm_x10);
}

void App_ObstacleReverse_Init(void)
{
  Key_Init();
  ChassisSpeedControl_Init();
  AI1811_Init();
  app_state = APP_STATE_IDLE;
  state_start_tick = HAL_GetTick();
  run_start_tick = state_start_tick;
  last_control_tick = state_start_tick;
  last_report_tick = state_start_tick;
  last_frame_sequence = 0U;
  last_distance_mm = 0U;
  detect_count = 0U;
  App_ClearTargets();

  UartDebug_SendString("\r\nLaser13A_ObstacleReverse_Test started.\r\n");
  UartDebug_SendString("AI-1811: PA3/USART2 RX 115200 8N1, frame AA 05 + SUM.\r\n");
  UartDebug_SendString("USART3 log: PB10/PB11 115200 8N1.\r\n");
  UartDebug_SendString("KEY2=start/stop, KEY1=stop, WK_UP=emergency stop.\r\n");
  UartDebug_SendString("3 frames at 50..300mm -> stop 200ms -> reverse 1.0s.\r\n");
  UartDebug_SendString("Motor driver is disabled at startup; no hardware result recorded.\r\n");
}

void App_ObstacleReverse_Task(void)
{
  uint32_t now;
  uint32_t elapsed_ms;

  App_ProcessKeys();
  now = HAL_GetTick();
  if ((uint32_t)(now - last_control_tick) >= APP_CONTROL_PERIOD_MS)
  {
    elapsed_ms = (uint32_t)(now - last_control_tick);
    last_control_tick = now;

    if (((app_state == APP_STATE_FORWARD) ||
         (app_state == APP_STATE_BRAKE_WAIT) ||
         (app_state == APP_STATE_REVERSE)) &&
        ((uint32_t)(now - run_start_tick) >= APP_MAX_RUN_MS))
    {
      App_EnterFault("MAX_RUNTIME");
    }

    if (app_state == APP_STATE_FORWARD)
    {
      App_ProcessLaser(now);
      if (app_state == APP_STATE_FORWARD)
      {
        ChassisSpeedControl_SetMotionTarget(CHASSIS_MOTION_FORWARD,
                                            APP_TARGET_RPM_X10);
        ChassisSpeedControl_Update(elapsed_ms);
      }
    }
    else if (app_state == APP_STATE_BRAKE_WAIT)
    {
      if ((uint32_t)(now - state_start_tick) >= APP_BRAKE_WAIT_MS)
      {
        App_EnterReverse();
      }
      ChassisSpeedControl_Update(elapsed_ms);
    }
    else if (app_state == APP_STATE_REVERSE)
    {
      if ((uint32_t)(now - state_start_tick) >= APP_REVERSE_RUN_MS)
      {
        App_EnterDone();
      }
      else
      {
        ChassisSpeedControl_SetMotionTarget(CHASSIS_MOTION_BACKWARD,
                                            APP_TARGET_RPM_X10);
        ChassisSpeedControl_Update(elapsed_ms);
      }
    }
    else
    {
      ChassisSpeedControl_Update(elapsed_ms);
    }

    if ((ChassisSpeedControl_HasStall() != 0U) &&
        ((app_state == APP_STATE_FORWARD) ||
         (app_state == APP_STATE_REVERSE)))
    {
      App_EnterFault("WHEEL_STALL");
    }
  }
  App_Report();
}
