#include "App_InfraredObstacleReverse.h"
#include "ChassisSpeedControl.h"
#include "Infrared.h"
#include "Key.h"
#include "UartDebug.h"
#include "main.h"

#define APP_CONTROL_PERIOD_MS       100U
#define APP_REPORT_PERIOD_MS        200U
#define APP_BRAKE_WAIT_MS           200U
#define APP_REVERSE_RUN_MS         1000U
#define APP_MAX_RUN_MS            10000U
#define APP_TARGET_RPM_X10          300L
#define APP_REQUIRED_DETECT_COUNT     3U

typedef enum
{
  APP_STATE_IDLE = 0,
  APP_STATE_FORWARD,
  APP_STATE_BRAKE_WAIT,
  APP_STATE_REVERSE,
  APP_STATE_DONE
} AppState_t;

static AppState_t app_state;
static uint32_t state_start_tick;
static uint32_t run_start_tick;
static uint32_t last_control_tick;
static uint32_t last_report_tick;
static uint8_t detect_count;
static uint8_t ir_detected;
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

static void App_Stop(const char *reason)
{
  App_ClearTargets();
  ChassisSpeedControl_Disable();
  app_state = APP_STATE_IDLE;
  state_start_tick = HAL_GetTick();
  detect_count = 0U;
  UartDebug_Printf("STOP=%s\r\n", reason);
}

static void App_EnterBrakeWait(void)
{
  App_ClearTargets();
  /* Stop clears all PI targets and integral state before any reversal. */
  ChassisSpeedControl_Stop();
  app_state = APP_STATE_BRAKE_WAIT;
  state_start_tick = HAL_GetTick();
  UartDebug_SendString("OBSTACLE_3X: STOP_WAIT_200MS\r\n");
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
  UartDebug_SendString("REVERSE_30RPM_1S\r\n");
}

static void App_EnterDone(void)
{
  App_ClearTargets();
  ChassisSpeedControl_Disable();
  app_state = APP_STATE_DONE;
  state_start_tick = HAL_GetTick();
  UartDebug_SendString("DONE_STOPPED\r\n");
}

static void App_Start(void)
{
  ChassisSpeedControl_Enable();
  ChassisSpeedControl_SetMotionTarget(CHASSIS_MOTION_FORWARD,
                                      APP_TARGET_RPM_X10);
  left_target_rpm_x10 = APP_TARGET_RPM_X10;
  right_target_rpm_x10 = APP_TARGET_RPM_X10;
  detect_count = 0U;
  run_start_tick = HAL_GetTick();
  state_start_tick = run_start_tick;
  app_state = APP_STATE_FORWARD;
  UartDebug_SendString("START_FORWARD_30RPM\r\n");
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
      App_Stop("KEY2");
    }
  }
  else if (event == KEY_EVENT_KEY1)
  {
    App_Stop("KEY1");
  }
  else if (event == KEY_EVENT_WK_UP)
  {
    App_Stop("WK_UP");
  }
}

static void App_Report(void)
{
  uint32_t now = HAL_GetTick();

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

void App_InfraredObstacleReverse_Init(void)
{
  Key_Init();
  Infrared_Init();
  ChassisSpeedControl_Init();
  app_state = APP_STATE_IDLE;
  state_start_tick = HAL_GetTick();
  run_start_tick = state_start_tick;
  last_control_tick = state_start_tick;
  last_report_tick = state_start_tick;
  detect_count = 0U;
  ir_detected = 0U;
  App_ClearTargets();

  UartDebug_SendString("\r\nInfrared13A_ObstacleReverse_Test\r\n");
  UartDebug_SendString("IR: PB14 input/no-pull, 0=DETECTED, 1=CLEAR.\r\n");
  UartDebug_SendString("KEY2=start/stop, KEY1/WK_UP=immediate stop.\r\n");
  UartDebug_SendString("Validated 13A obstacle reverse sequence is documented in Docs.\r\n");
}

void App_InfraredObstacleReverse_Task(void)
{
  uint32_t now;
  uint32_t elapsed_ms;

  App_ProcessKeys();
  now = HAL_GetTick();
  if ((uint32_t)(now - last_control_tick) >= APP_CONTROL_PERIOD_MS)
  {
    elapsed_ms = (uint32_t)(now - last_control_tick);
    last_control_tick = now;
    ir_detected = Infrared_IsDetected();

    if (((app_state == APP_STATE_FORWARD) ||
         (app_state == APP_STATE_BRAKE_WAIT) ||
         (app_state == APP_STATE_REVERSE)) &&
        ((uint32_t)(now - run_start_tick) >= APP_MAX_RUN_MS))
    {
      App_Stop("MAX_RUNTIME");
    }

    if (app_state == APP_STATE_FORWARD)
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
        ((app_state == APP_STATE_FORWARD) || (app_state == APP_STATE_REVERSE)))
    {
      App_Stop("WHEEL_STALL");
    }
  }
  App_Report();
}
