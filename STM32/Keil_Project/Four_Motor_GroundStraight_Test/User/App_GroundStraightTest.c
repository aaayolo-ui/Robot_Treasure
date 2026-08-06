#include "App_GroundStraightTest.h"
#include "ChassisSpeedControl.h"
#include "Key.h"
#include "UartDebug.h"
#include "main.h"

#define SPEED_CONTROL_PERIOD_MS       100U
#define GROUND_REPORT_PERIOD_MS       500U
#define GROUND_TEST_TARGET_RPM_X10    800L
#define GROUND_TARGET_STEP_RPM_X10    40L
#define GROUND_TEST_MAX_PWM           300U
#define GROUND_TEST_MAX_RUN_MS        3000U

typedef enum
{
  GROUND_MOTION_FORWARD = 0,
  GROUND_MOTION_BACKWARD,
  GROUND_MOTION_COUNT
} GroundMotion_t;

static GroundMotion_t selected_motion;
static int32_t requested_target_rpm_x10;
static int32_t active_target_rpm_x10;
static uint32_t run_start_tick;
static uint32_t last_control_tick;
static uint32_t last_report_tick;
static uint8_t stall_announced;

static const char *App_GroundMotionName(GroundMotion_t motion)
{
  return (motion == GROUND_MOTION_BACKWARD) ? "BACKWARD" : "FORWARD";
}

static ChassisMotion_t App_GroundChassisMotion(void)
{
  return (selected_motion == GROUND_MOTION_BACKWARD) ?
         CHASSIS_MOTION_BACKWARD : CHASSIS_MOTION_FORWARD;
}

static int64_t App_Absolute(int32_t value)
{
  return (value < 0) ? -(int64_t)value : (int64_t)value;
}

static void App_SplitRpmX10(int32_t rpm_x10, const char **sign,
                            long *integer, long *decimal)
{
  int64_t value = (int64_t)rpm_x10;
  int64_t absolute_value = (value < 0) ? -value : value;

  *sign = (value < 0) ? "-" : "";
  *integer = (long)(absolute_value / 10LL);
  *decimal = (long)(absolute_value % 10LL);
}

static void App_PrintRpm(const char *label, int32_t value)
{
  const char *sign;
  long integer;
  long decimal;

  App_SplitRpmX10(value, &sign, &integer, &decimal);
  UartDebug_Printf("%s%s%ld.%ld", label, sign, integer, decimal);
}

static void App_PrintWheel(const char *label, ChassisWheelId_t wheel)
{
  const WheelSpeedPi_t *controller = ChassisSpeedControl_GetController(wheel);
  const ChassisWheelFeedback_t *feedback = Chassis_GetWheelFeedback(wheel);
  int32_t target_rpm_x10 = WheelSpeedPi_GetTarget(controller);
  int32_t error_rpm_x10 = target_rpm_x10 - feedback->rpm_x10;
  int64_t output = (int64_t)WheelSpeedPi_GetOutput(controller);

  UartDebug_Printf("%s: ", label);
  App_PrintRpm("target: ", target_rpm_x10);
  UartDebug_SendString(", ");
  App_PrintRpm("measured: ", feedback->rpm_x10);
  UartDebug_SendString(", ");
  App_PrintRpm("error: ", error_rpm_x10);
  UartDebug_Printf(", PWM: %ld%s\r\n", (long)((output < 0) ? -output : output),
                   (controller->saturated != 0U) ? " SAT" : "");
}

static void App_StopGroundTest(void)
{
  requested_target_rpm_x10 = 0;
  active_target_rpm_x10 = 0;
  ChassisSpeedControl_Stop();
}

static void App_StartGroundTest(void)
{
  ChassisSpeedControl_Enable();
  requested_target_rpm_x10 = (selected_motion == GROUND_MOTION_BACKWARD) ?
                             -GROUND_TEST_TARGET_RPM_X10 : GROUND_TEST_TARGET_RPM_X10;
  active_target_rpm_x10 = 0;
  run_start_tick = HAL_GetTick();
  stall_announced = 0U;
  UartDebug_SendString("Ground straight test started.\r\n");
  UartDebug_Printf("Motion: %s\r\n", App_GroundMotionName(selected_motion));
  UartDebug_SendString("Final target: 80.0 RPM\r\n");
  UartDebug_SendString("Target ramp: 4.0 RPM per 100 ms\r\n");
  UartDebug_SendString("Automatic stop after 3.0 seconds.\r\n");
}

static void App_UpdateTargetRamp(void)
{
  if (active_target_rpm_x10 < requested_target_rpm_x10)
  {
    active_target_rpm_x10 += GROUND_TARGET_STEP_RPM_X10;
    if (active_target_rpm_x10 > requested_target_rpm_x10)
    {
      active_target_rpm_x10 = requested_target_rpm_x10;
    }
  }
  else if (active_target_rpm_x10 > requested_target_rpm_x10)
  {
    active_target_rpm_x10 -= GROUND_TARGET_STEP_RPM_X10;
    if (active_target_rpm_x10 < requested_target_rpm_x10)
    {
      active_target_rpm_x10 = requested_target_rpm_x10;
    }
  }
  ChassisSpeedControl_SetMotionTarget(App_GroundChassisMotion(),
                                      (int32_t)App_Absolute(active_target_rpm_x10));
}

static void App_ProcessKeys(void)
{
  switch (Key_GetEvent())
  {
    case KEY_EVENT_KEY1:
      if (ChassisSpeedControl_IsEnabled() != 0U)
      {
        UartDebug_SendString("Stop the ground test before changing motion.\r\n");
      }
      else
      {
        selected_motion = (selected_motion == GROUND_MOTION_FORWARD) ?
                          GROUND_MOTION_BACKWARD : GROUND_MOTION_FORWARD;
        UartDebug_Printf("Selected motion: %s\r\n", App_GroundMotionName(selected_motion));
      }
      break;
    case KEY_EVENT_KEY2:
      if (ChassisSpeedControl_IsEnabled() == 0U)
      {
        App_StartGroundTest();
      }
      else
      {
        App_StopGroundTest();
        UartDebug_SendString("Ground straight test stopped by KEY2.\r\n");
      }
      break;
    case KEY_EVENT_WK_UP:
      requested_target_rpm_x10 = 0;
      active_target_rpm_x10 = 0;
      ChassisSpeedControl_Disable();
      UartDebug_SendString("Emergency stop. Ground test disabled.\r\n");
      break;
    case KEY_EVENT_NONE:
    default:
      break;
  }
}

static void App_Report(void)
{
  uint32_t now = HAL_GetTick();
  uint32_t report_elapsed_ms;
  uint32_t run_elapsed_ms;
  int32_t left_rpm_x10;
  int32_t right_rpm_x10;
  int32_t difference_rpm_x10;
  const char *controller_state;

  if ((uint32_t)(now - last_report_tick) < GROUND_REPORT_PERIOD_MS)
  {
    return;
  }
  report_elapsed_ms = (uint32_t)(now - last_report_tick);
  last_report_tick = now;
  run_elapsed_ms = (ChassisSpeedControl_IsEnabled() != 0U) ?
                   (uint32_t)(now - run_start_tick) : 0U;
  left_rpm_x10 = Chassis_GetLeftAverageRpmX10();
  right_rpm_x10 = Chassis_GetRightAverageRpmX10();
  difference_rpm_x10 = left_rpm_x10 - right_rpm_x10;
  if (ChassisSpeedControl_IsDisabled() != 0U)
  {
    controller_state = "DISABLED";
  }
  else if (ChassisSpeedControl_IsEnabled() != 0U)
  {
    controller_state = "RUNNING";
  }
  else
  {
    controller_state = "STOPPED";
  }

  UartDebug_Printf("Selected: %s\r\nController: %s\r\nDriver: %s\r\n",
                   App_GroundMotionName(selected_motion), controller_state,
                   (Chassis_IsEnabled() != 0U) ? "ENABLED" : "DISABLED");
  UartDebug_Printf("Run time: %lu / %u ms, report dt: %lu ms\r\n",
                   (unsigned long)run_elapsed_ms, (unsigned int)GROUND_TEST_MAX_RUN_MS,
                   (unsigned long)report_elapsed_ms);
  App_PrintRpm("Requested target: ", requested_target_rpm_x10);
  UartDebug_SendString(" RPM\r\n");
  App_PrintRpm("Active ramp target: ", active_target_rpm_x10);
  UartDebug_SendString(" RPM\r\n");
  App_PrintWheel("FL(D)", CHASSIS_WHEEL_FRONT_LEFT);
  App_PrintWheel("FR(C)", CHASSIS_WHEEL_FRONT_RIGHT);
  App_PrintWheel("RL(B)", CHASSIS_WHEEL_REAR_LEFT);
  App_PrintWheel("RR(A)", CHASSIS_WHEEL_REAR_RIGHT);
  App_PrintRpm("Left average RPM: ", left_rpm_x10);
  UartDebug_SendString("\r\n");
  App_PrintRpm("Right average RPM: ", right_rpm_x10);
  UartDebug_SendString("\r\n");
  App_PrintRpm("Left-right difference RPM: ", difference_rpm_x10);
  UartDebug_SendString("\r\n\r\n");
}

void App_GroundStraightTest_Init(void)
{
  Key_Init();
  ChassisSpeedControl_Init();
  selected_motion = GROUND_MOTION_FORWARD;
  requested_target_rpm_x10 = 0;
  active_target_rpm_x10 = 0;
  run_start_tick = HAL_GetTick();
  last_control_tick = run_start_tick;
  last_report_tick = run_start_tick;
  stall_announced = 0U;

  UartDebug_SendString("\r\nFour_Motor_GroundStraight_Test started.\r\n");
  UartDebug_SendString("LOW-SPEED GROUND TEST.\r\nPlace the chassis on a clear flat floor.\r\n");
  UartDebug_SendString("Keep at least 1 meter of clear space.\r\nKeep one hand ready on WK_UP.\r\n");
  UartDebug_SendString("Do not stand in front of the chassis.\r\nWheel mapping:\r\n");
  UartDebug_SendString("FL = Motor D\r\nFR = Motor C\r\nRL = Motor B\r\nRR = Motor A\r\n");
  UartDebug_SendString("Target speed: 80.0 RPM\r\nTarget ramp: 4.0 RPM per 100 ms\r\n");
  UartDebug_SendString("Maximum test PWM: 300\r\nMaximum run time: 3.0 seconds\r\n");
  UartDebug_SendString("KEY1: select FORWARD or BACKWARD while stopped\r\n");
  UartDebug_SendString("KEY2: start or stop ground test\r\nWK_UP: emergency stop and disable chassis\r\n");
  UartDebug_SendString("Ground test is disabled at startup.\r\n");
}

void App_GroundStraightTest_Task(void)
{
  uint32_t now;
  uint32_t elapsed_ms;

  App_ProcessKeys();
  now = HAL_GetTick();
  if ((uint32_t)(now - last_control_tick) >= SPEED_CONTROL_PERIOD_MS)
  {
    elapsed_ms = (uint32_t)(now - last_control_tick);
    last_control_tick = now;
    if (ChassisSpeedControl_IsEnabled() != 0U)
    {
      if ((uint32_t)(now - run_start_tick) >= GROUND_TEST_MAX_RUN_MS)
      {
        App_StopGroundTest();
        UartDebug_SendString("Ground straight test automatically stopped after 3.0 seconds.\r\n");
      }
      else
      {
        App_UpdateTargetRamp();
      }
    }
    ChassisSpeedControl_Update(elapsed_ms);
  }
  if ((ChassisSpeedControl_HasStall() != 0U) && (stall_announced == 0U))
  {
    requested_target_rpm_x10 = 0;
    active_target_rpm_x10 = 0;
    switch (ChassisSpeedControl_GetStallWheel())
    {
      case CHASSIS_WHEEL_FRONT_LEFT: UartDebug_SendString("Ground test wheel stall detected: FL(D).\r\n"); break;
      case CHASSIS_WHEEL_FRONT_RIGHT: UartDebug_SendString("Ground test wheel stall detected: FR(C).\r\n"); break;
      case CHASSIS_WHEEL_REAR_LEFT: UartDebug_SendString("Ground test wheel stall detected: RL(B).\r\n"); break;
      case CHASSIS_WHEEL_REAR_RIGHT: UartDebug_SendString("Ground test wheel stall detected: RR(A).\r\n"); break;
      default: break;
    }
    UartDebug_SendString("All wheels stopped for safety.\r\n");
    stall_announced = 1U;
  }
  App_Report();
}
