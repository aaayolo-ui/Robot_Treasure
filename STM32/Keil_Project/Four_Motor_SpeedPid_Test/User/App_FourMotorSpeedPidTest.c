#include "App_FourMotorSpeedPidTest.h"
#include "ChassisSpeedControl.h"
#include "Key.h"
#include "UartDebug.h"
#include "main.h"

#define SPEED_CONTROL_PERIOD_MS     100U
#define SPEED_REPORT_PERIOD_MS      500U
#define SPEED_TEST_TARGET_RPM_X10   300L

typedef enum
{
  TEST_MOTION_FORWARD = 0,
  TEST_MOTION_BACKWARD,
  TEST_MOTION_TURN_LEFT,
  TEST_MOTION_TURN_RIGHT,
  TEST_MOTION_COUNT
} TestMotion_t;

static TestMotion_t selected_motion;
static uint32_t last_control_tick;
static uint32_t last_report_tick;
static uint8_t stall_announced;

static const char *App_MotionName(TestMotion_t motion)
{
  switch (motion)
  {
    case TEST_MOTION_FORWARD: return "FORWARD";
    case TEST_MOTION_BACKWARD: return "BACKWARD";
    case TEST_MOTION_TURN_LEFT: return "TURN_LEFT";
    case TEST_MOTION_TURN_RIGHT: return "TURN_RIGHT";
    default: return "FORWARD";
  }
}

static ChassisMotion_t App_GetChassisMotion(void)
{
  return (ChassisMotion_t)((uint32_t)selected_motion +
                            (uint32_t)CHASSIS_MOTION_FORWARD);
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

  UartDebug_Printf("%s: ", label);
  App_PrintRpm("target: ", target_rpm_x10);
  UartDebug_SendString(", ");
  App_PrintRpm("measured: ", feedback->rpm_x10);
  UartDebug_SendString(", ");
  App_PrintRpm("error: ", error_rpm_x10);
  UartDebug_Printf(", PWM: %ld%s\r\n",
                   (long)((WheelSpeedPi_GetOutput(controller) < 0) ?
                          -(int64_t)WheelSpeedPi_GetOutput(controller) :
                          (int64_t)WheelSpeedPi_GetOutput(controller)),
                   (controller->saturated != 0U) ? " SAT" : "");
}

static void App_Start(void)
{
  ChassisSpeedControl_Enable();
  ChassisSpeedControl_SetMotionTarget(App_GetChassisMotion(), SPEED_TEST_TARGET_RPM_X10);
  stall_announced = 0U;
  UartDebug_SendString("Four-wheel speed control started.\r\n");
  UartDebug_Printf("Motion: %s\r\n", App_MotionName(selected_motion));
  UartDebug_SendString("Target speed: 30.0 RPM\r\n");
}

static void App_ProcessKeys(void)
{
  switch (Key_GetEvent())
  {
    case KEY_EVENT_KEY1:
      if (ChassisSpeedControl_IsEnabled() != 0U)
      {
        UartDebug_SendString("Stop the speed controller before changing motion.\r\n");
      }
      else
      {
        selected_motion++;
        if (selected_motion >= TEST_MOTION_COUNT)
        {
          selected_motion = TEST_MOTION_FORWARD;
        }
        UartDebug_Printf("Selected motion: %s\r\n", App_MotionName(selected_motion));
      }
      break;
    case KEY_EVENT_KEY2:
      if (ChassisSpeedControl_IsEnabled() == 0U)
      {
        App_Start();
      }
      else
      {
        ChassisSpeedControl_Stop();
        UartDebug_SendString("Four-wheel speed control stopped.\r\n");
      }
      break;
    case KEY_EVENT_WK_UP:
      ChassisSpeedControl_Disable();
      UartDebug_SendString("Emergency stop. Speed control disabled.\r\n");
      break;
    case KEY_EVENT_NONE:
    default:
      break;
  }
}

static void App_Report(void)
{
  uint32_t now = HAL_GetTick();
  uint32_t elapsed_ms;
  int32_t left_rpm_x10;
  int32_t right_rpm_x10;
  int32_t difference_rpm_x10;
  const char *controller_state;

  if ((uint32_t)(now - last_report_tick) < SPEED_REPORT_PERIOD_MS)
  {
    return;
  }
  elapsed_ms = (uint32_t)(now - last_report_tick);
  last_report_tick = now;
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

  UartDebug_Printf("Selected: %s, Controller: %s, Driver: %s, dt: %lu ms\r\n",
                   App_MotionName(selected_motion),
                   controller_state,
                   (Chassis_IsEnabled() != 0U) ? "ENABLED" : "DISABLED",
                   (unsigned long)elapsed_ms);
  UartDebug_Printf("Control period: %u ms\r\n", (unsigned int)SPEED_CONTROL_PERIOD_MS);
  UartDebug_SendString("Target magnitude: 30.0 RPM\r\n");
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

void App_FourMotorSpeedPidTest_Init(void)
{
  Key_Init();
  ChassisSpeedControl_Init();
  selected_motion = TEST_MOTION_FORWARD;
  last_control_tick = HAL_GetTick();
  last_report_tick = last_control_tick;
  stall_announced = 0U;

  UartDebug_SendString("\r\nFour_Motor_SpeedPid_Test started.\r\n");
  UartDebug_SendString("Four independent wheel speed PI controllers.\r\nWheel mapping:\r\n");
  UartDebug_SendString("FL = Motor D\r\nFR = Motor C\r\nRL = Motor B\r\nRR = Motor A\r\n");
  UartDebug_SendString("Vehicle forward electrical mapping:\r\nFL(D) = FWD\r\nFR(C) = FWD\r\nRL(B) = REV\r\nRR(A) = REV\r\n");
  UartDebug_SendString("Logical encoder signs:\r\nA = -1\r\nB = +1\r\nC = -1\r\nD = +1\r\n");
  UartDebug_SendString("Encoder counts per revolution: 1560\r\nTarget speed: 30.0 RPM\r\n");
  UartDebug_SendString("Control period: 100 ms\r\nKp: 0.80\r\nKi: 0.25\r\nMaximum test PWM: 300\r\n");
  UartDebug_SendString("KEY1: select motion while stopped\r\nKEY2: start or stop four-wheel speed control\r\n");
  UartDebug_SendString("WK_UP: emergency stop and disable chassis\r\nMotion changes must pass through STOP.\r\n");
  UartDebug_SendString("All wheels must remain lifted.\r\nSpeed control is disabled at startup.\r\n");
}

void App_FourMotorSpeedPidTest_Task(void)
{
  uint32_t now;
  uint32_t elapsed_ms;

  App_ProcessKeys();
  now = HAL_GetTick();
  if ((uint32_t)(now - last_control_tick) >= SPEED_CONTROL_PERIOD_MS)
  {
    elapsed_ms = (uint32_t)(now - last_control_tick);
    last_control_tick = now;
    ChassisSpeedControl_Update(elapsed_ms);
  }
  if ((ChassisSpeedControl_HasStall() != 0U) && (stall_announced == 0U))
  {
    switch (ChassisSpeedControl_GetStallWheel())
    {
      case CHASSIS_WHEEL_FRONT_LEFT: UartDebug_SendString("Wheel stall detected: FL(D).\r\n"); break;
      case CHASSIS_WHEEL_FRONT_RIGHT: UartDebug_SendString("Wheel stall detected: FR(C).\r\n"); break;
      case CHASSIS_WHEEL_REAR_LEFT: UartDebug_SendString("Wheel stall detected: RL(B).\r\n"); break;
      case CHASSIS_WHEEL_REAR_RIGHT: UartDebug_SendString("Wheel stall detected: RR(A).\r\n"); break;
      default: break;
    }
    UartDebug_SendString("Speed control stopped for safety.\r\n");
    stall_announced = 1U;
  }
  App_Report();
}
