#include "App_ChassisMotionTest.h"
#include "Chassis.h"
#include "Key.h"
#include "UartDebug.h"
#include "main.h"

#define CHASSIS_TEST_PWM             100U
#define CHASSIS_REPORT_PERIOD_MS     500U

typedef enum
{
  TEST_MOTION_FORWARD = 0,
  TEST_MOTION_BACKWARD,
  TEST_MOTION_TURN_LEFT,
  TEST_MOTION_TURN_RIGHT,
  TEST_MOTION_COUNT
} TestMotion_t;

static TestMotion_t selected_motion;
static uint32_t last_report_tick;

static const char *App_TestMotionName(TestMotion_t motion)
{
  switch (motion)
  {
    case TEST_MOTION_FORWARD:
      return "FORWARD";
    case TEST_MOTION_BACKWARD:
      return "BACKWARD";
    case TEST_MOTION_TURN_LEFT:
      return "TURN_LEFT";
    case TEST_MOTION_TURN_RIGHT:
      return "TURN_RIGHT";
    default:
      return "FORWARD";
  }
}

static const char *App_ChassisMotionName(ChassisMotion_t motion)
{
  switch (motion)
  {
    case CHASSIS_MOTION_FORWARD:
      return "FORWARD";
    case CHASSIS_MOTION_BACKWARD:
      return "BACKWARD";
    case CHASSIS_MOTION_TURN_LEFT:
      return "TURN_LEFT";
    case CHASSIS_MOTION_TURN_RIGHT:
      return "TURN_RIGHT";
    case CHASSIS_MOTION_STOP:
    default:
      return "STOP";
  }
}

static void App_SplitRpmX10(int32_t rpm_x10,
                            const char **sign,
                            long *integer,
                            long *decimal)
{
  int64_t value;
  int64_t absolute_value;

  value = (int64_t)rpm_x10;
  absolute_value = (value < 0) ? -value : value;
  *sign = (value < 0) ? "-" : "";
  *integer = (long)(absolute_value / 10LL);
  *decimal = (long)(absolute_value % 10LL);
}

static void App_PrintWheelFeedback(const char *label,
                                   ChassisWheelId_t wheel)
{
  const ChassisWheelFeedback_t *feedback;
  const char *sign;
  long rpm_integer;
  long rpm_decimal;

  feedback = Chassis_GetWheelFeedback(wheel);
  App_SplitRpmX10(feedback->rpm_x10, &sign, &rpm_integer, &rpm_decimal);
  UartDebug_Printf("%s: delta: %d, total: %ld, RPM: %s%ld.%ld\r\n",
                   label,
                   (int)feedback->delta_count,
                   (long)feedback->total_count,
                   sign,
                   rpm_integer,
                   rpm_decimal);
}

static void App_SelectNextMotion(void)
{
  if (Chassis_GetMotion() != CHASSIS_MOTION_STOP)
  {
    UartDebug_SendString("Stop the chassis before changing motion.\r\n");
    return;
  }

  selected_motion++;
  if (selected_motion >= TEST_MOTION_COUNT)
  {
    selected_motion = TEST_MOTION_FORWARD;
  }
  UartDebug_Printf("Selected motion: %s\r\n", App_TestMotionName(selected_motion));
}

static void App_StartSelectedMotion(void)
{
  switch (selected_motion)
  {
    case TEST_MOTION_FORWARD:
      Chassis_Forward(CHASSIS_TEST_PWM);
      UartDebug_SendString("Chassis forward started at PWM 100.\r\n");
      break;
    case TEST_MOTION_BACKWARD:
      Chassis_Backward(CHASSIS_TEST_PWM);
      UartDebug_SendString("Chassis backward started at PWM 100.\r\n");
      break;
    case TEST_MOTION_TURN_LEFT:
      Chassis_TurnLeft(CHASSIS_TEST_PWM);
      UartDebug_SendString("Chassis turn left started at PWM 100.\r\n");
      break;
    case TEST_MOTION_TURN_RIGHT:
      Chassis_TurnRight(CHASSIS_TEST_PWM);
      UartDebug_SendString("Chassis turn right started at PWM 100.\r\n");
      break;
    default:
      break;
  }
}

static void App_ProcessKeys(void)
{
  switch (Key_GetEvent())
  {
    case KEY_EVENT_KEY1:
      App_SelectNextMotion();
      break;

    case KEY_EVENT_KEY2:
      if (Chassis_GetMotion() == CHASSIS_MOTION_STOP)
      {
        App_StartSelectedMotion();
      }
      else
      {
        Chassis_Stop();
        UartDebug_SendString("Chassis stopped.\r\n");
      }
      break;

    case KEY_EVENT_WK_UP:
      Chassis_Disable();
      UartDebug_SendString("Emergency stop. Chassis disabled.\r\n");
      break;

    case KEY_EVENT_NONE:
    default:
      break;
  }
}

static void App_Report(void)
{
  uint32_t now;
  uint32_t elapsed_ms;
  int32_t left_rpm_x10;
  int32_t right_rpm_x10;
  int32_t difference_rpm_x10;
  const char *left_sign;
  const char *right_sign;
  const char *difference_sign;
  long left_integer;
  long left_decimal;
  long right_integer;
  long right_decimal;
  long difference_integer;
  long difference_decimal;

  now = HAL_GetTick();
  if ((uint32_t)(now - last_report_tick) < CHASSIS_REPORT_PERIOD_MS)
  {
    return;
  }

  elapsed_ms = (uint32_t)(now - last_report_tick);
  last_report_tick = now;
  Chassis_UpdateFeedback(elapsed_ms);
  left_rpm_x10 = Chassis_GetLeftAverageRpmX10();
  right_rpm_x10 = Chassis_GetRightAverageRpmX10();
  difference_rpm_x10 = left_rpm_x10 - right_rpm_x10;
  App_SplitRpmX10(left_rpm_x10, &left_sign, &left_integer, &left_decimal);
  App_SplitRpmX10(right_rpm_x10, &right_sign, &right_integer, &right_decimal);
  App_SplitRpmX10(difference_rpm_x10,
                   &difference_sign,
                   &difference_integer,
                   &difference_decimal);

  UartDebug_Printf("Selected: %s, Motion: %s, Driver: %s, PWM: %u, dt: %lu ms\r\n",
                   App_TestMotionName(selected_motion),
                   App_ChassisMotionName(Chassis_GetMotion()),
                   (Chassis_IsEnabled() != 0U) ? "ENABLED" : "DISABLED",
                   (unsigned int)Chassis_GetPwm(),
                   (unsigned long)elapsed_ms);
  App_PrintWheelFeedback("FL(D)", CHASSIS_WHEEL_FRONT_LEFT);
  App_PrintWheelFeedback("FR(C)", CHASSIS_WHEEL_FRONT_RIGHT);
  App_PrintWheelFeedback("RL(B)", CHASSIS_WHEEL_REAR_LEFT);
  App_PrintWheelFeedback("RR(A)", CHASSIS_WHEEL_REAR_RIGHT);
  UartDebug_Printf("Left average RPM: %s%ld.%ld\r\n",
                   left_sign, left_integer, left_decimal);
  UartDebug_Printf("Right average RPM: %s%ld.%ld\r\n",
                   right_sign, right_integer, right_decimal);
  UartDebug_Printf("Left-right difference RPM: %s%ld.%ld\r\n\r\n",
                   difference_sign, difference_integer, difference_decimal);
}

void App_ChassisMotionTest_Init(void)
{
  Key_Init();
  Chassis_Init();
  selected_motion = TEST_MOTION_FORWARD;
  last_report_tick = HAL_GetTick();

  UartDebug_SendString("\r\n");
  UartDebug_SendString("Four_Motor_Chassis_Test started.\r\n");
  UartDebug_SendString("Four wheel chassis motion test.\r\n");
  UartDebug_SendString("Wheel mapping:\r\n");
  UartDebug_SendString("FL = Motor D\r\n");
  UartDebug_SendString("FR = Motor C\r\n");
  UartDebug_SendString("RL = Motor B\r\n");
  UartDebug_SendString("RR = Motor A\r\n");
  UartDebug_SendString("Front motors use electrical forward for vehicle forward.\r\n");
  UartDebug_SendString("Rear motors use electrical reverse for vehicle forward.\r\n");
  UartDebug_SendString("Vehicle forward electrical mapping:\r\n");
  UartDebug_SendString("FL(D) = FWD\r\n");
  UartDebug_SendString("FR(C) = FWD\r\n");
  UartDebug_SendString("RL(B) = REV\r\n");
  UartDebug_SendString("RR(A) = REV\r\n");
  UartDebug_SendString("Logical encoder signs:\r\n");
  UartDebug_SendString("A = -1\r\n");
  UartDebug_SendString("B = +1\r\n");
  UartDebug_SendString("C = -1\r\n");
  UartDebug_SendString("D = +1\r\n");
  UartDebug_SendString("KEY1: select forward, backward, turn left or turn right\r\n");
  UartDebug_SendString("KEY2: start or stop selected motion\r\n");
  UartDebug_SendString("WK_UP: emergency stop and disable chassis\r\n");
  UartDebug_SendString("Test PWM: 100\r\n");
  UartDebug_SendString("Motion changes must pass through STOP.\r\n");
  UartDebug_SendString("All wheels must remain lifted.\r\n");
  UartDebug_SendString("Chassis is disabled at startup.\r\n");
}

void App_ChassisMotionTest_Task(void)
{
  App_ProcessKeys();
  App_Report();
}
