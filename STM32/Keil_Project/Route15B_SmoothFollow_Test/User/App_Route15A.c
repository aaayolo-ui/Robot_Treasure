#include "App_Route15A.h"
#include "ChassisSpeedControl.h"
#include "gray_coordinate.h"
#include "gray_position.h"
#include "Key.h"
#include "LineFollowPID.h"
#include "FollowSmoothing.h"
#include "nchd12.h"
#include "nchd12_rear.h"
#include "SystemTime.h"
#include "TftRoutePage.h"
#include "UartDebug.h"

#define ROUTE_LEAVE_START_RPM_X10          300L
#define ROUTE_TURN_RPM_X10                 280L
#define ROUTE_TURN_STOP_MS                  180U
#define ROUTE_TURN_SETTLE_MS                150U
#define ROUTE_TURN_TIMEOUT_MS              4000U
#define ROUTE_MAX_RUN_MS                  60000U
#define ROUTE_REAR_VALID_REQUIRED             3U
#define ROUTE_REARM_VALID_REQUIRED            3U
#define ROUTE_TURN_CENTER_ERROR_X100        100

typedef enum
{
  ROUTE_GRAY_UNKNOWN = 0,
  ROUTE_GRAY_VALID,
  ROUTE_GRAY_NO_LINE,
  ROUTE_GRAY_ALL_BLACK,
  ROUTE_GRAY_WIDE_BLACK,
  ROUTE_GRAY_I2C_ERROR,
  ROUTE_GRAY_UNAVAILABLE
} RouteGrayState_t;

typedef enum
{
  ROUTE_FOLLOW_FRONT_ONLY = 0,
  ROUTE_FOLLOW_DUAL
} RouteFollowMode_t;

typedef enum
{
  ROUTE_STATE_IDLE = 0,
  ROUTE_STATE_LEAVE_START,
  ROUTE_STATE_FOLLOW,
  ROUTE_STATE_TURN_STOP,
  ROUTE_STATE_TURN_RIGHT,
  ROUTE_STATE_TURN_SETTLE,
  ROUTE_STATE_DONE,
  ROUTE_STATE_FAULT
} RouteState_t;

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
} RouteGraySample_t;

static LineFollowPID_Controller_t line_follow_pid;
static FollowSmoothing_t follow_smoothing;
static uint8_t route_active;
static uint8_t rear_hw_ready;
static uint8_t rear_valid_count;
static uint8_t junction_count;
static uint8_t junction_armed;
static uint8_t junction_latched;
static uint8_t front_was_valid;
static uint8_t rearm_valid_count;
static uint8_t turn_center_valid_count;
static RouteState_t route_state;
static RouteFollowMode_t follow_mode;
static RouteGrayState_t front_state;
static RouteGrayState_t rear_state;
static uint32_t run_start_tick;
static uint32_t route_state_tick;
static uint32_t last_control_tick;

static const char *Route_StateName(RouteState_t state)
{
  static const char *const names[] = {
      "IDLE", "LEAVE", "FOLLOW", "STOP", "TURN", "SETTLE", "DONE",
      "FAULT"};

  return ((uint32_t)state < (sizeof(names) / sizeof(names[0]))) ?
         names[state] : "FAULT";
}

static const char *Route_GrayStateName(RouteGrayState_t state)
{
  static const char *const names[] = {
      "UNKNOWN", "VALID", "NO_LINE", "ALL_BLACK", "WIDE_BLACK",
      "I2C_ERROR", "UNAVAILABLE"};

  return ((uint32_t)state < (sizeof(names) / sizeof(names[0]))) ?
         names[state] : "UNKNOWN";
}

static int32_t Route_Absolute(int32_t value)
{
  return (value < 0) ? -value : value;
}

static void Route_ResetSample(RouteGraySample_t *sample)
{
  sample->front_raw12 = 0U;
  sample->rear_raw12 = 0U;
  sample->front_error_x100 = 0;
  sample->rear_error_x100 = 0;
  sample->position_error_x100 = 0;
  sample->heading_error_x100 = 0;
}

static RouteGrayState_t Route_ClassifyPosition(
    const GrayPosition_Result_t *position)
{
  if (position == 0)
  {
    return ROUTE_GRAY_I2C_ERROR;
  }
  if (position->status == GRAY_POSITION_STATUS_NO_LINE)
  {
    return ROUTE_GRAY_NO_LINE;
  }
  if (position->status == GRAY_POSITION_STATUS_ALL_BLACK)
  {
    return ROUTE_GRAY_ALL_BLACK;
  }
  if ((position->valid == 0U) || (position->black_count > 4U))
  {
    return ROUTE_GRAY_WIDE_BLACK;
  }
  return ROUTE_GRAY_VALID;
}

static void Route_ResetControl(uint8_t enable)
{
  ChassisSpeedControl_Disable();
  LineFollowPID_Reset(&line_follow_pid);
  FollowSmoothing_Reset(&follow_smoothing);
  if (enable != 0U)
  {
    ChassisSpeedControl_Enable();
  }
}

static void Route_StopDone(void)
{
  Route_ResetControl(0U);
  route_active = 0U;
  route_state = ROUTE_STATE_DONE;
  UartDebug_Printf("ROUTE_DONE JUNCTION=%u\r\n", (unsigned)junction_count);
}

static void Route_StopFault(const char *reason)
{
  Route_ResetControl(0U);
  route_active = 0U;
  route_state = ROUTE_STATE_FAULT;
  UartDebug_Printf("ROUTE_FAULT=%s JUNCTION=%u\r\n", reason,
                   (unsigned)junction_count);
}

static uint8_t Route_InitializeGray(void)
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

static RouteGrayState_t Route_ReadFront(RouteGraySample_t *sample)
{
  uint16_t raw16;
  RouteGrayState_t state;

  if (NCHD12_ReadRaw16(&raw16) != NCHD12_STATUS_OK)
  {
    return ROUTE_GRAY_I2C_ERROR;
  }
  sample->front_raw12 = NCHD12_Extract12(raw16);
  GrayPosition_Calculate(sample->front_raw12, &sample->front_position);
  state = Route_ClassifyPosition(&sample->front_position);
  if (state != ROUTE_GRAY_VALID)
  {
    return state;
  }
  if (!GrayCoordinate_ToVehicleError(sample->front_position.position_x100,
                                     GRAY_MOUNT_P12_LEFT_P1_RIGHT,
                                     &sample->front_error_x100))
  {
    return ROUTE_GRAY_I2C_ERROR;
  }
  return ROUTE_GRAY_VALID;
}

static RouteGrayState_t Route_ReadRear(RouteGraySample_t *sample)
{
  uint16_t raw16;
  RouteGrayState_t state;

  if (rear_hw_ready == 0U)
  {
    if (NCHD12Rear_ConfigureInputs() != NCHD12_STATUS_OK)
    {
      return ROUTE_GRAY_UNAVAILABLE;
    }
    rear_hw_ready = 1U;
  }
  if (NCHD12Rear_ReadRaw16(&raw16) != NCHD12_STATUS_OK)
  {
    rear_hw_ready = 0U;
    return ROUTE_GRAY_I2C_ERROR;
  }
  sample->rear_raw12 = NCHD12_Extract12(raw16);
  GrayPosition_Calculate(sample->rear_raw12, &sample->rear_position);
  state = Route_ClassifyPosition(&sample->rear_position);
  if (state != ROUTE_GRAY_VALID)
  {
    return state;
  }
  if (!GrayCoordinate_ToVehicleError(sample->rear_position.position_x100,
                                     GRAY_MOUNT_P1_LEFT_P12_RIGHT,
                                     &sample->rear_error_x100))
  {
    return ROUTE_GRAY_I2C_ERROR;
  }
  return ROUTE_GRAY_VALID;
}

static void Route_UpdateErrors(RouteGraySample_t *sample)
{
  if ((follow_mode == ROUTE_FOLLOW_DUAL) &&
      (rear_state == ROUTE_GRAY_VALID))
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

static void Route_UpdateFollowMode(RouteGraySample_t *sample)
{
  rear_state = Route_ReadRear(sample);
  if (rear_state == ROUTE_GRAY_VALID)
  {
    if (rear_valid_count < ROUTE_REAR_VALID_REQUIRED)
    {
      rear_valid_count++;
    }
    if ((follow_mode == ROUTE_FOLLOW_FRONT_ONLY) &&
        (rear_valid_count >= ROUTE_REAR_VALID_REQUIRED))
    {
      follow_mode = ROUTE_FOLLOW_DUAL;
      LineFollowPID_Reset(&line_follow_pid);
      UartDebug_SendString("FOLLOW_MODE=DUAL\r\n");
    }
  }
  else
  {
    rear_valid_count = 0U;
    if (follow_mode == ROUTE_FOLLOW_DUAL)
    {
      follow_mode = ROUTE_FOLLOW_FRONT_ONLY;
      LineFollowPID_Reset(&line_follow_pid);
      UartDebug_Printf("FOLLOW_MODE=FRONT_ONLY REAR=%s\r\n",
                       Route_GrayStateName(rear_state));
    }
  }
  Route_UpdateErrors(sample);
}

static void Route_DriveStraight(uint32_t elapsed_ms)
{
  ChassisSpeedControl_SetWheelTargets(
      ROUTE_LEAVE_START_RPM_X10, ROUTE_LEAVE_START_RPM_X10,
      ROUTE_LEAVE_START_RPM_X10, ROUTE_LEAVE_START_RPM_X10);
  ChassisSpeedControl_Update(elapsed_ms);
}

static void Route_DriveRightTurn(uint32_t elapsed_ms)
{
  ChassisSpeedControl_SetWheelTargets(
      ROUTE_TURN_RPM_X10, -ROUTE_TURN_RPM_X10,
      ROUTE_TURN_RPM_X10, -ROUTE_TURN_RPM_X10);
  ChassisSpeedControl_Update(elapsed_ms);
}

static void Route_EnterFollow(uint32_t now)
{
  Route_ResetControl(1U);
  route_state = ROUTE_STATE_FOLLOW;
  route_state_tick = now;
  follow_mode = ROUTE_FOLLOW_FRONT_ONLY;
  rear_valid_count = 0U;
  front_was_valid = 0U;
  junction_armed = 0U;
  junction_latched = 0U;
  rearm_valid_count = 0U;
  UartDebug_Printf("STATE=FOLLOW JUNCTION=%u\r\n",
                   (unsigned)junction_count);
}

static void Route_EnterTurnStop(uint32_t now)
{
  Route_ResetControl(0U);
  route_state = ROUTE_STATE_TURN_STOP;
  route_state_tick = now;
  turn_center_valid_count = 0U;
  UartDebug_Printf("STATE=STOP_FOR_TURN JUNCTION=%u\r\n",
                   (unsigned)junction_count);
}

static void Route_EnterTurnSettle(uint32_t now)
{
  Route_ResetControl(0U);
  route_state = ROUTE_STATE_TURN_SETTLE;
  route_state_tick = now;
  UartDebug_Printf("STATE=SETTLE JUNCTION=%u\r\n",
                   (unsigned)junction_count);
}

static uint8_t Route_RunFollow(RouteGraySample_t *sample,
                               uint32_t elapsed_ms)
{
  LineFollowPID_Output_t control;

  Route_UpdateFollowMode(sample);
  LineFollowPID_Calculate(&line_follow_pid, sample->position_error_x100,
                          elapsed_ms, &control);
  if (follow_mode == ROUTE_FOLLOW_DUAL)
  {
    LineFollowPID_ApplyHeadingP(&control, sample->heading_error_x100);
  }
  FollowSmoothing_Apply(&follow_smoothing, sample->position_error_x100,
                         elapsed_ms, &control);
  ChassisSpeedControl_SetWheelTargets(control.left_target_rpm_x10,
                                      control.right_target_rpm_x10,
                                      control.left_target_rpm_x10,
                                      control.right_target_rpm_x10);
  ChassisSpeedControl_Update(elapsed_ms);
  if (ChassisSpeedControl_HasStall() != 0U)
  {
    Route_StopFault("STALL");
    return 0U;
  }

  front_was_valid = 1U;
  if (junction_armed == 0U)
  {
    if (rearm_valid_count < ROUTE_REARM_VALID_REQUIRED)
    {
      rearm_valid_count++;
    }
    if (rearm_valid_count >= ROUTE_REARM_VALID_REQUIRED)
    {
      junction_armed = 1U;
      junction_latched = 0U;
      UartDebug_Printf("JUNCTION_ARMED=%u\r\n", (unsigned)junction_count);
    }
  }
  return 1U;
}

static uint8_t Route_IsJunctionCandidate(RouteGrayState_t state)
{
  return ((state == ROUTE_GRAY_WIDE_BLACK) ||
          (state == ROUTE_GRAY_ALL_BLACK)) ? 1U : 0U;
}

static void Route_HandleJunction(uint32_t now)
{
  junction_latched = 1U;
  junction_armed = 0U;
  front_was_valid = 0U;
  if (junction_count < 3U)
  {
    junction_count++;
  }
  UartDebug_Printf("JUNCTION_DETECTED=%u\r\n", (unsigned)junction_count);

  if (junction_count >= 3U)
  {
    Route_StopDone();
  }
  else
  {
    Route_EnterTurnStop(now);
  }
}

static void Route_Start(void)
{
  RouteGraySample_t sample;
  uint32_t now;

  Route_ResetControl(0U);
  route_active = 0U;
  rear_valid_count = 0U;
  junction_count = 0U;
  junction_armed = 0U;
  junction_latched = 0U;
  front_was_valid = 0U;
  rearm_valid_count = 0U;
  turn_center_valid_count = 0U;
  follow_mode = ROUTE_FOLLOW_FRONT_ONLY;
  front_state = ROUTE_GRAY_UNKNOWN;
  rear_state = ROUTE_GRAY_UNKNOWN;
  Route_ResetSample(&sample);

  if (Route_InitializeGray() == 0U)
  {
    front_state = ROUTE_GRAY_I2C_ERROR;
    Route_StopFault("FRONT_I2C");
    return;
  }
  front_state = Route_ReadFront(&sample);
  UartDebug_Printf("START_FRONT=%s RAW12=%u ERROR=%d\r\n",
                   Route_GrayStateName(front_state),
                   (unsigned)sample.front_raw12,
                   (int)sample.front_error_x100);
  if ((front_state != ROUTE_GRAY_ALL_BLACK) &&
      (front_state != ROUTE_GRAY_VALID))
  {
    Route_StopFault("START_FRONT_INVALID");
    return;
  }

  now = SystemTime_GetMs();
  route_active = 1U;
  run_start_tick = now;
  route_state_tick = now;
  last_control_tick = now;
  if (front_state == ROUTE_GRAY_ALL_BLACK)
  {
    Route_ResetControl(1U);
    route_state = ROUTE_STATE_LEAVE_START;
    UartDebug_SendString("START_OK STATE=LEAVE\r\n");
  }
  else
  {
    Route_EnterFollow(now);
    UartDebug_SendString("START_OK STATE=FOLLOW\r\n");
  }
}

void App_Route15A_Init(void)
{
  Key_Init();
  ChassisSpeedControl_Init();
  LineFollowPID_Reset(&line_follow_pid);
  FollowSmoothing_Reset(&follow_smoothing);
  route_active = 0U;
  rear_hw_ready = 0U;
  rear_valid_count = 0U;
  junction_count = 0U;
  junction_armed = 0U;
  junction_latched = 0U;
  front_was_valid = 0U;
  rearm_valid_count = 0U;
  turn_center_valid_count = 0U;
  route_state = ROUTE_STATE_IDLE;
  follow_mode = ROUTE_FOLLOW_FRONT_ONLY;
  front_state = ROUTE_GRAY_UNKNOWN;
  rear_state = ROUTE_GRAY_UNKNOWN;
  run_start_tick = SystemTime_GetMs();
  route_state_tick = run_start_tick;
  last_control_tick = run_start_tick;
  UartDebug_SendString("Route15A_TwoRightTurn_Stop_Test\r\n");
  UartDebug_SendString("KEY2=start KEY1/WK_UP=stop\r\n");
  TftRoutePage_Init();
}

void App_Route15A_Task(void)
{
  KeyEvent_t event = Key_GetEvent();
  RouteGraySample_t sample;
  uint32_t now = SystemTime_GetMs();
  uint32_t elapsed_ms;

  if (route_active != 0U)
  {
    if ((Key_IsPressed(KEY_ID_KEY1) != 0U) ||
        (Key_IsPressed(KEY_ID_WK_UP) != 0U) ||
        (event == KEY_EVENT_KEY2_SHORT))
    {
      Route_StopFault("KEY_STOP");
      goto task_end;
    }
  }
  else if ((event == KEY_EVENT_KEY2_SHORT) ||
           (event == KEY_EVENT_KEY2_LONG))
  {
    Route_Start();
    goto task_end;
  }

  if ((route_active == 0U) ||
      ((uint32_t)(now - last_control_tick) <
       LINE_FOLLOW_PID_CONTROL_PERIOD_MS))
  {
    goto task_end;
  }

  elapsed_ms = (uint32_t)(now - last_control_tick);
  last_control_tick = now;
  if ((uint32_t)(now - run_start_tick) >= ROUTE_MAX_RUN_MS)
  {
    Route_StopFault("RUN_TIMEOUT");
    goto task_end;
  }

  switch (route_state)
  {
    case ROUTE_STATE_LEAVE_START:
      Route_ResetSample(&sample);
      front_state = Route_ReadFront(&sample);
      if (front_state == ROUTE_GRAY_I2C_ERROR)
      {
        Route_StopFault("FRONT_I2C");
      }
      else if (front_state == ROUTE_GRAY_VALID)
      {
        Route_EnterFollow(now);
      }
      else
      {
        Route_DriveStraight(elapsed_ms);
        if (ChassisSpeedControl_HasStall() != 0U)
        {
          Route_StopFault("STALL");
        }
      }
      break;

    case ROUTE_STATE_FOLLOW:
      Route_ResetSample(&sample);
      front_state = Route_ReadFront(&sample);
      if (front_state == ROUTE_GRAY_VALID)
      {
        (void)Route_RunFollow(&sample, elapsed_ms);
      }
      else if ((Route_IsJunctionCandidate(front_state) != 0U) &&
               (junction_armed != 0U) &&
               (junction_latched == 0U) &&
               (front_was_valid != 0U))
      {
        Route_HandleJunction(now);
      }
      else
      {
        Route_StopFault((front_state == ROUTE_GRAY_I2C_ERROR) ?
                        "FRONT_I2C" : "FRONT_NOT_VALID");
      }
      break;

    case ROUTE_STATE_TURN_STOP:
      if ((uint32_t)(now - route_state_tick) >= ROUTE_TURN_STOP_MS)
      {
        Route_ResetControl(1U);
        route_state = ROUTE_STATE_TURN_RIGHT;
        route_state_tick = now;
        turn_center_valid_count = 0U;
        UartDebug_Printf("STATE=TURN JUNCTION=%u\r\n",
                         (unsigned)junction_count);
      }
      break;

    case ROUTE_STATE_TURN_RIGHT:
      if ((uint32_t)(now - route_state_tick) >= ROUTE_TURN_TIMEOUT_MS)
      {
        Route_StopFault("TURN_TIMEOUT");
        break;
      }
      Route_ResetSample(&sample);
      front_state = Route_ReadFront(&sample);
      if (front_state == ROUTE_GRAY_I2C_ERROR)
      {
        Route_StopFault("FRONT_I2C");
        break;
      }
      Route_DriveRightTurn(elapsed_ms);
      if ((front_state == ROUTE_GRAY_VALID) &&
          (Route_Absolute((int32_t)sample.front_error_x100) <=
           ROUTE_TURN_CENTER_ERROR_X100))
      {
        if (turn_center_valid_count < ROUTE_REARM_VALID_REQUIRED)
        {
          turn_center_valid_count++;
        }
        if (turn_center_valid_count >= ROUTE_REARM_VALID_REQUIRED)
        {
          Route_EnterTurnSettle(now);
        }
      }
      else
      {
        turn_center_valid_count = 0U;
      }
      if ((route_active != 0U) && (ChassisSpeedControl_HasStall() != 0U))
      {
        Route_StopFault("STALL");
      }
      break;

    case ROUTE_STATE_TURN_SETTLE:
      if ((uint32_t)(now - route_state_tick) >= ROUTE_TURN_SETTLE_MS)
      {
        Route_EnterFollow(now);
      }
      break;

    case ROUTE_STATE_IDLE:
    case ROUTE_STATE_DONE:
    case ROUTE_STATE_FAULT:
    default:
      Route_StopFault("STATE");
      break;
  }

task_end:
  TftRoutePage_Task(Route_StateName(route_state), junction_count);
}
