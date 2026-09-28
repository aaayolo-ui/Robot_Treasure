#include "App_LineFollowExperiment.h"
#include "Chassis.h"
#include "ChassisSpeedControl.h"
#include "gray_coordinate.h"
#include "gray_position.h"
#include "Key.h"
#include "LineFollowPID.h"
#include "FollowSmoothing.h"
#include "JY61P.h"
#include "nchd12.h"
#include "nchd12_rear.h"
#include "SystemTime.h"
#include "TftRoutePage.h"
#include "UartDebug.h"

/* RPM targets use x10 units. Keep the user's cruise/turn settings separate. */
#define ROUTE_LEAVE_START_RPM_X10          1000L
#define ROUTE_LEAVE_INITIAL_RPM_X10        1000L
#define ROUTE_LEAVE_RAMP_STEP_RPM_X10       10L
#define ROUTE_AFTER_TURN_START_RPM_X10     1000L
#define ROUTE_TURN_EQUAL_PWM                500L
#define ROUTE_TURN_APPROACH_RPM_X10       1000L
#define ROUTE_TURN_APPROACH_DELAY_MS        200U /* Controlled forward approach before braking. */
#define ROUTE_TURN_STOP_MS                  80U
#define ROUTE_TURN_STOP_MAX_MS             800U /* Fault braked if wheels do not settle. */
#define ROUTE_TURN_STOP_SPEED_RPM_X10        20L /* About 2 RPM. */
#define ROUTE_TURN_STOP_STABLE_SAMPLES        2U
#define ROUTE_TURN_GYRO_WAIT_MS             500U
#define ROUTE_TURN_TIMEOUT_MS              4000U
#define ROUTE_TURN_TARGET_ANGLE_X100       5000L /* 50.00 deg of measured YAW. */
#define ROUTE_JY61P_STALE_MS                150U /* No fresh angle during turn: stop safely. */
#define ROUTE_JUNCTION_WATCH_PERIOD_MS        5U /* Fast gray watch; PID stays at 20 ms. */
#define ROUTE_FRONT_RECOVERY_MS             200U /* Brake and retry a transient front-gray fault. */
#define ROUTE_FRONT_RECOVERY_VALID_SAMPLES    2U
#define ROUTE_GYRO_RECOVERY_MS              600U /* Brake while waiting for fresh YAW. */
#define ROUTE_AFTER_TURN_FORWARD_TIMEOUT_MS 1200U
#define ROUTE_MAX_RUN_MS                  180000U
#define ROUTE_REAR_VALID_REQUIRED             3U
#define ROUTE_REARM_VALID_REQUIRED            3U
#define ROUTE_LEAVE_TIMEOUT_MS             4000U
#define ROUTE_LEFT_FOUR_MASK             0x0F00U /* Front P9-P12: vehicle left */
#define ROUTE_RIGHT_FOUR_MASK            0x000FU /* Front P1-P4: vehicle right */
#define ROUTE_LEFT_P11_MASK              0x0400U /* Front P11: vehicle left */
#define ROUTE_LEFT_P12_MASK              0x0800U /* Front P12: vehicle left */
#define ROUTE_RIGHT_OUTER_TWO_MASK       0x0003U /* Front P1-P2: right turn */
#define ROUTE_TURN_EXIT_P1_P12_MASK      0x0801U /* Front P1 OR P12 after both release */
#define ROUTE_CENTER_EDGE_MASK           0x0090U /* Front P5/P8: small PID correction */
/* Gray PID zone gain: center edge = 400%, other center = 300%, side = 400%. */
#define ROUTE_PID_CENTER_EDGE_GAIN_PCT     400L
#define ROUTE_PID_CENTER_GAIN_PCT          300L
#define ROUTE_PID_SIDE_GAIN_PCT            400L

typedef enum
{
  ROUTE_GRAY_UNKNOWN = 0,
  ROUTE_GRAY_VALID,
  ROUTE_GRAY_NO_LINE,
  ROUTE_GRAY_ALL_BLACK,
  ROUTE_GRAY_LEFT_BLACK,
  ROUTE_GRAY_RIGHT_BLACK,
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
  ROUTE_STATE_TURN_APPROACH,
  ROUTE_STATE_TURN_STOP,
  ROUTE_STATE_TURN_RIGHT,
  ROUTE_STATE_TURN_LEFT,
  ROUTE_STATE_AFTER_TURN_FORWARD,
  ROUTE_STATE_FRONT_RECOVERY,
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
static uint8_t turn_exit_sensor_armed;
static uint8_t turn_stop_stable_count;
static int32_t leave_target_rpm_x10;
static RouteState_t route_state;
static RouteState_t turn_target_state;
static RouteFollowMode_t follow_mode;
static RouteGrayState_t front_state;
static RouteGrayState_t rear_state;
static uint32_t run_start_tick;
static uint32_t route_state_tick;
static uint32_t last_control_tick;
static uint32_t last_front_watch_tick;
static uint32_t turn_stop_frame_count;
static int16_t turn_start_angle_x100;
static RouteState_t front_resume_state;
static uint32_t front_resume_tick;
static int32_t front_resume_base_rpm_x10;
static uint8_t front_recovery_valid_count;
static uint8_t gyro_paused;
static uint32_t gyro_pause_tick;
static const char *route_fault_reason;

static const char *Route_StateName(RouteState_t state)
{
  static const char *const names[] = {
      "IDLE", "LEAVE", "FOLLOW", "APPROACH", "STOP", "TURN_RIGHT",
      "TURN_LEFT", "FORWARD", "FRONT_WAIT", "DONE", "FAULT"};

  return ((uint32_t)state < (sizeof(names) / sizeof(names[0]))) ?
         names[state] : "FAULT";
}

static const char *Route_GrayStateName(RouteGrayState_t state)
{
  static const char *const names[] = {
      "UNKNOWN", "VALID", "NO_LINE", "ALL_BLACK", "LEFT_BLACK", "RIGHT_BLACK", "WIDE_BLACK",
      "I2C_ERROR", "UNAVAILABLE"};

  return ((uint32_t)state < (sizeof(names) / sizeof(names[0]))) ?
         names[state] : "UNKNOWN";
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
    const GrayPosition_Result_t *position, uint8_t detect_branch)
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
  /* Both outer channels must see black; if both sides qualify, turn right. */
  if ((detect_branch != 0U) &&
      ((position->gray12 & ROUTE_RIGHT_OUTER_TWO_MASK) ==
       ROUTE_RIGHT_OUTER_TWO_MASK))
  {
    return ROUTE_GRAY_RIGHT_BLACK;
  }
  if ((detect_branch != 0U) &&
      ((position->gray12 & ROUTE_LEFT_P11_MASK) != 0U) &&
      ((position->gray12 & ROUTE_LEFT_P12_MASK) != 0U))
  {
    return ROUTE_GRAY_LEFT_BLACK;
  }
  if (position->valid == 0U)
  {
    return ROUTE_GRAY_WIDE_BLACK;
  }
  return ROUTE_GRAY_VALID;
}

/* The first junction frame can be asymmetric while the car is entering a T.
 * Keep checking the front board during the approach window. A right-side hit
 * or all-black frame latches RIGHT and cannot be changed back by later left
 * samples; a junction that remains left-only therefore still turns left. */
static void Route_ConfirmTurnDirection(uint16_t front_raw12)
{
  uint8_t right_seen =
      ((front_raw12 & ROUTE_RIGHT_OUTER_TWO_MASK) ==
       ROUTE_RIGHT_OUTER_TWO_MASK) ? 1U : 0U;

  if ((front_raw12 == 0x0FFFU) || (right_seen != 0U))
  {
    if (turn_target_state != ROUTE_STATE_TURN_RIGHT)
    {
      turn_target_state = ROUTE_STATE_TURN_RIGHT;
      UartDebug_Printf("TURN_DIR_UPDATE=RIGHT RAW12=%03X\r\n",
                       (unsigned)front_raw12);
    }
  }
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

static void Route_BrakeControl(void)
{
  /* Assert the hardware short brake before resetting any software state. */
  ChassisSpeedControl_Brake();
  LineFollowPID_Reset(&line_follow_pid);
  FollowSmoothing_Reset(&follow_smoothing);
}

static void Route_StopFault(const char *reason)
{
  RouteState_t fault_from = route_state;
  ChassisWheelId_t fault_wheel = ChassisSpeedControl_GetStallWheel();
  uint32_t now;
  uint32_t last_yaw_tick;

  /* Every abnormal running exit actively brakes. FOLLOW/LEAVE faults used to
   * disable STBY and coast, so a missed junction classification could slide
   * out of the track and then appear to "quit". */
  Route_BrakeControl();
  route_active = 0U;
  route_state = ROUTE_STATE_FAULT;
  route_fault_reason = reason;
  now = SystemTime_GetMs();
  last_yaw_tick = JY61P_GetLastUpdateMs();
  UartDebug_Printf(
      "ROUTE_FAULT=%s FROM=%s JUNCTION=%u YAW_AGE_MS=%lu FRAME=%lu "
      "CHECKSUM_ERR=%lu UART_ERR=%lu STALL_WHEEL=%u\r\n",
      reason, Route_StateName(fault_from), (unsigned)junction_count,
      (unsigned long)(now - last_yaw_tick),
      (unsigned long)JY61P_GetFrameCount(),
      (unsigned long)JY61P_GetChecksumErrorCount(),
      (unsigned long)JY61P_GetUartErrorCount(),
       (unsigned)fault_wheel);
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
  state = Route_ClassifyPosition(&sample->front_position, 1U);
  if ((state == ROUTE_GRAY_NO_LINE) || (state == ROUTE_GRAY_ALL_BLACK) ||
      (state == ROUTE_GRAY_I2C_ERROR))
  {
    return state;
  }
  if (!GrayCoordinate_ToVehicleError(sample->front_position.position_x100,
                                     GRAY_MOUNT_P12_LEFT_P1_RIGHT,
                                     &sample->front_error_x100))
  {
    return ROUTE_GRAY_I2C_ERROR;
  }
  return state;
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
  /* Rear sensor contributes position/heading only; branch recognition is front-only. */
  state = Route_ClassifyPosition(&sample->rear_position, 0U);
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
  /* Front P5/P8 must retain a position correction even when the rear sensor
   * sees an opposite offset. Rear gray still contributes heading correction. */
  sample->position_error_x100 = sample->front_error_x100;
  if ((follow_mode == ROUTE_FOLLOW_DUAL) &&
      (rear_state == ROUTE_GRAY_VALID))
  {
    sample->heading_error_x100 = (int16_t)(
        (int32_t)sample->front_error_x100 -
        (int32_t)sample->rear_error_x100);
  }
  else
  {
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
  int32_t step = (int32_t)(((int64_t)ROUTE_LEAVE_RAMP_STEP_RPM_X10 *
                            (int64_t)elapsed_ms) /
                           LINE_FOLLOW_PID_CONTROL_PERIOD_MS);

  if (step < 1L) step = 1L;
  if (leave_target_rpm_x10 < ROUTE_LEAVE_START_RPM_X10)
  {
    leave_target_rpm_x10 += step;
    if (leave_target_rpm_x10 > ROUTE_LEAVE_START_RPM_X10)
    {
      leave_target_rpm_x10 = ROUTE_LEAVE_START_RPM_X10;
    }
  }
  ChassisSpeedControl_SetWheelTargets(
      leave_target_rpm_x10, leave_target_rpm_x10,
      leave_target_rpm_x10, leave_target_rpm_x10);
  ChassisSpeedControl_Update(elapsed_ms);
}

static void Route_DriveUnlocatedFollow(uint32_t elapsed_ms)
{
  int32_t base = follow_smoothing.base_rpm_x10;

  if (base < ROUTE_AFTER_TURN_START_RPM_X10)
  {
    base = ROUTE_AFTER_TURN_START_RPM_X10;
  }
  ChassisSpeedControl_SetWheelTargets(base, base, base, base);
  ChassisSpeedControl_Update(elapsed_ms);
}

#if ROUTE_TURN_APPROACH_DELAY_MS > 0U
static void Route_DriveTurnApproach(uint32_t elapsed_ms)
{
  ChassisSpeedControl_SetWheelTargets(
      ROUTE_TURN_APPROACH_RPM_X10, ROUTE_TURN_APPROACH_RPM_X10,
      ROUTE_TURN_APPROACH_RPM_X10, ROUTE_TURN_APPROACH_RPM_X10);
  ChassisSpeedControl_Update(elapsed_ms);
}
#endif

static void Route_DriveRightTurn(uint32_t elapsed_ms)
{
  Chassis_SetWheelLogicalPwm(CHASSIS_WHEEL_FRONT_LEFT,
                             ROUTE_TURN_EQUAL_PWM);
  Chassis_SetWheelLogicalPwm(CHASSIS_WHEEL_FRONT_RIGHT,
                             -ROUTE_TURN_EQUAL_PWM);
  Chassis_SetWheelLogicalPwm(CHASSIS_WHEEL_REAR_LEFT,
                             ROUTE_TURN_EQUAL_PWM);
  Chassis_SetWheelLogicalPwm(CHASSIS_WHEEL_REAR_RIGHT,
                             -ROUTE_TURN_EQUAL_PWM);
  Chassis_UpdateFeedback(elapsed_ms);
}

static void Route_DriveLeftTurn(uint32_t elapsed_ms)
{
  Chassis_SetWheelLogicalPwm(CHASSIS_WHEEL_FRONT_LEFT,
                             -ROUTE_TURN_EQUAL_PWM);
  Chassis_SetWheelLogicalPwm(CHASSIS_WHEEL_FRONT_RIGHT,
                             ROUTE_TURN_EQUAL_PWM);
  Chassis_SetWheelLogicalPwm(CHASSIS_WHEEL_REAR_LEFT,
                             -ROUTE_TURN_EQUAL_PWM);
  Chassis_SetWheelLogicalPwm(CHASSIS_WHEEL_REAR_RIGHT,
                             ROUTE_TURN_EQUAL_PWM);
  Chassis_UpdateFeedback(elapsed_ms);
}

/* Hardware observation confirms YAW changes continuously with planar turning.
 * Return the shortest signed YAW change in degree x100 across +/-180 deg. */
static int16_t Route_TurnAngleDeltaX100(int16_t current_angle_x100,
                                        int16_t start_angle_x100)
{
  int32_t delta = (int32_t)current_angle_x100 - (int32_t)start_angle_x100;

  while (delta > 18000L)
  {
    delta -= 36000L;
  }
  while (delta < -18000L)
  {
    delta += 36000L;
  }
  return (int16_t)delta;
}

static uint8_t Route_IsTurnAngleFresh(uint32_t now, uint8_t angle_valid)
{
  return ((angle_valid != 0U) &&
          ((uint32_t)(now - JY61P_GetLastUpdateMs()) <= ROUTE_JY61P_STALE_MS)) ?
             1U : 0U;
}

static uint8_t Route_HasReachedTurnAngle(int16_t current_angle_x100)
{
  int32_t delta = (int32_t)Route_TurnAngleDeltaX100(current_angle_x100,
                                                    turn_start_angle_x100);

  if (delta < 0L)
  {
    delta = -delta;
  }
  return (delta >= ROUTE_TURN_TARGET_ANGLE_X100) ? 1U : 0U;
}

static uint8_t Route_AllWheelsStopped(void)
{
  ChassisWheelId_t wheel;

  for (wheel = CHASSIS_WHEEL_FRONT_LEFT;
       wheel < CHASSIS_WHEEL_COUNT; wheel++)
  {
    const ChassisWheelFeedback_t *feedback =
        Chassis_GetWheelFeedback(wheel);
    int32_t rpm_x10;

    if (feedback == 0)
    {
      return 0U;
    }
    rpm_x10 = feedback->rpm_x10;
    if (rpm_x10 < 0L)
    {
      rpm_x10 = -rpm_x10;
    }
    if (rpm_x10 > ROUTE_TURN_STOP_SPEED_RPM_X10)
    {
      return 0U;
    }
  }
  return 1U;
}

static void Route_EnterFollow(uint32_t now, uint8_t keep_speed_loop,
                             int32_t seed_base_rpm_x10,
                             uint8_t dual_on_first_valid_rear)
{
  if (keep_speed_loop != 0U)
  {
    LineFollowPID_Reset(&line_follow_pid);
    follow_smoothing.base_rpm_x10 = seed_base_rpm_x10;
    follow_smoothing.correction_rpm_x10 = 0L;
  }
  else
  {
    Route_ResetControl(1U);
    follow_smoothing.base_rpm_x10 = seed_base_rpm_x10;
  }
  route_state = ROUTE_STATE_FOLLOW;
  route_state_tick = now;
  follow_mode = ROUTE_FOLLOW_FRONT_ONLY;
  rear_valid_count = (dual_on_first_valid_rear != 0U) ?
                     ROUTE_REAR_VALID_REQUIRED : 0U;
  front_was_valid = 0U;
  junction_armed = 0U;
  junction_latched = 0U;
  rearm_valid_count = 0U;
  last_front_watch_tick = now;
  UartDebug_Printf("STATE=FOLLOW JUNCTION=%u\r\n",
                   (unsigned)junction_count);
}

static void Route_EnterAfterTurnForward(uint32_t now, uint32_t elapsed_ms,
                                        const char *finish_reason,
                                        int16_t current_angle_x100)
{
  int16_t delta = Route_TurnAngleDeltaX100(current_angle_x100,
                                            turn_start_angle_x100);

  Route_ResetControl(1U);
  leave_target_rpm_x10 = ROUTE_AFTER_TURN_START_RPM_X10;
  route_state = ROUTE_STATE_AFTER_TURN_FORWARD;
  route_state_tick = now;
  follow_mode = ROUTE_FOLLOW_FRONT_ONLY;
  rear_valid_count = 0U;
  front_was_valid = 0U;
  rearm_valid_count = 0U;
  UartDebug_Printf("TURN_DONE=%s START=%d CURRENT=%d DELTA=%d\r\n",
                   finish_reason, (int)turn_start_angle_x100,
                   (int)current_angle_x100, (int)delta);
  Route_DriveStraight(elapsed_ms);
}

static void Route_EnterTurnStop(uint32_t now);

static void Route_EnterTurnApproach(uint32_t now, uint32_t elapsed_ms,
                                    RouteGrayState_t branch)
{
  /* Do not let stall time accumulated during the preceding high-speed follow
   * cancel this short approach. Reset wheel PI integral so the requested
   * 100 RPM takes effect immediately instead of carrying 200 RPM windup. */
  ChassisSpeedControl_ResetPiState();
  turn_target_state = (branch == ROUTE_GRAY_LEFT_BLACK) ?
                      ROUTE_STATE_TURN_LEFT : ROUTE_STATE_TURN_RIGHT;
  route_state = ROUTE_STATE_TURN_APPROACH;
  route_state_tick = now;
#if ROUTE_TURN_APPROACH_DELAY_MS == 0U
  (void)elapsed_ms;
  /* No blocking UART/TFT work is allowed before the brake assertion. */
  Route_EnterTurnStop(now);
#else
  /* Apply the 100 RPM approach target before queuing diagnostics. */
  Route_DriveTurnApproach(elapsed_ms);
  UartDebug_Printf("STATE=TURN_APPROACH DIR=%s DELAY_MS=%u JUNCTION=%u\r\n",
                   (turn_target_state == ROUTE_STATE_TURN_LEFT) ? "LEFT" : "RIGHT",
                   (unsigned)ROUTE_TURN_APPROACH_DELAY_MS,
                   (unsigned)junction_count);
#endif
}

static void Route_EnterTurnStop(uint32_t now)
{
  Route_BrakeControl();
  turn_stop_frame_count = JY61P_GetFrameCount();
  turn_exit_sensor_armed = 0U;
  turn_stop_stable_count = 0U;
  gyro_paused = 0U;
  route_state = ROUTE_STATE_TURN_STOP;
  route_state_tick = now;
  UartDebug_Printf("STATE=STOP_FOR_TURN DIR=%s JUNCTION=%u\r\n",
                   (turn_target_state == ROUTE_STATE_TURN_LEFT) ? "LEFT" : "RIGHT",
                   (unsigned)junction_count);
}

static void Route_EnterFrontRecovery(uint32_t now, RouteGrayState_t cause)
{
  /* A single failed I2C read or momentary loss of line must not permanently
   * end a race. Hold the short brake while the front board is retried. */
  front_resume_state = route_state;
  front_resume_tick = route_state_tick;
  front_resume_base_rpm_x10 = follow_smoothing.base_rpm_x10;
  front_recovery_valid_count = 0U;
  Route_BrakeControl();
  route_state = ROUTE_STATE_FRONT_RECOVERY;
  route_state_tick = now;
  UartDebug_Printf("FRONT_WAIT FROM=%s CAUSE=%s\r\n",
                   Route_StateName(front_resume_state), Route_GrayStateName(cause));
}

static void Route_UpdateJunctionArm(RouteGrayState_t state)
{
  if (state != ROUTE_GRAY_VALID)
  {
    /* Three valid frames must be consecutive. Keep the history that a
     * normal line was seen before a transient gray read failure. */
    rearm_valid_count = 0U;
    return;
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
}

static uint8_t Route_RunFollow(RouteGraySample_t *sample,
                               uint32_t elapsed_ms)
{
  LineFollowPID_Output_t control;
  int32_t gain_pct = ROUTE_PID_CENTER_GAIN_PCT;

  Route_UpdateFollowMode(sample);
  LineFollowPID_Calculate(&line_follow_pid, sample->position_error_x100,
                          elapsed_ms, &control);
  if (follow_mode == ROUTE_FOLLOW_DUAL)
  {
    LineFollowPID_ApplyHeadingP(&control, sample->heading_error_x100);
  }
  if ((sample->front_raw12 &
       (ROUTE_LEFT_FOUR_MASK | ROUTE_RIGHT_FOUR_MASK)) != 0U)
  {
    gain_pct = ROUTE_PID_SIDE_GAIN_PCT;
  }
  else if ((sample->front_raw12 & ROUTE_CENTER_EDGE_MASK) != 0U)
  {
    gain_pct = ROUTE_PID_CENTER_EDGE_GAIN_PCT;
  }
  /* Scale the complete gray PID correction, not the four wheel-speed PI gains. */
  control.final_correction_rpm_x10 =
      (control.final_correction_rpm_x10 * gain_pct) / 100L;
  control.left_target_rpm_x10 = control.base_rpm_x10 +
                                control.final_correction_rpm_x10;
  control.right_target_rpm_x10 = control.base_rpm_x10 -
                                 control.final_correction_rpm_x10;
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

  Route_UpdateJunctionArm(front_state);
  return 1U;
}

static uint8_t Route_IsJunctionCandidate(RouteGrayState_t state)
{
  return ((state == ROUTE_GRAY_LEFT_BLACK) ||
          (state == ROUTE_GRAY_RIGHT_BLACK) ||
          (state == ROUTE_GRAY_ALL_BLACK)) ? 1U : 0U;
}

static void Route_HandleJunction(uint32_t now, uint32_t elapsed_ms,
                                 RouteGrayState_t branch)
{
  junction_latched = 1U;
  junction_armed = 0U;
  front_was_valid = 0U;
  if (junction_count < 255U)
  {
    junction_count++;
  }
  /* Brake/state transition precedes diagnostic output. */
  Route_EnterTurnApproach(now, elapsed_ms, branch);
  UartDebug_Printf("JUNCTION_DETECTED=%u FRONT=%s\r\n",
                   (unsigned)junction_count, Route_GrayStateName(branch));
}

static void Route_Start(void)
{
  RouteGraySample_t sample;
  uint32_t now;

  Route_ResetControl(0U);
  route_active = 0U;
  route_state = ROUTE_STATE_IDLE;
  rear_valid_count = 0U;
  junction_count = 0U;
  junction_armed = 0U;
  junction_latched = 0U;
  front_was_valid = 0U;
  rearm_valid_count = 0U;
  turn_exit_sensor_armed = 0U;
  turn_stop_stable_count = 0U;
  front_recovery_valid_count = 0U;
  gyro_paused = 0U;
  route_fault_reason = 0;
  leave_target_rpm_x10 = ROUTE_LEAVE_INITIAL_RPM_X10;
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
  if (front_state != ROUTE_GRAY_ALL_BLACK)
  {
    Route_StopFault("START_REQUIRES_ALL_BLACK");
    return;
  }

  now = SystemTime_GetMs();
  route_active = 1U;
  run_start_tick = now;
  route_state_tick = now;
  turn_start_angle_x100 = 0;
  turn_stop_frame_count = 0U;
  last_control_tick = now;
  last_front_watch_tick = now;
  Route_ResetControl(1U);
  route_state = ROUTE_STATE_LEAVE_START;
  UartDebug_SendString("START_OK STATE=LEAVE\r\n");
}

void App_LineFollowExperiment_Init(void)
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
  turn_exit_sensor_armed = 0U;
  turn_stop_stable_count = 0U;
  front_recovery_valid_count = 0U;
  gyro_paused = 0U;
  route_fault_reason = 0;
  leave_target_rpm_x10 = ROUTE_LEAVE_INITIAL_RPM_X10;
  route_state = ROUTE_STATE_IDLE;
  turn_target_state = ROUTE_STATE_TURN_RIGHT;
  follow_mode = ROUTE_FOLLOW_FRONT_ONLY;
  front_state = ROUTE_GRAY_UNKNOWN;
  rear_state = ROUTE_GRAY_UNKNOWN;
  run_start_tick = SystemTime_GetMs();
  route_state_tick = run_start_tick;
  last_control_tick = run_start_tick;
  last_front_watch_tick = run_start_tick;
  turn_stop_frame_count = 0U;
  JY61P_Init();
  UartDebug_SendString("Gray12_SideTurn_LineFollow_Test\r\n");
  UartDebug_SendString("JY61P=UART5 9600 PC12_TX PD2_RX\r\n");
  UartDebug_SendString("KEY2=start on ALL_BLACK KEY1/WK_UP=stop\r\n");
  TftRoutePage_Init();
}

void App_LineFollowExperiment_Task(void)
{
  KeyEvent_t event = Key_GetEvent();
  RouteGraySample_t sample;
  uint32_t now = SystemTime_GetMs();
  uint32_t elapsed_ms;
  int16_t turn_angle_x100 = 0;
  uint8_t turn_angle_valid;
  uint8_t turn_angle_fresh;
  uint8_t turn_gray_backup_hit;

  UartDebug_Task();
  JY61P_Task();
  /* Use the axis verified on the real car: planar left/right turning changes YAW. */
  turn_angle_valid = JY61P_GetYawX100(&turn_angle_x100);

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

#if ROUTE_TURN_APPROACH_DELAY_MS > 0U
  /* Enforce the configured approach deadline from the fast main loop. Waiting
   * for the 20 ms PID schedule used to extend it by up to one control period. */
  if ((route_active != 0U) &&
      (route_state == ROUTE_STATE_TURN_APPROACH) &&
      ((uint32_t)(now - route_state_tick) >= ROUTE_TURN_APPROACH_DELAY_MS))
  {
    Route_EnterTurnStop(now);
    goto task_end;
  }
#endif

  /* Junction detection must not wait for the 20 ms PID period. At 200 RPM the
   * car advances about 1.36 cm in 20 ms, so a narrow P11+P12 or P1+P2 event
   * can otherwise fall between two control samples. This 5 ms watch only
   * reads the front board and enters active braking; PID timing is unchanged. */
  if ((route_active != 0U) &&
      (route_state == ROUTE_STATE_FOLLOW) &&
      ((uint32_t)(now - last_front_watch_tick) >=
       ROUTE_JUNCTION_WATCH_PERIOD_MS))
  {
    last_front_watch_tick = now;
    Route_ResetSample(&sample);
    front_state = Route_ReadFront(&sample);
    if ((Route_IsJunctionCandidate(front_state) != 0U) &&
        (junction_armed != 0U) &&
        (junction_latched == 0U) &&
        (front_was_valid != 0U))
    {
      elapsed_ms = (uint32_t)(now - last_control_tick);
      Route_HandleJunction(now, elapsed_ms, front_state);
      goto task_end;
    }
    /* Arm after three consecutive fast valid samples (about 15 ms). A
     * junction/invalid sample between them must restart that count. */
    Route_UpdateJunctionArm(front_state);
    if ((front_state == ROUTE_GRAY_NO_LINE) ||
        (front_state == ROUTE_GRAY_WIDE_BLACK) ||
        (front_state == ROUTE_GRAY_I2C_ERROR))
    {
      Route_EnterFrontRecovery(now, front_state);
      goto task_end;
    }
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
      if ((uint32_t)(now - route_state_tick) >= ROUTE_LEAVE_TIMEOUT_MS)
      {
        Route_StopFault("LEAVE_TIMEOUT");
        break;
      }
      Route_ResetSample(&sample);
      front_state = Route_ReadFront(&sample);
      if (front_state == ROUTE_GRAY_I2C_ERROR)
      {
        Route_EnterFrontRecovery(now, front_state);
      }
      else if (front_state == ROUTE_GRAY_VALID)
      {
        Route_EnterFollow(now, 1U, leave_target_rpm_x10, 0U);
      }
      else if ((front_state == ROUTE_GRAY_ALL_BLACK) ||
               (front_state == ROUTE_GRAY_LEFT_BLACK) ||
               (front_state == ROUTE_GRAY_RIGHT_BLACK) ||
               (front_state == ROUTE_GRAY_WIDE_BLACK))
      {
        Route_DriveStraight(elapsed_ms);
        if (ChassisSpeedControl_HasStall() != 0U)
        {
          Route_StopFault("STALL");
        }
      }
      else
      {
        Route_EnterFrontRecovery(now, front_state);
      }
      break;

    case ROUTE_STATE_FOLLOW:
      Route_ResetSample(&sample);
      front_state = Route_ReadFront(&sample);
      if ((Route_IsJunctionCandidate(front_state) != 0U) &&
          (junction_armed != 0U) &&
          (junction_latched == 0U) &&
          (front_was_valid != 0U))
      {
        Route_HandleJunction(now, elapsed_ms, front_state);
      }
      else if ((front_state == ROUTE_GRAY_VALID) ||
                (front_state == ROUTE_GRAY_LEFT_BLACK) ||
                (front_state == ROUTE_GRAY_RIGHT_BLACK))
      {
        (void)Route_RunFollow(&sample, elapsed_ms);
      }
      else if (front_state == ROUTE_GRAY_ALL_BLACK)
      {
        /* An unarmed all-black patch has no usable PID position. Pass it
         * straight instead of entering an unrecoverable stationary fault. */
        Route_DriveUnlocatedFollow(elapsed_ms);
        if (ChassisSpeedControl_HasStall() != 0U)
        {
          Route_StopFault("STALL");
        }
      }
      else
      {
        UartDebug_Printf("FRONT_REJECT RAW12=%u STATE=%s ARMED=%u\r\n",
                         (unsigned)sample.front_raw12,
                         Route_GrayStateName(front_state),
                         (unsigned)junction_armed);
        Route_EnterFrontRecovery(now, front_state);
      }
      break;

    case ROUTE_STATE_TURN_APPROACH:
#if ROUTE_TURN_APPROACH_DELAY_MS == 0U
      /* Defensive fallback: the zero-delay entry path normally changes to
       * TURN_STOP immediately, so this state should not persist. */
      Route_EnterTurnStop(now);
#else
      /* Use the complete approach interval as the T-junction confirmation
       * window. A temporary read failure does not discard the direction that
       * was already latched when the junction was first detected. */
      Route_ResetSample(&sample);
      front_state = Route_ReadFront(&sample);
      if (front_state != ROUTE_GRAY_I2C_ERROR)
      {
        Route_ConfirmTurnDirection(sample.front_raw12);
      }
      if ((uint32_t)(now - route_state_tick) >=
          ROUTE_TURN_APPROACH_DELAY_MS)
      {
        Route_EnterTurnStop(now);
      }
      else
      {
        Route_DriveTurnApproach(elapsed_ms);
        if (ChassisSpeedControl_HasStall() != 0U)
        {
          Route_StopFault("STALL");
        }
      }
#endif
      break;

    case ROUTE_STATE_TURN_STOP:
      /* Reassert short brake on every control pass and verify actual wheel
       * speed. A fixed delay alone is not proof that a 200 RPM chassis has
       * stopped, especially when battery voltage or floor grip changes. */
      Chassis_Brake();
      Chassis_UpdateFeedback(elapsed_ms);
      if (Route_AllWheelsStopped() != 0U)
      {
        if (turn_stop_stable_count < ROUTE_TURN_STOP_STABLE_SAMPLES)
        {
          turn_stop_stable_count++;
        }
      }
      else
      {
        turn_stop_stable_count = 0U;
      }

      /* While braking, use the remaining inertial movement only to retain the
       * T-junction right-priority confirmation. Rear gray remains uninvolved. */
      Route_ResetSample(&sample);
      front_state = Route_ReadFront(&sample);
      if (front_state != ROUTE_GRAY_I2C_ERROR)
      {
        Route_ConfirmTurnDirection(sample.front_raw12);
      }

      if (((uint32_t)(now - route_state_tick) >= ROUTE_TURN_STOP_MAX_MS) &&
          (turn_stop_stable_count < ROUTE_TURN_STOP_STABLE_SAMPLES))
      {
        Route_StopFault("BRAKE_NOT_STOPPED");
        break;
      }
      if (((uint32_t)(now - route_state_tick) >= ROUTE_TURN_STOP_MS) &&
          (turn_stop_stable_count >= ROUTE_TURN_STOP_STABLE_SAMPLES))
      {
        if ((Route_IsTurnAngleFresh(now, turn_angle_valid) == 0U) ||
            (JY61P_GetFrameCount() == turn_stop_frame_count))
        {
          /* Hold the active brake until an angle frame received after braking
           * is available. This prevents starting from a pre-brake YAW sample. */
          if ((uint32_t)(now - route_state_tick) >=
              (ROUTE_TURN_STOP_MS + ROUTE_TURN_GYRO_WAIT_MS))
          {
            Route_StopFault((JY61P_GetFrameCount() == turn_stop_frame_count) ?
                            "JY61P_NO_NEW_FRAME" : "JY61P_STALE");
          }
          break;
        }
        /* Keep STBY enabled and transition directly from short brake to the
         * fixed-PWM turn in this same cycle. The old disable/enable sequence
         * released the brake and then waited one more 20 ms control period. */
        turn_start_angle_x100 = turn_angle_x100;
        route_state = turn_target_state;
        route_state_tick = now;
        if (route_state == ROUTE_STATE_TURN_LEFT)
        {
          Route_DriveLeftTurn(elapsed_ms);
        }
        else
        {
          Route_DriveRightTurn(elapsed_ms);
        }
        UartDebug_Printf("STATE=%s JUNCTION=%u YAW_START=%d\r\n",
                         Route_StateName(route_state), (unsigned)junction_count,
                         (int)turn_start_angle_x100);
      }
      break;

    case ROUTE_STATE_TURN_RIGHT:
    case ROUTE_STATE_TURN_LEFT:
      /* Healthy YAW data is authoritative: P1/P12 must not end a normal turn
       * before the configured relative angle. Gray is fallback only. */
      turn_angle_fresh = Route_IsTurnAngleFresh(now, turn_angle_valid);
      turn_gray_backup_hit = 0U;
      Route_ResetSample(&sample);
      front_state = Route_ReadFront(&sample);
      if (front_state != ROUTE_GRAY_I2C_ERROR)
      {
        /* Leave the original junction first, then arm the gray fallback. */
        if ((sample.front_raw12 & ROUTE_TURN_EXIT_P1_P12_MASK) == 0U)
        {
          turn_exit_sensor_armed = 1U;
        }
        else if (turn_exit_sensor_armed != 0U)
        {
          turn_gray_backup_hit = 1U;
        }
      }

      if ((turn_angle_fresh != 0U) &&
          (Route_HasReachedTurnAngle(turn_angle_x100) != 0U))
      {
        Route_EnterAfterTurnForward(now, elapsed_ms, "YAW_TARGET",
                                    turn_angle_x100);
        break;
      }
      if (turn_angle_fresh == 0U)
      {
        if (turn_gray_backup_hit != 0U)
        {
          Route_EnterAfterTurnForward(now, elapsed_ms, "GRAY_BACKUP_STALE",
                                      turn_angle_x100);
        }
        else
        {
          if (gyro_paused == 0U)
          {
            Route_BrakeControl();
            gyro_paused = 1U;
            gyro_pause_tick = now;
            UartDebug_SendString("TURN_GYRO_WAIT\r\n");
          }
          else
          {
            Chassis_Brake();
            if ((uint32_t)(now - gyro_pause_tick) >= ROUTE_GYRO_RECOVERY_MS)
            {
              Route_StopFault("JY61P_STALE");
            }
          }
        }
        break;
      }
      if (gyro_paused != 0U)
      {
        route_state_tick += (uint32_t)(now - gyro_pause_tick);
        gyro_paused = 0U;
        UartDebug_SendString("TURN_GYRO_RECOVERED\r\n");
      }
      if ((uint32_t)(now - route_state_tick) >= ROUTE_TURN_TIMEOUT_MS)
      {
        if (turn_gray_backup_hit != 0U)
        {
          Route_EnterAfterTurnForward(now, elapsed_ms, "GRAY_BACKUP_TIMEOUT",
                                      turn_angle_x100);
        }
        else
        {
          Route_StopFault("TURN_TIMEOUT");
        }
        break;
      }
      if (route_state == ROUTE_STATE_TURN_LEFT)
      {
        Route_DriveLeftTurn(elapsed_ms);
      }
      else
      {
        Route_DriveRightTurn(elapsed_ms);
      }
      break;

    case ROUTE_STATE_AFTER_TURN_FORWARD:
      if ((uint32_t)(now - route_state_tick) >=
          ROUTE_AFTER_TURN_FORWARD_TIMEOUT_MS)
      {
        Route_StopFault("AFTER_TURN_NO_LINE");
        break;
      }
      Route_ResetSample(&sample);
      front_state = Route_ReadFront(&sample);
      if (front_state == ROUTE_GRAY_I2C_ERROR)
      {
        Route_EnterFrontRecovery(now, front_state);
      }
      else if ((front_state == ROUTE_GRAY_VALID) ||
               (front_state == ROUTE_GRAY_LEFT_BLACK) ||
               (front_state == ROUTE_GRAY_RIGHT_BLACK))
      {
        Route_EnterFollow(now, 1U, leave_target_rpm_x10, 1U);
        (void)Route_RunFollow(&sample, elapsed_ms);
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

    case ROUTE_STATE_FRONT_RECOVERY:
      Chassis_Brake();
      Route_ResetSample(&sample);
      front_state = Route_ReadFront(&sample);
      if ((front_state == ROUTE_GRAY_I2C_ERROR) ||
          ((front_resume_state != ROUTE_STATE_AFTER_TURN_FORWARD) &&
           (front_state == ROUTE_GRAY_NO_LINE)) ||
          ((front_resume_state == ROUTE_STATE_FOLLOW) &&
           (front_state == ROUTE_GRAY_WIDE_BLACK)))
      {
        front_recovery_valid_count = 0U;
      }
      else if (front_recovery_valid_count < ROUTE_FRONT_RECOVERY_VALID_SAMPLES)
      {
        front_recovery_valid_count++;
      }
      if ((front_recovery_valid_count >= ROUTE_FRONT_RECOVERY_VALID_SAMPLES) &&
          ((uint32_t)(now - route_state_tick) <= ROUTE_FRONT_RECOVERY_MS))
      {
        uint32_t recovery_ms = (uint32_t)(now - route_state_tick);

        /* Resume the original state with its timeout paused for the time
         * spent safely braking; never retain stale wheel PI integral. */
        ChassisSpeedControl_Enable();
        route_state = front_resume_state;
        route_state_tick = front_resume_tick + recovery_ms;
        follow_smoothing.base_rpm_x10 = front_resume_base_rpm_x10;
        last_control_tick = now;
        last_front_watch_tick = now;
        UartDebug_Printf("FRONT_RECOVERED STATE=%s WAIT_MS=%lu\r\n",
                         Route_StateName(route_state), (unsigned long)recovery_ms);
        if (route_state == ROUTE_STATE_FOLLOW)
        {
          if ((Route_IsJunctionCandidate(front_state) != 0U) &&
              (junction_armed != 0U) && (junction_latched == 0U) &&
              (front_was_valid != 0U))
          {
            Route_HandleJunction(now, elapsed_ms, front_state);
          }
          else if ((front_state == ROUTE_GRAY_VALID) ||
                   (front_state == ROUTE_GRAY_LEFT_BLACK) ||
                   (front_state == ROUTE_GRAY_RIGHT_BLACK))
          {
            (void)Route_RunFollow(&sample, elapsed_ms);
          }
          else
          {
            Route_DriveUnlocatedFollow(elapsed_ms);
          }
        }
        else if ((route_state == ROUTE_STATE_AFTER_TURN_FORWARD) &&
                 (front_state == ROUTE_GRAY_VALID))
        {
          Route_EnterFollow(now, 1U, leave_target_rpm_x10, 1U);
          (void)Route_RunFollow(&sample, elapsed_ms);
        }
        else if ((route_state == ROUTE_STATE_LEAVE_START) &&
                 (front_state == ROUTE_GRAY_VALID))
        {
          Route_EnterFollow(now, 1U, leave_target_rpm_x10, 0U);
          (void)Route_RunFollow(&sample, elapsed_ms);
        }
        else
        {
          Route_DriveStraight(elapsed_ms);
        }
        if (ChassisSpeedControl_HasStall() != 0U)
        {
          Route_StopFault("STALL");
        }
      }
      else if ((uint32_t)(now - route_state_tick) >= ROUTE_FRONT_RECOVERY_MS)
      {
        Route_StopFault((front_state == ROUTE_GRAY_I2C_ERROR) ?
                        "FRONT_I2C_PERSIST" :
                        (front_state == ROUTE_GRAY_WIDE_BLACK) ?
                        "FRONT_WIDE_BLACK" : "FRONT_NO_LINE");
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
  TftRoutePage_Task(Route_StateName(route_state), route_fault_reason,
                    junction_count,
                    turn_angle_valid,
                    turn_angle_valid != 0U ? turn_angle_x100 : 0);
}
