#include "App_ThiefStrategy.h"
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

#include "UartDebug.h"
#include "MapTimingLog.h"
#include "MapRouteNavigator.h"
#include "MapRoutePosition.h"
#include "TreasureGrayMask.h"
#include "TravelFrame.h"
#include "RestartLineSearch.h"
#include "LaserRx.h"
#include "FrontLaserGate.h"
#include "ThiefMapPlanner.h"
#include "ThiefStrategyConfig.h"
#include "ThiefLocalization.h"
#include "TreasureLaserStart.h"
#include "MapLaserMask.h"
#include "ThiefThreatModel.h"
#include "ThiefInterceptPolicy.h"
#include "TreasureWheelLog.h"
#include "TreasureMotorSnapshot.h"
#include <stdio.h>

/* RPM targets use x10 units. Keep the user's cruise/turn settings separate. */
#define ROUTE_LEAVE_START_RPM_X10          2000L
#define ROUTE_LEAVE_INITIAL_RPM_X10        2000L
#define ROUTE_LEAVE_RAMP_STEP_RPM_X10       10L
#define ROUTE_AFTER_TURN_START_RPM_X10     2000L
#define ROUTE_TURN_EQUAL_PWM                550L
#define ROUTE_TURN_APPROACH_RPM_X10       1000L
#define ROUTE_TURN_APPROACH_DELAY_MS        180U /* Controlled forward approach before braking. */
#define ROUTE_TURN_STOP_MS                  80U
#define ROUTE_TURN_STOP_MAX_MS             800U /* Fault braked if wheels do not settle. */
#define ROUTE_TURN_STOP_SPEED_RPM_X10        20L /* About 2 RPM. */
#define ROUTE_TURN_STOP_STABLE_SAMPLES        2U
#define ROUTE_TURN_GYRO_WAIT_MS             500U
#define ROUTE_TURN_TIMEOUT_MS              4000U
#define ROUTE_TURN_TARGET_ANGLE_X100       4500L /* 45.00 deg spin, then gray-controlled line acquisition. */
#define ROUTE_JY61P_STALE_MS                150U /* No fresh angle during turn: stop safely. */
#define ROUTE_JUNCTION_WATCH_PERIOD_MS        5U /* PID period is in LineFollowPID.h. */
#define MAP_ROUTE_PASS_TIMEOUT_MS         1800U
#define MAP_ROUTE_ALIGN_TIMEOUT_MS        2500U
#define ROUTE_RESTART_BLIND_MM              180L
#define ROUTE_FRONT_RECOVERY_MS             200U /* Brake and retry a transient front-gray fault. */
#define ROUTE_FRONT_RECOVERY_VALID_SAMPLES    2U
#define ROUTE_GYRO_RECOVERY_MS              600U /* Brake while waiting for fresh YAW. */
#define ROUTE_AFTER_TURN_FORWARD_TIMEOUT_MS 1200U
#define ROUTE_MAX_RUN_MS                  1200000U
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
  ROUTE_STATE_FAULT,
  ROUTE_STATE_MAP_PASS,
  ROUTE_STATE_INTERCEPT_STOP,
  ROUTE_STATE_HIDE_APPROACH
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

typedef struct {
  RouteGraySample_t sample;
  RouteGrayState_t state;
  uint8_t valid;
} RouteLeadingCache_t;

static LineFollowPID_Controller_t line_follow_pid;
static FollowSmoothing_t follow_smoothing;
static uint8_t route_active;
static uint8_t route_key_release_pending;
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
static MapRouteNavigator_t map_nav;
static int32_t map_leg_start_counts;
static uint8_t map_pass_valid_count;
static uint8_t reverse_pending;
static int32_t map_progress_offset_mm;
static uint8_t departure_turn;
static uint8_t departure_entry_mode;
static uint8_t node_centered;
static uint8_t departure_centered;
static uint8_t departure_reacquire;
static uint8_t restart_search_active;
static uint8_t restart_search_braked;
static RestartLineSearch restart_search;
static int16_t last_leading_error;
static uint8_t last_leading_mode;
static uint32_t restart_log_tick;
static uint8_t replan_after_leg;
static uint8_t retreat_active;
static uint8_t intercept_pending;
static uint8_t intercept_has_reversed;
static uint32_t intercept_reverse_tick;
static uint32_t intercept_reversals,intercept_continues;
static int32_t Route_LegProgress(void)
{
  return map_progress_offset_mm+MapRoutePosition_DistanceMm(map_leg_start_counts);
}
typedef enum {
  THIEF_STRATEGY_READY = 0,
  THIEF_STRATEGY_TO_HIDE,
  THIEF_STRATEGY_HIDING,
  THIEF_STRATEGY_ESCAPING,
  THIEF_STRATEGY_FAULT,
  THIEF_STRATEGY_SETTLING
} ThiefStrategyState_t;
static ThiefStrategyState_t thief_strategy_state;
static ThiefRoutePlan_t thief_plan;
static ThiefMapPose_t thief_pose;
static FrontLaserGate_t thief_laser[LASER_COUNT];
static uint8_t thief_trigger_direction;
static const uint8_t laser_heading_offset[LASER_COUNT] = {0U,3U,1U,2U};
static const char *const laser_name[LASER_COUNT] = {"FRONT","LEFT","RIGHT","REAR"};
static uint16_t thief_current_node;
static uint16_t thief_previous_hide_node;
static uint8_t thief_body_heading;
static uint8_t thief_escape_requested;
static uint32_t thief_last_laser_tick[LASER_COUNT];
static uint8_t thief_laser_seen[LASER_COUNT];
static uint8_t thief_treasure_passed;
static uint32_t thief_laser_pose_since;
static uint32_t thief_laser_allowed_since[LASER_COUNT];
static uint8_t thief_laser_active_mask;
static ThiefLocalization_t thief_localization;
static void Route_StopFault(const char *reason);

static void Thief_LocalYaw(ThiefLocYaw_t *yaw)
{
  uint32_t irq=__get_PRIMASK();
  /* A bounded scalar snapshot: do not pair an old angle with a new IRQ stamp. */
  __disable_irq();
  yaw->valid=JY61P_GetYawX100(&yaw->yaw_x100);
  yaw->stamp_ms=JY61P_GetLastUpdateMs();
  __set_PRIMASK(irq);
}

static const char *Thief_StrategyName(void)
{
  static const char *const names[] = {
    "READY", "TO_HIDE", "HIDING", "ESCAPING", "FAULT", "SETTLING"
  };
  return names[(uint8_t)thief_strategy_state];
}

static void Thief_UpdatePose(void)
{
  if (thief_localization.initialized && thief_strategy_state!=THIEF_STRATEGY_FAULT) {
    ThiefLocYaw_t yaw;
    Thief_LocalYaw(&yaw);
    if (!ThiefLocalization_Update(&thief_localization,
        route_active ? Route_LegProgress() : 0L,
        &yaw,SystemTime_GetMs())) {
      Route_StopFault(ThiefLocalization_ErrorText(thief_localization.error));
    }
  }
  thief_pose=thief_localization.pose;
  if(route_state==ROUTE_STATE_HIDE_APPROACH) thief_pose.valid=0U;
  if(thief_strategy_state==THIEF_STRATEGY_FAULT) thief_pose.valid=0U;
}

static void Thief_LocalizationLog(uint32_t now)
{
  static uint32_t last;
  if(route_active || thief_strategy_state==THIEF_STRATEGY_SETTLING || (uint32_t)(now-last)<500UL) return;
  last=now;
  UartDebug_Printf("LOC NODE=N%03u X_MM=%d Y_MM=%d BODY=%u TRAVEL=%u MODE=%u VALID=%u PHASE=%u YAW_ERR=%d YAW_AGE=%lu CW_SIGN=%d FIXES=%lu STATUS=%s\r\n",
    (unsigned)thief_localization.confirmed_node,(int)thief_pose.x_mm,(int)thief_pose.y_mm,
    (unsigned)thief_pose.body_heading,(unsigned)thief_pose.travel_heading,(unsigned)thief_pose.drive_mode,
    (unsigned)thief_pose.valid,(unsigned)thief_localization.phase,(int)thief_localization.heading_error_x100,
    (unsigned long)thief_localization.gyro_age_ms,(int)thief_localization.yaw_cw_sign,
    (unsigned long)thief_localization.confirmations,ThiefLocalization_ErrorText(thief_localization.error));
}

static void Thief_ResetLaserEvidence(void)
{
  uint8_t i;
  for (i=0U;i<LASER_COUNT;i++) {
    FrontLaserGate_ResetEvidence(&thief_laser[i]);
    thief_laser_seen[i]=0U;
  }
  thief_escape_requested=0U;
}
static uint8_t Thief_EnableAfterTreasure(const MapRouteStep_t *step, int32_t progress_mm)
{
  if (thief_treasure_passed || !route_active ||
      thief_strategy_state != THIEF_STRATEGY_TO_HIDE ||
      map_nav.route == NULL || map_nav.route->id != 1600U ||
      thief_localization.error != THIEF_LOC_OK ||
      !TreasureLaserStart_Crossed(step, progress_mm)) return 1U;
  Thief_ResetLaserEvidence();
  TreasureWheelLog_Event(TREASURE_WHEEL_BEFORE_LASER,SystemTime_GetMs(),(uint8_t)route_state);
  if (!LaserRx_SetEnabled(1U)) {
    Route_StopFault("LASER_TREASURE_START_FAILED");
    return 0U;
  }
  TreasureWheelLog_Event(TREASURE_WHEEL_AFTER_LASER,SystemTime_GetMs(),(uint8_t)route_state);
  thief_treasure_passed=1U;
  UartDebug_SendString("TREASURE_N005 LASER_RX=FOUR_UARTCH4 FRONT_UART2=OFF VALID_RAW_MM=1000 TRIGGER_CENTER_MM=600\r\n");
  return 1U;
}
static void Thief_UpdateLaserMask(uint32_t now)
{
  uint8_t i, changed, next=0U;
  if(thief_treasure_passed && thief_strategy_state!=THIEF_STRATEGY_FAULT &&
      (route_active || thief_strategy_state==THIEF_STRATEGY_HIDING))
    next=MapLaserMask_DetectionMask(&thief_pose);
  changed=(uint8_t)(next^thief_laser_active_mask);
  if(changed) {
    /* Clear-ray/entry inference must not span a masked direction or pose. */
    for(i=0U;i<LASER_COUNT;i++) if(changed&(1U<<i)) {
      FrontLaserGate_ResetEvidence(&thief_laser[i]);
      thief_laser_seen[i]=0U;
      thief_laser_allowed_since[i]=now;
    }
  }
  thief_laser_active_mask=next;
}
static void Thief_OnLaserFrame(LaserDirection_t direction,uint16_t distance,
                               uint8_t confidence,uint16_t firmware,uint32_t tick)
{
  ThiefMapPose_t sensor_pose;
  uint8_t i=(uint8_t)direction;
  uint8_t allow;
  if(i>=LASER_COUNT) return;
  /* Outward rays cannot provide positive threats OR negative clear-road
   * evidence. Drop them before any model/gate processing. */
  if(!MapLaserMask_FrameAllowed(thief_laser_active_mask,i,tick,
      thief_laser_allowed_since[i])) {
    FrontLaserGate_ResetEvidence(&thief_laser[i]);
    thief_laser_seen[i]=0U;
    return;
  }
  if((int32_t)(tick-thief_laser_pose_since)<0L) {
    FrontLaserGate_ResetEvidence(&thief_laser[i]);
    thief_laser_seen[i]=0U;
    return;
  }
  /* The pose snapshot is updated once after motion control, not for each
   * UART frame. Do not project a queued moving frame from an old location. */
  if(route_active && (uint32_t)(SystemTime_GetMs()-tick)>THIEF_LASER_MOVING_FRAME_MAX_MS) {
    FrontLaserGate_ResetEvidence(&thief_laser[i]);
    thief_laser_seen[i]=0U;
    return;
  }
  if(!thief_laser_seen[i] || (uint32_t)(tick-thief_last_laser_tick[i])>250U || !distance)
    FrontLaserGate_ResetEvidence(&thief_laser[i]);
  thief_last_laser_tick[i]=tick;
  thief_laser_seen[i]=1U;
  sensor_pose=thief_pose;
  sensor_pose.body_heading=(uint8_t)((thief_pose.body_heading+laser_heading_offset[i])%4U);
  allow=(uint8_t)(thief_treasure_passed && thief_pose.valid &&
      (route_active || thief_strategy_state==THIEF_STRATEGY_HIDING));
  if(route_active &&
      (thief_localization.heading_error_x100>THIEF_LASER_MOVING_YAW_TOL_X100 ||
       thief_localization.heading_error_x100< -THIEF_LASER_MOVING_YAW_TOL_X100)) allow=0U;
  if(!allow) FrontLaserGate_ResetEvidence(&thief_laser[i]);
  if(FrontLaserGate_Observe(&thief_laser[i],&sensor_pose,
      allow,
      distance,confidence,firmware,i)) {
    if(thief_strategy_state==THIEF_STRATEGY_HIDING) {
      ThiefMapPlanner_ObserveThreat(thief_pose.node_id,sensor_pose.body_heading,
          thief_laser[i].hit_x_mm,thief_laser[i].hit_y_mm,tick);
      thief_escape_requested=1U;
    } else {
      ThiefThreatModel_Record(sensor_pose.body_heading,
          thief_laser[i].hit_x_mm,thief_laser[i].hit_y_mm,tick);
      intercept_pending=1U;
    }
  }
}
static uint8_t Thief_ThreatMask(uint32_t now)
{
  uint8_t i,mask=0U;
  uint16_t nearest=0xFFFFU;
  for(i=0U;i<LASER_COUNT;i++) {
    if(!(thief_laser_active_mask&(1U<<i))) continue;
    if(!thief_laser_seen[i] || (uint32_t)(now-thief_last_laser_tick[i])>250U) {
      FrontLaserGate_ResetEvidence(&thief_laser[i]); continue;
    }
    if(thief_laser[i].status==FRONT_LASER_OPPONENT) {
      mask|=(uint8_t)(1U<<((thief_pose.body_heading+laser_heading_offset[i])%4U));
      if(thief_laser[i].distance_mm<nearest) {
        nearest=thief_laser[i].distance_mm; thief_trigger_direction=i;
      }
    }
  }
  return mask;
}
static void Thief_LaserLog(uint32_t now)
{
  static uint32_t last;
  static uint8_t i;
  uint32_t bad,dropped,errors;
  const char *status;
  uint8_t usable;
  char line[256];
  if((uint32_t)(now-last)<25U) return;
  last=now;
  status=FrontLaserGate_StatusText(thief_laser[i].status);
  usable=(uint8_t)((thief_laser_active_mask&(1U<<i))!=0U);
  if(!LaserRx_IsEnabled()) status=thief_treasure_passed ? "RX_OFF" : "WAIT_TREASURE";
  else if(!usable) status=thief_pose.valid ? "EDGE_OFF" : "POSE_OFF";
  else if(!thief_laser_seen[i] || (uint32_t)(now-thief_last_laser_tick[i])>250U) status="NO_DATA";
  LaserRx_Counters((LaserDirection_t)i,&bad,&dropped,&errors);
  (void)snprintf(line,sizeof(line),"LASER DIR=%s D=%u C=%u STATUS=%s AGE=%lu BAD=%lu DROP=%lu ERR=%lu STATE=%s FAULT=%s\r\n",
    laser_name[i],(unsigned)(usable?thief_laser[i].distance_mm:0U),
    (unsigned)(usable?thief_laser[i].confidence:0U),
    status,(unsigned long)(thief_laser_seen[i]?now-thief_last_laser_tick[i]:0U),
    (unsigned long)bad,(unsigned long)dropped,(unsigned long)errors,
    Thief_StrategyName(),route_fault_reason?route_fault_reason:"NONE");
  (void)UartDebug_TrySendString(line);
  i=(uint8_t)((i+1U)%LASER_COUNT);
}

static void Route_MapContext(void)
{
  const MapRouteStep_t *step = MapRouteNavigator_Current(&map_nav);
  if (step != NULL) MapTimingLog_SetRoute(map_nav.route->id, map_nav.step,
                                         step->from_node, step->to_node);
}

static void Route_MapPreview(void)
{
  const MapRoute_t *route = &thief_plan.route;
  const MapRouteStep_t *step = &thief_plan.steps[0];
  static const char *const headings[] = {"N", "E", "S", "W"};
  if (thief_plan.valid == 0U) {
    UartDebug_SendString("THIEF_PLAN INVALID\r\n");
    return;
  }
  UartDebug_Printf("MAP_READY R%03u START=N%03u HEADING=%s FIRST=N%03u STEPS=%u SIGNATURE=%lu\r\n",
      (unsigned)route->id, (unsigned)route->start_node, headings[route->heading],
      (unsigned)step->to_node, (unsigned)route->count, (unsigned long)map_route_signature);
}

static uint8_t Route_MapBeginLeg(uint32_t now)
{
  ThiefLocYaw_t yaw;
  Thief_LocalYaw(&yaw);
  now=SystemTime_GetMs();
  if(!ThiefLocalization_BeginStep(&thief_localization,
      MapRouteNavigator_Current(&map_nav),&yaw,now)) {
    Route_StopFault(ThiefLocalization_ErrorText(thief_localization.error));
    return 0U;
  }
  Route_MapContext();
  map_leg_start_counts = MapRoutePosition_ForwardCounts();
  map_progress_offset_mm = 0L;
  node_centered=0U;
  /* A frame from before this confirmed outgoing axis must not be projected
   * with the new heading, even if it is under the moving age limit. */
  thief_laser_pose_since=now;
  Thief_ResetLaserEvidence();
  MapTimingLog_Event(MAP_TIMING_LEG_BEGIN, junction_count, now, 0L);
  return 1U;
}

static const char *Route_StateName(RouteState_t state)
{
  static const char *const names[] = {
      "IDLE", "LEAVE", "FOLLOW", "APPROACH", "STOP", "TURN_RIGHT",
      "TURN_LEFT", "FORWARD", "FRONT_WAIT", "DONE", "FAULT", "MAP_PASS", "INTERCEPT_STOP", "HIDE_APPROACH"};

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
  /* Classify a side hit. The route table chooses direction even if both hit. */
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

/* Collect the available side branches during approach and braking. The map
 * route remains authoritative; an asymmetric first frame cannot override it. */
static void Route_ConfirmTurnDirection(uint16_t front_raw12)
{
  /* Observe available branches; the ROM route determines the direction. */
  MapRouteNavigator_Observe(&map_nav, front_raw12);
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

  TreasureWheelLog_Event(TREASURE_WHEEL_BEFORE_FAULT,SystemTime_GetMs(),(uint8_t)fault_from);
  /* Every abnormal running exit actively brakes. FOLLOW/LEAVE faults used to
   * disable STBY and coast, so a missed junction classification could slide
   * out of the track and then appear to "quit". */
  Route_BrakeControl();
  route_active = 0U;
  route_state = ROUTE_STATE_FAULT;
  thief_strategy_state = THIEF_STRATEGY_FAULT;
  thief_localization.pose.valid=0U;
  (void)LaserRx_SetEnabled(0U);
  route_fault_reason = reason;
  intercept_pending=0U;replan_after_leg=0U;retreat_active=0U;departure_turn=0U;
  now = SystemTime_GetMs();
  last_yaw_tick = JY61P_GetLastUpdateMs();
  MapTimingLog_Event(MAP_TIMING_END, junction_count, now,
                      (reason == NULL) ? 0L :
                      ((reason[0] == 'K') && (reason[1] == 'E') && (reason[2] == 'Y')) ? 1L : 2L);
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

static uint16_t Route_FilterTreasureGray(uint16_t raw12, uint8_t leading)
{
  const MapRouteStep_t *step=MapRouteNavigator_Current(&map_nav);
  /* FOLLOW also serves post-turn alignment before the incoming step is
   * advanced. Never use that old step's text side after a body rotation. */
  uint8_t approaching=(uint8_t)(!map_nav.arrived &&
      (route_state==ROUTE_STATE_FOLLOW || route_state==ROUTE_STATE_LEAVE_START ||
       route_state==ROUTE_STATE_FRONT_RECOVERY));
  uint8_t straight_pass=(uint8_t)(step!=NULL && step->action==MAP_ACTION_STRAIGHT &&
      (route_state==ROUTE_STATE_MAP_PASS ||
       (route_state==ROUTE_STATE_FRONT_RECOVERY && front_resume_state==ROUTE_STATE_MAP_PASS)));
  return TreasureGrayMask_Apply(step,(uint8_t)(route_active && (approaching || straight_pass)),
      leading,raw12);
}

static RouteGrayState_t Route_ReadFront(RouteGraySample_t *sample)
{
  uint16_t raw16;
  RouteGrayState_t state;

  if (TravelFrame_ReadLeading(&raw16) != NCHD12_STATUS_OK)
  {
    return ROUTE_GRAY_I2C_ERROR;
  }
  sample->front_raw12 = Route_FilterTreasureGray(NCHD12_Extract12(raw16),1U);
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
  if(state==ROUTE_GRAY_VALID) {
    last_leading_error=sample->front_error_x100;
    last_leading_mode=TravelFrame_IsReverse();
  }
  return state;
}

static RouteGrayState_t Route_ReadRear(RouteGraySample_t *sample)
{
  uint16_t raw16;
  RouteGrayState_t state;

  if ((TravelFrame_IsReverse() == 0U) && (rear_hw_ready == 0U))
  {
    if (NCHD12Rear_ConfigureInputs() != NCHD12_STATUS_OK)
    {
      return ROUTE_GRAY_UNAVAILABLE;
    }
    rear_hw_ready = 1U;
  }
  if (TravelFrame_ReadTrailing(&raw16) != NCHD12_STATUS_OK)
  {
    rear_hw_ready = 0U;
    return ROUTE_GRAY_I2C_ERROR;
  }
  sample->rear_raw12 = Route_FilterTreasureGray(NCHD12_Extract12(raw16),0U);
  GrayPosition_Calculate(sample->rear_raw12, &sample->rear_position);
  /* Logical trailing board contributes heading; leading board detects branches. */
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

static RouteGrayState_t Route_ReadLeadingOnce(RouteGraySample_t *sample,
                                               RouteLeadingCache_t *cache)
{
  if (cache->valid == 0U)
  {
    Route_ResetSample(&cache->sample);
    cache->state = Route_ReadFront(&cache->sample);
    cache->valid = 1U;
  }
  *sample = cache->sample;
  return cache->state;
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
  TravelFrame_SetWheelTargets(
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
  TravelFrame_SetWheelTargets(base, base, base, base);
  ChassisSpeedControl_Update(elapsed_ms);
}

#if ROUTE_TURN_APPROACH_DELAY_MS > 0U
static void Route_DriveTurnApproach(uint32_t elapsed_ms)
{
  TravelFrame_SetWheelTargets(
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
  MapTimingLog_Event(MAP_TIMING_FOLLOW, junction_count, now, 0L);
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

  MapTimingLog_Event(MAP_TIMING_SPIN_END, junction_count, now,
                      (finish_reason[0] == 'Y') ? 0L : 1L);
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
  const MapRouteStep_t *step = MapRouteNavigator_Current(&map_nav);
  (void)branch;
  /* Do not let stall time accumulated during the preceding high-speed follow
   * cancel this short approach. Reset wheel PI integral so the requested
   * 100 RPM takes effect immediately instead of carrying 200 RPM windup. */
  ChassisSpeedControl_ResetPiState();
  turn_target_state = (step->action == MAP_ACTION_LEFT) ?
                      ROUTE_STATE_TURN_LEFT : ROUTE_STATE_TURN_RIGHT;
  route_state = ROUTE_STATE_TURN_APPROACH;
  route_state_tick = now;
  MapTimingLog_Event(MAP_TIMING_APPROACH, junction_count, now,
                      (turn_target_state == ROUTE_STATE_TURN_LEFT) ? -1L : 1L);
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
  MapTimingLog_Event(MAP_TIMING_BRAKE, junction_count, now,
                      (reverse_pending != 0U) ? (int32_t)MAP_ACTION_REVERSE :
                      (turn_target_state == ROUTE_STATE_TURN_LEFT) ? -1L : 1L);
  UartDebug_Printf("STATE=STOP_FOR_TURN DIR=%s JUNCTION=%u\r\n",
                   (reverse_pending != 0U) ? "REVERSE_TRAVEL" :
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
  MapTimingLog_Event(MAP_TIMING_FRONT_WAIT, junction_count, now,
                      (int32_t)cause);
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
  uint8_t map_stable = (front_state == ROUTE_GRAY_VALID) ? 1U : 0U;
  const MapRouteStep_t *map_step = departure_turn ? &thief_localization.step :
      MapRouteNavigator_Current(&map_nav);

  Route_UpdateFollowMode(sample);
  TreasureWheelLog_GrayContext(SystemTime_GetMs(),sample->front_raw12,
      sample->rear_raw12,sample->position_error_x100,sample->heading_error_x100);
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
  TravelFrame_SetWheelTargets(control.left_target_rpm_x10,
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
  if ((map_nav.arrived || departure_turn) && (map_step != NULL) &&
      ((map_step->action == MAP_ACTION_LEFT) || (map_step->action == MAP_ACTION_RIGHT))) {
    int16_t yaw = 0;
    int32_t angle;
    uint8_t fresh = JY61P_GetYawX100(&yaw);
    angle = Route_TurnAngleDeltaX100(TravelFrame_Yaw(yaw), turn_start_angle_x100);
    if (angle < 0L) angle = -angle;
    if ((Route_IsTurnAngleFresh(SystemTime_GetMs(), fresh) == 0U) ||
        (angle < 7000L) || (angle > 11000L)) map_stable = 0U;
  }
  if((map_nav.arrived || departure_turn) && map_step!=NULL &&
      (departure_turn || map_nav.step+1U<map_nav.route->count)) {
    ThiefLocYaw_t yaw;
    Thief_LocalYaw(&yaw);
    if(!ThiefLocalization_ExitReady(&thief_localization,
        &map_nav.steps[map_nav.step+(departure_turn?0U:1U)],
        map_stable,&yaw,SystemTime_GetMs())) map_stable=0U;
    if(thief_localization.error!=THIEF_LOC_OK) {
      Route_StopFault(ThiefLocalization_ErrorText(thief_localization.error));
      return 0U;
    }
  }
  if (MapTimingLog_FollowSample(SystemTime_GetMs(), junction_count, map_stable) != 0U) {
    if(map_nav.arrived || departure_turn) {
      ThiefLocYaw_t yaw;
      Thief_LocalYaw(&yaw);
      if(!ThiefLocalization_CommitExit(&thief_localization,
          &map_nav.steps[map_nav.step+(departure_turn?0U:1U)],
          &yaw,SystemTime_GetMs()) || (!departure_turn && !MapRouteNavigator_Advance(&map_nav)) ||
          !Route_MapBeginLeg(SystemTime_GetMs())) {
        Route_StopFault("LOC_EXIT_NOT_CONFIRMED");
        return 0U;
      }
      departure_turn=0U;
    }
  }
  return 1U;
}

static uint8_t Route_IsJunctionCandidate(RouteGrayState_t state)
{
  return ((state == ROUTE_GRAY_LEFT_BLACK) ||
          (state == ROUTE_GRAY_RIGHT_BLACK) ||
          (state == ROUTE_GRAY_ALL_BLACK)) ? 1U : 0U;
}

static void Route_FinishArrivalStop(uint32_t now,uint8_t centered)
{
  const MapRouteStep_t *finished=MapRouteNavigator_Current(&map_nav);
  Route_BrakeControl();
  route_active=0U;
  route_state=ROUTE_STATE_DONE;
  node_centered=centered;
  thief_previous_hide_node=thief_current_node;
  thief_current_node=finished->to_node;
  thief_body_heading=(uint8_t)((finished->heading+2U*finished->drive_mode)%4U);
  thief_strategy_state=THIEF_STRATEGY_SETTLING;
  route_state_tick=now;last_control_tick=now;turn_stop_stable_count=0U;
  thief_escape_requested=0U;
  Thief_ResetLaserEvidence();
  ThiefMapPlanner_PoseAtNode(thief_current_node,thief_body_heading,&thief_pose);
  MapTimingLog_Event(MAP_TIMING_END,junction_count,now,3L);
  UartDebug_Printf("THIEF_STOP R%03u AT=N%03u BODY=%u CENTERED=%u\r\n",
      (unsigned)map_nav.route->id,(unsigned)thief_current_node,
      (unsigned)thief_body_heading,(unsigned)centered);
}

static void Route_HandleJunction(uint32_t now, uint32_t elapsed_ms,
                                 RouteGrayState_t branch, uint16_t raw12)
{
  MapRouteAction_t action;
  junction_latched = 1U;
  junction_armed = 0U;
  front_was_valid = 0U;
  if (junction_count < 255U)
  {
    junction_count++;
  }
  action = MapRouteNavigator_Arrive(&map_nav, raw12,
                                     Route_LegProgress());
  if (action == MAP_ACTION_ERROR) {
    Route_StopFault("MAP_NODE_DISTANCE_OR_ORDER");
    return;
  }
  {
    ThiefLocYaw_t yaw;
    Thief_LocalYaw(&yaw);
    if(!ThiefLocalization_Arrive(&thief_localization,MapRouteNavigator_Current(&map_nav),
        raw12,Route_LegProgress(),&yaw,SystemTime_GetMs())) {
      Route_StopFault(ThiefLocalization_ErrorText(thief_localization.error));
      return;
    }
  }
  /* A validated N004 arrival also proves passage if a loop crossed the marker
   * between the preceding update and this gray event. N005 is not a gray fix. */
  if(!Thief_EnableAfterTreasure(MapRouteNavigator_Current(&map_nav),
      (int32_t)MapRouteNavigator_Current(&map_nav)->length_mm)) return;
  MapTimingLog_Event(MAP_TIMING_NODE_ARRIVE, junction_count, now, (int32_t)raw12);
  MapTimingLog_Event(MAP_TIMING_JUNCTION, junction_count, now,
                      (int32_t)branch);
  if (action == MAP_ACTION_STOP) {
    const MapRouteStep_t *finished = MapRouteNavigator_Current(&map_nav);
#if ROUTE_TURN_APPROACH_DELAY_MS > 0U
    if(!replan_after_leg && ThiefIntercept_HideAxis(finished->to_node)) {
      /* Same speed, time and travel frame as the pre-turn approach. */
      route_state=ROUTE_STATE_HIDE_APPROACH;route_state_tick=now;
      last_control_tick=now;
      ChassisSpeedControl_ResetPiState();
      Route_DriveTurnApproach(elapsed_ms);
    } else
#endif
      Route_FinishArrivalStop(now,0U);
  } else if (action == MAP_ACTION_REVERSE) {
    reverse_pending = 1U;
    MapTimingLog_Event(MAP_TIMING_REVERSE_BEGIN, junction_count, now,
                       (int32_t)(TravelFrame_IsReverse() ^ 1U));
    Route_EnterTurnStop(now);
  } else if (action == MAP_ACTION_STRAIGHT) {
    route_state = ROUTE_STATE_MAP_PASS;
    route_state_tick = now;
    map_pass_valid_count = 0U;
    MapTimingLog_Event(MAP_TIMING_PASS_BEGIN, junction_count, now, 0L);
    Route_DriveUnlocatedFollow(elapsed_ms);
  } else {
    Route_EnterTurnApproach(now, elapsed_ms, branch);
  }
  UartDebug_Printf("MAP_NODE R%03u STEP=%u N%03u ACTION=%d RAW12=%03X\r\n",
      (unsigned)map_nav.route->id, (unsigned)map_nav.step,
      (unsigned)MapRouteNavigator_Current(&map_nav)->to_node, (int)action, (unsigned)raw12);
}

static void Route_Start(void)
{
  RouteGraySample_t sample;
  uint32_t now;
  uint8_t turn_delta=0U;

  restart_search_active=0U;
  departure_reacquire=(uint8_t)(thief_localization.initialized &&
      thief_localization.phase==THIEF_LOC_AT_NODE &&
      thief_localization.confirmed_node==thief_plan.route.start_node);
  departure_entry_mode=TravelFrame_IsReverse();
  departure_centered=node_centered;
  node_centered=0U;
  Route_ResetControl(0U);
  /* RX remains off before N005 and stays on in later treasure/hide transfers. */
  if (LaserRx_SetEnabled(thief_treasure_passed) == 0U)
  {
    Route_StopFault("LASER_ROUTE_RX_FAILED");
    return;
  }
  TravelFrame_SetReverse(thief_plan.route.drive_mode);
  reverse_pending = 0U;
  departure_turn=0U;retreat_active=0U;replan_after_leg=0U;intercept_pending=0U;
  map_progress_offset_mm=0L;
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
  MapRouteNavigator_StartRuntime(&map_nav, &thief_plan.route,
                                 thief_plan.steps);
  if (map_nav.fault != 0U) {
    Route_StopFault("MAP_RUNTIME_PLAN_INVALID");
    return;
  }
  if(thief_localization.initialized) {
    turn_delta=(uint8_t)((thief_plan.steps[0].heading+
        2U*thief_plan.steps[0].drive_mode+4U-thief_localization.body_heading)%4U);
    if(turn_delta==1U || turn_delta==3U) TravelFrame_SetReverse(departure_entry_mode);
  }
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
  if (rear_hw_ready == 0U) {
    Route_StopFault("REAR_I2C_REQUIRED");
    return;
  }
  front_state = Route_ReadFront(&sample);
  UartDebug_Printf("START_FRONT=%s RAW12=%u ERROR=%d\r\n",
                   Route_GrayStateName(front_state),
                   (unsigned)sample.front_raw12,
                   (int)sample.front_error_x100);
  if ((front_state != ROUTE_GRAY_VALID) && (front_state != ROUTE_GRAY_ALL_BLACK) &&
      (front_state != ROUTE_GRAY_LEFT_BLACK) && (front_state != ROUTE_GRAY_RIGHT_BLACK) &&
      !(departure_reacquire && front_state==ROUTE_GRAY_NO_LINE))
  {
    Route_StopFault("MAP_START_REQUIRES_LINE");
    return;
  }

  now = SystemTime_GetMs();
  {
    ThiefLocYaw_t yaw;
    Thief_LocalYaw(&yaw);
    if(!thief_localization.initialized && thief_strategy_state==THIEF_STRATEGY_READY) {
      now=SystemTime_GetMs();
      if(!ThiefLocalization_Start(&thief_localization,THIEF_START_NODE,
          THIEF_START_BODY_HEADING,&yaw,now)) {
        Route_StopFault(ThiefLocalization_ErrorText(thief_localization.error));
        return;
      }
    }
  }
  Route_MapContext();
  MapTimingLog_Begin(now);
  turn_delta=(uint8_t)((thief_plan.steps[0].heading+
      2U*thief_plan.steps[0].drive_mode+4U-thief_localization.body_heading)%4U);
  if(turn_delta==1U || turn_delta==3U) {
    ThiefLocYaw_t yaw;
    Thief_LocalYaw(&yaw);
    if(!ThiefLocalization_PrepareDeparture(&thief_localization,&thief_plan.steps[0],&yaw,now)) {
      Route_StopFault("ROOT_TURN_POSE");return;
    }
    departure_turn=1U;
    thief_laser_pose_since=now;
    map_leg_start_counts=MapRoutePosition_ForwardCounts();
  } else if(turn_delta!=0U || !Route_MapBeginLeg(now)) return;
  route_active = 1U;
  run_start_tick = now;
  route_state_tick = now;
  turn_start_angle_x100 = 0;
  turn_stop_frame_count = 0U;
  last_control_tick = now;
  last_front_watch_tick = now;
  Route_ResetControl(1U);
  route_state = ROUTE_STATE_LEAVE_START;
  thief_strategy_state = (thief_plan.route.id == 1600U) ?
      THIEF_STRATEGY_TO_HIDE : THIEF_STRATEGY_ESCAPING;
  Thief_ResetLaserEvidence();
  UartDebug_SendString(thief_treasure_passed ?
      "START_OK STATE=LEAVE LASER_RX=FOUR_UARTCH4 FRONT_UART2=OFF\r\n" :
      "START_OK STATE=LEAVE LASER_RX=WAIT_TREASURE\r\n");
  TreasureWheelLog_Begin(now);
  if(departure_turn) {
    turn_target_state=turn_delta==1U?ROUTE_STATE_TURN_RIGHT:ROUTE_STATE_TURN_LEFT;
#if ROUTE_TURN_APPROACH_DELAY_MS > 0U
    if(departure_centered) {
      Route_EnterTurnStop(now);
    } else {
    /* A gray-arrival stop leaves the leading sensor at the junction. Move
     * into the turn using the incoming travel direction, including reverse.
     * Switch to the new plan's drive frame only once that approach is braked. */
    route_state=ROUTE_STATE_TURN_APPROACH;route_state_tick=now;
    ChassisSpeedControl_ResetPiState();
    Route_DriveTurnApproach(LINE_FOLLOW_PID_CONTROL_PERIOD_MS);
    }
#else
    Route_EnterTurnStop(now);
#endif
  }
}

/* Called after the bounded RX batch: no path search or UART formatting occurs
 * before braking. Return to a real gray-confirmed junction before replanning. */
static void Thief_InterceptReact(uint32_t now)
{
  uint8_t mask,i;
  uint16_t clearance[4]={0U,0U,0U,0U};
  const MapRouteStep_t *s=MapRouteNavigator_Current(&map_nav);
  if(!intercept_pending || !route_active || !thief_pose.valid || !s ||
      departure_turn || map_nav.arrived || thief_localization.phase!=THIEF_LOC_ON_LEG ||
      route_state==ROUTE_STATE_INTERCEPT_STOP) return;
  /* Do not override a gray/IMU recovery brake with an obstacle maneuver. */
  if(route_state!=ROUTE_STATE_FOLLOW && route_state!=ROUTE_STATE_LEAVE_START) return;
  mask=Thief_ThreatMask(now);
  intercept_pending=0U;
  if(!mask) return;
  for(i=0U;i<LASER_COUNT;i++) if(thief_laser[i].status==FRONT_LASER_OPPONENT &&
      thief_laser_seen[i] && (uint32_t)(now-thief_last_laser_tick[i])<=250UL)
    clearance[(thief_pose.body_heading+laser_heading_offset[i])%4U]=
        FrontLaserGate_CenterDistanceMm(thief_laser[i].distance_mm,i);
  replan_after_leg=1U;
  if(ThiefIntercept_ShouldReverse(s->heading,mask,clearance,
      (uint8_t)(intercept_has_reversed &&
       (uint32_t)(now-intercept_reverse_tick)<THIEF_INTERCEPT_COOLDOWN_MS))) {
    Route_BrakeControl();
    intercept_reversals++;
    ThiefIntercept_BlockLeg(s->from_node,s->to_node,now);
    route_state=ROUTE_STATE_INTERCEPT_STOP;route_state_tick=now;
    turn_stop_stable_count=0U;last_control_tick=now;
  } else {
    intercept_continues++;
    /* Rear-only pursuit or both directions obstructed: keep motion and
     * choose another exit at the next observed junction. */
    thief_plan.steps[map_nav.step].action=MAP_ACTION_STOP;
    thief_plan.route.count=(uint16_t)(map_nav.step+1U);
    thief_localization.step.action=MAP_ACTION_STOP;
  }
}

static void Thief_InterceptStopped(uint32_t now)
{
  MapRouteStep_t old,back;
  ThiefLocYaw_t yaw;
  RouteGraySample_t sample;
  int32_t progress;
  uint32_t dt=(uint32_t)(now-last_control_tick);
  if(dt<LINE_FOLLOW_PID_CONTROL_PERIOD_MS) return;
  last_control_tick=now;
  Chassis_Brake();Chassis_UpdateFeedback(dt);
  if(Route_AllWheelsStopped()) {
    if(turn_stop_stable_count<ROUTE_TURN_STOP_STABLE_SAMPLES) turn_stop_stable_count++;
  } else turn_stop_stable_count=0U;
  if((uint32_t)(now-route_state_tick)>=ROUTE_TURN_STOP_MAX_MS &&
      turn_stop_stable_count<ROUTE_TURN_STOP_STABLE_SAMPLES) {
    Route_StopFault("INTERCEPT_BRAKE_TIMEOUT");return;
  }
  if((uint32_t)(now-route_state_tick)<ROUTE_TURN_STOP_MS ||
      turn_stop_stable_count<ROUTE_TURN_STOP_STABLE_SAMPLES) return;
  old=*MapRouteNavigator_Current(&map_nav);
  progress=Route_LegProgress(); /* Includes the measured braking displacement. */
  back=old;back.from_node=old.to_node;back.to_node=old.from_node;
  back.heading=(uint8_t)((old.heading+2U)%4U);back.drive_mode^=1U;
  back.target_kind=ThiefMapPlanner_Node(back.to_node)->kind;
  back.expected_ports=ThiefMapPlanner_Node(back.to_node)->degree;
  back.action=MAP_ACTION_STOP;
  Thief_LocalYaw(&yaw);
  if(!ThiefLocalization_ReverseLeg(&thief_localization,&back,progress,&yaw,now)) {
    Route_StopFault("INTERCEPT_REVERSE_POSE");return;
  }
  thief_plan.steps[0]=back;thief_plan.route.id=1602U;
  thief_plan.route.start_node=back.from_node;thief_plan.route.heading=back.heading;
  thief_plan.route.drive_mode=back.drive_mode;thief_plan.route.count=1U;
  thief_plan.goal_node=back.to_node;thief_plan.final_body_heading=thief_localization.body_heading;
  thief_plan.valid=1U;
  MapRouteNavigator_StartRuntime(&map_nav,&thief_plan.route,thief_plan.steps);
  if(map_nav.fault) {Route_StopFault("INTERCEPT_REVERSE_ROUTE");return;}
  restart_search_active=0U;departure_reacquire=0U;
  TravelFrame_SetReverse(back.drive_mode);
  map_leg_start_counts=MapRoutePosition_ForwardCounts();
  map_progress_offset_mm=(int32_t)back.length_mm-progress;
  thief_body_heading=thief_localization.body_heading;
  thief_laser_pose_since=now;Thief_ResetLaserEvidence();
  intercept_pending=0U;retreat_active=1U;intercept_has_reversed=1U;
  intercept_reverse_tick=now;
  junction_armed=0U;junction_latched=0U;front_was_valid=0U;rearm_valid_count=0U;
  rear_valid_count=0U;follow_mode=ROUTE_FOLLOW_FRONT_ONLY;
  route_state_tick=now;last_front_watch_tick=now;
  leave_target_rpm_x10=ROUTE_LEAVE_INITIAL_RPM_X10;
  route_state=ROUTE_STATE_LEAVE_START;
  Route_ResetSample(&sample);front_state=Route_ReadFront(&sample);
  if(front_state!=ROUTE_GRAY_VALID && !Route_IsJunctionCandidate(front_state)) {
    /* The newly leading board may miss one sample during the sensor swap.
     * Reuse the bounded gray recovery instead of latching a one-shot fault. */
    Route_EnterFrontRecovery(now,front_state);return;
  }
  Route_ResetControl(1U);
  /* The departure node may still be under the new leading sensor. Accept
   * it only near the expected endpoint and through normal gray+gyro checks. */
  if(Route_LegProgress()>=(int32_t)back.length_mm-100L && Route_IsJunctionCandidate(front_state))
    Route_HandleJunction(now,dt,front_state,sample.front_raw12);
  else Route_DriveStraight(dt);
}

void App_ThiefStrategy_Init(void)
{
  TravelFrame_SetReverse(0U);
  reverse_pending = 0U;
  map_progress_offset_mm=0L;departure_turn=0U;replan_after_leg=0U;
  retreat_active=0U;intercept_pending=0U;intercept_has_reversed=0U;
  node_centered=0U;departure_centered=0U;
  MapTimingLog_Init();
  TreasureWheelLog_Init();
  TreasureMotorSnapshot_Init();
  Key_Init();
  ChassisSpeedControl_Init();
  LineFollowPID_Reset(&line_follow_pid);
  FollowSmoothing_Reset(&follow_smoothing);
  route_active = 0U;
  rear_hw_ready = 0U;
  route_key_release_pending = 1U; /* Discard keys held during boot configuration. */
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
  ThiefLocalization_Reset(&thief_localization);
  thief_treasure_passed=0U;
  thief_laser_pose_since=SystemTime_GetMs();
  thief_laser_active_mask=0U;
  {
    uint8_t i;
    for(i=0U;i<LASER_COUNT;i++) thief_laser_allowed_since[i]=thief_laser_pose_since;
  }
  UartDebug_SendString("BOOT FW=UARTCH4_16P_RUN1 STAGE=LASER_CONFIG USART2=OFF\r\n");
  LaserRx_Init();
  Thief_ResetLaserEvidence();
  if(!LaserRx_LinkReady() || !LaserRx_SetEnabled(0U)) {
    thief_strategy_state=THIEF_STRATEGY_FAULT;
    route_fault_reason="UARTCH4_CONFIG_FAILED";
    UartDebug_Printf("Thief16M_UARTCh4_InterceptEscape_Test UARTCH4_READY=0 FAULT=%u MOTORS=OFF USART2=OFF\r\n",
        (unsigned)LaserRx_InitFault());
    return;
  }
  thief_current_node = THIEF_START_NODE;
  thief_previous_hide_node = 0U;
  thief_body_heading = THIEF_START_BODY_HEADING;
  thief_escape_requested = 0U;
  thief_strategy_state = THIEF_STRATEGY_READY;
  ThiefMapPlanner_PoseAtNode(thief_current_node, thief_body_heading,
                             &thief_pose);
  thief_pose.valid=0U; /* Do not invent a gyro/gray fix before KEY2 at N054/W. */
  ThiefMapPlanner_Init();
  if (ThiefMapPlanner_PlanInitial(&thief_plan) == 0U) {
    thief_strategy_state = THIEF_STRATEGY_FAULT;
    route_fault_reason = "INITIAL_PLAN_FAILED";
  }
  UartDebug_Printf("Thief16M_UARTCh4_InterceptEscape_Test FW=UARTCH4_16P_RUN1 ALGO=16P UARTCH4_READY=1 LINK=%s MAIN=115200 FOUR_LASER DIN2=PB15 USART2=OFF\r\n",LaserRx_LinkSource());
  {
    TreasureDIn2PinMap_t pins;
    TreasureMotorSnapshot_ReadDIn2PinMap(&pins);
    UartDebug_Printf("PIN_ISOLATION DIN2=PB15 PA10_CFG=%u PB15_CFG=%u DIN2_LEVEL=%u STBY=%u\r\n",
        (unsigned)pins.pa10_config, (unsigned)pins.pb15_config,
        (unsigned)pins.din2_level, (unsigned)pins.stby);
  }
  UartDebug_SendString("UARTCH4: startup link verified; KEY2 starts treasure+hide; KEY1 stops\r\n");
  UartDebug_Printf("MAP_PARAMS BASE_RPM_X10=%ld CONTROL_MS=%u TURN_PWM=%ld SPIN_TARGET_X100=%ld WHEEL_CIRCUMFERENCE_UM=%lu\r\n",
      (long)LINE_FOLLOW_PID_BASE_RPM_X10, (unsigned)LINE_FOLLOW_PID_CONTROL_PERIOD_MS,
      (long)ROUTE_TURN_EQUAL_PWM, (long)ROUTE_TURN_TARGET_ANGLE_X100,
      (unsigned long)MAP_ROUTE_WHEEL_CIRCUMFERENCE_UM);
  UartDebug_SendString("LASER UARTCH4 MAIN_TX=PB6 MAIN_RX=PB7 CH1=FRONT CH2=LEFT CH3=RIGHT CH4=REAR 115200; LOG_TX=PB10; TFT=OFF; ENCODER_B_B=PB14\r\n");
  UartDebug_SendString("LASER VALID_RAW_MM=1000 TRIGGER_CENTER_MM=600 START=AFTER_N005 MOVING_RX=ON REPLAN=INTERCEPT_RETREAT_FIRST\r\n");
  UartDebug_SendString("HIDES N011=EW N038=EW N026=NS N035=EW N063=NS FAR=THREAT_SEPARATION EDGE=OUTWARD_OFF\r\n");
  UartDebug_SendString("TREASURE_GRAY=MASK1 N004_N006=NORTH_OUTER_TWO TRANSLATION_ONLY\r\n");
  UartDebug_SendString("THIEF: N054 HEAD=W; KEY2=start treasure+hide; laser map gate triggers escape\r\n");
  UartDebug_Printf("STRATEGY16E EMERGENCY=%u FAST_NEAR=%u HIDE_SCORE=%u TURN_REACH=%u BRANCH_HIDES=%u INITIAL_HIDE=%u\r\n",
      (unsigned)THIEF_EMERGENCY_ESCAPE,(unsigned)THIEF_FAST_NEAR_CONFIRM,
      (unsigned)THIEF_REACTION_HIDE_SCORE,(unsigned)THIEF_TURN_REACH_MODEL,
      (unsigned)THIEF_BRANCH_JUNCTION_HIDES,(unsigned)THIEF_INITIAL_HIDE_NODE);
  Route_MapPreview();
  {
    const ThiefDecision_t *d=ThiefMapPlanner_Decision();
    UartDebug_Printf("PLAN_V3 GOAL=N%03u T_MS=%lu MARGIN_MS=%ld STATES=%u THREATS=%u DEGRADED=%u REUSED=%u RAM=%lu\r\n",
      (unsigned)d->goal_node,(unsigned long)d->travel_ms,(long)d->worst_margin_ms,
      (unsigned)d->states_expanded,(unsigned)d->threat_count,(unsigned)d->degraded,(unsigned)d->reused_hide,
      (unsigned long)ThiefMapPlanner_WorkBytes());
    UartDebug_Printf("HIDE_V4 NODE=N%03u BODY=%u EXITS=%u AXIS_BRANCH_MS=%u\r\n",
      (unsigned)d->goal_node,(unsigned)d->parking_heading,
      (unsigned)d->escape_exits,(unsigned)d->exit_branch_ms);
  }

}

static void StartupStatus_Task(uint32_t now)
{
  static uint32_t last;
  static char line[256];
  TreasureEncoderMap_t s;
  int length;
  /* Formatting is only allowed while parked in READY/FAULT, never in motion
   * or during an active hide/escape response. Fault reasons must repeat so a
   * late Bluetooth connection cannot miss the one-shot startup failure. */
  if(route_active || (thief_strategy_state!=THIEF_STRATEGY_READY &&
      thief_strategy_state!=THIEF_STRATEGY_FAULT) || (uint32_t)(now-last)<1000U) return;
  TreasureMotorSnapshot_ReadEncoderMap(&s);
  length=snprintf(line,sizeof(line),
      "BOOT_STATUS FW=UARTCH4_16P_RUN1 T=%lu STATE=%s UARTCH4_READY=%u LINK=%s FAULT=%u REASON=%s KEY_LOCK=%u PLAN=%u PWM=%u,%u,%u,%u STBY=%u\r\n",
      (unsigned long)now,Thief_StrategyName(),(unsigned)LaserRx_LinkReady(),
      LaserRx_LinkSource(),(unsigned)LaserRx_InitFault(),route_fault_reason?route_fault_reason:"NONE",
      (unsigned)route_key_release_pending,(unsigned)thief_plan.valid,
      (unsigned)s.pwm_a,(unsigned)s.pwm_b,(unsigned)s.pwm_c,(unsigned)s.pwm_d,
      (unsigned)s.stby);
  if(length>0 && (unsigned)length<sizeof(line) && UartDebug_TrySendString(line)) last=now;
}

static void WheelMapLog_Task(uint32_t now)
{
  static uint32_t last;
  static char line[160];
  TreasureEncoderMap_t s;
  int length;
  if (thief_strategy_state != THIEF_STRATEGY_READY || route_active ||
      Chassis_IsEnabled() || (uint32_t)(now-last) < 250U) return;
  TreasureMotorSnapshot_ReadEncoderMap(&s);
  length=snprintf(line,sizeof(line),
      "WM T=%lu A=%ld B=%ld C=%ld D=%ld PWM=%u,%u,%u,%u STBY=%u\r\n",
      (unsigned long)now,(long)s.raw_a,(long)s.raw_b,(long)s.raw_c,(long)s.raw_d,
      (unsigned)s.pwm_a,(unsigned)s.pwm_b,(unsigned)s.pwm_c,(unsigned)s.pwm_d,
      (unsigned)s.stby);
  if (length>0 && (unsigned)length<sizeof(line) && UartDebug_TrySendString(line)) last=now;
}

void App_ThiefStrategy_Task(void)
{
  KeyEvent_t event = Key_GetEvent();
  RouteGraySample_t sample;
  RouteLeadingCache_t leading_cache;

  uint32_t now = SystemTime_GetMs();
  uint32_t elapsed_ms;
  int16_t turn_angle_x100 = 0;
  uint8_t turn_angle_valid;
  uint8_t turn_angle_fresh;
  uint8_t turn_gray_backup_hit;

  leading_cache.valid = 0U;
  TreasureMotorSnapshot_Probe(now);
  UartDebug_Task();
  MapTimingLog_Task();
  JY61P_Task();
  now = SystemTime_GetMs();
  /* Use the axis verified on the real car: planar left/right turning changes YAW. */
  turn_angle_valid = JY61P_GetYawX100(&turn_angle_x100);
  turn_angle_x100 = TravelFrame_Yaw(turn_angle_x100);

  /* Key_GetEvent reports a short press on release. Consume the stop gesture
   * through release so it cannot restart the car or select a different route. */
  if (route_key_release_pending != 0U)
  {
    if ((Key_IsPressed(KEY_ID_KEY1) == 0U) &&
        (Key_IsPressed(KEY_ID_WK_UP) == 0U) &&
        (Key_IsPressed(KEY_ID_KEY2) == 0U))
    {
      route_key_release_pending = 0U;
    }
    goto task_end;
  }

  if ((route_active != 0U) ||
      (thief_strategy_state == THIEF_STRATEGY_HIDING) ||
      (thief_strategy_state == THIEF_STRATEGY_SETTLING))
  {
    if ((Key_IsPressed(KEY_ID_KEY1) != 0U) ||
        (Key_IsPressed(KEY_ID_WK_UP) != 0U) ||
        (Key_IsPressed(KEY_ID_KEY2) != 0U) ||
        (event == KEY_EVENT_KEY2_SHORT))
    {
      route_key_release_pending = 1U;
      Route_StopFault("KEY_STOP");
      goto task_end;
    }
  }
  else if (event == KEY_EVENT_KEY2_SHORT)
  {
    if ((thief_strategy_state == THIEF_STRATEGY_READY) &&
        (thief_plan.valid != 0U)) Route_Start();
    else UartDebug_Printf("START_REJECT STATE=%s REASON=%s UARTCH4_READY=%u FAULT=%u PLAN=%u\r\n",
        Thief_StrategyName(),route_fault_reason?route_fault_reason:"NO_VALID_PLAN",
        (unsigned)LaserRx_LinkReady(),(unsigned)LaserRx_InitFault(),(unsigned)thief_plan.valid);
    goto task_end;
  }

  if (route_active || thief_strategy_state==THIEF_STRATEGY_SETTLING ||
      thief_strategy_state==THIEF_STRATEGY_HIDING) {
    ThiefLocYaw_t yaw;
    Thief_LocalYaw(&yaw);
    now=SystemTime_GetMs();
    if(!yaw.valid || (uint32_t)(now-yaw.stamp_ms)>THIEF_LOC_GYRO_STALE_MS) {
      thief_localization.error=THIEF_LOC_GYRO_STALE;
      thief_localization.gyro_age_ms=(uint32_t)(now-yaw.stamp_ms);
      Route_StopFault("LOC_GYRO_STALE");
      goto task_end;
    }
  }
  if(route_active && route_state==ROUTE_STATE_HIDE_APPROACH) {
#if ROUTE_TURN_APPROACH_DELAY_MS > 0U
    if((uint32_t)(now-route_state_tick)>=ROUTE_TURN_APPROACH_DELAY_MS) {
      Route_FinishArrivalStop(now,1U);
    } else if((uint32_t)(now-last_control_tick)>=LINE_FOLLOW_PID_CONTROL_PERIOD_MS) {
      uint32_t dt=(uint32_t)(now-last_control_tick);
      last_control_tick=now;
      Route_DriveTurnApproach(dt);
    }
#endif
    goto task_end;
  }
  if(route_active && route_state==ROUTE_STATE_INTERCEPT_STOP) {
    Thief_InterceptStopped(now);
    goto task_end;
  }
  if (thief_strategy_state == THIEF_STRATEGY_SETTLING) {
    if ((uint32_t)(now - last_control_tick) >=
        LINE_FOLLOW_PID_CONTROL_PERIOD_MS) {
      Chassis_UpdateFeedback((uint32_t)(now - last_control_tick));
      last_control_tick = now;
      if (Route_AllWheelsStopped() != 0U) {
        if (turn_stop_stable_count < ROUTE_TURN_STOP_STABLE_SAMPLES)
          turn_stop_stable_count++;
      } else turn_stop_stable_count = 0U;
      if (((uint32_t)(now - route_state_tick) >= ROUTE_TURN_STOP_MS) &&
          (turn_stop_stable_count >= ROUTE_TURN_STOP_STABLE_SAMPLES)) {
        thief_strategy_state = THIEF_STRATEGY_HIDING;
        thief_laser_pose_since=SystemTime_GetMs();
        Thief_ResetLaserEvidence();
        thief_escape_requested = replan_after_leg;
        if (!thief_treasure_passed || LaserRx_SetEnabled(thief_treasure_passed) == 0U)
          Route_StopFault("LASER_RESTART_FAILED");
      } else if ((uint32_t)(now - route_state_tick) >= ROUTE_TURN_STOP_MAX_MS) {
        Route_StopFault("HIDE_NOT_SETTLED");
      }
    }
    goto task_end;
  }

  if ((route_active == 0U) &&
      (thief_strategy_state == THIEF_STRATEGY_HIDING) &&
      (thief_escape_requested != 0U || replan_after_leg)) {
    uint16_t old_hide = thief_current_node;
    uint32_t planning_started = SystemTime_GetMs();
    uint8_t threat_mask = Thief_ThreatMask(now);
    UartDebug_Printf("INTERCEPT_REPLAN NODE=N%03u RETREAT=%u REVERSALS=%lu CONTINUES=%lu\r\n",
        (unsigned)thief_current_node,(unsigned)replan_after_leg,
        (unsigned long)intercept_reversals,(unsigned long)intercept_continues);
    thief_escape_requested = 0U;
    if (!threat_mask && !replan_after_leg) goto task_end;
    if (ThiefMapPlanner_PlanEscape(thief_current_node, thief_body_heading,
                                   threat_mask,
                                   thief_previous_hide_node,now,
                                   &thief_plan) == 0U) {
      thief_strategy_state = THIEF_STRATEGY_FAULT;
      route_fault_reason = "ESCAPE_PLAN_FAILED";
      UartDebug_Printf("ESCAPE_PLAN_FAILED AT=N%03u BODY=%u\r\n",
          (unsigned)thief_current_node, (unsigned)thief_body_heading);
      goto task_end;
    }
    thief_previous_hide_node = old_hide;
    if ((uint32_t)(SystemTime_GetMs()-planning_started)>THIEF_PLAN_BUDGET_MS) {
      /* Search is bounded and the chassis is already braked. A slow but
       * valid result must not permanently end the escape strategy. */
      UartDebug_Printf("PLAN_SLOW MS=%lu\r\n",
          (unsigned long)(SystemTime_GetMs()-planning_started));
    }
    UartDebug_Printf("LASER_ESCAPE DIR=%s MASK=%u D=%u C=%u FROM=N%03u TO=N%03u\r\n",
        laser_name[thief_trigger_direction], (unsigned)threat_mask,
        (unsigned)thief_laser[thief_trigger_direction].distance_mm, (unsigned)thief_laser[thief_trigger_direction].confidence,
        (unsigned)thief_current_node, (unsigned)thief_plan.goal_node);
    {
      const ThiefDecision_t *d=ThiefMapPlanner_Decision();
      UartDebug_Printf("PLAN_V3 GOAL=N%03u T_MS=%lu MARGIN_MS=%ld STATES=%u THREATS=%u DEGRADED=%u REUSED=%u COMPUTE_MS=%lu\r\n",
        (unsigned)d->goal_node,(unsigned long)d->travel_ms,(long)d->worst_margin_ms,
        (unsigned)d->states_expanded,(unsigned)d->threat_count,(unsigned)d->degraded,(unsigned)d->reused_hide,
        (unsigned long)(SystemTime_GetMs()-planning_started));
      UartDebug_Printf("HIDE_V4 NODE=N%03u BODY=%u EXITS=%u AXIS_BRANCH_MS=%u\r\n",
        (unsigned)d->goal_node,(unsigned)d->parking_heading,
        (unsigned)d->escape_exits,(unsigned)d->exit_branch_ms);
      if(d->planner_kind) {
        UartDebug_Printf("RISK_TIERS_V7 KIND=%u CAND=%u MODELS=%u CLEAR=%u HIT_MS=%u EXIT_MS=%ld STEPS=%lu ENTRY=%u COVER=%u BAND_MS=%u RUNWAY_MS=%u\r\n",
          (unsigned)d->planner_kind,(unsigned)d->candidates_evaluated,
          (unsigned)d->model_rollouts,(unsigned)d->forecast_clear,
          (unsigned)d->forecast_contact_ms,(long)d->terminal_escape_ms,
          (unsigned long)d->model_steps,(unsigned)d->entry_inferred,
          (unsigned)d->surviving_models,(unsigned)d->risk_band_ms,(unsigned)d->recovery_runway_ms);
      } else {
        UartDebug_Printf("TURN_WINDOW_V5 FIRST_RISK_MS=%u APPROACH_MASK=%u HORIZON_MS=%lu ENVELOPE_MM=%lu\r\n",
          (unsigned)d->first_pause_risk_ms,(unsigned)d->approach_mask,
          (unsigned long)THIEF_RISK_HORIZON_MS,(unsigned long)THIEF_CONTACT_MM);
      }
    }
    Route_Start();
    goto task_end;
  }

  if ((route_active != 0U) && !departure_turn && (map_nav.arrived == 0U) &&
      (MapRouteNavigator_TooFar(&map_nav, Route_LegProgress()) != 0U)) {
    Route_StopFault("MAP_EXPECTED_NODE_MISSED");
    goto task_end;
  }
  if ((route_active != 0U) && (map_nav.arrived || departure_turn) &&
      (route_state == ROUTE_STATE_FOLLOW) &&
      ((uint32_t)(now - route_state_tick) > MAP_ROUTE_ALIGN_TIMEOUT_MS)) {
    Route_StopFault("MAP_TURN_NOT_ALIGNED");
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
  if ((route_active != 0U) && !departure_turn &&
      (route_state == ROUTE_STATE_FOLLOW) &&
      ((uint32_t)(now - last_front_watch_tick) >=
       ROUTE_JUNCTION_WATCH_PERIOD_MS))
  {
    last_front_watch_tick = now;
    Route_ResetSample(&sample);
    front_state = Route_ReadLeadingOnce(&sample, &leading_cache);
    if(retreat_active && !map_nav.arrived &&
        Route_LegProgress()>=(int32_t)MapRouteNavigator_Current(&map_nav)->length_mm-100L &&
        Route_IsJunctionCandidate(front_state)) {
      Route_HandleJunction(SystemTime_GetMs(),(uint32_t)(now-last_control_tick),front_state,sample.front_raw12);
      goto task_end;
    }
    if ((map_nav.arrived == 0U) && (Route_IsJunctionCandidate(front_state) != 0U) &&
        (junction_armed != 0U) &&
        (junction_latched == 0U) &&
        (front_was_valid != 0U))
    {
      elapsed_ms = (uint32_t)(now - last_control_tick);
      TreasureWheelLog_ControlElapsed(elapsed_ms);
      Route_HandleJunction(SystemTime_GetMs(), elapsed_ms, front_state, sample.front_raw12);
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
  TreasureWheelLog_ControlElapsed(elapsed_ms);
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
      front_state = Route_ReadLeadingOnce(&sample, &leading_cache);
      if(retreat_active && !map_nav.arrived &&
          Route_LegProgress()>=(int32_t)MapRouteNavigator_Current(&map_nav)->length_mm-100L &&
          Route_IsJunctionCandidate(front_state)) {
        Route_HandleJunction(now,elapsed_ms,front_state,sample.front_raw12);break;
      }
      if (front_state == ROUTE_GRAY_I2C_ERROR)
      {
        Route_EnterFrontRecovery(now, front_state);
      }
      else if (departure_reacquire &&
               (restart_search_active || front_state==ROUTE_GRAY_NO_LINE))
      {
        RestartLineInput input;
        RestartLineOutput output;
        const MapRouteStep_t *step=MapRouteNavigator_Current(&map_nav);
        int32_t limit=ROUTE_RESTART_BLIND_MM;
        if(!restart_search_active) {
          int16_t bias=last_leading_error;
          if(last_leading_mode!=TravelFrame_IsReverse()) bias=(int16_t)-bias;
          RestartLineSearch_Reset(&restart_search,bias);
          restart_search_active=1U;restart_search_braked=0U;
          ChassisSpeedControl_ResetPiState();
          restart_log_tick=now-100U;
        }
        rear_state=Route_ReadRear(&sample);
        input.leading_valid=(uint8_t)(front_state==ROUTE_GRAY_VALID);
        input.trailing_valid=(uint8_t)(rear_state==ROUTE_GRAY_VALID);
        input.leading_error=sample.front_error_x100;
        input.trailing_error=sample.rear_error_x100;
        input.yaw_known=(uint8_t)(thief_localization.yaw_cw_sign!=0);
        input.yaw_cw_x100=(int16_t)(thief_localization.heading_error_x100*
                                  thief_localization.yaw_cw_sign);
        input.stopped=Route_AllWheelsStopped();
        RestartLineSearch_Update(&restart_search,&input,&output);
        if(limit>(int32_t)step->length_mm-50L) limit=(int32_t)step->length_mm-50L;
        if((uint32_t)(now-restart_log_tick)>=100U) {
          char line[180];
          restart_log_tick=now;
          (void)snprintf(line,sizeof(line),
              "RESTART_CTRL T=%lu MODE=%u PH=%u RAW=%03X,%03X E=%d,%d YAW=%d MM=%ld LR=%d,%d BRAKE=%u\r\n",
              (unsigned long)now,(unsigned)TravelFrame_IsReverse(),(unsigned)restart_search.phase,
              (unsigned)sample.front_raw12,(unsigned)sample.rear_raw12,
              (int)input.leading_error,(int)input.trailing_error,(int)input.yaw_cw_x100,
              (long)Route_LegProgress(),(int)output.left_x10,(int)output.right_x10,(unsigned)output.brake);
          (void)UartDebug_TrySendString(line);
        }
        if(output.failed || (!output.ready && Route_LegProgress()>=limit)) {
          Route_StopFault(output.failed?"RESTART_SCAN_NO_LINE":"RESTART_SEARCH_DISTANCE");
        } else if(output.brake) {
          Route_BrakeControl();Chassis_UpdateFeedback(elapsed_ms);
          restart_search_braked=1U;
        } else {
          if(restart_search_braked) {
            Route_ResetControl(1U);restart_search_braked=0U;
          }
          leave_target_rpm_x10=500L;
          TravelFrame_SetWheelTargets(output.left_x10,output.right_x10,
                                     output.left_x10,output.right_x10);
          ChassisSpeedControl_Update(elapsed_ms);
          if(ChassisSpeedControl_HasStall()) Route_StopFault("STALL");
          else if(output.ready) {
            departure_reacquire=0U;restart_search_active=0U;
            Route_EnterFollow(now,1U,500L,0U);
          }
        }
      }
      else if (front_state == ROUTE_GRAY_VALID)
      {
        departure_reacquire=0U;
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
      front_state = Route_ReadLeadingOnce(&sample, &leading_cache);
      if (!departure_turn && (map_nav.arrived == 0U) && (Route_IsJunctionCandidate(front_state) != 0U) &&
          (junction_armed != 0U) &&
          (junction_latched == 0U) &&
          (front_was_valid != 0U))
      {
        Route_HandleJunction(SystemTime_GetMs(), elapsed_ms, front_state, sample.front_raw12);
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
      front_state = Route_ReadLeadingOnce(&sample, &leading_cache);
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

      /* Keep observing the required map branch while braking. Rear gray is
       * never used to choose the turn direction. */
      Route_ResetSample(&sample);
      front_state = Route_ReadLeadingOnce(&sample, &leading_cache);
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
        if (reverse_pending != 0U) {
          int32_t seed;
          /* Braking and two measured standstill samples precede sensor/motor swap. */
          TravelFrame_SetReverse((uint8_t)(TravelFrame_IsReverse() ^ 1U));
          reverse_pending = 0U;
          leading_cache.valid = 0U;
          front_state = Route_ReadLeadingOnce(&sample, &leading_cache);
          if (front_state != ROUTE_GRAY_VALID) {
            Route_StopFault("REVERSE_START_REQUIRES_LEADING_LINE");
            break;
          }
          now = SystemTime_GetMs();
          MapTimingLog_Event(MAP_TIMING_REVERSE_READY, junction_count,
                             now, (int32_t)TravelFrame_IsReverse());
          seed = ROUTE_AFTER_TURN_START_RPM_X10;
          Route_EnterFollow(now, 0U, seed, 1U);
          last_front_watch_tick = now;
          (void)Route_RunFollow(&sample, elapsed_ms);
          break;
        }
        if ((!departure_turn && MapRouteNavigator_ConfirmTurn(&map_nav) == 0U) ||
            (departure_turn && !departure_centered && !(map_nav.seen_ports &
              (((turn_target_state==ROUTE_STATE_TURN_LEFT) ^
                 (departure_entry_mode!=thief_plan.route.drive_mode))?1U:2U)))) {
          Route_StopFault("MAP_REQUIRED_BRANCH_NOT_SEEN");
          break;
        }
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
        if(departure_turn) {
          TravelFrame_SetReverse(thief_plan.route.drive_mode);
          if(!JY61P_GetYawX100(&turn_angle_x100)) {Route_StopFault("ROOT_TURN_YAW");break;}
          turn_angle_x100=TravelFrame_Yaw(turn_angle_x100);
        }
        turn_start_angle_x100 = turn_angle_x100;
        route_state = turn_target_state;
        route_state_tick = now;
        MapTimingLog_Event(MAP_TIMING_ROTATE, junction_count, now,
                            (route_state == ROUTE_STATE_TURN_LEFT) ? -1L : 1L);
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
      front_state = Route_ReadLeadingOnce(&sample, &leading_cache);
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
            MapTimingLog_Event(MAP_TIMING_GYRO_WAIT, junction_count, now, 0L);
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
        MapTimingLog_Event(MAP_TIMING_GYRO_RECOVER, junction_count, now, 0L);
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
      front_state = Route_ReadLeadingOnce(&sample, &leading_cache);
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
      front_state = Route_ReadLeadingOnce(&sample, &leading_cache);
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
          if ((map_nav.arrived == 0U) && (Route_IsJunctionCandidate(front_state) != 0U) &&
              (junction_armed != 0U) && (junction_latched == 0U) &&
              (front_was_valid != 0U))
          {
            Route_HandleJunction(SystemTime_GetMs(), elapsed_ms, front_state, sample.front_raw12);
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

    case ROUTE_STATE_MAP_PASS:
      Route_ResetSample(&sample);
      front_state = Route_ReadLeadingOnce(&sample, &leading_cache);
      if ((front_state == ROUTE_GRAY_I2C_ERROR) || (front_state == ROUTE_GRAY_NO_LINE)) {
        map_pass_valid_count=0U;
        Route_EnterFrontRecovery(now,front_state);
        break;
      }
      if ((uint32_t)(now - route_state_tick) > MAP_ROUTE_PASS_TIMEOUT_MS) {
        Route_StopFault("MAP_STRAIGHT_PASS_TIMEOUT");
        break;
      }
      if (front_state == ROUTE_GRAY_VALID) {
        if (map_pass_valid_count < 3U) map_pass_valid_count++;
      } else map_pass_valid_count = 0U;
      if (map_pass_valid_count >= 3U) {
        ThiefLocYaw_t yaw;
        Thief_LocalYaw(&yaw);
        now=SystemTime_GetMs();
        if(!ThiefLocalization_ExitReady(&thief_localization,&map_nav.steps[map_nav.step+1U],
            1U,&yaw,now)) {
          if(thief_localization.error!=THIEF_LOC_OK) {
            Route_StopFault(ThiefLocalization_ErrorText(thief_localization.error));
            break;
          }
          map_pass_valid_count=0U;
          Route_DriveUnlocatedFollow(elapsed_ms);
          break;
        }
        MapTimingLog_Event(MAP_TIMING_LINE_STABLE, junction_count, now, 3L);
        if (!ThiefLocalization_CommitExit(&thief_localization,&map_nav.steps[map_nav.step+1U],
            &yaw,now) || MapRouteNavigator_Advance(&map_nav) == 0U) {
          Route_StopFault("MAP_PASS_ADVANCE");
          break;
        }
        if(!Route_MapBeginLeg(now)) break;
        Route_EnterFollow(now, 1U, follow_smoothing.base_rpm_x10, 1U);
        (void)Route_RunFollow(&sample, elapsed_ms);
      } else {
        Route_DriveUnlocatedFollow(elapsed_ms);
        if (ChassisSpeedControl_HasStall() != 0U) Route_StopFault("STALL");
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
  if(!route_active && (thief_strategy_state==THIEF_STRATEGY_READY ||
      thief_strategy_state==THIEF_STRATEGY_FAULT)) LaserRx_BootLogTask();
  StartupStatus_Task(SystemTime_GetMs());
  WheelMapLog_Task(SystemTime_GetMs());
  TreasureWheelLog_Task(SystemTime_GetMs(),(uint8_t)route_state,
      (uint8_t)(route_active || thief_strategy_state==THIEF_STRATEGY_HIDING ||
                 thief_strategy_state==THIEF_STRATEGY_SETTLING));
  MapTimingLog_Task();
  Thief_UpdatePose();
  if(thief_localization.phase==THIEF_LOC_ON_LEG && thief_pose.valid)
    (void)Thief_EnableAfterTreasure(MapRouteNavigator_Current(&map_nav),
        (int32_t)thief_pose.segment_progress_mm);
  /* Motion/gray work has priority. Multiplex RX processes at most 128 raw bytes here. */
  Thief_UpdateLaserMask(SystemTime_GetMs());
  if(thief_treasure_passed && thief_strategy_state!=THIEF_STRATEGY_FAULT)
    LaserRx_Poll(Thief_OnLaserFrame);
  Thief_InterceptReact(SystemTime_GetMs());
  Thief_LaserLog(SystemTime_GetMs());
  Thief_LocalizationLog(SystemTime_GetMs());
}
