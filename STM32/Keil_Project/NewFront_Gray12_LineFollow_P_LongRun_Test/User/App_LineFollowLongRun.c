#include "App_LineFollowLongRun.h"
#include "ChassisSpeedControl.h"
#include "LineFollowP.h"
#include "Key.h"
#include "nchd12.h"
#include "gray_position.h"
#include "gray_coordinate.h"
#include "UartDebug.h"
#include "SystemTime.h"

#define TEST_MAX_MS 10000U
#define LOG_CAPACITY 128U

typedef enum { STOP_NONE = 0, STOP_KEY2, STOP_KEY1, STOP_WK_UP, STOP_TIMEOUT, STOP_NO_LINE, STOP_ALL_BLACK, STOP_WIDE_BLACK, STOP_I2C } StopReason_t;
typedef enum { LINE_STATUS_NO_LINE = 0, LINE_STATUS_VALID, LINE_STATUS_ALL_BLACK, LINE_STATUS_WIDE_BLACK } LineStatus_t;
typedef struct {
  uint32_t time_ms; uint16_t raw12; uint8_t black_count; uint8_t gray_status; uint8_t line_status; int16_t error_x100;
  int32_t base_rpm_x10; int32_t correction_rpm_x10; int32_t left_target_x10; int32_t right_target_x10;
  int32_t a_rpm_x10; int32_t b_rpm_x10; int32_t c_rpm_x10; int32_t d_rpm_x10;
  uint16_t a_pwm; uint16_t b_pwm; uint16_t c_pwm; uint16_t d_pwm; uint8_t stop_reason;
} LineLog_t;
static LineLog_t log_data[LOG_CAPACITY];
static uint16_t sample_count;
static uint8_t has_previous_record;
static uint32_t run_start, last_tick;
static uint32_t run_elapsed_ms;
static StopReason_t stop_reason;

static const char *App_StopName(StopReason_t reason)
{
  static const char *const names[] = {"NONE","KEY2","KEY1","WK_UP","TIMEOUT","NO_LINE","ALL_BLACK","WIDE_BLACK","I2C_ERROR"};
  return ((uint32_t)reason < (sizeof(names) / sizeof(names[0]))) ? names[reason] : "UNKNOWN";
}
static const char *App_LineStatusName(LineStatus_t status)
{
  static const char *const names[] = {"NO_LINE","VALID","ALL_BLACK","WIDE_BLACK"};
  return ((uint32_t)status < (sizeof(names) / sizeof(names[0]))) ? names[status] : "UNKNOWN";
}
static uint8_t App_CountBlack(uint16_t raw12)
{
  uint8_t bit;
  uint8_t black_count = 0U;

  raw12 &= 0x0FFFU;
  for (bit = 0U; bit < 12U; bit++)
  {
    if ((raw12 & ((uint16_t)1U << bit)) != 0U)
    {
      black_count++;
    }
  }
  return black_count;
}
static LineStatus_t App_EvaluateSingleLine(uint8_t black_count, const GrayPosition_Result_t *position)
{
  if ((black_count == 0U) || (position->status == GRAY_POSITION_STATUS_NO_LINE))
  {
    return LINE_STATUS_NO_LINE;
  }
  if ((black_count == 12U) || (position->status == GRAY_POSITION_STATUS_ALL_BLACK))
  {
    return LINE_STATUS_ALL_BLACK;
  }
  if (black_count > 4U)
  {
    return LINE_STATUS_WIDE_BLACK;
  }
  return LINE_STATUS_VALID;
}
static StopReason_t App_LineStatusStopReason(LineStatus_t status)
{
  if (status == LINE_STATUS_NO_LINE) return STOP_NO_LINE;
  if (status == LINE_STATUS_ALL_BLACK) return STOP_ALL_BLACK;
  if (status == LINE_STATUS_WIDE_BLACK) return STOP_WIDE_BLACK;
  return STOP_NONE;
}
static uint16_t App_Pwm(const WheelSpeedPi_t *pi)
{ int32_t value = WheelSpeedPi_GetOutput(pi); return (uint16_t)((value < 0) ? -value : value); }
static void App_Stop(StopReason_t reason)
{
  uint8_t test_was_running = ChassisSpeedControl_IsEnabled();

  ChassisSpeedControl_Disable();

  if (test_was_running != 0U)
  {
    stop_reason = reason;
    run_elapsed_ms = SystemTime_GetMs() - run_start;

    if (sample_count != 0U)
    {
      log_data[sample_count - 1U].stop_reason = (uint8_t)reason;
      has_previous_record = 1U;
    }
  }

  UartDebug_Printf("LINE_FOLLOW_STOP=%s\r\n", App_StopName(reason));
  UartDebug_Printf("LOG_RUN_END SAMPLES=%u VALID=%u REASON=%s\r\n",
                   (unsigned int)sample_count,
                   (unsigned int)has_previous_record,
                   App_StopName(reason));
}
static void App_ResetLogForNewRun(void)
{
  sample_count = 0U;
  has_previous_record = 0U;
  stop_reason = STOP_NONE;
  run_elapsed_ms = 0U;
}
static void App_Log(uint32_t now, uint16_t raw12, uint8_t black_count, LineStatus_t line_status, const GrayPosition_Result_t *position, int16_t error, int32_t correction, int32_t left, int32_t right)
{
  LineLog_t *r; const ChassisWheelFeedback_t *a,*b,*c,*d; const WheelSpeedPi_t *pa,*pb,*pc,*pd;
  if (sample_count >= LOG_CAPACITY) return;
  a=Chassis_GetWheelFeedback(CHASSIS_WHEEL_FRONT_RIGHT); b=Chassis_GetWheelFeedback(CHASSIS_WHEEL_FRONT_LEFT);
  c=Chassis_GetWheelFeedback(CHASSIS_WHEEL_REAR_RIGHT); d=Chassis_GetWheelFeedback(CHASSIS_WHEEL_REAR_LEFT);
  pa=ChassisSpeedControl_GetController(CHASSIS_WHEEL_FRONT_RIGHT); pb=ChassisSpeedControl_GetController(CHASSIS_WHEEL_FRONT_LEFT);
  pc=ChassisSpeedControl_GetController(CHASSIS_WHEEL_REAR_RIGHT); pd=ChassisSpeedControl_GetController(CHASSIS_WHEEL_REAR_LEFT);
  r=&log_data[sample_count++]; r->time_ms=now-run_start; r->raw12=raw12; r->black_count=black_count; r->gray_status=(uint8_t)position->status; r->line_status=(uint8_t)line_status; r->error_x100=error;
  r->base_rpm_x10=LINE_FOLLOW_P_BASE_RPM_X10; r->correction_rpm_x10=correction; r->left_target_x10=left; r->right_target_x10=right;
  r->a_rpm_x10=a->rpm_x10; r->b_rpm_x10=b->rpm_x10; r->c_rpm_x10=c->rpm_x10; r->d_rpm_x10=d->rpm_x10;
  r->a_pwm=App_Pwm(pa); r->b_pwm=App_Pwm(pb); r->c_pwm=App_Pwm(pc); r->d_pwm=App_Pwm(pd); r->stop_reason=(uint8_t)STOP_NONE;
}
static void App_PrintLog(void)
{
  uint8_t i; int32_t max_error=0, max_left_right_diff=0; int64_t abs_error_sum=0, left_min=2147483647L,left_max=-2147483647L,right_min=2147483647L,right_max=-2147483647L;
  int64_t ar=0,br=0,cr=0,dr=0,ap=0,bp=0,cp=0,dp=0;
  UartDebug_Printf("LOG_QUERY SAMPLES=%u VALID=%u\r\n",(unsigned int)sample_count,(unsigned int)has_previous_record);
  if ((has_previous_record == 0U) || (sample_count == 0U)) { UartDebug_SendString("NO_PREVIOUS_TEST_RECORD\r\n"); return; }
  UartDebug_Printf("LAST_TEST_RECORD_BEGIN SAMPLES=%u\r\n",(unsigned int)sample_count);
  for(i=0;i<sample_count;i++) { LineLog_t *r=&log_data[i]; int32_t ae=(r->error_x100<0)?-r->error_x100:r->error_x100; int32_t lr=((r->b_rpm_x10+r->d_rpm_x10)-(r->a_rpm_x10+r->c_rpm_x10))/2;
    if(lr<0)lr=-lr; if(lr>max_left_right_diff)max_left_right_diff=lr;
    if(ae>max_error)max_error=ae; abs_error_sum+=ae; if(r->left_target_x10<left_min)left_min=r->left_target_x10; if(r->left_target_x10>left_max)left_max=r->left_target_x10; if(r->right_target_x10<right_min)right_min=r->right_target_x10; if(r->right_target_x10>right_max)right_max=r->right_target_x10;
    ar+=r->a_rpm_x10;br+=r->b_rpm_x10;cr+=r->c_rpm_x10;dr+=r->d_rpm_x10;ap+=r->a_pwm;bp+=r->b_pwm;cp+=r->c_pwm;dp+=r->d_pwm;
    UartDebug_Printf("T=%lu RAW12=0x%03X BLACK_COUNT=%u GRAY_STATUS=%u LINE_STATUS=%s ERR_X100=%d CORR_X10=%ld L_TGT_X10=%ld R_TGT_X10=%ld STOP_REASON=%s\r\n",(unsigned long)r->time_ms,(unsigned int)r->raw12,(unsigned int)r->black_count,(unsigned int)r->gray_status,App_LineStatusName((LineStatus_t)r->line_status),(int)r->error_x100,(long)r->correction_rpm_x10,(long)r->left_target_x10,(long)r->right_target_x10,App_StopName((StopReason_t)r->stop_reason));
    UartDebug_Printf("A_RPM_X10=%ld A_PWM=%u B_RPM_X10=%ld B_PWM=%u C_RPM_X10=%ld C_PWM=%u D_RPM_X10=%ld D_PWM=%u\r\n",(long)r->a_rpm_x10,(unsigned int)r->a_pwm,(long)r->b_rpm_x10,(unsigned int)r->b_pwm,(long)r->c_rpm_x10,(unsigned int)r->c_pwm,(long)r->d_rpm_x10,(unsigned int)r->d_pwm);
  }
  UartDebug_Printf("SUMMARY RUN_TIME_MS=%lu VALID_SAMPLES=%u MAX_LEFT_RIGHT_DIFF_RPM_X10=%ld MAX_ABS_GRAY_ERROR_X100=%ld AVG_ABS_GRAY_ERROR_X100=%ld\r\n",(unsigned long)run_elapsed_ms,(unsigned int)sample_count,(long)max_left_right_diff,(long)max_error,(long)(abs_error_sum/sample_count));
  UartDebug_Printf("SUMMARY LEFT_TARGET_RANGE_X10=%ld..%ld RIGHT_TARGET_RANGE_X10=%ld..%ld\r\n",(long)left_min,(long)left_max,(long)right_min,(long)right_max);
  UartDebug_Printf("SUMMARY AVG_RPM_X10 A=%ld B=%ld C=%ld D=%ld AVG_PWM A=%ld B=%ld C=%ld D=%ld STOP_REASON=%s\r\n",(long)(ar/sample_count),(long)(br/sample_count),(long)(cr/sample_count),(long)(dr/sample_count),(long)(ap/sample_count),(long)(bp/sample_count),(long)(cp/sample_count),(long)(dp/sample_count),App_StopName(stop_reason));
  UartDebug_SendString("LAST_TEST_RECORD_END\r\n");
}
static uint8_t App_GrayReady(void)
{ NCHD12_Init(); if(NCHD12_ScanBus()!=NCHD12_STATUS_OK)return 0U; return (NCHD12_ConfigureInputs()==NCHD12_STATUS_OK)?1U:0U; }
static uint8_t App_StartGrayPrecheck(uint16_t *raw12, GrayPosition_Result_t *position, int16_t *error_x100, StopReason_t *reject_reason)
{
  uint16_t raw16;
  uint8_t black_count;
  LineStatus_t line_status;

  if (NCHD12_ReadRaw16(&raw16) != NCHD12_STATUS_OK)
  {
    *reject_reason = STOP_I2C;
    UartDebug_SendString("START_STATUS=I2C_ERROR\r\n");
    return 0U;
  }

  *raw12 = NCHD12_Extract12(raw16);
  GrayPosition_Calculate(*raw12, position);
  black_count = App_CountBlack(*raw12);
  line_status = App_EvaluateSingleLine(black_count, position);
  *reject_reason = App_LineStatusStopReason(line_status);

  UartDebug_Printf("START_RAW12=0x%03X\r\n", (unsigned int)*raw12);
  UartDebug_Printf("START_BLACK_COUNT=%u\r\n", (unsigned int)black_count);
  if ((line_status == LINE_STATUS_VALID) &&
      GrayCoordinate_ToVehicleError(position->position_x100, GRAY_MOUNT_P12_LEFT_P1_RIGHT, error_x100))
  {
    UartDebug_Printf("START_ERR_X100=%d\r\n", (int)*error_x100);
  }
  else
  {
    UartDebug_SendString("START_ERR_X100=INVALID\r\n");
  }
  UartDebug_Printf("START_STATUS=%s\r\n", App_LineStatusName(line_status));

  return (*reject_reason == STOP_NONE) ? 1U : 0U;
}
static void App_Start(void)
{
  uint16_t raw12;
  GrayPosition_Result_t position;
  int16_t error_x100;
  StopReason_t precheck_reason;

  if (!App_GrayReady())
  {
    UartDebug_SendString("START_STATUS=I2C_ERROR\r\n");
    UartDebug_SendString("LINE_FOLLOW_START_REJECTED=I2C_ERROR\r\n");
    return;
  }
  if (!App_StartGrayPrecheck(&raw12, &position, &error_x100, &precheck_reason))
  {
    UartDebug_Printf("LINE_FOLLOW_START_REJECTED=%s\r\n", App_StopName(precheck_reason));
    return;
  }
  App_ResetLogForNewRun();
  run_start=SystemTime_GetMs(); last_tick=run_start;
  ChassisSpeedControl_Enable();
  UartDebug_SendString("LOG_RUN_BEGIN SAMPLES=0 VALID=0\r\n");
  UartDebug_SendString("LINE_FOLLOW_P_LONGRUN_STARTED\r\n");
}
void App_LineFollowLongRun_Init(void)
{ Key_Init(); ChassisSpeedControl_Init(); run_start=SystemTime_GetMs(); last_tick=run_start; UartDebug_SendString("NewFront_Gray12_LineFollow_P_LongRun_Test\r\nPHYSICAL_MAP B=FL A=FR D=RL C=RR\r\nBASE_RPM=40 GRAY_KP=0.015 MAX_CORRECTION_RPM=12 RUN_MAX_MS=10000\r\nKEY2=start/stop KEY1=stop/log WK_UP=emergency stop\r\n"); }
void App_LineFollowLongRun_Task(void)
{
  KeyEvent_t key=Key_GetEvent(); uint32_t now=SystemTime_GetMs();
  if(key==KEY_EVENT_WK_UP){App_Stop(STOP_WK_UP);return;} if(key==KEY_EVENT_KEY1){if(ChassisSpeedControl_IsEnabled())App_Stop(STOP_KEY1);else App_PrintLog();return;} if(key==KEY_EVENT_KEY2){if(ChassisSpeedControl_IsEnabled())App_Stop(STOP_KEY2);else App_Start();return;}
  if(!ChassisSpeedControl_IsEnabled() || (uint32_t)(now-last_tick)<LINE_FOLLOW_P_CONTROL_PERIOD_MS)return;
  last_tick=now;
  { uint8_t timeout_due = ((uint32_t)(now-run_start)>=TEST_MAX_MS) ? 1U : 0U; uint16_t raw16,raw12; uint8_t black_count; GrayPosition_Result_t pos; LineStatus_t line_status; StopReason_t line_stop_reason; int16_t error; LineFollowP_Output_t line_follow;
    if(NCHD12_ReadRaw16(&raw16)!=NCHD12_STATUS_OK){App_Stop(STOP_I2C);return;} raw12=NCHD12_Extract12(raw16); GrayPosition_Calculate(raw12,&pos);
    black_count=App_CountBlack(raw12); line_status=App_EvaluateSingleLine(black_count,&pos); line_stop_reason=App_LineStatusStopReason(line_status);
    if(line_stop_reason!=STOP_NONE){App_Log(now,raw12,black_count,line_status,&pos,0,0,0,0);App_Stop(line_stop_reason);return;}
    if(!GrayCoordinate_ToVehicleError(pos.position_x100,GRAY_MOUNT_P12_LEFT_P1_RIGHT,&error)){App_Stop(STOP_I2C);return;}
    LineFollowP_Calculate(error,&line_follow);
    ChassisSpeedControl_SetWheelTargets(line_follow.left_target_rpm_x10,line_follow.right_target_rpm_x10,line_follow.left_target_rpm_x10,line_follow.right_target_rpm_x10);
    ChassisSpeedControl_Update(LINE_FOLLOW_P_CONTROL_PERIOD_MS);
    App_Log(now,raw12,black_count,line_status,&pos,error,line_follow.correction_rpm_x10,line_follow.left_target_rpm_x10,line_follow.right_target_rpm_x10);
    if(timeout_due!=0U)App_Stop(STOP_TIMEOUT);
  }
}
