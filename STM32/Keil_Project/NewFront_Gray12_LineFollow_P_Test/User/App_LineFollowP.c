#include "App_LineFollowP.h"
#include "ChassisSpeedControl.h"
#include "Key.h"
#include "nchd12.h"
#include "gray_position.h"
#include "gray_coordinate.h"
#include "UartDebug.h"
#include "main.h"

#define CONTROL_PERIOD_MS 100U
#define TEST_MAX_MS 3000U
#define LOG_CAPACITY 32U
#define BASE_RPM_X10 400L
#define GRAY_KP_NUM 2L
#define GRAY_KP_DEN 100L
#define MAX_CORRECTION_RPM_X10 120L

typedef enum { STOP_NONE = 0, STOP_KEY2, STOP_KEY1, STOP_WK_UP, STOP_TIMEOUT, STOP_NO_LINE, STOP_ALL_BLACK, STOP_I2C } StopReason_t;
typedef struct {
  uint32_t time_ms; uint16_t raw12; uint8_t gray_status; int16_t error_x100;
  int32_t base_rpm_x10; int32_t correction_rpm_x10; int32_t left_target_x10; int32_t right_target_x10;
  int32_t a_rpm_x10; int32_t b_rpm_x10; int32_t c_rpm_x10; int32_t d_rpm_x10;
  uint16_t a_pwm; uint16_t b_pwm; uint16_t c_pwm; uint16_t d_pwm;
} LineLog_t;
static LineLog_t log_data[LOG_CAPACITY];
static uint8_t log_count, log_valid;
static uint32_t run_start, last_tick;
static StopReason_t stop_reason;

static const char *App_StopName(StopReason_t reason)
{
  static const char *const names[] = {"NONE","KEY2","KEY1","WK_UP","TIMEOUT","NO_LINE","ALL_BLACK","I2C_ERROR"};
  return ((uint32_t)reason < (sizeof(names) / sizeof(names[0]))) ? names[reason] : "UNKNOWN";
}
static uint16_t App_Pwm(const WheelSpeedPi_t *pi)
{ int32_t value = WheelSpeedPi_GetOutput(pi); return (uint16_t)((value < 0) ? -value : value); }
static void App_Stop(StopReason_t reason)
{ ChassisSpeedControl_Disable(); stop_reason = reason; UartDebug_Printf("LINE_FOLLOW_STOP=%s\r\n", App_StopName(reason)); }
static void App_ClearLog(void) { log_count = 0U; log_valid = 0U; stop_reason = STOP_NONE; }
static void App_Log(uint32_t now, uint16_t raw12, const GrayPosition_Result_t *position, int16_t error, int32_t correction, int32_t left, int32_t right)
{
  LineLog_t *r; const ChassisWheelFeedback_t *a,*b,*c,*d; const WheelSpeedPi_t *pa,*pb,*pc,*pd;
  if (log_count >= LOG_CAPACITY) return;
  a=Chassis_GetWheelFeedback(CHASSIS_WHEEL_FRONT_RIGHT); b=Chassis_GetWheelFeedback(CHASSIS_WHEEL_FRONT_LEFT);
  c=Chassis_GetWheelFeedback(CHASSIS_WHEEL_REAR_RIGHT); d=Chassis_GetWheelFeedback(CHASSIS_WHEEL_REAR_LEFT);
  pa=ChassisSpeedControl_GetController(CHASSIS_WHEEL_FRONT_RIGHT); pb=ChassisSpeedControl_GetController(CHASSIS_WHEEL_FRONT_LEFT);
  pc=ChassisSpeedControl_GetController(CHASSIS_WHEEL_REAR_RIGHT); pd=ChassisSpeedControl_GetController(CHASSIS_WHEEL_REAR_LEFT);
  r=&log_data[log_count++]; r->time_ms=now-run_start; r->raw12=raw12; r->gray_status=(uint8_t)position->status; r->error_x100=error;
  r->base_rpm_x10=BASE_RPM_X10; r->correction_rpm_x10=correction; r->left_target_x10=left; r->right_target_x10=right;
  r->a_rpm_x10=a->rpm_x10; r->b_rpm_x10=b->rpm_x10; r->c_rpm_x10=c->rpm_x10; r->d_rpm_x10=d->rpm_x10;
  r->a_pwm=App_Pwm(pa); r->b_pwm=App_Pwm(pb); r->c_pwm=App_Pwm(pc); r->d_pwm=App_Pwm(pd); log_valid=1U;
}
static void App_PrintLog(void)
{
  uint8_t i; int32_t max_error=0, max_left_right_diff=0; int64_t abs_error_sum=0, left_min=2147483647L,left_max=-2147483647L,right_min=2147483647L,right_max=-2147483647L;
  int64_t ar=0,br=0,cr=0,dr=0,ap=0,bp=0,cp=0,dp=0;
  if (!log_valid) { UartDebug_SendString("NO_PREVIOUS_TEST_RECORD\r\n"); return; }
  UartDebug_Printf("LAST_TEST_RECORD_BEGIN SAMPLES=%u\r\n",(unsigned int)log_count);
  for(i=0;i<log_count;i++) { LineLog_t *r=&log_data[i]; int32_t ae=(r->error_x100<0)?-r->error_x100:r->error_x100; int32_t lr=((r->b_rpm_x10+r->d_rpm_x10)-(r->a_rpm_x10+r->c_rpm_x10))/2;
    if(lr<0)lr=-lr; if(lr>max_left_right_diff)max_left_right_diff=lr;
    if(ae>max_error)max_error=ae; abs_error_sum+=ae; if(r->left_target_x10<left_min)left_min=r->left_target_x10; if(r->left_target_x10>left_max)left_max=r->left_target_x10; if(r->right_target_x10<right_min)right_min=r->right_target_x10; if(r->right_target_x10>right_max)right_max=r->right_target_x10;
    ar+=r->a_rpm_x10;br+=r->b_rpm_x10;cr+=r->c_rpm_x10;dr+=r->d_rpm_x10;ap+=r->a_pwm;bp+=r->b_pwm;cp+=r->c_pwm;dp+=r->d_pwm;
    UartDebug_Printf("T=%lu RAW12=0x%03X STATUS=%u ERR_X100=%d BASE_X10=%ld CORR_X10=%ld L_TGT_X10=%ld R_TGT_X10=%ld A=%ld/%u B=%ld/%u C=%ld/%u D=%ld/%u\r\n",(unsigned long)r->time_ms,(unsigned int)r->raw12,(unsigned int)r->gray_status,(int)r->error_x100,(long)r->base_rpm_x10,(long)r->correction_rpm_x10,(long)r->left_target_x10,(long)r->right_target_x10,(long)r->a_rpm_x10,(unsigned int)r->a_pwm,(long)r->b_rpm_x10,(unsigned int)r->b_pwm,(long)r->c_rpm_x10,(unsigned int)r->c_pwm,(long)r->d_rpm_x10,(unsigned int)r->d_pwm);
  }
  UartDebug_Printf("SUMMARY MAX_LEFT_RIGHT_DIFF_RPM_X10=%ld MAX_ABS_GRAY_ERROR_X100=%ld AVG_ABS_GRAY_ERROR_X100=%ld LEFT_TARGET_RANGE_X10=%ld..%ld RIGHT_TARGET_RANGE_X10=%ld..%ld\r\n",(long)max_left_right_diff,(long)max_error,(long)(abs_error_sum/log_count),(long)left_min,(long)left_max,(long)right_min,(long)right_max);
  UartDebug_Printf("SUMMARY AVG_RPM_X10 A=%ld B=%ld C=%ld D=%ld AVG_PWM A=%ld B=%ld C=%ld D=%ld STOP_REASON=%s\r\n",(long)(ar/log_count),(long)(br/log_count),(long)(cr/log_count),(long)(dr/log_count),(long)(ap/log_count),(long)(bp/log_count),(long)(cp/log_count),(long)(dp/log_count),App_StopName(stop_reason));
  UartDebug_SendString("LAST_TEST_RECORD_END\r\n");
}
static uint8_t App_GrayReady(void)
{ NCHD12_Init(); if(NCHD12_ScanBus()!=NCHD12_STATUS_OK)return 0U; return (NCHD12_ConfigureInputs()==NCHD12_STATUS_OK)?1U:0U; }
static void App_Start(void)
{ App_ClearLog(); if(!App_GrayReady()){App_Stop(STOP_I2C);return;} ChassisSpeedControl_Enable(); run_start=HAL_GetTick(); last_tick=run_start; UartDebug_SendString("LINE_FOLLOW_P_STARTED\r\n"); }
void App_LineFollowP_Init(void)
{ Key_Init(); ChassisSpeedControl_Init(); App_ClearLog(); run_start=HAL_GetTick(); last_tick=run_start; UartDebug_SendString("NewFront_Gray12_LineFollow_P_Test\r\nPHYSICAL_MAP B=FL A=FR D=RL C=RR\r\nBASE_RPM=40 GRAY_KP=0.02 MAX_CORRECTION_RPM=12\r\nKEY2=start/stop KEY1=stop/log WK_UP=emergency stop\r\n"); }
void App_LineFollowP_Task(void)
{
  KeyEvent_t key=Key_GetEvent(); uint32_t now=HAL_GetTick();
  if(key==KEY_EVENT_WK_UP){App_Stop(STOP_WK_UP);return;} if(key==KEY_EVENT_KEY1){if(ChassisSpeedControl_IsEnabled())App_Stop(STOP_KEY1);else App_PrintLog();return;} if(key==KEY_EVENT_KEY2){if(ChassisSpeedControl_IsEnabled())App_Stop(STOP_KEY2);else App_Start();return;}
  if(!ChassisSpeedControl_IsEnabled() || (uint32_t)(now-last_tick)<CONTROL_PERIOD_MS)return;
  last_tick=now;
  if((uint32_t)(now-run_start)>=TEST_MAX_MS){App_Stop(STOP_TIMEOUT);return;}
  { uint16_t raw16,raw12; GrayPosition_Result_t pos; int16_t error; int32_t correction,left,right;
    if(NCHD12_ReadRaw16(&raw16)!=NCHD12_STATUS_OK){App_Stop(STOP_I2C);return;} raw12=NCHD12_Extract12(raw16); GrayPosition_Calculate(raw12,&pos);
    if(pos.status==GRAY_POSITION_STATUS_NO_LINE){App_Log(now,raw12,&pos,0,0,0,0);App_Stop(STOP_NO_LINE);return;} if(pos.status==GRAY_POSITION_STATUS_ALL_BLACK){App_Log(now,raw12,&pos,0,0,0,0);App_Stop(STOP_ALL_BLACK);return;}
    if(!GrayCoordinate_ToVehicleError(pos.position_x100,GRAY_MOUNT_P12_LEFT_P1_RIGHT,&error)){App_Stop(STOP_I2C);return;}
    correction=((int32_t)error*GRAY_KP_NUM)/GRAY_KP_DEN; if(correction>MAX_CORRECTION_RPM_X10)correction=MAX_CORRECTION_RPM_X10; if(correction<-MAX_CORRECTION_RPM_X10)correction=-MAX_CORRECTION_RPM_X10;
    left=BASE_RPM_X10+correction; right=BASE_RPM_X10-correction; ChassisSpeedControl_SetWheelTargets(left,right,left,right); ChassisSpeedControl_Update(CONTROL_PERIOD_MS); App_Log(now,raw12,&pos,error,correction,left,right);
  }
}
