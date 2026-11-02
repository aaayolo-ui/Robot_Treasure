#include "StartupWheelLog.h"
#include "ChassisSpeedControl.h"
#include "DriveOutputSnapshot.h"
#include "MotorDriver.h"
#include "NewFrontDriveMap.h"
#include "UartDebug.h"
#include <stdio.h>

#define STARTUP_RECORDS 64U
#define STARTUP_SAMPLE_MS 50U

typedef struct
{
  uint32_t time_ms;
  int32_t target[4], measured[4];
  uint16_t pwm[4];
  uint8_t direction[4], state;
  DriveOutputSnapshot_t hardware;
} StartupRecord_t;

static StartupRecord_t records[STARTUP_RECORDS];
static uint32_t start_tick, sample_tick;
static uint16_t count, exported;
static uint8_t recording, pending, phase;
static char output_line[320];

void StartupWheelLog_Begin(uint32_t now)
{
  /* Preserve an unexported run instead of silently losing evidence. */
  if (pending != 0U) return;
  count = 0U;
  exported = 0U;
  phase = 0U;
  start_tick = now;
  sample_tick = now - STARTUP_SAMPLE_MS;
  recording = 1U;
  pending = 1U;
}

void StartupWheelLog_Task(uint32_t now, uint8_t state, uint8_t running)
{
  StartupRecord_t *r;
  uint8_t i;
  int length;
  if (pending == 0U) return;
  if (running != 0U)
  {
    if ((recording == 0U) || (count >= STARTUP_RECORDS) ||
        ((uint32_t)(now - sample_tick) < STARTUP_SAMPLE_MS)) return;
    sample_tick = now;
    r = &records[count++];
    r->time_ms = (uint32_t)(now - start_tick);
    r->state = state;
    for (i = 0U; i < 4U; i++)
    {
      const WheelSpeedPi_t *pi =
          ChassisSpeedControl_GetController((ChassisWheelId_t)i);
      const NewFrontDriveMap_t *map = NewFrontDriveMap_Get((ChassisWheelId_t)i);
      r->target[i] = WheelSpeedPi_GetTarget(pi);
      r->measured[i] = Chassis_GetWheelFeedback((ChassisWheelId_t)i)->rpm_x10;
      r->pwm[i] = Motor_GetPwm(map->motor);
      r->direction[i] = (uint8_t)Motor_GetDirection(map->motor);
    }
    DriveOutputSnapshot_Read(&r->hardware);
    return;
  }
  recording = 0U;
  /* Format/transmit only after the route has stopped. Whole-line retries
   * preserve evidence without blocking the motor-control loop. */
  if (phase == 0U)
  {
    length = snprintf(output_line, sizeof(output_line),
        "WD_BEGIN COUNT=%u ORDER=FL_B,FR_A,RL_D,RR_C UNITS=RPM_X10,PWM "
        "DIRECTION=0_STOP,1_FWD,2_REV\r\n", (unsigned)count);
  }
  else if (exported >= count)
  {
    if (UartDebug_TrySendString("WD_END\r\n") != 0U) pending = 0U;
    return;
  }
  else
  {
    r = &records[exported];
    if (phase == 1U)
      length = snprintf(output_line, sizeof(output_line),
          "WD T=%lu S=%u FL=%ld,%ld,%u,%u FR=%ld,%ld,%u,%u "
          "RL=%ld,%ld,%u,%u RR=%ld,%ld,%u,%u\r\n",
          (unsigned long)r->time_ms, (unsigned)r->state,
          (long)r->target[0], (long)r->measured[0], (unsigned)r->pwm[0], (unsigned)r->direction[0],
          (long)r->target[1], (long)r->measured[1], (unsigned)r->pwm[1], (unsigned)r->direction[1],
          (long)r->target[2], (long)r->measured[2], (unsigned)r->pwm[2], (unsigned)r->direction[2],
          (long)r->target[3], (long)r->measured[3], (unsigned)r->pwm[3], (unsigned)r->direction[3]);
    else
      length = snprintf(output_line, sizeof(output_line),
          "WD_HW T=%lu PA2_CFG=%X IN1=%u IN2=%u STBY=%u "
          "CCR3=%lu ARR=%lu CR1=%lX CCER=%lX CCMR2=%lX U2CR1=%lX MAPR=%lX APB1=%lX\r\n",
          (unsigned long)r->time_ms, (unsigned)r->hardware.pa2_config,
          (unsigned)r->hardware.d_in1, (unsigned)r->hardware.d_in2,
          (unsigned)r->hardware.stby, (unsigned long)r->hardware.tim5_ccr3,
          (unsigned long)r->hardware.tim5_arr, (unsigned long)r->hardware.tim5_cr1,
          (unsigned long)r->hardware.tim5_ccer, (unsigned long)r->hardware.tim5_ccmr2,
          (unsigned long)r->hardware.usart2_cr1, (unsigned long)r->hardware.afio_mapr,
          (unsigned long)r->hardware.apb1_enr);
  }
  if ((length < 0) || ((unsigned)length >= sizeof(output_line))) return;
  if (UartDebug_TrySendString(output_line) == 0U) return;
  if (phase == 0U) phase = 1U;
  else if (phase == 1U) phase = 2U;
  else { phase = 1U; exported++; }
}
