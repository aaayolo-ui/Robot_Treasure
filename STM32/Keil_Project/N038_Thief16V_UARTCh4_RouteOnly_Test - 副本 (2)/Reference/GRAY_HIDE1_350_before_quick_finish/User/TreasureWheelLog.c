#include "TreasureWheelLog.h"
#include "TreasureMotorSnapshot.h"
#include "ChassisSpeedControl.h"
#include "MotorDriver.h"
#include "NewFrontDriveMap.h"
#include "UartDebug.h"
#include <stdio.h>
#include <string.h>

#define TREASURE_RECORDS 96U
#define TREASURE_PRE_RECORDS 32U
#define TREASURE_SAMPLE_MS 100U
#define TREASURE_POST_MS 6000U
#define TREASURE_POST_RECORDS 60U

typedef struct
{
  uint32_t time_ms;
  int32_t target[4], measured[4];
  uint16_t pwm[4];
  int16_t encoder_delta[4];
  TreasureMotorSnapshot_t hardware;
  uint16_t control_ms;
  uint8_t direction[4], state, event;
  uint16_t gray_front, gray_rear, gray_age_ms;
  int16_t position_error, heading_error;
} TreasureRecord_t;

typedef struct
{
  uint32_t start_tick, trigger_tick, sample_tick, task_tick;
  uint32_t max_control_ms, max_task_ms;
  uint16_t head, count, post_count, exported, control_ms;
  uint8_t pending, recording, triggered, marker_mask, stopped, phase;
} TreasureLogState_t;

static TreasureRecord_t records[TREASURE_RECORDS];
static TreasureLogState_t log_state;
static char output_line[320];
static uint32_t gray_tick;
static uint16_t gray_front, gray_rear;
static int16_t position_error, heading_error;
static uint8_t gray_valid;
/* Compile-time bound includes every module-owned writable byte. */
typedef char TreasureLogRamBound[
  ((sizeof(records) + sizeof(log_state) + sizeof(output_line) + 20U) <= 13312U) ? 1 : -1];

static void Capture(uint32_t now, uint8_t state, uint8_t event)
{
  TreasureRecord_t *r;
  uint8_t i;
  uint16_t index;
  if (log_state.count >= TREASURE_RECORDS)
  {
    if (log_state.triggered != 0U) return;
    log_state.head = (uint16_t)((log_state.head + 1U) % TREASURE_RECORDS);
    log_state.count--;
  }
  index = (uint16_t)((log_state.head + log_state.count) % TREASURE_RECORDS);
  r = &records[index];
  r->time_ms = (uint32_t)(now - log_state.start_tick);
  r->state = state;
  r->event = event;
  r->control_ms = log_state.control_ms;
  r->gray_front = gray_front; r->gray_rear = gray_rear;
  r->position_error = position_error; r->heading_error = heading_error;
  r->gray_age_ms = (!gray_valid || now-gray_tick >= 65535U) ?
      65535U : (uint16_t)(now-gray_tick);
  for (i = 0U; i < 4U; i++)
  {
    const WheelSpeedPi_t *pi = ChassisSpeedControl_GetController((ChassisWheelId_t)i);
    const ChassisWheelFeedback_t *feedback = Chassis_GetWheelFeedback((ChassisWheelId_t)i);
    const NewFrontDriveMap_t *map = NewFrontDriveMap_Get((ChassisWheelId_t)i);
    r->target[i] = WheelSpeedPi_GetTarget(pi);
    r->measured[i] = feedback->rpm_x10;
    r->encoder_delta[i] = feedback->delta_count;
    r->pwm[i] = Motor_GetPwm(map->motor);
    r->direction[i] = (uint8_t)Motor_GetDirection(map->motor);
  }
  TreasureMotorSnapshot_Read(&r->hardware);
  log_state.count++;
}

void TreasureWheelLog_Init(void)
{
  memset(&log_state, 0, sizeof(log_state));
  gray_valid = 0U;
}

void TreasureWheelLog_GrayContext(uint32_t now, uint16_t front, uint16_t rear,
                                  int16_t position, int16_t heading)
{
  gray_tick=now; gray_front=front; gray_rear=rear;
  position_error=position; heading_error=heading; gray_valid=1U;
}

void TreasureWheelLog_Begin(uint32_t now)
{
  if (log_state.pending != 0U) return;
  memset(&log_state, 0, sizeof(log_state));
  log_state.start_tick = now;
  log_state.task_tick = now;
  log_state.sample_tick = now - TREASURE_SAMPLE_MS;
  log_state.pending = 1U;
  log_state.recording = 1U;
  gray_valid = 0U;
}

void TreasureWheelLog_ControlElapsed(uint32_t elapsed_ms)
{
  if (log_state.pending == 0U || log_state.recording == 0U) return;
  log_state.control_ms = (elapsed_ms > 65535U) ? 65535U : (uint16_t)elapsed_ms;
  if (elapsed_ms > log_state.max_control_ms) log_state.max_control_ms = elapsed_ms;
}

void TreasureWheelLog_Event(uint8_t kind, uint32_t now, uint8_t state)
{
  uint8_t bit;
  if (log_state.pending == 0U || log_state.stopped != 0U ||
      kind < TREASURE_WHEEL_BEFORE_LASER || kind > TREASURE_WHEEL_BEFORE_FAULT) return;
  bit = (uint8_t)(1U << (kind - 1U));
  if ((log_state.marker_mask & bit) != 0U) return;
  /* Fault markers remain admissible after the six-second sampling window.
   * Four record slots are reserved for before/after/ fault/stop markers. */
  if (log_state.recording == 0U && kind != TREASURE_WHEEL_BEFORE_FAULT) return;
  if (log_state.triggered == 0U)
  {
    if (log_state.count > TREASURE_PRE_RECORDS)
    {
      log_state.head = (uint16_t)((log_state.head + log_state.count - TREASURE_PRE_RECORDS) % TREASURE_RECORDS);
      log_state.count = TREASURE_PRE_RECORDS;
    }
    log_state.triggered = 1U;
    log_state.trigger_tick = now;
    log_state.sample_tick = now;
  }
  Capture(now, state, kind);
  log_state.marker_mask |= bit;
  if (kind == TREASURE_WHEEL_BEFORE_FAULT) log_state.recording = 0U;
}

void TreasureWheelLog_Task(uint32_t now, uint8_t state, uint8_t running)
{
  TreasureRecord_t *r;
  TreasureMotorSnapshot_t *h;
  uint32_t interval;
  int length;
  if (log_state.pending == 0U) return;
  if (running != 0U)
  {
    if (log_state.recording == 0U) return;
    interval = (uint32_t)(now - log_state.task_tick);
    log_state.task_tick = now;
    if (interval > log_state.max_task_ms) log_state.max_task_ms = interval;
    if (log_state.triggered != 0U &&
        ((uint32_t)(now - log_state.trigger_tick) > TREASURE_POST_MS ||
         log_state.post_count >= TREASURE_POST_RECORDS))
    {
      log_state.recording = 0U;
      return;
    }
    if ((uint32_t)(now - log_state.sample_tick) < TREASURE_SAMPLE_MS) return;
    log_state.sample_tick = now;
    Capture(now, state, 0U);
    if (log_state.triggered != 0U) log_state.post_count++;
    return;
  }
  if (log_state.stopped == 0U)
  {
    Capture(now, state, TREASURE_WHEEL_AFTER_STOP);
    log_state.stopped = 1U;
    log_state.recording = 0U;
  }
  /* All formatting is below the running inhibit. Retry the identical whole
   * line on a full UART queue and advance only after successful admission. */
  if (log_state.phase == 0U)
  {
    length = snprintf(output_line, sizeof(output_line),
        "TW_BEGIN COUNT=%u PRE_MAX=32 POST_MS=6000 SAMPLE_MS=100 RAM=%lu "
        "MAX_CONTROL_MS=%lu MAX_TASK_MS=%lu ORDER=FL_B,FR_A,RL_D,RR_C "
        "EV=0_SAMPLE,1_PRE_LASER,2_POST_LASER,3_PRE_FAULT,4_STOP\r\n",
        (unsigned)log_state.count, (unsigned long)TreasureWheelLog_WorkBytes(),
        (unsigned long)log_state.max_control_ms, (unsigned long)log_state.max_task_ms);
  }
  else if (log_state.exported >= log_state.count)
  {
    if (UartDebug_TrySendString("TW_END\r\n") != 0U) log_state.pending = 0U;
    return;
  }
  else
  {
    r = &records[(log_state.head + log_state.exported) % TREASURE_RECORDS];
    h = &r->hardware;
    if (log_state.phase == 1U)
      length = snprintf(output_line, sizeof(output_line),
          "TW T=%lu S=%u EV=%u DT=%u FL=%ld,%ld,%u,%u,%d "
          "FR=%ld,%ld,%u,%u,%d RL=%ld,%ld,%u,%u,%d RR=%ld,%ld,%u,%u,%d\r\n",
          (unsigned long)r->time_ms, (unsigned)r->state, (unsigned)r->event, (unsigned)r->control_ms,
          (long)r->target[0], (long)r->measured[0], (unsigned)r->pwm[0], (unsigned)r->direction[0], (int)r->encoder_delta[0],
          (long)r->target[1], (long)r->measured[1], (unsigned)r->pwm[1], (unsigned)r->direction[1], (int)r->encoder_delta[1],
          (long)r->target[2], (long)r->measured[2], (unsigned)r->pwm[2], (unsigned)r->direction[2], (int)r->encoder_delta[2],
          (long)r->target[3], (long)r->measured[3], (unsigned)r->pwm[3], (unsigned)r->direction[3], (int)r->encoder_delta[3]);
    else if (log_state.phase == 2U)
      length = snprintf(output_line, sizeof(output_line),
          "TW_PWM T=%lu EV=%u CCR2=%u CCR3=%u ARR=%u CR1=%X CCER=%X "
          "CCMR1=%X CCMR2=%X PA1=%X PA2=%X OUT=%X\r\n",
          (unsigned long)r->time_ms, (unsigned)r->event,
          (unsigned)h->tim5_ccr2, (unsigned)h->tim5_ccr3, (unsigned)h->tim5_arr,
          (unsigned)h->tim5_cr1, (unsigned)h->tim5_ccer,
          (unsigned)h->tim5_ccmr1, (unsigned)h->tim5_ccmr2,
          (unsigned)h->pa1_config, (unsigned)h->pa2_config, (unsigned)h->outputs);
    else if (log_state.phase == 3U)
      length = snprintf(output_line, sizeof(output_line),
          "TW_ENC T=%lu EV=%u CNT=%u SMCR=%X CR1=%X CCER=%X PA15=%X PB3=%X IN=%X\r\n",
          (unsigned long)r->time_ms, (unsigned)r->event,
          (unsigned)h->tim2_cnt, (unsigned)h->tim2_smcr, (unsigned)h->tim2_cr1,
          (unsigned)h->tim2_ccer, (unsigned)h->pa15_config,
          (unsigned)h->pb3_config, (unsigned)(h->inputs & 3U));
    else if (log_state.phase == 4U)
      length = snprintf(output_line, sizeof(output_line),
          "TW_UART T=%lu EV=%u U1CR1=%X U2CR1=%X U4CR1=%X CLK=%X PB7=%X PC11=%X APB1=%lX\r\n",
          (unsigned long)r->time_ms, (unsigned)r->event,
          (unsigned)h->usart1_cr1, (unsigned)h->usart2_cr1, (unsigned)h->uart4_cr1,
          (unsigned)((h->inputs >> 4U) & 7U),
          (unsigned)h->pb7_config, (unsigned)h->pc11_config, (unsigned long)h->apb1_enr);
    else
      length = snprintf(output_line, sizeof(output_line),
          "TW_PIN T=%lu EV=%u N=%u HIGH_C=%u HIGH_D=%u CHECK_C=%u BAD_C=%u "
          "CHECK_D=%u BAD_D=%u PHASE=%X IN=%X GRAY=%03X,%03X POS=%d HEAD=%d AGE=%u\r\n",
          (unsigned long)r->time_ms,(unsigned)r->event,
          (unsigned)h->probe_samples,(unsigned)h->c_high,(unsigned)h->d_high,
          (unsigned)h->c_checked,(unsigned)h->c_bad,(unsigned)h->d_checked,(unsigned)h->d_bad,
          (unsigned)h->phase_mask,(unsigned)h->actual_outputs,
          (unsigned)r->gray_front,(unsigned)r->gray_rear,
          (int)r->position_error,(int)r->heading_error,(unsigned)r->gray_age_ms);
  }
  if (length < 0 || (unsigned)length >= sizeof(output_line)) return;
  if (UartDebug_TrySendString(output_line) == 0U) return;
  if (log_state.phase == 0U) log_state.phase = 1U;
  else if (log_state.phase < 5U) log_state.phase++;
  else { log_state.phase = 1U; log_state.exported++; }
}

uint32_t TreasureWheelLog_WorkBytes(void)
{
  return (uint32_t)(sizeof(records) + sizeof(log_state) + sizeof(output_line) +
      sizeof(gray_tick) + sizeof(gray_front) + sizeof(gray_rear) +
      sizeof(position_error) + sizeof(heading_error) + sizeof(gray_valid));
}
