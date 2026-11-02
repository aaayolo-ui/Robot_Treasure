#include "MapTimingLog.h"
#include "Encoder.h"
#include "JY61P.h"
#include "UartDebug.h"
#include "MapRoutePlan.h"
#include "SystemTime.h"
#include "TravelFrame.h"
#include <stdio.h>
#include <string.h>

#define MAP_TIMING_QUEUE_CAPACITY 64U
#define MAP_TIMING_SAMPLE_PERIOD_MS 100U
#define MAP_TIMING_STABLE_SAMPLES 3U

typedef struct
{
  uint32_t run;
  uint32_t sequence;
  uint32_t time_ms;
  uint32_t yaw_age_ms;
  uint32_t yaw_frames;
  uint32_t lost;
  int32_t argument;
  int32_t encoder[4];
  int16_t yaw_x100;
  uint8_t event;
  uint8_t junction;
  uint8_t yaw_valid;
  uint8_t drive_mode;
  uint16_t route, step, from_node, to_node;
} MapTimingRecord_t;

static MapTimingRecord_t queue[MAP_TIMING_QUEUE_CAPACITY];
static uint16_t head;
static uint16_t tail;
static uint32_t run_id;
static uint32_t sequence;
static uint32_t lost;
static uint32_t last_sample_ms;
static uint32_t last_follow_sample_ms;
static uint8_t have_follow_sample;
static uint8_t stable_count;
static uint8_t stable_sent;
static uint8_t active;
static uint16_t current_route, current_step, current_from, current_to;

void MapTimingLog_SetRoute(uint16_t route, uint16_t step,
                          uint16_t from_node, uint16_t to_node)
{
  current_route = route;
  current_step = step;
  current_from = from_node;
  current_to = to_node;
}

static uint16_t MapTimingLog_Crc16(const char *text)
{
  uint16_t crc = 0xFFFFU;
  uint8_t bit;
  while (*text != '\0')
  {
    crc ^= (uint16_t)((uint16_t)(uint8_t)*text << 8);
    text++;
    for (bit = 0U; bit < 8U; bit++)
    {
      crc = ((crc & 0x8000U) != 0U) ?
            (uint16_t)((crc << 1) ^ 0x1021U) : (uint16_t)(crc << 1);
    }
  }
  return crc;
}

void MapTimingLog_Init(void)
{
  head = 0U;
  tail = 0U;
  run_id = 0U;
  sequence = 0U;
  lost = 0U;
  active = 0U;
  have_follow_sample = 0U;
  stable_count = 0U;
  stable_sent = 0U;
}

void MapTimingLog_Begin(uint32_t now)
{
  /* Never discard unsent records from the previous test. run separates them. */
  run_id++;
  sequence = 0U;
  lost = 0U;
  last_sample_ms = now;
  have_follow_sample = 0U;
  stable_count = 0U;
  stable_sent = 0U;
  active = 1U;
  MapTimingLog_Event(MAP_TIMING_START, 0U, now, (int32_t)map_route_signature);
}

void MapTimingLog_Event(MapTimingEvent_t event, uint8_t junction,
                        uint32_t now, int32_t argument)
{
  uint16_t next = (uint16_t)((head + 1U) % MAP_TIMING_QUEUE_CAPACITY);
  MapTimingRecord_t *record;
  JY61P_Data_t yaw;
  uint8_t wheel;

  if ((event == MAP_TIMING_END) && (active == 0U)) return;

  if (event == MAP_TIMING_FOLLOW)
  {
    stable_count = 0U;
    stable_sent = 0U;
    have_follow_sample = 0U;
  }
  if (event == MAP_TIMING_END)
  {
    active = 0U;
  }
  sequence++;
  if (next == tail)
  {
    lost++;
    return;
  }
  record = &queue[head];
  JY61P_GetData(&yaw);
  record->run = run_id;
  record->sequence = sequence;
  record->time_ms = now;
  record->event = (uint8_t)event;
  record->junction = junction;
  record->argument = argument;
  record->yaw_x100 = yaw.yaw_x100;
  record->yaw_valid = yaw.valid;
  /* A JY frame may arrive between the caller's tick and this snapshot. */
  record->yaw_age_ms = (uint32_t)(SystemTime_GetMs() - yaw.last_update_ms);
  record->yaw_frames = yaw.frame_count;
  record->lost = lost;
  record->drive_mode = TravelFrame_IsReverse();
  record->route = current_route;
  record->step = current_step;
  record->from_node = current_from;
  record->to_node = current_to;
  for (wheel = 0U; wheel < 4U; wheel++)
  {
    /* Existing accumulated counts: may lag the current wheel-control sample.
     * Do not consume delta counters or reset encoders for instrumentation. */
    record->encoder[wheel] = Encoder_GetTotal((EncoderId_t)wheel);
  }
  head = next;
}

uint8_t MapTimingLog_FollowSample(uint32_t now, uint8_t junction,
                                  uint8_t front_valid)
{
  uint8_t became_stable = 0U;
  if ((active == 0U) ||
      ((have_follow_sample != 0U) && (now == last_follow_sample_ms)))
  {
    return 0U;
  }
  last_follow_sample_ms = now;
  have_follow_sample = 1U;
  if (front_valid == 0U)
  {
    stable_count = 0U;
  }
  else if (stable_count < MAP_TIMING_STABLE_SAMPLES)
  {
    stable_count++;
  }
  if ((stable_count >= MAP_TIMING_STABLE_SAMPLES) && (stable_sent == 0U))
  {
    MapTimingLog_Event(MAP_TIMING_LINE_STABLE, junction, now,
                        (int32_t)stable_count);
    stable_sent = 1U;
    became_stable = 1U;
  }
  if ((uint32_t)(now - last_sample_ms) >= MAP_TIMING_SAMPLE_PERIOD_MS)
  {
    last_sample_ms = now;
    MapTimingLog_Event(MAP_TIMING_SAMPLE, junction, now,
                        (int32_t)front_valid);
  }
  return became_stable;
}

void MapTimingLog_Task(void)
{
  const MapTimingRecord_t *record;
  char line[256];
  int length;
  uint16_t crc;

  if (tail == head)
  {
    return;
  }
  record = &queue[tail];
  length = snprintf(line, sizeof(line),
      "MT3,%lu,%lu,%lu,%u,%u,%u,%u,%u,%u,%ld,%d,%u,%lu,%lu,%ld,%ld,%ld,%ld,%lu,%u",
      (unsigned long)record->run, (unsigned long)record->sequence,
      (unsigned long)record->time_ms, (unsigned)record->event,
      (unsigned)record->junction, (unsigned)record->route,
      (unsigned)record->step, (unsigned)record->from_node,
      (unsigned)record->to_node, (long)record->argument,
      (int)record->yaw_x100, (unsigned)record->yaw_valid,
      (unsigned long)record->yaw_age_ms, (unsigned long)record->yaw_frames,
      (long)record->encoder[0], (long)record->encoder[1],
      (long)record->encoder[2], (long)record->encoder[3],
      (unsigned long)record->lost, (unsigned)record->drive_mode);
  if ((length < 0) || ((size_t)length + 8U >= sizeof(line)))
  {
    lost++;
    tail = (uint16_t)((tail + 1U) % MAP_TIMING_QUEUE_CAPACITY);
    return;
  }
  crc = MapTimingLog_Crc16(line);
  (void)snprintf(line + length, sizeof(line) - (size_t)length,
                 "*%04X\r\n", (unsigned)crc);
  /* Whole-line retry avoids partially queued measurement records. This task
   * is called by the same main-loop producer as the diagnostic channel. */
  if (UartDebug_TrySendString(line) != 0U)
  {
    tail = (uint16_t)((tail + 1U) % MAP_TIMING_QUEUE_CAPACITY);
  }
}
