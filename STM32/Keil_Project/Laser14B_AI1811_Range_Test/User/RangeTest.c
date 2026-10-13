#include "RangeTest.h"
#include "LaserRx.h"
#include "RangeTftPage.h"
#include "main.h"
#include <stdio.h>
#include <string.h>
#define CONFIDENCE_MIN 10U
#define FRESH_MS 200U
extern UART_HandleTypeDef huart3;
static uint32_t sequence, last_frame_ms, last_summary, window_frames, window_good;
static uint32_t sum_mm, max_gap_ms, previous_bad, previous_drop, previous_errors;
static uint16_t min_mm = 65535U, max_mm;
static uint8_t have_frame;
static uint16_t current_distance_mm;
static uint8_t current_confidence;
static uint8_t current_good;
static uint32_t previous_window_frames;
static uint32_t previous_window_good;
static void Print(const char *s)
{ (void)HAL_UART_Transmit(&huart3, (uint8_t *)s, (uint16_t)strlen(s), 100U); }
static void OnFrame(uint16_t distance, uint8_t confidence, uint16_t firmware)
{
  char line[150];
  uint32_t now = HAL_GetTick();
  uint32_t gap = have_frame ? now - last_frame_ms : now - last_summary;
  uint8_t good = (distance >= 50U && distance <= 4000U && confidence >= CONFIDENCE_MIN && confidence <= 100U);
  if (gap > max_gap_ms) { max_gap_ms = gap; }
  last_frame_ms = now; have_frame = 1U; sequence++; window_frames++;
  current_distance_mm = distance;
  current_confidence = confidence;
  current_good = good;
  if (good) {
    window_good++; sum_mm += distance;
    if (distance < min_mm) { min_mm = distance; }
    if (distance > max_mm) { max_mm = distance; }
  }
  (void)snprintf(line, sizeof(line), "LASER MS=%lu SEQ=%lu DIST_MM=%u CONF=%u GOOD=%u FW=%u GAP_MS=%lu\r\n",
    (unsigned long)now, (unsigned long)sequence, (unsigned)distance, (unsigned)confidence,
    (unsigned)good, (unsigned)firmware, (unsigned long)gap);
  Print(line);
}
void RangeTest_Init(void)
{
  Print("\r\nLaser14B_AI1811_Range_Test; STATIONARY STBY=0 PWM=0\r\n");
  Print("RX=PA3 USART2 115200 8N1; LOG=PB10 USART3 115200 8N1\r\n");
  Print("GOOD=checksum_ok AND 50<=mm<=4000 AND 10<=conf<=100; not ground-truth accuracy\r\n");
  last_summary = HAL_GetTick();
  LaserRx_Init();
  RangeTftPage_Init();
}
void RangeTest_Task(void)
{
  char line[280];
  uint32_t now, elapsed, age, bad, drop, errors;
  LaserRx_Poll(OnFrame);
  now = HAL_GetTick();
  age = have_frame ? now - last_frame_ms : now - last_summary;
  RangeTftPage_Task(have_frame,
                    current_distance_mm,
                    current_confidence,
                    current_good,
                    previous_window_frames,
                    previous_window_good,
                    age);
  elapsed = now - last_summary;
  if (elapsed < 1000U) { return; }
  if (age > max_gap_ms) { max_gap_ms = age; }
  LaserRx_Counters(&bad, &drop, &errors);
  (void)snprintf(line, sizeof(line), "WINDOW MS=%lu DT_MS=%lu FRAMES=%lu GOOD=%lu MIN_MM=%u MAX_MM=%u MEAN_MM=%lu BAD_SUM=%lu DROP=%lu UART_ERR=%lu MAX_GAP_MS=%lu LINK=%s\r\n",
    (unsigned long)now, (unsigned long)elapsed, (unsigned long)window_frames, (unsigned long)window_good,
    (unsigned)(window_good ? min_mm : 0U), (unsigned)max_mm,
    (unsigned long)(window_good ? sum_mm / window_good : 0U),
    (unsigned long)(bad - previous_bad), (unsigned long)(drop - previous_drop), (unsigned long)(errors - previous_errors),
    (unsigned long)max_gap_ms, !have_frame ? "NO_DATA" : age > FRESH_MS ? "STALE" : "FRESH");
  Print(line);
  previous_window_frames = window_frames;
  previous_window_good = window_good;
  previous_bad=bad; previous_drop=drop; previous_errors=errors;
  window_frames=0U; window_good=0U; sum_mm=0U; min_mm=65535U; max_mm=0U; max_gap_ms=0U; last_summary=now;
}
