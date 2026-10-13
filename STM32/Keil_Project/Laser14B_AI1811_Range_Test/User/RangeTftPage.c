#include "RangeTftPage.h"
#include "TFT.h"
#include "main.h"
#include <stdio.h>

#define RANGE_TFT_REFRESH_MS 200U
#define RANGE_TFT_LINE_CHARS 18U
#define RANGE_TFT_STALE_MS   200U

static uint32_t last_refresh_ms;

static void DrawLine(uint8_t y,
                     const char *text,
                     uint16_t foreground)
{
  char line[RANGE_TFT_LINE_CHARS + 1U];
  uint8_t index;

  for (index = 0U; index < RANGE_TFT_LINE_CHARS; index++)
  {
    line[index] = ' ';
  }
  line[RANGE_TFT_LINE_CHARS] = '\0';
  for (index = 0U;
       (index < RANGE_TFT_LINE_CHARS) && (text[index] != '\0');
       index++)
  {
    line[index] = text[index];
  }
  TFT_DrawString(10U, y, line, foreground, TFT_COLOR_BLACK);
}

void RangeTftPage_Init(void)
{
  TFT_Init();
  DrawLine(8U, "LASER RANGE", TFT_COLOR_WHITE);
  DrawLine(24U, "WAIT DATA", TFT_COLOR_YELLOW);
  DrawLine(40U, "DIST 0", TFT_COLOR_WHITE);
  DrawLine(56U, "CONF 0", TFT_COLOR_WHITE);
  DrawLine(72U, "GOOD 0", TFT_COLOR_YELLOW);
  DrawLine(88U, "FR 0 GOOD 0", TFT_COLOR_WHITE);
  DrawLine(104U, "LINK NO_DATA", TFT_COLOR_YELLOW);
  last_refresh_ms = HAL_GetTick();
}

void RangeTftPage_Task(uint8_t have_frame,
                       uint16_t distance_mm,
                       uint8_t confidence,
                       uint8_t good,
                       uint32_t frames,
                       uint32_t good_frames,
                       uint32_t frame_age_ms)
{
  char line[24];
  uint32_t now = HAL_GetTick();
  const char *link;

  if ((uint32_t)(now - last_refresh_ms) < RANGE_TFT_REFRESH_MS)
  {
    return;
  }
  last_refresh_ms = now;

  if (have_frame == 0U)
  {
    link = "NO_DATA";
  }
  else if (frame_age_ms > RANGE_TFT_STALE_MS)
  {
    link = "STALE";
  }
  else
  {
    link = "FRESH";
  }

  DrawLine(24U, (good != 0U) ? "VALID DATA" : "CHECK DATA",
           (good != 0U) ? TFT_COLOR_GREEN : TFT_COLOR_YELLOW);
  (void)snprintf(line, sizeof(line), "DIST %u", (unsigned)distance_mm);
  DrawLine(40U, line, TFT_COLOR_WHITE);
  (void)snprintf(line, sizeof(line), "CONF %u", (unsigned)confidence);
  DrawLine(56U, line, TFT_COLOR_WHITE);
  (void)snprintf(line, sizeof(line), "GOOD %u", (unsigned)good);
  DrawLine(72U, line,
           (good != 0U) ? TFT_COLOR_GREEN : TFT_COLOR_YELLOW);
  (void)snprintf(line, sizeof(line), "FR %lu GOOD %lu",
                 (unsigned long)frames, (unsigned long)good_frames);
  DrawLine(88U, line, TFT_COLOR_WHITE);
  (void)snprintf(line, sizeof(line), "LINK %s", link);
  DrawLine(104U, line,
           (have_frame != 0U && frame_age_ms <= RANGE_TFT_STALE_MS) ?
             TFT_COLOR_GREEN : TFT_COLOR_YELLOW);
}
