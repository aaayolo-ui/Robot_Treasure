#include "ThiefTftPage.h"
#include "SystemTime.h"
#include "TFT.h"
#include <stdio.h>

#define REFRESH_MS 200U
#define LINE_CHARS 18U
#define TEXT_X 8U

static uint32_t last_refresh;
static uint8_t first_draw;

static void DrawLine(uint8_t y, const char *text)
{
  char line[20];
  uint8_t i;
  for (i = 0U; i < LINE_CHARS; i++) line[i] = ' ';
  line[LINE_CHARS] = '\0';
  if (text != 0) {
    for (i = 0U; (i < LINE_CHARS) && (text[i] != '\0'); i++) line[i] = text[i];
  }
  TFT_DrawString(TEXT_X, y, line, TFT_COLOR_WHITE, TFT_COLOR_BLACK);
}

static char HeadingChar(uint8_t heading)
{
  static const char h[] = {'N', 'E', 'S', 'W'};
  return h[heading & 3U];
}

void ThiefTftPage_Init(void)
{
  TFT_Init();
  last_refresh = 0U;
  first_draw = 1U;
}

void ThiefTftPage_Task(const char *motion_state, const char *fault_text,
                       const char *strategy_state,
                       const ThiefMapPose_t *pose,
                       const FrontLaserGate_t *laser,
                       uint16_t goal_node)
{
  uint32_t now = SystemTime_GetMs();
  char line[24];
  if ((first_draw == 0U) && ((uint32_t)(now - last_refresh) < REFRESH_MS)) return;
  first_draw = 0U;
  last_refresh = now;
  DrawLine(0U, "THIEF MAP+LASER");
  (void)snprintf(line, sizeof(line), "S:%s", strategy_state != 0 ? strategy_state : "IDLE");
  DrawLine(16U, line);
  (void)snprintf(line, sizeof(line), "M:%s", motion_state != 0 ? motion_state : "IDLE");
  DrawLine(32U, line);
  if ((pose != 0) && (pose->valid != 0U)) {
    (void)snprintf(line, sizeof(line), "N%03u H%c %s",
                   (unsigned)pose->node_id, HeadingChar(pose->body_heading),
                   pose->drive_mode != 0U ? "REV" : "FWD");
    DrawLine(48U, line);
    (void)snprintf(line, sizeof(line), "X%+04d Y%+04d",
                   (int)pose->x_mm, (int)pose->y_mm);
    DrawLine(64U, line);
  } else {
    DrawLine(48U, "POSE INVALID");
    DrawLine(64U, "");
  }
  if (laser != 0) {
    (void)snprintf(line, sizeof(line), "D%04u C%03u %u/5",
                   (unsigned)laser->distance_mm, (unsigned)laser->confidence,
                   (unsigned)laser->accepted_count);
    DrawLine(80U, line);
    (void)snprintf(line, sizeof(line), "L:%s",
                   FrontLaserGate_StatusText(laser->status));
    DrawLine(96U, line);
  }
  (void)snprintf(line, sizeof(line), "GOAL N%03u", (unsigned)goal_node);
  DrawLine(112U, (fault_text != 0) ? fault_text : line);
}

