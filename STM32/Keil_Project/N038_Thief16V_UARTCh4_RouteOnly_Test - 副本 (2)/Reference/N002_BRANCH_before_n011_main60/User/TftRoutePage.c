#include "TftRoutePage.h"
#include "SystemTime.h"
#include "TFT.h"

#define TFT_ROUTE_PAGE_REFRESH_PERIOD_MS 200U
#define TFT_ROUTE_PAGE_LINE_LENGTH        18U
#define TFT_ROUTE_PAGE_TEXT_X             16U

static uint32_t tft_route_page_last_refresh_tick;
static char tft_route_page_last_state[22];
static uint8_t tft_route_page_last_junction;
static uint8_t tft_route_page_valid;

static const char *TftRoutePage_StateText(const char *state_text)
{
  return (state_text != 0) ? state_text : "IDLE";
}

static uint8_t TftRoutePage_StateChanged(const char *state_text)
{
  const char *text = TftRoutePage_StateText(state_text);
  uint8_t index;

  for (index = 0U; index < (uint8_t)(sizeof(tft_route_page_last_state) - 1U);
       index++)
  {
    if (tft_route_page_last_state[index] != text[index])
    {
      return 1U;
    }
    if (text[index] == '\0')
    {
      return 0U;
    }
  }
  return (text[index] == '\0') ? 0U : 1U;
}

static void TftRoutePage_SaveState(const char *state_text)
{
  const char *text = TftRoutePage_StateText(state_text);
  uint8_t index;

  for (index = 0U; index < (uint8_t)(sizeof(tft_route_page_last_state) - 1U);
       index++)
  {
    tft_route_page_last_state[index] = text[index];
    if (text[index] == '\0')
    {
      return;
    }
  }
  tft_route_page_last_state[sizeof(tft_route_page_last_state) - 1U] = '\0';
}

static void TftRoutePage_DrawLine(uint8_t y, const char *text)
{
  char line[22];
  uint8_t index;

  for (index = 0U; index < TFT_ROUTE_PAGE_LINE_LENGTH; index++)
  {
    line[index] = ' ';
  }
  line[TFT_ROUTE_PAGE_LINE_LENGTH] = '\0';
  for (index = 0U;
       (index < TFT_ROUTE_PAGE_LINE_LENGTH) && (text[index] != '\0');
       index++)
  {
    line[index] = text[index];
  }
  TFT_DrawString(TFT_ROUTE_PAGE_TEXT_X, y, line,
                 TFT_COLOR_WHITE, TFT_COLOR_BLACK);
}

void TftRoutePage_Init(void)
{
  TFT_Init();
  tft_route_page_last_refresh_tick = SystemTime_GetMs();
  tft_route_page_last_junction = 0U;
  tft_route_page_valid = 0U;
}

void TftRoutePage_Task(const char *state_text, const char *fault_text,
                       uint8_t junction_count,
                       uint8_t turn_angle_valid, int16_t turn_angle_x100)
{
  uint32_t now;
  char line[22];
  const char *state = TftRoutePage_StateText(state_text);
  int32_t magnitude;
  uint8_t first_draw;
  uint8_t state_changed;
  uint8_t junction_changed;

  now = SystemTime_GetMs();
  first_draw = (tft_route_page_valid == 0U) ? 1U : 0U;
  state_changed = TftRoutePage_StateChanged(state);
  junction_changed = (tft_route_page_last_junction != junction_count) ? 1U : 0U;

  /* YAW normally changes on every received frame. Do not let that bypass the
   * refresh period: this display uses blocking software SPI, and repainting
   * the whole page can delay high-speed gray sampling. State/junction changes
   * remain immediate; angle-only updates are limited to 5 Hz. */
  if ((tft_route_page_valid != 0U) &&
      (state_changed == 0U) &&
      (junction_changed == 0U) &&
      ((uint32_t)(now - tft_route_page_last_refresh_tick) <
       TFT_ROUTE_PAGE_REFRESH_PERIOD_MS))
  {
    return;
  }

  tft_route_page_last_refresh_tick = now;
  tft_route_page_last_junction = junction_count;
  TftRoutePage_SaveState(state);
  tft_route_page_valid = 1U;

  if (first_draw != 0U)
  {
    TftRoutePage_DrawLine(8U, "SIDE TURN FOLLOW");
    TftRoutePage_DrawLine(32U, "STATE");
  }
  if ((first_draw != 0U) || (state_changed != 0U))
  {
    TftRoutePage_DrawLine(40U, state);
  }
  if ((first_draw != 0U) || (state_changed != 0U) || (fault_text != 0))
  {
    TftRoutePage_DrawLine(56U, (fault_text != 0) ? fault_text : "");
  }
  if ((first_draw != 0U) || (junction_changed != 0U))
  {
    line[0] = 'J';
    line[1] = 'U';
    line[2] = 'N';
    line[3] = 'C';
    line[4] = 'T';
    line[5] = 'I';
    line[6] = 'O';
    line[7] = 'N';
    line[8] = ' ';
    line[9] = (char)('0' + ((junction_count / 100U) % 10U));
    line[10] = (char)('0' + ((junction_count / 10U) % 10U));
    line[11] = (char)('0' + (junction_count % 10U));
    line[12] = '\0';
    TftRoutePage_DrawLine(72U, line);
  }

  if (turn_angle_valid == 0U)
  {
    TftRoutePage_DrawLine(104U, "YAW NO DATA");
    return;
  }

  magnitude = (turn_angle_x100 < 0) ?
              -(int32_t)turn_angle_x100 : (int32_t)turn_angle_x100;
  line[0] = 'Y';
  line[1] = 'A';
  line[2] = 'W';
  line[3] = ' ';
  line[4] = (turn_angle_x100 < 0) ? '-' : '+';
  line[5] = (char)('0' + ((magnitude / 10000L) % 10L));
  line[6] = (char)('0' + ((magnitude / 1000L) % 10L));
  line[7] = (char)('0' + ((magnitude / 100L) % 10L));
  line[8] = '.';
  line[9] = (char)('0' + ((magnitude / 10L) % 10L));
  line[10] = (char)('0' + (magnitude % 10L));
  line[11] = '\0';
  /* 128x128 屏：x=16、y=104，整行完整可见且位于右下方。 */
  TftRoutePage_DrawLine(104U, line);
}
