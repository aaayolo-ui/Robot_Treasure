#include "TftRoutePage.h"
#include "SystemTime.h"
#include "TFT.h"

#define TFT_ROUTE_PAGE_REFRESH_PERIOD_MS 200U
#define TFT_ROUTE_PAGE_LINE_LENGTH        21U

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
  TFT_DrawString(0U, y, line, TFT_COLOR_WHITE, TFT_COLOR_BLACK);
}

void TftRoutePage_Init(void)
{
  TFT_Init();
  tft_route_page_last_refresh_tick = SystemTime_GetMs();
  tft_route_page_last_junction = 0U;
  tft_route_page_valid = 0U;
}

void TftRoutePage_Task(const char *state_text, uint8_t junction_count)
{
  uint32_t now;
  char line[22];
  const char *state = TftRoutePage_StateText(state_text);

  now = SystemTime_GetMs();
  if ((tft_route_page_valid != 0U) &&
      (TftRoutePage_StateChanged(state) == 0U) &&
      (tft_route_page_last_junction == junction_count) &&
      ((uint32_t)(now - tft_route_page_last_refresh_tick) <
       TFT_ROUTE_PAGE_REFRESH_PERIOD_MS))
  {
    return;
  }

  tft_route_page_last_refresh_tick = now;
  tft_route_page_last_junction = junction_count;
  TftRoutePage_SaveState(state);
  tft_route_page_valid = 1U;

  TftRoutePage_DrawLine(0U, "ROUTE 15A");
  line[0] = 'S';
  line[1] = 'T';
  line[2] = 'A';
  line[3] = 'T';
  line[4] = 'E';
  line[5] = ' ';
  line[6] = '\0';
  TftRoutePage_DrawLine(24U, line);
  TftRoutePage_DrawLine(32U, state);
  line[0] = 'J';
  line[1] = 'U';
  line[2] = 'N';
  line[3] = 'C';
  line[4] = 'T';
  line[5] = 'I';
  line[6] = 'O';
  line[7] = 'N';
  line[8] = ' ';
  line[9] = (char)('0' + ((junction_count > 3U) ? 3U : junction_count));
  line[10] = '\0';
  TftRoutePage_DrawLine(64U, line);
}
