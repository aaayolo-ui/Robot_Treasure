#include "TftPidPage.h"
#include "TFT.h"
#include "SystemTime.h"

#define TFT_PID_PAGE_REFRESH_PERIOD_MS 500U
#define TFT_PID_PAGE_STATUS_LENGTH      21U

static uint32_t tft_pid_page_last_refresh_tick;
static LineFollowPID_Parameters_t tft_pid_page_last_parameters;
static LineFollowPID_ParameterId_t tft_pid_page_last_selected;
static uint8_t tft_pid_page_last_running;
static char tft_pid_page_last_status[24];
static uint8_t tft_pid_page_valid;

static const char *TftPidPage_StatusText(const char *status_text)
{
  return (status_text != 0) ? status_text : "IDLE";
}

static uint8_t TftPidPage_StatusChanged(const char *status_text)
{
  const char *text = TftPidPage_StatusText(status_text);
  uint8_t index;

  for (index = 0U; index < (uint8_t)(sizeof(tft_pid_page_last_status) - 1U);
       index++)
  {
    if (tft_pid_page_last_status[index] != text[index])
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

static void TftPidPage_SaveStatus(const char *status_text)
{
  const char *text = TftPidPage_StatusText(status_text);
  uint8_t index;

  for (index = 0U; index < (uint8_t)(sizeof(tft_pid_page_last_status) - 1U);
       index++)
  {
    tft_pid_page_last_status[index] = text[index];
    if (text[index] == '\0')
    {
      return;
    }
  }
  tft_pid_page_last_status[sizeof(tft_pid_page_last_status) - 1U] = '\0';
}

static void TftPidPage_FormatValue(char *text, int32_t value_x1000)
{
  uint32_t magnitude;
  uint32_t fraction;

  if (value_x1000 < 0)
  {
    text[0] = '-';
    magnitude = (uint32_t)(-value_x1000);
  }
  else
  {
    text[0] = ' ';
    magnitude = (uint32_t)value_x1000;
  }
  text[1] = (char)('0' + (magnitude / 1000U));
  text[2] = '.';
  fraction = magnitude % 1000U;
  text[3] = (char)('0' + (fraction / 100U));
  text[4] = (char)('0' + ((fraction / 10U) % 10U));
  text[5] = (char)('0' + (fraction % 10U));
  text[6] = '\0';
}

static void TftPidPage_FormatParameterLine(char *line,
                                           const char *name,
                                           int32_t value_x1000)
{
  char value[7];

  line[0] = name[0];
  line[1] = name[1];
  line[2] = ' ';
  line[3] = ' ';
  TftPidPage_FormatValue(value, value_x1000);
  line[4] = value[0];
  line[5] = value[1];
  line[6] = value[2];
  line[7] = value[3];
  line[8] = value[4];
  line[9] = value[5];
  line[10] = '\0';
}

static const char *TftPidPage_ParameterName(
    LineFollowPID_ParameterId_t parameter)
{
  static const char *const names[] = {"KP", "KI", "KD", "KH"};

  if ((uint32_t)parameter >= (uint32_t)LINE_FOLLOW_PARAMETER_COUNT)
  {
    return names[0];
  }
  return names[parameter];
}

void TftPidPage_Init(void)
{
  TFT_Init();
  tft_pid_page_last_refresh_tick = SystemTime_GetMs();
  tft_pid_page_valid = 0U;
}

void TftPidPage_Task(const LineFollowPID_Parameters_t *parameters,
                    LineFollowPID_ParameterId_t selected_parameter,
                    uint8_t running,
                    const char *status_text)
{
  uint32_t now;
  char line[24];
  const char *selected_name;
  const char *state_name;
  const char *display_status;
  uint8_t parameters_changed;
  uint8_t status_changed;
  uint8_t index;

  if (parameters == 0)
  {
    return;
  }
  now = SystemTime_GetMs();
  display_status = TftPidPage_StatusText(status_text);
  status_changed = TftPidPage_StatusChanged(display_status);
  parameters_changed =
      (tft_pid_page_last_parameters.kp_x1000 != parameters->kp_x1000) ||
      (tft_pid_page_last_parameters.ki_x1000 != parameters->ki_x1000) ||
      (tft_pid_page_last_parameters.kd_x1000 != parameters->kd_x1000) ||
      (tft_pid_page_last_parameters.kh_x1000 != parameters->kh_x1000);
  if ((tft_pid_page_valid != 0U) &&
      (parameters_changed == 0U) &&
      (tft_pid_page_last_selected == selected_parameter) &&
      (tft_pid_page_last_running == running) &&
      (status_changed == 0U) &&
      ((running != 0U) ||
       ((uint32_t)(now - tft_pid_page_last_refresh_tick) <
        TFT_PID_PAGE_REFRESH_PERIOD_MS)))
  {
    return;
  }
  tft_pid_page_last_refresh_tick = now;
  tft_pid_page_last_parameters = *parameters;
  tft_pid_page_last_selected = selected_parameter;
  tft_pid_page_last_running = running;
  TftPidPage_SaveStatus(display_status);
  tft_pid_page_valid = 1U;
  selected_name = TftPidPage_ParameterName(selected_parameter);
  state_name = (running != 0U) ? "RUN" : "IDLE";

  TFT_DrawString(0U, 0U, "LINE PID", TFT_COLOR_WHITE, TFT_COLOR_BLACK);
  TftPidPage_FormatParameterLine(line, "KP", parameters->kp_x1000);
  TFT_DrawString(0U, 16U, line, TFT_COLOR_WHITE, TFT_COLOR_BLACK);
  TftPidPage_FormatParameterLine(line, "KI", parameters->ki_x1000);
  TFT_DrawString(0U, 32U, line, TFT_COLOR_WHITE, TFT_COLOR_BLACK);
  TftPidPage_FormatParameterLine(line, "KD", parameters->kd_x1000);
  TFT_DrawString(0U, 48U, line, TFT_COLOR_WHITE, TFT_COLOR_BLACK);
  TftPidPage_FormatParameterLine(line, "KH", parameters->kh_x1000);
  TFT_DrawString(0U, 64U, line, TFT_COLOR_WHITE, TFT_COLOR_BLACK);

  line[0] = 'S';
  line[1] = 'E';
  line[2] = 'L';
  line[3] = ' ';
  line[4] = selected_name[0];
  line[5] = selected_name[1];
  line[6] = '\0';
  TFT_DrawString(0U, 80U, line, TFT_COLOR_YELLOW, TFT_COLOR_BLACK);

  line[0] = 'S';
  line[1] = 'T';
  line[2] = 'A';
  line[3] = 'T';
  line[4] = 'E';
  line[5] = ' ';
  line[6] = state_name[0];
  line[7] = state_name[1];
  line[8] = state_name[2];
  line[9] = '\0';
  TFT_DrawString(0U, 96U, line,
                 (running != 0U) ? TFT_COLOR_GREEN : TFT_COLOR_WHITE,
                 TFT_COLOR_BLACK);

  for (index = 0U; index < TFT_PID_PAGE_STATUS_LENGTH; index++)
  {
    line[index] = ' ';
  }
  line[TFT_PID_PAGE_STATUS_LENGTH] = '\0';
  for (index = 0U;
       (index < TFT_PID_PAGE_STATUS_LENGTH) &&
       (display_status[index] != '\0');
       index++)
  {
    line[index] = display_status[index];
  }
  TFT_DrawString(0U, 112U, line, TFT_COLOR_YELLOW, TFT_COLOR_BLACK);
}
