#include "App_MotorChannelTest.h"
#include "Encoder.h"
#include "Key.h"
#include "MotorDriver.h"
#include "UartDebug.h"
#include "main.h"

#define CHANNEL_TEST_PWM              100U
#define CHANNEL_REPORT_PERIOD_MS       500U

typedef enum
{
  TEST_CHANNEL_A = 0,
  TEST_CHANNEL_B,
  TEST_CHANNEL_C,
  TEST_CHANNEL_D,
  TEST_CHANNEL_COUNT
} TestChannel_t;

typedef enum
{
  CHANNEL_TEST_STOP = 0,
  CHANNEL_TEST_FORWARD,
  CHANNEL_TEST_STOP_AFTER_FORWARD,
  CHANNEL_TEST_REVERSE,
  CHANNEL_TEST_STOP_AFTER_REVERSE
} ChannelTestState_t;

static const MotorId_t channel_motor[TEST_CHANNEL_COUNT] =
{
  MOTOR_ID_A,
  MOTOR_ID_B,
  MOTOR_ID_C,
  MOTOR_ID_D
};

static const EncoderId_t channel_encoder[TEST_CHANNEL_COUNT] =
{
  ENCODER_ID_A,
  ENCODER_ID_B,
  ENCODER_ID_C,
  ENCODER_ID_D
};

static const char channel_name[TEST_CHANNEL_COUNT] = {'A', 'B', 'C', 'D'};

static TestChannel_t selected_channel;
static ChannelTestState_t channel_state[TEST_CHANNEL_COUNT];
static uint32_t last_report_tick;

static const char *App_DirectionName(MotorDirection_t direction)
{
  if (direction == MOTOR_DIRECTION_FORWARD)
  {
    return "FWD";
  }
  if (direction == MOTOR_DIRECTION_REVERSE)
  {
    return "REV";
  }
  return "STOP";
}

static void App_SplitRpmX10(int32_t rpm_x10,
                            const char **sign,
                            long *integer,
                            long *decimal)
{
  int64_t value;
  int64_t absolute_value;

  value = (int64_t)rpm_x10;
  absolute_value = (value < 0) ? -value : value;
  *sign = (value < 0) ? "-" : "";
  *integer = (long)(absolute_value / 10LL);
  *decimal = (long)(absolute_value % 10LL);
}

static uint8_t App_AreAllMotorsStopped(void)
{
  TestChannel_t channel;

  for (channel = TEST_CHANNEL_A; channel < TEST_CHANNEL_COUNT; channel++)
  {
    if ((Motor_GetPwm(channel_motor[channel]) != 0U) ||
        (Motor_GetDirection(channel_motor[channel]) != MOTOR_DIRECTION_STOP))
    {
      return 0U;
    }
  }

  return 1U;
}

static void App_ResetAllTestStates(void)
{
  TestChannel_t channel;

  for (channel = TEST_CHANNEL_A; channel < TEST_CHANNEL_COUNT; channel++)
  {
    channel_state[channel] = CHANNEL_TEST_STOP;
  }
}

static void App_EnsureOnlySelectedMotorStopped(void)
{
  TestChannel_t channel;
  uint8_t other_motor_running = 0U;

  for (channel = TEST_CHANNEL_A; channel < TEST_CHANNEL_COUNT; channel++)
  {
    if (channel == selected_channel)
    {
      continue;
    }
    if ((Motor_GetPwm(channel_motor[channel]) != 0U) ||
        (Motor_GetDirection(channel_motor[channel]) != MOTOR_DIRECTION_STOP))
    {
      other_motor_running = 1U;
      break;
    }
  }

  if (other_motor_running != 0U)
  {
    Motor_StopAll();
    App_ResetAllTestStates();
    UartDebug_SendString("Safety stop: another motor was active.\r\n");
  }
}

static void App_SelectNextChannel(void)
{
  if (App_AreAllMotorsStopped() == 0U)
  {
    UartDebug_SendString("Stop the running motor before changing channel.\r\n");
    return;
  }

  selected_channel++;
  if (selected_channel >= TEST_CHANNEL_COUNT)
  {
    selected_channel = TEST_CHANNEL_A;
  }
  UartDebug_Printf("Selected motor channel: %c\r\n", channel_name[selected_channel]);
}

static void App_RunSelectedChannel(void)
{
  MotorId_t motor;

  motor = channel_motor[selected_channel];
  switch (channel_state[selected_channel])
  {
    case CHANNEL_TEST_STOP:
    case CHANNEL_TEST_STOP_AFTER_REVERSE:
      App_EnsureOnlySelectedMotorStopped();
      if (MotorDriver_IsEnabled() == 0U)
      {
        MotorDriver_Enable();
      }
      Motor_Forward(motor, CHANNEL_TEST_PWM);
      channel_state[selected_channel] = CHANNEL_TEST_FORWARD;
      UartDebug_Printf("Motor %c electrical forward at PWM 100.\r\n",
                       channel_name[selected_channel]);
      break;

    case CHANNEL_TEST_FORWARD:
      Motor_Stop(motor);
      channel_state[selected_channel] = CHANNEL_TEST_STOP_AFTER_FORWARD;
      UartDebug_Printf("Motor %c stopped.\r\n", channel_name[selected_channel]);
      break;

    case CHANNEL_TEST_STOP_AFTER_FORWARD:
      App_EnsureOnlySelectedMotorStopped();
      if (MotorDriver_IsEnabled() == 0U)
      {
        MotorDriver_Enable();
      }
      Motor_Reverse(motor, CHANNEL_TEST_PWM);
      channel_state[selected_channel] = CHANNEL_TEST_REVERSE;
      UartDebug_Printf("Motor %c electrical reverse at PWM 100.\r\n",
                       channel_name[selected_channel]);
      break;

    case CHANNEL_TEST_REVERSE:
      Motor_Stop(motor);
      channel_state[selected_channel] = CHANNEL_TEST_STOP_AFTER_REVERSE;
      UartDebug_Printf("Motor %c stopped.\r\n", channel_name[selected_channel]);
      break;

    default:
      channel_state[selected_channel] = CHANNEL_TEST_STOP;
      Motor_Stop(motor);
      break;
  }
}

static void App_ProcessKeys(void)
{
  switch (Key_GetEvent())
  {
    case KEY_EVENT_KEY1:
      App_SelectNextChannel();
      break;

    case KEY_EVENT_KEY2:
      App_RunSelectedChannel();
      break;

    case KEY_EVENT_WK_UP:
      MotorDriver_Disable();
      App_ResetAllTestStates();
      UartDebug_SendString("Emergency stop. All motors disabled.\r\n");
      break;

    case KEY_EVENT_NONE:
    default:
      break;
  }
}

static void App_PrintChannelReport(TestChannel_t channel,
                                   int16_t delta,
                                   int32_t total,
                                   int32_t rpm_x10)
{
  const char *sign;
  long rpm_integer;
  long rpm_decimal;

  App_SplitRpmX10(rpm_x10, &sign, &rpm_integer, &rpm_decimal);
  UartDebug_Printf("%c: %s, PWM: %u, delta: %d, total: %ld, RPM: %s%ld.%ld\r\n",
                   channel_name[channel],
                   App_DirectionName(Motor_GetDirection(channel_motor[channel])),
                   (unsigned int)Motor_GetPwm(channel_motor[channel]),
                   (int)delta,
                   (long)total,
                   sign,
                   rpm_integer,
                   rpm_decimal);
}

static void App_ReportAllChannels(void)
{
  uint32_t now;
  uint32_t elapsed_ms;
  int16_t delta_a;
  int16_t delta_b;
  int16_t delta_c;
  int16_t delta_d;
  int32_t total_a;
  int32_t total_b;
  int32_t total_c;
  int32_t total_d;
  int32_t rpm_a_x10;
  int32_t rpm_b_x10;
  int32_t rpm_c_x10;
  int32_t rpm_d_x10;

  now = HAL_GetTick();
  if ((uint32_t)(now - last_report_tick) < CHANNEL_REPORT_PERIOD_MS)
  {
    return;
  }

  elapsed_ms = (uint32_t)(now - last_report_tick);
  last_report_tick = now;
  delta_a = Encoder_GetDelta(channel_encoder[TEST_CHANNEL_A]);
  delta_b = Encoder_GetDelta(channel_encoder[TEST_CHANNEL_B]);
  delta_c = Encoder_GetDelta(channel_encoder[TEST_CHANNEL_C]);
  delta_d = Encoder_GetDelta(channel_encoder[TEST_CHANNEL_D]);
  total_a = Encoder_GetTotal(channel_encoder[TEST_CHANNEL_A]);
  total_b = Encoder_GetTotal(channel_encoder[TEST_CHANNEL_B]);
  total_c = Encoder_GetTotal(channel_encoder[TEST_CHANNEL_C]);
  total_d = Encoder_GetTotal(channel_encoder[TEST_CHANNEL_D]);
  rpm_a_x10 = Encoder_CalculateRpmX10(channel_encoder[TEST_CHANNEL_A], delta_a, elapsed_ms);
  rpm_b_x10 = Encoder_CalculateRpmX10(channel_encoder[TEST_CHANNEL_B], delta_b, elapsed_ms);
  rpm_c_x10 = Encoder_CalculateRpmX10(channel_encoder[TEST_CHANNEL_C], delta_c, elapsed_ms);
  rpm_d_x10 = Encoder_CalculateRpmX10(channel_encoder[TEST_CHANNEL_D], delta_d, elapsed_ms);

  UartDebug_Printf("Selected: %c, Driver: %s, dt: %lu ms\r\n",
                   channel_name[selected_channel],
                   (MotorDriver_IsEnabled() != 0U) ? "ENABLED" : "DISABLED",
                   (unsigned long)elapsed_ms);
  App_PrintChannelReport(TEST_CHANNEL_A, delta_a, total_a, rpm_a_x10);
  App_PrintChannelReport(TEST_CHANNEL_B, delta_b, total_b, rpm_b_x10);
  App_PrintChannelReport(TEST_CHANNEL_C, delta_c, total_c, rpm_c_x10);
  App_PrintChannelReport(TEST_CHANNEL_D, delta_d, total_d, rpm_d_x10);
  UartDebug_SendString("\r\n");
}

void App_MotorChannelTest_Init(void)
{
  Key_Init();
  MotorDriver_Init();
  Encoder_InitAll();
  selected_channel = TEST_CHANNEL_A;
  App_ResetAllTestStates();
  last_report_tick = HAL_GetTick();

  UartDebug_SendString("\r\n");
  UartDebug_SendString("Four_Motor_Channel_Test started.\r\n");
  UartDebug_SendString("Four motor and encoder channel test.\r\n");
  UartDebug_SendString("Only one motor may run at a time.\r\n");
  UartDebug_SendString("Test PWM: 100\r\n");
  UartDebug_SendString("KEY1: select motor A, B, C or D\r\n");
  UartDebug_SendString("KEY2: forward, stop, reverse, stop\r\n");
  UartDebug_SendString("WK_UP: emergency stop and disable all motors\r\n");
  UartDebug_SendString("Forward and reverse are electrical directions.\r\n");
  UartDebug_SendString("Do not assume electrical forward means vehicle forward.\r\n");
  UartDebug_SendString("All four encoder values are reported every 500 ms.\r\n");
  UartDebug_SendString("Wheels must remain lifted.\r\n");
  UartDebug_SendString("Motor driver is disabled at startup.\r\n");
}

void App_MotorChannelTest_Task(void)
{
  App_ProcessKeys();
  App_ReportAllChannels();
}
