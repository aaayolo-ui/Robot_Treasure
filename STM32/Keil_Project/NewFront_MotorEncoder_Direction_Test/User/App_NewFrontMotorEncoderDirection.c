#include "App_NewFrontMotorEncoderDirection.h"
#include "Encoder.h"
#include "Key.h"
#include "MotorDriver.h"
#include "NewFrontDirectionMap.h"
#include "SystemTime.h"
#include "UartDebug.h"

#define MOTOR_TEST_PWM                 180U
#define MOTOR_TEST_RUN_TIMEOUT_MS      800U
#define MOTOR_TEST_WK_UP_HOLD_MS       1000U

typedef enum
{
  APP_STATE_IDLE = 0,
  APP_STATE_WK_UP_HOLD,
  APP_STATE_RUNNING,
  APP_STATE_WAIT_RELEASE
} AppState_t;

static MotorId_t selected_motor;
static MotorDirection_t selected_electrical_direction;
static AppState_t app_state;
static uint32_t hold_start_ms;
static uint32_t run_start_ms;
static uint16_t encoder_raw_start;
static uint8_t run_wk_up_released;
static uint8_t run_diagnostic_reported;

static const char *App_ChannelName(MotorId_t motor)
{
  switch (motor)
  {
    case MOTOR_ID_A: return "A";
    case MOTOR_ID_B: return "B";
    case MOTOR_ID_C: return "C";
    case MOTOR_ID_D: return "D";
    default: return "INVALID";
  }
}

static const char *App_PhysicalWheelName(MotorId_t motor)
{
  switch (motor)
  {
    case MOTOR_ID_A: return "FR";
    case MOTOR_ID_B: return "FL";
    case MOTOR_ID_C: return "RR";
    case MOTOR_ID_D: return "RL";
    default: return "INVALID";
  }
}

static const char *App_ElectricalDirectionName(MotorDirection_t direction)
{
  return (direction == MOTOR_DIRECTION_REVERSE) ? "ELEC_REV" : "ELEC_FWD";
}

static const char *App_RawSignName(int16_t delta)
{
  if (delta > 0)
  {
    return "POSITIVE";
  }
  if (delta < 0)
  {
    return "NEGATIVE";
  }
  return "ZERO";
}

static void App_PrintVerifiedDirectionMap(MotorId_t motor)
{
  MotorDirection_t vehicle_forward_electrical_direction;
  int8_t encoder_sign = NewFrontDirectionMap_GetEncoderLogicalSign(motor);

  if (NewFrontDirectionMap_GetElectricalDirection(
        motor, NEW_FRONT_VEHICLE_DIRECTION_FORWARD,
        &vehicle_forward_electrical_direction) == 0U)
  {
    UartDebug_SendString("VERIFIED_MAP_ERROR\r\n");
    return;
  }

  UartDebug_Printf("CHANNEL_%s=%s: VEHICLE_FORWARD=%s ENCODER_LOGICAL_SIGN=%d\r\n",
                   App_ChannelName(motor), App_PhysicalWheelName(motor),
                   App_ElectricalDirectionName(vehicle_forward_electrical_direction),
                   (int)encoder_sign);
}

static uint8_t App_PhysicalMapSelfTest(void)
{
  return ((App_PhysicalWheelName(MOTOR_ID_B)[0] == 'F') &&
          (App_PhysicalWheelName(MOTOR_ID_B)[1] == 'L') &&
          (App_PhysicalWheelName(MOTOR_ID_A)[0] == 'F') &&
          (App_PhysicalWheelName(MOTOR_ID_A)[1] == 'R') &&
          (App_PhysicalWheelName(MOTOR_ID_D)[0] == 'R') &&
          (App_PhysicalWheelName(MOTOR_ID_D)[1] == 'L') &&
          (App_PhysicalWheelName(MOTOR_ID_C)[0] == 'R') &&
          (App_PhysicalWheelName(MOTOR_ID_C)[1] == 'R')) ? 1U : 0U;
}

static uint8_t App_IsSelectionValid(void)
{
  return (((uint32_t)selected_motor < (uint32_t)MOTOR_ID_COUNT) &&
          ((selected_electrical_direction == MOTOR_DIRECTION_FORWARD) ||
           (selected_electrical_direction == MOTOR_DIRECTION_REVERSE))) ? 1U : 0U;
}

static void App_ForceSafeState(void)
{
  Motor_StopAll();
  MotorDriver_Disable();
}

static const char *App_HighLowText(uint8_t high)
{
  return (high != 0U) ? "HIGH" : "LOW";
}

static const char *App_YesNoText(uint8_t value)
{
  return (value != 0U) ? "YES" : "NO";
}

static void App_PrintMotorDiagnostics(const char *phase)
{
  MotorDriver_Diagnostics_t diagnostics;

  if (MotorDriver_GetDiagnostics(selected_motor, &diagnostics) == 0U)
  {
    UartDebug_SendString("DIAGNOSTIC_ERROR=INVALID_CHANNEL\r\n");
    return;
  }

  UartDebug_Printf("DIAGNOSTIC_PHASE=%s\r\n", phase);
  UartDebug_Printf("DIAG_CHANNEL=%s\r\n", App_ChannelName(selected_motor));
  UartDebug_Printf("DIAG_ELECTRICAL_DIRECTION=%s\r\n",
                   App_ElectricalDirectionName(selected_electrical_direction));
  UartDebug_Printf("PWM_TIMER=TIM%u\r\n", (unsigned int)diagnostics.timer_number);
  UartDebug_Printf("PWM_CHANNEL=CH%u\r\n", (unsigned int)diagnostics.timer_channel);
  UartDebug_Printf("PWM_ARR=%u\r\n", (unsigned int)diagnostics.pwm_arr);
  UartDebug_Printf("PWM_CCR=%u\r\n", (unsigned int)diagnostics.pwm_compare);
  UartDebug_Printf("IN1_EXPECTED=%s\r\n", App_HighLowText(diagnostics.in1_expected_high));
  UartDebug_Printf("IN2_EXPECTED=%s\r\n", App_HighLowText(diagnostics.in2_expected_high));
  UartDebug_Printf("IN1_ACTUAL=%s\r\n", App_HighLowText(diagnostics.in1_actual_high));
  UartDebug_Printf("IN2_ACTUAL=%s\r\n", App_HighLowText(diagnostics.in2_actual_high));
  UartDebug_Printf("STBY_STATE=%s\r\n", App_HighLowText(diagnostics.stby_high));
  UartDebug_Printf("PWM_STARTED=%s\r\n", App_YesNoText(diagnostics.pwm_started));
}

static void App_PrintSelection(void)
{
  UartDebug_Printf("SELECTED_CHANNEL=%s\r\n", App_ChannelName(selected_motor));
  UartDebug_Printf("PHYSICAL_WHEEL=%s\r\n", App_PhysicalWheelName(selected_motor));
  UartDebug_Printf("ELECTRICAL_DIRECTION=%s\r\n",
                   App_ElectricalDirectionName(selected_electrical_direction));
  UartDebug_Printf("TEST_PWM_COMPARE=%u\r\n", (unsigned int)MOTOR_TEST_PWM);
  UartDebug_Printf("RUN_TIMEOUT_MS=%u\r\n",
                   (unsigned int)MOTOR_TEST_RUN_TIMEOUT_MS);
  UartDebug_SendString("MOTOR_STATE=DISABLED\r\nHOLD_WK_UP_TO_RUN\r\n");
}

static void App_AbortTest(const char *message)
{
  App_ForceSafeState();
  app_state = APP_STATE_WAIT_RELEASE;
  UartDebug_SendString(message);
}

static void App_FinishTest(void)
{
  uint16_t encoder_raw_end = Encoder_GetRawCount((EncoderId_t)selected_motor);
  int16_t encoder_raw_delta = Encoder_CalculateRawDelta(encoder_raw_start,
                                                          encoder_raw_end);

  App_ForceSafeState();
  app_state = APP_STATE_WAIT_RELEASE;

  UartDebug_SendString("TEST_END\r\n");
  UartDebug_Printf("CHANNEL=%s\r\n", App_ChannelName(selected_motor));
  UartDebug_Printf("PHYSICAL_WHEEL=%s\r\n", App_PhysicalWheelName(selected_motor));
  UartDebug_Printf("ELECTRICAL_DIRECTION=%s\r\n",
                   App_ElectricalDirectionName(selected_electrical_direction));
  UartDebug_Printf("RUN_TIME_MS=%lu\r\n",
                   (unsigned long)(SystemTime_GetMs() - run_start_ms));
  UartDebug_Printf("ENC_RAW_START=%u\r\n", (unsigned int)encoder_raw_start);
  UartDebug_Printf("ENC_RAW_END=%u\r\n", (unsigned int)encoder_raw_end);
  UartDebug_Printf("ENC_RAW_DELTA=%d\r\n", (int)encoder_raw_delta);
  UartDebug_Printf("ENC_RAW_SIGN=%s\r\n", App_RawSignName(encoder_raw_delta));
  if (encoder_raw_delta == 0)
  {
    UartDebug_SendString("ENCODER_NO_COUNT_WARNING\r\n");
  }
  UartDebug_SendString("MOTOR_STATE=DISABLED\r\n");
  UartDebug_SendString("PHYSICAL_DIRECTION=USER_OBSERVATION_REQUIRED\r\n");
}

static void App_StartTest(void)
{
  if (App_IsSelectionValid() == 0U)
  {
    App_AbortTest("ERROR=INVALID_CHANNEL_OR_ELECTRICAL_DIRECTION\r\n");
    return;
  }

  App_ForceSafeState();

  if (MotorDriver_SetElectricalDirection(selected_motor,
                                         selected_electrical_direction) == 0U)
  {
    App_AbortTest("ERROR=ELECTRICAL_DIRECTION_PREPARE_FAILED\r\n");
    return;
  }

  Encoder_Reset((EncoderId_t)selected_motor);
  encoder_raw_start = Encoder_GetRawCount((EncoderId_t)selected_motor);

  if ((Motor_GetPwm(MOTOR_ID_A) != 0U && selected_motor != MOTOR_ID_A) ||
      (Motor_GetPwm(MOTOR_ID_B) != 0U && selected_motor != MOTOR_ID_B) ||
      (Motor_GetPwm(MOTOR_ID_C) != 0U && selected_motor != MOTOR_ID_C) ||
      (Motor_GetPwm(MOTOR_ID_D) != 0U && selected_motor != MOTOR_ID_D) ||
      (MotorDriver_SetPreparedPwm(selected_motor, MOTOR_TEST_PWM) == 0U))
  {
    App_AbortTest("ERROR=ONE_MOTOR_PREPARE_CHECK_FAILED\r\n");
    return;
  }

  MotorDriver_EnablePrepared();
  run_start_ms = SystemTime_GetMs();
  run_wk_up_released = 0U;
  run_diagnostic_reported = 0U;
  app_state = APP_STATE_RUNNING;

  UartDebug_SendString("TEST_START\r\n");
  UartDebug_Printf("CHANNEL=%s\r\n", App_ChannelName(selected_motor));
  UartDebug_Printf("PHYSICAL_WHEEL=%s\r\n", App_PhysicalWheelName(selected_motor));
  UartDebug_Printf("ELECTRICAL_DIRECTION=%s\r\n",
                   App_ElectricalDirectionName(selected_electrical_direction));
  UartDebug_Printf("PWM_COMPARE=%u\r\n", (unsigned int)MOTOR_TEST_PWM);
  UartDebug_SendString("MOTOR_STATE=ENABLED\r\n");
  App_PrintMotorDiagnostics("RUN_START");
}

static void App_ProcessIdleKey(KeyEvent_t event)
{
  if (event == KEY_EVENT_KEY1)
  {
    switch (selected_motor)
    {
      case MOTOR_ID_B: selected_motor = MOTOR_ID_A; break;
      case MOTOR_ID_A: selected_motor = MOTOR_ID_D; break;
      case MOTOR_ID_D: selected_motor = MOTOR_ID_C; break;
      case MOTOR_ID_C:
      default: selected_motor = MOTOR_ID_B; break;
    }
    App_PrintSelection();
  }
  else if (event == KEY_EVENT_KEY2)
  {
    selected_electrical_direction =
      (selected_electrical_direction == MOTOR_DIRECTION_FORWARD) ?
      MOTOR_DIRECTION_REVERSE : MOTOR_DIRECTION_FORWARD;
    App_PrintSelection();
  }
  else if (event == KEY_EVENT_WK_UP)
  {
    hold_start_ms = SystemTime_GetMs();
    app_state = APP_STATE_WK_UP_HOLD;
    UartDebug_SendString("WK_UP_HOLD_STARTED\r\n");
  }
}

void App_NewFrontMotorEncoderDirection_Init(void)
{
  MotorDriver_Init();
  Encoder_InitAll();
  Key_Init();
  SystemTime_Init();
  App_ForceSafeState();

  selected_motor = MOTOR_ID_B;
  selected_electrical_direction = MOTOR_DIRECTION_FORWARD;
  app_state = APP_STATE_IDLE;
  hold_start_ms = SystemTime_GetMs();
  run_start_ms = hold_start_ms;
  encoder_raw_start = 0U;
  run_wk_up_released = 0U;
  run_diagnostic_reported = 0U;

  UartDebug_SendString("\r\n=== NEW FRONT MOTOR ENCODER DIRECTION TEST ===\r\n");
  UartDebug_SendString("CURRENT_PHYSICAL_MAP:\r\nB=FL\r\nA=FR\r\nD=RL\r\nC=RR\r\n");
  UartDebug_SendString("MOTOR_DIRECTION_STATUS=VERIFIED\r\n");
  UartDebug_SendString("ENCODER_LOGICAL_SIGN_STATUS=VERIFIED\r\n");
  UartDebug_SendString("VERIFIED_VEHICLE_FORWARD_MAP:\r\n");
  App_PrintVerifiedDirectionMap(MOTOR_ID_B);
  App_PrintVerifiedDirectionMap(MOTOR_ID_A);
  App_PrintVerifiedDirectionMap(MOTOR_ID_D);
  App_PrintVerifiedDirectionMap(MOTOR_ID_C);
  UartDebug_Printf("NEW_FRONT_DIRECTION_MAP_SELFTEST=%s\r\n",
                   (NewFrontDirectionMap_RunSelfTest() != 0U) ? "PASS" : "FAIL");
  UartDebug_SendString("ELECTRICAL_DIRECTION_ONLY:\r\n");
  UartDebug_SendString("ELEC_FWD and ELEC_REV are not vehicle directions.\r\n");
  UartDebug_SendString("SAFETY:\r\nONE_MOTOR_ONLY\r\nPWM_MAX_20_PERCENT\r\n");
  UartDebug_SendString("RUN_TIMEOUT_MS=800\r\nHOLD_WK_UP_1000MS_TO_RUN\r\n");
  UartDebug_SendString("ANY_KEY_DURING_RUN=EMERGENCY_STOP\r\n");
  UartDebug_SendString("CONTROL:\r\nKEY1=SELECT_WHEEL\r\nKEY2=SELECT_ELECTRICAL_DIRECTION\r\n");
  UartDebug_SendString("WK_UP_HOLD=RUN_TEST\r\nSTARTUP_STATE=MOTOR_DISABLED\r\n");
  UartDebug_Printf("PHYSICAL_MAP_SELFTEST=%s\r\n",
                   (App_PhysicalMapSelfTest() != 0U) ? "4/4 PASS" : "FAIL");
  UartDebug_SendString("Lift all wheels off the ground on a stable support.\r\n");
  UartDebug_SendString("Use the 11C new vehicle front for observation.\r\n");
  UartDebug_SendString("Bottom tread toward new rear=PHYSICAL_FORWARD; toward new front=PHYSICAL_REVERSE.\r\n");
  App_PrintSelection();
}

void App_NewFrontMotorEncoderDirection_Task(void)
{
  uint32_t now = SystemTime_GetMs();

  if (app_state == APP_STATE_RUNNING)
  {
    if ((Key_IsPressed(KEY_EVENT_KEY1) != 0U) ||
        (Key_IsPressed(KEY_EVENT_KEY2) != 0U) ||
        ((run_wk_up_released != 0U) &&
         (Key_IsPressed(KEY_EVENT_WK_UP) != 0U)))
    {
      App_AbortTest("TEST_ABORTED\r\n");
    }
    else
    {
      if (Key_IsPressed(KEY_EVENT_WK_UP) == 0U)
      {
        run_wk_up_released = 1U;
      }
      if (((uint32_t)(now - run_start_ms) >= (MOTOR_TEST_RUN_TIMEOUT_MS / 2U)) &&
          (run_diagnostic_reported == 0U))
      {
        run_diagnostic_reported = 1U;
        App_PrintMotorDiagnostics("RUN_MIDPOINT");
      }
      if ((uint32_t)(now - run_start_ms) >= MOTOR_TEST_RUN_TIMEOUT_MS)
      {
        App_FinishTest();
      }
    }
    return;
  }

  if (app_state == APP_STATE_WK_UP_HOLD)
  {
    if ((Key_IsPressed(KEY_EVENT_KEY1) != 0U) ||
        (Key_IsPressed(KEY_EVENT_KEY2) != 0U))
    {
      App_AbortTest("ERROR=KEY_CONFLICT\r\n");
    }
    else if (Key_IsPressed(KEY_EVENT_WK_UP) == 0U)
    {
      app_state = APP_STATE_IDLE;
      UartDebug_SendString("WK_UP_HOLD_CANCELLED\r\n");
    }
    else if ((uint32_t)(now - hold_start_ms) >= MOTOR_TEST_WK_UP_HOLD_MS)
    {
      App_StartTest();
    }
    return;
  }

  if (app_state == APP_STATE_WAIT_RELEASE)
  {
    if (Key_IsAnyPressed() == 0U)
    {
      app_state = APP_STATE_IDLE;
      App_PrintSelection();
    }
    return;
  }

  if ((Key_IsPressed(KEY_EVENT_KEY1) != 0U) &&
      (Key_IsPressed(KEY_EVENT_KEY2) != 0U))
  {
    App_AbortTest("ERROR=KEY_CONFLICT\r\n");
    return;
  }

  if (app_state != APP_STATE_IDLE)
  {
    App_AbortTest("ERROR=APP_STATE_INVALID\r\n");
    return;
  }

  App_ProcessIdleKey(Key_GetEvent());
}
