#include "Encoder.h"
#include "main.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

static TIM_HandleTypeDef *const encoder_timer[ENCODER_ID_COUNT] =
{
  &htim3,
  0,
  &htim2,
  &htim1
};

static uint16_t previous_count[ENCODER_ID_COUNT];
static int32_t total_count[ENCODER_ID_COUNT];
static volatile int32_t software_b_pending_count;
static volatile int32_t software_b_total_count;
static volatile uint8_t software_b_last_state;

/*
 * The table follows TIMx encoder mode 3 with TI1=channel A and TI2=channel B.
 * The 00->10->11->01->00 sequence is positive, matching encoder B's
 * previously verified raw forward-count sign.
 */
static const int8_t software_b_decode_table[16] =
{
  0, -1,  1,  0,
  1,  0,  0, -1,
 -1,  0,  0,  1,
  0,  1, -1,  0
};

static uint8_t Encoder_SoftwareB_ReadState(void)
{
  uint32_t input_state = ENCODER_B_A_GPIO_Port->IDR;
  uint8_t state = 0U;

  if ((input_state & ENCODER_B_A_Pin) != 0U)
  {
    state |= 2U;
  }
  if ((input_state & ENCODER_B_B_Pin) != 0U)
  {
    state |= 1U;
  }
  return state;
}

static void Encoder_SoftwareB_Reset(void)
{
  uint32_t primask = __get_PRIMASK();

  __disable_irq();
  software_b_pending_count = 0;
  software_b_total_count = 0;
  software_b_last_state = Encoder_SoftwareB_ReadState();
  if (primask == 0U)
  {
    __enable_irq();
  }
}

static int16_t Encoder_SoftwareB_GetDelta(void)
{
  uint32_t primask = __get_PRIMASK();
  int32_t delta;

  __disable_irq();
  delta = software_b_pending_count;
  software_b_pending_count = 0;
  if (primask == 0U)
  {
    __enable_irq();
  }

  if (delta > 32767L)
  {
    return 32767;
  }
  if (delta < -32768L)
  {
    return -32768;
  }
  return (int16_t)delta;
}

static int32_t Encoder_SoftwareB_GetTotal(void)
{
  uint32_t primask = __get_PRIMASK();
  int32_t total;

  __disable_irq();
  total = software_b_total_count;
  if (primask == 0U)
  {
    __enable_irq();
  }
  return total;
}

static uint8_t Encoder_IsValid(EncoderId_t encoder)
{
  return ((uint32_t)encoder < (uint32_t)ENCODER_ID_COUNT) ? 1U : 0U;
}

void Encoder_InitAll(void)
{
  EncoderId_t encoder;

  for (encoder = ENCODER_ID_A; encoder < ENCODER_ID_COUNT; encoder++)
  {
    if (encoder == ENCODER_ID_B)
    {
      continue;
    }
    if (HAL_TIM_Encoder_Start(encoder_timer[encoder], TIM_CHANNEL_ALL) != HAL_OK)
    {
      Error_Handler();
    }
  }
  Encoder_ResetAll();
}

void Encoder_Reset(EncoderId_t encoder)
{
  if (Encoder_IsValid(encoder) == 0U)
  {
    return;
  }

  if (encoder == ENCODER_ID_B)
  {
    Encoder_SoftwareB_Reset();
    return;
  }

  __HAL_TIM_SET_COUNTER(encoder_timer[encoder], 0U);
  previous_count[encoder] = 0U;
  total_count[encoder] = 0;
}

void Encoder_ResetAll(void)
{
  EncoderId_t encoder;

  for (encoder = ENCODER_ID_A; encoder < ENCODER_ID_COUNT; encoder++)
  {
    Encoder_Reset(encoder);
  }
}

int16_t Encoder_GetDelta(EncoderId_t encoder)
{
  uint16_t current_count;
  int16_t delta;

  if (Encoder_IsValid(encoder) == 0U)
  {
    return 0;
  }

  if (encoder == ENCODER_ID_B)
  {
    return Encoder_SoftwareB_GetDelta();
  }

  current_count = (uint16_t)__HAL_TIM_GET_COUNTER(encoder_timer[encoder]);
  delta = (int16_t)(current_count - previous_count[encoder]);
  previous_count[encoder] = current_count;
  total_count[encoder] += (int32_t)delta;

  return delta;
}

int32_t Encoder_GetTotal(EncoderId_t encoder)
{
  if (encoder == ENCODER_ID_B)
  {
    return Encoder_SoftwareB_GetTotal();
  }
  return (Encoder_IsValid(encoder) != 0U) ? total_count[encoder] : 0;
}

int32_t Encoder_CalculateRpmX10(EncoderId_t encoder,
                                int16_t delta_count,
                                uint32_t elapsed_ms)
{
  int64_t numerator;
  int64_t denominator;

  if ((Encoder_IsValid(encoder) == 0U) || (elapsed_ms == 0U))
  {
    return 0;
  }

  numerator = (int64_t)delta_count * 600000LL;
  denominator = (int64_t)MOTOR_ENCODER_COUNTS_PER_REV * (int64_t)elapsed_ms;

  if (numerator >= 0)
  {
    numerator += denominator / 2LL;
  }
  else
  {
    numerator -= denominator / 2LL;
  }

  return (int32_t)(numerator / denominator);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  uint8_t current_state;
  uint8_t transition;
  int8_t step;

  if ((GPIO_Pin != ENCODER_B_A_Pin) && (GPIO_Pin != ENCODER_B_B_Pin))
  {
    return;
  }

  current_state = Encoder_SoftwareB_ReadState();
  transition = (uint8_t)((software_b_last_state << 2U) | current_state);
  step = software_b_decode_table[transition];
  software_b_last_state = current_state;
  software_b_pending_count += (int32_t)step;
  software_b_total_count += (int32_t)step;
}
