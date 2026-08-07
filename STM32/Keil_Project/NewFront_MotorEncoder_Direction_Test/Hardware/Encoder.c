#include "Encoder.h"
#include "main.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

static TIM_HandleTypeDef *const encoder_timer[ENCODER_ID_COUNT] =
{
  &htim3,
  &htim4,
  &htim2,
  &htim1
};

static uint16_t previous_count[ENCODER_ID_COUNT];
static int32_t total_count[ENCODER_ID_COUNT];

static uint8_t Encoder_IsValid(EncoderId_t encoder)
{
  return ((uint32_t)encoder < (uint32_t)ENCODER_ID_COUNT) ? 1U : 0U;
}

void Encoder_InitAll(void)
{
  EncoderId_t encoder;

  for (encoder = ENCODER_ID_A; encoder < ENCODER_ID_COUNT; encoder++)
  {
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

  current_count = (uint16_t)__HAL_TIM_GET_COUNTER(encoder_timer[encoder]);
  delta = (int16_t)(current_count - previous_count[encoder]);
  previous_count[encoder] = current_count;
  total_count[encoder] += (int32_t)delta;

  return delta;
}

int32_t Encoder_GetTotal(EncoderId_t encoder)
{
  return (Encoder_IsValid(encoder) != 0U) ? total_count[encoder] : 0;
}

uint16_t Encoder_GetRawCount(EncoderId_t encoder)
{
  if (Encoder_IsValid(encoder) == 0U)
  {
    return 0U;
  }

  return (uint16_t)__HAL_TIM_GET_COUNTER(encoder_timer[encoder]);
}

int16_t Encoder_CalculateRawDelta(uint16_t start_count, uint16_t end_count)
{
  return (int16_t)(end_count - start_count);
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
