#include "Encoder.h"
#include "main.h"

extern TIM_HandleTypeDef htim3;

static uint16_t previous_count;
static int32_t total_count;

void EncoderA_Init(void)
{
  if (HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL) != HAL_OK)
  {
    Error_Handler();
  }

  EncoderA_Reset();
}

int16_t EncoderA_GetDelta(void)
{
  uint16_t current_count;
  int16_t delta;

  current_count = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);

  /*
   * Casting the 16-bit counter difference to int16_t preserves the natural
   * 0-to-65535 rollover, provided one sampling interval changes by less than
   * 32768 counts.
   */
  delta = (int16_t)(current_count - previous_count);
  previous_count = current_count;
  total_count += (int32_t)delta;

  return delta;
}

int32_t EncoderA_GetTotal(void)
{
  return total_count;
}

void EncoderA_Reset(void)
{
  __HAL_TIM_SET_COUNTER(&htim3, 0U);
  previous_count = 0U;
  total_count = 0;
}

int32_t EncoderA_CalculateRpmX10(int16_t delta_count, uint32_t elapsed_ms)
{
  int64_t numerator;
  int64_t denominator;

  if (elapsed_ms == 0U)
  {
    return 0;
  }

  numerator = (int64_t)delta_count * 600000LL;
  denominator = (int64_t)ENCODER_A_COUNTS_PER_REV * (int64_t)elapsed_ms;

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
