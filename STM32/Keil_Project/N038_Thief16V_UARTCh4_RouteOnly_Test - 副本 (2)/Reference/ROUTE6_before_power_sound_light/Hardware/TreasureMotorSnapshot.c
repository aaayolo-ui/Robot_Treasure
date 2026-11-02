#include "TreasureMotorSnapshot.h"
#include "main.h"
#include "Encoder.h"
#include <string.h>

static uint16_t probe[8];
static uint16_t compare_c, compare_d;
static uint32_t compare_c_tick, compare_d_tick;
static uint8_t probe_initialized;

void TreasureMotorSnapshot_ReadDIn2PinMap(TreasureDIn2PinMap_t *snapshot)
{
  snapshot->pa10_config = (uint8_t)((GPIOA->CRH >> 8U) & 0xFU);
  snapshot->pb15_config = (uint8_t)((GPIOB->CRH >> 28U) & 0xFU);
  snapshot->din2_level = (uint8_t)((MOTOR_D_IN2_GPIO_Port->IDR & MOTOR_D_IN2_Pin) != 0U);
  snapshot->stby = (uint8_t)((MOTOR_STBY_GPIO_Port->IDR & MOTOR_STBY_Pin) != 0U);
}

void TreasureMotorSnapshot_Init(void)
{
  memset(probe, 0, sizeof(probe));
  probe_initialized = 0U;
}

static uint8_t AwayFromEdge(uint16_t count, uint16_t compare, uint16_t period)
{
  int32_t delta = (int32_t)count - compare;
  return (uint8_t)(count > 8U && (uint32_t)count + 8U < period &&
                  (delta > 8L || delta < -8L));
}

void TreasureMotorSnapshot_Probe(uint32_t now)
{
  uint16_t before = (uint16_t)TIM5->CNT;
  uint16_t c = (uint16_t)TIM5->CCR2, d = (uint16_t)TIM5->CCR3;
  uint16_t pins = (uint16_t)GPIOA->IDR;
  uint16_t after = (uint16_t)TIM5->CNT;
  uint16_t period = (uint16_t)TIM5->ARR;
  uint8_t high_c = (uint8_t)((pins >> 1U) & 1U);
  uint8_t high_d = (uint8_t)((pins >> 2U) & 1U);
  if (!probe_initialized || c != compare_c) {
    compare_c = c; compare_c_tick = now;
  }
  if (!probe_initialized || d != compare_d) {
    compare_d = d; compare_d_tick = now;
  }
  probe_initialized = 1U;
  if (after < before || (uint16_t)(after - before) > 8U || probe[0] == 65535U) return;
  probe[0]++; probe[1] += high_c; probe[2] += high_d;
  /* Current project uses up-counting PWM1, active high, ARR=999. Reject
   * other configurations rather than assigning them a false expected level. */
  if ((TIM5->CR1 & 0x11U) != 1U || period != 999U) return;
  if ((TIM5->CCER & 0x30U) == 0x10U &&
      (TIM5->CCMR1 & 0x7000U) == 0x6000U &&
      (uint32_t)(now - compare_c_tick) >= 2U &&
      AwayFromEdge(before,c,period) && AwayFromEdge(after,c,period)) {
    probe[3]++;
    probe[4] += (uint16_t)((before < c) != high_c);
    probe[7] |= (uint16_t)(1U << (before >> 7U));
  }
  if ((TIM5->CCER & 0x300U) == 0x100U &&
      (TIM5->CCMR2 & 0x70U) == 0x60U &&
      (uint32_t)(now - compare_d_tick) >= 2U &&
      AwayFromEdge(before,d,period) && AwayFromEdge(after,d,period)) {
    probe[5]++;
    probe[6] += (uint16_t)((before < d) != high_d);
    probe[7] |= (uint16_t)(1U << (8U + (before >> 7U)));
  }
}

void TreasureMotorSnapshot_ReadEncoderMap(TreasureEncoderMap_t *s)
{
  if (s == 0) return;
  /* Raw hardware counters wrap at 16 bits. Do not consume Encoder_GetDelta
   * or reset any counter: this check must not perturb route odometry. */
  s->raw_a = (uint16_t)TIM3->CNT;
  s->raw_b = Encoder_GetTotal(ENCODER_ID_B);
  s->raw_c = (uint16_t)TIM2->CNT;
  s->raw_d = (uint16_t)TIM1->CNT;
  s->pwm_a = (uint16_t)TIM8->CCR1; s->pwm_b = (uint16_t)TIM8->CCR2;
  s->pwm_c = (uint16_t)TIM5->CCR2; s->pwm_d = (uint16_t)TIM5->CCR3;
  s->stby = (uint8_t)((MOTOR_STBY_GPIO_Port->ODR & MOTOR_STBY_Pin) != 0U);
}

void TreasureMotorSnapshot_Read(TreasureMotorSnapshot_t *s)
{
  uint32_t pa_input, pb_input;
  uint8_t uart_clocks;
  if (s == 0) return;
  s->apb1_enr = RCC->APB1ENR;
  s->tim5_ccr2 = (uint16_t)TIM5->CCR2;
  s->tim5_ccr3 = (uint16_t)TIM5->CCR3;
  s->tim5_arr = (uint16_t)TIM5->ARR;
  s->tim5_cr1 = (uint16_t)TIM5->CR1;
  s->tim5_ccer = (uint16_t)TIM5->CCER;
  s->tim5_ccmr1 = (uint16_t)TIM5->CCMR1;
  s->tim5_ccmr2 = (uint16_t)TIM5->CCMR2;
  s->tim2_cnt = (uint16_t)TIM2->CNT;
  s->tim2_smcr = (uint16_t)TIM2->SMCR;
  s->tim2_cr1 = (uint16_t)TIM2->CR1;
  s->tim2_ccer = (uint16_t)TIM2->CCER;
  uart_clocks = 0U;
  if (__HAL_RCC_USART1_IS_CLK_ENABLED()) uart_clocks |= 1U;
  if (__HAL_RCC_USART2_IS_CLK_ENABLED()) uart_clocks |= 2U;
  if (__HAL_RCC_UART4_IS_CLK_ENABLED()) uart_clocks |= 4U;
  s->usart1_cr1 = (uart_clocks & 1U) ? (uint16_t)USART1->CR1 : 0U;
  s->usart2_cr1 = (uart_clocks & 2U) ? (uint16_t)USART2->CR1 : 0U;
  s->uart4_cr1 = (uart_clocks & 4U) ? (uint16_t)UART4->CR1 : 0U;
  s->pa1_config = (uint8_t)((GPIOA->CRL >> 4U) & 0xFU);
  s->pa2_config = (uint8_t)((GPIOA->CRL >> 8U) & 0xFU);
  s->pa15_config = (uint8_t)((GPIOA->CRH >> 28U) & 0xFU);
  s->pb3_config = (uint8_t)((GPIOB->CRL >> 12U) & 0xFU);
  s->pb7_config = (uint8_t)((GPIOB->CRL >> 28U) & 0xFU);
  s->pc11_config = (uint8_t)((GPIOC->CRH >> 12U) & 0xFU);
  s->outputs = 0U;
  if ((MOTOR_C_IN1_GPIO_Port->ODR & MOTOR_C_IN1_Pin) != 0U) s->outputs |= 1U;
  if ((MOTOR_C_IN2_GPIO_Port->ODR & MOTOR_C_IN2_Pin) != 0U) s->outputs |= 2U;
  if ((MOTOR_D_IN1_GPIO_Port->ODR & MOTOR_D_IN1_Pin) != 0U) s->outputs |= 4U;
  if ((MOTOR_D_IN2_GPIO_Port->ODR & MOTOR_D_IN2_Pin) != 0U) s->outputs |= 8U;
  if ((MOTOR_STBY_GPIO_Port->ODR & MOTOR_STBY_Pin) != 0U) s->outputs |= 16U;
  pa_input = GPIOA->IDR;
  pb_input = GPIOB->IDR;
  s->inputs = (uint8_t)(((pa_input >> 15U) & 1U) | (((pb_input >> 3U) & 1U) << 1U) |
                        ((uint8_t)(uart_clocks << 4U)));
  s->probe_samples = probe[0]; s->c_high = probe[1]; s->d_high = probe[2];
  s->c_checked = probe[3]; s->c_bad = probe[4];
  s->d_checked = probe[5]; s->d_bad = probe[6]; s->phase_mask = probe[7];
  s->actual_outputs = (uint8_t)(
      ((MOTOR_C_IN1_GPIO_Port->IDR & MOTOR_C_IN1_Pin) ? 1U : 0U) |
      ((MOTOR_C_IN2_GPIO_Port->IDR & MOTOR_C_IN2_Pin) ? 2U : 0U) |
      ((MOTOR_D_IN1_GPIO_Port->IDR & MOTOR_D_IN1_Pin) ? 4U : 0U) |
      ((MOTOR_D_IN2_GPIO_Port->IDR & MOTOR_D_IN2_Pin) ? 8U : 0U) |
      ((MOTOR_STBY_GPIO_Port->IDR & MOTOR_STBY_Pin) ? 16U : 0U));
  memset(probe, 0, sizeof(probe));
}
