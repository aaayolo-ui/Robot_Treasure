#include "DriveOutputSnapshot.h"
#include "main.h"

void DriveOutputSnapshot_Read(DriveOutputSnapshot_t *s)
{
  if (s == 0) return;
  s->tim5_cr1 = TIM5->CR1;
  s->tim5_ccer = TIM5->CCER;
  s->tim5_ccmr2 = TIM5->CCMR2;
  s->tim5_ccr3 = TIM5->CCR3;
  s->tim5_arr = TIM5->ARR;
  s->apb1_enr = RCC->APB1ENR;
  /* Register reads with a gated peripheral clock are not evidence of UE. */
  s->usart2_cr1 = __HAL_RCC_USART2_IS_CLK_ENABLED() ? USART2->CR1 : 0U;
  s->afio_mapr = AFIO->MAPR;
  s->pa2_config = (uint8_t)((GPIOA->CRL >> 8U) & 0xFU);
  s->d_in1 = ((MOTOR_D_IN1_GPIO_Port->ODR & MOTOR_D_IN1_Pin) != 0U);
  s->d_in2 = ((MOTOR_D_IN2_GPIO_Port->ODR & MOTOR_D_IN2_Pin) != 0U);
  s->stby = ((MOTOR_STBY_GPIO_Port->ODR & MOTOR_STBY_Pin) != 0U);
}
