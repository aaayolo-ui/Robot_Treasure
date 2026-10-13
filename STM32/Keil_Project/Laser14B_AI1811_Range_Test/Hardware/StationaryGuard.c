#include "StationaryGuard.h"
#include "main.h"
void StationaryGuard_Init(void)
{
  GPIO_InitTypeDef gpio = {0};
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  /* Existing car map: PC2 STBY; PA1/PA2/PC6/PC7 motor PWM. */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2 | GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1 | GPIO_PIN_2, GPIO_PIN_RESET);
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  gpio.Pin = GPIO_PIN_2 | GPIO_PIN_6 | GPIO_PIN_7;
  HAL_GPIO_Init(GPIOC, &gpio);
  gpio.Pin = GPIO_PIN_1 | GPIO_PIN_2;
  HAL_GPIO_Init(GPIOA, &gpio);
}
