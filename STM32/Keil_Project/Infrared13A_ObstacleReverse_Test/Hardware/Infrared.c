#include "Infrared.h"
#include "main.h"

void Infrared_Init(void)
{
}

uint8_t Infrared_IsDetected(void)
{
  return (HAL_GPIO_ReadPin(IR_OUT_GPIO_Port, IR_OUT_Pin) == GPIO_PIN_RESET) ?
         1U : 0U;
}
