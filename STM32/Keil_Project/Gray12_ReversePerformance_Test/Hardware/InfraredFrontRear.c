#include "InfraredFrontRear.h"
#include "main.h"

uint8_t InfraredFrontRear_FrontDetected(void)
{
  return (HAL_GPIO_ReadPin(IR_FRONT_OUT_GPIO_Port, IR_FRONT_OUT_Pin) ==
          GPIO_PIN_RESET) ? 1U : 0U;
}

uint8_t InfraredFrontRear_RearDetected(void)
{
  return (HAL_GPIO_ReadPin(IR_REAR_OUT_GPIO_Port, IR_REAR_OUT_Pin) ==
          GPIO_PIN_RESET) ? 1U : 0U;
}
