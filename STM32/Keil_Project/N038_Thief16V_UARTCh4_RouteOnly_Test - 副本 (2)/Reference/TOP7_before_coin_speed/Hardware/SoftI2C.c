#include "SoftI2C.h"

static uint8_t SoftI2C_BusIsValid(const SoftI2C_Bus_t *bus)
{
  if ((bus == 0) || (bus->port == 0) ||
      (bus->scl_pin == 0U) || (bus->sda_pin == 0U) ||
      (bus->scl_pin == bus->sda_pin))
  {
    return 0U;
  }
  return 1U;
}

static void SoftI2C_EnablePortClock(GPIO_TypeDef *port)
{
  if (port == GPIOA)
  {
    __HAL_RCC_GPIOA_CLK_ENABLE();
  }
  else if (port == GPIOB)
  {
    __HAL_RCC_GPIOB_CLK_ENABLE();
  }
  else if (port == GPIOC)
  {
    __HAL_RCC_GPIOC_CLK_ENABLE();
  }
  else if (port == GPIOD)
  {
    __HAL_RCC_GPIOD_CLK_ENABLE();
  }
#if defined(GPIOE)
  else if (port == GPIOE)
  {
    __HAL_RCC_GPIOE_CLK_ENABLE();
  }
#endif
}

static void SoftI2C_Delay(void)
{
  volatile uint32_t count;
  for (count = 0U; count < 12U; count++)
  {
    __NOP();
  }
}

static void SoftI2C_ConfigurePin(const SoftI2C_Bus_t *bus,
                                 uint16_t pin,
                                 uint32_t mode,
                                 uint32_t pull)
{
  GPIO_InitTypeDef gpio = {0};

  gpio.Pin = pin;
  gpio.Mode = mode;
  gpio.Pull = pull;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(bus->port, &gpio);
}

static void SoftI2C_SclHigh(const SoftI2C_Bus_t *bus)
{
  /* Release SCL; the input pull-up prevents a floating high level. */
  HAL_GPIO_WritePin(bus->port, bus->scl_pin, GPIO_PIN_SET);
  SoftI2C_ConfigurePin(bus, bus->scl_pin, GPIO_MODE_INPUT, GPIO_PULLUP);
}

static void SoftI2C_SclLow(const SoftI2C_Bus_t *bus)
{
  HAL_GPIO_WritePin(bus->port, bus->scl_pin, GPIO_PIN_RESET);
  SoftI2C_ConfigurePin(bus, bus->scl_pin, GPIO_MODE_OUTPUT_OD,
                       GPIO_NOPULL);
}

static void SoftI2C_SdaHigh(const SoftI2C_Bus_t *bus)
{
  /* Release SDA; the input pull-up lets the device acknowledge/read it. */
  HAL_GPIO_WritePin(bus->port, bus->sda_pin, GPIO_PIN_SET);
  SoftI2C_ConfigurePin(bus, bus->sda_pin, GPIO_MODE_INPUT, GPIO_PULLUP);
}

static void SoftI2C_SdaLow(const SoftI2C_Bus_t *bus)
{
  HAL_GPIO_WritePin(bus->port, bus->sda_pin, GPIO_PIN_RESET);
  SoftI2C_ConfigurePin(bus, bus->sda_pin, GPIO_MODE_OUTPUT_OD,
                       GPIO_NOPULL);
}

static GPIO_PinState SoftI2C_SdaRead(const SoftI2C_Bus_t *bus)
{
  return HAL_GPIO_ReadPin(bus->port, bus->sda_pin);
}

static void SoftI2C_Start(const SoftI2C_Bus_t *bus)
{
  SoftI2C_SdaHigh(bus); SoftI2C_SclHigh(bus); SoftI2C_Delay();
  SoftI2C_SdaLow(bus); SoftI2C_Delay(); SoftI2C_SclLow(bus);
}

static void SoftI2C_Stop(const SoftI2C_Bus_t *bus)
{
  SoftI2C_SdaLow(bus); SoftI2C_SclHigh(bus); SoftI2C_Delay();
  SoftI2C_SdaHigh(bus); SoftI2C_Delay();
}

static uint8_t SoftI2C_WriteByte(const SoftI2C_Bus_t *bus, uint8_t value)
{
  uint8_t bit;
  uint8_t acknowledged;

  for (bit = 0U; bit < 8U; bit++)
  {
    if ((value & 0x80U) != 0U) SoftI2C_SdaHigh(bus);
    else SoftI2C_SdaLow(bus);
    SoftI2C_SclHigh(bus); SoftI2C_Delay(); SoftI2C_SclLow(bus);
    value <<= 1U;
  }
  SoftI2C_SdaHigh(bus); SoftI2C_SclHigh(bus); SoftI2C_Delay();
  acknowledged = (SoftI2C_SdaRead(bus) == GPIO_PIN_RESET) ? 1U : 0U;
  SoftI2C_SclLow(bus);
  return acknowledged;
}

static uint8_t SoftI2C_ReadByte(const SoftI2C_Bus_t *bus,
                                uint8_t acknowledge,
                                uint8_t *value)
{
  uint8_t bit;
  uint8_t result = 0U;

  if (value == 0) return 0U;
  SoftI2C_SdaHigh(bus);
  for (bit = 0U; bit < 8U; bit++)
  {
    result <<= 1U;
    SoftI2C_SclHigh(bus); SoftI2C_Delay();
    if (SoftI2C_SdaRead(bus) != GPIO_PIN_RESET) result |= 1U;
    SoftI2C_SclLow(bus); SoftI2C_Delay();
  }
  if (acknowledge != 0U) SoftI2C_SdaLow(bus);
  else SoftI2C_SdaHigh(bus);
  SoftI2C_SclHigh(bus); SoftI2C_Delay(); SoftI2C_SclLow(bus);
  SoftI2C_SdaHigh(bus);
  *value = result;
  return 1U;
}

void SoftI2C_Init(const SoftI2C_Bus_t *bus)
{
  GPIO_InitTypeDef gpio = {0};

  if (SoftI2C_BusIsValid(bus) == 0U)
  {
    return;
  }

  SoftI2C_EnablePortClock(bus->port);
  gpio.Pin = bus->scl_pin | bus->sda_pin;
  gpio.Mode = GPIO_MODE_OUTPUT_OD;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(bus->port, &gpio);
  SoftI2C_SdaHigh(bus);
  SoftI2C_SclHigh(bus);
}

uint8_t SoftI2C_IsDeviceReady(const SoftI2C_Bus_t *bus,
                              uint8_t address7,
                              uint8_t trials)
{
  uint8_t trial;

  if ((SoftI2C_BusIsValid(bus) == 0U) || (trials == 0U))
  {
    return 0U;
  }

  for (trial = 0U; trial < trials; trial++)
  {
    SoftI2C_Start(bus);
    if (SoftI2C_WriteByte(bus, (uint8_t)(address7 << 1U)) != 0U)
    {
      SoftI2C_Stop(bus);
      return 1U;
    }
    SoftI2C_Stop(bus);
  }
  return 0U;
}

uint8_t SoftI2C_WriteRegister(const SoftI2C_Bus_t *bus,
                              uint8_t address7,
                              uint8_t reg,
                              uint8_t value)
{
  uint8_t ok;

  if (SoftI2C_BusIsValid(bus) == 0U)
  {
    return 0U;
  }

  SoftI2C_Start(bus);
  ok = SoftI2C_WriteByte(bus, (uint8_t)(address7 << 1U));
  if (ok != 0U) ok = SoftI2C_WriteByte(bus, reg);
  if (ok != 0U) ok = SoftI2C_WriteByte(bus, value);
  SoftI2C_Stop(bus);
  return ok;
}

uint8_t SoftI2C_ReadRegister16(const SoftI2C_Bus_t *bus,
                               uint8_t address7,
                               uint8_t reg,
                               uint16_t *value)
{
  uint8_t low;
  uint8_t high;
  uint8_t ok;

  if ((SoftI2C_BusIsValid(bus) == 0U) || (value == 0))
  {
    return 0U;
  }

  SoftI2C_Start(bus);
  ok = SoftI2C_WriteByte(bus, (uint8_t)(address7 << 1U));
  if (ok != 0U) ok = SoftI2C_WriteByte(bus, reg);
  if (ok != 0U)
  {
    SoftI2C_Start(bus);
    ok = SoftI2C_WriteByte(bus, (uint8_t)((address7 << 1U) | 1U));
  }
  if (ok != 0U) ok = SoftI2C_ReadByte(bus, 1U, &low);
  if (ok != 0U) ok = SoftI2C_ReadByte(bus, 0U, &high);
  SoftI2C_Stop(bus);
  if (ok != 0U) *value = (uint16_t)low | ((uint16_t)high << 8U);
  return ok;
}
