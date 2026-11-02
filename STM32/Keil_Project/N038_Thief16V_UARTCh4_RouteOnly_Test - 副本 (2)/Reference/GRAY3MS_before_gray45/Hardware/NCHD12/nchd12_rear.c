#include "nchd12_rear.h"
#include "SoftI2C.h"
#include "main.h"

#define NCHD12_REAR_ADDRESS7       0x20U
#define NCHD12_REG_INPUT_PORT0     0x00U
#define NCHD12_REG_INPUT_PORT1     0x01U
#define NCHD12_REG_CONFIG_PORT0    0x06U
#define NCHD12_REG_CONFIG_PORT1    0x07U

static NCHD12_Status_t rear_status;
static const SoftI2C_Bus_t nchd12_rear_bus =
{
  NCHD12_REAR_SCL_GPIO_Port,
  NCHD12_REAR_SCL_Pin,
  NCHD12_REAR_SDA_Pin
};

void NCHD12Rear_Init(void)
{
  SoftI2C_Init(&nchd12_rear_bus);
  rear_status = NCHD12_STATUS_NO_DEVICE;
}

NCHD12_Status_t NCHD12Rear_ConfigureInputs(void)
{
  if ((SoftI2C_WriteRegister(&nchd12_rear_bus,
                            NCHD12_REAR_ADDRESS7,
                            NCHD12_REG_CONFIG_PORT0,
                            0xFFU) == 0U) ||
      (SoftI2C_WriteRegister(&nchd12_rear_bus,
                            NCHD12_REAR_ADDRESS7,
                            NCHD12_REG_CONFIG_PORT1,
                            0xFFU) == 0U))
  {
    rear_status = NCHD12_STATUS_I2C_ERROR;
  }
  else
  {
    rear_status = NCHD12_STATUS_OK;
  }
  return rear_status;
}

NCHD12_Status_t NCHD12Rear_ReadRaw16(uint16_t *raw16)
{
  if ((raw16 == 0) ||
      (SoftI2C_ReadRegister16(&nchd12_rear_bus,
                              NCHD12_REAR_ADDRESS7,
                              NCHD12_REG_INPUT_PORT0,
                              raw16) == 0U))
  {
    rear_status = (raw16 == 0) ? NCHD12_STATUS_INVALID_ARGUMENT : NCHD12_STATUS_I2C_ERROR;
  }
  else
  {
    rear_status = NCHD12_STATUS_OK;
  }
  return rear_status;
}
