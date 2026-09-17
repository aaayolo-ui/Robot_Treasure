#include "nchd12.h"
#include "main.h"
#include "SoftI2C.h"

#define NCHD12_I2C_TRIALS                 2U
#define NCHD12_SCAN_ADDRESS_FIRST          0x08U
#define NCHD12_SCAN_ADDRESS_LAST           0x77U
#define NCHD12_MAX_FOUND_DEVICES           (NCHD12_SCAN_ADDRESS_LAST - NCHD12_SCAN_ADDRESS_FIRST + 1U)
#define NCHD12_COMPATIBLE_ADDRESS_FIRST    0x20U
#define NCHD12_COMPATIBLE_ADDRESS_LAST     0x27U

/* PCA9555-compatible register access. */
#define NCHD12_REG_INPUT_PORT0             0x00U
#define NCHD12_REG_INPUT_PORT1             0x01U
#define NCHD12_REG_CONFIG_PORT0            0x06U
#define NCHD12_REG_CONFIG_PORT1            0x07U
#define NCHD12_INPUT_MODE                  0xFFU

static const SoftI2C_Bus_t nchd12_front_bus =
{
    NCHD12_FRONT_SCL_GPIO_Port,
    NCHD12_FRONT_SCL_Pin,
    NCHD12_FRONT_SDA_Pin
};

static uint8_t nchd12_found_addresses[NCHD12_MAX_FOUND_DEVICES];
static uint8_t nchd12_found_count;
static uint8_t nchd12_selected_address;
static uint8_t nchd12_device_selected;
static NCHD12_Status_t nchd12_last_status;
static uint32_t nchd12_last_update_ms;
static uint32_t nchd12_last_error_ms;

static void NCHD12_SetStatus(NCHD12_Status_t status)
{
    nchd12_last_status = status;

    if (status != NCHD12_STATUS_OK)
    {
        nchd12_last_error_ms = HAL_GetTick();
    }
}

void NCHD12_Init(void)
{
    SoftI2C_Init(&nchd12_front_bus);
    nchd12_found_count = 0U;
    nchd12_selected_address = 0U;
    nchd12_device_selected = 0U;
    nchd12_last_status = NCHD12_STATUS_NO_DEVICE;
    nchd12_last_update_ms = 0U;
    nchd12_last_error_ms = 0U;
}

NCHD12_Status_t NCHD12_ScanBus(void)
{
    uint16_t address;

    nchd12_found_count = 0U;
    nchd12_selected_address = 0U;
    nchd12_device_selected = 0U;

    for (address = NCHD12_SCAN_ADDRESS_FIRST; address <= NCHD12_SCAN_ADDRESS_LAST; address++)
    {
        if (SoftI2C_IsDeviceReady(&nchd12_front_bus,
                                  (uint8_t)address,
                                  NCHD12_I2C_TRIALS) != 0U)
        {
            nchd12_found_addresses[nchd12_found_count] = (uint8_t)address;
            nchd12_found_count++;

            if ((nchd12_device_selected == 0U) &&
                (address >= NCHD12_COMPATIBLE_ADDRESS_FIRST) &&
                (address <= NCHD12_COMPATIBLE_ADDRESS_LAST))
            {
                nchd12_selected_address = (uint8_t)address;
                nchd12_device_selected = 1U;
            }
        }
    }

    if (nchd12_found_count == 0U)
    {
        NCHD12_SetStatus(NCHD12_STATUS_NO_DEVICE);
    }
    else if (nchd12_device_selected == 0U)
    {
        NCHD12_SetStatus(NCHD12_STATUS_NOT_COMPATIBLE);
    }
    else
    {
        NCHD12_SetStatus(NCHD12_STATUS_OK);
    }

    return nchd12_last_status;
}

uint8_t NCHD12_GetDeviceCount(void)
{
    return nchd12_found_count;
}

uint8_t NCHD12_GetFoundAddress(uint8_t index, uint8_t *address7)
{
    if ((address7 == NULL) || (index >= nchd12_found_count))
    {
        return 0U;
    }

    *address7 = nchd12_found_addresses[index];
    return 1U;
}

uint8_t NCHD12_HasSelectedDevice(void)
{
    return nchd12_device_selected;
}

uint8_t NCHD12_GetSelectedAddress(void)
{
    return nchd12_selected_address;
}

NCHD12_Status_t NCHD12_ConfigureInputs(void)
{
    uint8_t input_mode = NCHD12_INPUT_MODE;

    if (nchd12_device_selected == 0U)
    {
        NCHD12_SetStatus(NCHD12_STATUS_NO_DEVICE);
        return nchd12_last_status;
    }

    if ((SoftI2C_WriteRegister(&nchd12_front_bus,
                               nchd12_selected_address,
                               NCHD12_REG_CONFIG_PORT0,
                               input_mode) == 0U) ||
        (SoftI2C_WriteRegister(&nchd12_front_bus,
                               nchd12_selected_address,
                               NCHD12_REG_CONFIG_PORT1,
                               input_mode) == 0U))
    {
        NCHD12_SetStatus(NCHD12_STATUS_I2C_ERROR);
        return nchd12_last_status;
    }

    NCHD12_SetStatus(NCHD12_STATUS_OK);
    return nchd12_last_status;
}

NCHD12_Status_t NCHD12_ReadRaw16(uint16_t *raw16)
{
    uint16_t input16;

    if (raw16 == NULL)
    {
        NCHD12_SetStatus(NCHD12_STATUS_INVALID_ARGUMENT);
        return nchd12_last_status;
    }

    if (nchd12_device_selected == 0U)
    {
        NCHD12_SetStatus(NCHD12_STATUS_NO_DEVICE);
        return nchd12_last_status;
    }

    if ((SoftI2C_ReadRegister16(&nchd12_front_bus,
                                nchd12_selected_address,
                                NCHD12_REG_INPUT_PORT0,
                                &input16) == 0U))
    {
        NCHD12_SetStatus(NCHD12_STATUS_I2C_ERROR);
        return nchd12_last_status;
    }

    *raw16 = input16;
    nchd12_last_update_ms = HAL_GetTick();
    NCHD12_SetStatus(NCHD12_STATUS_OK);
    return nchd12_last_status;
}

uint16_t NCHD12_Extract12(uint16_t raw16)
{
    return (uint16_t)(raw16 & NCHD12_GRAY12_MASK);
}

uint8_t NCHD12_GetChannel(uint16_t gray12, uint8_t channel)
{
    if ((channel == 0U) || (channel > NCHD12_CHANNEL_COUNT))
    {
        return 0U;
    }

    return (uint8_t)((gray12 >> (channel - 1U)) & 0x01U);
}

uint8_t NCHD12_CountBlack(uint16_t gray12)
{
    uint8_t channel;
    uint8_t black_count = 0U;

    for (channel = 1U; channel <= NCHD12_CHANNEL_COUNT; channel++)
    {
        black_count = (uint8_t)(black_count + NCHD12_GetChannel(gray12, channel));
    }

    return black_count;
}

NCHD12_Status_t NCHD12_GetLastStatus(void)
{
    return nchd12_last_status;
}

uint32_t NCHD12_GetLastUpdateMs(void)
{
    return nchd12_last_update_ms;
}

uint32_t NCHD12_GetLastErrorMs(void)
{
    return nchd12_last_error_ms;
}
