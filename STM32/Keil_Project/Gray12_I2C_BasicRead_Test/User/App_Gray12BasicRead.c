#include "App_Gray12BasicRead.h"
#include "main.h"
#include "MotorDriver.h"
#include "NCHD12/nchd12.h"
#include "UartDebug/UartDebug.h"

#define GRAY12_SCAN_INTERVAL_MS       1000U
#define GRAY12_READ_INTERVAL_MS       200U
#define GRAY12_ERROR_REPORT_MS        1000U

static uint8_t gray12_inputs_configured;
static uint32_t gray12_last_scan_ms;
static uint32_t gray12_last_read_ms;
static uint32_t gray12_last_error_report_ms;

static void Gray12_EnsureMotorDisabled(void)
{
    MotorDriver_Disable();
}

static void Gray12_ReportError(const char *message)
{
    uint32_t now = HAL_GetTick();

    if ((now - gray12_last_error_report_ms) >= GRAY12_ERROR_REPORT_MS)
    {
        gray12_last_error_report_ms = now;
        UartDebug_Printf("Gray12 error: %s\r\n", message);
    }
}

static void Gray12_PrintScanResult(void)
{
    uint8_t index;
    uint8_t address7;
    uint8_t device_count = NCHD12_GetDeviceCount();

    UartDebug_Printf("I2C scan complete: %u device(s) found\r\n", device_count);

    for (index = 0U; index < device_count; index++)
    {
        if (NCHD12_GetFoundAddress(index, &address7) != 0U)
        {
            UartDebug_Printf("  I2C address: 0x%02X\r\n", address7);
        }
    }

    if (NCHD12_HasSelectedDevice() != 0U)
    {
        UartDebug_Printf("Selected PCA9555-compatible address: 0x%02X\r\n",
                         NCHD12_GetSelectedAddress());
    }
    else if (device_count == 0U)
    {
        UartDebug_Printf("No I2C device detected; scan will retry.\r\n");
    }
    else
    {
        UartDebug_Printf("No PCA9555-compatible address in 0x20-0x27; scan will retry.\r\n");
    }
}

static void Gray12_PrintChannels(uint16_t raw16)
{
    uint8_t channel;
    uint16_t gray12 = NCHD12_Extract12(raw16);

    UartDebug_Printf("ADDR=0x%02X RAW16=0x%04X GRAY12=0x%03X\r\n",
                     NCHD12_GetSelectedAddress(), raw16, gray12);
    UartDebug_Printf("P1 P2 P3 P4 P5 P6 P7 P8 P9 P10 P11 P12\r\n");

    for (channel = 1U; channel <= NCHD12_CHANNEL_COUNT; channel++)
    {
        UartDebug_Printf("%2u ", NCHD12_GetChannel(gray12, channel));
    }

    UartDebug_Printf("\r\nBLACK_COUNT=%u\r\n", NCHD12_CountBlack(gray12));
}

void App_Gray12BasicRead_Init(void)
{
    Gray12_EnsureMotorDisabled();
    NCHD12_Init();

    gray12_inputs_configured = 0U;
    gray12_last_scan_ms = HAL_GetTick() - GRAY12_SCAN_INTERVAL_MS;
    gray12_last_read_ms = HAL_GetTick();
    gray12_last_error_report_ms = HAL_GetTick() - GRAY12_ERROR_REPORT_MS;

    UartDebug_Printf("\r\n=== 11A Gray12 I2C Basic Read Test ===\r\n");
    UartDebug_Printf("I2C1: PB8=SCL, PB9=SDA, 100 kHz\r\n");
    UartDebug_Printf("PCA9555-compatible access, 7-bit address scan: 0x08-0x77\r\n");
    UartDebug_Printf("Gray input mapping: P1=B0 ... P12=B11\r\n");
    UartDebug_Printf("Gray polarity: BLACK=1, WHITE=0\r\n");
    UartDebug_Printf("Unused input bits: B12-B15\r\n");
    UartDebug_Printf("Gray mask: 0x0FFF\r\n");
    UartDebug_Printf("Motor driver disabled; STY low; PWM outputs remain zero.\r\n");
}

void App_Gray12BasicRead_Task(void)
{
    uint32_t now = HAL_GetTick();
    uint16_t raw16;

    Gray12_EnsureMotorDisabled();

    if ((NCHD12_HasSelectedDevice() == 0U) &&
        ((now - gray12_last_scan_ms) >= GRAY12_SCAN_INTERVAL_MS))
    {
        gray12_last_scan_ms = now;
        UartDebug_Printf("Scanning I2C bus...\r\n");
        (void)NCHD12_ScanBus();
        Gray12_PrintScanResult();
        gray12_inputs_configured = 0U;
    }

    if ((NCHD12_HasSelectedDevice() == 0U) ||
        ((now - gray12_last_read_ms) < GRAY12_READ_INTERVAL_MS))
    {
        return;
    }

    gray12_last_read_ms = now;

    if (gray12_inputs_configured == 0U)
    {
        if (NCHD12_ConfigureInputs() != NCHD12_STATUS_OK)
        {
            Gray12_ReportError("input configuration failed");
            return;
        }

        gray12_inputs_configured = 1U;
        UartDebug_Printf("PCA9555-compatible input registers configured.\r\n");
    }

    if (NCHD12_ReadRaw16(&raw16) != NCHD12_STATUS_OK)
    {
        gray12_inputs_configured = 0U;
        Gray12_ReportError("input read failed; rescanning I2C bus");
        return;
    }

    Gray12_PrintChannels(raw16);
}
