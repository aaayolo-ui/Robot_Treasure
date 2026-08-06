#include "App_Gray12Position.h"
#include "gray_position.h"
#include "MotorDriver.h"
#include "NCHD12/nchd12.h"
#include "SystemTime.h"
#include "UartDebug/UartDebug.h"

#define GRAY12_SCAN_INTERVAL_MS       1000U
#define GRAY12_READ_INTERVAL_MS       200U
#define GRAY12_ERROR_REPORT_MS        1000U

static uint8_t gray12_inputs_configured;
static uint32_t gray12_last_scan_ms;
static uint32_t gray12_last_read_ms;
static uint32_t gray12_last_error_report_ms;

typedef struct
{
    uint16_t gray12;
    uint8_t black_count;
    int16_t position_x100;
    GrayPosition_Status_t status;
    uint8_t valid;
} Gray12Position_TestVector_t;

static const Gray12Position_TestVector_t gray12_position_test_vectors[] =
{
    {0x000U, 0U,    0, GRAY_POSITION_STATUS_NO_LINE,   0U},
    {0x001U, 1U, -550, GRAY_POSITION_STATUS_VALID,     1U},
    {0x020U, 1U,  -50, GRAY_POSITION_STATUS_VALID,     1U},
    {0x040U, 1U,   50, GRAY_POSITION_STATUS_VALID,     1U},
    {0x060U, 2U,    0, GRAY_POSITION_STATUS_VALID,     1U},
    {0x030U, 2U, -100, GRAY_POSITION_STATUS_VALID,     1U},
    {0x0C0U, 2U,  100, GRAY_POSITION_STATUS_VALID,     1U},
    {0x800U, 1U,  550, GRAY_POSITION_STATUS_VALID,     1U},
    {0xFFFU, 12U,   0, GRAY_POSITION_STATUS_ALL_BLACK, 0U}
};

static void Gray12_EnsureMotorDisabled(void)
{
    MotorDriver_Disable();
}

static void Gray12_ReportError(const char *message)
{
    uint32_t now = SystemTime_GetMs();

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

static const char *Gray12_PositionStatusText(GrayPosition_Status_t status)
{
    if (status == GRAY_POSITION_STATUS_VALID)
    {
        return "VALID";
    }
    if (status == GRAY_POSITION_STATUS_ALL_BLACK)
    {
        return "ALL_BLACK";
    }
    return "NO_LINE";
}

static void Gray12_RunSoftwareTests(void)
{
    uint8_t index;
    uint8_t passed = 0U;

    UartDebug_Printf("Software vectors:\r\n");
    for (index = 0U;
         index < (uint8_t)(sizeof(gray12_position_test_vectors) /
                           sizeof(gray12_position_test_vectors[0]));
         index++)
    {
        GrayPosition_Result_t result;
        const Gray12Position_TestVector_t *test = &gray12_position_test_vectors[index];

        GrayPosition_Calculate(test->gray12, &result);
        if ((result.black_count == test->black_count) &&
            (result.position_x100 == test->position_x100) &&
            (result.error_x100 == test->position_x100) &&
            (result.status == test->status) &&
            (result.valid == test->valid))
        {
            passed++;
        }
        else
        {
            UartDebug_Printf("  FAIL GRAY12=0x%03X got COUNT=%u POS=%d ERR=%d STATUS=%s VALID=%u\r\n",
                             test->gray12, result.black_count,
                             result.position_x100, result.error_x100,
                             Gray12_PositionStatusText(result.status), result.valid);
        }
    }

    UartDebug_Printf("Software vectors: %u/%u PASS\r\n", passed,
                     (uint8_t)(sizeof(gray12_position_test_vectors) /
                               sizeof(gray12_position_test_vectors[0])));
}

static void Gray12_PrintResult(uint16_t raw16)
{
    GrayPosition_Result_t result;
    uint16_t gray12 = NCHD12_Extract12(raw16);

    GrayPosition_Calculate(gray12, &result);
    UartDebug_Printf("ADDR=0x%02X RAW16=0x%04X GRAY12=0x%03X COUNT=%u POS_X100=%d ERR_X100=%d STATUS=%s VALID=%u\r\n",
                     NCHD12_GetSelectedAddress(), raw16, result.gray12,
                     result.black_count, result.position_x100, result.error_x100,
                     Gray12_PositionStatusText(result.status), result.valid);
}

void App_Gray12Position_Init(void)
{
    Gray12_EnsureMotorDisabled();
    NCHD12_Init();

    gray12_inputs_configured = 0U;
    gray12_last_scan_ms = SystemTime_GetMs() - GRAY12_SCAN_INTERVAL_MS;
    gray12_last_read_ms = SystemTime_GetMs();
    gray12_last_error_report_ms = SystemTime_GetMs() - GRAY12_ERROR_REPORT_MS;

    UartDebug_Printf("\r\n=== 11B Gray12 Position and Lateral Error Test ===\r\n");
    UartDebug_Printf("I2C1: PB8=SCL, PB9=SDA, 100 kHz\r\n");
    UartDebug_Printf("PCA9555-compatible access, 7-bit address scan: 0x08-0x77\r\n");
    UartDebug_Printf("Gray input mapping: P1=B0 ... P12=B11\r\n");
    UartDebug_Printf("Gray polarity: BLACK=1, WHITE=0\r\n");
    UartDebug_Printf("Coordinates: P1=-550 ... P12=+550; P6/P7 midpoint=0\r\n");
    UartDebug_Printf("Coordinates use sensor labels only; vehicle left/right is unverified.\r\n");
    UartDebug_Printf("Motor driver disabled; STY low; PWM outputs remain zero.\r\n");
    Gray12_RunSoftwareTests();
}

void App_Gray12Position_Task(void)
{
    uint32_t now = SystemTime_GetMs();
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

    Gray12_PrintResult(raw16);
}
