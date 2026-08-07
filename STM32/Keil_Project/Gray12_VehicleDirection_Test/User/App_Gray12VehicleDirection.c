#include "App_Gray12VehicleDirection.h"
#include "gray_coordinate.h"
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

static const GrayMountOrientation_t g_gray_mount_orientation =
    GRAY_MOUNT_P12_LEFT_P1_RIGHT;

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

typedef struct
{
    int16_t label_position_x100;
    GrayMountOrientation_t orientation;
    uint8_t expected_success;
    int16_t expected_vehicle_error_x100;
} GrayCoordinate_TestVector_t;

static const GrayCoordinate_TestVector_t gray_coordinate_test_vectors[] =
{
    {-550, GRAY_MOUNT_ORIENTATION_UNKNOWN, 0U,    0},
    {-550, GRAY_MOUNT_P1_LEFT_P12_RIGHT,   1U, -550},
    { 550, GRAY_MOUNT_P1_LEFT_P12_RIGHT,   1U,  550},
    { 550, GRAY_MOUNT_P12_LEFT_P1_RIGHT,   1U, -550},
    {   0, GRAY_MOUNT_P12_LEFT_P1_RIGHT,   1U,    0},
    {-550, GRAY_MOUNT_P12_LEFT_P1_RIGHT,   1U,  550}
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

static uint8_t Gray12_RunPositionSoftwareTests(void)
{
    uint8_t index;
    uint8_t passed = 0U;

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
            return 0U;
        }
    }

    return (passed == (uint8_t)(sizeof(gray12_position_test_vectors) /
                                sizeof(gray12_position_test_vectors[0]))) ? 1U : 0U;
}

static uint8_t Gray12_RunCoordinateSoftwareTests(void)
{
    uint8_t index;
    uint8_t passed = 0U;

    for (index = 0U;
         index < (uint8_t)(sizeof(gray_coordinate_test_vectors) /
                           sizeof(gray_coordinate_test_vectors[0]));
         index++)
    {
        int16_t vehicle_error_x100 = 0;
        const GrayCoordinate_TestVector_t *test = &gray_coordinate_test_vectors[index];
        uint8_t success = GrayCoordinate_ToVehicleError(test->label_position_x100,
                                                         test->orientation,
                                                         &vehicle_error_x100);

        if ((success == test->expected_success) &&
            (vehicle_error_x100 == test->expected_vehicle_error_x100))
        {
            passed++;
        }
        else
        {
            return 0U;
        }
    }

    return (passed == (uint8_t)(sizeof(gray_coordinate_test_vectors) /
                                sizeof(gray_coordinate_test_vectors[0]))) ? 1U : 0U;
}

static const char *Gray12_LabelSideText(int16_t label_position_x100)
{
    if (label_position_x100 < 0)
    {
        return "P1_SIDE";
    }
    if (label_position_x100 > 0)
    {
        return "P12_SIDE";
    }
    return "CENTER";
}

static const char *Gray12_MountOrientationText(GrayMountOrientation_t orientation)
{
    if (orientation == GRAY_MOUNT_P1_LEFT_P12_RIGHT)
    {
        return "P1_LEFT_P12_RIGHT";
    }
    if (orientation == GRAY_MOUNT_P12_LEFT_P1_RIGHT)
    {
        return "P12_LEFT_P1_RIGHT";
    }
    return "UNKNOWN";
}

static const char *Gray12_VehicleSideText(int16_t vehicle_error_x100)
{
    if (vehicle_error_x100 < 0)
    {
        return "VEHICLE_LEFT";
    }
    if (vehicle_error_x100 > 0)
    {
        return "VEHICLE_RIGHT";
    }
    return "CENTER";
}

static void Gray12_PrintChannels(uint16_t gray12)
{
    uint8_t channel;

    for (channel = 1U; channel <= NCHD12_CHANNEL_COUNT; channel++)
    {
        UartDebug_Printf("P%u=%u%s", channel, NCHD12_GetChannel(gray12, channel),
                         (channel == 6U) ? "\r\n" : " ");
    }
    UartDebug_Printf("\r\n");
}

static void Gray12_PrintResult(uint16_t raw16)
{
    GrayPosition_Result_t result;
    int16_t vehicle_error_x100 = 0;
    uint16_t gray12 = NCHD12_Extract12(raw16);

    GrayPosition_Calculate(gray12, &result);
    UartDebug_Printf("ADDR=0x%02X RAW16=0x%04X GRAY12=0x%03X\r\n",
                     NCHD12_GetSelectedAddress(), raw16, result.gray12);
    UartDebug_Printf("COUNT=%u STATUS=%s VALID=%u\r\n",
                     result.black_count, Gray12_PositionStatusText(result.status), result.valid);

    if (result.valid == 0U)
    {
        UartDebug_Printf("LABEL_POSITION=INVALID\r\nVEHICLE_ERROR=INVALID\r\n");
        return;
    }

    UartDebug_Printf("LABEL_POS_X100=%d LABEL_SIDE=%s\r\n",
                     result.position_x100, Gray12_LabelSideText(result.position_x100));
    UartDebug_Printf("MOUNT_ORIENTATION=%s\r\n",
                     Gray12_MountOrientationText(g_gray_mount_orientation));
    if (GrayCoordinate_ToVehicleError(result.position_x100,
                                      g_gray_mount_orientation,
                                      &vehicle_error_x100) == 0U)
    {
        UartDebug_Printf("VEHICLE_ERROR=INVALID\r\n");
    }
    else
    {
        UartDebug_Printf("VEHICLE_ERROR_X100=%d VEHICLE_SIDE=%s\r\n",
                         vehicle_error_x100,
                         Gray12_VehicleSideText(vehicle_error_x100));
    }
    Gray12_PrintChannels(result.gray12);
}

void App_Gray12VehicleDirection_Init(void)
{
    Gray12_EnsureMotorDisabled();
    NCHD12_Init();

    gray12_inputs_configured = 0U;
    gray12_last_scan_ms = SystemTime_GetMs() - GRAY12_SCAN_INTERVAL_MS;
    gray12_last_read_ms = SystemTime_GetMs();
    gray12_last_error_report_ms = SystemTime_GetMs() - GRAY12_ERROR_REPORT_MS;

    UartDebug_Printf("\r\n=== GRAY12 VEHICLE DIRECTION TEST ===\r\n");
    UartDebug_Printf("I2C1: PB8=SCL, PB9=SDA, 100 kHz\r\n");
    UartDebug_Printf("PCA9555-compatible access, 7-bit address scan: 0x08-0x77\r\n");
    UartDebug_Printf("Gray input mapping: P1=B0 ... P12=B11\r\n");
    UartDebug_Printf("Gray polarity: BLACK=1, WHITE=0\r\n");
    UartDebug_Printf("GRAY_POSITION_SELFTEST=%s\r\n",
                     (Gray12_RunPositionSoftwareTests() != 0U) ? "9/9 PASS" : "FAIL");
    UartDebug_Printf("GRAY_COORDINATE_SELFTEST=%s\r\n",
                     (Gray12_RunCoordinateSoftwareTests() != 0U) ? "PASS" : "FAIL");
    UartDebug_Printf("MOUNT_ORIENTATION=P12_LEFT_P1_RIGHT\r\n");
    UartDebug_Printf("LABEL_COORDINATE: P1_SIDE=NEGATIVE, P12_SIDE=POSITIVE\r\n");
    UartDebug_Printf("VEHICLE_COORDINATE: LEFT=NEGATIVE, RIGHT=POSITIVE\r\n");
    UartDebug_Printf("1. Keep the vehicle facing forward.\r\n");
    UartDebug_Printf("2. Stand behind the vehicle and look forward.\r\n");
    UartDebug_Printf("3. Place a narrow black line under VEHICLE_LEFT.\r\n");
    UartDebug_Printf("4. Record active P channels and LABEL_POS_X100.\r\n");
    UartDebug_Printf("5. Move the line to VEHICLE_CENTER.\r\n");
    UartDebug_Printf("6. Move the line to VEHICLE_RIGHT.\r\n");
    UartDebug_Printf("7. Do not rotate the sensor during the test.\r\n");
    UartDebug_Printf("Motor driver disabled; STY low; PWM outputs remain zero.\r\n");
}

void App_Gray12VehicleDirection_Task(void)
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
