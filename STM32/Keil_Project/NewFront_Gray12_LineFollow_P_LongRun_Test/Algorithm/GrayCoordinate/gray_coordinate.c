#include "gray_coordinate.h"

uint8_t GrayCoordinate_ToVehicleError(int16_t label_position_x100,
                                      GrayMountOrientation_t orientation,
                                      int16_t *vehicle_error_x100)
{
    if (vehicle_error_x100 == 0)
    {
        return 0U;
    }

    *vehicle_error_x100 = 0;

    if (orientation == GRAY_MOUNT_P1_LEFT_P12_RIGHT)
    {
        *vehicle_error_x100 = label_position_x100;
        return 1U;
    }
    if (orientation == GRAY_MOUNT_P12_LEFT_P1_RIGHT)
    {
        *vehicle_error_x100 = (int16_t)(-label_position_x100);
        return 1U;
    }

    return 0U;
}
