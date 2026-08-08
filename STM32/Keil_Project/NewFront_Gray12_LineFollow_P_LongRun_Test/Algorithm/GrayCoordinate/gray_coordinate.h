#ifndef GRAY_COORDINATE_H
#define GRAY_COORDINATE_H

#include <stdint.h>

typedef enum
{
    GRAY_MOUNT_ORIENTATION_UNKNOWN = 0,
    GRAY_MOUNT_P1_LEFT_P12_RIGHT,
    GRAY_MOUNT_P12_LEFT_P1_RIGHT
} GrayMountOrientation_t;

uint8_t GrayCoordinate_ToVehicleError(int16_t label_position_x100,
                                      GrayMountOrientation_t orientation,
                                      int16_t *vehicle_error_x100);

#endif /* GRAY_COORDINATE_H */
