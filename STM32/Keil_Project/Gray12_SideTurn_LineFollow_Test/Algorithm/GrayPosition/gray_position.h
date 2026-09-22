#ifndef GRAY_POSITION_H
#define GRAY_POSITION_H

#include <stdint.h>

typedef enum
{
    GRAY_POSITION_STATUS_NO_LINE = 0,
    GRAY_POSITION_STATUS_VALID,
    GRAY_POSITION_STATUS_ALL_BLACK
} GrayPosition_Status_t;

typedef struct
{
    uint16_t gray12;
    uint8_t black_count;
    int32_t weighted_sum_x100;
    int16_t position_x100;
    int16_t error_x100;
    GrayPosition_Status_t status;
    uint8_t valid;
} GrayPosition_Result_t;

void GrayPosition_Calculate(uint16_t gray12, GrayPosition_Result_t *result);

#endif /* GRAY_POSITION_H */
