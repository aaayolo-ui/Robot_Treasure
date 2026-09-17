#include "gray_position.h"

#define GRAY_POSITION_CHANNEL_COUNT 12U
#define GRAY_POSITION_MASK          0x0FFFU

/* P1 side is negative, P12 side is positive, and zero lies between P6/P7.
 * This is sensor-label space only; vehicle left/right awaits installation verification. */
static const int16_t gray_position_weights_x100[GRAY_POSITION_CHANNEL_COUNT] =
{
    -550, -450, -350, -250, -150, -50,
      50,  150,  250,  350,  450, 550
};

void GrayPosition_Calculate(uint16_t gray12, GrayPosition_Result_t *result)
{
    uint8_t channel;
    uint8_t black_count = 0U;
    int32_t weighted_sum_x100 = 0;

    if (result == 0)
    {
        return;
    }

    gray12 &= GRAY_POSITION_MASK;
    result->gray12 = gray12;
    result->black_count = 0U;
    result->weighted_sum_x100 = 0;
    result->position_x100 = 0;
    result->error_x100 = 0;
    result->status = GRAY_POSITION_STATUS_NO_LINE;
    result->valid = 0U;

    if (gray12 == 0U)
    {
        return;
    }
    if (gray12 == GRAY_POSITION_MASK)
    {
        result->black_count = GRAY_POSITION_CHANNEL_COUNT;
        result->status = GRAY_POSITION_STATUS_ALL_BLACK;
        return;
    }

    for (channel = 0U; channel < GRAY_POSITION_CHANNEL_COUNT; channel++)
    {
        if ((gray12 & ((uint16_t)1U << channel)) != 0U)
        {
            weighted_sum_x100 += gray_position_weights_x100[channel];
            black_count++;
        }
    }

    result->black_count = black_count;
    result->weighted_sum_x100 = weighted_sum_x100;
    result->position_x100 = (int16_t)(weighted_sum_x100 / (int32_t)black_count);
    result->error_x100 = result->position_x100;
    result->status = GRAY_POSITION_STATUS_VALID;
    result->valid = 1U;
}
