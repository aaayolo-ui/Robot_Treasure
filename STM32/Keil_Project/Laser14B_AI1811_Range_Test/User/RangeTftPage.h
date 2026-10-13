#ifndef RANGE_TFT_PAGE_H
#define RANGE_TFT_PAGE_H

#include <stdint.h>

void RangeTftPage_Init(void);
void RangeTftPage_Task(uint8_t have_frame,
                       uint16_t distance_mm,
                       uint8_t confidence,
                       uint8_t good,
                       uint32_t frames,
                       uint32_t good_frames,
                       uint32_t frame_age_ms);

#endif /* RANGE_TFT_PAGE_H */
