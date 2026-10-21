#ifndef MAP_LASER_MASK_H
#define MAP_LASER_MASK_H

#include "ThiefMapPlanner.h"
#include <stdint.h>

/* Road centerline coordinates; the 3.6 x 3 m physical field extends another
 * 150 mm beyond each outermost line. Update these if the map changes. */
#define MAP_LASER_MIN_X_MM 0L
#define MAP_LASER_MAX_X_MM 3300L
#define MAP_LASER_MIN_Y_MM 0L
#define MAP_LASER_MAX_Y_MM 2700L

/* Bit0 front, bit1 left, bit2 right, bit3 rear. This masks software use, not
 * laser emission/power. Reverse travel does not rotate the vehicle body. */
uint8_t MapLaserMask_DetectionMask(const ThiefMapPose_t *pose);
/* Drop queued data from before/within the millisecond of an enable change.
 * Tick ordering assumes observations are less than 2^31 ms apart. */
uint8_t MapLaserMask_FrameAllowed(uint8_t mask, uint8_t channel,
    uint32_t frame_tick, uint32_t enabled_since);

#endif
