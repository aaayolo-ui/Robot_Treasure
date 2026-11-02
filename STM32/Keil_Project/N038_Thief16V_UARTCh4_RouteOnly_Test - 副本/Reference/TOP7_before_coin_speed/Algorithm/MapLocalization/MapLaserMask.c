#include "MapLaserMask.h"
#include <stddef.h>

uint8_t MapLaserMask_DetectionMask(const ThiefMapPose_t *pose)
{
  static const uint8_t offset[4] = {0U, 3U, 1U, 2U};
  uint8_t outside=0U, allowed=0U, i;
  if(pose==NULL || !pose->valid || pose->body_heading>3U ||
      pose->x_mm<MAP_LASER_MIN_X_MM || pose->x_mm>MAP_LASER_MAX_X_MM ||
      pose->y_mm<MAP_LASER_MIN_Y_MM || pose->y_mm>MAP_LASER_MAX_Y_MM) return 0U;
  if(pose->y_mm==MAP_LASER_MIN_Y_MM) outside|=1U<<0U; /* North. */
  if(pose->x_mm==MAP_LASER_MAX_X_MM) outside|=1U<<1U; /* East. */
  if(pose->y_mm==MAP_LASER_MAX_Y_MM) outside|=1U<<2U; /* South. */
  if(pose->x_mm==MAP_LASER_MIN_X_MM) outside|=1U<<3U; /* West. */
  for(i=0U;i<4U;i++) {
    uint8_t heading=(uint8_t)((pose->body_heading+offset[i])%4U);
    if(!(outside & (1U<<heading))) allowed|=(uint8_t)(1U<<i);
  }
  return allowed;
}

uint8_t MapLaserMask_FrameAllowed(uint8_t mask, uint8_t channel,
    uint32_t frame_tick, uint32_t enabled_since)
{
  return (channel<4U && (mask&(1U<<channel)) &&
      (int32_t)(frame_tick-enabled_since)>0L) ? 1U : 0U;
}
