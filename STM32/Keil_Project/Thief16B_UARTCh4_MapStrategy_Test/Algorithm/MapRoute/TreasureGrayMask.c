#include "TreasureGrayMask.h"
#include <stddef.h>

uint16_t TreasureGrayMask_Apply(const MapRouteStep_t *step, uint8_t enabled,
                               uint8_t leading, uint16_t gray12)
{
  uint16_t mask;
  if ((enabled == 0U) || (step == NULL)) return gray12;
  if ((step->from_node == 4U) && (step->to_node == 6U) &&
      (step->heading == 1U)) {
    /* East: north is travel-left. */
    mask = (leading != 0U) ? 0x0C00U : 0x0003U;
  } else if ((step->from_node == 6U) && (step->to_node == 4U) &&
             (step->heading == 3U)) {
    /* West: north is travel-right. */
    mask = (leading != 0U) ? 0x0003U : 0x0C00U;
  } else {
    return gray12;
  }
  return (uint16_t)(gray12 & (uint16_t)~mask);
}
