#ifndef TREASURE_GRAY_MASK_H
#define TREASURE_GRAY_MASK_H
#include "MapRoutePlan.h"

/* Logical leading board: P12 is travel-left. Trailing board: P1 is travel-left.
 * TravelFrame already swaps physical boards in reverse mode. Text is north. */
uint16_t TreasureGrayMask_Apply(const MapRouteStep_t *step, uint8_t enabled,
                               uint8_t leading, uint16_t gray12);
/* Turn at a border node: heading is the measured current travel quadrant. */
uint16_t TreasureGrayMask_AtNode(uint16_t node,uint8_t heading,
                                 uint8_t leading,uint16_t gray12);
#endif
