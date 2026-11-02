#ifndef TREASURE_LASER_START_H
#define TREASURE_LASER_START_H

#include "MapRoutePlan.h"
#include <stdint.h>

/* N005 is a map marker within the compressed N006->N004 leg, not a gray
 * junction or a physical treasure-contact sensor. Units are nominal mm. */
uint8_t TreasureLaserStart_Crossed(const MapRouteStep_t *step, int32_t progress_mm);

#endif
