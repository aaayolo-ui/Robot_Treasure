#include "TreasureLaserStart.h"
#include "ThiefMapPlanner.h"
#include <stddef.h>

uint8_t TreasureLaserStart_Crossed(const MapRouteStep_t *step, int32_t progress_mm)
{
  const MapRouteNode_t *from, *treasure, *to;
  int32_t offset, length;
  if (step == NULL || step->from_node != 6U || step->to_node != 4U ||
      step->heading != 3U || step->drive_mode != 0U) return 0U;
  from = ThiefMapPlanner_Node(6U);
  treasure = ThiefMapPlanner_Node(5U);
  to = ThiefMapPlanner_Node(4U);
  if (from == NULL || treasure == NULL || to == NULL ||
      from->y_half != treasure->y_half || to->y_half != treasure->y_half)
    return 0U;
  offset = ((int32_t)from->x_half - treasure->x_half) * 150L;
  length = ((int32_t)from->x_half - to->x_half) * 150L;
  return (offset > 0L && offset < length && length == step->length_mm &&
          progress_mm >= offset && progress_mm <= length) ? 1U : 0U;
}
