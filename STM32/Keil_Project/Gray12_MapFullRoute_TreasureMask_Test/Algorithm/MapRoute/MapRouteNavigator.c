#include "MapRouteNavigator.h"
#include <stddef.h>

static const MapRouteNode_t *MapRouteNavigator_Node(uint16_t id)
{
  uint16_t i;
  for (i = 0U; i < MAP_ROUTE_NODE_COUNT; i++)
    if (map_route_nodes[i].id == id) return &map_route_nodes[i];
  return NULL;
}

static const MapRouteSegment_t *MapRouteNavigator_Segment(uint16_t id)
{
  uint16_t i;
  for (i = 0U; i < MAP_ROUTE_SEGMENT_COUNT; i++)
    if (map_route_segments[i].id == id) return &map_route_segments[i];
  return NULL;
}

static uint8_t MapRouteNavigator_Validate(const MapRoute_t *route)
{
  uint16_t i, j;
  if ((route == NULL) || (route->count == 0U)) return 0U;
  for (i = 0U; i < route->count; i++) {
    const MapRouteStep_t *step = &map_route_steps[route->offset + i];
    const MapRouteNode_t *from = MapRouteNavigator_Node(step->from_node);
    const MapRouteNode_t *to = MapRouteNavigator_Node(step->to_node);
    uint16_t cursor = step->from_node;
    uint32_t length = 0U;
    if ((from == NULL) || (to == NULL) || (from->kind == 0U) || (to->kind == 0U) ||
        (to->kind != step->target_kind) || (step->segment_count == 0U) || (step->drive_mode > 1U)) return 0U;
    if ((i == 0U) && ((step->drive_mode != 0U) || (route->start_node != step->from_node) || (route->heading != step->heading))) return 0U;
    for (j = 0U; j < step->segment_count; j++) {
      const MapRouteSegment_t *segment = MapRouteNavigator_Segment(map_route_step_segments[step->segment_offset + j]);
      uint8_t heading;
      if (segment == NULL) return 0U;
      if (cursor == segment->start) {
        cursor = segment->end;
        heading = segment->heading;
      } else if (cursor == segment->end) {
        cursor = segment->start;
        heading = (uint8_t)((segment->heading + 2U) % 4U);
      } else return 0U;
      if (heading != step->heading) return 0U;
      length += segment->length_mm;
    }
    if ((cursor != step->to_node) || (length != step->length_mm)) return 0U;
    if (i + 1U < route->count) {
      const MapRouteStep_t *next = step + 1;
      uint8_t difference = (uint8_t)((next->heading + 4U - step->heading) % 4U);
      int8_t expected = (difference == 3U) ? -1 :
                        (difference == 2U) ? MAP_ACTION_REVERSE : (int8_t)difference;
      if ((next->from_node != step->to_node) || (next->drive_mode != (step->drive_mode ^ ((difference == 2U) ? 1U : 0U))) ||
          (step->action != expected)) return 0U;
    } else if (step->action != MAP_ACTION_STOP) return 0U;
  }
  return 1U;
}

void MapRouteNavigator_Start(MapRouteNavigator_t *nav, uint16_t route_index)
{
  nav->route = (route_index < map_route_count) ? &map_routes[route_index] : NULL;
  nav->step = 0U;
  nav->arrived = 0U;
  nav->seen_ports = 0U;
  nav->fault = (MapRouteNavigator_Validate(nav->route) == 0U) ? 1U : 0U;
}

const MapRouteStep_t *MapRouteNavigator_Current(const MapRouteNavigator_t *nav)
{
  if ((nav->route == NULL) || (nav->step >= nav->route->count)) return NULL;
  return &map_route_steps[nav->route->offset + nav->step];
}

void MapRouteNavigator_Observe(MapRouteNavigator_t *nav, uint16_t raw12)
{
  if ((raw12 & 0x0C00U) == 0x0C00U) nav->seen_ports |= 1U;
  if ((raw12 & 0x0003U) == 0x0003U) nav->seen_ports |= 2U;
}

uint8_t MapRouteNavigator_TooFar(const MapRouteNavigator_t *nav, int32_t travel_mm)
{
  const MapRouteStep_t *step = MapRouteNavigator_Current(nav);
  if ((step == NULL) || (nav->arrived != 0U)) return 0U;
  return (travel_mm > ((int32_t)step->length_mm * 16L / 10L + 100L)) ? 1U : 0U;
}

MapRouteAction_t MapRouteNavigator_Arrive(MapRouteNavigator_t *nav,
                                         uint16_t raw12, int32_t travel_mm)
{
  const MapRouteStep_t *step = MapRouteNavigator_Current(nav);
  int32_t minimum;
  if ((step == NULL) || (nav->fault != 0U) || (nav->arrived != 0U)) return MAP_ACTION_ERROR;
  minimum = (int32_t)step->length_mm * 4L / 10L - 50L;
  if ((travel_mm < minimum) || (MapRouteNavigator_TooFar(nav, travel_mm) != 0U)) {
    nav->fault = 1U;
    return MAP_ACTION_ERROR;
  }
  nav->arrived = 1U;
  nav->seen_ports = 0U;
  MapRouteNavigator_Observe(nav, raw12);
  return (MapRouteAction_t)step->action;
}

uint8_t MapRouteNavigator_ConfirmTurn(const MapRouteNavigator_t *nav)
{
  const MapRouteStep_t *step = MapRouteNavigator_Current(nav);
  if ((step == NULL) || (nav->arrived == 0U)) return 0U;
  if (step->action == MAP_ACTION_LEFT) return ((nav->seen_ports & 1U) != 0U) ? 1U : 0U;
  if (step->action == MAP_ACTION_RIGHT) return ((nav->seen_ports & 2U) != 0U) ? 1U : 0U;
  return 1U;
}

uint8_t MapRouteNavigator_Advance(MapRouteNavigator_t *nav)
{
  const MapRouteStep_t *step = MapRouteNavigator_Current(nav);
  if ((step == NULL) || (nav->arrived == 0U) || (nav->fault != 0U) ||
      (step->action == MAP_ACTION_STOP) || (nav->step + 1U >= nav->route->count)) return 0U;
  nav->step++;
  nav->arrived = 0U;
  nav->seen_ports = 0U;
  return 1U;
}
