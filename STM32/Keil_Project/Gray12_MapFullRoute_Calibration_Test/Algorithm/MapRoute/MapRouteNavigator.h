#ifndef MAP_ROUTE_NAVIGATOR_H
#define MAP_ROUTE_NAVIGATOR_H
#include "MapRoutePlan.h"
typedef struct {
  const MapRoute_t *route;
  uint16_t step;
  uint8_t arrived, seen_ports, fault;
} MapRouteNavigator_t;
void MapRouteNavigator_Start(MapRouteNavigator_t *nav, uint16_t route_index);
const MapRouteStep_t *MapRouteNavigator_Current(const MapRouteNavigator_t *nav);
MapRouteAction_t MapRouteNavigator_Arrive(MapRouteNavigator_t *nav,
                                         uint16_t raw12, int32_t travel_mm);
void MapRouteNavigator_Observe(MapRouteNavigator_t *nav, uint16_t raw12);
uint8_t MapRouteNavigator_ConfirmTurn(const MapRouteNavigator_t *nav);
uint8_t MapRouteNavigator_Advance(MapRouteNavigator_t *nav);
uint8_t MapRouteNavigator_TooFar(const MapRouteNavigator_t *nav, int32_t travel_mm);
#endif
