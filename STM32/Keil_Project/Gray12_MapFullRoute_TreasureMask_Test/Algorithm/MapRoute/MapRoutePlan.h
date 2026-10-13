#ifndef MAP_ROUTE_PLAN_H
#define MAP_ROUTE_PLAN_H
#include <stdint.h>
typedef enum { MAP_ACTION_LEFT=-1, MAP_ACTION_STRAIGHT=0, MAP_ACTION_RIGHT=1,
               MAP_ACTION_STOP=2, MAP_ACTION_ERROR=3, MAP_ACTION_REVERSE=4 } MapRouteAction_t;
typedef struct { uint16_t id; int16_t x_half, y_half; uint8_t kind, degree; } MapRouteNode_t;
typedef struct { uint16_t id, start, end, length_mm; uint8_t heading; } MapRouteSegment_t;
typedef struct { uint16_t from_node, to_node, length_mm, segment_offset, segment_count;
                 uint8_t heading, expected_ports, target_kind; int8_t action; uint8_t drive_mode; } MapRouteStep_t;
typedef struct { uint16_t id, offset, count, start_node; uint8_t heading; } MapRoute_t;
extern const MapRouteNode_t map_route_nodes[];
extern const MapRouteSegment_t map_route_segments[];
extern const uint16_t map_route_step_segments[];
extern const MapRouteStep_t map_route_steps[];
extern const MapRoute_t map_routes[];
extern const uint16_t map_route_count;
extern const uint32_t map_route_signature;
#define MAP_ROUTE_NODE_COUNT 88U
#define MAP_ROUTE_SEGMENT_COUNT 106U
#endif
