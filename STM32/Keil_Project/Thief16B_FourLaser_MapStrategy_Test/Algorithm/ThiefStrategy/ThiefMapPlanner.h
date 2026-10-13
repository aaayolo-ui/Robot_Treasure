#ifndef THIEF_MAP_PLANNER_H
#define THIEF_MAP_PLANNER_H

#include "MapRoutePlan.h"
#include <stdint.h>

#define THIEF_ROUTE_MAX_STEPS 48U
#define THIEF_START_NODE 54U
#define THIEF_TREASURE_NODE 5U
#define THIEF_FIRST_HIDE_NODE 59U
#define THIEF_START_BODY_HEADING 3U

typedef struct {
  MapRoute_t route;
  MapRouteStep_t steps[THIEF_ROUTE_MAX_STEPS];
  uint16_t goal_node;
  uint8_t final_body_heading;
  uint8_t valid;
} ThiefRoutePlan_t;

typedef struct {
  int16_t x_mm;
  int16_t y_mm;
  uint16_t node_id;
  uint16_t segment_progress_mm;
  uint8_t travel_heading;
  uint8_t body_heading;
  uint8_t drive_mode;
  uint8_t valid;
} ThiefMapPose_t;

uint8_t ThiefMapPlanner_PlanInitial(ThiefRoutePlan_t *plan);
uint8_t ThiefMapPlanner_PlanEscape(uint16_t current_node,
                                   uint8_t body_heading,
                                   uint8_t threat_heading_mask,
                                   uint16_t previous_hide_node,
                                   ThiefRoutePlan_t *plan);
void ThiefMapPlanner_PoseAtNode(uint16_t node_id, uint8_t body_heading,
                                ThiefMapPose_t *pose);
void ThiefMapPlanner_PoseOnStep(const MapRouteStep_t *step,
                                int32_t distance_mm,
                                ThiefMapPose_t *pose);
const MapRouteNode_t *ThiefMapPlanner_Node(uint16_t node_id);
uint8_t ThiefMapPlanner_HasRoad(uint16_t node_id, uint8_t heading,
                                uint16_t *next_node, uint16_t *length_mm);

#endif

