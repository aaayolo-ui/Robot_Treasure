#include "ThiefMapPlanner.h"
#include <stddef.h>

#define DIST_INF 0x3FFFFFFFUL
#define ESCAPE_ID 1601U
#define INITIAL_ID 1600U
#define TARGET_ESCAPE_MM 1500UL

static const uint16_t safe_hide_nodes[] = {
  11U, 14U, 15U, 22U, 23U, 25U, 32U, 33U, 34U,
  35U, 38U, 39U, 46U, 49U, 52U, 53U, 58U, 59U, 61U,
  62U, 63U, 67U, 68U, 70U, 76U, 77U, 78U, 79U
};

/* The project stack is 1 KiB. Keep Dijkstra and path scratch buffers in
 * module RAM so a replan cannot overflow the call stack. The planner is used
 * synchronously from the main loop and is intentionally non-reentrant. */
static uint32_t work_distance[MAP_ROUTE_NODE_COUNT];
static uint16_t work_previous[MAP_ROUTE_NODE_COUNT];
static uint16_t work_reverse_path[MAP_ROUTE_NODE_COUNT];
static uint16_t work_path_a[MAP_ROUTE_NODE_COUNT];
static uint16_t work_path_b[MAP_ROUTE_NODE_COUNT];
static uint16_t work_path_c[MAP_ROUTE_NODE_COUNT];
static uint8_t work_visited[MAP_ROUTE_NODE_COUNT];

const MapRouteNode_t *ThiefMapPlanner_Node(uint16_t node_id)
{
  if ((node_id == 0U) || (node_id > MAP_ROUTE_NODE_COUNT)) return NULL;
  if (map_route_nodes[node_id - 1U].id != node_id) return NULL;
  return &map_route_nodes[node_id - 1U];
}

uint8_t ThiefMapPlanner_HasRoad(uint16_t node_id, uint8_t heading,
                                uint16_t *next_node, uint16_t *length_mm)
{
  uint16_t i;
  for (i = 0U; i < MAP_ROUTE_SEGMENT_COUNT; i++) {
    const MapRouteSegment_t *segment = &map_route_segments[i];
    uint8_t out_heading;
    uint16_t other;
    if (segment->start == node_id) {
      out_heading = segment->heading;
      other = segment->end;
    } else if (segment->end == node_id) {
      out_heading = (uint8_t)((segment->heading + 2U) % 4U);
      other = segment->start;
    } else continue;
    if (out_heading == heading) {
      if (next_node != NULL) *next_node = other;
      if (length_mm != NULL) *length_mm = segment->length_mm;
      return 1U;
    }
  }
  return 0U;
}

static uint8_t SegmentBetween(uint16_t from, uint16_t to,
                              uint8_t *heading, uint16_t *length)
{
  uint16_t i;
  for (i = 0U; i < MAP_ROUTE_SEGMENT_COUNT; i++) {
    const MapRouteSegment_t *segment = &map_route_segments[i];
    if ((segment->start == from) && (segment->end == to)) {
      *heading = segment->heading;
      *length = segment->length_mm;
      return 1U;
    }
    if ((segment->end == from) && (segment->start == to)) {
      *heading = (uint8_t)((segment->heading + 2U) % 4U);
      *length = segment->length_mm;
      return 1U;
    }
  }
  return 0U;
}

static uint8_t ShortestPath(uint16_t start, uint16_t goal, uint16_t blocked,
                            uint16_t *path, uint16_t *path_count,
                            uint32_t *total_length)
{
  uint16_t i, cursor, count = 0U;

  if ((ThiefMapPlanner_Node(start) == NULL) ||
      (ThiefMapPlanner_Node(goal) == NULL) || (start == blocked)) return 0U;
  for (i = 0U; i < MAP_ROUTE_NODE_COUNT; i++) {
    work_distance[i] = DIST_INF;
    work_previous[i] = 0U;
    work_visited[i] = 0U;
  }
  work_distance[start - 1U] = 0U;

  for (i = 0U; i < MAP_ROUTE_NODE_COUNT; i++) {
    uint16_t pick = 0U;
    uint32_t best = DIST_INF;
    uint16_t s;
    for (s = 1U; s <= MAP_ROUTE_NODE_COUNT; s++) {
      if ((work_visited[s - 1U] == 0U) &&
          (work_distance[s - 1U] < best)) {
        best = work_distance[s - 1U];
        pick = s;
      }
    }
    if ((pick == 0U) || (pick == goal)) break;
    work_visited[pick - 1U] = 1U;
    for (s = 0U; s < MAP_ROUTE_SEGMENT_COUNT; s++) {
      const MapRouteSegment_t *segment = &map_route_segments[s];
      uint16_t next = 0U;
      uint32_t candidate;
      if (segment->start == pick) next = segment->end;
      else if (segment->end == pick) next = segment->start;
      if ((next == 0U) || (next == blocked)) continue;
      candidate = work_distance[pick - 1U] + segment->length_mm;
      if (candidate < work_distance[next - 1U]) {
        work_distance[next - 1U] = candidate;
        work_previous[next - 1U] = pick;
      }
    }
  }

  if (work_distance[goal - 1U] == DIST_INF) return 0U;
  cursor = goal;
  while ((cursor != 0U) && (count < MAP_ROUTE_NODE_COUNT)) {
    work_reverse_path[count++] = cursor;
    if (cursor == start) break;
    cursor = work_previous[cursor - 1U];
  }
  if ((count == 0U) || (work_reverse_path[count - 1U] != start)) return 0U;
  *path_count = count;
  for (i = 0U; i < count; i++) path[i] = work_reverse_path[count - 1U - i];
  if (total_length != NULL) *total_length = work_distance[goal - 1U];
  return 1U;
}

static uint8_t BuildRoute(const uint16_t *nodes, uint16_t node_count,
                          uint8_t start_drive_mode, uint16_t route_id,
                          ThiefRoutePlan_t *plan)
{
  uint16_t i = 0U;
  uint16_t step_count = 0U;
  uint8_t drive_mode = start_drive_mode;
  if ((plan == NULL) || (node_count < 2U)) return 0U;

  while (i + 1U < node_count) {
    uint16_t j = (uint16_t)(i + 1U);
    uint16_t length = 0U;
    uint16_t segment_count = 0U;
    uint8_t heading, part_heading;
    uint16_t part_length;
    if (SegmentBetween(nodes[i], nodes[j], &heading, &part_length) == 0U) return 0U;
    length = part_length;
    segment_count = 1U;
    while ((j + 1U < node_count) &&
           (ThiefMapPlanner_Node(nodes[j])->kind == 0U)) {
      if (SegmentBetween(nodes[j], nodes[j + 1U], &part_heading, &part_length) == 0U)
        return 0U;
      if (part_heading != heading) return 0U;
      length = (uint16_t)(length + part_length);
      segment_count++;
      j++;
    }
    if ((ThiefMapPlanner_Node(nodes[j])->kind == 0U) ||
        (step_count >= THIEF_ROUTE_MAX_STEPS)) return 0U;
    plan->steps[step_count].from_node = nodes[i];
    plan->steps[step_count].to_node = nodes[j];
    plan->steps[step_count].length_mm = length;
    plan->steps[step_count].segment_offset = 0U;
    plan->steps[step_count].segment_count = segment_count;
    plan->steps[step_count].heading = heading;
    plan->steps[step_count].expected_ports = ThiefMapPlanner_Node(nodes[j])->degree;
    plan->steps[step_count].target_kind = ThiefMapPlanner_Node(nodes[j])->kind;
    plan->steps[step_count].drive_mode = drive_mode;
    plan->steps[step_count].action = MAP_ACTION_STOP;
    step_count++;
    i = j;
  }

  for (i = 0U; i + 1U < step_count; i++) {
    uint8_t difference = (uint8_t)((plan->steps[i + 1U].heading + 4U -
                                    plan->steps[i].heading) % 4U);
    if (difference == 3U) plan->steps[i].action = MAP_ACTION_LEFT;
    else if (difference == 2U) {
      plan->steps[i].action = MAP_ACTION_REVERSE;
      drive_mode ^= 1U;
    } else plan->steps[i].action = (int8_t)difference;
    plan->steps[i + 1U].drive_mode = drive_mode;
  }

  plan->route.id = route_id;
  plan->route.offset = 0U;
  plan->route.count = step_count;
  plan->route.start_node = nodes[0];
  plan->route.heading = plan->steps[0].heading;
  plan->route.drive_mode = start_drive_mode;
  plan->goal_node = nodes[node_count - 1U];
  plan->final_body_heading = (uint8_t)((plan->steps[step_count - 1U].heading +
      (plan->steps[step_count - 1U].drive_mode != 0U ? 2U : 0U)) % 4U);
  plan->valid = 1U;
  return 1U;
}

/* User-selected outbound route. Keep every observable intermediate junction.
 * W from N054; right at N053, left at N036, right at N033,
 * left at N006. N005 is crossed without stopping toward N004. */
static const uint16_t initial_treasure_path[] = {
  54U, 53U, 44U, 36U, 35U, 34U, 33U, 25U, 15U, 6U, 5U, 4U
};

uint8_t ThiefMapPlanner_PlanInitial(ThiefRoutePlan_t *plan)
{
  uint16_t *departure = work_path_b;
  uint16_t *combined = work_path_c;
  uint16_t departure_count, i, count;
  const uint16_t prefix_count = (uint16_t)(sizeof(initial_treasure_path) /
                                         sizeof(initial_treasure_path[0]));
  if (plan == NULL) return 0U;
  plan->valid = 0U;
  if (initial_treasure_path[0] != THIEF_START_NODE) return 0U;
  if (ShortestPath(4U, THIEF_FIRST_HIDE_NODE, THIEF_TREASURE_NODE,
                   departure, &departure_count, NULL) == 0U) return 0U;
  if ((uint32_t)prefix_count + (uint32_t)departure_count - 1UL >
      MAP_ROUTE_NODE_COUNT) return 0U;
  count = 0U;
  for (i = 0U; i < prefix_count; i++) combined[count++] = initial_treasure_path[i];
  for (i = 1U; i < departure_count; i++) combined[count++] = departure[i];
  return BuildRoute(combined, count, 0U, INITIAL_ID, plan);
}

uint8_t ThiefMapPlanner_PlanEscape(uint16_t current_node,
                                   uint8_t body_heading,
                                   uint16_t previous_hide_node,
                                   ThiefRoutePlan_t *plan)
{
  uint16_t forced_next, first_length;
  uint16_t *candidate_path = work_path_a;
  uint16_t *best_path = work_path_b;
  uint16_t count, best_count = 0U;
  uint32_t length, best_score = DIST_INF;
  uint16_t i;
  uint8_t away_heading = (uint8_t)((body_heading + 2U) % 4U);
  if ((plan == NULL) ||
      (ThiefMapPlanner_HasRoad(current_node, away_heading,
                               &forced_next, &first_length) == 0U)) return 0U;
  plan->valid = 0U;

  for (i = 0U; i < (uint16_t)(sizeof(safe_hide_nodes) / sizeof(safe_hide_nodes[0])); i++) {
    uint16_t goal = safe_hide_nodes[i];
    uint8_t arrival_heading;
    uint16_t dummy_length;
    uint32_t score;
    if ((goal == current_node) || (goal == previous_hide_node)) continue;
    if (ShortestPath(forced_next, goal, current_node,
                     &candidate_path[1], &count, &length) == 0U) continue;
    candidate_path[0] = current_node;
    count++;
    length += first_length;
    if ((count < 3U) || (length < 600UL)) continue;
    if (SegmentBetween(candidate_path[count - 2U], goal,
                       &arrival_heading, &dummy_length) == 0U) continue;
    if (ThiefMapPlanner_HasRoad(goal, arrival_heading, NULL, NULL) == 0U) continue;
    score = (length > TARGET_ESCAPE_MM) ?
            (length - TARGET_ESCAPE_MM) : (TARGET_ESCAPE_MM - length);
    if (score < best_score) {
      uint16_t j;
      best_score = score;
      best_count = count;
      for (j = 0U; j < count; j++) best_path[j] = candidate_path[j];
    }
  }
  if (best_count == 0U) return 0U;
  return BuildRoute(best_path, best_count, 1U, ESCAPE_ID, plan);
}

void ThiefMapPlanner_PoseAtNode(uint16_t node_id, uint8_t body_heading,
                                ThiefMapPose_t *pose)
{
  const MapRouteNode_t *node = ThiefMapPlanner_Node(node_id);
  if (pose == NULL) return;
  pose->valid = 0U;
  if (node == NULL) return;
  pose->x_mm = (int16_t)(node->x_half * 150);
  pose->y_mm = (int16_t)(node->y_half * 150);
  pose->node_id = node_id;
  pose->segment_progress_mm = 0U;
  pose->travel_heading = body_heading;
  pose->body_heading = body_heading;
  pose->drive_mode = 0U;
  pose->valid = 1U;
}

void ThiefMapPlanner_PoseOnStep(const MapRouteStep_t *step,
                                int32_t distance_mm,
                                ThiefMapPose_t *pose)
{
  const MapRouteNode_t *from;
  int32_t clamped;
  if ((pose == NULL) || (step == NULL)) return;
  pose->valid = 0U;
  from = ThiefMapPlanner_Node(step->from_node);
  if (from == NULL) return;
  clamped = distance_mm;
  if (clamped < 0L) clamped = 0L;
  if (clamped > step->length_mm) clamped = step->length_mm;
  pose->x_mm = (int16_t)(from->x_half * 150);
  pose->y_mm = (int16_t)(from->y_half * 150);
  if (step->heading == 0U) pose->y_mm = (int16_t)(pose->y_mm - clamped);
  else if (step->heading == 1U) pose->x_mm = (int16_t)(pose->x_mm + clamped);
  else if (step->heading == 2U) pose->y_mm = (int16_t)(pose->y_mm + clamped);
  else pose->x_mm = (int16_t)(pose->x_mm - clamped);
  pose->node_id = (clamped >= step->length_mm) ?
                  step->to_node : step->from_node;
  pose->segment_progress_mm = (uint16_t)clamped;
  pose->travel_heading = step->heading;
  pose->drive_mode = step->drive_mode;
  pose->body_heading = (uint8_t)((step->heading +
      (step->drive_mode != 0U ? 2U : 0U)) % 4U);
  pose->valid = 1U;
}

