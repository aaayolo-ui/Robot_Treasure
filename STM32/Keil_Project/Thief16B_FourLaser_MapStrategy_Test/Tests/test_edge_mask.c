#include "MapLaserMask.h"
#include "FrontLaserGate.h"

volatile unsigned test_checks, test_failures, test_done;
volatile unsigned test_first_line;
volatile unsigned test_node_poses, test_midpoint_poses;

static void checked(unsigned ok, unsigned line)
{
  test_checks++;
  if (!ok) {
    if (!test_failures) test_first_line = line;
    test_failures++;
  }
}
#define check(ok) checked((ok), __LINE__)

/* Independently tabulated sensor masks, ordered by body N/E/S/W.
 * Bits are front, left, right, rear. These tables avoid sharing the
 * production helper's world-heading rotation logic. */
static unsigned expected_mask(int32_t x, int32_t y, unsigned body)
{
  static const uint8_t left[]   = {13U, 7U, 11U, 14U};
  static const uint8_t right[]  = {11U, 14U, 13U, 7U};
  static const uint8_t top[]    = {14U, 13U, 7U, 11U};
  static const uint8_t bottom[] = {7U, 11U, 14U, 13U};
  unsigned mask = 15U;
  if (body > 3U || x < 0L || x > 3300L || y < 0L || y > 2700L)
    return 0U;
  if (x == 0L) mask &= left[body];
  if (x == 3300L) mask &= right[body];
  if (y == 0L) mask &= top[body];
  if (y == 2700L) mask &= bottom[body];
  return mask;
}

static void all_node_poses(void)
{
  unsigned i, body, reverse;
  for (i = 0U; i < MAP_ROUTE_NODE_COUNT; i++) {
    const MapRouteNode_t *node = &map_route_nodes[i];
    for (body = 0U; body < 4U; body++) {
      ThiefMapPose_t pose;
      ThiefMapPlanner_PoseAtNode(node->id, (uint8_t)body, &pose);
      check(pose.valid && pose.x_mm == node->x_half * 150L &&
            pose.y_mm == node->y_half * 150L);
      for (reverse = 0U; reverse < 2U; reverse++) {
        pose.drive_mode = (uint8_t)reverse;
        pose.travel_heading = (uint8_t)((body + 2U * reverse) % 4U);
        check(MapLaserMask_DetectionMask(&pose) ==
              expected_mask(node->x_half * 150L, node->y_half * 150L, body));
        test_node_poses++;
      }
    }
  }
}

static void all_segment_midpoints(void)
{
  unsigned i, body, reverse;
  for (i = 0U; i < MAP_ROUTE_SEGMENT_COUNT; i++) {
    const MapRouteSegment_t *segment = &map_route_segments[i];
    const MapRouteNode_t *a = ThiefMapPlanner_Node(segment->start);
    const MapRouteNode_t *b = ThiefMapPlanner_Node(segment->end);
    check(a != 0 && b != 0);
    if (a == 0 || b == 0) continue;
    for (body = 0U; body < 4U; body++) {
      ThiefMapPose_t pose;
      ThiefMapPlanner_PoseAtNode(a->id, (uint8_t)body, &pose);
      pose.x_mm = (int16_t)((a->x_half + b->x_half) * 75L);
      pose.y_mm = (int16_t)((a->y_half + b->y_half) * 75L);
      pose.segment_progress_mm = (uint16_t)(segment->length_mm / 2U);
      for (reverse = 0U; reverse < 2U; reverse++) {
        pose.drive_mode = (uint8_t)reverse;
        pose.travel_heading = (uint8_t)((body + 2U * reverse) % 4U);
        check(MapLaserMask_DetectionMask(&pose) ==
              expected_mask(pose.x_mm, pose.y_mm, body));
        test_midpoint_poses++;
      }
    }
  }
}

static void corners_and_invalid(void)
{
  static const int16_t corners[4][2] = {
    {0, 0}, {3300, 0}, {0, 2700}, {3300, 2700}
  };
  static const uint8_t masks[4][4] = {
    {12U, 5U, 3U, 10U}, {10U, 12U, 5U, 3U},
    {5U, 3U, 10U, 12U}, {3U, 10U, 12U, 5U}
  };
  static const int16_t outside[8][2] = {
    {-1, 1200}, {3301, 1200}, {1200, -1}, {1200, 2701},
    {-150, 1200}, {3450, 1200}, {1200, -150}, {1200, 2850}
  };
  unsigned i, body;
  ThiefMapPose_t pose;
  check(!MapLaserMask_DetectionMask(0));
  ThiefMapPlanner_PoseAtNode(5U, 3U, &pose);
  check(MapLaserMask_DetectionMask(&pose) == 0x0BU);
  ThiefMapPlanner_PoseAtNode(81U, 0U, &pose);
  check(MapLaserMask_DetectionMask(&pose) == 0x05U);
  for (i = 0U; i < 4U; i++) {
    pose.x_mm = corners[i][0]; pose.y_mm = corners[i][1];
    for (body = 0U; body < 4U; body++) {
      pose.body_heading = (uint8_t)body;
      check(MapLaserMask_DetectionMask(&pose) == masks[i][body]);
    }
  }
  for (i = 0U; i < 8U; i++) {
    pose.x_mm = outside[i][0]; pose.y_mm = outside[i][1];
    for (body = 0U; body < 4U; body++) {
      pose.body_heading = (uint8_t)body;
      check(!MapLaserMask_DetectionMask(&pose));
    }
  }
  pose.x_mm = 1500; pose.y_mm = 1200;
  for (body = 0U; body < 4U; body++) {
    pose.body_heading = (uint8_t)body;
    pose.valid = 0U;
    check(!MapLaserMask_DetectionMask(&pose));
    pose.valid = 1U;
    check(MapLaserMask_DetectionMask(&pose) == 0x0FU);
  }
  pose.body_heading = 4U;
  check(!MapLaserMask_DetectionMask(&pose));
  pose.body_heading = 255U;
  check(!MapLaserMask_DetectionMask(&pose));
  /* This interior coordinate is not a road/node. The edge mask must not
   * replace the separate road-projection filter in FrontLaserGate. */
  pose.node_id = 0U; pose.x_mm = 1000; pose.y_mm = 1000;
  pose.body_heading = 0U;
  check(MapLaserMask_DetectionMask(&pose) == 0x0FU);
  check(test_node_poses == MAP_ROUTE_NODE_COUNT * 8U);
  check(test_midpoint_poses == MAP_ROUTE_SEGMENT_COUNT * 8U);
}

static void history_restart(void)
{
  ThiefMapPose_t inside, edge;
  FrontLaserGate_t gate;
  unsigned i;
  ThiefMapPlanner_PoseAtNode(33U, 0U, &inside);
  ThiefMapPlanner_PoseAtNode(6U, 0U, &edge);
  check((MapLaserMask_DetectionMask(&inside) & 1U) != 0U);
  check((MapLaserMask_DetectionMask(&edge) & 1U) == 0U);
  FrontLaserGate_Init(&gate);
  /* One near frame must not survive a disable/re-enable transition and
   * combine with the first new frame into the two-frame fast trigger. */
  check(!FrontLaserGate_Observe(&gate, &inside, 1U, 300U, 20U, 123U));
  FrontLaserGate_ResetEvidence(&gate);
  check(!FrontLaserGate_Observe(&gate, &edge, 0U, 300U, 20U, 123U));
  check(gate.status == FRONT_LASER_NOT_HIDING);
  FrontLaserGate_ResetEvidence(&gate);
  check(!FrontLaserGate_Observe(&gate, &inside, 1U, 300U, 20U, 123U));
  check(FrontLaserGate_Observe(&gate, &inside, 1U, 300U, 20U, 123U));
  check(gate.status == FRONT_LASER_OPPONENT);
  /* Also prove an already confirmed opponent cannot survive masking.
   * Restored ordinary-quality frames must build five fresh observations. */
  FrontLaserGate_ResetEvidence(&gate);
  check(!FrontLaserGate_Observe(&gate, &edge, 0U, 650U, 10U, 123U));
  check(gate.status != FRONT_LASER_OPPONENT);
  FrontLaserGate_ResetEvidence(&gate);
  check(!gate.valid_history && !gate.history_count && !gate.near_previous);
  for (i = 0U; i < 4U; i++)
    check(!FrontLaserGate_Observe(&gate, &inside, 1U, 650U, 10U, 123U));
  check(FrontLaserGate_Observe(&gate, &inside, 1U, 650U, 10U, 123U));
}

static void frame_epoch(void)
{
  unsigned channel, mask;
  for (channel = 0U; channel < 4U; channel++) {
    for (mask = 0U; mask < 16U; mask++) {
      /* Older/same-ms frames must not contribute even when this sensor is
       * enabled; only a strictly later frame can start fresh evidence. */
      check(!MapLaserMask_FrameAllowed((uint8_t)mask, (uint8_t)channel,
                                      99UL, 100UL));
      check(!MapLaserMask_FrameAllowed((uint8_t)mask, (uint8_t)channel,
                                      100UL, 100UL));
      check(MapLaserMask_FrameAllowed((uint8_t)mask, (uint8_t)channel,
                                     101UL, 100UL) ==
            ((mask & (1U << channel)) != 0U));
    }
    check(MapLaserMask_FrameAllowed(15U, (uint8_t)channel,
                                   1UL, 0xFFFFFFFEUL));
    check(!MapLaserMask_FrameAllowed(15U, (uint8_t)channel,
                                    0xFFFFFFFEUL, 2UL));
    check(!MapLaserMask_FrameAllowed(0U, (uint8_t)channel,
                                    1UL, 0xFFFFFFFEUL));
  }
  check(!MapLaserMask_FrameAllowed(15U, 4U, 101UL, 100UL));
  check(!MapLaserMask_FrameAllowed(255U, 255U, 101UL, 100UL));
  check(!MapLaserMask_FrameAllowed(0U, 0U, 101UL, 100UL));
}

void test_main(void)
{
  all_node_poses();
  all_segment_midpoints();
  corners_and_invalid();
  history_restart();
  frame_epoch();
  test_done = 1U;
  while (1) {}
}
