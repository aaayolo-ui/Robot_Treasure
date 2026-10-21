#include "TreasureLaserStart.h"
#include "FrontLaserGate.h"
#include "ThiefStrategyConfig.h"

volatile unsigned test_checks, test_failures, test_done;
volatile unsigned test_first_line;

static void checked(unsigned ok, unsigned line)
{
  test_checks++;
  if (!ok) {
    if (!test_failures) test_first_line = line;
    test_failures++;
  }
}
#define check(ok) checked((ok), __LINE__)

/* Confidence 10 uses the ordinary five-frame confirmation path. */
static unsigned stable(FrontLaserGate_t *gate, const ThiefMapPose_t *pose,
                       unsigned enabled, unsigned distance,
                       unsigned confidence)
{
  unsigned i, accepted = 0U;
  FrontLaserGate_Init(gate);
  for (i = 0U; i < 5U; i++) {
    accepted = FrontLaserGate_Observe(gate, pose, (uint8_t)enabled,
        (uint16_t)distance, (uint8_t)confidence, 123U);
    if (confidence == 10U && i < 4U) check(!accepted);
  }
  return accepted;
}

static void every_moving_road(void)
{
  unsigned segment_index, fraction, reverse;
  for (segment_index = 0U; segment_index < MAP_ROUTE_SEGMENT_COUNT;
       segment_index++) {
    const MapRouteSegment_t *segment = &map_route_segments[segment_index];
    const MapRouteNode_t *a = ThiefMapPlanner_Node(segment->start);
    const MapRouteNode_t *b = ThiefMapPlanner_Node(segment->end);
    MapRouteStep_t leg;
    check(a != 0 && b != 0 && segment->length_mm >= 150U);
    if (a == 0 || b == 0 || segment->length_mm < 150U) continue;
    leg.from_node = segment->start;
    leg.to_node = segment->end;
    leg.length_mm = segment->length_mm;
    leg.segment_offset = 0U;
    leg.segment_count = 1U;
    leg.heading = segment->heading;
    leg.expected_ports = b->degree;
    leg.target_kind = b->kind;
    leg.action = MAP_ACTION_STOP;
    for (fraction = 1U; fraction <= 3U; fraction++) {
      int32_t progress = (int32_t)segment->length_mm * fraction / 4L;
      int32_t ax = a->x_half * 150L, ay = a->y_half * 150L;
      int32_t dx = (b->x_half - a->x_half) * 150L;
      int32_t dy = (b->y_half - a->y_half) * 150L;
      int32_t expected_x, expected_y;
      /* Quarter positions on 150 mm pieces are only 37 mm from an end.
       * Keep at least 50 mm on each side so both rays stay on this known
       * segment, rather than depending on a neighboring junction exit. */
      if (progress < 50L) progress = 50L;
      if (progress > (int32_t)segment->length_mm - 50L)
        progress = (int32_t)segment->length_mm - 50L;
      expected_x = ax + dx * progress / segment->length_mm;
      expected_y = ay + dy * progress / segment->length_mm;
      for (reverse = 0U; reverse <= 1U; reverse++) {
        FrontLaserGate_t gate;
        ThiefMapPose_t moving;
        int32_t sign = reverse ? -1L : 1L;
        leg.drive_mode = (uint8_t)reverse;
        ThiefMapPlanner_PoseOnStep(&leg, progress, &moving);
        check(moving.valid);
        check(moving.x_mm == expected_x && moving.y_mm == expected_y);
        check(moving.body_heading == (segment->heading + 2U * reverse) % 4U);
        check(stable(&gate, &moving, 1U, 50U, 10U));
        /* Endpoint interpolation is independent of the ray traversal code. */
        check(gate.hit_x_mm == expected_x + sign * dx * 50L / segment->length_mm &&
              gate.hit_y_mm == expected_y + sign * dy * 50L / segment->length_mm);
      }
    }
  }
}

void test_main(void)
{
  FrontLaserGate_t gate;
  ThiefMapPose_t pose;
  MapRouteStep_t step = {6U, 4U, 300U, 0U, 2U, 3U, 3U, 2U,
                        MAP_ACTION_LEFT, 0U};
  unsigned i;

  check(THIEF_LASER_MAX_MM == 650U);
  check(!TreasureLaserStart_Crossed(0, 150L));
  check(!TreasureLaserStart_Crossed(&step, -1L));
  check(!TreasureLaserStart_Crossed(&step, 0L));
  check(!TreasureLaserStart_Crossed(&step, 149L));
  check(TreasureLaserStart_Crossed(&step, 150L));
  check(TreasureLaserStart_Crossed(&step, 175L));
  check(TreasureLaserStart_Crossed(&step, 300L));
  check(!TreasureLaserStart_Crossed(&step, 301L));
  check(!TreasureLaserStart_Crossed(&step, 100000L));
  step.length_mm = 299U;
  check(!TreasureLaserStart_Crossed(&step, 150L));
  step.length_mm = 301U;
  check(!TreasureLaserStart_Crossed(&step, 150L));
  step.length_mm = 300U;
  step.drive_mode = 1U;
  check(!TreasureLaserStart_Crossed(&step, 150L));
  check(!TreasureLaserStart_Crossed(&step, 300L));
  step.drive_mode = 0U;
  step.heading = 1U;
  check(!TreasureLaserStart_Crossed(&step, 150L));
  step.heading = 3U;
  step.from_node = 5U;
  check(!TreasureLaserStart_Crossed(&step, 150L));
  step.from_node = 6U;
  step.to_node = 3U;
  check(!TreasureLaserStart_Crossed(&step, 150L));
  step.to_node = 4U;

  /* The 650 mm range boundary needs a road longer than 650 mm. */
  ThiefMapPlanner_PoseAtNode(33U, 0U, &pose);
  check(stable(&gate, &pose, 1U, 650U, 10U));
  check(gate.status == FRONT_LASER_OPPONENT);
  check(gate.hit_x_mm == 1800 && gate.hit_y_mm == 250);
  check(!stable(&gate, &pose, 1U, 651U, 10U));
  check(gate.status == FRONT_LASER_TOO_FAR);
  check(!stable(&gate, &pose, 1U, 650U, 9U));
  check(gate.status == FRONT_LASER_BAD_QUALITY);
  check(!stable(&gate, &pose, 1U, 49U, 20U));
  check(gate.status == FRONT_LASER_BAD_QUALITY);
  check(!stable(&gate, &pose, 0U, 650U, 10U));
  check(gate.status == FRONT_LASER_NOT_HIDING);
  pose.valid = 0U;
  check(!stable(&gate, &pose, 1U, 650U, 10U));
  check(gate.status == FRONT_LASER_NOT_HIDING);

  /* N038's north road ends at N021 after 600 mm; range alone is insufficient. */
  ThiefMapPlanner_PoseAtNode(38U, 0U, &pose);
  check(stable(&gate, &pose, 1U, 600U, 10U));
  check(gate.hit_x_mm == 300 && gate.hit_y_mm == 600);
  check(!stable(&gate, &pose, 1U, 650U, 10U));
  check(gate.status == FRONT_LASER_NO_MAP_ROAD);

  /* Moving origin is 175 mm west of N006, not the last confirmed node. */
  ThiefMapPlanner_PoseOnStep(&step, 175L, &pose);
  pose.node_id = 6U;
  check(pose.valid && pose.x_mm == 1625 && pose.y_mm == 0);
  check(pose.body_heading == 3U);
  check(stable(&gate, &pose, 1U, 650U, 10U));
  check(gate.hit_x_mm == 975 && gate.hit_y_mm == 0);
  check(!stable(&gate, &pose, 1U, 651U, 10U));
  check(gate.status == FRONT_LASER_TOO_FAR);

  /* Reverse preserves position but puts the front sensor toward world east. */
  step.drive_mode = 1U;
  ThiefMapPlanner_PoseOnStep(&step, 175L, &pose);
  pose.node_id = 6U;
  check(pose.valid && pose.x_mm == 1625 && pose.y_mm == 0);
  check(pose.body_heading == 1U && pose.travel_heading == 3U);
  check(stable(&gate, &pose, 1U, 650U, 10U));
  check(gate.hit_x_mm == 2275 && gate.hit_y_mm == 0);
  /* Side sensor at a corridor midpoint cannot invent a perpendicular road. */
  pose.body_heading = 2U;
  check(!stable(&gate, &pose, 1U, 300U, 10U));
  check(gate.status == FRONT_LASER_NO_MAP_ROAD);
  pose.body_heading = 1U;
  pose.valid = 0U;
  check(!stable(&gate, &pose, 1U, 300U, 10U));
  check(gate.status == FRONT_LASER_NOT_HIDING);

  /* Separate aligned road pieces do not permit a ray to jump the missing gap. */
  ThiefMapPlanner_PoseAtNode(7U, 1U, &pose);
  check(!stable(&gate, &pose, 1U, 300U, 10U));
  check(gate.status == FRONT_LASER_NO_MAP_ROAD);
  ThiefMapPlanner_PoseAtNode(8U, 3U, &pose);
  check(!stable(&gate, &pose, 1U, 300U, 10U));
  check(gate.status == FRONT_LASER_NO_MAP_ROAD);
  /* Being within the 650 mm scalar range does not authorize field exit. */
  ThiefMapPlanner_PoseAtNode(1U, 3U, &pose);
  check(!stable(&gate, &pose, 1U, 50U, 10U));
  check(gate.status == FRONT_LASER_NO_MAP_ROAD ||
        gate.status == FRONT_LASER_OUTSIDE_FIELD);

  /* A lost/invalid observation cannot complete the previous confirmation. */
  ThiefMapPlanner_PoseAtNode(33U, 0U, &pose);
  FrontLaserGate_Init(&gate);
  for (i = 0U; i < 2U; i++)
    check(!FrontLaserGate_Observe(&gate, &pose, 1U, 650U, 10U, 123U));
  FrontLaserGate_ResetEvidence(&gate);
  for (i = 0U; i < 4U; i++)
    check(!FrontLaserGate_Observe(&gate, &pose, 1U, 650U, 10U, 123U));
  check(FrontLaserGate_Observe(&gate, &pose, 1U, 650U, 10U, 123U));

  every_moving_road();

  test_done = 1U;
  while (1) {}
}
