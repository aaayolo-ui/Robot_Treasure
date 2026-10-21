#include "FrontLaserGate.h"
#include "ThiefMapPlanner.h"
#include "ThiefRefugePolicy.h"
#include "ThiefRiskSearch.h"
#include "ThiefRollout.h"
#include "ThiefMapTopology.h"
#include "ThiefStrategyConfig.h"

volatile unsigned test_checks, test_failures, test_done, test_first_line;
volatile unsigned test_observations, test_plans, test_conditional_plans;
volatile unsigned test_conditional_checked_candidates, test_fallback_plans;
volatile unsigned test_max_model_steps, test_max_terminal_steps;
volatile unsigned test_finalist_fixtures, test_unknown_terminal_plans;
volatile unsigned test_partial_terminal_plans;
static ThiefRoutePlan_t plan;

static void checked(unsigned ok, unsigned line)
{
  test_checks++;
  if (!ok) {
    if (!test_failures) test_first_line = line;
    test_failures++;
  }
}
#define check(ok) checked((ok), __LINE__)

static unsigned pocket(unsigned node)
{
  return node == 8U || node == 9U || node == 18U || node == 19U ||
         node == 27U || node == 28U;
}

static void route_roads(void)
{
  unsigned i;
  for (i = 0U; i < plan.route.count; i++) {
    const MapRouteStep_t *step = &plan.steps[i];
    unsigned cursor = step->from_node, distance = 0U, hops = 0U;
    while (cursor != step->to_node && hops++ < MAP_ROUTE_NODE_COUNT) {
      unsigned short next = 0U, length = 0U;
      check(ThiefMapPlanner_HasRoad((uint16_t)cursor, step->heading, &next, &length));
      if (!next || !length) break;
      distance += length; cursor = next;
      check(!pocket(cursor));
    }
    check(cursor == step->to_node && distance == step->length_mm);
  }
}

static void accepted_plan(unsigned start, unsigned body, unsigned mask)
{
  const ThiefDecision_t *decision = ThiefMapPlanner_Decision();
  test_plans++;
  check(plan.valid && plan.route.count > 0U && plan.route.start_node == start);
  check((plan.steps[0].heading + 2U * plan.steps[0].drive_mode) % 4U == body);
  check(!(mask & (1U << plan.steps[0].heading)));
  check(decision->model_steps < 20000UL);
  check(decision->terminal_steps < 20000UL);
  if (decision->model_steps > test_max_model_steps) test_max_model_steps = decision->model_steps;
  if (decision->terminal_steps > test_max_terminal_steps) test_max_terminal_steps = decision->terminal_steps;
  if (decision->terminal_candidates == 2U) test_conditional_checked_candidates++;
  if (plan.goal_node == 26U) {
    test_conditional_plans++;
#if THIEF_TERMINAL_ESCAPE_ENABLED
    check(decision->terminal_checked);
    check(decision->terminal_known_models == THIEF_ROLLOUT_MODELS);
    check(decision->terminal_clear_models == THIEF_ROLLOUT_MODELS);
    check(decision->terminal_reserve_ms > (int32_t)THIEF_TIMING_GUARD_MS);
    check((thief_parking_axis_mask[25] & (1U << plan.final_body_heading)) != 0U);
    check(plan.final_body_heading == 0U || plan.final_body_heading == 2U);
    check(decision->forecast_clear && decision->surviving_models == THIEF_ROLLOUT_MODELS);
#else
    check(0U); /* Terminal-off must never admit the conditional stop. */
#endif
  }
  route_roads();
}

static void finalist_case(unsigned body, unsigned world,
                          unsigned stable_frames, unsigned expected_known,
                          unsigned expected_clear, unsigned expected_checked)
{
  FrontLaserGate_t gate;
  ThiefMapPose_t sensor;
  const ThiefDecision_t *decision;
  unsigned frame, accepted = 0U;
  ThiefMapPlanner_Init();
  ThiefRollout_ResetEvidence();
  ThiefMapPlanner_PoseAtNode(38U, (uint8_t)world, &sensor);
  FrontLaserGate_Init(&gate);
  /* Good out-of-range returns establish a recent clear ray. A missing frame
   * would reset evidence and cannot stand in for negative evidence. */
  for (frame = 0U; frame < 4U; frame++) {
    unsigned tick = frame * 20U;
    ThiefRollout_Evidence(38U, (uint8_t)world, 9999U, 30U, tick);
    check(!FrontLaserGate_Observe(&gate, &sensor, 1U, 9999U, 30U, 123U));
  }
  for (frame = 0U; frame < stable_frames; frame++) {
    unsigned tick = 80U + frame * 20U;
    ThiefRollout_Evidence(38U, (uint8_t)world, 600U, 30U, tick);
    accepted = FrontLaserGate_Observe(&gate, &sensor, 1U, 600U, 30U, 123U);
    if (!accepted) continue;
    ThiefMapPlanner_ObserveThreat(38U, (uint8_t)world,
                                  gate.hit_x_mm, gate.hit_y_mm, tick);
    test_observations++;
    check(ThiefMapPlanner_PlanEscape(38U, (uint8_t)body,
                                    (uint8_t)(1U << world), 0U, tick, &plan));
    accepted_plan(38U, body, 1U << world);
  }
  check(accepted && gate.status == FRONT_LASER_OPPONENT);
  decision = ThiefMapPlanner_Decision();
  check(decision->entry_inferred && plan.goal_node != 26U);
  test_finalist_fixtures++;
#if THIEF_TERMINAL_ESCAPE_ENABLED
  /* Both the normal refuge and N026 finalist really reach the terminal
   * forecast. Unknown or partial coverage cannot authorize N026 parking. */
  check(decision->terminal_candidates == 2U);
  check(decision->terminal_checked == expected_checked);
  check(decision->terminal_steps > 0U);
  check(decision->terminal_known_models == expected_known);
  check(decision->terminal_clear_models == expected_clear);
  check(decision->terminal_clear_models < THIEF_ROLLOUT_MODELS);
  check(decision->degraded && decision->terminal_reserve_ms == 0L);
  if (!expected_checked) check(plan.goal_node == 62U && decision->emergency_hide);
  if (!decision->terminal_known_models) test_unknown_terminal_plans++;
  if (decision->terminal_clear_models) test_partial_terminal_plans++;
#else
  (void)expected_known; (void)expected_clear; (void)expected_checked;
  check(!decision->terminal_candidates && !decision->terminal_checked);
#endif
}

void test_main(void)
{
  static const uint16_t origins[] = {11U, 12U, 26U, 38U, 62U};
  unsigned origin, body, world, frame;
  unsigned short path[49], count, first_mode;
  ThiefDecision_t fallback;
  check(ThiefRefugePolicy_Role(26U) == 2U);
  check(ThiefMapPlanner_WorkBytes() < 20000UL);
  for (origin = 0U; origin < sizeof(origins) / sizeof(origins[0]); origin++)
    for (body = 0U; body < 4U; body++)
      for (world = 0U; world < 4U; world++) {
        unsigned short neighbor = 0U, length = 0U;
        const MapRouteNode_t *target;
        FrontLaserGate_t gate;
        ThiefMapPose_t sensor;
        unsigned accepted = 0U;
        unsigned now = 10000U + origin * 1000U + body * 100U + world * 20U;
        if (!ThiefMapPlanner_HasRoad(origins[origin], (uint8_t)world, &neighbor, &length) ||
            length > THIEF_LASER_MAX_MM) continue;
        target = ThiefMapPlanner_Node(neighbor);
        check(target != 0);
        ThiefMapPlanner_Init();
        ThiefMapPlanner_PoseAtNode(origins[origin], (uint8_t)world, &sensor);
        FrontLaserGate_Init(&gate);
        for (frame = 0U; frame < 5U; frame++)
          accepted = FrontLaserGate_Observe(&gate, &sensor, 1U, length, 10U, 123U);
        check(accepted && gate.status == FRONT_LASER_OPPONENT);
        check(gate.hit_x_mm == target->x_half * 150 && gate.hit_y_mm == target->y_half * 150);
        ThiefMapPlanner_ObserveThreat(origins[origin], (uint8_t)world,
                                      gate.hit_x_mm, gate.hit_y_mm, now);
        test_observations++;
        if (ThiefMapPlanner_PlanEscape(origins[origin], (uint8_t)body,
                                       (uint8_t)(1U << world), 0U, now, &plan))
          accepted_plan(origins[origin], body, 1U << world);
        /* The graph-search fallback has no terminal forecast; even when
         * metadata ranking is disabled it cannot claim gate26 as a hide. */
        count = 0U; first_mode = 0U;
        if (ThiefRiskSearch_Select(origins[origin], (uint8_t)body, 0U, 1U,
                                   (uint8_t)(1U << world), 0U, 0U, now,
                                   path, &count, (uint8_t *)&first_mode, &fallback)) {
          test_fallback_plans++;
          check(count > 1U && path[count - 1U] != 26U && fallback.goal_node != 26U);
        }
      }
  /* Production 650 mm visibility: after the first gate acceptance at120ms,
   * N040 entry has only3/6 safe exits. South entry retains emergencyN062 when
   * both finalist forecasts remain unknown. No fabricated N026 admission. */
  finalist_case(0U, 1U, 3U, 6U, 3U, 1U);
  finalist_case(1U, 2U, 5U, 0U, 0U, 0U);
  check(test_observations >= 60U && test_plans > 20U && test_fallback_plans > 20U);
#if THIEF_TERMINAL_ESCAPE_ENABLED
  check(test_conditional_checked_candidates >= 2U);
  check(test_unknown_terminal_plans == 1U && test_partial_terminal_plans == 1U);
#endif
#if !THIEF_TERMINAL_ESCAPE_ENABLED
  check(test_conditional_plans == 0U && test_conditional_checked_candidates == 0U);
  check(test_unknown_terminal_plans == 0U && test_partial_terminal_plans == 0U);
#endif
  test_done = 1U;
  while (1) {}
}
