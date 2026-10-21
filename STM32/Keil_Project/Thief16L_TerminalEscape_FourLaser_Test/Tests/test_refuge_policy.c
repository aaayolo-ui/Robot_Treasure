#include "ThiefRefugePolicy.h"
#include "ThiefMapPlanner.h"
#include "ThiefStrategyConfig.h"
#include "ThiefMapTopology.h"

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

static int8_t exact_order(int32_t a, int32_t b)
{
  return a == b ? 0 : a > b ? 1 : -1;
}

static void metadata_and_roads(void)
{
  uint16_t next, length;
  unsigned i;
#if THIEF_REFUGE_POLICY
  check(ThiefRefugePolicy_Class(11U) == 1U);
  check(ThiefRefugePolicy_Class(12U) == 1U);
  check(ThiefRefugePolicy_Class(38U) == 1U);
#else
  check(!ThiefRefugePolicy_Class(11U));
  check(!ThiefRefugePolicy_Class(12U));
  check(!ThiefRefugePolicy_Class(38U));
#endif
  check(!ThiefRefugePolicy_Class(62U));
#if THIEF_REFUGE_POLICY
  check(ThiefRefugePolicy_Class(26U) == 2U);
#else
  check(!ThiefRefugePolicy_Class(26U));
#endif
  /* Turning off ranking does not disable the conditional safety contract. */
  check(ThiefRefugePolicy_Role(26U) == 2U);
  check(ThiefRefugePolicy_Role(38U) == 1U);
  check(ThiefRefugePolicy_Role(62U) == 0U);
  check(!ThiefRefugePolicy_Role(0U) && !ThiefRefugePolicy_Role(89U));
  check(ThiefRefugePolicy_Preference(1U) > ThiefRefugePolicy_Preference(2U));
  check(ThiefRefugePolicy_Preference(2U) > ThiefRefugePolicy_Preference(0U));
  check(!ThiefRefugePolicy_Preference(3U) && !ThiefRefugePolicy_Preference(255U));
  check(ThiefRefugePolicy_FastExits(26U) == 2U);
  check(!ThiefRefugePolicy_PatrolDetectionMs(26U));
  check(!ThiefRefugePolicy_OnPatrolCorridor(26U));
  check(thief_parking_axis_mask[25] == 5U && thief_exit_count[25] == 3U);
  check(thief_external_node[25] && !thief_external_node[15]);
  check(thief_external_branch[25][0] == 6U);
  check(thief_external_branch[25][1] == 0U);
  check(thief_external_branch[25][2] == 35U);
  check(thief_external_branch[25][3] == 25U);
  check(thief_exit_branch_ms[25][0] == 2631U);
  check(thief_exit_branch_ms[25][1] == 65535U);
  check(thief_exit_branch_ms[25][2] == 526U);
  check(thief_exit_branch_ms[25][3] == 971U);
  check(ThiefRefugePolicy_FastExits(38U) == 4U);
  check(ThiefRefugePolicy_FastExits(62U) == 2U);
  check(ThiefRefugePolicy_PatrolDetectionMs(38U) == 4481U);
  check(!ThiefRefugePolicy_PatrolDetectionMs(62U));
  check(!ThiefRefugePolicy_Class(0U));
  check(!ThiefRefugePolicy_Class(89U));
  check(!ThiefRefugePolicy_Class(65535U));
  check(!ThiefRefugePolicy_FastExits(0U));
  check(!ThiefRefugePolicy_FastExits(89U));
  check(!ThiefRefugePolicy_FastExits(65535U));
  check(!ThiefRefugePolicy_PatrolDetectionMs(0U));
  check(!ThiefRefugePolicy_PatrolDetectionMs(89U));
  check(!ThiefRefugePolicy_PatrolDetectionMs(65535U));
  /* Refuge quality must not remove the corridor connection used by real
   * routes. Verify N062 explicitly, then both directions of every road. */
  check(ThiefMapPlanner_HasRoad(62U, 3U, &next, &length));
  check(next == 61U && length == 300U);
  check(ThiefMapPlanner_HasRoad(61U, 1U, &next, &length));
  check(next == 62U && length == 300U);
  /* Conditional screening does not erase the raw east gateway road. */
  check(ThiefMapPlanner_HasRoad(26U, 1U, &next, &length));
  check(next == 27U && length == 300U);
  check(ThiefMapPlanner_HasRoad(27U, 3U, &next, &length));
  check(next == 26U && length == 300U);
  check(ThiefMapPlanner_HasRoad(26U, 0U, &next, &length));
  check(next == 17U && length == 300U);
  check(ThiefMapPlanner_HasRoad(17U, 0U, &next, &length));
  check(next == 7U && length == 300U);
  check(ThiefMapPlanner_HasRoad(7U, 3U, &next, &length));
  check(next == 6U && length == 600U);
  for (i = 0U; i < MAP_ROUTE_SEGMENT_COUNT; i++) {
    const MapRouteSegment_t *edge = &map_route_segments[i];
    check(ThiefMapPlanner_HasRoad(edge->start, edge->heading, &next, &length));
    check(next == edge->end && length == edge->length_mm);
    check(ThiefMapPlanner_HasRoad(edge->end, (uint8_t)((edge->heading + 2U) % 4U),
                                  &next, &length));
    check(next == edge->start && length == edge->length_mm);
  }
}

static void explicit_slack_cases(void)
{
  /* The on/off columns are independent contract examples: metadata may
   * win a positive same-band uncertainty tie, but never a different band,
   * an unsafe/nonpositive comparison, or disabled-policy exact ordering. */
  static const struct {
    int32_t a, b;
    uint8_t a_class, b_class;
    int8_t enabled, disabled;
  } cases[] = {
    {120L, 239L, 1U, 0U, 1, -1},
    {239L, 120L, 0U, 1U, -1, 1},
    {240L, 239L, 0U, 1U, 1, 1},
    {239L, 240L, 1U, 0U, -1, -1},
    {120L, 119L, 0U, 1U, 1, 1},
    {119L, 120L, 1U, 0U, -1, -1},
    {1L, 119L, 1U, 0U, 1, -1},
    {1L, 119L, 1U, 1U, -1, -1},
    {239L, 120L, 0U, 0U, 1, 1},
    {100L, 100L, 1U, 0U, 1, 0},
    {100L, 100L, 0U, 1U, -1, 0},
    {100L, 100L, 1U, 1U, 0, 0},
    {0L, 0L, 1U, 0U, 0, 0},
    {-1L, 0L, 1U, 0U, -1, -1},
    {0L, -1L, 0U, 1U, 1, 1},
    {-120L, -239L, 0U, 1U, 1, 1},
    {-239L, -120L, 1U, 0U, -1, -1},
    {1L, 0L, 0U, 1U, 1, 1},
    {-1L, 1L, 1U, 0U, -1, -1},
    {-100L, -100L, 1U, 0U, 0, 0},
    {2147483647L, 2147483646L, 0U, 0U, 1, 1},
    {-2147483647L - 1L, -2147483647L, 1U, 0U, -1, -1},
    /* Role2 is conditional, not numerically better than normal Role1. */
    {239L, 120L, 2U, 1U, -1, 1},
    {120L, 239L, 1U, 2U, 1, -1},
    {120L, 239L, 2U, 0U, 1, -1},
    {239L, 120L, 0U, 2U, -1, 1},
    {240L, 239L, 2U, 1U, 1, 1},
    {239L, 240L, 1U, 2U, -1, -1},
    {100L, 100L, 2U, 1U, -1, 0},
    {100L, 100L, 2U, 0U, 1, 0},
    {0L, 0L, 2U, 1U, 0, 0},
    {-1L, 1L, 2U, 0U, -1, -1},
    {239L, 120L, 2U, 2U, 1, 1},
    {120L, 239L, 2U, 2U, -1, -1}
  };
  unsigned i;
  check(THIEF_TIMING_GUARD_MS == 120UL);
  for (i = 0U; i < sizeof(cases) / sizeof(cases[0]); i++) {
#if THIEF_REFUGE_POLICY
    int8_t expected = cases[i].enabled;
#else
    int8_t expected = cases[i].disabled;
#endif
    check(ThiefRefugePolicy_CompareSlack(cases[i].a, cases[i].a_class,
                                         cases[i].b, cases[i].b_class) == expected);
  }
}

static void ordering_properties(void)
{
  static const int32_t times[] = {-240L, -120L, -1L, 0L, 1L, 119L,
                                 120L, 239L, 240L, 359L, 360L, 481L};
  unsigned i, j, a_class, b_class;
  for (i = 0U; i < sizeof(times) / sizeof(times[0]); i++)
    for (j = 0U; j < sizeof(times) / sizeof(times[0]); j++)
      for (a_class = 0U; a_class < 2U; a_class++)
        for (b_class = 0U; b_class < 2U; b_class++) {
          int32_t a = times[i], b = times[j];
          int8_t order = ThiefRefugePolicy_CompareSlack(a, (uint8_t)a_class,
                                                        b, (uint8_t)b_class);
          check(order == -ThiefRefugePolicy_CompareSlack(b, (uint8_t)b_class,
                                                          a, (uint8_t)a_class));
#if THIEF_REFUGE_POLICY
          if (a <= 0L || b <= 0L || a_class == b_class ||
              a / 120L != b / 120L)
            check(order == exact_order(a, b));
          else
            check(order == (a_class > b_class ? 1 : -1));
#else
          check(order == exact_order(a, b));
#endif
        }
}

void test_main(void)
{
  metadata_and_roads();
  explicit_slack_cases();
  ordering_properties();
  test_done = 1U;
  while (1) {}
}
