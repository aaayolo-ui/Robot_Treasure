#include "TreasureGrayMask.h"
#include "ThiefLocalization.h"
#include "gray_position.h"
#include <stddef.h>
#include <string.h>

volatile unsigned test_checks, test_failures, test_done;
volatile unsigned test_first_line, test_patterns;
static ThiefLocalization_t loc;

static void checked(unsigned ok, unsigned line)
{
  test_checks++;
  if (!ok) {
    if (!test_failures) test_first_line = line;
    test_failures++;
  }
}
#define check(ok) checked((ok), __LINE__)

static MapRouteStep_t treasure_step(unsigned west, unsigned reverse)
{
  MapRouteStep_t step;
  memset(&step, 0, sizeof(step));
  step.from_node = west ? 6U : 4U;
  step.to_node = west ? 4U : 6U;
  step.length_mm = 300U;
  step.segment_count = 2U;
  step.heading = west ? 3U : 1U;
  step.target_kind = 2U;
  step.expected_ports = 3U;
  step.action = MAP_ACTION_STRAIGHT;
  step.drive_mode = (uint8_t)reverse;
  return step;
}

static void same_position(const GrayPosition_Result_t *a,
                          const GrayPosition_Result_t *b)
{
  check(a->gray12 == b->gray12);
  check(a->black_count == b->black_count);
  check(a->weighted_sum_x100 == b->weighted_sum_x100);
  check(a->position_x100 == b->position_x100);
  check(a->error_x100 == b->error_x100);
  check(a->status == b->status);
  check(a->valid == b->valid);
}

void test_main(void)
{
  unsigned west, leading, reverse, raw;
  uint16_t mask, south, filtered;
  MapRouteStep_t step, other;
  GrayPosition_Result_t base, actual;
  ThiefLocYaw_t yaw;

  /* A physical north-side text patch must not change the ten retained bits.
   * TravelFrame swaps boards for backing: its logical leading/trailing board
   * labels keep this same geographic mask in either drive mode. */
  for (west = 0U; west < 2U; west++) {
    for (leading = 0U; leading < 2U; leading++) {
      mask = ((west != 0U) == (leading != 0U)) ? 0x0003U : 0x0C00U;
      south = (mask == 0x0003U) ? 0x0C00U : 0x0003U;
      for (reverse = 0U; reverse < 2U; reverse++) {
        step = treasure_step(west, reverse);
        for (raw = 0U; raw < 4096U; raw++) {
          filtered = TreasureGrayMask_Apply(&step, 1U, (uint8_t)leading,
                                            (uint16_t)raw);
          test_patterns++;
          check(filtered == (uint16_t)(raw & (uint16_t)~mask));
          check((filtered & mask) == 0U);
          check((filtered & (uint16_t)(0x0FFFU ^ mask)) ==
                (raw & (uint16_t)(0x0FFFU ^ mask)));
          check(TreasureGrayMask_Apply(&step, 1U, (uint8_t)leading, filtered)
                == filtered);
          check(TreasureGrayMask_Apply(&step, 0U, (uint8_t)leading,
                                      (uint16_t)raw) == raw);
          check(TreasureGrayMask_Apply(NULL, 1U, (uint8_t)leading,
                                      (uint16_t)raw) == raw);
        }

        /* Text on an otherwise centered line must leave exactly the same
         * real GrayPosition output, rather than just suppress a branch. */
        GrayPosition_Calculate(0x0060U, &base);
        filtered = TreasureGrayMask_Apply(&step, 1U, (uint8_t)leading,
                                          (uint16_t)(0x0060U | mask));
        GrayPosition_Calculate(filtered, &actual);
        same_position(&base, &actual);
        check(actual.position_x100 == 0 && actual.valid);

        /* Do not invent a center line if only the printed text is detected. */
        filtered = TreasureGrayMask_Apply(&step, 1U, (uint8_t)leading, mask);
        GrayPosition_Calculate(filtered, &actual);
        check(filtered == 0U);
        check(actual.status == GRAY_POSITION_STATUS_NO_LINE && !actual.valid);

        /* The south branch of the real N004/N006 T junction survives. */
        filtered = TreasureGrayMask_Apply(&step, 1U, (uint8_t)leading,
                                          (uint16_t)(south | mask | 0x0060U));
        check(filtered == (uint16_t)(south | 0x0060U));
        check((filtered & south) == south);
        GrayPosition_Calculate(filtered, &actual);
        check(actual.status == GRAY_POSITION_STATUS_VALID && actual.valid);
        check(actual.black_count == 4U);

        /* All-black input retains the actual ten unsuppressed channels. */
        filtered = TreasureGrayMask_Apply(&step, 1U, (uint8_t)leading, 0x0FFFU);
        GrayPosition_Calculate(filtered, &actual);
        check(actual.black_count == 10U);
        check((filtered & south) == south);
        check(TreasureGrayMask_Apply(&step, 1U, (uint8_t)leading, 0xAFFFU) ==
              (uint16_t)(0xAFFFU & (uint16_t)~mask));

        /* The mask must not spill onto an adjacent map leg or another yaw. */
        other = step; other.to_node = west ? 3U : 7U;
        check(TreasureGrayMask_Apply(&other, 1U, (uint8_t)leading, 0x0FFFU)
              == 0x0FFFU);
        other = step; other.from_node = west ? 15U : 14U;
        check(TreasureGrayMask_Apply(&other, 1U, (uint8_t)leading, 0x0FFFU)
              == 0x0FFFU);
        other = step; other.heading = 0U;
        check(TreasureGrayMask_Apply(&other, 1U, (uint8_t)leading, 0x0FFFU)
              == 0x0FFFU);
        other.heading = 2U;
        check(TreasureGrayMask_Apply(&other, 1U, (uint8_t)leading, 0x0FFFU)
              == 0x0FFFU);
        other.heading = (uint8_t)((step.heading + 2U) % 4U);
        check(TreasureGrayMask_Apply(&other, 1U, (uint8_t)leading, 0x0FFFU)
              == 0x0FFFU);
      }
    }
  }

  /* Exercise the actual localization: correct south-side T evidence confirms
   * the endpoint; a north-side print halfway along the leg cannot do so. */
  ThiefMapPlanner_Init();
  yaw.yaw_x100 = 0; yaw.stamp_ms = 1000U; yaw.valid = 1U;
  for (west = 0U; west < 2U; west++) {
    mask = west ? 0x0003U : 0x0C00U;
    south = west ? 0x0C00U : 0x0003U;
    for (reverse = 0U; reverse < 2U; reverse++) {
      uint8_t body;
      step = treasure_step(west, reverse);
      body = (uint8_t)((step.heading + 2U * reverse) % 4U);
      check(ThiefLocalization_Start(&loc, step.from_node, body, &yaw, 1000U));
      check(ThiefLocalization_BeginStep(&loc, &step, &yaw, 1000U));
      filtered = TreasureGrayMask_Apply(&step, 1U, 1U,
                                          (uint16_t)(south | mask | 0x0060U));
      check(ThiefLocalization_Arrive(&loc, &step, filtered, 300L, &yaw, 1000U));
      check(loc.confirmed_node == step.to_node && loc.error == THIEF_LOC_OK);
      check(loc.gray_raw12 == (uint16_t)(south | 0x0060U));
      check(loc.phase == THIEF_LOC_AT_JUNCTION);

      check(ThiefLocalization_Start(&loc, step.from_node, body, &yaw, 1000U));
      check(ThiefLocalization_BeginStep(&loc, &step, &yaw, 1000U));
      filtered = TreasureGrayMask_Apply(&step, 1U, 1U, mask);
      check(!ThiefLocalization_Arrive(&loc, &step, filtered, 150L, &yaw, 1000U));
      check(loc.error == THIEF_LOC_GRAY_MISMATCH && !loc.pose.valid);
      check(loc.confirmed_node == step.from_node);
    }
  }
  check(test_patterns == 32768U);
  test_done = 1U;
  while (1) {}
}
