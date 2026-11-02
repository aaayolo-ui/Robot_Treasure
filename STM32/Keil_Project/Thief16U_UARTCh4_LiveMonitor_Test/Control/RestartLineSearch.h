#ifndef RESTART_LINE_SEARCH_H
#define RESTART_LINE_SEARCH_H
#include <stdint.h>

/* Logical travel frame: positive gray error and yaw are to the right.
 * Targets must pass through TravelFrame exactly once, including reverse. */
typedef struct {
  int16_t leading_error, trailing_error, yaw_cw_x100;
  uint8_t leading_valid, trailing_valid, stopped, yaw_known;
} RestartLineInput;
typedef struct {
  int16_t left_x10, right_x10;
  uint8_t brake, ready, failed;
} RestartLineOutput;
typedef struct {
  uint8_t phase, next_phase, stopped_count, line_count, sweep_count;
  int8_t bias;
  int16_t target_yaw;
} RestartLineSearch;

void RestartLineSearch_Reset(RestartLineSearch *s, int16_t last_error);
/* Force measured-stop scan; caller supplies leading-only visibility here. */
void RestartLineSearch_BeginScan(RestartLineSearch *s,int16_t last_error);
void RestartLineSearch_Update(RestartLineSearch *s,
    const RestartLineInput *in, RestartLineOutput *out);
/* Compatibility entry: allow_translate no longer authorizes blind motion.
 * Leading loss permits brake/scan, never rear-only or map-only translation. */
void RestartLineSearch_Probe(RestartLineSearch *s,
    const RestartLineInput *in,uint8_t allow_translate,RestartLineOutput *out);
#endif
