#include "RestartLineSearch.h"

/* Trial gains: 50 RPM translation, <=20 RPM differential; a stationary scan
 * targets +/-4 degrees from the mapped road heading. No elapsed-time turns. */
#define TRACK 0U
#define BRAKE 1U
#define SWEEP 2U
#define FAILED 3U
static int16_t Clamp(int32_t x, int16_t bound)
{
  return (int16_t)(x>bound?bound:x < -bound?-bound:x);
}
static void BrakeTo(RestartLineSearch *s, uint8_t next)
{
  s->phase=BRAKE;s->next_phase=next;s->stopped_count=0U;
}
void RestartLineSearch_Reset(RestartLineSearch *s, int16_t last_error)
{
  s->phase=TRACK;s->next_phase=TRACK;s->stopped_count=0U;
  s->line_count=0U;s->sweep_count=0U;s->target_yaw=0;
  s->bias=last_error<0?-1:1;
}
void RestartLineSearch_Update(RestartLineSearch *s,
    const RestartLineInput *in, RestartLineOutput *out)
{
  int16_t turn,desired;
  uint8_t visible=(uint8_t)(in->leading_valid || in->trailing_valid);
  out->left_x10=0;out->right_x10=0;out->brake=0U;out->ready=0U;out->failed=0U;
  if(!in->yaw_known) {out->brake=1U;out->failed=1U;return;}
  if(s->phase==BRAKE) {
    out->brake=1U;
    if(in->stopped) s->stopped_count++;else s->stopped_count=0U;
    if(s->stopped_count>=2U) {
      s->phase=visible?TRACK:s->next_phase;s->line_count=0U;
      if(s->phase==FAILED) out->failed=1U;
    }
    return;
  }
  if(s->phase==SWEEP) {
    if(visible) {BrakeTo(s,TRACK);out->brake=1U;return;}
    desired=(int16_t)(s->target_yaw-in->yaw_cw_x100);
    if(desired>=-50 && desired<=50) {
      s->sweep_count++;
      s->target_yaw=(int16_t)-s->target_yaw;
      BrakeTo(s,s->sweep_count>=2U?FAILED:SWEEP);
      out->brake=1U;return;
    }
    turn=Clamp(desired,300);
    out->left_x10=turn;out->right_x10=(int16_t)-turn;
    return;
  }
  if(s->phase==FAILED) {out->brake=1U;out->failed=1U;return;}
  if(!visible) {
    s->line_count=0U;s->sweep_count=0U;s->target_yaw=(int16_t)(s->bias*400);
    BrakeTo(s,SWEEP);out->brake=1U;return;
  }
  if(in->leading_valid) {
    if(s->line_count<3U) s->line_count++;
    if(in->leading_error) s->bias=in->leading_error<0?-1:1;
    turn=Clamp((int32_t)in->leading_error-in->yaw_cw_x100/2,200);
    if(s->line_count>=3U) out->ready=1U;
  } else {
    s->line_count=0U;
    if(in->trailing_error) s->bias=in->trailing_error<0?-1:1;
    /* Rear-only P is non-minimum-phase. Convert its offset to a small desired
     * heading and close that loop with yaw instead of copying the front PID. */
    desired=Clamp((int32_t)in->trailing_error*2,400);
    turn=Clamp(((int32_t)desired-in->yaw_cw_x100)/2,200);
  }
  out->left_x10=(int16_t)(500+turn);out->right_x10=(int16_t)(500-turn);
}
