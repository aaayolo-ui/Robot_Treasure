#include "RestartLineSearch.h"

/* Leading-line tracking uses50RPM and <=20RPM differential. Without the
 * leading line, brake/scan; rear black never authorizes forward translation. */
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
void RestartLineSearch_BeginScan(RestartLineSearch *s,int16_t last_error)
{
  RestartLineSearch_Reset(s,last_error);
  s->target_yaw=(int16_t)(s->bias*400);
  BrakeTo(s,SWEEP);
}
void RestartLineSearch_Update(RestartLineSearch *s,
    const RestartLineInput *in, RestartLineOutput *out)
{
  int16_t turn,desired;
  /* Rear-only black is evidence to retain, not proof of a road ahead. */
  uint8_t visible=in->leading_valid;
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
  if(s->line_count<3U) s->line_count++;
  if(in->leading_error) s->bias=in->leading_error<0?-1:1;
  turn=Clamp((int32_t)in->leading_error-in->yaw_cw_x100/2,200);
  if(s->line_count>=3U) out->ready=1U;
  out->left_x10=(int16_t)(500+turn);out->right_x10=(int16_t)(500-turn);
}

void RestartLineSearch_Probe(RestartLineSearch *s,
    const RestartLineInput *in,uint8_t allow_translate,RestartLineOutput *out)
{
  (void)allow_translate; /* LINE_FIRST: no blind translation on a map axis. */
  RestartLineSearch_Update(s,in,out);
  /* Recovery sweeps must overcome breakaway load at small angular errors.
   * Stop/visibility and the original +/-4degree sweep targets still apply. */
  if(!out->brake && out->left_x10==-out->right_x10 && out->left_x10) {
    if(out->left_x10>0 && out->left_x10<200)out->left_x10=200;
    if(out->left_x10<0 && out->left_x10> -200)out->left_x10=-200;
    out->right_x10=(int16_t)-out->left_x10;
  }
}
