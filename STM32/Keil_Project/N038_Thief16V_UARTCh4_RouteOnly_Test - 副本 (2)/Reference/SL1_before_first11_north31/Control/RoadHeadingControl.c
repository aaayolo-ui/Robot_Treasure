#include "RoadHeadingControl.h"
static int16_t Delta(int32_t value)
{
  while(value>18000L)value-=36000L;
  while(value< -18000L)value+=36000L;
  return (int16_t)value;
}
int16_t RoadHeading_AxisError(int16_t origin,uint8_t origin_heading,
    uint8_t body,int8_t sign,int16_t current)
{
  int32_t expected=(int32_t)origin+((int32_t)body-origin_heading)*9000L*(sign?sign:1);
  return Delta((int32_t)current-expected);
}
int16_t RoadHeading_Correction(int16_t error,int16_t bound)
{
  int32_t turn=-(int32_t)error/2L;
  return (int16_t)(turn>bound?bound:turn< -bound?-bound:turn);
}
uint8_t RoadHeading_TurnError(int16_t start,int16_t current,int8_t quarter,
    int8_t sign,int16_t *error)
{
  int16_t delta=Delta((int32_t)current-start);
  int32_t magnitude=delta<0?-(int32_t)delta:delta;
  if((quarter!=1 && quarter!=-1) || magnitude>11000L) return 0U;
  if(!sign) {
    if(magnitude<1000L)return 0U;
    sign=(int8_t)((int32_t)delta*quarter>0L?1:-1);
  }
  if((int32_t)delta*quarter*sign<0L)return 0U;
  *error=(int16_t)((int32_t)delta*sign-(int32_t)quarter*9000L);
  return 1U;
}
void RoadHeading_Reset(RoadHeadingControl_t *s,uint32_t now)
{
  s->brake_tick=now;s->last_yaw_tick=now;
  s->last_turn=0;s->spinning=0U;s->stopped_count=0U;s->stable_count=0U;
}
void RoadHeading_Update(RoadHeadingControl_t *s,const RoadHeadingInput_t *in,
    uint32_t now,int16_t tolerance,RoadHeadingOutput_t *out)
{
  int32_t magnitude=in->cw_error_x100<0?-(int32_t)in->cw_error_x100:in->cw_error_x100;
  int16_t turn=RoadHeading_Correction(in->cw_error_x100,300);
  out->turn_x10=0;out->brake=1U;out->ready=0U;out->failed=0U;
  if(s->spinning && (magnitude<=tolerance || (int32_t)turn*s->last_turn<=0L)) {
    RoadHeading_Reset(s,now);return;
  }
  if(!s->spinning) {
    if(in->stopped) {if(s->stopped_count<2U)s->stopped_count++;}
    else {s->stopped_count=0U;s->stable_count=0U;}
    if(s->stopped_count<2U || (uint32_t)(now-s->brake_tick)<80U)return;
    if(magnitude<=tolerance) {
      if((int32_t)(in->yaw_tick-s->last_yaw_tick)>0L) {
        s->last_yaw_tick=in->yaw_tick;
        if(s->stable_count<3U)s->stable_count++;
      }
      if(s->stable_count>=3U)out->ready=1U;
      return;
    }
    if(!in->direction_known) {out->failed=1U;return;}
    s->spinning=1U;s->stable_count=0U;
  }
  s->last_turn=turn;out->turn_x10=turn;out->brake=0U;
}
