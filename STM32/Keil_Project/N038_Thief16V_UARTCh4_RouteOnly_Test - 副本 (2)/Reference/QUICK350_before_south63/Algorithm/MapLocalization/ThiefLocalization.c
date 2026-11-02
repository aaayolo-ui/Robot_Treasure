#include "ThiefLocalization.h"
#include <stddef.h>
#include <string.h>

static int16_t Delta(int32_t a, int32_t b)
{
  int32_t d=a-b;
  while(d>18000L) d-=36000L;
  while(d< -18000L) d+=36000L;
  return (int16_t)d;
}
static int32_t Abs(int32_t x) { return x<0L ? -x : x; }
static uint8_t Fail(ThiefLocalization_t *l, ThiefLocError_t e)
{
  if(l!=NULL) { l->error=(uint8_t)e; l->pose.valid=0U; }
  return 0U;
}
static uint8_t Fresh(ThiefLocalization_t *l, const ThiefLocYaw_t *y, uint32_t now)
{
  if(y==NULL) return 0U;
  l->gyro_age_ms=(uint32_t)(now-y->stamp_ms);
  return (y->valid && l->gyro_age_ms<=THIEF_LOC_GYRO_STALE_MS) ? 1U : 0U;
}
static uint8_t HeadingOK(ThiefLocalization_t *l, uint8_t body,
                         const ThiefLocYaw_t *y, int8_t sign)
{
  int32_t quarter=(int32_t)body-(int32_t)l->origin_heading;
  int32_t expected;
  if(body>3U || (sign==0 && body!=l->origin_heading)) return 0U;
  expected=(int32_t)l->origin_yaw_x100+quarter*9000L*(sign==0 ? 1L : (int32_t)sign);
  l->heading_error_x100=Delta(y->yaw_x100,expected);
  return Abs(l->heading_error_x100)<=THIEF_LOC_HEADING_TOL_X100 ? 1U : 0U;
}
static uint8_t SameStep(const MapRouteStep_t *a,const MapRouteStep_t *b)
{
  return (a!=NULL && b!=NULL && a->from_node==b->from_node && a->to_node==b->to_node &&
    a->heading==b->heading && a->drive_mode==b->drive_mode && a->length_mm==b->length_mm &&
    a->action==b->action && a->expected_ports==b->expected_ports && a->target_kind==b->target_kind) ? 1U : 0U;
}
static uint8_t GrayMatch(const MapRouteStep_t *s,uint16_t raw)
{
  uint8_t ports=0U,expected=0U;
  /* Runtime 16J routes put the node degree in expected_ports; legacy ROM
   * routes put a branch bitmask there. Derive actual relative roads instead. */
  if(ThiefMapPlanner_HasRoad(s->to_node,(uint8_t)((s->heading+3U)%4U),NULL,NULL)) expected|=1U;
  if(ThiefMapPlanner_HasRoad(s->to_node,(uint8_t)((s->heading+1U)%4U),NULL,NULL)) expected|=2U;
  if((raw&0x0C00U)==0x0C00U) ports|=1U;
  if((raw&0x0003U)==0x0003U) ports|=2U;
  /* An initial asymmetric frame need not expose every T/cross branch.
   * The existing controller also accumulates the required turning branch. */
  if(expected) return (ports&expected)!=0U ? 1U : 0U;
  return (s->target_kind==4U && raw==0x0FFFU) ? 1U : 0U;
}

void ThiefLocalization_Reset(ThiefLocalization_t *l)
{
  if(l==NULL) return;
  memset(l,0,sizeof(*l));
  l->error=THIEF_LOC_UNINITIALIZED;
}
uint8_t ThiefLocalization_Start(ThiefLocalization_t *l,uint16_t node,
    uint8_t body,const ThiefLocYaw_t *y,uint32_t now)
{
  if(l==NULL) return 0U;
  ThiefLocalization_Reset(l);
  if(ThiefMapPlanner_Node(node)==NULL || ThiefMapPlanner_Node(node)->kind==0U || body>3U)
    return Fail(l,THIEF_LOC_STEP_MISMATCH);
  if(!Fresh(l,y,now)) return Fail(l,THIEF_LOC_GYRO_STALE);
  l->initialized=1U; l->error=THIEF_LOC_OK;
  l->origin_yaw_x100=y->yaw_x100; l->origin_heading=body;
  l->body_heading=body; l->confirmed_node=node; l->phase=THIEF_LOC_AT_NODE;
  l->step.heading=body;
  ThiefMapPlanner_PoseAtNode(node,body,&l->pose);
  return 1U;
}
uint8_t ThiefLocalization_BeginStep(ThiefLocalization_t *l,const MapRouteStep_t *s,
    const ThiefLocYaw_t *y,uint32_t now)
{
  uint8_t body;
  if(l==NULL) return 0U;
  if(!l->initialized || l->error!=THIEF_LOC_OK) return 0U;
  if(s==NULL || s->heading>3U || s->drive_mode>1U || !s->length_mm ||
     ThiefMapPlanner_Node(s->to_node)==NULL || ThiefMapPlanner_Node(s->to_node)->kind==0U ||
     l->phase!=THIEF_LOC_AT_NODE ||
     s->from_node!=l->confirmed_node) return Fail(l,THIEF_LOC_STEP_MISMATCH);
  body=(uint8_t)((s->heading+2U*s->drive_mode)%4U);
  if(body!=l->body_heading) return Fail(l,THIEF_LOC_HEADING_MISMATCH);
  if(!Fresh(l,y,now)) return Fail(l,THIEF_LOC_GYRO_STALE);
  if(!HeadingOK(l,body,y,l->yaw_cw_sign)) return Fail(l,THIEF_LOC_HEADING_MISMATCH);
  l->step=*s; l->phase=THIEF_LOC_ON_LEG;
  ThiefMapPlanner_PoseOnStep(s,0L,&l->pose);
  l->pose.node_id=l->confirmed_node;
  return 1U;
}
uint8_t ThiefLocalization_CancelDeparture(ThiefLocalization_t *l,
    int32_t mm,const ThiefLocYaw_t *y,uint32_t now)
{
  if(!l || !l->initialized || l->error!=THIEF_LOC_OK) return 0U;
  if(mm!=0L || l->phase!=THIEF_LOC_ON_LEG || l->step.from_node!=l->confirmed_node)
    return Fail(l,THIEF_LOC_STEP_MISMATCH);
  if(!Fresh(l,y,now) || !HeadingOK(l,l->body_heading,y,l->yaw_cw_sign))
    return Fail(l,THIEF_LOC_HEADING_MISMATCH);
  l->phase=THIEF_LOC_AT_NODE;
  ThiefMapPlanner_PoseAtNode(l->confirmed_node,l->body_heading,&l->pose);
  l->pose.drive_mode=l->step.drive_mode;l->pose.travel_heading=l->step.heading;
  return 1U;
}
uint8_t ThiefLocalization_ReverseLeg(ThiefLocalization_t *l,
    const MapRouteStep_t *s,int32_t progress,const ThiefLocYaw_t *y,uint32_t now)
{
  if(!l || !s || !l->initialized || l->error!=THIEF_LOC_OK) return 0U;
  if(l->phase!=THIEF_LOC_ON_LEG || s->from_node!=l->step.to_node ||
      s->to_node!=l->step.from_node || s->length_mm!=l->step.length_mm ||
      s->heading!=(l->step.heading+2U)%4U ||
      s->drive_mode!=(l->step.drive_mode^1U) || s->action!=MAP_ACTION_STOP ||
      progress<0L || progress>(int32_t)s->length_mm)
    return Fail(l,THIEF_LOC_STEP_MISMATCH);
  if(!Fresh(l,y,now) || !HeadingOK(l,l->body_heading,y,l->yaw_cw_sign))
    return Fail(l,THIEF_LOC_HEADING_MISMATCH);
  l->step=*s;
  ThiefMapPlanner_PoseOnStep(s,(int32_t)s->length_mm-progress,&l->pose);
  l->pose.node_id=l->confirmed_node; /* No new gray-confirmed node. */
  return 1U;
}
uint8_t ThiefLocalization_PrepareDeparture(ThiefLocalization_t *l,
    const MapRouteStep_t *s,const ThiefLocYaw_t *y,uint32_t now)
{
  uint8_t body,delta;
  if(!l || !s || !l->initialized || l->error!=THIEF_LOC_OK) return 0U;
  if(l->phase!=THIEF_LOC_AT_NODE || s->from_node!=l->confirmed_node ||
      !Fresh(l,y,now) || !HeadingOK(l,l->body_heading,y,l->yaw_cw_sign))
    return Fail(l,THIEF_LOC_STEP_MISMATCH);
  body=(uint8_t)((s->heading+2U*s->drive_mode)%4U);
  delta=(uint8_t)((body+4U-l->body_heading)%4U);
  if(delta!=1U && delta!=3U) return Fail(l,THIEF_LOC_STEP_MISMATCH);
  l->step=*s;
  l->step.heading=(uint8_t)((l->body_heading+2U*s->drive_mode)%4U);
  l->step.action=delta==1U?MAP_ACTION_RIGHT:MAP_ACTION_LEFT;
  l->arrival_yaw_x100=y->yaw_x100;l->phase=THIEF_LOC_AT_JUNCTION;
  l->pose.valid=0U;
  return 1U;
}
uint8_t ThiefLocalization_Update(ThiefLocalization_t *l,int32_t mm,
    const ThiefLocYaw_t *y,uint32_t now)
{
  if(l==NULL || !l->initialized || l->error!=THIEF_LOC_OK) return 0U;
  if(!Fresh(l,y,now)) return Fail(l,THIEF_LOC_GYRO_STALE);
  if(l->phase==THIEF_LOC_AT_JUNCTION && l->step.action!=MAP_ACTION_STOP) {
    /* Pose direction is transitional until the outgoing gray line and yaw
     * agree. Fresh yaw is still required throughout the transition. */
    l->pose.valid=0U; return 1U;
  }
  if(!HeadingOK(l,l->body_heading,y,l->yaw_cw_sign)) return Fail(l,THIEF_LOC_HEADING_MISMATCH);
  if(l->phase==THIEF_LOC_ON_LEG) {
    if(mm< -50L || mm>(int32_t)l->step.length_mm*16L/10L+100L)
      return Fail(l,THIEF_LOC_DISTANCE);
    ThiefMapPlanner_PoseOnStep(&l->step,mm,&l->pose);
    /* Encoder progress never promotes an unobserved endpoint into a node fix. */
    l->pose.node_id=l->confirmed_node;
  } else {
    ThiefMapPlanner_PoseAtNode(l->confirmed_node,l->body_heading,&l->pose);
    l->pose.drive_mode=l->step.drive_mode;
    l->pose.travel_heading=l->step.heading;
  }
  return 1U;
}
uint8_t ThiefLocalization_Arrive(ThiefLocalization_t *l,const MapRouteStep_t *s,
    uint16_t raw,int32_t mm,const ThiefLocYaw_t *y,uint32_t now)
{
  if(l==NULL || !l->initialized || l->error!=THIEF_LOC_OK) return 0U;
  if(l->phase!=THIEF_LOC_ON_LEG || !SameStep(s,&l->step)) return Fail(l,THIEF_LOC_STEP_MISMATCH);
  if(!ThiefLocalization_Update(l,mm,y,now)) return 0U;
  if(mm<(int32_t)s->length_mm*4L/10L-50L) return Fail(l,THIEF_LOC_DISTANCE);
  if(!GrayMatch(s,raw)) return Fail(l,THIEF_LOC_GRAY_MISMATCH);
  l->gray_raw12=raw; l->arrival_yaw_x100=y->yaw_x100;
  l->confirmed_node=s->to_node;
  l->phase=(s->action==MAP_ACTION_STOP) ? THIEF_LOC_AT_NODE : THIEF_LOC_AT_JUNCTION;
  l->confirmations++;
  ThiefMapPlanner_PoseAtNode(l->confirmed_node,l->body_heading,&l->pose);
  l->pose.drive_mode=s->drive_mode; l->pose.travel_heading=s->heading;
  if(s->action!=MAP_ACTION_STOP) l->pose.valid=0U;
  return 1U;
}
uint8_t ThiefLocalization_ExitReady(ThiefLocalization_t *l,const MapRouteStep_t *next,
    uint8_t line,const ThiefLocYaw_t *y,uint32_t now)
{
  uint8_t body;
  int32_t delta,abs_delta,quarter;
  int8_t sign;
  if(l==NULL || !l->initialized || l->error!=THIEF_LOC_OK) return 0U;
  if(l->phase!=THIEF_LOC_AT_JUNCTION || next==NULL || next->heading>3U || next->drive_mode>1U ||
     next->from_node!=l->confirmed_node) return Fail(l,THIEF_LOC_STEP_MISMATCH);
  if(!line || !Fresh(l,y,now)) return 0U;
  body=(uint8_t)((next->heading+2U*next->drive_mode)%4U);
  quarter=((int32_t)body+4L-(int32_t)l->body_heading)%4L;
  if(quarter==3L) quarter=-1L;
  if(quarter==2L) return Fail(l,THIEF_LOC_STEP_MISMATCH);
  if((l->step.action==MAP_ACTION_LEFT && quarter!=-1L) ||
     (l->step.action==MAP_ACTION_RIGHT && quarter!=1L) ||
     ((l->step.action==MAP_ACTION_STRAIGHT || l->step.action==MAP_ACTION_REVERSE) && quarter!=0L) ||
     (l->step.action==MAP_ACTION_STOP) ||
     next->drive_mode!=(l->step.drive_mode^((l->step.action==MAP_ACTION_REVERSE)?1U:0U)))
    return Fail(l,THIEF_LOC_STEP_MISMATCH);
  delta=Delta(y->yaw_x100,l->arrival_yaw_x100); abs_delta=Abs(delta);
  sign=l->yaw_cw_sign;
  if(quarter!=0L) {
    if(abs_delta<THIEF_LOC_TURN_MIN_X100) return 0U;
    if(abs_delta>THIEF_LOC_TURN_MAX_X100) return Fail(l,THIEF_LOC_HEADING_MISMATCH);
    if(sign==0) sign=(int8_t)((delta*quarter>0L) ? 1 : -1);
    if(delta*quarter*(int32_t)sign<=0L) return Fail(l,THIEF_LOC_TURN_DIRECTION);
  } else if(abs_delta>THIEF_LOC_HEADING_TOL_X100) return Fail(l,THIEF_LOC_HEADING_MISMATCH);
  /* Keep the original startup yaw reference: do not silently re-anchor away
   * accumulated errors at every junction. Signed direction is learned once. */
  if(HeadingOK(l,body,y,sign)) return 1U;
  /* A valid-direction quarter turn may still be converging on the outgoing
   * line. Keep it pending; the application's existing alignment timeout
   * bounds this wait. Do not commit or relax the absolute heading tolerance. */
  if(quarter!=0L) return 0U;
  return Fail(l,THIEF_LOC_HEADING_MISMATCH);
}
uint8_t ThiefLocalization_CommitExit(ThiefLocalization_t *l,const MapRouteStep_t *next,
    const ThiefLocYaw_t *y,uint32_t now)
{
  uint8_t body;
  int32_t quarter,delta;
  if(!ThiefLocalization_ExitReady(l,next,1U,y,now)) return 0U;
  body=(uint8_t)((next->heading+2U*next->drive_mode)%4U);
  quarter=((int32_t)body+4L-(int32_t)l->body_heading)%4L;
  if(quarter==3L) quarter=-1L;
  delta=Delta(y->yaw_x100,l->arrival_yaw_x100);
  if(l->yaw_cw_sign==0 && quarter!=0L) l->yaw_cw_sign=(int8_t)(delta*quarter>0L ? 1 : -1);
  l->body_heading=body; l->phase=THIEF_LOC_AT_NODE;
  ThiefMapPlanner_PoseAtNode(l->confirmed_node,body,&l->pose);
  return 1U;
}
const char *ThiefLocalization_ErrorText(uint8_t e)
{
  static const char *const names[]={"OK","LOC_UNINITIALIZED","LOC_GYRO_STALE",
    "LOC_HEADING_MISMATCH","LOC_STEP_MISMATCH","LOC_DISTANCE","LOC_GRAY_MISMATCH","LOC_TURN_DIRECTION"};
  return e<sizeof(names)/sizeof(names[0]) ? names[e] : "LOC_UNKNOWN";
}
