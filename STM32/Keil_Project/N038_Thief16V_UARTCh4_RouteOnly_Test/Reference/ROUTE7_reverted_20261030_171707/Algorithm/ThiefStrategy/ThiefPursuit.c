#include "ThiefPursuit.h"
#include "ThiefStrategyConfig.h"
#include "FrontLaserGate.h"
#include <string.h>

/* Q15 sine at 5-degree intervals; interpolation avoids floating point and
 * represents the actual rotating ray rather than its old cardinal channel. */
static int32_t Sin(int32_t angle)
{
  static const int16_t table[]={0,2856,5690,8481,11207,13848,16384,18794,
      21063,23170,25102,26841,28378,29698,30792,31650,32270,32642,32767};
  int32_t sign=1,index,remainder;
  angle%=36000L;if(angle<0L)angle+=36000L;
  if(angle>18000L){angle-=18000L;sign=-1;}
  if(angle>9000L)angle=18000L-angle;
  index=angle/500L;remainder=angle%500L;
  return sign*(table[index]+(index<18L?
      (table[index+1L]-table[index])*remainder/500L:0L));
}
static int32_t Along(int32_t dx,int32_t dy,uint8_t heading)
{
  return heading==0U?-dy:heading==1U?dx:heading==2U?dy:-dx;
}
static uint8_t NearRoad(int32_t x,int32_t y)
{
  uint16_t i;
  for(i=0U;i<MAP_ROUTE_SEGMENT_COUNT;i++) {
    const MapRouteSegment_t *s=&map_route_segments[i];
    const MapRouteNode_t *a=ThiefMapPlanner_Node(s->start),*b=ThiefMapPlanner_Node(s->end);
    int32_t ax=a->x_half*150L,ay=a->y_half*150L,bx=b->x_half*150L,by=b->y_half*150L;
    int32_t px=x,py=y,dx,dy;
    if(px<(ax<bx?ax:bx))px=ax<bx?ax:bx;
    if(px>(ax>bx?ax:bx))px=ax>bx?ax:bx;
    if(py<(ay<by?ay:by))py=ay<by?ay:by;
    if(py>(ay>by?ay:by))py=ay>by?ay:by;
    dx=x-px;dy=y-py;if(dx*dx+dy*dy<=10000L)return 1U;
  }
  return 0U;
}
void ThiefPursuit_Reset(ThiefPursuit *p){memset(p,0,sizeof(*p));}
void ThiefPursuit_Observe(ThiefPursuit *p,uint8_t channel,uint16_t raw,
    uint8_t confidence,const ThiefPursuitPose *pose,uint32_t tick)
{
  static const uint8_t offset[]={0U,3U,1U,2U};
  ThiefPursuitRay *r;
  int32_t angle,distance,x,y,dx,dy;
  if(channel>=4U)return;
  r=&p->rays[channel];
  if(raw<50U || raw>THIEF_LASER_CHANNEL_MAX_MM(channel) || confidence<THIEF_NEAR_CONFIDENCE) {
    r->count=0U;return;
  }
  angle=pose->body_cw_x100+offset[channel]*9000L;
  distance=FrontLaserGate_CenterDistanceMm(raw,channel);
  x=pose->x+distance*Sin(angle)/32767L;
  y=pose->y-distance*Sin(angle+9000L)/32767L;
  /* A confirmed pursuer must be inside the map, behind the intended motion,
   * and near a real road. Neither a blank map gap nor an outward wall counts. */
  if(x<0L || x>3300L || y<0L || y>2700L ||
      Along(x-pose->x,y-pose->y,pose->travel_heading)>-100L || !NearRoad(x,y)) {
    r->count=0U;return;
  }
  if(r->count && (uint32_t)(tick-r->last)<10U)return; /* No duplicate frames. */
  dx=x-r->x;dy=y-r->y;
  if(!r->count || (uint32_t)(tick-r->last)>100U || dx*dx+dy*dy>14400L) {
    r->count=1U;r->first=tick;
  } else if(r->count<3U)r->count++;
  r->x=(int16_t)x;r->y=(int16_t)y;r->last=tick;
  if(r->count>=3U && (uint32_t)(tick-r->first)>=30U) {
    p->x=(int16_t)x;p->y=(int16_t)y;p->stamp=tick;
  }
}
uint8_t ThiefPursuit_GoalMask(const ThiefPursuit *p,const MapRouteStep_t *goal,uint32_t now)
{
  const MapRouteNode_t *n=ThiefMapPlanner_Node(goal->to_node);
  int32_t dx=p->x-n->x_half*150L,dy=p->y-n->y_half*150L;
  int32_t ax=dx<0L?-dx:dx,ay=dy<0L?-dy:dy;
  uint8_t heading;
  if(!p->stamp || (uint32_t)(now-p->stamp)>1500U ||
      Along(dx,dy,goal->heading)>-(int32_t)THIEF_CONTACT_MM)return 0U;
  heading=ax>ay?(dx>0L?1U:3U):(dy>0L?2U:0U);
  return (uint8_t)(1U<<heading);
}
uint8_t ThiefPursuit_Append(ThiefRoutePlan_t *plan,const ThiefRoutePlan_t *tail)
{
  uint16_t count=plan->route.count,i;
  uint8_t delta,mode;
  MapRouteStep_t *last;
  if(!plan->valid || !tail->valid || !count || !tail->route.count ||
      count+tail->route.count>THIEF_ROUTE_MAX_STEPS ||
      tail->route.start_node!=plan->goal_node || tail->goal_node==plan->goal_node)return 0U;
  last=&plan->steps[count-1U];
  delta=(uint8_t)((tail->steps[0].heading+4U-last->heading)%4U);
  mode=(uint8_t)(last->drive_mode^(delta==2U));
  last->action=delta==3U?MAP_ACTION_LEFT:(delta==2U?MAP_ACTION_REVERSE:(int8_t)delta);
  for(i=0U;i<tail->route.count;i++) {
    MapRouteStep_t *s=&plan->steps[count+i];
    *s=tail->steps[i];s->drive_mode=mode;
    if(s->action==MAP_ACTION_REVERSE)mode^=1U;
  }
  plan->route.count=(uint16_t)(count+tail->route.count);plan->goal_node=tail->goal_node;
  last=&plan->steps[plan->route.count-1U];
  plan->final_body_heading=(uint8_t)((last->heading+2U*last->drive_mode)%4U);
  if(plan->goal_node==63U && plan->final_body_heading%2U)plan->final_body_heading=0U;
  return 1U;
}
