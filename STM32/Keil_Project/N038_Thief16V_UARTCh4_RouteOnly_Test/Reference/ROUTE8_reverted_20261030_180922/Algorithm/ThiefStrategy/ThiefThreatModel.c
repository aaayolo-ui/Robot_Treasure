#include "ThiefThreatModel.h"
#include "ThiefStrategyConfig.h"
#include <stddef.h>

typedef struct {
  int16_t x_mm, y_mm;
  uint32_t tick;
  uint8_t valid;
} ThreatObservation_t;
static ThreatObservation_t observation[4];
/* Multi-source earliest reach times relative to the current decision, possibly
 * negative when the last sighting has aged. Unknown orientation is treated
 * conservatively: the opponent may use any connected road, with no turn delay. */
#if THIEF_TURN_REACH_MODEL
#define REACH_STATES (MAP_ROUTE_NODE_COUNT*4U)
#else
#define REACH_STATES MAP_ROUTE_NODE_COUNT
#endif
static int32_t earliest[REACH_STATES];
static uint8_t visited[REACH_STATES], active;

static uint16_t ReachState(uint16_t node,uint8_t heading)
{
#if THIEF_TURN_REACH_MODEL
  return (uint16_t)((node-1U)*4U+heading);
#else
  (void)heading;return node-1U;
#endif
}
static uint32_t DirectionDelay(uint8_t incoming,uint8_t outgoing)
{
#if THIEF_TURN_REACH_MODEL
  uint8_t delta=(uint8_t)((outgoing+4U-incoming)%4U);
  return delta==0U?0UL:delta==2U?THIEF_POLICE_MIN_SWITCH_MS:THIEF_POLICE_MIN_TURN_MS;
#else
  (void)incoming;(void)outgoing;return 0UL;
#endif
}
static int32_t ReachAlong(uint16_t node,uint8_t outgoing,uint32_t mm)
{
  uint8_t h;
  int32_t best=(int32_t)THIEF_COST_INF;
  for(h=0U;h<4U;h++) {
    int32_t value=earliest[ReachState(node,h)];
    if(mm) value+=(int32_t)DirectionDelay(h,outgoing);
    if(value<best) best=value;
  }
  return best;
}

static uint32_t TimeForDistance(uint32_t mm)
{
  return (mm * 1000UL) / THIEF_POLICE_MAX_MM_S;
}

void ThiefThreatModel_Init(void)
{
  uint16_t i;
  for (i=0U;i<4U;i++) observation[i].valid=0U;
  active=0U;
}

void ThiefThreatModel_Record(uint8_t channel,int16_t x,int16_t y,uint32_t tick)
{
  if(channel>=4U) return;
  observation[channel].x_mm=x;
  observation[channel].y_mm=y;
  observation[channel].tick=tick;
  observation[channel].valid=1U;
}

uint8_t ThiefThreatModel_HeadingHit(uint8_t heading,uint32_t now,int16_t *x,int16_t *y)
{
  if(heading>=4U || !observation[heading].valid ||
      (uint32_t)(now-observation[heading].tick)>250UL)return 0U;
  *x=observation[heading].x_mm;*y=observation[heading].y_mm;
  return 1U;
}

uint8_t ThiefThreatModel_Snapshot(uint32_t now,int16_t *x,int16_t *y)
{
  uint32_t age=0xFFFFFFFFUL;
  uint8_t c,pick=4U;
  uint16_t s;
  if(!x || !y) return 0U;
  for(c=0U;c<4U;c++) if(observation[c].valid && now-observation[c].tick<age) {
    age=now-observation[c].tick;pick=c;
  }
  if(pick==4U || age>250UL) return 0U;
  *x=observation[pick].x_mm;*y=observation[pick].y_mm;
  for(c=0U;c<4U;c++) if(observation[c].valid && now-observation[c].tick==age &&
      (observation[c].x_mm!=*x || observation[c].y_mm!=*y)) return 0U;
  for(s=0U;s<MAP_ROUTE_SEGMENT_COUNT;s++) {
    const MapRouteSegment_t *e=&map_route_segments[s];
    const MapRouteNode_t *a=&map_route_nodes[e->start-1U],*b=&map_route_nodes[e->end-1U];
    int ax=a->x_half*150,ay=a->y_half*150,bx=b->x_half*150,by=b->y_half*150;
    if((ax==bx && *x==ax && *y>=(ay<by?ay:by) && *y<=(ay>by?ay:by)) ||
       (ay==by && *y==ay && *x>=(ax<bx?ax:bx) && *x<=(ax>bx?ax:bx))) return 1U;
  }
  return 0U;
}

uint16_t ThiefThreatModel_Separation(uint16_t node,uint32_t now)
{
  uint8_t c;
  uint16_t best=0xFFFFU;
  if(!node || node>MAP_ROUTE_NODE_COUNT) return 0U;
  for(c=0U;c<4U;c++) if(observation[c].valid &&
      (uint32_t)(now-observation[c].tick)<=THIEF_THREAT_MEMORY_MS) {
    int32_t x=map_route_nodes[node-1U].x_half*150L-observation[c].x_mm;
    int32_t y=map_route_nodes[node-1U].y_half*150L-observation[c].y_mm;
    uint16_t d=(uint16_t)((x<0?-x:x)+(y<0?-y:y));
    if(d<best) best=d;
  }
  return best==0xFFFFU?0U:best;
}
uint8_t ThiefThreatModel_Prepare(uint32_t now)
{
  uint16_t i, s, pass;
  uint8_t c;
  active=0U;
  for(i=0U;i<REACH_STATES;i++) {
    earliest[i]=(int32_t)THIEF_COST_INF; visited[i]=0U;
  }
  for(c=0U;c<4U;c++) {
    uint32_t age=now-observation[c].tick;
    uint8_t seeded=0U;
    if(!observation[c].valid || age>THIEF_THREAT_MEMORY_MS) continue;
    for(s=0U;s<MAP_ROUTE_SEGMENT_COUNT;s++) {
      const MapRouteSegment_t *e=&map_route_segments[s];
      const MapRouteNode_t *a=&map_route_nodes[e->start-1U];
      const MapRouteNode_t *b=&map_route_nodes[e->end-1U];
      int32_t ax=a->x_half*150L, ay=a->y_half*150L;
      int32_t bx=b->x_half*150L, by=b->y_half*150L;
      int32_t x=observation[c].x_mm, y=observation[c].y_mm;
      int32_t da,db,value;
      if(ax==bx) {
        if(x!=ax || y<(ay<by?ay:by) || y>(ay>by?ay:by)) continue;
        da=y-ay; db=y-by;
      } else {
        if(y!=ay || x<(ax<bx?ax:bx) || x>(ax>bx?ax:bx)) continue;
        da=x-ax; db=x-bx;
      }
      if(da<0L) da=-da;
      if(db<0L) db=-db;
      value=(int32_t)TimeForDistance((uint32_t)da)-(int32_t)age;
      i=ReachState(e->start,(uint8_t)((e->heading+2U)%4U));
      if(value<earliest[i]) earliest[i]=value;
      value=(int32_t)TimeForDistance((uint32_t)db)-(int32_t)age;
      i=ReachState(e->end,e->heading);
      if(value<earliest[i]) earliest[i]=value;
      /* At a node the initial orientation is unknown; seed every heading,
       * including an orientation already aligned with the next road. */
      if(da==0L || db==0L) {
        uint8_t h;
        uint16_t node=da==0L?e->start:e->end;
        for(h=0U;h<4U;h++) {
          i=ReachState(node,h);
          if(-(int32_t)age<earliest[i]) earliest[i]=-(int32_t)age;
        }
      }
      seeded=1U;
    }
    if(seeded) active++;
  }
  if(!active) return 0U;
  for(pass=0U;pass<REACH_STATES;pass++) {
    uint16_t pick=0U,node;
    uint8_t heading=0U;
    int32_t best=(int32_t)THIEF_COST_INF;
    for(i=0U;i<REACH_STATES;i++)
      if(!visited[i] && earliest[i]<best) {best=earliest[i];pick=i+1U;}
    if(!pick) break;
    visited[pick-1U]=1U;
#if THIEF_TURN_REACH_MODEL
    node=(pick-1U)/4U+1U;heading=(uint8_t)((pick-1U)%4U);
#else
    node=pick;
#endif
    for(s=0U;s<MAP_ROUTE_SEGMENT_COUNT;s++) {
      const MapRouteSegment_t *e=&map_route_segments[s];
      uint16_t next=0U;
      uint8_t out=0U;
      int32_t value;
      if(e->start==node) {next=e->end;out=e->heading;}
      else if(e->end==node) {next=e->start;out=(uint8_t)((e->heading+2U)%4U);}
      if(!next) continue;
      value=best+(int32_t)DirectionDelay(heading,out)+(int32_t)TimeForDistance(e->length_mm);
      i=ReachState(next,out);
      if(value<earliest[i]) earliest[i]=value;
    }
  }
  return active;
}

int32_t ThiefThreatModel_Margin(uint16_t from,uint16_t to,
                               uint16_t progress,uint32_t own_time)
{
  uint16_t s;
  int32_t reach=(int32_t)THIEF_COST_INF;
  if(!active || own_time>THIEF_RISK_HORIZON_MS) return 6000L;
  if(!from || from>MAP_ROUTE_NODE_COUNT || !to || to>MAP_ROUTE_NODE_COUNT)
    return -6000L;
  if(from==to) reach=ReachAlong(from,0U,0UL);
  else for(s=0U;s<MAP_ROUTE_SEGMENT_COUNT;s++) {
    const MapRouteSegment_t *e=&map_route_segments[s];
    int32_t a,b;
    uint8_t out;
    if(!((e->start==from && e->end==to)||(e->end==from && e->start==to))) continue;
    if(progress>e->length_mm) progress=e->length_mm;
    out=(uint8_t)(e->start==from?e->heading:(e->heading+2U)%4U);
    a=ReachAlong(from,out,progress)+(int32_t)TimeForDistance(progress);
    b=ReachAlong(to,(uint8_t)((out+2U)%4U),e->length_mm-progress)+(int32_t)TimeForDistance(e->length_mm-progress);
    reach=a<b?a:b;break;
  }
  /* This is a conservative timing screen, not proof of collision avoidance.
   * Edges are sampled by the planner; motion/localization uncertainty remains. */
  return reach-(int32_t)own_time-(int32_t)TimeForDistance(THIEF_CONTACT_MM)
         -(int32_t)THIEF_TIMING_GUARD_MS;
}

uint8_t ThiefThreatModel_ApproachMask(uint16_t node)
{
  uint16_t s;
  int32_t value[4],best=(int32_t)THIEF_COST_INF;
  uint8_t h,mask=0U;
  if(!active || !node || node>MAP_ROUTE_NODE_COUNT) return 0U;
  for(h=0U;h<4U;h++) value[h]=(int32_t)THIEF_COST_INF;
  for(s=0U;s<MAP_ROUTE_SEGMENT_COUNT;s++) {
    const MapRouteSegment_t *e=&map_route_segments[s];
    uint16_t neighbor;
    uint8_t from;
    int32_t at;
    if(e->start==node) {neighbor=e->end;from=e->heading;}
    else if(e->end==node) {neighbor=e->start;from=(uint8_t)((e->heading+2U)%4U);}
    else continue;
    at=ReachAlong(neighbor,(uint8_t)((from+2U)%4U),e->length_mm)+
       (int32_t)TimeForDistance(e->length_mm);
    value[from]=at;if(at<best) best=at;
  }
  for(h=0U;h<4U;h++) if(value[h]<=best+50L) mask|=(uint8_t)(1U<<h);
  return mask;
}

uint32_t ThiefThreatModel_WorkBytes(void)
{
  return (uint32_t)(sizeof(observation)+sizeof(earliest)+sizeof(visited)+sizeof(active));
}
