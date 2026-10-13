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
static int32_t earliest[MAP_ROUTE_NODE_COUNT];
static uint8_t visited[MAP_ROUTE_NODE_COUNT], active;

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

uint8_t ThiefThreatModel_Prepare(uint32_t now)
{
  uint16_t i, s, pass;
  uint8_t c;
  active=0U;
  for(i=0U;i<MAP_ROUTE_NODE_COUNT;i++) {
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
      if(value<earliest[e->start-1U]) earliest[e->start-1U]=value;
      value=(int32_t)TimeForDistance((uint32_t)db)-(int32_t)age;
      if(value<earliest[e->end-1U]) earliest[e->end-1U]=value;
      seeded=1U;
    }
    if(seeded) active++;
  }
  if(!active) return 0U;
  for(pass=0U;pass<MAP_ROUTE_NODE_COUNT;pass++) {
    uint16_t pick=0U;
    int32_t best=(int32_t)THIEF_COST_INF;
    for(i=0U;i<MAP_ROUTE_NODE_COUNT;i++)
      if(!visited[i] && earliest[i]<best) {best=earliest[i];pick=i+1U;}
    if(!pick) break;
    visited[pick-1U]=1U;
    for(s=0U;s<MAP_ROUTE_SEGMENT_COUNT;s++) {
      const MapRouteSegment_t *e=&map_route_segments[s];
      uint16_t next=0U;
      int32_t value;
      if(e->start==pick) next=e->end;
      else if(e->end==pick) next=e->start;
      if(!next) continue;
      value=best+(int32_t)TimeForDistance(e->length_mm);
      if(value<earliest[next-1U]) earliest[next-1U]=value;
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
  if(from==to) reach=earliest[from-1U];
  else for(s=0U;s<MAP_ROUTE_SEGMENT_COUNT;s++) {
    const MapRouteSegment_t *e=&map_route_segments[s];
    int32_t a,b;
    if(!((e->start==from && e->end==to)||(e->end==from && e->start==to))) continue;
    if(progress>e->length_mm) progress=e->length_mm;
    a=earliest[from-1U]+(int32_t)TimeForDistance(progress);
    b=earliest[to-1U]+(int32_t)TimeForDistance(e->length_mm-progress);
    reach=a<b?a:b;break;
  }
  /* This is a conservative timing screen, not proof of collision avoidance.
   * Edges are sampled by the planner; motion/localization uncertainty remains. */
  return reach-(int32_t)own_time-(int32_t)TimeForDistance(THIEF_CONTACT_MM)
         -(int32_t)THIEF_TIMING_GUARD_MS;
}

uint32_t ThiefThreatModel_WorkBytes(void)
{
  return (uint32_t)(sizeof(observation)+sizeof(earliest)+sizeof(visited)+sizeof(active));
}
