#include "ThiefRiskSearch.h"
#include "ThiefMapPlanner.h"
#include "ThiefThreatModel.h"
#include "ThiefStrategyConfig.h"
#include "ThiefMapTopology.h"
#include <stddef.h>

#define NO_STATE 0xFFFFU
/* Bounded RAM workspace; no recursion, allocation or HAL calls. Decisions are
 * made only while the chassis is stopped. One label per heading/mode state is
 * a bounded heuristic, not a complete SIPP or minimax game solver. */
static uint32_t cost[THIEF_STATE_COUNT],arrival[THIEF_STATE_COUNT];
static int32_t safety[THIEF_STATE_COUNT];
static uint16_t parent[THIEF_STATE_COUNT],reverse_states[THIEF_STATE_COUNT];
static uint8_t visited[THIEF_STATE_COUNT];
static uint32_t exposed_at[MAP_ROUTE_NODE_COUNT];
static uint8_t exposed[MAP_ROUTE_NODE_COUNT];

static uint16_t State(uint16_t node,uint8_t heading,uint8_t mode)
{ return (uint16_t)((node-1U)*8U+heading*2U+mode); }
static uint16_t StateNode(uint16_t state) {return state/8U+1U;}
static uint8_t StateHeading(uint16_t state) {return (uint8_t)((state%8U)/2U);}

void ThiefRiskSearch_Init(void)
{
  uint16_t n;
  for(n=0U;n<MAP_ROUTE_NODE_COUNT;n++) {exposed[n]=0U;exposed_at[n]=0UL;}
}
void ThiefRiskSearch_MarkExposed(uint16_t node,uint32_t now)
{
  if(!node || node>MAP_ROUTE_NODE_COUNT) return;
  exposed[node-1U]=1U;exposed_at[node-1U]=now;
}
uint8_t ThiefRiskSearch_HideSafe(uint16_t node)
{ return (node && node<=MAP_ROUTE_NODE_COUNT)?thief_hide_safe[node-1U]:0U; }

uint32_t ThiefRiskSearch_LegTime(uint16_t length,uint8_t mode,
                                uint8_t turn,uint8_t change)
{
  uint32_t speed=mode?THIEF_REVERSE_MM_S:THIEF_FORWARD_MM_S;
  uint32_t time=((uint32_t)length*1000UL+speed-1UL)/speed;
  if(turn) time+=THIEF_TURN_EXTRA_MS;
  if(change) time+=mode?THIEF_SWITCH_TO_REV_MS:THIEF_SWITCH_TO_FWD_MS;
  return time;
}

static uint8_t HasHeading(uint16_t node,uint8_t heading)
{
  uint16_t s;
  for(s=0U;s<MAP_ROUTE_SEGMENT_COUNT;s++) {
    const MapRouteSegment_t *e=&map_route_segments[s];
    if(e->start==node && e->heading==heading) return 1U;
    if(e->end==node && ((e->heading+2U)%4U)==heading) return 1U;
  }
  return 0U;
}

static int32_t LegMargin(uint16_t from,uint16_t to,uint16_t length,
                         uint32_t before,uint32_t motion_start,uint32_t duration)
{
  int32_t worst=ThiefThreatModel_Margin(from,from,0U,before);
  int32_t value=ThiefThreatModel_Margin(from,from,0U,motion_start);
  uint16_t progress=0U;
  if(value<worst) worst=value;
  /* <=75 mm sampling, plus contact radius and timing guard. Near contact and
   * real continuous motion still require hardware validation. */
  for(;;) {
    uint32_t when=motion_start+duration*progress/length;
    value=ThiefThreatModel_Margin(from,to,progress,when);
    if(value<worst) worst=value;
    if(progress==length) break;
    progress=(uint16_t)((length-progress>75U)?progress+75U:length);
  }
  return worst;
}

uint8_t ThiefRiskSearch_Select(uint16_t start,uint8_t heading,uint8_t mode,
    uint8_t axis_only,uint8_t threat_mask,uint16_t blocked,uint16_t previous,
    uint32_t now,uint16_t *path,uint16_t *count,uint8_t *first_mode,
    ThiefDecision_t *decision)
{
  uint16_t root,i,pass,s,pick,expanded=0U,best_goal=NO_STATE;
  uint8_t threats;
  int32_t best_quality=-0x3FFFFFFFL;
  int32_t best_margin=-0x3FFFFFFFL;
  uint8_t have_safe=0U;
  if(!path || !count || !first_mode || !decision || !start ||
      start>MAP_ROUTE_NODE_COUNT || heading>3U || mode>1U) return 0U;
  decision->goal_node=0U;decision->states_expanded=0U;
  threats=ThiefThreatModel_Prepare(now);
  for(i=0U;i<THIEF_STATE_COUNT;i++) {
    cost[i]=THIEF_COST_INF;arrival[i]=0UL;safety[i]=6000L;
    parent[i]=NO_STATE;visited[i]=0U;
  }
  root=State(start,heading,mode);cost[root]=0UL;
  for(pass=0U;pass<THIEF_STATE_COUNT;pass++) {
    uint32_t best=THIEF_COST_INF;
    pick=NO_STATE;
    for(i=0U;i<THIEF_STATE_COUNT;i++)
      if(!visited[i] && cost[i]<best) {best=cost[i];pick=i;}
    if(pick==NO_STATE) break;
    visited[pick]=1U;expanded++;
    for(s=0U;s<MAP_ROUTE_SEGMENT_COUNT;s++) {
      const MapRouteSegment_t *e=&map_route_segments[s];
      uint16_t from=StateNode(pick),to=0U,next;
      uint8_t out=0U,delta,next_mode,turn,change;
      uint32_t travel,wait,at,new_cost,penalty=0UL;
      int32_t margin;
      if(e->start==from) {to=e->end;out=e->heading;}
      else if(e->end==from) {to=e->start;out=(uint8_t)((e->heading+2U)%4U);}
      if(!to || to==blocked || to==start) continue;
      /* A committed hide transfer must not bounce immediately over its last
       * edge. Starting in reverse remains allowed at the root. */
      if(parent[pick]!=NO_STATE && to==StateNode(parent[pick])) continue;
      delta=(uint8_t)((out+4U-StateHeading(pick))%4U);
      if(map_route_nodes[from-1U].kind==0U && delta!=0U) continue;
      if(pick==root && axis_only && (delta==1U || delta==3U)) continue;
      if(pick==root && (threat_mask&(1U<<out))) continue;
      next_mode=(uint8_t)((pick&1U)^((delta==2U)?1U:0U));
      turn=(delta==1U || delta==3U)?1U:0U;
      change=(delta==2U && pick!=root)?1U:0U;
      travel=ThiefRiskSearch_LegTime(e->length_mm,next_mode,0U,0U);
      wait=turn?THIEF_TURN_EXTRA_MS:0UL;
      if(change) wait+=next_mode?THIEF_SWITCH_TO_REV_MS:THIEF_SWITCH_TO_FWD_MS;
      if(pick==root) wait+=THIEF_STARTUP_MS;
      at=arrival[pick]+wait+travel;
      margin=LegMargin(from,to,e->length_mm,arrival[pick],arrival[pick]+wait,travel);
      if(pick==root && threats && margin<=0L) continue;
      if(margin<=0L) {
        uint32_t danger=(uint32_t)(-margin);
        if(danger>6000UL) danger=6000UL;
        penalty=15000UL+danger*4UL;
      } else if(margin<500L) penalty=(uint32_t)(500L-margin)*4UL;
      new_cost=cost[pick]+wait+travel+penalty;
      next=State(to,out,next_mode);
      if(!visited[next] && new_cost<cost[next]) {
        cost[next]=new_cost;arrival[next]=at;parent[next]=pick;
        safety[next]=margin<safety[pick]?margin:safety[pick];
      }
    }
  }
  for(i=0U;i<THIEF_STATE_COUNT;i++) {
    uint16_t node=StateNode(i),cursor,count_states=0U;
    uint32_t mm=0UL;
    int32_t quality,margin;
    uint8_t h=StateHeading(i),safe;
    if(cost[i]==THIEF_COST_INF || node==start || node==previous ||
       !thief_hide_safe[node-1U] || !(thief_parking_axis_mask[node-1U]&(1U<<h)) ||
       !HasHeading(node,h)) continue;
    if(exposed[node-1U] && (uint32_t)(now-exposed_at[node-1U])<THIEF_HIDE_COOLDOWN_MS) continue;
    cursor=i;
    while(cursor!=root && cursor!=NO_STATE && count_states<MAP_ROUTE_NODE_COUNT) {
      uint16_t p=parent[cursor];
      if(p==NO_STATE) break;
      for(s=0U;s<MAP_ROUTE_SEGMENT_COUNT;s++) {
        const MapRouteSegment_t *e=&map_route_segments[s];
        uint16_t a=StateNode(p),b=StateNode(cursor);
        if((e->start==a && e->end==b)||(e->start==b && e->end==a)) {mm+=e->length_mm;break;}
      }
      cursor=p;count_states++;
    }
    if(cursor!=root || count_states>=THIEF_ROUTE_MAX_STEPS ||
       (axis_only && mm<THIEF_MIN_RELOCATE_MM)) continue;
    margin=safety[i];safe=(margin>0L)?1U:0U;
    /* Prefer safe short-horizon paths first; if all complete plans are unsafe,
     * select the least-bad one only when its first leg passed the hard screen.
     * Search-time/exit quality is secondary, never the old 1.5 m target. */
    quality=(int32_t)thief_hide_quality[node-1U]*2L-(int32_t)arrival[i];
    if(!threats) quality+=(int32_t)thief_police_prior_ms[node-1U]/2L;
    if((safe && !have_safe) || (safe==have_safe &&
       ((!safe && margin>best_margin) || ((safe || margin==best_margin) && quality>best_quality)))) {
      best_goal=i;have_safe=safe;best_margin=margin;best_quality=quality;
    }
  }
  decision->states_expanded=expanded;decision->threat_count=threats;
  if(best_goal==NO_STATE) return 0U;
  i=0U;pick=best_goal;
  while(pick!=root && pick!=NO_STATE && i<THIEF_STATE_COUNT) {
    reverse_states[i++]=pick;pick=parent[pick];
  }
  if(pick!=root || !i || i+1U>MAP_ROUTE_NODE_COUNT) return 0U;
  path[0]=start;
  *first_mode=(uint8_t)(reverse_states[i-1U]&1U);
  *count=i+1U;
  for(s=0U;s<i;s++) path[s+1U]=StateNode(reverse_states[i-1U-s]);
  decision->goal_node=StateNode(best_goal);decision->travel_ms=arrival[best_goal];
  decision->worst_margin_ms=best_margin;decision->degraded=(uint8_t)!have_safe;
  return 1U;
}

uint32_t ThiefRiskSearch_WorkBytes(void)
{
  return (uint32_t)(sizeof(cost)+sizeof(arrival)+sizeof(safety)+sizeof(parent)+
      sizeof(reverse_states)+sizeof(visited)+sizeof(exposed_at)+sizeof(exposed));
}
