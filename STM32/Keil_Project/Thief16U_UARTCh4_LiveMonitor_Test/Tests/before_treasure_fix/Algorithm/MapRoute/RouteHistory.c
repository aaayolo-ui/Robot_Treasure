#include "RouteHistory.h"
#include "ThiefInterceptPolicy.h"
#include <stddef.h>
typedef struct {MapRouteStep_t step;uint16_t gray;int16_t yaw;} HistoryEntry;
static HistoryEntry history[ROUTE_HISTORY_CAPACITY];
static uint16_t history_count;
uint32_t RouteHistory_WorkBytes(void){return (uint32_t)(sizeof(history)+sizeof(history_count));}
void RouteHistory_Reset(void){history_count=0U;}
uint16_t RouteHistory_Count(void){return history_count;}
uint16_t RouteHistory_LastNode(void){return history_count?history[history_count-1U].step.to_node:0U;}
void RouteHistory_Record(const MapRouteStep_t *s,uint16_t raw,int16_t yaw)
{
  uint16_t i;
  if(!s || !raw || !s->length_mm)return;
  if(history_count) {
    const MapRouteStep_t *last=&history[history_count-1U].step;
    if(last->to_node!=s->from_node)history_count=0U;
    else if(last->from_node==s->to_node && last->length_mm==s->length_mm &&
        (last->heading+2U)%4U==s->heading) {history_count--;return;}
  }
  /* Bounded recent history; dropping the oldest cannot stop the vehicle. */
  if(history_count==ROUTE_HISTORY_CAPACITY) {
    for(i=1U;i<history_count;i++)history[i-1U]=history[i];
    history_count--;
  }
  history[history_count].step=*s;history[history_count].gray=raw;
  history[history_count].yaw=yaw;history_count++;
}
uint8_t RouteHistory_PlanReturn(uint16_t node,uint8_t body,uint8_t threats,
    uint32_t now,ThiefRoutePlan_t *plan)
{
  uint16_t n=0U,cursor=node,left=history_count,i;
  uint8_t mode;
  if(!plan || body>3U || RouteHistory_LastNode()!=node)return 0U;
  while(left && n<THIEF_ROUTE_MAX_STEPS) {
    const MapRouteStep_t *old=&history[left-1U].step;
    MapRouteStep_t *s=&plan->steps[n];
    const MapRouteNode_t *to;
    uint8_t heading=(uint8_t)((old->heading+2U)%4U);
    if(old->to_node!=cursor || (!n && (threats&(1U<<heading))) ||
        ThiefIntercept_EdgeBlocked(old->to_node,old->from_node,now))break;
    to=ThiefMapPlanner_Node(old->from_node);if(!to)break;
    *s=*old;s->from_node=old->to_node;s->to_node=old->from_node;
    s->heading=heading;s->target_kind=to->kind;s->expected_ports=to->degree;
    s->segment_offset=0U;s->action=MAP_ACTION_STOP;
    cursor=s->to_node;n++;left--;
    if(to->degree>=3U)break;
  }
  if(!n)return 0U;
  mode=(uint8_t)(plan->steps[0].heading==body?0U:1U);
  plan->steps[0].drive_mode=mode;
  for(i=0U;i+1U<n;i++) {
    uint8_t d=(uint8_t)((plan->steps[i+1U].heading+4U-plan->steps[i].heading)%4U);
    plan->steps[i].action=d==3U?MAP_ACTION_LEFT:d==2U?MAP_ACTION_REVERSE:(int8_t)d;
    if(d==2U)mode^=1U;
    plan->steps[i+1U].drive_mode=mode;
  }
  plan->route.id=1603U;plan->route.offset=0U;plan->route.count=n;
  plan->route.start_node=node;plan->route.heading=plan->steps[0].heading;
  plan->route.drive_mode=plan->steps[0].drive_mode;plan->goal_node=cursor;
  plan->final_body_heading=(uint8_t)((plan->steps[n-1U].heading+2U*mode)%4U);
  plan->valid=1U;return 1U;
}
