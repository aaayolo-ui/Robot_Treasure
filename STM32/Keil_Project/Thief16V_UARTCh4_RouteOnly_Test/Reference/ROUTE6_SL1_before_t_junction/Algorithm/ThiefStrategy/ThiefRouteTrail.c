#include "ThiefRouteTrail.h"

uint8_t ThiefRouteTrail_TrackLine(uint16_t raw)
{
  uint8_t width=0U;
  raw&=0xFFFU;if(!raw)return 0U;
  while(!(raw&1U))raw>>=1;
  while(raw&1U){width++;raw>>=1;}
  return (uint8_t)(raw==0U && width<=4U);
}

void ThiefRouteTrail_Begin(ThiefRouteTrail *t,uint16_t home,uint8_t body)
{
  t->home=home;t->home_body=body;t->count=0U;t->pending_valid=0U;
  t->returning=0U;t->return_left=0U;t->return_pending=0U;t->first_back=0U;
  t->expected_return_node=0U;t->enabled=(uint8_t)(home!=0U);
}

uint8_t ThiefRouteTrail_Stage(ThiefRouteTrail *t,const MapRouteStep_t *s,
    uint16_t leading,uint16_t trailing,int16_t yaw,int32_t mm)
{
  ThiefTrailEntry *e=&t->pending;
  if(!t->enabled)return 0U;
  if(t->returning) {
    if(!t->return_left || s->to_node!=t->expected_return_node)return 0U;
    t->return_pending=1U;
    return 1U;
  }
  if(t->count>=THIEF_TRAIL_CAPACITY || t->pending_valid ||
      s->from_node!=(t->count?t->entries[t->count-1U].to:t->home)) {
    t->enabled=0U;return 0U;
  }
  e->from=s->from_node;e->to=s->to_node;e->length_mm=s->length_mm;
  e->segments=s->segment_count;e->heading=s->heading;e->drive_mode=s->drive_mode;
  e->leading_raw=leading;e->trailing_raw=trailing;e->yaw_x100=yaw;
  e->measured_mm=(uint16_t)(mm<0L?0L:(mm>65535L?65535L:mm));
  t->pending_valid=1U;
  return 1U;
}

uint8_t ThiefRouteTrail_Commit(ThiefRouteTrail *t)
{
  if(!t->enabled)return 0U;
  if(t->returning) {
    if(!t->return_pending)return 0U;
    t->return_pending=0U;t->return_left--;
    if(t->first_back)t->first_back=0U;
    else if(t->count)t->count--;
    t->expected_return_node=t->count?t->entries[t->count-1U].from:t->home;
    return 1U;
  }
  if(!t->pending_valid)return 0U;
  t->entries[t->count++]=t->pending;t->pending_valid=0U;
  return 1U;
}

static void BackStep(MapRouteStep_t *s,uint16_t from,uint16_t to,
    uint16_t length,uint16_t segments,uint8_t heading,uint8_t mode)
{
  const MapRouteNode_t *node=ThiefMapPlanner_Node(from);
  s->from_node=to;s->to_node=from;s->length_mm=length;
  s->segment_offset=0U;s->segment_count=segments;
  s->heading=(uint8_t)((heading+2U)%4U);s->drive_mode=(uint8_t)(mode^1U);
  s->expected_ports=node->degree;s->target_kind=node->kind;s->action=MAP_ACTION_STOP;
}

uint8_t ThiefRouteTrail_BuildReturn(ThiefRouteTrail *t,
    const MapRouteStep_t *partial,ThiefRoutePlan_t *p)
{
  uint8_t i;
  if(!t->enabled || t->returning || t->pending_valid ||
      partial->from_node!=(t->count?t->entries[t->count-1U].to:t->home))return 0U;
  BackStep(&p->steps[0],partial->from_node,partial->to_node,partial->length_mm,
      partial->segment_count,partial->heading,partial->drive_mode);
  for(i=0U;i<t->count;i++) {
    const ThiefTrailEntry *e=&t->entries[t->count-1U-i];
    BackStep(&p->steps[i+1U],e->from,e->to,e->length_mm,e->segments,e->heading,e->drive_mode);
  }
  for(i=0U;i<t->count;i++) {
    uint8_t d=(uint8_t)((p->steps[i+1U].heading+4U-p->steps[i].heading)%4U);
    p->steps[i].action=d==3U?MAP_ACTION_LEFT:(d==2U?MAP_ACTION_REVERSE:(int8_t)d);
  }
  p->route.id=1603U;p->route.offset=0U;p->route.count=(uint16_t)(t->count+1U);
  p->route.start_node=partial->to_node;p->route.heading=p->steps[0].heading;
  p->route.drive_mode=p->steps[0].drive_mode;
  p->goal_node=t->home;p->final_body_heading=t->home_body;p->valid=1U;
  t->returning=1U;t->return_left=(uint8_t)p->route.count;t->return_pending=0U;
  t->first_back=1U;t->expected_return_node=partial->from_node;
  return 1U;
}
