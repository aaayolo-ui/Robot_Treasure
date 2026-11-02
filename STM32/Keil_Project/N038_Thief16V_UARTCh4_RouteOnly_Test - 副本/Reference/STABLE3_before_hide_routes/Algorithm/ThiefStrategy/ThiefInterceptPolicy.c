#include "ThiefInterceptPolicy.h"
#include "ThiefMapPlanner.h"
#include "ThiefStrategyConfig.h"
#include <string.h>
typedef struct {uint16_t from,to;uint32_t tick;uint8_t used;} Block;
static Block blocks[4];
static uint8_t write_index;
uint8_t ThiefIntercept_HideAxis(uint16_t node)
{
  if(node==11U || node==38U || node==35U) return 0x0AU;
  if(node==26U || node==63U || node==78U) return 0x05U;
  return 0U;
}
uint8_t ThiefIntercept_ShouldReverse(uint8_t travel,uint8_t mask,
    const uint16_t clearance[4],uint8_t cooldown)
{
  (void)clearance;(void)cooldown;
  /* A single pursuer on a parallel/side road is not blocking this leg.
   * Reverse only for a hit ahead of TRAVEL, including when driving backwards.
   * A cooldown must not authorize driving into a newly blocked forward road. */
  return (uint8_t)(travel<4U && (mask&(1U<<travel))!=0U);
}
void ThiefIntercept_Reset(void) {memset(blocks,0,sizeof(blocks));write_index=0U;}
void ThiefIntercept_BlockLeg(uint16_t from,uint16_t to,uint32_t now)
{
  blocks[write_index].from=from;blocks[write_index].to=to;
  blocks[write_index].tick=now;blocks[write_index].used=1U;
  write_index=(uint8_t)((write_index+1U)%4U);
}
uint8_t ThiefIntercept_EdgeBlocked(uint16_t from,uint16_t to,uint32_t now)
{
  uint8_t i;
  const MapRouteNode_t *a=ThiefMapPlanner_Node(from),*b=ThiefMapPlanner_Node(to);
  if(!a || !b) return 1U;
  for(i=0U;i<4U;i++) if(blocks[i].used &&
      (uint32_t)(now-blocks[i].tick)<THIEF_THREAT_MEMORY_MS) {
    const MapRouteNode_t *c=ThiefMapPlanner_Node(blocks[i].from);
    const MapRouteNode_t *d=ThiefMapPlanner_Node(blocks[i].to);
    int16_t lo,hi;
    if(!c || !d) continue;
    if(c->x_half==d->x_half && a->x_half==c->x_half && b->x_half==c->x_half) {
      lo=c->y_half<d->y_half?c->y_half:d->y_half;
      hi=c->y_half>d->y_half?c->y_half:d->y_half;
      if(a->y_half>=lo && a->y_half<=hi && b->y_half>=lo && b->y_half<=hi) return 1U;
    }
    if(c->y_half==d->y_half && a->y_half==c->y_half && b->y_half==c->y_half) {
      lo=c->x_half<d->x_half?c->x_half:d->x_half;
      hi=c->x_half>d->x_half?c->x_half:d->x_half;
      if(a->x_half>=lo && a->x_half<=hi && b->x_half>=lo && b->x_half<=hi) return 1U;
    }
  }
  return 0U;
}
