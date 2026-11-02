#include "TreasureGrayMask.h"
#include <stddef.h>

uint16_t TreasureGrayMask_AtNode(uint16_t node,uint8_t heading,
                                 uint8_t leading,uint16_t gray12)
{
  uint16_t mask;
  if(node<3U || node>7U || !(heading&1U) || heading>3U)return gray12;
  /* Text is outside the north border. Exclude all four outer side sensors,
   * preserving the four central line sensors and the real south branches.
   * TravelFrame swaps physical boards; the trailing installation is mirrored. */
  mask=((heading==1U && leading) || (heading==3U && !leading))?0x0F00U:0x000FU;
  return (uint16_t)(gray12 & (uint16_t)~mask);
}
uint16_t TreasureGrayMask_Apply(const MapRouteStep_t *step,uint8_t enabled,
                               uint8_t leading,uint16_t gray12)
{
  if(!enabled || !step || step->from_node<3U || step->from_node>7U ||
      step->to_node<3U || step->to_node>7U)return gray12;
  return TreasureGrayMask_AtNode(step->from_node,step->heading,leading,gray12);
}
