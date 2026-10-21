#include "ThiefRefugePolicy.h"
#include "ThiefRefugeMetadata.h"
#include "ThiefStrategyConfig.h"
uint8_t ThiefRefugePolicy_OnPatrolCorridor(uint16_t node)
{
  return (node && node<=88U) ? thief_refuge_patrol_corridor[node-1U] : 0U;
}

uint8_t ThiefRefugePolicy_Class(uint16_t node)
{
  if(!node || node>88U) return 0U;
#if THIEF_REFUGE_POLICY
  return thief_refuge_class[node-1U];
#else
  return 0U;
#endif
}
uint8_t ThiefRefugePolicy_Role(uint16_t node)
{
  return (node && node<=88U) ? thief_refuge_class[node-1U] : 0U;
}
uint8_t ThiefRefugePolicy_Preference(uint8_t role)
{
  return role==1U ? 2U : role==2U ? 1U : 0U;
}
uint8_t ThiefRefugePolicy_FastExits(uint16_t node)
{
  return (node && node<=88U) ? thief_refuge_fast_exits[node-1U] : 0U;
}
uint16_t ThiefRefugePolicy_PatrolDetectionMs(uint16_t node)
{
  return (node && node<=88U) ? thief_refuge_patrol_detection_ms[node-1U] : 0U;
}

int8_t ThiefRefugePolicy_CompareSlack(int32_t a, uint8_t a_class,
    int32_t b, uint8_t b_class)
{
#if THIEF_REFUGE_POLICY
  if(a>0L && b>0L) {
    int32_t a_band=a/(int32_t)THIEF_TIMING_GUARD_MS;
    int32_t b_band=b/(int32_t)THIEF_TIMING_GUARD_MS;
    if(a_band!=b_band) return a_band>b_band ? 1 : -1;
    a_class=ThiefRefugePolicy_Preference(a_class);
    b_class=ThiefRefugePolicy_Preference(b_class);
    if(a_class!=b_class) return a_class>b_class ? 1 : -1;
  }
#else
  (void)a_class;(void)b_class;
#endif
  return a==b ? 0 : a>b ? 1 : -1;
}
