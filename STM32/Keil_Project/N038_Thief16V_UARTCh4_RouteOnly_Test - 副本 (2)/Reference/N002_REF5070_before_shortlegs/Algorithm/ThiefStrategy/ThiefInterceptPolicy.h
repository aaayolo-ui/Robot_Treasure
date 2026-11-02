#ifndef THIEF_INTERCEPT_POLICY_H
#define THIEF_INTERCEPT_POLICY_H
#include <stdint.h>
/* N/E/S/W = 0/1/2/3. Opposite headings share a parking axis. */
uint8_t ThiefIntercept_HideAxis(uint16_t node);
uint8_t ThiefIntercept_ShouldReverse(uint8_t travel, uint8_t threat_mask,
    const uint16_t clearance[4], uint8_t cooldown);
void ThiefIntercept_Reset(void);
void ThiefIntercept_BlockLeg(uint16_t from,uint16_t to,uint32_t now);
uint8_t ThiefIntercept_EdgeBlocked(uint16_t from,uint16_t to,uint32_t now);
#endif
