#ifndef THIEF_REFUGE_POLICY_H
#define THIEF_REFUGE_POLICY_H
#include <stdint.h>
uint8_t ThiefRefugePolicy_OnPatrolCorridor(uint16_t node);

/* Public-map stationary-refuge prior only. Never removes a road or reads
 * an unobserved opponent pose. Tier0 remains available as a last resort. */
uint8_t ThiefRefugePolicy_Class(uint16_t node);
uint8_t ThiefRefugePolicy_FastExits(uint16_t node);
uint16_t ThiefRefugePolicy_PatrolDetectionMs(uint16_t node);
/* Structural preference may only break a timing-uncertainty tie. */
int8_t ThiefRefugePolicy_CompareSlack(int32_t a, uint8_t a_class,
    int32_t b, uint8_t b_class);
#endif
