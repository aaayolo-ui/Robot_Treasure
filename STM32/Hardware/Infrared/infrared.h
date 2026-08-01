#ifndef ROBOT_TREASURE_INFRARED_H
#define ROBOT_TREASURE_INFRARED_H

#include <stdint.h>

typedef enum
{
    INFRARED_FRONT = 0,
    INFRARED_REAR,
    INFRARED_LEFT,
    INFRARED_RIGHT
} InfraredDirection;

void Infrared_Init(void);
uint8_t Infrared_IsDetected(InfraredDirection direction);

#endif
