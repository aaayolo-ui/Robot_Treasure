#ifndef ROBOT_TREASURE_GRAY_H
#define ROBOT_TREASURE_GRAY_H

#include <stdint.h>

#define GRAY_SENSOR_COUNT 12U

void Gray_Init(void);
uint16_t Gray_Read(void);

#endif
