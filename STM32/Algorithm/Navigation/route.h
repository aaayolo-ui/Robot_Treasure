#ifndef ROBOT_TREASURE_ROUTE_H
#define ROBOT_TREASURE_ROUTE_H

#include <stdint.h>

typedef enum
{
    ROUTE_FORWARD = 0,
    ROUTE_LEFT,
    ROUTE_RIGHT,
    ROUTE_STOP
} RouteCommand;

void Route_Init(void);
RouteCommand Route_Update(uint16_t gray_data);

#endif
