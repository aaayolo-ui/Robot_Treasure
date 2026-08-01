#include "route.h"

void Route_Init(void)
{
    /* TODO: 正式规则和路口形式确认后初始化路线数据。 */
}

RouteCommand Route_Update(uint16_t gray_data)
{
    (void)gray_data;
    /* TODO: 后续实现简单路口判断与路线选择。 */
    return ROUTE_STOP;
}
