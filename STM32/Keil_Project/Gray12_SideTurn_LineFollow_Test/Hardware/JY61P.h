#ifndef JY61P_H
#define JY61P_H

#include <stdint.h>

/* JY61P default UART: 9600, 8N1. The yaw value is degree x100. */
void JY61P_Init(void);
void JY61P_Task(void);
uint8_t JY61P_GetYawX100(int16_t *yaw_x100);
uint32_t JY61P_GetLastUpdateMs(void);

#endif /* JY61P_H */
