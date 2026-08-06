#ifndef SPEED_PID_H
#define SPEED_PID_H

#include <stdint.h>

#define SPEED_PID_PWM_MAX               600U
#define SPEED_PID_PWM_START_MIN         100U
#define SPEED_PID_PWM_RUNNING_MIN       50U
#define SPEED_PID_RUNNING_RPM_X10       50L

#define SPEED_PID_KP_X100               80L
#define SPEED_PID_KI_X100               25L
#define SPEED_PID_KD_X100               0L

void SpeedPid_Init(void);
void SpeedPid_Reset(void);

uint16_t SpeedPid_Update(int32_t target_rpm_x10,
                         int32_t measured_rpm_x10,
                         uint32_t elapsed_ms);

int32_t SpeedPid_GetErrorX10(void);
int32_t SpeedPid_GetFeedforwardPwm(void);
int32_t SpeedPid_GetPTermPwm(void);
int32_t SpeedPid_GetITermPwm(void);
int32_t SpeedPid_GetDTermPwm(void);

#endif /* SPEED_PID_H */
