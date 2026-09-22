#ifndef WHEEL_SPEED_PI_H
#define WHEEL_SPEED_PI_H

#include <stdint.h>

/* TUNING AREA - PI output PWM, not commanded speed. Timer period is 999;
 * this experiment caps PI output at 300 as in the verified wheel-speed tests.
 * START_MIN/RUNNING_MIN are lower floors while a nonzero target is active. */
#define WHEEL_SPEED_PI_PWM_MAX          300L
#define WHEEL_SPEED_PI_PWM_START_MIN    100L
#define WHEEL_SPEED_PI_PWM_RUNNING_MIN  50L
#define WHEEL_SPEED_PI_RUNNING_RPM_X10  50L
/* TUNING AREA - four independent wheel-speed PI loops, gain units x100.
 * Physical mapping under the verified new-front installation:
 * B=front-left, A=front-right, D=rear-left, C=rear-right.
 * Set each wheel separately here. Gray line-follow gains are in LineFollowPID.h. */
#define WHEEL_FL_B_KP_X100             80L
#define WHEEL_FL_B_KI_X100             25L
#define WHEEL_FR_A_KP_X100             80L
#define WHEEL_FR_A_KI_X100             25L
#define WHEEL_RL_D_KP_X100             80L
#define WHEEL_RL_D_KI_X100             25L
#define WHEEL_RR_C_KP_X100             80L
#define WHEEL_RR_C_KI_X100             25L
#define WHEEL_SPEED_PI_I_TERM_LIMIT_PWM 150L

typedef struct
{
  int32_t target_rpm_x10;
  int32_t measured_rpm_x10;
  int32_t error_rpm_x10;
  int32_t integral;
  int32_t output_pwm;
  uint8_t enabled;
  uint8_t saturated;
  int32_t kp_x100;
  int32_t ki_x100;
} WheelSpeedPi_t;

void WheelSpeedPi_Init(WheelSpeedPi_t *controller);
void WheelSpeedPi_SetGains(WheelSpeedPi_t *controller, int32_t kp_x100,
                           int32_t ki_x100);
void WheelSpeedPi_Reset(WheelSpeedPi_t *controller);
void WheelSpeedPi_SetEnabled(WheelSpeedPi_t *controller, uint8_t enabled);
void WheelSpeedPi_SetTarget(WheelSpeedPi_t *controller, int32_t target_rpm_x10);
void WheelSpeedPi_UpdateFeedback(WheelSpeedPi_t *controller,
                                 int32_t measured_rpm_x10);
int32_t WheelSpeedPi_Update(WheelSpeedPi_t *controller, int32_t measured_rpm_x10,
                            uint32_t elapsed_ms);
int32_t WheelSpeedPi_GetTarget(const WheelSpeedPi_t *controller);
int32_t WheelSpeedPi_GetMeasured(const WheelSpeedPi_t *controller);
int32_t WheelSpeedPi_GetError(const WheelSpeedPi_t *controller);
int32_t WheelSpeedPi_GetOutput(const WheelSpeedPi_t *controller);

#endif /* WHEEL_SPEED_PI_H */
