#ifndef WHEEL_SPEED_PI_H
#define WHEEL_SPEED_PI_H

#include <stdint.h>

#define WHEEL_SPEED_PI_PWM_MAX          300L
#define WHEEL_SPEED_PI_PWM_START_MIN    100L
#define WHEEL_SPEED_PI_PWM_RUNNING_MIN  50L
#define WHEEL_SPEED_PI_RUNNING_RPM_X10  50L
#define WHEEL_SPEED_PI_KP_X100          80L
#define WHEEL_SPEED_PI_KI_X100          25L
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
} WheelSpeedPi_t;

void WheelSpeedPi_Init(WheelSpeedPi_t *controller);
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
