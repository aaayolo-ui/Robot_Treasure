#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

void MotorA_Init(void);
void MotorA_Stop(void);
void MotorA_Forward(uint16_t pwm);
void MotorA_Reverse(uint16_t pwm);

#endif /* MOTOR_H */
