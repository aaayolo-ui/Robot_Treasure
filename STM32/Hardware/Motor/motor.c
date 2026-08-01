#include "motor.h"

void Motor_Init(void)
{
    /* TODO: 接入实际 GPIO 和 PWM 外设。 */
}

void Motor_Stop(void)
{
    Motor_SetSpeed(0, 0);
}

void Motor_SetSpeed(int left_speed, int right_speed)
{
    (void)left_speed;
    (void)right_speed;
    /* TODO: 根据电机驱动型号设置左右轮组方向和 PWM。 */
}
