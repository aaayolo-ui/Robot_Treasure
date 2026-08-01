#include "pid.h"

void PID_Init(PIDController *pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral = 0.0f;
    pid->previous_error = 0.0f;
}

float PID_Update(PIDController *pid, float error)
{
    (void)pid;
    (void)error;
    /* TODO: 循迹测试阶段再实现并限制积分与输出。 */
    return 0.0f;
}
