#ifndef ROBOT_TREASURE_PID_H
#define ROBOT_TREASURE_PID_H

typedef struct
{
    float kp;
    float ki;
    float kd;
    float integral;
    float previous_error;
} PIDController;

void PID_Init(PIDController *pid, float kp, float ki, float kd);
float PID_Update(PIDController *pid, float error);

#endif
