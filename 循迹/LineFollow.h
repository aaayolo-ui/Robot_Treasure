#ifndef __LINEFOLLOW_H
#define __LINEFOLLOW_H

#include <stdint.h>

/* ============================================================
循迹 PID 参数

这些值必须根据你的 Trace_GetError() 单位来调。

如果 Trace_GetError() 返回的是 cm：
  LINE_KP 通常 1 ~ 4
  LINE_KD 通常 0.01 ~ 0.1

如果 Trace_GetError() 返回的是传感器编号偏差：
  LINE_KP 可能 0.5 ~ 2
  LINE_KD 可能 0.01 ~ 0.1
============================================================ */

#ifndef LINE_KP
#define LINE_KP                 3.8f
#endif

#ifndef LINE_KD
#define LINE_KD                 0.30f
#endif

#ifndef LINE_STEER_MAX
#define LINE_STEER_MAX          14.0f
#endif

/*
 * 转向量变化率限制。
 * 单位：cm/s per second
 */
#ifndef LINE_STEER_RATE_MAX
#define LINE_STEER_RATE_MAX     200.0f
#endif

/*
 * 微分滤波系数。
 */
#ifndef LINE_D_FILTER_ALPHA
#define LINE_D_FILTER_ALPHA     0.30f
#endif

/*
 * 丢线最大保持周期。
 * 20 ms 一个周期。
 */
#ifndef LINE_LOST_MAX_TICKS
#define LINE_LOST_MAX_TICKS     30
#endif

/*
 * 基础速度斜坡默认值。
 */
#ifndef LINE_BASE_ACCEL_DEFAULT
#define LINE_BASE_ACCEL_DEFAULT 30.0f
#endif

#ifndef LINE_BASE_DECEL_DEFAULT
#define LINE_BASE_DECEL_DEFAULT 60.0f
#endif

/*
 * 误差减速增益。
 */
#ifndef LINE_ERROR_DECEL_GAIN
#define LINE_ERROR_DECEL_GAIN   1.5f
#endif

/*
 * 最低运行速度。
 */
#ifndef LINE_MIN_RUN_SPEED
#define LINE_MIN_RUN_SPEED       6.0f
#endif

/* ============================================================
API
============================================================ */

void LineFollow_Init(void);

void LineFollow_Start(float base_speed_cm_s);
void LineFollow_Stop(void);

uint8_t LineFollow_IsRunning(void);

void LineFollow_SetBaseSpeed(float base_speed_cm_s);
void LineFollow_SetRamp(float accel_cm_s2, float decel_cm_s2);

float LineFollow_GetRampSpeed(void);

void LineFollow_Update20ms(void);

#endif
