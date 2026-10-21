/**********************************************************
 * TracePid.c
 *
 * 循迹 PD 外环
 *
 * 符号约定：
 *
 *   Trace_GetError() > 0
 *     线偏左，车应左转
 *
 *   Trace_GetError() < 0
 *     线偏右，车应右转
 *
 * 电机公式：
 *
 *   left  = base - steer
 *   right = base + steer
 *
 * 如果实测方向反了，
 * 修改：
 *
 *   TRACE_PID_STEER_SIGN
 *
 **********************************************************/

#include "TracePid.h"
#include "TRACE.h"
#include "PID_Config.h"
#include <math.h>

/* ============================================================
 * 循迹参数
 *
 * 这些参数可以先用宏固定。
 * 后续可以做成 UI 可调。
 * ============================================================ */

/*
 * 循迹 PD 参数。
 *
 * 建议初始：
 *
 *   KP = 4 ~ 8
 *   KD = 6 ~ 12
 */
#define TRACE_PID_KP                2.0f
#define TRACE_PID_KD                8.0f

/*
 * 基准速度，单位 cm/s。
 */
#define TRACE_PID_BASE_SPEED        8.0f

/*
 * 最低速度，单位 cm/s。
 *
 * 偏差较大时降速，但不至于完全不动。
 */
#define TRACE_PID_MIN_SPEED         3.0f

/*
 * 最大转向量。
 */
#define TRACE_PID_STEER_MAX         12.0f

/*
 * 速度上限。
 */
#define TRACE_PID_SPEED_LIMIT       25.0f

/*
 * 偏差越大，速度下降越多。
 *
 * base = BASE - |error| * DROP
 */
#define TRACE_PID_SPEED_DROP        1.5f

/*
 * D 项低通滤波系数。
 *
 * 0~1，越小越平滑。
 */
#define TRACE_PID_D_FILTER_ALPHA    0.30f

/*
 * 控制周期。
 */
#define TRACE_PID_DT                0.020f

/*
 * 丢线后保持上次误差的时间。
 *
 * 10 tick = 100ms
 */
#define TRACE_PID_LOST_HOLD_TICKS   10u

/*
 * 丢线超过该时间后进入故障停止。
 *
 * 100 tick = 1s
 *
 * 如果不想丢线自动停止，可以改成 0。
 */
#define TRACE_PID_LOST_FAULT_TICKS  100u

/*
 * 全黑超过该时间后进入故障停止。
 *
 * 150 tick = 1.5s
 *
 * 如果不想全黑自动停止，可以改成 0。
 */
#define TRACE_PID_BLACK_FAULT_TICKS 150u

/*
 * 转向符号。
 *
 * 如果小车循迹方向反了，
 * 改成：
 *
 *   #define TRACE_PID_STEER_SIGN  -1.0f
 */
#define TRACE_PID_STEER_SIGN        1.0f

/*
 * 是否允许单轮反向。
 *
 * 0：不允许，最低速度为 0
 * 1：允许，最低速度为 -TRACE_PID_SPEED_LIMIT
 */
#define TRACE_PID_ALLOW_REVERSE     0

/* ============================================================
 * 内部状态
 * ============================================================ */

static uint8_t s_trace_pid_running = 0;
static uint8_t s_trace_pid_fault = 0;

static float s_last_error = 0.0f;
static float s_d_filter = 0.0f;

static float s_steer = 0.0f;
static float s_base = 0.0f;

static uint16_t s_lost_ticks = 0;
static uint16_t s_black_ticks = 0;

/* ============================================================
 * 内部工具
 * ============================================================ */

static float TracePid_Clamp(float value, float min_value, float max_value)
{
    if (value > max_value)
    {
        return max_value;
    }

    if (value < min_value)
    {
        return min_value;
    }

    return value;
}

/* ============================================================
 * API
 * ============================================================ */

void TracePid_Init(void)
{
    s_trace_pid_running = 0;
    s_trace_pid_fault = 0;

    s_last_error = 0.0f;
    s_d_filter = 0.0f;

    s_steer = 0.0f;
    s_base = 0.0f;

    s_lost_ticks = 0;
    s_black_ticks = 0;
}

void TracePid_Start(void)
{
    s_trace_pid_running = 1;
    s_trace_pid_fault = 0;

    s_last_error = Trace_GetError();
    s_d_filter = 0.0f;

    s_steer = 0.0f;
    s_base = 0.0f;

    s_lost_ticks = 0;
    s_black_ticks = 0;

    /*
     * 启动速度环。
     *
     * 先给 0 目标速度，
     * 后续 TracePid_Update20ms() 会持续更新左右目标速度。
     */
    PID_StartSpeedTest_LR(0.0f, 0.0f);
}

void TracePid_Stop(void)
{
    s_trace_pid_running = 0;
    s_trace_pid_fault = 0;

    s_steer = 0.0f;
    s_base = 0.0f;

    PID_Stop();
}

uint8_t TracePid_IsRunning(void)
{
    return s_trace_pid_running;
}

uint8_t TracePid_IsFault(void)
{
    return s_trace_pid_fault;
}

float TracePid_GetSteer(void)
{
    return s_steer;
}

float TracePid_GetBase(void)
{
    return s_base;
}

void TracePid_Update20ms(void)
{
    Trace_State_t state;

    float error = 0.0f;
    float d_error = 0.0f;
    float steer = 0.0f;
    float base = 0.0f;

    float left_speed = 0.0f;
    float right_speed = 0.0f;

    float speed_min;

    if (!s_trace_pid_running)
    {
        return;
    }

    state = Trace_GetState();

    /* ========================================================
     * 1. 根据路线状态确定 error
     * ======================================================== */

    if (state == TRACE_STATE_ALL_WHITE)
    {
        /*
         * 丢线。
         *
         * 短时间内保持上一次误差，
         * 让车继续沿最后方向找线。
         */
        if (s_lost_ticks < 0xFFFFu)
        {
            s_lost_ticks++;
        }

        s_black_ticks = 0;

#if (TRACE_PID_LOST_FAULT_TICKS > 0u)
        if (s_lost_ticks >= TRACE_PID_LOST_FAULT_TICKS)
        {
            s_trace_pid_fault = 1;
        }
#endif

        error = s_last_error;
        d_error = 0.0f;
    }
    else if (state == TRACE_STATE_ALL_BLACK)
    {
        /*
         * 全黑。
         *
         * 短时间全黑按直行处理。
         * 长时间全黑可进入故障停止。
         */
        if (s_black_ticks < 0xFFFFu)
        {
            s_black_ticks++;
        }

        s_lost_ticks = 0;

#if (TRACE_PID_BLACK_FAULT_TICKS > 0u)
        if (s_black_ticks >= TRACE_PID_BLACK_FAULT_TICKS)
        {
            s_trace_pid_fault = 1;
        }
#endif

        error = 0.0f;
        s_last_error = 0.0f;
        d_error = 0.0f;
    }
    else
    {
        /*
         * 正常检测到线。
         */
        float prev_error = s_last_error;

        s_lost_ticks = 0;
        s_black_ticks = 0;
        s_trace_pid_fault = 0;

        error = Trace_GetError();
        s_last_error = error;

        d_error = (error - prev_error) / TRACE_PID_DT;
    }

    /* ========================================================
     * 2. 故障处理
     * ======================================================== */

    if (s_trace_pid_fault)
    {
        s_steer = 0.0f;
        s_base = 0.0f;

        PID_SetDebugSpeedTarget_LR(0.0f, 0.0f);

        return;
    }

    /* ========================================================
     * 3. PD 计算
     * ======================================================== */

    s_d_filter =
        TRACE_PID_D_FILTER_ALPHA * d_error +
        (1.0f - TRACE_PID_D_FILTER_ALPHA) * s_d_filter;

    steer = TRACE_PID_KP * error + TRACE_PID_KD * s_d_filter;
    steer *= -TRACE_PID_STEER_SIGN;

    steer = TracePid_Clamp(steer,
                           -TRACE_PID_STEER_MAX,
                           TRACE_PID_STEER_MAX);

    /* ========================================================
     * 4. 速度规划
     *
     * 偏差越大，速度越低。
     * ======================================================== */

    base = TRACE_PID_BASE_SPEED - fabsf(error) * TRACE_PID_SPEED_DROP;

    if (base < TRACE_PID_MIN_SPEED)
    {
        base = TRACE_PID_MIN_SPEED;
    }

    if (base > TRACE_PID_BASE_SPEED)
    {
        base = TRACE_PID_BASE_SPEED;
    }

    /* ========================================================
     * 5. 差速合成
     * ======================================================== */

    left_speed  = base - steer;
    right_speed = base + steer;

#if (TRACE_PID_ALLOW_REVERSE == 0)
    speed_min = 0.0f;
#else
    speed_min = -TRACE_PID_SPEED_LIMIT;
#endif

    left_speed = TracePid_Clamp(left_speed,
                                speed_min,
                                TRACE_PID_SPEED_LIMIT);

    right_speed = TracePid_Clamp(right_speed,
                                 speed_min,
                                 TRACE_PID_SPEED_LIMIT);

    s_steer = steer;
    s_base = base;

    /* ========================================================
     * 6. 交给速度环
     * ======================================================== */

    PID_SetDebugSpeedTarget_LR(left_speed, right_speed);
}
