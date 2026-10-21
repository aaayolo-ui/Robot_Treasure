/**********************************************************
LineFollow.c
循迹外环

本次修改重点：
1. 增加基础速度斜坡；
2. 启动更平滑；
3. 弯道与误差减速更柔和；
4. 保留全黑、丢线处理；
5. 为第 4 项启动平滑服务。
**********************************************************/

#include "LineFollow.h"
#include "TRACE.h"
#include "PID_Config.h"
#include "MOTOR.h"

#include <math.h>

#define LINE_DT     0.020f

/* ============================================================
内部状态
============================================================ */

static uint8_t  s_line_enable       = 0;

static float    s_target_base_speed = 0.0f;
static float    s_ramp_base_speed   = 0.0f;

static float    s_accel             = LINE_BASE_ACCEL_DEFAULT;
static float    s_decel             = LINE_BASE_DECEL_DEFAULT;

static float    s_last_error        = 0.0f;
static float    s_d_filter          = 0.0f;
static float    s_last_steer        = 0.0f;

static uint16_t s_lost_ticks        = 0;

/* ============================================================
内部工具
============================================================ */

static float LineFollow_Clamp(float value, float min_value, float max_value)
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
初始化
============================================================ */

void LineFollow_Init(void)
{
    s_line_enable = 0;

    s_target_base_speed = 0.0f;
    s_ramp_base_speed = 0.0f;

    s_accel = LINE_BASE_ACCEL_DEFAULT;
    s_decel = LINE_BASE_DECEL_DEFAULT;

    s_last_error = 0.0f;
    s_d_filter = 0.0f;
    s_last_steer = 0.0f;

    s_lost_ticks = 0;
}

/* ============================================================
启动 / 停止
============================================================ */

void LineFollow_Start(float base_speed_cm_s)
{
    s_target_base_speed = LineFollow_Clamp(base_speed_cm_s, 0.0f, 40.0f);

    /*
     * 启动时实际基础速度从 0 开始爬升。
     */
    s_ramp_base_speed = 0.0f;

    s_last_error = Trace_GetError();
    s_d_filter = 0.0f;
    s_last_steer = 0.0f;
    s_lost_ticks = 0;

    /*
     * 启动底层速度环。
     */
    PID_StartSpeedTest_LR(0.0f, 0.0f);

    s_line_enable = 1;
}

void LineFollow_Stop(void)
{
    s_line_enable = 0;

    s_target_base_speed = 0.0f;
    s_ramp_base_speed = 0.0f;
    s_last_steer = 0.0f;

    PID_Stop();
}

uint8_t LineFollow_IsRunning(void)
{
    return s_line_enable;
}

/* ============================================================
参数设置
============================================================ */

void LineFollow_SetBaseSpeed(float base_speed_cm_s)
{
    s_target_base_speed = LineFollow_Clamp(base_speed_cm_s, 0.0f, 40.0f);
}

void LineFollow_SetRamp(float accel_cm_s2, float decel_cm_s2)
{
    if (accel_cm_s2 < 1.0f)
    {
        accel_cm_s2 = 1.0f;
    }

    if (decel_cm_s2 < 1.0f)
    {
        decel_cm_s2 = 1.0f;
    }

    s_accel = accel_cm_s2;
    s_decel = decel_cm_s2;
}

float LineFollow_GetRampSpeed(void)
{
    return s_ramp_base_speed;
}

/* ============================================================
20ms 循迹更新
============================================================ */

void LineFollow_Update20ms(void)
{
    Trace_State_t state;

    float error = 0.0f;
    float d_error;
    float steer;
    float steer_delta;
    float max_steer_delta;

    float base;
    float ramp_delta;
    float max_accel_step;
    float max_decel_step;

    float left_speed;
    float right_speed;

    if (!s_line_enable)
    {
        return;
    }

    state = Trace_GetState();

    /* ------------------------------------------------------------
     * 1. 循迹误差
     * ------------------------------------------------------------ */
    if (state == TRACE_STATE_ALL_WHITE)
    {
        /*
         * 丢线。
         *
         * 短时间内保持上次误差方向。
         */
//        if (s_lost_ticks < LINE_LOST_MAX_TICKS)
//        {
//            s_lost_ticks++;
//            error = s_last_error;
//        }
//        else
//        {
//            /*
//             * 丢线过久，停止。
//             */
//            LineFollow_Stop();
//            return;
//        }

    s_lost_ticks = 0;
    error = s_last_error;
    d_error = 0.0f;
		
    }
    else if (state == TRACE_STATE_ALL_BLACK)
    {
        /*
         * 全黑。
         *
         * A 点启停线、十字路口等场景。
         * 这里按直行处理。
         */
        s_lost_ticks = 0;
        error = 0.0f;
        d_error = 0.0f;
				s_last_error=0;
    }
    else
    {
        float prev_error;

        prev_error = s_last_error;

        s_lost_ticks = 0;
        error = Trace_GetError();
        s_last_error = error;

        d_error = (error - prev_error) / LINE_DT;
    }

    /* ------------------------------------------------------------
     * 2. 微分滤波
     * ------------------------------------------------------------ */
    s_d_filter = LINE_D_FILTER_ALPHA * d_error +
                 (1.0f - LINE_D_FILTER_ALPHA) * s_d_filter;

    /* ------------------------------------------------------------
     * 3. 转向量
     * ------------------------------------------------------------ */
    steer = LINE_KP * error + LINE_KD * s_d_filter;
    steer = LineFollow_Clamp(steer, -LINE_STEER_MAX, LINE_STEER_MAX);

    /*
     * 转向量变化率限制。
     * 避免方向突变导致小车晃动。
     */
    max_steer_delta = LINE_STEER_RATE_MAX * LINE_DT;

    steer_delta = steer - s_last_steer;

    if (steer_delta > max_steer_delta)
    {
        steer_delta = max_steer_delta;
    }

    if (steer_delta < -max_steer_delta)
    {
        steer_delta = -max_steer_delta;
    }

    s_last_steer += steer_delta;
    steer = s_last_steer;

    /* ------------------------------------------------------------
     * 4. 基础速度斜坡
     * ------------------------------------------------------------ */
    ramp_delta = s_target_base_speed - s_ramp_base_speed;

    max_accel_step = s_accel * LINE_DT;
    max_decel_step = s_decel * LINE_DT;

    if (ramp_delta > max_accel_step)
    {
        ramp_delta = max_accel_step;
    }

    if (ramp_delta < -max_decel_step)
    {
        ramp_delta = -max_decel_step;
    }

    s_ramp_base_speed += ramp_delta;

    s_ramp_base_speed = LineFollow_Clamp(s_ramp_base_speed, 0.0f, 40.0f);

    base = s_ramp_base_speed;

    /* ------------------------------------------------------------
     * 5. 根据循迹误差适当减速
     * ------------------------------------------------------------ */
    base -= fabsf(error) * LINE_ERROR_DECEL_GAIN;

    if (base < 0.0f)
    {
        base = 0.0f;
    }
    /* ------------------------------------------------------------
     * 5.5 低速转向保护
     *
     * 低速时不能把转向完全限死，
     * 否则弯道时 base 被误差减速压低后，
     * steer 也会被压成 0，导致纠正力度很小。
     *
     * 这里改成：
     *   低速时允许一个最小转向能力。
     * ------------------------------------------------------------ */
    if (s_target_base_speed > 1.0f && base < 6.0f)
    {
        float max_low_steer;

        /*
         * base + 4.0f 是低速时允许的最小转向能力。
         * 例如 base = 2，则 steer 最大允许 6。
         */
        max_low_steer = base + 4.0f;

        if (max_low_steer > LINE_STEER_MAX)
        {
            max_low_steer = LINE_STEER_MAX;
        }

        if (steer > max_low_steer)
        {
            steer = max_low_steer;
        }

        if (steer < -max_low_steer)
        {
            steer = -max_low_steer;
        }

        s_last_steer = steer;
    }
    /*
     * 如果目标速度本身大于最低运行速度，
     * 说明不是停车阶段，可以限制最低速度防止卡死。
     *
     * 如果目标速度为 0，不要强行拉高。
     */
    if (s_target_base_speed > LINE_MIN_RUN_SPEED &&
        base < LINE_MIN_RUN_SPEED)
    {
        base = LINE_MIN_RUN_SPEED;
    }

    /* ------------------------------------------------------------
     * 6. 差速合成
     * ------------------------------------------------------------ */
    left_speed = base - steer;
    right_speed = base + steer;

    left_speed = LineFollow_Clamp(left_speed, -25.0f, 40.0f);
    right_speed = LineFollow_Clamp(right_speed, -25.0f, 40.0f);

    PID_SetDebugSpeedTarget_LR(left_speed, right_speed);
}
