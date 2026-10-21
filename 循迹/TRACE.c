/**********************************************************
 * TRACE.c
 *
 * 循迹传感器通用接口模块
 *
 * 本模块功能：
 * - 直接读取 6 路红外/灰度引脚
 * - 数字滤波
 * - 加权平均位置计算
 * - 循迹偏差计算
 * - 路线状态识别
 * - 丢线 / 全黑计时
 *
 * 本模块不包含：
 * - PID 计算
 * - 电机控制
 * - OLED 显示
 *
 * 后续 PID 文件只需要调用：
 *   Trace_GetError()
 *   Trace_GetState()
 *   Trace_GetLastValidError()
 *   Trace_IsLost()
 *   Trace_IsAllBlack()
 **********************************************************/

#include "TRACE.h"

/* ============================================================
 * 引脚接口
 *
 * 这里直接使用你原 GRAY.h 中的引脚定义：
 *
 *   GARY_PORT
 *   GARY_PIN_0_PIN
 *   GARY_PIN_1_PIN
 *   GARY_PIN_2_PIN
 *   GARY_PIN_3_PIN
 *   GARY_PIN_4_PIN
 *   GARY_PIN_5_PIN
 *
 * 对应关系：
 *
 *   bit0 = IO1，最左侧
 *   bit1 = IO2
 *   bit2 = IO3
 *   bit3 = IO4
 *   bit4 = IO5
 *   bit5 = IO6，最右侧
 *
 * 如果你的实际安装方向相反，
 * 可以在下面调整位序，或者在 PID 里反向。
 * ============================================================ */

#define TRACE_READ_IO1() \
    ((DL_GPIO_readPins(GARY_PORT, GARY_PIN_0_PIN) == GARY_PIN_0_PIN) ? 1u : 0u)

#define TRACE_READ_IO2() \
    ((DL_GPIO_readPins(GARY_PORT, GARY_PIN_1_PIN) == GARY_PIN_1_PIN) ? 1u : 0u)

#define TRACE_READ_IO3() \
    ((DL_GPIO_readPins(GARY_PORT, GARY_PIN_2_PIN) == GARY_PIN_2_PIN) ? 1u : 0u)

#define TRACE_READ_IO4() \
    ((DL_GPIO_readPins(GARY_PORT, GARY_PIN_3_PIN) == GARY_PIN_3_PIN) ? 1u : 0u)

#define TRACE_READ_IO5() \
    ((DL_GPIO_readPins(GARY_PORT, GARY_PIN_4_PIN) == GARY_PIN_4_PIN) ? 1u : 0u)

#define TRACE_READ_IO6() \
    ((DL_GPIO_readPins(GARY_PORT, GARY_PIN_5_PIN) == GARY_PIN_5_PIN) ? 1u : 0u)

/* ============================================================
 * 算法参数
 * ============================================================ */

/*
 * 6 路传感器中心位置。
 *
 * 位置范围：1 ~ 6
 * 中心值：3.5
 */
#define TRACE_CENTER            3.5f

/*
 * 无效位置。
 */
#define TRACE_POSITION_INVALID  0.0f

/*
 * 无效偏差。
 */
#define TRACE_ERROR_INVALID     (-999.0f)

/* ============================================================
 * 模块内部数据
 * ============================================================ */

static uint8_t s_raw = 0;
static uint8_t s_filtered = 0;

static float s_position = TRACE_POSITION_INVALID;
static float s_error = 0.0f;
static float s_last_valid_error = 0.0f;

static Trace_State_t s_state = TRACE_STATE_UNKNOWN;

static uint8_t s_active_count = 0;

static uint16_t s_lost_ticks = 0;
static uint16_t s_all_black_ticks = 0;

/* ============================================================
 * 滤波器历史数据
 * ============================================================ */

#if (TRACE_FILTER_SAMPLES > 1u)

static uint8_t s_filter_history[TRACE_FILTER_SAMPLES];
static uint8_t s_filter_index = 0;
static uint8_t s_filter_filled = 0;

#endif

/* ============================================================
 * 内部函数：读取 6 路原始状态
 * ============================================================ */

static uint8_t Trace_ReadRaw6(void)
{
    uint8_t raw = 0;

    if (TRACE_READ_IO1())
    {
        raw |= 0x01u;
    }

    if (TRACE_READ_IO2())
    {
        raw |= 0x02u;
    }

    if (TRACE_READ_IO3())
    {
        raw |= 0x04u;
    }

    if (TRACE_READ_IO4())
    {
        raw |= 0x08u;
    }

    if (TRACE_READ_IO5())
    {
        raw |= 0x10u;
    }

    if (TRACE_READ_IO6())
    {
        raw |= 0x20u;
    }

#if (TRACE_ACTIVE_HIGH == 0u)
    /*
     * 如果传感器低电平有效，
     * 则翻转低 6 位。
     */
    raw = (uint8_t)(~raw) & TRACE_RAW_MASK;
#endif

    return raw;
}

/* ============================================================
 * 内部函数：数字滤波
 *
 * 对 6 位分别做多数表决。
 * ============================================================ */

static uint8_t Trace_FilterRaw(uint8_t raw)
{
#if (TRACE_FILTER_SAMPLES <= 1u)

    return raw;

#else

    uint8_t out = 0;
    uint8_t bit;
    uint8_t i;
    uint8_t count;
    uint8_t threshold;

    s_filter_history[s_filter_index] = raw;
    s_filter_index++;

    if (s_filter_index >= TRACE_FILTER_SAMPLES)
    {
        s_filter_index = 0;
        s_filter_filled = 1;
    }

    if (!s_filter_filled)
    {
        return raw;
    }

    /*
     * 多数表决阈值。
     *
     * 如果 TRACE_FILTER_SAMPLES = 3，
     * threshold = 2。
     */
    threshold = (uint8_t)((TRACE_FILTER_SAMPLES / 2u) + 1u);

    for (bit = 0; bit < TRACE_SENSOR_COUNT; bit++)
    {
        count = 0;

        for (i = 0; i < TRACE_FILTER_SAMPLES; i++)
        {
            if (s_filter_history[i] & (uint8_t)(1u << bit))
            {
                count++;
            }
        }

        if (count >= threshold)
        {
            out |= (uint8_t)(1u << bit);
        }
    }

    return out;

#endif
}

/* ============================================================
 * 内部函数：统计有效传感器数量
 * ============================================================ */

static uint8_t Trace_CountActive(uint8_t raw)
{
    uint8_t count = 0;
    uint8_t i;

    for (i = 0; i < TRACE_SENSOR_COUNT; i++)
    {
        if (raw & (uint8_t)(1u << i))
        {
            count++;
        }
    }

    return count;
}

/* ============================================================
 * 内部函数：计算加权平均位置
 *
 * 返回值：
 *   1.0 ~ 6.0
 *
 * 如果丢线：
 *   返回 0
 * ============================================================ */

static float Trace_CalcPosition(uint8_t raw, uint8_t active_count)
{
    uint8_t i;
    uint16_t weight_sum = 0;

    if (active_count == 0u)
    {
        return TRACE_POSITION_INVALID;
    }

    for (i = 0; i < TRACE_SENSOR_COUNT; i++)
    {
        if (raw & (uint8_t)(1u << i))
        {
            weight_sum += (uint16_t)(i + 1u);
        }
    }

    return (float)weight_sum / (float)active_count;
}

/* ============================================================
 * 内部函数：状态判断
 * ============================================================ */

static Trace_State_t Trace_AnalyzeState(uint8_t raw, uint8_t active_count)
{
    if (active_count == 0u)
    {
        return TRACE_STATE_ALL_WHITE;
    }

    if (raw == TRACE_RAW_MASK)
    {
        return TRACE_STATE_ALL_BLACK;
    }

    /*
     * 如果只有最左两路触发：
     *
     * bit0 / bit1
     */
    if ((raw & 0x03u) != 0u && (raw & 0x3Cu) == 0u)
    {
        return TRACE_STATE_LEFT_EDGE;
    }

    /*
     * 如果只有最右两路触发：
     *
     * bit4 / bit5
     */
    if ((raw & 0x30u) != 0u && (raw & 0x0Fu) == 0u)
    {
        return TRACE_STATE_RIGHT_EDGE;
    }

    return TRACE_STATE_NORMAL;
}

/* ============================================================
 * 对外接口
 * ============================================================ */

void Trace_Init(void)
{
    uint8_t i;

    s_raw = 0;
    s_filtered = 0;

    s_position = TRACE_POSITION_INVALID;
    s_error = 0.0f;
    s_last_valid_error = 0.0f;

    s_state = TRACE_STATE_UNKNOWN;
    s_active_count = 0;

    s_lost_ticks = 0;
    s_all_black_ticks = 0;

#if (TRACE_FILTER_SAMPLES > 1u)

    s_filter_index = 0;
    s_filter_filled = 0;

    for (i = 0; i < TRACE_FILTER_SAMPLES; i++)
    {
        s_filter_history[i] = 0;
    }

#else

    (void)i;

#endif
}

void Trace_Update10ms(void)
{
    uint8_t raw;
    uint8_t filtered;
    uint8_t active_count;
    Trace_State_t state;
    float position;

    /*
     * 1. 读取原始 IO。
     */
    raw = Trace_ReadRaw6();

    /*
     * 2. 数字滤波。
     */
    filtered = Trace_FilterRaw(raw);

    /*
     * 3. 统计有效路数。
     */
    active_count = Trace_CountActive(filtered);

    /*
     * 4. 状态判断。
     */
    state = Trace_AnalyzeState(filtered, active_count);

    /*
     * 5. 位置计算。
     */
    position = Trace_CalcPosition(filtered, active_count);

    /*
     * 6. 保存基础数据。
     */
    s_raw = raw;
    s_filtered = filtered;
    s_active_count = active_count;
    s_state = state;
    s_position = position;

    /*
     * 7. 偏差和状态计时。
     */

    if (state == TRACE_STATE_ALL_WHITE)
    {
        /*
         * 丢线。
         *
         * 当前偏差使用上一次有效偏差。
         */
        if (s_lost_ticks < 0xFFFFu)
        {
            s_lost_ticks++;
        }

        s_error = s_last_valid_error;
    }
    else if (state == TRACE_STATE_ALL_BLACK)
    {
        /*
         * 全黑。
         *
         * 可能是十字、T 型、终点。
         *
         * 这里暂时把当前偏差置 0，
         * 但不更新 last_valid_error，
         * 以便后续路线逻辑可以结合丢线前方向判断。
         */
        if (s_all_black_ticks < 0xFFFFu)
        {
            s_all_black_ticks++;
        }

        s_lost_ticks = 0;
        s_error = 0.0f;
        s_position = TRACE_CENTER;
    }
    else
    {
        /*
         * 正常线。
         *
         * error = center - position
         *
         * 如果线偏左：
         *   position < 3.5
         *   error > 0
         *   车应左转
         *
         * 如果线偏右：
         *   position > 3.5
         *   error < 0
         *   车应右转
         */
        s_lost_ticks = 0;
        s_all_black_ticks = 0;

        s_error = TRACE_CENTER - position;
        s_last_valid_error = s_error;
    }
}

uint8_t Trace_GetRaw(void)
{
    return s_raw;
}

uint8_t Trace_GetFiltered(void)
{
    return s_filtered;
}

float Trace_GetPosition(void)
{
    return s_position;
}

float Trace_GetError(void)
{
    return s_error;
}

float Trace_GetLastValidError(void)
{
    return s_last_valid_error;
}

Trace_State_t Trace_GetState(void)
{
    return s_state;
}

uint8_t Trace_GetActiveCount(void)
{
    return s_active_count;
}

uint16_t Trace_GetLostTicks(void)
{
    return s_lost_ticks;
}

uint16_t Trace_GetAllBlackTicks(void)
{
    return s_all_black_ticks;
}

uint8_t Trace_IsLineDetected(void)
{
    return (s_active_count > 0u) ? 1u : 0u;
}

uint8_t Trace_IsLost(void)
{
    return (s_state == TRACE_STATE_ALL_WHITE) ? 1u : 0u;
}

uint8_t Trace_IsAllBlack(void)
{
    return (s_state == TRACE_STATE_ALL_BLACK) ? 1u : 0u;
}

void Trace_ResetCounters(void)
{
    s_lost_ticks = 0;
    s_all_black_ticks = 0;
}
