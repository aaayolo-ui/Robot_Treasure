/**********************************************************
 * TRACE.h
 *
 * 循迹传感器通用接口模块
 *
 * 功能：
 * - 6 路红外/灰度传感器读取
 * - 数字滤波
 * - 加权平均偏差计算
 * - 路线状态识别
 * - 丢线 / 全黑计时
 *
 * 注意：
 * 本模块暂不包含 PID 控制。
 * 后续 PID 计算文件只需要调用本模块接口。
 **********************************************************/

#ifndef TRACE_H
#define TRACE_H

#include "Header.h"
#include <stdint.h>

/* ============================================================
 * 传感器数量
 * ============================================================ */

#define TRACE_SENSOR_COUNT          6u
#define TRACE_RAW_MASK              0x3Fu

/* ============================================================
 * 传感器有效电平
 *
 * TRACE_ACTIVE_HIGH = 1：
 *   检测到黑线时 GPIO 输出高电平。
 *
 * TRACE_ACTIVE_HIGH = 0：
 *   检测到黑线时 GPIO 输出低电平。
 *
 * 你原来的 GRAY 代码默认按高电平有效处理。
 * ============================================================ */

#ifndef TRACE_ACTIVE_HIGH
#define TRACE_ACTIVE_HIGH           1u
#endif

/* ============================================================
 * 滤波样本数
 *
 * 1：不滤波，响应最快
 * 3：多数表决，抗干扰较好
 *
 * 默认使用 3。
 * 如果小车速度较快、感觉响应迟钝，可以改成 1。
 * ============================================================ */

#ifndef TRACE_FILTER_SAMPLES
#define TRACE_FILTER_SAMPLES        3u
#endif

/* ============================================================
 * 路线状态
 * ============================================================ */

typedef enum
{
    TRACE_STATE_UNKNOWN = 0,

    /* 正常检测到线 */
    TRACE_STATE_NORMAL,

    /* 全部白，丢线 */
    TRACE_STATE_ALL_WHITE,

    /* 全部黑，可能是十字、T 型、终点 */
    TRACE_STATE_ALL_BLACK,

    /* 线在最左侧 */
    TRACE_STATE_LEFT_EDGE,

    /* 线在最右侧 */
    TRACE_STATE_RIGHT_EDGE
} Trace_State_t;

/* ============================================================
 * API
 * ============================================================ */

/*
 * 初始化。
 * 建议在 main() 中调用一次。
 */
void Trace_Init(void);

/*
 * 传感器更新。
 *
 * 建议放在 10ms 任务中调用：
 *
 * Task_10ms()
 * {
 *     Trace_Update10ms();
 * }
 */
void Trace_Update10ms(void);

/*
 * 获取原始 6 路状态。
 *
 * bit0 = IO1，最左侧
 * bit1 = IO2
 * bit2 = IO3
 * bit3 = IO4
 * bit4 = IO5
 * bit5 = IO6，最右侧
 */
uint8_t Trace_GetRaw(void);

/*
 * 获取滤波后的 6 路状态。
 */
uint8_t Trace_GetFiltered(void);

/*
 * 获取加权平均位置。
 *
 * 返回值范围：
 *   1.0 ~ 6.0
 *
 * 中心值为：
 *   3.5
 *
 * 如果丢线，返回 0。
 */
float Trace_GetPosition(void);

/*
 * 获取循迹偏差。
 *
 * 推荐符号约定：
 *
 *   error > 0：线偏左，车应该左转
 *   error < 0：线偏右，车应该右转
 *   error = 0：线在中间
 *
 * 正常范围大约：
 *   -2.5 ~ +2.5
 */
float Trace_GetError(void);

/*
 * 获取最近一次有效偏差。
 *
 * 丢线时，这个值可以用于保持上一次方向。
 */
float Trace_GetLastValidError(void);

/*
 * 获取当前路线状态。
 */
Trace_State_t Trace_GetState(void);

/*
 * 获取当前检测到的传感器数量。
 *
 * 返回值：
 *   0 ~ 6
 */
uint8_t Trace_GetActiveCount(void);

/*
 * 丢线计时。
 *
 * 单位：Trace_Update10ms() 调用次数。
 * 如果 10ms 调一次，则 1 = 10ms。
 */
uint16_t Trace_GetLostTicks(void);

/*
 * 全黑计时。
 *
 * 单位：Trace_Update10ms() 调用次数。
 * 如果 10ms 调一次，则 1 = 10ms。
 */
uint16_t Trace_GetAllBlackTicks(void);

/*
 * 是否检测到线。
 *
 * 返回：
 *   1：至少一路传感器检测到线
 *   0：没有检测到线
 */
uint8_t Trace_IsLineDetected(void);

/*
 * 是否丢线。
 *
 * 返回：
 *   1：全部白
 *   0：非全部白
 */
uint8_t Trace_IsLost(void);

/*
 * 是否全黑。
 *
 * 返回：
 *   1：全部黑
 *   0：非全部黑
 */
uint8_t Trace_IsAllBlack(void);

/*
 * 清除丢线 / 全黑计时。
 *
 * 例如按键启动循迹前，可以调用一次。
 */
void Trace_ResetCounters(void);

#endif
