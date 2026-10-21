/**********************************************************
 * TracePid.h
 *
 * 循迹 PD 外环
 *
 * 功能：
 * - 根据 TRACE 传感器偏差计算左右目标速度
 * - 使用已有速度环执行
 * - 支持丢线处理
 * - 支持全黑处理
 * - 支持故障停止
 **********************************************************/

#ifndef TRACE_PID_H
#define TRACE_PID_H

#include "Header.h"

void TracePid_Init(void);

void TracePid_Start(void);

void TracePid_Stop(void);

uint8_t TracePid_IsRunning(void);

uint8_t TracePid_IsFault(void);

/*
 * 20ms 任务中调用。
 */
void TracePid_Update20ms(void);

float TracePid_GetSteer(void);

float TracePid_GetBase(void);

#endif
