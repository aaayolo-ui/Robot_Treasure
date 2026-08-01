# Robot_Treasure

这是一个基于 STM32 控制的自主移动机器人项目，用于参加 **2026 江苏省大学生机器人大赛——密室夺宝项目**。

## 项目简介

|项目|说明|
|-|-|
|比赛名称|2026 江苏省大学生机器人大赛——密室夺宝项目|
|项目目标|开发能够自主循迹、检测敌方、获取宝藏并完成声光提示的移动机器人|
|当前开发状态|已完成项目方案、目录与文档体系建设，准备建立 STM32 工程|

## 总体方案

```text
STM32
  ↓
传感器采集
  ↓
算法处理
  ↓
电机控制
```

机器人采用：

- 四轮普通轮差速底盘；
- STM32 主控；
- 12 路灰度传感器循迹；
- 四方向红外检测；
- 强力磁铁获取宝藏；
- LED + 蜂鸣器声光提示。

## 硬件方案

|模块|方案|
|-|-|
|主控|STM32|
|底盘|四轮差速底盘|
|循迹|12 路灰度传感器|
|敌方检测|四方向红外|
|宝藏获取|强力磁铁|
|提示|LED + 蜂鸣器|

## 软件架构

```text
main.c
  ↓
Control 控制层
  ↓
Algorithm 算法层
  ↓
Hardware 硬件层
  ↓
STM32 外设
```

|层级|主要模块|
|-|-|
|Hardware|Motor、GraySensor、Infrared、Buzzer_LED|
|Algorithm|PID、Navigation|
|Control|Robot_State、Strategy|

## 项目目录

```text
Robot_Treasure
├── Docs
├── STM32
├── Hardware
├── Algorithm
├── Control
└── Test
```

## 开发环境

|工具|用途|
|-|-|
|VS Code|项目管理、Markdown、AI 辅助开发|
|Codex|代码和文档辅助|
|Keil5|STM32 工程编译下载|
|STM32CubeMX|外设初始化|
|Git/GitHub|版本管理|

## 当前开发进度

- [x] 项目方案确定
- [x] 项目目录建立
- [x] Git 版本管理建立
- [x] 文档体系建立
- [ ] STM32 工程建立
- [ ] 电机驱动开发
- [ ] 灰度传感器测试
- [ ] PID 循迹
- [ ] 比赛策略开发

## 开发计划

```text
项目文档完善
  ↓
STM32 工程建立
  ↓
硬件驱动开发
  ↓
循迹算法开发
  ↓
比赛功能测试
```
