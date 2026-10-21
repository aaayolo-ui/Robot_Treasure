# Thief16L_TerminalEscape_FourLaser_Test

独立复制16K的条件藏点实验，16B/16J/16K保留。N038主要藏点；N026仅在六假设下完成等待、检测、反应及第一次拐弯后脱身检查才准入；N062明确标为应急备用。当前对照未证明生存时间提升，没有N026准入案例。

## 工程与验证

打开 [Keil工程](MDK-ARM/Thief16L_TerminalEscape_FourLaser_Test.uvprojx)，ARM Compiler 5，F7编译。本次为 `0 Error(s), 0 Warning(s)`；Flash215460B、RAM36480B。

**仅编译通过，尚未完成硬件验证**。没有下载、操作电机或Git写操作。

117个Core/Hardware/Control/Drivers/System文件字节保持不变；MCU、时钟、引脚、ST-Link和运动参数沿用16K。应用灰度样本/缓存移入静态RAM，原逐次失效缓存及采样逻辑保留。`copy_reference.json`记录199个复制源。

指定取宝前缀 N054→N053→N044→N036→N035→N034→N033→N025→N015→N006→N005→N004 不变；初始藏点N038。四激光650mm、取宝后启用、定位、边缘激光和宝藏文字灰度屏蔽沿用16K。

## 新增模块

- `ThiefTerminalForecast.c/.h`：纯C有界终端推演，无HAL、堆或硬件I/O。
- `ThiefRollout.c`：枚举先返回，再检查至多两个终端候选；静态工作区防重入。
- `ThiefStrategyConfig.h`：`THIEF_TERMINAL_*` 参数；100ms规划预算保护未放宽。
- `TERMINAL_V9` 串口行：EMERGENCY、CHECKED、KNOWN、CLEAR、WAIT_MS、RESERVE_MS、CAND、STEPS。未知/未检查不能当成安全。
- `Tests/verify_16l_tests.py`：11模式231715项检查、0失败；`VALIDATION_16L_TESTS.json`保存各模式及哈希。
- `Tests/verify_16l_integration.py`：39项集成检查；复制、配置、地图数据、准入及栈采样来源验证。

算法工作区16222B，其中终端381B。原启动栈1024B，算法桥O1/O3块入口峰值772/768B；正式应用调用图752B加Unknown。未涵盖全部函数指针、实车中断嵌套、瞬时栈峰值与板上耗时，需实板验证。

[技术说明](../../../Docs/23_条件藏点与停车后脱身16L.md) · [回放](http://127.0.0.1:8765/variants/16l/replay.html) · [全部文件与差异](../../../Tools/TreasurePursuitSimulator/variants/16l/FILES_16L.json)

复制保留的 `Tests/verify_16k_integration.py` 是旧阶段脚本，不用于本版结论。
