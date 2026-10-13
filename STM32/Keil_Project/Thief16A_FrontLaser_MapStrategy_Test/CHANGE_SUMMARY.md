# 修改摘要

2026-10-11 固定取宝路线更新：修改 Algorithm/ThiefStrategy/ThiefMapPlanner.c，将初始最短路选择替换为用户指定的 N053→N036→N033→N006→N005 顺序及沿途真实节点；取宝后的藏匿规划保留。修改 Tests/verify_thief_strategy.py 检查路段连接、指定转向及宝藏直穿；更新 README.md、VERIFICATION.md 和本摘要、BASELINE_DIFF.patch。ARMCC5 全量编译 0 错误、0 警告；未烧录、未运行电机。

- 新建独立工程 `Thief16A_FrontLaser_MapStrategy_Test`，不覆盖全图标定基线。
- 增加 USART2 RX（PA3，115200）和 AI-1811 接收。
- 合并 USART2 与 UART5 的 HAL 回调，避免激光与 JY61P 重复定义。
- 增加运行时地图路线导航，可执行 STM32 端生成的路线表。
- 增加 Dijkstra 初始取宝路线、藏点选择和反向逃跑路线。
- N005 按路段内点处理，强制一次直穿且不停车。
- 增加基于地图坐标、场界、合法道路和 3/5 稳定帧的激光过滤。
- 增加 TFT 策略状态、坐标、激光数据和判断原因显示。
- 电机仍由 KEY2 显式启动；上电默认停止，运行中任一按键制动。


## Startup diagnosis (2026-10-10)

See STARTUP_DIAG.md for this change: four new diagnostic module files, three application hooks and four Keil file entries. No motor control parameters or pin configuration changes. ROOT CAUSE UNCONFIRMED.

## USART2 isolation (2026-10-11)

See LASER_DRIVE_ISOLATION.md. USART2 is deinitialized, clock-gated and reset before motion; reception resumes after the hide stop is confirmed. 6 source/header files changed, no motor parameters or wiring changes. Build: 0 errors, 0 warnings. Hardware outcome pending.

## Source differences against full-route baseline

- modified: `Algorithm/MapRoute/MapRouteNavigator.c`
- modified: `Algorithm/MapRoute/MapRouteNavigator.h`
- modified: `Algorithm/MapRoute/MapRoutePlan.c`
- modified: `Algorithm/MapRoute/MapRoutePlan.h`
- added: `Algorithm/ThiefStrategy/FrontLaserGate.c`
- added: `Algorithm/ThiefStrategy/FrontLaserGate.h`
- added: `Algorithm/ThiefStrategy/ThiefMapPlanner.c`
- added: `Algorithm/ThiefStrategy/ThiefMapPlanner.h`
- modified: `Control/LineFollowPID.h`
- modified: `Core/Inc/stm32f1xx_it.h`
- modified: `Core/Src/main.c`
- modified: `Core/Src/stm32f1xx_hal_msp.c`
- modified: `Core/Src/stm32f1xx_it.c`
- added: `Hardware/DriveOutputSnapshot.c`
- added: `Hardware/DriveOutputSnapshot.h`
- modified: `Hardware/JY61P.c`
- modified: `Hardware/JY61P.h`
- added: `Hardware/LaserRx.c`
- added: `Hardware/LaserRx.h`
- added: `Hardware/UartRxDispatcher.c`
- added: `MDK-ARM/RTE/_Thief16A_FrontLaser_MapStrategy_Test/RTE_Components.h`
- modified: `MDK-ARM/Thief16A_FrontLaser_MapStrategy_Test.uvprojx`
- modified: `Thief16A_FrontLaser_MapStrategy_Test.ioc`
- modified: `User/App_ThiefStrategy.c`
- modified: `User/App_ThiefStrategy.h`
- added: `User/StartupWheelLog.c`
- added: `User/StartupWheelLog.h`
- added: `User/ThiefTftPage.c`
- added: `User/ThiefTftPage.h`

All project files: FILE_MANIFEST.md. Full source comparison: BASELINE_DIFF.patch.
