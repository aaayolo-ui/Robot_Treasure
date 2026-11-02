# 16N：回撤重启找线与激光上传

本版复制自同级 `Thief16M_UARTCh4_InterceptEscape_Test`。16M 原工程保留。

## 本次修改

- `User/App_ThiefStrategy.c`：已定位在本次路线起点的重新出发，允许首次 `NO_LINE`。直行/倒车离开路口期间，以 100 RPM 最多找线 300 ms；读到有效线即进入循迹，持续无黑线才报 `RESTART_LINE_NOT_FOUND`。首次上电未定位的无黑线起步仍不放行。
- 需要转向的重新出发沿用原有 180 ms 入路口、刹停、转向流程，允许初始一次空白，但仍需后续分支和陀螺仪证据。
- 激光串口上传：每 25 ms 轮询输出一路，四路各目标 10 Hz；行驶、停靠均可上传。使用已有非阻塞整行入队接口，缓冲满时丢弃该条遥测，不等待发送、不停止车辆。实际电脑接收速率受主循环和串口负载影响。
- 探头测量配置、串口波特率保持原值，本次没有提高探头硬件测量频率；网页使用现有 LASER 格式，无需再改。
- 固定藏点、隐藏方向、先回撤再规划、地图边缘屏蔽、N005 后启用激光等沿用 16M。

## 文件与验证

工程：`MDK-ARM/Thief16N_UARTCh4_RestartRecovery_Test.uvprojx`。
固件：`MDK-ARM/Thief16N_UARTCh4_RestartRecovery_Test/Thief16N_UARTCh4_RestartRecovery_Test.hex`。

`Tests/test_app_runtime.py` 执行实际 ARMCC 链接固件，硬件接口由离线桩代替。覆盖前进/倒车无黑线重启、重新见线、持续无黑线截止、首次未定位起步、运动中四路上传节拍与发送队列满。
`Tests/run_16m.py` 保留原算法回归；`Tests/verify_snapshot.py` 检查源码、项目引用、编译和测试哈希，生成 `VALIDATION_16N.json` 与 `SOURCE_CHANGESET.patch`。

除应用源码外，修改的配套文件为本 README、两个测试/审计脚本、Keil 工程与选项名称；IOC 仅重命名，内容未改。Core/Hardware/Control/System/Algorithm 与 16M 保持一致。此目录没有 Git，变更以差异文件核对。

仅编译通过，尚未完成硬件验证。300 ms 找线时间需实车确认；未自动烧录、未运行电机。前探头故障仍按用户决定暂不处理。
