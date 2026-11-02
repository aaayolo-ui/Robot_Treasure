# 16U RUN3：减少重复避让、起步出线和误判路口停车

直接修改用户指定的桌面 16U 工程，启动标识为 **UARTCH4_16U_RUN3**。700 mm 中心触发距离、1000 mm 原始有效距离上限、五藏点和停车轴向不变。保留 RUN2 的已确认节点起步支路修复。

## 日志结论

- Documents12 第 911 行：N038 由右激光 `D=380 C=60` 触发逃跑，前后有置信度为 0 的读数。不是定时自动换点；日志无法判断实际物体身份，也不能证明始终有障碍。
- 第 925、946 行：两次 `INTERCEPT_AT_START N=38 MM=0`，还没有位移就重新制动、规划，造成停顿。
- 第 1167 行：`MAP_NODE_DISTANCE_OR_ORDER`。N022 转向 N011 后约行驶 43 mm，就再次识别到支路形状；该 300 mm 路段的最早有效到达门槛为 70 mm，不应当作下一节点并锁死停车。
- 该日志没有记录用户另述的 N038→N037 出线。其物理原因尚未确认；代码中存在刹车滑行后仍补走完整入路口延时的问题，本次一并处理。

## 本次修复

1. 藏点静止时连续 **3 帧**有效近距离数据、距离跨度不超过 120 mm 才触发。无效帧或大于 100 ms 的帧间隔中断确认。50 Hz 数据约增加 40 ms 确认时间；行驶中的新拦截仍保持原快速判定。
2. 已纳入规划的同一侧向/后方目标，在首段前 100 mm 且前 500 ms 内不重复取消起步。行进前方、新方向、距离比原记录更近超过 100 mm 的侧向目标仍触发拦截。
3. 目标仅在行进后方时继续沿已选路线远离，不再每过一个路口就停车重规划。正反行驶均以实际行进方向判断。
4. 未到有效距离窗口的支路形状不触发节点到达，不增加路口计数、不修改坐标、不锁死停车；新路段重新清理路口触发状态。
5. 回撤停点的刹车位移用编码器计入，从原 180 ms、100 RPM 入路口动作中扣除。按现有名义轮径，30 mm 剩余约 92 ms，70 mm 无需再补走；正反行驶和零位移取消重启均覆盖。
6. 起步与转弯后找线种子速度降至约 **100 RPM**；对齐阶段及每段前 100 mm 的循迹平均目标速度上限为 100 RPM，左右轮按比例缩放保留曲率，之后恢复巡航。

保留地图外/无道路过滤、手动停止和必要的持续失线、电机、陀螺仪故障处理。未改地图拓扑、底层引脚、时钟、CubeMX 或 Keil 工程配置。

## 验证结果及限制

- 修改前 RUN2 的六项定向回归失败记录：`Tests/before_documents12/regression_before.json`；修复后通过。
- 2735 项定向应用检查通过，其中 240 种刹车补偿组合覆盖五藏点、N037、N078、可用入路方向、正反行驶和不同滑行距离。
- 原有 4980 项应用检查通过，含 508 种根节点转弯、128 种支路缓存组合。
- 五藏点 × 两种停车朝向 × 四路激光：36 个有道路方向触发后完成路线到下一藏点，4 个无道路方向按原规则忽略；四路再次拦截流程通过，共 23657 项检查。
- ARMCC5 算法回归 274234 项检查通过。
- 本工程 ARMCC5 全量编译：**0 Error(s), 0 Warning(s)**。Code 53420、RO 9492、RW 484、ZI 41380 字节。

应用测试执行实际链接 ARM 固件，但硬件接口使用合成灰度、YAW、编码器和电机反馈。补偿采用原有名义轮径，未进行实车标定。三帧确认减少孤立误测，不能识别物体身份；持续有效的近距离返回仍会触发避让。

**仅编译通过，尚未完成硬件验证。** 未自动烧录、连接串口或驱动电机，不能把离线通过当作任何实际情况下都不会出线的保证。

## 修改文件与备份

固件修改：

- `User/App_ThiefStrategy.c`
- `User/MapRoutePosition.c`、`User/MapRoutePosition.h`
- `Algorithm/MapRoute/MapRouteNavigator.c`、`Algorithm/MapRoute/MapRouteNavigator.h`
- `Algorithm/ThiefStrategy/FrontLaserGate.c`、`Algorithm/ThiefStrategy/FrontLaserGate.h`
- `Control/FollowSmoothing.c`、`Control/FollowSmoothing.h`

测试及说明修改：`Tests/test_app_runtime.py`、`Tests/test_escape_completion.py`、`Tests/test_16m.c`、`Tests/verify_snapshot.py`、本 README。新增 `Tests/test_documents12.py`、`Tests/results_documents12.json`、`DOCUMENTS12_CHANGESET.patch`；编译日志、其他测试结果、累计差异及 `VALIDATION_16U.json` 同步更新。

原 RUN2 文件、AXF 和 Documents12 原日志在 `Tests/before_documents12/`。更早证据在 `Tests/before_documents11/`，原 `DOCUMENTS11_CHANGESET.patch` 保留。`DOCUMENTS12_CHANGESET.patch` 是 RUN2→RUN3 的增量；`SOURCE_CHANGESET.patch` 是相对 16T 的累计差异。当前目录没有 Git 元数据，未提交或推送。

打开 `MDK-ARM/Thief16U_UARTCh4_ImmediateRetreat700_Test.uvprojx`。新 HEX/AXF 在 `MDK-ARM/Thief16U_UARTCh4_ImmediateRetreat700_Test/`。烧录后核对 **UARTCH4_16U_RUN3**。

先编译本工程，再依次执行：

```powershell
E:/Python/python.exe Tests/run_16m.py
E:/Python/python.exe Tests/test_app_runtime.py
E:/Python/python.exe Tests/test_escape_completion.py
E:/Python/python.exe Tests/test_documents12.py
E:/Python/python.exe Tests/verify_snapshot.py
```

验证报告包含测试固件 SHA-256，最终审计检查源码、测试记录和生成固件的一致性。
