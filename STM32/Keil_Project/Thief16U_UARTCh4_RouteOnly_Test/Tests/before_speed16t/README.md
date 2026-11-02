# 16U RUN4：直行速度连续、地图辅助灰度恢复、1000 mm 触发

本次直接修改用户指定桌面 16U 工程，启动标识 **UARTCH4_16U_RUN4**。工程目录和 Keil 工程文件仍保留原来的 `ImmediateRetreat700_Test` 名字；实际触发配置已改为 **1000 mm 中心距离**。

## Documents13 诊断依据

- RUN3 对每个路段前 100 mm 都限速到 100 RPM，包含直行通过路口后重新进入下一段，因此会重复减速、加速。
- 日志存在前灰度 `NO_LINE` 后制动、30～35 ms 后恢复的情况，会造成短暂停顿。
- 最后从 N036 转入 N035 路段约 8 mm 后，104859 ms 进入 `FRONT_WAIT`，约 215 ms 后触发 `FRONT_NO_LINE`。该记录仅能证明前板读数被判为无线，不能认定车身实际已偏出黑线。用户目测仍在线，这一物理观察没有被软件日志否定。

## 修改行为

1. **直行过路口维持连续速度。** 100 RPM 的低速获取阶段只在真实重新起步和转弯后启用。正常直行经过路口后，不再重新触发“每段前 100 mm 限速”。
2. **根据地图和实际传感器选择恢复方式。** 当前确认节点、正在行驶的地图边、行驶距离、定位状态和新鲜 YAW 有效，且航向误差在 5° 内时，前板 `NO_LINE` 可进入地图辅助恢复。距已确认出口 100 mm 内允许进行出口找线；更远的道路中段必须同时有实际后板有效黑线作为依据。
3. **受控找线。** 后板仍有线时，复用已有后板误差/YAW 闭环，以平均 50 RPM 获取前板；前板连续三次真实有效读数后恢复正常循迹。前后板都无有效线时先制动、确认轮速停止，再进行现有 ±4° 原地扫描，不盲目前冲。不会生成虚假的前板数据或把预测当作新节点确认。
4. **地图与读数不支持继续运动时停止。** 总恢复距离限制为 80 mm、总时间为 1000 ms；I2C 错误、未知航向、地图状态不可信时仍走原有制动重试。转弯尚未确认完成的阶段继续使用原对齐流程。新激光拦截可立即抢占地图辅助找线。
5. **增加可核查日志。** `MAP_GRAY` 输出当前节点、路段里程、预期为 `EXIT_LINE` 或 `ROAD_LINE`、前后原始灰度、YAW 误差、轮速目标与制动状态。这里的“预期”来自路线阶段，并不代表已标定的传感器几何覆盖模型。
6. **按用户要求将触发距离从 700 改为 1000 mm。** 原始有效距离上限仍为 1000 mm，保持现有侧向 +100 mm、后向 +150 mm、前向 ×0.9 的安装换算。侧向原始 900 mm、后向 850 mm 对应中心 1000 mm；前向受原始有效上限约束，原始 1000 mm 换算约 900 mm。串口明确输出 `TRIGGER_CENTER_MM=1000`。

五藏点与轴向、静止三帧激光确认、地图外过滤、刹车位移补偿、原有回撤和转弯策略均保留。未修改硬件引脚、时钟、地图拓扑、CubeMX 或 Keil 工程配置。

## 验证与实际边界

- 新增 139 项定向检查通过：直行下一段不再降至 100 RPM、前灰度缺失超过 200 ms 的受控恢复、正反行驶、双板无线时先制动再原地扫描、不可信上下文拒绝继续运动、恢复距离/时间限制和激光抢占。
- 原有应用检查 4980 项通过；Documents12 检查 2735 项通过，包含 240 种刹车位移补偿组合。
- 五藏点、两种停车朝向、四激光方向共 40 组合检查通过：36 个有地图道路的方向触发后完成路线，4 个无道路方向保持忽略；完整路线应用检查 23657 项通过。
- ARMCC5 算法和 1000 mm 距离边界/几何检查 365818 项通过。
- 指定工程 ARMCC5 全量编译：**0 Error(s), 0 Warning(s)**。Code 54324、RO 9576、RW 504、ZI 41384 字节。

离线测试执行实际链接 ARM 固件，硬件接口使用合成灰度、编码器、YAW、轮速和激光输入。地图坐标依赖原有编码器及灰度校正，不能独立证明物理位置绝对准确。此次没有验证实物打滑、传感器安装覆盖、机械回差或电机响应。

**仅编译通过，尚未完成硬件验证。** 未自动烧录、占用串口或驱动电机。

## 修改清单、备份和复核

修改：`User/App_ThiefStrategy.c`、`Algorithm/ThiefStrategy/ThiefStrategyConfig.h`、`Tests/test_16m.c`、`Tests/verify_snapshot.py`、本 README。

新增：`Tests/test_documents13.py`、`Tests/results_documents13.json`、`DOCUMENTS13_CHANGESET.patch`。构建日志、其他测试结果、`SOURCE_CHANGESET.patch`、`VALIDATION_16U.json` 同步更新。

RUN3 原文件、AXF、Documents13 原串口日志及修复前回归证据在 `Tests/before_documents13/`。RUN1/RUN2 的旧备份与 `DOCUMENTS11_CHANGESET.patch`、`DOCUMENTS12_CHANGESET.patch` 保留。`DOCUMENTS13_CHANGESET.patch` 是 RUN3→RUN4 增量，累计差异仍相对 16T。没有 Git 元数据，未提交或推送。

打开 `MDK-ARM/Thief16U_UARTCh4_ImmediateRetreat700_Test.uvprojx`。新 HEX/AXF 在同级 `Thief16U_UARTCh4_ImmediateRetreat700_Test/` 输出目录。烧录后核对 **UARTCH4_16U_RUN4** 及 **TRIGGER_CENTER_MM=1000**。

先编译本工程，再在工程目录运行：

```powershell
E:/Python/python.exe Tests/run_16m.py
E:/Python/python.exe Tests/test_app_runtime.py
E:/Python/python.exe Tests/test_escape_completion.py
E:/Python/python.exe Tests/test_documents12.py
E:/Python/python.exe Tests/test_documents13.py
E:/Python/python.exe Tests/verify_snapshot.py
```

最终审计核对源文件、工程引用、各测试脚本和测试固件 SHA-256，确保报告对应本次生成的固件。
