# 16U RUN7：藏点停车纠偏、转弯出口灰度恢复

当前固件 **UARTCH4_16U_RUN7**，直接修改本桌面16U工程。正常起步/转弯出口200 RPM与16T相同；1000 mm中心触发、灰度恢复±10°、五藏点与轴向保持原要求。

## Documents15定位与修复

N038停车日志显示 `CENTERED=1`，但YAW误差持续约13.9°。原来以入中心延时结束作为居中完成，未确认最终车身方向。本次保留与转弯前相同的平均 **100 RPM、180 ms** 入中心动作，在藏点入中心阶段加入YAW差速保持；制动后用原地纠偏将车身对齐道路。只有实际轮速停稳，且连续三个更新的YAW帧确认偏差在±2°内，才标记停车准备完成并开放藏点激光判断。±2°是停车姿态目标，灰度恢复仍是用户要求的±10°。

末尾N078反向右转约51.66°就进入FOLLOW，前板无线时由于定位仍在路口阶段PHASE=2，未进入原ON_LEG地图恢复，约211 ms后报FRONT_NO_LINE。本次对已确认路口的待完成转弯增加出口恢复：先按计划转向纠正至出口航向，再用真实前后灰度寻找出口；前板连续确认后恢复正常循迹，节点/路段切换仍须原有定位检查。覆盖正反行驶、两种YAW符号和首次转弯符号尚未学习的情形。

新增控制逻辑在 `Control/RoadHeadingControl.c/.h`，应用负责状态调度。纠偏前、旋转换向前均先制动并确认停稳；恢复使用既有2500 ms对齐时限，出口找线位移不超过80 mm。未删除真实I2C、定位、堵转故障处理。

可观察日志：`HIDE_ALIGN`、`HIDE_READY`含航向误差和编码器入中心距离；`TURN_EXIT_RECOVER`含节点、恢复阶段、真实灰度和行驶模式。延时和里程不能独立证明物理车中心精确重合路口中心，实际安装与滑行仍需实车校准。

## 验证及工程规则

- 原RUN6实际AXF复现停车姿态和转弯出口两项失败；原应用、AXF、串口日志、散列和失败记录保存在 `Tests/before_documents15/`。
- 新增Documents15回归4297项通过：41种停车场景、34种转弯恢复场景，以及控制器制动/新鲜帧/航向环绕边界。
- 既有应用、五藏点完整逃跑路线、Documents12/13/14及16T速度对比重新运行；最终数量与固件散列以 `VALIDATION_16U.json` 为准。
- ARMCC5全量编译 **0 Error(s), 0 Warning(s)**。Code56460、RO9672、RW544、ZI41416字节。

**仅编译通过，尚未完成硬件验证。** 离线回归执行实际ARM固件，传感器和轮速来自模型，不能替代实车对齐效果、打滑及传感器覆盖验证。未自动烧录、占用串口或驱动电机。

用户要求的修改规范位于 `Docs/修改规范与回归清单.md`，工程 `AGENTS.md` 已要求每次修改源码、参数或工程配置之前重新读取，包括同一轮中的后续修改。新故障必须保留修复前复现和受影响旧回归，报告须绑定本次固件。

本轮修改应用、Keil工程引用、应用/路线测试与审计脚本，新增控制模块、Documents15测试、AGENTS和修改规范；详细清单见 `DOCUMENTS15_CHANGESET.patch` 与最终报告。保留所有此前增量差异。

打开 `MDK-ARM/Thief16U_UARTCh4_ImmediateRetreat700_Test.uvprojx`；HEX/AXF仍在其同名子目录。文件名保留原样，启动标识应为 **UARTCH4_16U_RUN7**，实际触发为1000 mm。

复核顺序：全量编译，运行Tests中的test_app_runtime、test_escape_completion、test_documents12/13/14/15、test_speed16t，最后运行 `E:/Python/python.exe Tests/verify_snapshot.py`。纯算法报告365818项沿用源码散列相符的结果；新增控制模块由Documents15实际ARM用例覆盖。

以下为历史版本说明，当前行为以上述RUN7为准。

# 16U RUN6：修复Documents14在线误停车

本次直接修改用户指定桌面16U，固件标识 **UARTCH4_16U_RUN6**。保留16T正常速度、1000 mm中心触发和原有地图辅助灰度恢复。

## 本次原因与修改

Documents14中，54445 ms开始恢复时前后灰度均为000；54550 ms已读到前010、后E00，YAW误差为-4.66°；54610 ms误差为-6.51°，随即 `MAP_GRAY_REACQUIRE_FAILED`。当时恢复约166 ms、位移约27 mm，均未达到1000 ms或80 mm的上限。因此该次停止由新增的独立5°恢复门槛触发，不能认定小车物理偏出黑线；日志也显示前板已有黑线读数。

按用户指定，将灰度恢复航向范围从±5°改为 **±10°**，参数为 `ROUTE_MAP_GRAY_YAW_TOL_X100=1000`。正常定位的25°容差没有修改；陀螺仪新鲜度、实际前后灰度、地图步骤及恢复距离/时间约束继续有效。连续真实前板读数确认后恢复正常循迹，不制造前板数据、不虚构到达节点。

若恢复仍失败，新日志 `MAP_GRAY_END` 在制动之后打印具体 `CONTEXT/I2C/DISTANCE/TIMEOUT` 原因、当前前板原始值、灰度状态、定位状态、航向误差、已移动距离和持续时间，便于核对真实触发条件。

## 验证

- 原RUN5复现Documents14的恢复误停，以及合法航向变化被重复限制的情况，证据在 `Tests/before_documents14/regression_before.json`。
- 新增390项检查通过，其中44组覆盖正反行驶、两种YAW符号、±5°附近、±6.51°、±8°及指定±10°边界；实际前板重新读到线后均能恢复。
- 新增恢复过程±10.01°拒绝检查；超出原定位容差和陀螺仪过期仍由原定位逻辑拒绝。
- 正常速度输出与16T参考固件一致，47项速度检查通过；原应用4980项、Documents13的139项、Documents12的2735项以及五藏点路线23657项检查通过。
- ARMCC5全量编译 **0 Error(s), 0 Warning(s)**。Code54336、RO9672、RW504、ZI41384字节。

应用测试运行真实链接ARM代码，使用合成传感器与轮速输入。它证明软件状态和目标输出，不证明实车传感器覆盖、打滑或机械状态。

**仅编译通过，尚未完成硬件验证。** 未自动烧录、占用串口或驱动电机。

## 文件与交付

修改：`User/App_ThiefStrategy.c`、`Tests/verify_snapshot.py`、本README。新增 `Tests/test_documents14.py`、`Tests/results_documents14.json`、`DOCUMENTS14_CHANGESET.patch`；编译日志、已有测试输出、累计差异与验证报告同步更新。原RUN5应用、固件AXF、Documents14完整日志在 `Tests/before_documents14/`，其源文件散列和修复前失败记录一并保留。

新HEX/AXF仍在 `MDK-ARM/Thief16U_UARTCh4_ImmediateRetreat700_Test/`；工程名保留原样。烧录后核对 **UARTCH4_16U_RUN6**。最终报告为 `VALIDATION_16U.json`，增量差异为 `DOCUMENTS14_CHANGESET.patch`。

复核时先编译工程，再运行已有应用、完整路线、Documents12/13及速度测试，追加 `E:/Python/python.exe Tests/test_documents14.py`，最后执行 `E:/Python/python.exe Tests/verify_snapshot.py`。

以下为此前版本说明；涉及独立5°恢复条件的旧描述已由RUN6取代。

# 16U RUN5：恢复16T正常速度，保留灰度误停修复

本次按用户指定的16T速度恢复正常运动，直接修改桌面16U，固件标识 **UARTCH4_16U_RUN5**。1000 mm中心触发和地图辅助灰度恢复继续有效。

- 正常起步、倒车起步、转弯后起步种子目标恢复为16T的 **200 RPM**。
- 去掉RUN3/RUN4附加的正常循迹100 RPM上限及其状态变量；正常巡航、速度渐变、灰度增益和转弯参数与16T保持一致。
- 转弯前180 ms、100 RPM入路口动作属于16T原有配置，保持原值；没有修改该基准工程。
- 真正处于前板异常的地图辅助找线仍复用16T原有的50 RPM找线控制器；前板三次真实确认后恢复异常前速度，不再额外从50 RPM重新启动正常循迹。

验证中同时执行新固件与现有16T的实际ARM代码，在相同合成输入下，正向、倒车和转弯出口前三条目标曲线完全一致：212、224、236、248 RPM。该结果是软件目标输出一致，不是实车速度测量。

本次修改文件：`User/App_ThiefStrategy.c`、`Tests/test_documents12.py`、`Tests/verify_snapshot.py`、本README；新增 `Tests/test_speed16t.py`、两份 `Tests/results_speed16t*.json` 和 `SPEED16T_CHANGESET.patch`。构建日志、现有测试结果、累计差异和验证报告更新。RUN4原文件及AXF在 `Tests/before_speed16t/`；以前的修复差异保留。

新旧速度比较47项检查通过；应用、灰度恢复、五藏点路线回归结果见 `VALIDATION_16U.json`，最终审计核对它们与新固件一致。ARMCC5全量编译 **0 Error(s), 0 Warning(s)**。Code54204、RO9576、RW504、ZI41384字节。

**仅编译通过，尚未完成硬件验证。** 未自动烧录、占用串口或驱动电机。工程文件名仍是 `Thief16U_UARTCh4_ImmediateRetreat700_Test.uvprojx`，烧录后核对 **UARTCH4_16U_RUN5**。

复核速度：`E:/Python/python.exe Tests/test_speed16t.py` 和 `E:/Python/python.exe Tests/test_speed16t.py --baseline-16t`；然后执行既有应用、完整路线、Documents12、Documents13测试及 `Tests/verify_snapshot.py`。

以下保留RUN4修复说明，涉及正常速度的100 RPM描述已被上述RUN5变更取代。

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
