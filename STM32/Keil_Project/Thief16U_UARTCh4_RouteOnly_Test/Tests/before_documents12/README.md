# 16U RUN2：修复 Documents11 中逃跑重规划后的误停车

本次直接修改用户指定的桌面 `Thief16U_UARTCh4_ImmediateRetreat700_Test` 工程。启动标识为 **`UARTCH4_16U_RUN2`**，策略标识仍为 `ALGO=16U`，触发中心距离仍为 **700 mm**。

## 停车原因

Documents11.txt 第 2008 行记录 N063 触发逃跑；第 2124–2125 行记录回到 N078，灰度 `DF4`，`CENTERED=0`；第 2143 行附近显示 `NODE_BRANCH_REUSE N=78 ABS=2 ENTRY=2 PORTS=1`。重新规划的首段是 N078→N077，车身朝南，需要向右转往西，但保留的灰度支路证据只有东侧。第 2167 行因此触发 `MAP_REQUIRED_BRANCH_NOT_SEEN`，进入 FAULT。

这次支路证据没有丢失，而是只观测到了另一侧。确认到达路口不等于灰度板同时看到了所有支路。不能用“没读到所选支路”否定已确认节点上、地图校验通过的重新出发。

## 修改范围

- 只改一个固件源文件：`User/App_ThiefStrategy.c`。删除已确认节点重新出发时的重复支路否决条件，更新 RUN2 标识。
- 起步仍须通过运行时地图路线校验、当前确认节点匹配以及新鲜 YAW 检查；转弯之后仍须满足出口灰度稳定、YAW 方向与角度对齐，才正式进入下一路段。
- 继续使用已有 180 ms 入路口延时；已居中的藏点跳过重复前进。保留途中普通路口的支路确认、手动停止、持续失线恢复和电机/陀螺仪故障处理。
- 五个藏点及停车轴向不变：N011/N038/N035 横向，N026/N063 纵向。700 mm 触发、原有效测距范围、地图外及无道路过滤不变。
- 不修改引脚、时钟、CubeMX 配置或电机底层；未烧录，未连接串口，未驱动电机。

## 验证证据

- 原固件在新增 Documents11 回归中复现 `N078: MAP_REQUIRED_BRANCH_NOT_SEEN`，记录位于 `Tests/before_documents11/regression_before.txt`。
- 修复后同一回归能执行右转并到达 N077。
- 通过五藏点 × 两种合法停车朝向 × 四路激光的 40 组合检查：36 个方向有道路，真实激光判定和规划器均触发，并在合成灰度/YAW/编码器输入下完成整条路线、到达下一藏点且轴向正确；4 个方向没有地图道路，维持忽略，不伪造为小车。
- 四路激光的再次拦截完成测试：前后激光在已行驶路段触发；左右激光在路口起步、尚未位移时触发。覆盖制动、回撤或继续远离、停稳重规划、再启动及最终到达藏点。
- 无效起点、路段中途冒充节点、地图不存在的边仍被拒绝。
- 原有 4980 项应用检查、272703 项算法检查通过；包括 508 种根节点转弯与 128 种支路缓存组合。
- 对本工程执行 Keil ARMCC5 全量重编译：**0 Error(s), 0 Warning(s)**。日志 `build_16U.log`；完整校验及固件散列 `VALIDATION_16U.json`。

应用测试运行实际链接的 ARM 固件，硬件接口被替代为合成输入；没有验证实车打滑、灰度板安装偏差、真实电机响应或激光误判。**仅编译通过，尚未完成硬件验证**。不能据此承诺任何硬件状况下都一定成功逃跑。

## 文件与复核

打开 `MDK-ARM/Thief16U_UARTCh4_ImmediateRetreat700_Test.uvprojx`。本次生成的 HEX 与 AXF 位于 `MDK-ARM/Thief16U_UARTCh4_ImmediateRetreat700_Test/`。

- `Tests/test_escape_completion.py`：Documents11 回归、40 组合检查和完整逃跑路线测试。
- `Tests/results_escape_completion.json`：每个起点、朝向、激光、目标藏点、途中确认节点及测试固件散列。
- `Tests/test_app_runtime.py` / `Tests/results_app_runtime.json`：原应用回归，已更新已确认节点起步的预期行为。
- `Tests/run_16m.py` / `Tests/results_16m.json`：算法回归。
- `Tests/verify_snapshot.py`：源码、工程引用、测试固件一致性和文件改动审计。
- `DOCUMENTS11_CHANGESET.patch`：这次 RUN1→RUN2 的改动；`SOURCE_CHANGESET.patch`：相对 16T 的累计改动。
- `Tests/before_documents11/`：本次修改前文件、原 AXF、原串口日志与源码散列。保留原版本便于对比和回退。

在此工程目录依次运行：

```powershell
E:/Python/python.exe Tests/run_16m.py
E:/Python/python.exe Tests/test_app_runtime.py
E:/Python/python.exe Tests/test_escape_completion.py
E:/Python/python.exe Tests/verify_snapshot.py
```

请先在 Keil 编译本工程，再运行上述应用测试。新增测试可附加 `--documents11-only` 仅验证日志对应的起步问题。
