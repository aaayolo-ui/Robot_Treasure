# 验证记录 — 2026-10-06

**仅编译通过，尚未完成硬件验证**。

## 编译

- Keil MDK，ARM Compiler 5.06 update 5 (build 528)，全量 rebuild。
- 工程：`MDK-ARM/Gray12_MapFullRoute_Calibration_Test.uvprojx`。
- 最终结果：**0 Error(s), 0 Warning(s)**。
- Program Size：Code=30532，RO-data=7724，RW-data=296，ZI-data=7424。
- 本地构建日志 `MDK-ARM/build_full_map.log` 和 hex 为忽略的构建产物；没有执行 Flash Download。

## 离线验证

- `python -m unittest discover -s Tools/MapTimingLogger -p "test_*.py"`：**38 项通过**。包含原 31 项兼容测试、新 7 项连续路线/MT3 测试。首次运行有 1 项因沙箱临时目录权限失败，正常权限重跑全部通过。
- `python -m unittest discover -s Tools/TreasurePursuitSimulator -p "test_*.py"`：**107 项通过**。
- `python Tools/MapTimingLogger/verify_full_project.py`：**通过**。指定倒车源工程的 128 个模块文件 SHA 未变；新工程 Core、Drivers、Control、Hardware 与其逐字节一致。69 个 Keil 文件登记无重复、无外部工程依赖；Include 路径存在，IOC 除工程名外相同，设备/启动/编译器关键配置保持基线。
- 固定起点 N081、固定随机种子候选次序：1 条闭合路线，210 个识别步骤，106 条路段的 212 个地理方向各一次，名义 82.8 m。只有最终动作 STOP，其余端点均切换行驶方向。路口之间连接、方向、F/R 模式连续，输入顺序反转后计划及签名不变，生成 C 表与交付表一致。
- N005 未伪装成灰度路口；其两侧直道合并成测量区间。
- 合成 MT3 日志命令行端到端分析：212 个方向项、0 个地理覆盖缺失；24 个换向、118 个转弯均可分别识别。测试数据为人工构造，输出在忽略目录 `Tools/MapTimingLogger/results/SYNTHETIC_*`，**不是实车记录**。
- 错误模式、缺少/重复换向就绪、源端时钟回绕、损坏 CRC、故障结束、日志截断均有覆盖；前进和倒车速度分别计算，候选仿真配置不混入倒车样本。

## 仍需实物确认

1. 在 N081 朝 N 的摆放、前后灰度逻辑方向和第一个右转正确。
2. 需要返回时确实停稳，再切换新前端灰度；前进→倒车及倒车→前进都能正常找线。
3. 倒车中的左/右转与当前行驶方向一致，完整转后重新循线；T/十字和短路段不漏检、不重复计数。
4. STM32 节点顺序与真实车辆逐步对应，能最终回 N081；仅凭日志完整不能独立证明车辆物理位置正确。
5. 蓝牙长程日志 CRC、序号及队列丢失计数正常；运行结束后尾部记录能收全。
6. 实测地图长度/轮径尺度、前后传感器参考点偏移及重复运行波动后，再决定仿真参数精度。

未自动操作电机、未打开串口、未烧录、未提交或推送。没有改写历史阶段总结，也没有把合成日志记为硬件验证。
