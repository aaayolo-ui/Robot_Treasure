# Thief16I：多路线最坏情况推演实验

由16H独立复制，Keil工程为[Thief16I_RolloutMinimax_FourLaser_Test.uvprojx](MDK-ARM/Thief16I_RolloutMinimax_FourLaser_Test.uvprojx)。

**仅编译通过，尚未完成硬件验证**。

新增算法位于`Algorithm/ThiefStrategy/ThiefRollout.c`：有界候选路线、六种警方行为推演、maximin/leximin选择和乐观上界剪枝。`User/App_ThiefStrategy.c`在停车检测阶段记录入口线索，逃跑规划使用同一C模块；`ROLLOUT_V6`输出候选数量、实际推演数量、风险及入口推断标志。

起步取宝路线、两格检测、底层循迹、PID、引脚及移动时关闭激光的机制沿用16H。算法没有警方全局坐标输入。路线表放Flash，运行时不需要电脑、服务器或虚拟机。

修改`ThiefStrategyConfig.h`中的相关速度和时间参数后，在本工程目录执行：

```powershell
E:\Python\python.exe Tests\generate_topology.py
E:\Python\python.exe Tests\generate_rollout_data.py
E:\Python\python.exe Tests\run_algorithms.py
E:\Python\python.exe Tests\run_algorithms.py --uart
```

再在Keil中全量编译。不要只修改时间宏而继续使用旧查表；新生成器会嵌入参数检查。地图发生变化时，也需重新生成全部地图/拓扑数据并验证。

`THIEF_PLAN_BUDGET_MS=100`仍是板上规划超时保护；模拟器指令统计不能替代实际`COMPUTE_MS`。当前完整对照均最终被抓，一个近距离路口专项还出现退步，不能作为已验证实车夺宝策略。

详细算法和结果见[Docs/17](../../../Docs/17_有界推演与最坏情况逃跑16I.md)。
验证范围见[VERIFICATION.md](VERIFICATION.md)，逐文件清单见[FILE_MANIFEST.md](FILE_MANIFEST.md)，相对16H的代码见[SOURCE_DIFF.patch](SOURCE_DIFF.patch)。
