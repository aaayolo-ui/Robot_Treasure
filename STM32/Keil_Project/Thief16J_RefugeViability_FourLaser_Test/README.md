# Thief16J_RefugeViability_FourLaser_Test

独立复制自16I，新增风险时间分档、完整转移模型覆盖数和下一次强制拐弯前的直行余地排序。纯C整数规划，无服务器依赖。取宝前缀、四激光接线、底层运动、传感器启停和100ms规划保护沿用原版。

[算法、数据、未采用方案和限制](../../../Docs/18_风险分档与藏点撤离余地16J.md) · [回放](http://127.0.0.1:8765/variants/16j/replay.html) · [文件清单](FILE_MANIFEST.md)

打开 `MDK-ARM/Thief16J_RefugeViability_FourLaser_Test.uvprojx`。本次没有自动烧录。

主要修改：

- `Algorithm/ThiefStrategy/ThiefRollout.c`：`RiskOrder`及其乐观剪枝。
- `Algorithm/ThiefStrategy/ThiefStrategyConfig.h`：`THIEF_ROLLOUT_RISK_TIERS=1`，`THIEF_ROLLOUT_TIME_BUCKET_MS=120`。
- `User/App_ThiefStrategy.c`：停稳决策日志 `RISK_TIERS_V7`，其中 `COVER` 是通过模型数，`BAND_MS` 是风险档，`RUNWAY_MS` 是较短方向的直行余地估计。

参数为规划参考值，须与控制工程和实测数据同步；120ms档位不是实测精度。沿用原579条只读路线。底层117个文件与16I逐字节一致。

Keil全量编译0错误0警告；详细验证见[VERIFICATION.md](VERIFICATION.md)。**仅编译通过，尚未完成硬件验证**。
