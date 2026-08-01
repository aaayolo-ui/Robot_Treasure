# Robot_State

管理机器人运行状态，暂不实现具体状态机。

```text
待机 → 启动 → 循迹/导航 → 宝藏区域 → 声光提示 → 继续或结束
```

建议状态：`IDLE`、`TRACKING`、`TREASURE`、`AVOIDING`、`FINISHED`、`ERROR`。
