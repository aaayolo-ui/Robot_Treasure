# Thief16H 转向安全窗口实验

复制自 `Thief16G_MultiExitHide_FourLaser_Test`，保留四激光门控、运动控制、接线和取宝路线。修改只涉及纯C规划、离线拓扑缓存和停车规划日志，原工程保持原样。

本版排除前后出口抵达下一分岔前必须拐弯的藏点，并检查制动/转向/换向及起步离开路口的时间余量。无安全退路时仍可降级逃离，不承诺一定逃脱。

Keil入口：`MDK-ARM/Thief16H_TurnClearance_FourLaser_Test.uvprojx`。开机电机停止，原按键启动/停止流程保持不变，本次未烧录或测试电机。

参数在 `Algorithm/ThiefStrategy/ThiefStrategyConfig.h`。实车速度、PWM、角度、PID和控制周期未改变。算法在704状态上限内执行，不使用堆分配或递归。

[详细方案](../../../Docs/16_转向前后追尾与安全窗口16H.md)，[派生差异](SOURCE_DIFF.patch)，[全部新增文件](FILE_MANIFEST.md)，[验证范围](VERIFICATION.md)。

**仅编译通过，尚未完成硬件验证**。移动时激光仍按基线关闭，途中新出现的对手、安装偏移和惯性误差未验证；板上100毫秒规划预算需以实车串口日志确认。
