# 前激光恢复对照工程

2026-10-16，独立复制自已经完成一次实车路线的 [DIN2/PB15三激光对照](../Thief16B_DIN2_PB15_Compare_Test/README.md)，保留该工程和所有文件。**仅编译通过，尚未完成硬件验证**。父工程的实车通过不等于本工程通过。

## 恢复的内容

父工程为排查轮子异常，在初始化、接收重新挂载、回调和解析中跳过前激光。本工程恢复此前四路接收驱动，不只修改显示文字；前USART2在取宝后与另外三路一起接收，并保持跨藏点转移接收。每方向每轮最多处理32字节，沿用原校验、超时、错误恢复及地图过滤。

仅运行功能变量为前激光接收恢复。DIN2仍为PB15、PA10仍为输入、PB14/E2B不变；Core初始化、MSP、IRQ、四轮控制、PID、260RPM直行目标、转弯PWM550、取宝路线和藏匿算法均继承已通过父工程。应用中仅调整开机、启动和取宝状态文字；不把前激光提前到起步前启用。

## 接线和预期日志

沿用当前接线，无需为本工程挪线：前激光T/TX→PA3，R/RX悬空；左PB7、右PC11、后PB11；DIN2→PB15，PB14→E2B，日志输出PB10。供电保持已确认方案。

```text
Thief16B_FrontLaserRestore_Compare_Test ALGO=16J DIAG=FRONT_RESTORE DIN2=PB15 FRONT_UART2=ON_AFTER_TREASURE FOUR_RX
PIN_ISOLATION DIN2=PB15 PA10_CFG=4 PB15_CFG=2 DIN2_LEVEL=0 STBY=0
```

取宝前仍应显示WAIT_TREASURE；经过N005的软件取宝门槛后出现 `TREASURE_N005 LASER_RX=ALL_ON FRONT_UART2=ON RANGE_MM=650`，前向距离开始参与过滤与决策。65cm是有效目标判断阈值，不是传感器所有原始距离的输出上限；地图外侧方向继续屏蔽，不能把被屏蔽的方向等同于串口未启用。

## 验证结果与手动步骤

- Keil ARMCC5全量编译：0错误、0警告。
- 实际ARMCC编译的四通道接收代码CPU测试：1263项、0失败，覆盖前通道恢复、交错接收、错误校验、超时、溢出隔离、前UART错误恢复、初始化/接收失败回滚、停止后回调和每轮解析上限。
- 实际链接AXF的GPIO/MSP与被动引脚读回CPU检查：71项、0失败。它不会模拟真实芯片外设复用优先级，不能保证USART2启用后的PA2电气输出。
- 取宝启用与测距过滤的纯算法CPU测试：5947项、0失败。
- [COPY_REFERENCE_FRONT.json](COPY_REFERENCE_FRONT.json)与 `Tests/verify_front_restore.py`记录父工程哈希、继承差异及Keil配置核对；[VALIDATION_FRONT.json](VALIDATION_FRONT.json)记录构建与验证范围；[FILES_FRONT.json](FILES_FRONT.json)列出全部文件及差异。

人工下一步先打开 [Keil工程](MDK-ARM/Thief16B_FrontLaserRestore_Compare_Test.uvprojx)烧录，暂不按KEY2。之后重新打开电脑端COM9、115200、8N1的只接收采集，人工复位核对上述版本与PWM=0/STBY=0，再另行安排同路线试跑。试跑中若轮子异常或明显跑偏，人工立即KEY1停车并等TW_END，不能由电脑自动启动。

本轮构建未烧录、未驱动电机。相对已通过父工程只恢复前USART2，有助于检查此前PA2异常是否随此变量再次出现；尚未确定唯一根因，不将本工程称为四激光实车修复完成版。
