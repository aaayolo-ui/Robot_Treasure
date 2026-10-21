# DIN2 改接 PB15 对照工程

2026-10-16。独立复制自 `Thief16B_FrontUartOff_Compare_Test`，原工程保留。用户已确认 PB15 空闲、PB14 接 E2B；先前“PB15 接 E3A/E2B”的口述已纠正，不据此认定编码器接错。

**2026-10-16 已完成一次本工程实车路线验证**。用户人工烧录并将 DIN2 改接 PB15，实车反馈“非常不错，而且避障和藏点也没问题”。这是电机控制引脚隔离对照，前激光 USART2 仍关闭，不能作为完整四激光比赛策略。

[本轮实车记录](Evidence/20261016_hardware_capture.md)：COM9 收到新固件及 PA10/PB15 配置，取宝后 PA2 数字读回恢复与 PWM 相符；首条路线软件到达 N038，并由右侧检测触发转移至 N011。95组轮速/寄存器记录完整，253条 MT3 校验通过。仅覆盖本次路线和环境，不能单独区分前 USART2 关闭与 DIN2 迁移的贡献。

## 本轮只改变的运行接线

| 信号 | 旧对照工程 | 本工程 |
|---|---|---|
| 驱动板 D 通道 DIN2 | PA10 | PB15（所提供原理图 JP1 第25脚，以板上丝印为准） |
| MCU PA10 | 电机推挽输出 | 保持复位后的浮空输入，不再由电机驱动代码输出 |
| 左前 B 编码器 B 相 / E2B | PB14 | PB14，保持 |
| 前激光 USART2 | 关闭 | 关闭，保持 |

四轮映射、编码器读取、灰度/PID、260RPM目标、转弯PWM550、路线与另外三路激光处理均沿用父工程。未改写 `MX_GPIO_Init` 函数；它原有的 GPIO 初始化通过 `main.h` 宏配置 PB15，`.ioc` 同步迁移标签与引脚。时钟、MSP、IRQ、定时器和电机模块均保持原值。

用户报告断电测量：表笔短接0.1Ω，PA10到板载CH340C第2脚TXD也是0.1Ω且持续响，交换表笔后0.1Ω。这支持两者低阻直连，故避开PA10输出；不据此认定已经解释了取宝后PA2的异常。相对于最后一次失败的四激光实车运行，本工程同时包含前UART2关闭和DIN2迁移，若改善不能只归因于其中一项。父工程的前UART2关闭路线对照尚未收到有效数据。

## 手动验证顺序

1. 先保持车辆不运行。确认目前PB15确实空闲、PB14仍是E2B，不拆CH340或JC1/JC2。
2. 手动打开 [Keil工程](MDK-ARM/Thief16B_DIN2_PB15_Compare_Test.uvprojx)，F7检查编译后F8烧录；暂不按KEY2。
3. 完全断电（包括可能供电的USB/ST-Link/外接模块）后，仅将原来的DIN2控制线由PA10迁到PB15。若该线在测量时已从PA10拔下，接至PB15即可；驱动板DIN2这一端保持。不要移动PB14、PA15或其他编码器线。
4. 使用已经确认的原供电方式，人工RESET，保持车辆静止，通过实际枚举的串口、115200、8N1只接收日志。本轮USB转TTL重新插入后为COM9，此前为COM13。不得先按KEY2。
5. 检查开机名为 `Thief16B_DIN2_PB15_Compare_Test`，出现：

   ```text
   DIAG=DIN2_PB15 FRONT_UART2=OFF OTHER_THREE_ON
   PIN_ISOLATION DIN2=PB15 PA10_CFG=4 PB15_CFG=2 DIN2_LEVEL=0 STBY=0
   ```

   `PA10_CFG=4` 表示浮空输入，`PB15_CFG=2` 表示2MHz推挽输出。WM记录应PWM全0、STBY0；配置或接线不符时保持不运行并保留日志。
6. 收到并核对静止日志后，再另行指导人工短按KEY2试跑。异常立即人工KEY1停车；等TW_END导出完成。电脑不自动启动车辆。

## 核对与证据边界

- 新增的 `TreasureMotorSnapshot_ReadDIn2PinMap` 只读取GPIO配置和输入电平；只在开机打印，不向行驶主循环加入格式化工作。
- `Tests/verify_din2_pin_init.py` 从实际链接的ARMCC5 AXF执行GPIO、定时器/编码器/UART的MSP和新读回函数，使用CPU模拟寄存器核对PA10保持输入、PB15输出、PB14/PC10编码器输入及STBY默认关闭。它不连接实车或串口，不证明真实电压、轮速或路线已正常。
- [COPY_REFERENCE_DIN2.json](COPY_REFERENCE_DIN2.json)保留父工程输入哈希；[VALIDATION_DIN2.json](VALIDATION_DIN2.json)记录构建及核对结果；[FILES_DIN2.json](FILES_DIN2.json)列出全部新增文件和逐文件差异。

烧录、复位、按键启动和停车均由用户人工执行，电脑只接收日志；未提交或推送。本轮实车运行及 PA2 数字采样已取得正常结果；前激光关闭条件下的这一次通过，不等于完整四激光恢复、长期可靠性或驱动板端实际波形已验证。
