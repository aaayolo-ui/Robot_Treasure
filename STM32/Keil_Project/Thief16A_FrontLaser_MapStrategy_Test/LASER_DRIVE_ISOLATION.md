# 左后轮不转：隔离新增 USART2 的修正

日期：2026-10-11。

## 对照事实与原因判断

用户反馈：相同小车运行全图工程时起步正常，只有 Thief16A 左后轮持续不转。两工程的 MotorDriver、Encoder、Chassis、ChassisSpeedControl、TravelFrame、NewFrontDriveMap、WheelSpeedPi 和电机引脚定义一致。

影响左后轮外设环境的关键新增项是 USART2：全图工程不启用它，小偷工程从开机到行驶始终启用。左后轮 D 的 PWM 为 PA2/TIM5_CH3，PA2 同时是 USART2 默认 TX；PA3 为激光 RX。旧代码虽然为 RX-only、只初始化 PA3，仍让 USART2 本体持续工作。

PA2 的复用关系可查 [ST RM0008 USART2 remapping](https://www.st.com/resource/en/reference_manual/cd00171190-stm32f101xx-stm32f102xx-stm32f103xx-stm32f105xx-and-stm32f107xx-cortexm3-mcus-and-mpus-stmicroelectronics.pdf)。实物为兼容 APM32，尚无引脚波形或故障寄存器实测，不能把引脚共享本身当作“RX-only 必然阻止 PWM”的证明。USART2 与 PA2 输出路径的相互影响是本次针对的主要疑点，仍需修正版实车对照确认。

## 修正方式

当前策略仅在藏点停稳后依据激光触发逃跑，因此采用状态切换隔离，不改变接线：

1. READY 电机未运行时可以接收激光。
2. 每次 Route_Start（首次取宝和后续逃跑共用）先停止速度控制，调用 LaserRx_SetEnabled(0)。关闭 USART2 IRQ，终止接收、HAL 反初始化以关闭 UE/时钟，再复位 USART2 并清除挂起中断和旧帧。确认时钟已关闭，才继续使能底盘。
3. 行驶、转弯和藏点制动期间 USART2 保持关闭；停止软件解析也不会重新启动接收。
4. 确认藏点停稳后重新初始化 USART2，接收全新的帧，并重新积累检测证据。
5. 隔离失败不启动，报告 LASER_DRIVE_ISOLATION；恢复接收失败报告 LASER_RESTART_FAILED。

这把行驶时 USART2 的状态恢复为全图工程的条件。TIM5 参数、PA2 电机接线、四轮 PID 和目标速度均保持原值。激光数据在行驶期间不会更新；这与原先“仅藏匿状态触发激光策略”的功能范围一致。

起步日志新增 APB1 时钟寄存器，确认行驶时 USART2EN（位 17）为 0。U2CR1 在串口时钟关闭时显示 0 占位，不把门控后的外设读数当作有效测量。

## 复测与边界

重新烧录同名工程的新 HEX，先做短时间起步检查。左后轮恢复后还要验证“停稳后激光有新数据—触发倒车—到下个藏点恢复激光”的整个循环。若仍不转，保留 WD 日志，继续沿 PWM、方向与编码器反馈排查；本次不是已完成实车证明的根因结论。

仅编译通过，尚未完成硬件验证。

## 本次文件清单与差异

- Hardware/LaserRx.c、LaserRx.h：增加串口停用/恢复接口和关闭期间的接收保护。
- User/App_ThiefStrategy.c：在底盘使能前停用串口；藏点停稳后恢复。
- Hardware/DriveOutputSnapshot.c、DriveOutputSnapshot.h：增加 APB1 时钟快照。
- User/StartupWheelLog.c：输出 APB1，辅助核验隔离结果。
- 新增本说明；更新 README、VERIFICATION、STARTUP_DIAG、CHANGE_SUMMARY、FILE_MANIFEST、BASELINE_DIFF 和 BASELINE_SOURCE_SHA256。

整个实验工程尚未纳入 Git 跟踪，因此 git diff --stat 不显示其源文件差异。BASELINE_DIFF.patch 提供相对于全图工程的完整源码对照，FILE_MANIFEST.md 列出全部新增工程文件。未提交、未推送、未烧录或自动运行电机。
