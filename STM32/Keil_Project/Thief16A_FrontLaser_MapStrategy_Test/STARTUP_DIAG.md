# 左后轮起步不转：诊断版本

日期：2026-10-10。

实车反馈：烧录 Thief16A 后左后轮持续不转，其他轮正常；激光 TX 接 PA3。尚未确认故障原因，不能据此认定电机损坏或串口抢占 PWM。

2026-10-11 后续版本已加入 [USART2 行驶隔离](LASER_DRIVE_ISOLATION.md)，当前不再只是记录日志：行驶前关闭 USART2、藏点停稳后重启。WD_HW 新增 APB1 字段；行驶时 USART2 时钟位 17 应为 0，U2CR1 显示 0 占位。

## 代码核对结果

- 左后 RL = 电机 D，PWM 为 PA2/TIM5_CH3，方向为 PB1/PA10。
- 初始 LEAVE 直行目标四轮都是 1000（100 RPM）；进入 FOLLOW 后按原循迹控制输出。
- USART2 为 RX-only，MSP 仅初始化 PA3，未把 PA2 配置为 USART2_TX。
- MotorDriver、Encoder、NewFrontDriveMap、ChassisSpeedControl、Chassis、TravelFrame、WheelSpeedPi 和 main.h 与全图标定工程逐字节一致。

## 怎样取得记录

1. 编译并手动烧录同名 Keil 工程的新 HEX；本次没有代为烧录。
2. 电脑打开现有串口接收，115200、8N1、文本模式。串口输出仍为 USART3/PB10，使用实际电脑端口。
3. 首次短测将四轮架空、手不接触轮子，灰度需能读到有效黑线，否则程序会拒绝启动。短按 KEY2；若左后轮仍不转，约 0.5～1 秒按 KEY1 停止，避免持续三轮拖行。
4. 停车后保持供电和串口接收，等待 `WD_BEGIN` 到 `WD_END` 输出完毕。截取整段记录给出诊断；不要复位，以免清空 RAM 记录。

这版不会自动结束短测，仍使用原按键停止和故障保护。若出现 `MAP_START_REQUIRES_LINE`，没有实际起步，不能用它判断电机输出。

## 记录含义

启动后的前 64 个样本以至少 50 ms 间隔保存，约覆盖前 3.2 秒；程序未停止时不输出新增日志。每条 `WD` 的 FL、FR、RL、RR 依次是：目标转速×10、测得转速×10、驱动 PWM、底层电气方向（0 停止、1 正、2 反）。左右电机的电气方向不同，不等于车辆方向错误。

对应 `WD_HW` 保存左后轮寄存器：

- `CCR3`：TIM5_CH3 比较值；`ARR`：周期值。
- `PA2_CFG`：PA2 的 GPIO 配置半字节，当前预期 A（2 MHz 复用推挽）。
- `CR1`、`CCER`、`CCMR2`：TIM5 运行与通道配置；`CCER` 的位 8 是 CH3 输出使能。
- `U2CR1`：USART2 控制寄存器，TE（位 3）应为 0；接收和中断等其他位按运行状态变化。
- `IN1`、`IN2`、`STBY`：GPIO 输出锁存值，不等于用仪器测到的实际引脚电压。
- `MAPR`：用于观察重映射配置；不应把该寄存器全部位都当作可靠的可读回状态。

初始 t=0 的目标/PWM 可能仍为 0；应检查后续实际控制样本。故障制动后的寄存器为零不代表制动前没有输出，所以本版本保留制动前的起步样本。

若 RL 目标和 CCR3 非零、配置正常但反馈为零，下一步测 PA2 PWM 以及驱动端信号/电机连接；仅凭寄存器不能证明物理 PWM 存在。若目标或 CCR3 异常，则沿软件输出链继续排查。暂不调整 PID 或更换电机方向映射。

## 本次修改文件

- 新增 `Hardware/DriveOutputSnapshot.c`、`.h`：只读寄存器快照。
- 新增 `User/StartupWheelLog.c`、`.h`：RAM 缓存及停车后分段串口输出。
- 修改 `User/App_ThiefStrategy.c`：增加头文件、启动记录和任务调用，共三个接入点。
- 修改 `MDK-ARM/Thief16A_FrontLaser_MapStrategy_Test.uvprojx`：登记新增模块。
- 新增本说明；更新 README、VERIFICATION、CHANGE_SUMMARY、FILE_MANIFEST、BASELINE_DIFF 和 BASELINE_SOURCE_SHA256。

ARMCC5 V5.06 update 5 build 528 全量构建为 0 Error(s), 0 Warning(s)。Code=34752、RO-data=4888、RW-data=352、ZI-data=15824 字节。未更改引脚、PID、速度、停止条件；未烧录或操作电机。仅编译通过，尚未完成硬件验证。
