# 12 路灰度最外侧双黑转弯循迹试验

本工程独立复制 `Route15B_SmoothFollow_Test`，复用现有 NCHD12 I2C、前后灰度位置、四轮独立速度 PI、循迹 PID 与平顺控制。来自提供的 `TRACE.c`、`TracePid.c`、`LineFollow.c` 的可用思路是二值加权位置、状态驱动的循迹 PD、偏差降速、基准速度斜坡和转向变化率限制；原文件按 6 路 GPIO 和另一套电机接口编写，本工程按现有 12 路硬件接口实现这些逻辑。

## 操作与转弯

- 前灰度 P12 在车左、P1 在车右；后灰度 P1 在车左、P12 在车右。转弯标志只看前灰度。黑色为 1。
- KEY2 仅在前灰度 12 路全黑 `0xFFF` 时起步。当前离开黑区速度为 100 RPM；看到普通有效线后直接进入循迹，不重启四轮速度环。4 秒还未离开则停车报错。
- 循迹中，前灰度最左侧 P11 **与** P12 同时识别黑线时向左转，最右侧 P1 **与** P2 同时识别黑线时向右转。PID 和四轮速度控制仍为 20 ms 周期，另设 5 ms 前灰度路口监测，避免高速时在相邻两个控制采样之间越过窄路口。识别后先以 100 RPM 受控前进 200 ms；进入该阶段时清除四轮速度 PI 积分，避免继续沿用高速输出。200 ms 到时后的第一项动作是 TB6612FNG 短路制动。制动至少保持 80 ms，并用四轮编码器确认连续 2 次均低于约 2 RPM 后才允许转向；800 ms 仍未停稳则保持制动并报错，绝不释放车轮继续运行。制动期间仍以前灰度确认方向，右侧双黑或全黑锁定右转。随后以四轮固定 PWM 500 原地转向，相对 YAW 达到 50.00°后结束。仅前灰度参与转向保底，后灰度不参与。停止原地转向后，当前灰度可循迹时立即恢复 PID，否则以 100 RPM 直行找线；1.2 秒仍找不到线则主动制动并报错。
- 其余有黑线的范围继续用灰度 PID：线在 P5 或 P8 时，将最终差速修正提高到 400%；线触及 P1～P4 或 P9～P12、但未满足最外侧双黑转弯条件时，也用 400%；其余中心位置用 300%。前灰度位置偏差始终进入位置 PID，后灰度有效时只额外形成前后差的航向修正。四轮速度 PI 不变。
- 停转并切回循迹的同一帧就读取后灰度；后灰度有效则立即启用前、后双灰度位置和航向 PID，不再等待 3 帧。后灰度暂时无效时先以前灰度循迹，后续有效后再切入双灰度，避免无效读数控制电机。
- 两次转弯之间要求至少 3 帧有效循迹线，以免同一黑区重复触发。转弯最长 4 秒；运行最长 180 秒。KEY1、WK_UP 或运行中 KEY2 停车。前灰度一次读错、瞬间丢线或宽黑会先主动制动并在 200 ms 内重读，恢复后重新启动速度环；持续异常、真实堵转等故障仍保持制动。
- JY61P 转向轴按实车验证结果使用 YAW：车身左右转动时 YAW 连续变化，显示正负方向与实际一致。UART5 使用 PC12 接模块 RX、PD2 接模块 TX，默认 9600 8N1；接收使用 UART5 中断，避免 TFT/I2C 占用主循环时漏帧。相对角计算可跨越 ±180° 边界。
- TFT 的 YAW 数值按 200 ms 周期（5 Hz）刷新；YAW 变化不再触发整页重画，只更新必要行。状态和路口计数变化仍立即显示，减少软件 SPI 阻塞灰度采样的时间。
- USART3 调试文字使用 1024 字节环形缓冲非阻塞发送；日志拥塞时允许丢弃调试字节，绝不允许日志等待延迟灰度检测、接近控制或主动制动。FOLLOW、LEAVE、转弯和所有异常退出统一使用主动制动，不再通过关闭 STBY 滑行。
- 5 ms 前灰度安全监测除捕获正常转弯外，也会对丢线、无法解释的宽黑和前灰度 I²C 错误立即主动制动；不再因单次异常永久停机。转向中 YAW 短暂过期时同样先制动，600 ms 内恢复新帧则继续完成原目标角。TFT 的 STATE 下方显示最终故障原因。

## 调参位置

| 文件 | 参数 | 控制对象 |
|---|---|---|
| `Control/LineFollowPID.h` | `KP/KI/KD` | 前灰度位置误差；后灰度不会抵消前灰度 P5/P8 的偏差 |
| `Control/LineFollowPID.h` | `KH` | 前后灰度误差之差，即双灰度航向修正 |
| `Algorithm/SpeedPI/WheelSpeedPi.h` | `WHEEL_FL_B_*` | B 电机，左前轮速度 PI |
| 同上 | `WHEEL_FR_A_*` | A 电机，右前轮速度 PI |
| 同上 | `WHEEL_RL_D_*` | D 电机，左后轮速度 PI |
| 同上 | `WHEEL_RR_C_*` | C 电机，右后轮速度 PI |
| `Control/FollowSmoothing.c` | `SMOOTH_*` | 循迹基准速度斜坡、弯道降速和差速变化率 |
| `User/App_LineFollowExperiment.c` | `ROUTE_*` | 起步、左右侧转向触发、P1或P12保底停转、原地转弯速度、YAW目标角、转弯后找线及超时；中心/侧面灰度 PID 修正倍率 |

灰度控制不是每块灰度各一套 PID：前灰度始终负责位置误差；后灰度有效时，前后之差负责航向修正。四轮速度 PI 则各有独立状态和独立增益入口。以下数值以当前源码为准；用户反馈当前循迹版本已能在地图上以不错的速度和准确度运行。

## 当前参数与测试边界

| 设置 | 当前值 | 修改位置 |
|---|---:|---|
| 起步直行目标 | 100 RPM | `ROUTE_LEAVE_START_RPM_X10=1000` |
| 起步初值/斜坡 | 100 RPM | `ROUTE_LEAVE_INITIAL_RPM_X10=1000` / `ROUTE_LEAVE_RAMP_STEP_RPM_X10=10` |
| 循迹直线基准 | 220 RPM | `LINE_FOLLOW_PID_BASE_RPM_X10=2200`；控制周期 20 ms |
| 大偏差时最低循迹基准 | 120 RPM | `SMOOTH_CURVE_MIN_BASE_X10=1200` |
| 中心边缘/普通中心/侧面修正倍率 | 400% / 300% / 400% | `ROUTE_PID_*_GAIN_PCT`，只放缩灰度 PID 差速输出 |
| 转弯后循迹初值 | 100 RPM | `ROUTE_AFTER_TURN_START_RPM_X10=1000` |
| 转弯后直行找线超时 | 1.2 s | `ROUTE_AFTER_TURN_FORWARD_TIMEOUT_MS=1200` |
| 原地转弯四轮统一PWM | 500 | `ROUTE_TURN_EQUAL_PWM=500`；四轮幅值完全相同，左右仅方向相反 |
| 高速路口监测周期 | 5 ms | `ROUTE_JUNCTION_WATCH_PERIOD_MS=5`；只提高前灰度转弯捕获频率，不改变 20 ms PID 周期或直线速度 |
| 前灰度瞬时异常恢复 | 最多 200 ms | `ROUTE_FRONT_RECOVERY_MS=200`；先主动制动，连续 2 次读数恢复后重新启动循迹；持续异常报故障 |
| 识别路口后的受控前进 | 200 ms，100 RPM | `ROUTE_TURN_APPROACH_DELAY_MS=200` / `ROUTE_TURN_APPROACH_RPM_X10=1000`；进入时清除四轮 PI 积分 |
| 识别路口后的主动制动 | 至少80 ms，最长800 ms | `ROUTE_TURN_STOP_MS=80` / `ROUTE_TURN_STOP_MAX_MS=800`；编码器连续2次低于约2 RPM才允许转向 |
| 停车后等待新YAW帧 | 最多500 ms | `ROUTE_TURN_GYRO_WAIT_MS=500`；短暂串口间隔不再直接故障 |
| 转向途中YAW断帧恢复 | 最多600 ms | `ROUTE_GYRO_RECOVERY_MS=600`；先主动制动，收到新帧后继续原目标角 |
| JY61P转向目标 | 50.00° | `ROUTE_TURN_TARGET_ANGLE_X100=5000`；使用实车验证的 YAW |
| 灰度外环 Kp/Ki/Kd/Kh | 0.075 / 0.008 / 0.007 / 0.005 | `Control/LineFollowPID.h`，系数按 x1000 写入 |
| 差速修正每 20 ms 最大变化 | 25 RPM | `Control/FollowSmoothing.c` 中 `SMOOTH_STEER_STEP_RPM_X10=250` |
| B/A/D/C 四轮速度 PI Kp/Ki | 各 0.80 / 0.25 | `Algorithm/SpeedPI/WheelSpeedPi.h`，系数按 x100 写入 |
| 速度 PI 的 PWM 上限 | 800 | `WHEEL_SPEED_PI_PWM_MAX`；硬件定时器周期为 999 |
| 起步/运行 PWM 下限 | 100 / 50 | `WHEEL_SPEED_PI_PWM_START_MIN` / `WHEEL_SPEED_PI_PWM_RUNNING_MIN` |

## 速度与转向流畅度调节

数值名称带 `RPM_X10` 时，代码中的 `1000` 表示 100.0 RPM；角度名称带 `ANGLE_X100` 时，`5000` 表示 50.00°。建议每次只改一项并记录实车结果。

### 行驶速度

| 参数 | 当前值 | 作用与调大后的结果 | 位置 |
|---|---:|---|---|
| `LINE_FOLLOW_PID_BASE_RPM_X10` | 2200 | 正常循迹基准速度，即 220 RPM。调大后直线更快，但必须给轮速 PI 留出左右差速余量 | `Control/LineFollowPID.h` |
| `SMOOTH_CURVE_MIN_BASE_X10` | 1200 | 大偏差/弯道时最低基准速度为 120 RPM。调大后过弯更快但更容易冲出线；调小更稳 | `Control/FollowSmoothing.c` |
| `SMOOTH_BASE_STEP_RPM_X10` | 120 | 每 20 ms 基准速度最多改变 12 RPM。调大则加减速更快、冲击更明显 | `Control/FollowSmoothing.c` |
| `ROUTE_TURN_APPROACH_RPM_X10` | 1000 | 识别路口后的200 ms受控前进速度，即100 RPM | `User/App_LineFollowExperiment.c` |
| `ROUTE_TURN_APPROACH_DELAY_MS` | 200 ms | 路口进入时间；到时立即短路制动 | `User/App_LineFollowExperiment.c` |
| `ROUTE_AFTER_TURN_START_RPM_X10` | 1000 | 转弯结束后以 100 RPM 找线。调大恢复更快，但刚转完更容易再次冲线 | `User/App_LineFollowExperiment.c` |
| `ROUTE_LEAVE_INITIAL_RPM_X10` / `ROUTE_LEAVE_START_RPM_X10` | 1000 / 1000 | 起步和离开起点均为 100 RPM | `User/App_LineFollowExperiment.c` |

### 转向与循迹流畅度

| 参数 | 当前值 | 作用与调大后的结果 | 位置 |
|---|---:|---|---|
| `ROUTE_TURN_EQUAL_PWM` | 500 | 原地转弯时四轮共同的固定 PWM 幅值。调大转得更有力更快，但惯性和过转增加 | `User/App_LineFollowExperiment.c` |
| `ROUTE_TURN_TARGET_ANGLE_X100` | 5000 | 陀螺仪相对转角停止值，当前为 50.00° | `User/App_LineFollowExperiment.c` |
| `ROUTE_TURN_STOP_MS` / `MAX_MS` | 80 / 800 ms | 80 ms只是最短制动时间；四轮未确认停稳时继续制动，800 ms仍未停稳则安全故障 | `User/App_LineFollowExperiment.c` |
| `SMOOTH_STEER_STEP_RPM_X10` | 250 | 每 20 ms 灰度差速修正最多变化 25 RPM | `Control/FollowSmoothing.c` |
| `LINE_FOLLOW_PID_DEFAULT_KP_X1000` | 75 | 灰度位置误差的主要纠偏力度。调大转向响应更强，过大则左右摆动 | `Control/LineFollowPID.h` |
| `LINE_FOLLOW_PID_DEFAULT_KI_X1000` | 8 | 消除持续的单侧偏差。调大可修正长期跑偏，过大易积累并慢性摆动 | `Control/LineFollowPID.h` |
| `LINE_FOLLOW_PID_DEFAULT_KD_X1000` | 7 | 抑制快速变化和过冲。适当调大可减小摆动 | `Control/LineFollowPID.h` |
| `LINE_FOLLOW_PID_DEFAULT_KH_X1000` | 5 | 前后灰度差形成的车身航向修正。调大可更快拉直车身，过大易蛇形摆动 | `Control/LineFollowPID.h` |
| `LINE_FOLLOW_PID_MAX_CORRECTION_RPM_X10` | 500 | 灰度 PID 区域倍率前最大修正为 50 RPM；当前区域倍率可能进一步放大 | `Control/LineFollowPID.h` |
| `ROUTE_PID_CENTER_EDGE_GAIN_PCT` / `CENTER` / `SIDE` | 400 / 300 / 400% | 依次对应 P5/P8、普通中间区、P1～P4/P9～P12 | `User/App_LineFollowExperiment.c` |

### 四个轮子的速度 PI

`Algorithm/SpeedPI/WheelSpeedPi.h` 中 `WHEEL_FL_B_*`、`WHEEL_FR_A_*`、`WHEEL_RL_D_*`、`WHEEL_RR_C_*` 分别是左前 B、右前 A、左后 D、右后 C 电机。每个轮子的 `KP_X100=80`、`KI_X100=25` 控制该轮是否跟上目标 RPM，不是灰度循迹 PID。当前 `WHEEL_SPEED_PI_PWM_MAX=800`，起步/运行 PWM 下限为 100/50。

这里的 PWM 是四轮 PI 每个控制周期计算的输出，不是固定给电机的 PWM。任何配置都不能越过当前 PI PWM 上限 800。串口故障行现在包含故障前状态、YAW帧年龄、有效帧数、校验错误、UART错误和堵转轮号；若仍出现停车，记录完整 `ROUTE_FAULT` 行即可区分堵转、JY61P漏帧或其他状态故障。

用户反馈当前循迹版本已在地图上运行；本次仓库同步仅执行了编译检查，JY61P 重新接线后和四路红外避障尚未在本次验证。
