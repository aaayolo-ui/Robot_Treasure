# STM32 Development Rules

## 平台与工程约束

## Toolchain

- Generate HAL projects with STM32CubeMX.
- Use Keil MDK-ARM with ARM Compiler 5.
- Use STM32F103RCT6 as the current compatible CubeMX target; the physical board uses the compatible APM32F103RCT6.
- Do not switch to the Standard Peripheral Library, LL library, or another compiler version without an explicit request.

## CubeMX Safety

- Do not casually change the MCU, clock, or pin configuration in `.ioc` files.
- Unless explicitly requested, do not modify `SystemClock_Config`, `MX_GPIO_Init`, `MX_USART1_UART_Init`, or `MX_TIM1_Init`.
- Put custom code in CubeMX `USER CODE` regions.
- Do not place user modules in `Drivers`, or move/rename `Core` or `Drivers`.
- After CubeMX regeneration, verify that custom code remains intact.

## Project Structure

- `Core`: CubeMX-generated entry point and initialization code.
- `Drivers`: vendor HAL and CMSIS code.
- `System`: reusable services such as UART debug output and software timers.
- `Hardware`: hardware drivers such as motors, keys, encoders, and sensors.
- `User`: application logic and test tasks.
- Prefer a dedicated `.c` and `.h` pair for each functional module. `main.c` performs initialization and task dispatch only.

## 强制分层与目录职责

总体原则：一个明确功能模块应有独立的 `.c/.h`，并放在职责正确的目录；Keil
Project Group 也应尽量对应实际目录，便于人工查找和修改。

- `Core/`：CubeMX/HAL 生成的外设初始化代码。不要随意手工重构生成代码。
- `Hardware/`：真实物理硬件驱动，原则上一种物理设备一组 `.c/.h`，例如
  `MotorDriver`、`Encoder`、`Key`、`NCHD12`、`UartDebug`。直接 HAL 硬件访问应
  集中在这一层。
- `Algorithm/`：纯算法，不直接操作 HAL，例如 `SpeedPI`、`GrayPosition`、
  `GrayCoordinate`。
- `Control/`：车辆级控制与状态逻辑，例如 `NewFrontDriveMap`、`LineFollowP`、
  `ChassisControl`。灰度差速和车辆运动映射不得堆入 `main.c`。
- `User/` 或 `App/`：当前实验/应用流程；负责调用各层模块、测试状态机、RAM 日志
  组织和串口输出流程，但不重复实现底层驱动或核心算法。
- `System/`：如 `SystemTime` 等系统级封装，同样使用清晰的 `.c/.h` 模块。

`main.c` 必须保持最小化，只保留 `HAL_Init`、时钟与 CubeMX 外设初始化、
`App_Init` 和 `App_Task`/主循环调用。禁止将电机驱动、编码器处理、灰度读取、
PI/PID、灰度位置计算、循迹差速、RAM 日志、按键业务状态机或大量 UART 格式化输出
直接写入其中。

## 车辆语义、安全与验证

- `MotorDriver` 底层的电气 `FWD/REV` 与车辆前后左右语义必须分层；Hardware
  层不写车辆方向语义。
- Algorithm 层不得直接访问 HAL；`main.c` 不承担业务逻辑。
- 每次代码阶段修改后应使用 ARMCC 5.06 全量编译，目标为 `0 Error(s), 0 Warning(s)`。
- 保留已经验证的 ST-Link、Flash、Keil 工程配置。

## Header Rules

- Every header must have an include guard.
- Declare public APIs in `.h` files and implement them in `.c` files.
- Never include a `.c` file to reuse code.
- Declare shared peripheral handles with `extern`; never define `htim1` or `huart1` in more than one source file.

## Keil Project Rules

- A `.c` file may appear only once in a `.uvprojx` file.
- When adding a source file, add it to exactly one appropriate Keil Group.
- When adding a header directory, add it once to Include Paths.
- Do not add `main.c` to more than one Group.
- After editing `.uvprojx`, check for duplicate `FilePath` entries.
- Name module Groups `System`, `Hardware`, and `User`.
- Preserve ARMCC 5, device selection, startup file, ST-Link, and Flash Download settings unless explicitly asked to change them.

## Motor Safety

- Motors must be stopped by default at power-on, with PWM set to 0 and STBY low.
- Stop a motor before changing direction; use only low duty cycles for the first test.
- Do not automatically run real motors, switch motor power, assume the motor-driver V3 interface direction, or recommend power-on without confirmed voltage, current rating, and polarity.

## Validation

After every STM32 project change:

1. Check for duplicate source files and duplicate-symbol risks.
2. Check Include Paths and confirm CubeMX-generated files were not moved incorrectly.
3. Attempt a Keil command-line build when available, and report error and warning counts.
4. Do not commit or push automatically.

If a deeper directory has its own `AGENTS.md`, preserve and follow its more specific rules.
