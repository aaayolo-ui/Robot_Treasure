# STM32 Development Rules

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
