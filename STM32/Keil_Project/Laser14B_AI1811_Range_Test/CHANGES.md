# 新增文件与基线差异

整个 Laser14B_AI1811_Range_Test 为新增独立目录，旧12B工程无改动。构建产物由现有.gitignore排除。

ARMCC 5.06全量编译（含TFT）：0 Error(s), 0 Warning(s)，结果见本次重新生成的 `MDK-ARM/build_check.log`。仅编译通过，尚未完成硬件验证。

| 新增文件（相对本目录） | 来源/变化 |
|---|---|
| `.mxproject` | 沿用12B |
| `Core/Inc/main.h` | 沿用12B |
| `Core/Inc/stm32f1xx_hal_conf.h` | 沿用12B |
| `Core/Inc/stm32f1xx_it.h` | 沿用12B |
| `Core/Src/main.c` | 由12B修改 |
| `Core/Src/stm32f1xx_hal_msp.c` | 沿用12B |
| `Core/Src/stm32f1xx_it.c` | 沿用12B |
| `Core/Src/system_stm32f1xx.c` | 沿用12B |
| `Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xe.h` | 沿用12B |
| `Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f1xx.h` | 沿用12B |
| `Drivers/CMSIS/Device/ST/STM32F1xx/Include/system_stm32f1xx.h` | 沿用12B |
| `Drivers/CMSIS/Device/ST/STM32F1xx/LICENSE.txt` | 沿用12B |
| `Drivers/CMSIS/Include/cmsis_armcc.h` | 沿用12B |
| `Drivers/CMSIS/Include/cmsis_armclang.h` | 沿用12B |
| `Drivers/CMSIS/Include/cmsis_compiler.h` | 沿用12B |
| `Drivers/CMSIS/Include/cmsis_gcc.h` | 沿用12B |
| `Drivers/CMSIS/Include/cmsis_iccarm.h` | 沿用12B |
| `Drivers/CMSIS/Include/cmsis_version.h` | 沿用12B |
| `Drivers/CMSIS/Include/core_armv8mbl.h` | 沿用12B |
| `Drivers/CMSIS/Include/core_armv8mml.h` | 沿用12B |
| `Drivers/CMSIS/Include/core_cm0.h` | 沿用12B |
| `Drivers/CMSIS/Include/core_cm0plus.h` | 沿用12B |
| `Drivers/CMSIS/Include/core_cm1.h` | 沿用12B |
| `Drivers/CMSIS/Include/core_cm23.h` | 沿用12B |
| `Drivers/CMSIS/Include/core_cm3.h` | 沿用12B |
| `Drivers/CMSIS/Include/core_cm33.h` | 沿用12B |
| `Drivers/CMSIS/Include/core_cm4.h` | 沿用12B |
| `Drivers/CMSIS/Include/core_cm7.h` | 沿用12B |
| `Drivers/CMSIS/Include/core_sc000.h` | 沿用12B |
| `Drivers/CMSIS/Include/core_sc300.h` | 沿用12B |
| `Drivers/CMSIS/Include/mpu_armv7.h` | 沿用12B |
| `Drivers/CMSIS/Include/mpu_armv8.h` | 沿用12B |
| `Drivers/CMSIS/Include/tz_context.h` | 沿用12B |
| `Drivers/CMSIS/LICENSE.txt` | 沿用12B |
| `Drivers/STM32F1xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h` | 沿用12B |
| `Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal.h` | 沿用12B |
| `Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_cortex.h` | 沿用12B |
| `Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_def.h` | 沿用12B |
| `Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_dma.h` | 沿用12B |
| `Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_dma_ex.h` | 沿用12B |
| `Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_exti.h` | 沿用12B |
| `Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_flash.h` | 沿用12B |
| `Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_flash_ex.h` | 沿用12B |
| `Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_gpio.h` | 沿用12B |
| `Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_gpio_ex.h` | 沿用12B |
| `Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_pwr.h` | 沿用12B |
| `Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_rcc.h` | 沿用12B |
| `Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_rcc_ex.h` | 沿用12B |
| `Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_uart.h` | 沿用12B |
| `Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_ll_bus.h` | 沿用12B |
| `Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_ll_cortex.h` | 沿用12B |
| `Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_ll_dma.h` | 沿用12B |
| `Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_ll_exti.h` | 沿用12B |
| `Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_ll_gpio.h` | 沿用12B |
| `Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_ll_pwr.h` | 沿用12B |
| `Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_ll_rcc.h` | 沿用12B |
| `Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_ll_system.h` | 沿用12B |
| `Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_ll_usart.h` | 沿用12B |
| `Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_ll_utils.h` | 沿用12B |
| `Drivers/STM32F1xx_HAL_Driver/LICENSE.txt` | 沿用12B |
| `Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal.c` | 沿用12B |
| `Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_cortex.c` | 沿用12B |
| `Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_dma.c` | 沿用12B |
| `Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_exti.c` | 沿用12B |
| `Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_flash.c` | 沿用12B |
| `Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_flash_ex.c` | 沿用12B |
| `Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio.c` | 沿用12B |
| `Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio_ex.c` | 沿用12B |
| `Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_pwr.c` | 沿用12B |
| `Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc.c` | 沿用12B |
| `Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc_ex.c` | 沿用12B |
| `Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_uart.c` | 沿用12B |
| `Hardware/LaserRx.c` | 新增 |
| `Hardware/LaserRx.h` | 新增 |
| `Hardware/StationaryGuard.c` | 新增 |
| `Hardware/StationaryGuard.h` | 新增 |
| `Laser14B_AI1811_Range_Test.ioc` | 由12B修改 |
| `MDK-ARM/DebugConfig/Laser14B_AI1811_Range_Test_STM32F103RC_1.0.0.dbgconf` | 沿用12B |
| `MDK-ARM/Laser14B_AI1811_Range_Test.uvprojx` | 由12B修改 |
| `MDK-ARM/RTE/_Laser14B_AI1811_Range_Test/RTE_Components.h` | 由12B修改 |
| `MDK-ARM/startup_stm32f103xe.s` | 沿用12B |
| `README.md` | 新增 |
| `User/RangeTest.c` | 新增 |
| `User/RangeTest.h` | 新增 |
| `CHANGES.md` | 本清单 |

## 2026-10-10 TFT显示增量

- 修改 `Core/Inc/main.h` 与 `Core/Src/main.c`，配置板载TFT固定接口PB4～PB9。
- 新增 `User/RangeTftPage.c/.h`，以5Hz显示距离、可信度、GOOD、上一秒帧数和链路状态。
- 修改 `User/RangeTest.c`，把最新值及上一秒统计送入TFT，串口输出保持不变。
- Keil工程复用 `Gray12_MapFullRoute_Calibration_Test/Hardware/TFT.c/.h`，并加入对应Include Path；没有改动该已存在驱动。
- 修改IOC同步记录TFT六个GPIO，未改变USART2、USART3、时钟或电机静止引脚配置。
