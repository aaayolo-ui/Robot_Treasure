# PA3上电持续声光提示

用户确认接线：声光模块VCC→3.3V、GND→主控GND、IN→PA3；上电后持续亮灯并鸣叫，实际断电后停止。厂家产品说明将其标为低电平触发：https://www.xinlucity.com/?s=resourcedetail/index/id/84.html 。照片板上标明3.3–5V。

当前工程Thief16V_UARTCh4_RouteOnly_Test，固件UARTCH4_16V_ROUTE6_SHORT_RETURN_SL1，基于最新ROUTE6_SHORT_RETURN添加。备份Reference/ROUTE6_before_power_sound_light。

PA3初始化完成后保持推挽低电平。不等待按KEY2；待机、行驶、KEY停车、正常躲藏或故障均保持，软件没有周期性蜂鸣/闪灯计时。不要把“停车”当成“断电”。模块实际断电自然停止；USB仍供电则继续声光。MCU上电/复位至初始化的短暂期间GPIO尚未配置，此时不能用这段固件保证IN电平。

## 文件变化

新增Hardware/PowerSoundLight.c、Hardware/PowerSoundLight.h。修改Core/Src/main.c（USER CODE Includes和2各一处）；修改MDK-ARM/Thief16V_UARTCh4_RouteOnly_Test.uvprojx（Hardware组新增.c/.h）；User/App_ThiefStrategy.c仅三处FW标识追加_SL1，其他内容与ROUTE6备份逐字一致。

更新README.md、AGENTS.md、Docs/修改规范与回归清单.md。新增本说明、Tests/verify_power_sound_light.py、VALIDATION_POWER_SOUND_LIGHT.json、POWER_SOUND_LIGHT_CHANGESET.patch及build_power_sound_light.log。Keil生成的AXF、HEX、map、htm和对象文件属于构建产物，未烧录。

## 验证和限制

ARMCC5全量编译0 Error(s), 0 Warning(s)。Tests/verify_power_sound_light.py专项16项检查覆盖：真实链接ARM/HAL初始化只打开GPIOA时钟、不动其他时钟；仅PA3配置为低速推挽；BSRR执行PA3拉低命令；保留相邻引脚配置；启动调用在策略初始化前；USART2未启用；Keil无重复引用；已有控制/策略代码、配置及备份对照；映像容量与固件标识。MMIO为模拟寄存器内存，不能当作物理电压、亮灯或蜂鸣效果证明。

新ROM68192字节（较ROUTE6增加96字节），静态RAM44888字节未增加。ROM/RAM仍在256KB/48KB能力内。基线15组行驶离线回归未为此GPIO功能重复运行，不将旧映像报告当成SL1通过证明。

仅编译通过，尚未完成硬件验证。待用户烧录后观察：复位初始化完成即持续亮灯鸣叫，待机/按键停车不关闭，模块实际断电才关闭。未自动烧录或驱动实物电机。
