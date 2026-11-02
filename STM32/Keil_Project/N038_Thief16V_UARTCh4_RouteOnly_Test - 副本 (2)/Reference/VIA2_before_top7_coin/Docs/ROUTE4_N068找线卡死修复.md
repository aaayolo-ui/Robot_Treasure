# ROUTE4：N068回程找线卡死修复与验证

当前固件：UARTCH4_16V_ROUTE4_RECOVER。
工程：MDK-ARM/Thief16V_UARTCh4_RouteOnly_Test.uvprojx。
HEX：MDK-ARM/Thief16V_UARTCh4_RouteOnly_Test/Thief16V_UARTCh4_RouteOnly_Test.hex。
修改前ROUTE3完整源码、映像、文档、测试快照：Reference/ROUTE3_before_recovery_fix，清单SNAPSHOT_SHA256.json。

## 现场与根因

用户确认小车确实位于N068。原始日志：C:/Users/Administrator/Desktop/小车串口_20261029_220511.txt，1248行起。
主控42995ms识别N068直行路口，42998ms进入恢复；后续反复GRAY_RECOVER RAW=007,8C0 LR=0,0 LOST_MS=0，保持ESCAPING/FAULT=NONE。

007被真实分类器判作侧边支路，即使是单簇3路窄线，也被ROUTE3回程恢复的VALID条件拒绝。8C0是分散黑块，亦不能作为可靠后板线。原RestartLineSearch因此进入制动后扫线过程。制动等待却没有刷新软件轮速缓存，Route_AllWheelsStopped会继续读取制动前转速，导致长期不释放制动。

ROUTE3旧AXF实跑复现两类失败，证据Tests/n068_before.json，旧AXF SHA256=0b80471cbe83528830b1b166bc9c6cfbf5bcd56863d6f39045d4d58482ae23a9。日志本身未直接记录缓存轮速/找线内部阶段；此因果由实际代码及带缓存轮速的离线复现补充确认。

## 本轮修改

生产代码仅修改User/App_ThiefStrategy.c：
- 制动找线分支刷新实际轮速，下一周期再据此判断停稳。行驶分支仍由ChassisSpeedControl_Update采样，不重复读取编码器。
- 只在回程已识别的直行路口恢复上下文，允许007/E00等单簇不超过4路的边缘线进入原50RPM灰度恢复控制。原控制参数保持。
- 该边缘证据不能提前提交路口。普通有效线连续3次后恢复MAP_PASS，由原直行确认流程推进一次，保留首次路口编码器锚点。
- 宽黑块/分散黑块不能据此前冲；未完成转弯时的侧线与后板旧线限制保持。双板无线2秒、KEY停止、真实故障仍保留。
- 恢复采样后读取当前时间，避免把本轮采样期间刚更新的YAW时间戳与更早时间比较。
- GRAY_RECOVER新增PH（0沿线、1制动等待、2原地扫线、3本轮搜索失败）、STOP（本次输入的停稳判定）、YAW_OK（新鲜YAW是否可用）。本轮搜索失败仍沿用原逻辑复位重试，不代表新增永久故障。
- 更新固件标识为ROUTE4_RECOVER。

没有修改Control、Algorithm、Hardware、Core、System、Drivers、Keil项目配置、激光阈值、普通转弯/停车动作或正常PID/平滑/轮速参数。13个基础动作函数和正常转弯段逐字对照；四路仍前后1000mm、左右900mm。

## 文件清单

修改：User/App_ThiefStrategy.c、README.md、AGENTS.md、Docs/修改规范与回归清单.md。
新增：Tests/test_n068_recovery.py、Tests/run_route4.py、Tests/validate_route4.py、本说明。
生成证据：build_route4.log、ROUTE4_CHANGESET.patch、VALIDATION_ROUTE4.json、Tests/route4_inputs.json、Tests/route4_suites.json、Tests/route4_*.log、Tests/n068_before.json、Tests/results_n068_recovery.json，以及13组当前AXF绑定的测试结果。
原Tests/run_route3.py等历史入口未改，不作为当前版验证入口。

## 验证结果

ARMCC5全量编译0 Error(s), 0 Warning(s)。13组回归全部通过。
新N068专项1228项检查，真实Route_ReadFront/Rear、GrayPosition/分类器、Route_AllWheelsStopped参与运行。模拟器只提供灰度原始值、编码器/轮子响应及其他硬件接口，不将任务或历史游标强设为成功。
覆盖正反行驶×左右边缘4种完整生命周期，均恢复后回到N038并竖停；每个恢复周期只采一次反馈；宽前板/分散后板不能盲目前进，实测停稳后能释放原地扫线；窄线恢复不消耗路口，接线后只推进一次。原16次完整历史返回及其他藏点/路线/激光/转弯/按键/双板无线回归也通过。

ROM=67856字节，RAM=44896字节，48KB中剩余4256字节。RAM含3072字节预留栈；链接器已知最大调用栈920字节，实车中断叠加/栈高水位未测。
AXF SHA256=daffa48b1f25a153a407b1d1c0609b3b0b793d4a46187df9f3f27ebc95f83801。

所有测试属于离线合成传感器/车体模型，不能证明真实惯性、安装偏差、轮滑或干扰下必定恢复。仅编译通过，尚未完成硬件验证。未自动烧录、占用串口、驱动电机或提交推送。
