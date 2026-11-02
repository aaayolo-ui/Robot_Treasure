# 16V STABLE1：从实测16U基线重建

本工程从C:/Users/Administrator/Desktop/Thief16U_UARTCh4_ImmediateRetreat700_Test的RUN8复制，保留原工程和MONITOR13。固件标识UARTCH4_16V_STABLE1。

本轮只做控制链收敛：正常巡线/前后板切换/转弯/倒车保留RUN8，去掉地图航向纠偏与多层探线接管；位置估计改为观察日志，路线执行仍由真实灰度和已规划动作推进。不会加入MONITOR13的8候选重定位控制。

保留四路激光前后1000mm、左右800mm及原取宝后启用规则，N011只走N010或N012-N013。100RPM/180ms入中心及正常PID/轮速参数保持。
失线恢复只复用基线灰度找线控制器，后板可提供实际线索；双板白底不做地图轴盲行，连续2秒双板无线停车。真实硬件错误/按键停车仍有效。

本版已完成ARMCC5编译（0错误、0警告）及5组针对性离线验证，尚未实车验证、未自动烧录。位置只是路线估计；漏认路口后的实际节点无法保证，取消定位停车不等于已具备绝对重定位能力。

打开MDK-ARM/Thief16V_UARTCh4_StableBase_Test.uvprojx，选择同名目标；HEX在MDK-ARM/Thief16V_UARTCh4_StableBase_Test/下。日志应出现FW=UARTCH4_16V_STABLE1。原串口监控协议保留，同一串口可观察原始激光、路线估计和轨迹事件，不需要刷激光专用测试固件。

5组验证入口：python -X utf8 Tests/run_stable1.py（需要unicorn和pyelftools，当前Tests/build/deps提供）。其中完整路线模型覆盖40种藏点/朝向/探头触发及4种途中拦截，均为模拟灰度、YAW、编码器；不能证明实车惯性、打滑、线宽或通信时序正常。

链接占用ROM63292字节、RAM44016字节，48KB RAM余5136字节；已分配3KB栈，链接器已知最大调用栈896字节，未知调用及中断叠加未做实车高水位测量。最终证据见VALIDATION_STABLE1.json和build_stable1.log。

源码角色和变更证据见Docs/修改规范与回归清单.md、STABLE_BASE_CHANGESET.patch。Reference保存RUN8源码、AXF与来源散列，不在桌面原工程上修改。Tests/build_stable1_source.py、finish_stable1_source.py、remove_map_veto.py是本次重建过程的历史脚本，不是日常构建入口，不要重跑覆盖当前源文件。
