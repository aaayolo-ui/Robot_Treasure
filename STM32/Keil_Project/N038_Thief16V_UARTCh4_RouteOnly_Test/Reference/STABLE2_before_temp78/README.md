# 16V STABLE2：稳定基础巡线 + 地图出口策略

本工程从C:/Users/Administrator/Desktop/Thief16U_UARTCh4_ImmediateRetreat700_Test的RUN8复制，保留原工程和MONITOR13。当前固件标识UARTCH4_16V_STABLE2；上一版源文件、AXF、HEX和验证报告保存在Reference/STABLE1_before_route_fix/。

本轮只做控制链收敛：正常巡线/前后板切换/转弯/倒车保留RUN8，去掉地图航向纠偏与多层探线接管；位置估计改为观察日志，路线执行仍由真实灰度和已规划动作推进。不会加入MONITOR13的8候选重定位控制。

保留四路激光前后1000mm、左右800mm及原取宝后启用规则，N011只走N010或N012-N013。100RPM/180ms入中心及正常PID/轮速参数保持。
失线恢复只复用基线灰度找线控制器，后板可提供实际线索；双板白底不做地图轴盲行，连续2秒双板无线停车。真实硬件错误/按键停车仍有效。

STABLE2只改4个应用/策略源文件，基础16个Control文件和13个基础动作函数保持。已完成ARMCC5编译（0错误、0警告）及6组针对性离线验证，尚未实车验证、未自动烧录。位置只是路线估计；漏认路口后的实际节点无法保证，取消定位停车不等于已具备绝对重定位能力。

方向全部是地图北/东/南/西，不是探头编号：N038西、北、南侧来车优先且固定先走N039→N040，东侧来车先走N021；之后仍用地图风险搜索选择五个藏点。N011上方来车向N010离开后，经N020到N037转向N038，不进入N055下方封闭环路。N011下方来车的N012→N013偏好保留。

行驶中只因行进前方障碍立即回撤（倒车时按真实行进方向判断）。侧面/后方来车保留已选路线；仅当下一出口将迎向敌车，才沿当前线走到真实灰度路口后改道。N076沿东西方向行驶时，北侧来车不再反复换向。原风险模型按敌车沿地图道路的最早可达时间评估，不把离开当前方向就当成不会被抓的保证。

打开MDK-ARM/Thief16V_UARTCh4_StableBase_Test.uvprojx，选择同名目标；HEX在MDK-ARM/Thief16V_UARTCh4_StableBase_Test/下。日志应出现FW=UARTCH4_16V_STABLE2，新增ESCAPE_EXIT行显示当前节点、下一节点和地图方向掩码。原串口监控协议保留，同一串口可观察原始激光、路线估计和轨迹事件，不需要刷激光专用测试固件。

6组验证入口：python -X utf8 Tests/run_stable2.py（需要unicorn和pyelftools，当前Tests/build/deps提供）。原test_stable1.py名称保留为基础控制回归，其报告已绑定当前STABLE2 AXF。完整路线模型覆盖40种藏点/朝向/探头触发及4种途中触发，均为模拟灰度、YAW、编码器；不能证明实车惯性、打滑、线宽或通信时序正常。test_strategy_stable2.py另外覆盖233项策略检查；--before运行相同预期，能在保存的STABLE1 AXF复现问题。

链接占用ROM63200字节、RAM44008字节，48KB RAM余5144字节；已分配3KB栈，链接器已知最大调用栈896字节，未知调用及中断叠加未做实车高水位测量。最终证据见VALIDATION_STABLE2.json和build_stable2.log。

源码角色和变更证据见Docs/修改规范与回归清单.md、STABLE2_CHANGESET.patch、STABLE_BASE_CHANGESET.patch。Reference保存RUN8源码、AXF与来源散列，不在桌面原工程上修改。Tests/build_stable1_source.py、finish_stable1_source.py、remove_map_veto.py是重建过程的历史脚本，不是日常构建入口，不要重跑覆盖当前源文件。
