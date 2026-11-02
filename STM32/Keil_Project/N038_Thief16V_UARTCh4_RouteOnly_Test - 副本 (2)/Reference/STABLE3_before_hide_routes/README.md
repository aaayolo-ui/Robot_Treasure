# 16V STABLE3：N078临时避让与分流

本工程从C:/Users/Administrator/Desktop/Thief16U_UARTCh4_ImmediateRetreat700_Test的RUN8复制，保留原工程和MONITOR13。当前固件标识UARTCH4_16V_STABLE3；上一版STABLE2源文件、AXF、HEX和验证报告保存在Reference/STABLE2_before_temp78/，STABLE1备份也保留。

本轮只做控制链收敛：正常巡线/前后板切换/转弯/倒车保留RUN8，去掉地图航向纠偏与多层探线接管；位置估计改为观察日志，路线执行仍由真实灰度和已规划动作推进。不会加入MONITOR13的8候选重定位控制。

保留四路激光前后1000mm、左右800mm及原取宝后启用规则，N011只走N010或N012-N013。100RPM/180ms入中心及正常PID/轮速参数保持。
失线恢复只复用基线灰度找线控制器，后板可提供实际线索；双板白底不做地图轴盲行，连续2秒双板无线停车。真实硬件错误/按键停车仍有效。

STABLE3仍只改4个应用/策略源文件，基础16个Control文件和13个基础动作函数保持。已完成ARMCC5编译（0错误、0警告）及7组针对性离线验证，尚未实车验证、未自动烧录。位置只是路线估计；漏认路口后的实际节点无法保证，取消定位停车不等于已具备绝对重定位能力。

新增N078临时避让：N063西侧来车只走到N078，南北轴停车；正常180ms入中心后清除N063旧威胁/重规划锁存，重新收集N078检测。没有新威胁就一直等；新的北侧威胁先走N077→N076→N075，再去五藏点之一，N075只路过；新的西侧威胁先走N063→N052，继续去N063以北的藏点，不停在N063。N078不是通用搜索的第六个藏点，普通路线经过它不会自动停车。

方向全部是地图北/东/南/西，不是探头编号：N038西、北、南侧来车优先且固定先走N039→N040，东侧来车先走N021；之后仍用地图风险搜索选择五个藏点。N011上方来车向N010离开后，经N020到N037转向N038，不进入N055下方封闭环路。N011下方来车的N012→N013偏好保留。

行驶中只因行进前方障碍立即回撤（倒车时按真实行进方向判断）。侧面/后方来车保留已选路线；只有新检测的落点距离下一条实际路段不超过既有275mm接触膨胀半径，才到当前真实灰度路口后改道，不能仅凭“西侧见车、下一段也向西”截断路线。N061附近落点与N078→N077道路相隔600mm，不因此取消转弯。该几何筛选使用路线估计，仍有定位误差限制。原风险模型按敌车沿地图道路的最早可达时间评估，不把离开当前方向就当成不会被抓的保证。

打开MDK-ARM/Thief16V_UARTCh4_StableBase_Test.uvprojx，选择同名目标；HEX在MDK-ARM/Thief16V_UARTCh4_StableBase_Test/下。日志应出现FW=UARTCH4_16V_STABLE3；TEMP_HIDE N=78 AXIS=NS WAIT=FRESH_THREAT表示进入临时等待，ESCAPE_EXIT显示下一出口。原串口监控协议保留，同一串口可观察原始激光、路线估计和轨迹事件，不需要刷激光专用测试固件。

7组验证入口：python -X utf8 Tests/run_stable3.py（需要unicorn和pyelftools，当前Tests/build/deps提供）。test_stable1.py和test_strategy_stable2.py保留旧文件名作为原有回归，报告均绑定当前STABLE3 AXF。完整路线模型覆盖40种藏点/朝向/探头触发与4种途中触发；新增test_temp78_stable3.py有62项检查和2条完整生命周期：N063实际门控触发→N078巡线/居中/等待超过6秒→新北/西侧激光触发→抵达下一藏点。其--before用相同预期复现STABLE2问题。全部使用模拟灰度、YAW、编码器，不能证明实车惯性、打滑、线宽或通信时序正常。

链接占用ROM63928字节、RAM44008字节，48KB RAM余5144字节；已分配3KB栈，链接器已知最大调用栈896字节，未知调用及中断叠加未做实车高水位测量。最终证据见VALIDATION_STABLE3.json和build_stable3.log。

源码角色和变更证据见Docs/修改规范与回归清单.md、Docs/STABLE3_N078临时避让.md、STABLE3_CHANGESET.patch、STABLE_BASE_CHANGESET.patch。Reference保存RUN8源码、AXF与来源散列，不在桌面原工程上修改。Tests/build_stable1_source.py、finish_stable1_source.py、remove_map_veto.py是重建过程的历史脚本，不是日常构建入口，不要重跑覆盖当前源文件。
