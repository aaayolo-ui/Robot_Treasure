# 小偷正常转弯对照版 COMPARE1

固件标识 `UARTCH4_16U_POLICE_COMPARE1`。基于主聊天最终 MONITOR12 独立复制，原主工程未改。

## 本轮唯一控制变量

恢复 H:/Police17A_UARTCh4_LinePursuit_Test 的正常转弯退出和交接函数：相对起转角达到45度门槛（实际可能因采样越过），随即进入原 AFTER_TURN_FORWARD，下一次实际灰度决定巡线或找线。删除普通转弯地图轴粗转判断与额外驻车交接阶段；45度不是最终车身转角，不宣称接线已经完成90度转向。
正常左右PWM旋转、灰度接管状态分支与警察一致；保留MONITOR12真实PI重新启用修复和后板宽黑分类。

## 有意保持不变

小偷的N011世界方向出口优先、完整逃跑规划、行驶拦截回撤、五藏点停靠、前后1000/左右800mm激光、串口与网页协议、直道YAW参考学习、正常速度、100RPM/180ms入路口、双板无线2秒和定位恢复均继承MONITOR12。没有移植警察追捕/规划/重定位策略。
宝藏区仍用MONITOR12的N003—N007文字侧外四路屏蔽，并非警察N004—N006外两路。先单独比较转弯，不同时改变屏蔽；本版不能用来断言两种屏蔽等效。

## 打开与实测

打开 MDK-ARM/Thief16U_PoliceTurnCompare_Test.uvprojx，选择同名目标。HEX位于 MDK-ARM/Thief16U_PoliceTurnCompare_Test/Thief16U_PoliceTurnCompare_Test.hex。
按原小偷按键规则启动。先在同一路线、同一摆放姿态对比MONITOR12与COMPARE1：普通左右转弯、N004/N006两方向、N007出弯、正反行驶；再在相同位置拦截。记录实物出线位置、转弯方向及完整串口，尤其TURN_DONE、FRONT_WAIT、TURN_EXIT_RECOVER、MAP_NODE和最终FAULT。转弯遇敌仍由小偷原策略处理，不能以警察日志代替验证。

## 验证范围

结果见 VALIDATION_COMPARE1.json。历史VALIDATION_MONITOR*.json仅属复制基线，不是本版验证；旧test_general_turns/test_line_handoff等要求地图轴阶段的断言不适用COMPARE1，以新test_police_compare.py和报告为准。软件测试使用模拟传感器，不证明实物已捕线、居中、无过冲或定位正确。
仅编译通过，尚未完成硬件验证。未自动烧录、未运行电机、未占串口、未提交推送。

## COMPARE1本轮结果

ARMCC5全量编译0错误0警告。全地图1016种合法转弯、正反行驶/两YAW符号和根节点灰度分支共6374项对照检查通过；正常速度47项、宝藏区2950项、激光阈值442项、原始监控21项、真实PI重启3组、完整逃跑28768项及纯算法366155项/378组通过。保存MONITOR12上同一对照测试的3项预期行为差异；不是把基线标为故障版本。
生产源码仅User/App_ThiefStrategy.c变化。其余算法/控制/硬件/核心源码与MONITOR12散列一致；原主工程源码和AXF未改。107条Keil文件引用无缺失或重复。详见VALIDATION_COMPARE1.json，增量见POLICE_COMPARE_CHANGESET.patch。
