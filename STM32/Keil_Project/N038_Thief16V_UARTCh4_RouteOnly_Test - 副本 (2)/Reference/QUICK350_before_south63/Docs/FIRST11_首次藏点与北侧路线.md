# FIRST11_NORTH31_SL1

只修改E盘Thief16V_UARTCh4_RouteOnly_Test。用户明确要求桌面工程不要动；本轮对桌面仅作版本读取，未写入。E盘恢复后的当前基线为ROUTE6_SHORT_RETURN_SL1，并非此前聊天中的TREASURE26。完整修改前快照Reference/SL1_before_first11_north31，288项散列。

首次取宝到N004之前的路径不变。N004之后固定经过N003、N013、N012，到N011横停；没有把N005当成灰度路口或独立取宝传感器。

N011首次确认地图北侧来车，与原西侧分支共用固定通道：11→12→23→31→32→48→49→61→62→63。N063为东侧藏点，先居中停稳，复用原90°停车动作竖停。东侧来车仍走10→20→37→38→39→40；南侧来车仍走12→13→3→4→5→6→7及后续搜索。首次方向锁到藏点解除；原行进前方遇阻回退例外保留。没有新增每次转弯航向微调。

生产改动仅Algorithm/ThiefStrategy/ThiefMapPlanner.c及User/App_ThiefStrategy.c三处FW字符串。App除FW外逐字不变，Control/Hardware/Core/System/Drivers、Keil和全部其他算法不变。现有激光前后1000mm、左右900mm，PA3上电持续声光保留。

旧test_hide_routes_stable4.py和test_strategy_stable2.py仍断言北侧必须走N010，本轮按用户的新路线修改预期；初始藏点预期从N038改N011，原完整任务到点和停车轴检查保留。test_n011_first_plan_stable10.py覆盖四种车头方向×四种地图敌车方向、首次锁存、完整逃跑、到点解锁与下一轮触发；北侧按新N031通道检查。test_trace_front_probe.py原固定后激光CH4作为前路，但新路线改向东且正向行驶后实际行进前方是CH1，改成按当前TravelFrame选择前探头；保留“锁定路线仍能前方遇阻回退并返回N011”的断言。

初次15组中13组直接通过；上述两个旧预期/夹具测试失败。同步夹具后只重跑两项，均通过，生产映像不变。失败码与重跑说明保存在first11_suites.json，未用跳过测试或删除断言掩盖失败。

ARMCC5全量构建0错误0警告。ROM68152字节、RAM44888字节，RAM较基线不增加，48KB剩4264字节；含3KB预留栈。链接器已知最大920字节，实物栈水位未测。当前15组离线报告绑定最终AXF，证据VALIDATION_FIRST11.json、FIRST11_CHANGESET.patch、FIRST11_FILES.json。所有传感器和电机反馈均为模拟，不是实车通过。

本轮没有移入BRANCH1支路误停补丁、TREASURE26硬币遮挡辅助或LR1重定位。现有SL1本身的保护和灰度行为保持，后续如需改控制应另立明确范围，不能在纯策略改动中混入。

修改文件：上述2个生产文件，README.md、AGENTS.md、Docs/修改规范与回归清单.md；4个既有测试：test_strategy_stable2.py、test_hide_routes_stable4.py、test_n011_first_plan_stable10.py、test_trace_front_probe.py。新增本说明、Tests/run_first11.py、Tests/validate_first11.py及构建/散列/报告/差异文件。完整相对路径和散列见FIRST11_FILES.json。

复核命令：先ARMCC5全量构建，再Tests/run_first11.py，最后Tests/validate_first11.py。历史run_route6和声光身份验收脚本保留历史，不应用其旧身份断言替代当前验收。

仅编译通过，尚未完成硬件验证；未烧录、未占串口、未运行实物电机、未提交推送。实测核对BOOT标识FIRST11_NORTH31_SL1及真实首次N011、北侧触发首出口N012，不能只用网页轨迹判断实物。

