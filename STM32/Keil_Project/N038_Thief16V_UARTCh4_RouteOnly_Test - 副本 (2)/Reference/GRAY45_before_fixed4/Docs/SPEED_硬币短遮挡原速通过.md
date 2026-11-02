# TOP7_SPEED_SL1

按用户要求，短瞬间无线保持原速通过。只改E盘本工程，桌面未写入。基线TOP7完整静态文件和映像快照Reference/TOP7_before_coin_speed，298项散列。

Route_TreasureRearBridge原500L固定基准改成follow_smoothing.base_rpm_x10，沿用本次正常巡线实际使用的平滑基速。两侧目标为base±后板P修正（修正限200，即20RPM），平均目标严格等于base。正常巡线PID、速度、平滑及20ms目标更新周期不变。前板恢复后交回原巡线，无50RPM降速动作；不将旧的高于当前平滑速度的预设速度硬塞回控制器。

启用范围仍是首次1600路线N006→N004、正向FOLLOW、前板无线且后板真实紧凑线，最多50mm。没有真实后板线索则不辅助前进；双白两秒、KEY和其他原规则保留。普通路段、倒车、离开宝藏后的路线不放宽。真实前板RAW不伪造、地图游标不推进。此处基速是目标速度，不是实车实测轮速保证。

首次完整路线仍54→53→44→36→35→26→17→7→6→5→4→3→2→11，N011到点停稳后转90°朝西横停；N011北侧逃跑仍经12→23→31到63。激光前后1000/左右900mm、PA3声光保持。

生产只改User/App_ThiefStrategy.c中短遮挡辅助的基速来源及三处固件标识。ThiefMapPlanner及其他生产文件保持TOP7。Tests/test_top7_coin.py改为检查120/220/260RPM三种入口速度、三种后板偏差，要求左右目标为正、平均值等于入口速度且差速不超40RPM；50mm、前板恢复、非目标状态、双白两秒等原断言继续保留。

最初专项测试的循环变量与DWARF辅助函数base重名，导致夹具报int不可调用；改名后只重跑该专项通过，生产映像未改变。初次失败及重跑说明保存在speed_suites.json。原15组和专项共16组最终通过，全部报告绑定当前AXF。输入、陀螺仪和电机反馈仍是合成，不是实车测试。

ARMCC5全量编译0错误0警告。ROM68548、RAM44896字节，RAM不增加，48KB剩4256字节；预留栈3072、已知最大920字节，实际栈水位未测。证据VALIDATION_SPEED.json、SPEED_CHANGESET.patch、SPEED_FILES.json。复核：全量编译后Tests/run_speed.py，再Tests/validate_speed.py。

修改文件：User/App_ThiefStrategy.c、Tests/test_top7_coin.py、README.md、AGENTS.md、Docs/修改规范与回归清单.md。新增本说明、Tests/run_speed.py、Tests/validate_speed.py及SPEED构建/输入散列/报告/差异清单。每次更改前继续读本工程三份规范，旧50RPM说明不能作为现行规则。

仅编译通过，尚未完成硬件验证；未烧录、未占串口、未运行实物电机、未提交推送。

