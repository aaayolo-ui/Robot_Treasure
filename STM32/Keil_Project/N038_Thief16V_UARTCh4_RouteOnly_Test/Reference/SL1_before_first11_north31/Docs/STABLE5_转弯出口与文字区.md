# STABLE5：转弯出口交接与弱灰度恢复

用户确认实车已在N004北侧宝藏区文字上。23:01日志最后一轮N006左转起角-3015，主转结束1632，累计46.47°；其后同一YAW值下就确认出口并推进6→4。后续3529/4055/4544对应65.44°/70.70°/75.59°，随后前板无线、后板001，恢复输出300/700（均值50RPM）继续前推，最后双白计时。该日志没有FW启动行，也未保存每帧原始灰度；不能断言具体哪个字被误识别，不能把地图上估计的横线位置当成实际坐标。

在保存的STABLE4 AXF上，用该轮实际YAW序列和合成线样/碎片灰度输入可复现：45°交接后首个有效前板读数授权前进，几次前板有效即可推进地图；后板外侧单点也能支持普通恢复前推。新测试使用完整6→5→4直线段计数（执行步为6→4），没有跳过结构校验或强设成功状态。

## 生产改动只有User/App_ThiefStrategy.c

1. 保留45°主转、PWM550、100RPM/180ms入中心和正常200RPM巡线参数；主转结束只进入出口接线，不再输出原先第一拍直行。
2. 出口接线保持原地，实际YAW相对入弯角与已知左右转方向换算，目标约90°；无配对线时在80/100°目标之间小范围原地扫线，输出左右轮反号、纵向目标为零。不是基于地图全局航向或里程纠偏，也不新增±2°停稳精调。
3. 放行条件：实际转动80～100°，前后板均为有效紧凑线迹，归一到车体后的前后偏差相差不超过200（两个通道权重间距，不是200mm），连续3个控制采样满足。线迹可横向偏移，剩余误差仍交原PID，不强求两板居中。
4. 紧凑线迹使用内部P3～P10中的单段连续1～4个黑点；外侧P1/P2/P11/P12单独读数或不连续碎片不能授权恢复平移。普通RestartLineSearch仍保留，但输入前/后“有可信线”必须经过该筛选。因此前000后001不再单独前推；真正前板线迹恢复时仍可在后板短暂无线时回到原PID。
5. 出口接线期间位置估计对激光投影无效，原始测距仍可显示；确认出口后才进入FOLLOW，再通过正常后续线样本推进地图。不保留原1.2秒超时跳入普通恢复的旁路。状态7历史名字AFTER_TURN_FORWARD为协议兼容保留，实际是原地接线；TURN_DONE表示主转结束，TURN_LINE_READY才是出口接线完成。
6. 连续真实前后双白2秒仍停车；灰度I2C持续失败仍按原200ms故障处理，陀螺仪失效、按键、原堵转规则保留。不会因为地图LOC_DISTANCE或全局航向不符新增锁死。

## 保留、验证与限制

16个Control文件、13个基础动作函数、正常PID→平滑→四轮控制段及其参数与RUN8对照一致。相对STABLE4，所有其他生产源文件散列不变；N038竖停、N011指定全路线/N063停车旋转、N078临时避让等策略保留。TreasureGrayMask没有扩大范围、没有给N004/N006单独加新的电机动作。STABLE4源码、AXF/HEX和验证证据保存在Reference/STABLE4_before_turn_exit/。

Tests/run_stable5.py为当前9组入口。新增120项转弯/恢复检查覆盖：实际YAW片段、左右转/前进倒车/两YAW符号及跨±180°、碎片中断连续确认、偏移一致线迹交PID、弱后板不平移、正确线迹恢复及双白/I2C故障。其他8组仍通过，完整路线40种触发、N078两条生命周期、首次N038和N011→N063共7条生命周期均绑定同一最终AXF。旧版复现与新结果见VALIDATION_STABLE5.json，当前源码差异见STABLE5_CHANGESET.patch。

编译0错误、0警告；ROM65484字节、RAM44016字节（48KB余5136），3KB栈已计入；链接器已知最大栈904字节加Unknown项，未做实车栈高水位测试。仅编译通过，尚未完成硬件验证；未自动烧录/运行电机/占用串口。

这不是文字识别器或绝对重定位。文字也可能形成与线相似的紧凑图案，实际转动惯性、板中心安装偏差、灰度线宽和传输延迟未由离线模型证明；也不能保证整车已经驶出道路后仍能自动找回。下一次实车需要保存TURN_LINE_WAIT/READY的角度、双板RAW和后续位置事件，核对接线与真实车身位置。

## 本轮文件清单

- 生产源码修改：User/App_ThiefStrategy.c。
- 原有测试修改：Tests/test_escape_completion.py（明确后板桩）、test_stable1.py、test_strategy_stable2.py、test_temp78_stable3.py、test_hide_routes_stable4.py（当前固件标识）。
- 新增测试/工具：Tests/test_turn_line_stable5.py、prepare_stable5_validation.py、validate_stable5.py、run_stable5.py。
- 当前说明修改：README.md、AGENTS.md、Docs/修改规范与回归清单.md；新增本说明。
- 生成证据：build_stable5.log、VALIDATION_STABLE5.json、STABLE5_CHANGESET.patch、更新的STABLE_BASE_CHANGESET.patch，Tests/results_stable1.json、results_escape_completion.json、results_turn_resume.json、results_left800.json、results_live_monitor.json、results_strategy_stable2.json、results_temp78_stable3.json、results_hide_routes_stable4.json、results_turn_line_stable5.json，及Tests/stable5_*.log运行日志。Reference/STABLE4_before_turn_exit/results_turn_line.json保存旧版失败对照。
- 编译产物：MDK-ARM/Thief16V_UARTCh4_StableBase_Test/下的当前AXF/HEX及Keil自动生成的对象/映射文件。生产代码对上版的差异仅1个文件，详细补丁和所有生产源散列已绑定在验证报告中；当前目录没有Git仓库，未提交或推送。
