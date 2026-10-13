# 一次跑完全图并采集

使用 `STM32/Keil_Project/Gray12_MapFullRoute_Calibration_Test`。仅编译通过，尚未完成硬件验证。它集成用户新增的 `Gray12_DesktopBase_ReverseReturn_Test` 倒车逻辑；旧 `Gray12_MapRoute_Calibration_Test` 仍是 21 条分批方案。

## 车辆怎么放、怎么启动

前灰度对准 **N081 起点黑线，车头朝地图上方 N**。运行前两块灰度、陀螺和已配对的串口链路需要工作。短按一次 KEY2 连续执行整条路线，最终返回 N081 停车；中途换向先停稳再倒车/前进，无需搬车。运行中任意 KEY1/WK_UP/KEY2 停车，松手不会重启。

路线表见 [full_map_route.md](full_map_route.md)，机器可读数据为 [full_map_route.json](full_map_route.json)。名义长度 82.8 m，210 个可识别到达步骤、212 个地理路段方向项，118 次 90°转弯、24 次换向。路线是对有向路段的连续覆盖，比较固定次序以减少换向；不宣称全局耗时最优。

## 保存数据

先开电脑接收，再由用户按 KEY2。以下 `COM9` 只是已有记录的端口示例，可替换成实际串口；当前任务没有打开或测试这个端口。

```powershell
& F:\Robot_Treasure\Tools\MapTimingLogger\capture.ps1 -Port COM9 -Baud 115200
```

看到 `Gray12_MapFullRoute_Calibration_Test MT3` 和 `MAP_READY R001 START=N081 HEADING=N` 核对工程。正常末尾 `MAP_ROUTE_DONE R001 AT=N081`，END 参数 3；人工停止 END=1，故障 END=2。停车后继续接收几秒再 Ctrl+C，日志位于 `Tools/MapTimingLogger/logs`。电脑时间只是文件元信息，所有耗时使用 STM32 时间。

## 分析连续日志

在项目根目录使用可用的 Python 3 执行，将下面示例文件名替换成实际 `.log` 文件名：

```powershell
python Tools/MapTimingLogger/analyze_routes.py Tools/MapTimingLogger/logs/实际文件名.log --routes Tools/MapTimingLogger/full_map_route.json --output Tools/MapTimingLogger/results/full_report.json
```

报告包括每段的行驶模式、节点间耗时、转弯分阶段时间、`reversals` 换向时间、前进/倒车分别估计的 `mode_candidates`、未测路段和缺失速度项。CRC 损坏、丢包、地图版本不一致、模式错误和不完整换向会被剔除，不当作有效精确值。一次采集要确认 `sessions` 的 `valid=true`、`complete=true`；发生错误时保留原日志分析原因。

按实物确认起点摆放和行驶路线正确后，可额外生成警方或小偷的候选仿真配置。这里以警方为例，小偷改用 `--role thief`：

```powershell
python Tools/MapTimingLogger/analyze_routes.py Tools/MapTimingLogger/logs/实际文件名.log --routes Tools/MapTimingLogger/full_map_route.json --output Tools/MapTimingLogger/results/full_report.json --role police --confirm-start-placements --config-out Tools/MapTimingLogger/results/police_measured_candidate.json
```

不直接覆盖现有仿真配置。**当前仿真不区分持久的前进/倒车车身模式，候选配置只用前进样本；倒车数据单独保留，不与前进混合。** 未覆盖的前进方向使用仿真参考速度，不能称为每条路均已独立精确标定。报告地理覆盖完整，也不代表每条路前进/倒车两种模式都测全。

地图长度、65 mm 轮径和 1560 count/rev 为现有名义尺度；没有实测长度校准就不能承诺绝对速度准确度。转弯完整流程含移动找线，不能把整个流程时间再次叠加到路段耗时中；换向切换了前后灰度参考点，分析不把换向后的节点间时间直接换算为纯直道速度。N005 两侧路段共用一次测量，注明非独立测量。

## 协议和复现

MT3：`MT3,run,seq,time_ms,event,junction,route,step,from_node,to_node,arg,yaw_x100,yaw_valid,yaw_age_ms,yaw_frames,encA,encB,encC,encD,lost,drive_mode*CRC16`。

`drive_mode`：0 前进，1 倒车。事件 1–17 与 MT2 相同；18=REVERSE_BEGIN，19=REVERSE_READY，两者 arg 为目标模式。BEGIN/BRAKE 仍为旧模式，READY/FOLLOW/LINE_STABLE 为新模式但仍在原步骤，之后 LEG_BEGIN 才进入下一步骤。yaw 始终是物理车身原始角度，不人为加 180°；轮编码器同样保留原值。分析仅在同一模式的有效采样间换算带符号轮速。

```powershell
python Tools/MapTimingLogger/build_full_route.py
python -m unittest discover -s Tools/MapTimingLogger -p "test_*.py"
```

生成器只更新新工程的地图 ROM 和 `full_map_route.json/.md`；旧分批地图不变。重新生成后需重新编译并由用户烧录匹配签名的固件。测试使用合成日志，仅证明离线处理和路线表一致，不是实车采集结果。
