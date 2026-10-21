# 65厘米检测与取宝后启用

日期：2026-10-13。直接修改 `Thief16B_FourLaser_MapStrategy_Test`，不是另建工程。独立16J、地图回放、其他STM32工程未修改。

## 当前行为

- 上电、READY与取宝前四路UART接收关闭。固定取宝路线保持 N054→N053→N044→N036→N035→N034→N033→N025→N015→N006→N005→N004。
- N005没有独立灰度路口。在合并的N006→N004（300毫米）路段中，定位有效且名义里程经过150毫米时锁存启用；若主循环恰好漏过该阈值，联合定位验证成功的N004到达事件兜底开启。以实际路段长度兜底，因此合法到达时的轻微里程超出300毫米不会遗漏启用。
- 这是地图与编码器估计的宝藏点经过事件，不是新增了实物取宝传感器。模块既有供电不变，软件控制的是UART接收及策略使用，未控制激光发射或电源。
- 取宝后行驶、转弯、藏匿保持四路接收。原16J继续在停稳藏点检测后规划换线；**移动观测只更新威胁记忆，不会中途换路线、急转或自动制动避障**。转弯/路口过渡位姿无效时只收数据，不构造目标坐标。按键或故障仍主动制动并关闭接收。
- 前、左、右、后共用软件上限650毫米。50毫米下限、置信度≥10、5帧3有效、跨度≤120毫米及原近距2帧规则保留。650毫米可接受，651毫米不触发威胁；无道路、地图缺口、场地外回波仍过滤。地图内物体不是警方身份识别。
- 激光射线由当前估计XY及车头方向投影，再沿真实道路连接检查。移动时拒绝超过25毫秒的帧、早于当前有效路段起始时刻的旧帧及朝向误差超过5°的观测。定位本身仍沿用25°容差，这个较严格的5°只约束移动激光投影。
- 一次主循环在控制和定位更新之后，最多解析每路32字节（四路共128字节）。失效帧和转弯过渡会清确认窗口，不能借用前一朝向的投票。移动期不增加逐帧格式化日志。

## 接线和参数

接线仍为前PA3、左PB7、右PC11、后PB11，编码器B相PB14、日志TX为PB10；四路115200/8N1。RX-only MSP保留原实现，不改GPIO、时钟、串口/TIM初始化、PID、轮位映射、灰度循迹或速度/转向参数。

检测上限在 `Algorithm/ThiefStrategy/ThiefStrategyConfig.h` 的 `THIEF_LASER_MAX_MM=650U`；移动新鲜度为 `THIEF_LASER_MOVING_FRAME_MAX_MS=25UL`，移动朝向筛选为 `THIEF_LASER_MOVING_YAW_TOL_X100=500L`。启动判断在 `Algorithm/MapLocalization/TreasureLaserStart.c/.h`，调用和启用锁存在 `User/App_ThiefStrategy.c`。滤波/地图投影在 `Algorithm/ThiefStrategy/FrontLaserGate.c/.h`，有界解析在 `Hardware/LaserRx.c/.h`。

仅统一小偷实际检测及其入路证据的范围；16J对警方的600毫米模型视野假设不随此改动变成650毫米。仿真参考速度和转向耗时未重新标定。

## 串口确认

上电输出：

```text
LASER RANGE_MM=650 START=AFTER_N005 MOVING_RX=ON REPLAN=SETTLED_HIDE
START_OK STATE=LEAVE LASER_RX=WAIT_TREASURE
```

经过N005开启时仅输出一次：

```text
TREASURE_N005 LASER_RX=ALL_ON RANGE_MM=650
```

之后逃跑起步应为 `START_OK STATE=LEAVE LASER_RX=ALL_ON`；取宝前停止状态显示 `WAIT_TREASURE`。四路距离日志仅在停止状态轮换输出，移动期不输出连续测距文本。不存在“日志没有逐帧输出，所以移动接收关闭”的等价关系。

## 离线验证

| 检查 | 结果 |
|---|---|
| Keil ARMCC5全量编译 | 0 Error(s), 0 Warning(s) |
| 650毫米/宝藏启用/移动投影C测试 | 5947项，0失败；覆盖106路段、636移动位姿 |
| 四路UART实际C解析 | 824项，0失败；默认关闭、启用保留、四路积压各按32字节公平处理 |
| 灰度/陀螺定位C测试 | 5162项，0失败；58构造路线、436节点确认 |
| 16J策略C测试 | 11625项，0失败；初始藏点N038、17执行步，固定取宝前缀不变 |
| Keil登记与保护审计 | 92条文件路径有效且无重复；设备/下载/调试配置保留；115个Core/Hardware/Control/Drivers/System文件与原16B逐字节一致，仅LaserRx.c/.h为此功能调整 |

总计23558项C断言通过。固件Flash为196456字节，RAM为35368字节（当前配置256 KiB Flash/48 KiB RAM）；RAM计数不能证明运行时栈峰值余量。9个策略C/H仍与独立16J逐字节一致，4个因范围/投影调整。源码审查确认启用锁存、单次定位快照、有界解析、旧朝向时间过滤及停稳藏点重规划；没有对整车状态机进行硬件闭环运行。

算法测试为真实ARMCC编译的C代码在电脑CPU模拟器上运行，使用构造观测。UART测试调用真实解析代码与HAL测试桩，不代表真实蓝牙链路或中断时序。实测控制周期、四路传感器帧率、安装偏移、行驶定位误差与检测有效距离仍需实车确认；25毫秒和5°是过滤参数，不是测量精度保证。

- 可复现命令：`python Tests/run_algorithms.py --treasure-laser`、`--uart`、`--localization`及无参数策略测试；环境需要ARMCC5、Unicorn和pyelftools。
- 源码/配置审计：`python Tests/verify_localization_integration.py`。
- [机器可读验证结果](VALIDATION_LASER65.json)。
- [完整修改/新增文件、当前哈希与git diff摘要](LASER65_FILES.json)。清单包含此前尚未提交的16J定位融合，以及本次65厘米增量；`latest_request_files`区分本次增量。
- `VERIFICATION_16J_LOCALIZATION.md`与`16J_LOCALIZATION_FILES.json`保留为65厘米改动前的定位阶段记录。

**仅编译通过，尚未完成硬件验证**。未自动烧录、运行电机、暂存、提交或推送。未将旧四激光实车控制响应问题宣布为已解决。
