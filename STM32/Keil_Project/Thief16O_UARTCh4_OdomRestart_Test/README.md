# 16O：按编码器距离恢复重新起步

基线为同级 Thief16N_UARTCh4_RestartRecovery_Test，保留原工程。

## 本次修复

Documents2.txt 第2338行显示：N026回撤后再次倒车，起步后301 ms、估算行进68 mm，触发 RESTART_LINE_NOT_FOUND。16O取消这一固定300 ms截止条件。

在已定位路口重新起步且行进方向前侧灰度无黑线时：

- 以100 RPM低速继续移动，根据编码器判断行进距离。
- 两块灰度均未见有效线时，最多行进180 mm。
- 本次起步过程中后侧灰度见过有效线，可延长至300 mm；前侧见到有效线立即进入正常循迹。后侧见线只提供恢复依据，不冒充前侧灰度、不更新节点。
- 找线距离始终不超过当前路段长度减50 mm。例如N026→N035长300 mm，有后侧线证据时最多找线250 mm。
- 达到距离上限仍找不到线，打印 RESTART_SEARCH_END 的实际距离、上限及两侧灰度，再报 RESTART_SEARCH_DISTANCE。原有4秒离开起点超时和堵转检查仍有效。
- 前侧/后侧按行进方向定义：倒车时物理后灰度是前侧，物理前灰度是后侧。新一轮起步清除上一轮后侧线证据。

这是沿既定方向低速找线，没有加入摆头扫描。180/300 mm为本次试验参数，需要实车验证。

## 继承功能

五个固定藏点及隐藏朝向、途中拦截回撤、地图外目标过滤、600 mm两格触发、原始有效范围上限1000 mm、四路各目标10 Hz上传，继承16N。

## 工程与证据

工程：MDK-ARM/Thief16O_UARTCh4_OdomRestart_Test.uvprojx
固件：MDK-ARM/Thief16O_UARTCh4_OdomRestart_Test/Thief16O_UARTCh4_OdomRestart_Test.hex

生产源码仅修改 User/App_ThiefStrategy.c；修改 Tests/test_app_runtime.py、Tests/verify_snapshot.py 和本说明，重命名Keil工程/选项及IOC文件。IOC内容、Algorithm、Control、Core、Hardware、System未修改。无Git目录，差异保存在 SOURCE_CHANGESET.patch。

Tests/test_app_runtime.py 使用实际链接固件和硬件桩，覆盖日志中的N026→N035倒车、前进/倒车、301 ms行进68 mm不锁死、前侧重新见线、后侧证据延长距离、无证据距离截止及原有离开起点超时。Tests/run_16m.py执行既有算法回归，Tests/verify_snapshot.py核对产物与源码哈希，结果见 VALIDATION_16O.json。

仅编译通过，尚未完成硬件验证。没有自动烧录或驱动实车。
