# ROUTE3交付与变更清单

固件标识：UARTCH4_16V_ROUTE3_TRACE。工程仍为MDK-ARM/Thief16V_UARTCh4_RouteOnly_Test.uvprojx。修改前完整副本Reference/ROUTE2_before_trace_return；本版没有自动烧录、串口连接或实物电机运行。

## 源码与工程

- 新增Algorithm/ThiefStrategy/ThiefRouteTrail.c、ThiefRouteTrail.h：876字节固定容量历史、路口暂存与提交、反向路线生成、回程记录消耗。
- 修改User/App_ThiefStrategy.c：从真实藏点出发建历史，确认路口/接线时记录，前方三帧触发原路返回，回家清旧威胁/恢复停车轴向，新回程缺线恢复与通过路口的里程继承，初始化清历史及固件标识。
- 修改MDK-ARM/Thief16V_UARTCh4_RouteOnly_Test.uvprojx：只增加ThiefRouteTrail.c引用。
- 全部16个Control、25个Hardware、7个Core、6个System、68个Drivers源文件保持ROUTE2；既有Algorithm文件（含激光配置、规划器、搜索）也保持，新增两份Trail文件。13个基础动作函数、原PID/平滑/PI链、普通入弯/停稳/主旋转逐段对照相同。

## 文档

README.md、AGENTS.md、Docs/修改规范与回归清单.md新增本版最新约束；Docs/ROUTE3_原路返回说明.md解释流程、日志和局限。本文件为交付索引。

## 测试与证据

- 新增Tests/test_trace_front_probe.py：旧ROUTE2忽略N011行进前方障碍的复现，新版返回原N011。
- 新增Tests/test_trace_return.py：16次完整返回（含10/44/120mm折返、多个路口、停车轴恢复、两种陀螺仪符号、跨±180度）、47条历史加部分段的48步容量、暂存与提交边界。
- 新增Tests/test_trace_recovery.py：错侧线、旧后板线/未完成回程转弯、新回程前方再次被堵后的退出、KEY停止、双白2秒、零位移、17mm刹车位移、60mm路口通过信用。
- 修改Tests/test_n011_first_plan_stable10.py：按新要求只验证侧后方不打断N011锁定；行进前方由新增返回测试验证。保留原路线/解除/按键测试。
- 修改Tests/test_escape_completion.py：途中障碍输入按新要求提供3帧，保留原完整逃跑与途中检测。
- 新增Tests/run_route3.py、Tests/validate_route3.py：统一运行12组当前回归，检查源码/AXF散列、基线冻结、工程引用和容量。
- 当前报告：VALIDATION_ROUTE3.json、ROUTE3_CHANGESET.patch、Tests/route3_inputs.json、Tests/route3_suites.json、Tests/route3_*.log及验证JSON列出的12份结果。旧版报告和中间构建只作历史。

## 最终验证

ARMCC5全量编译0错误、0警告；12组当前回归通过。真实ARM固件在模拟外设/车辆响应上执行，不是实车证明。

链接器ROM 67720字节，RAM 44896字节，48KB内剩余4256字节；3KB保留栈已计入RAM。已知最大调用栈920字节，实际中断叠加及运行高水位未实测。

最终AXF SHA256：0b80471cbe83528830b1b166bc9c6cfbf5bcd56863d6f39045d4d58482ae23a9。

仅编译通过，尚未完成硬件验证。节点编号仍是基于传感器和地图的推断，历史不能消除已发生的路口误认。
