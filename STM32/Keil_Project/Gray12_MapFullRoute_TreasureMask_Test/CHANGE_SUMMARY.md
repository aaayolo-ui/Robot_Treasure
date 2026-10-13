# 宝藏区过滤版变更

来源为全图标定工程当前工作目录，原工程保留。本次只新增独立工程目录。

- 新增 `Algorithm/MapRoute/TreasureGrayMask.c`、`.h`：N004↔N006 北侧两路过滤。
- 修改副本 `User/App_LineFollowExperiment.c`：增加领头/尾随过滤调用及状态门控，更新启动标识。
- 更新新名称 `.uvprojx`、`.ioc`：工程名、输出目录与新模块登记；无硬件配置改动。
- Keil 生成新目标的 `RTE_Components.h`。
- 更新本版 `.gitignore`、`README.md`、`CHANGE_SUMMARY.md`、`FILE_MANIFEST.md`、`BASELINE_SOURCE_SHA256.json`、`BASELINE_DIFF.patch`。

完整新增文件见 FILE_MANIFEST.md；与来源的源码差异见 BASELINE_DIFF.patch。Git 中为新增目录，未执行 add、commit 或 push；工作区其他原有修改不属于本次。

核验：来源文件哈希未变；Control/Hardware/Core/System 和地图路线逐字节相同；Keil 源码引用无重复；两路过滤调用均在位置计算前；N004/N006 北侧无真实支路。ARMCC5 全量编译 0 错误、0 警告，已生成 HEX。

仅编译通过，尚未完成硬件验证。
