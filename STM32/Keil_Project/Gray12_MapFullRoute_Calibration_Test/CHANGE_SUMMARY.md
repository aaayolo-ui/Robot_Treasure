# 本次改动范围

新建独立全图工程；原倒车工程和分批工程未修改。源码相对倒车基线的差异：

| 文件 | 行数变化 |
| --- | --- |
| `Algorithm/MapRoute/MapRouteNavigator.c` | +123 / -0 |
| `Algorithm/MapRoute/MapRouteNavigator.h` | +17 / -0 |
| `Algorithm/MapRoute/MapRoutePlan.c` | +433 / -0 |
| `Algorithm/MapRoute/MapRoutePlan.h` | +20 / -0 |
| `System/UartDebug.c` | +26 / -0 |
| `System/UartDebug.h` | +3 / -0 |
| `User/App_LineFollowExperiment.c` | +260 / -186 |
| `User/MapRoutePosition.c` | +25 / -0 |
| `User/MapRoutePosition.h` | +8 / -0 |
| `User/MapTimingLog.c` | +228 / -0 |
| `User/MapTimingLog.h` | +39 / -0 |

另外新建地图表、计时/里程模块、工程描述、基线 SHA、操作与验证文档。新 Keil 工程只修改工程名称、输出目录及地图模块和头文件目录登记；硬件初始化/引脚和底层控制保持基线。完整新增清单见 FILE_MANIFEST.md。

项目外修改（均在 Tools/MapTimingLogger）：`build_routes.py`（兼容 MT3 ROM）、`analyze_routes.py`（MT2/MT3 和模式分离）、`README.md`（新旧入口）。新增：`build_full_route.py`、`full_map_route.json`、`full_map_route.md`、`full_run.md`、`test_full_route.py`、`verify_full_project.py`。

Git diff --stat 只显示原有已跟踪的 4 个其他文件变化（118 insertions, 68 deletions）；本次工程和 MapTimingLogger 目录尚未跟踪，故本次源码差异另以 BASELINE_DIFF.patch 和此清单提供。没有 git add、commit 或 push。
