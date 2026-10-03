# 密室夺宝离线路径规划工具

按用户提供的地图与 2026 规则建立黑线道路图，使用 Dijkstra 计算双方最短距离和带转弯代价的最小时间路线。警方最短为 5.25 m、七次转弯，小偷最短为 3.15 m、四次转弯。距离按 0.30 m 网格推算，实车时间需输入测量参数。

详细路线、算法、警方拦截及小偷逃跑建议见 [策略说明.md](策略说明.md)。本目录为独立离线工具，不控制机器人。

## 使用

需要 Python 3，`planner.py` 和测试只使用标准库，不需额外安装。以下命令在本目录运行：

```powershell
python planner.py --start P --target T --mode distance
python planner.py --start B --target T --mode time --speed 0.35 --turn90 0.60 --uturn 1.20
python planner.py --start P --target T --mode time --speed 0.35 --turn90 1.00
python planner.py --start P --target T --blocked '(6,3)'
python planner.py --start '(6,7)' --target T --heading N --mode time
python planner.py --report results.json
python -m unittest discover -s . -p 'test_*.py' -v
```

`P/police`、`B/thief`、`T/treasure` 是起点与目标别名；也可传入 `(x,y)` 节点。`--speed` 为 m/s，`--turn90` 和 `--uturn` 为相较直行模型额外增加的秒数；默认起始朝向为警方 N、小偷 W，可用 `--heading` 覆盖。`--blocked` 用于静态禁入节点，不自动估计移动对手或车辆尺寸。

重新生成 PNG 需要 Pillow：`python build_visuals.py`。本次使用 Codex 自带 Python 和 Pillow，未安装全局依赖。

## 输出字段

- `nodes/xy`：完整采样节点路径，不等于实际路口计数。
- `legs`：同方向合并后的直行段和段起点转向。
- `navigation_events`：岔路、转弯及预期全局出口方向。
- `estimated_seconds`：输入参数下的行驶时间，不含取宝及确认。
- `shortest_alternatives`：等长最短路；通用接口最多返回 20 条，当前地图的小偷三条全部返回。
- `cycles`：明确列出的四条候选回路，没有声称枚举全部简单回路。
- `structure`：终点、割点、桥与环路秩；不能把普通交汇点当成必经割点。

## 文件清单和变更范围

本次全部新增文件如下，未修改已有仓库文件：

| 文件 | 用途 |
| --- | --- |
| `.gitignore` | 忽略本工具 Python 缓存及 QA 临时文件 |
| `map.json` | 人工描图的线段、网格、起点、端帽和候选回路 |
| `planner.py` | 标准库最短路、朝向时间代价、等长路线枚举及图结构分析 |
| `test_planner.py` | 八项离线算法测试 |
| `results.json` | 已计算的节点边表、双方路线、指令、回路与参数 |
| `build_visuals.py` | 根据建图数据生成 PNG 和 SVG |
| `reference_map.png` | 用户原始地图截图副本 |
| `route_paths.png` | 双方取宝路线图，红色选用小偷上侧等长路线 |
| `escape_cycles.png` | 小偷取宝后的两条候选回路示意图 |
| `map_graph.svg` | 可编辑节点拓扑图 |
| `策略说明.md` | 规则、计算结果、双方策略、执行边界及验证记录 |
| `README.md` | 使用说明及完整变更清单 |

变更摘要：新增 `Tools/TreasureRoutePlanner/` 下上述十二个文件。所有既有 STM32 工程、历史阶段和当前基线文档保持原状。未执行 `git add`、`git commit` 或 `git push`。因为新增文件尚未暂存，普通 `git diff` 不包含这些文件，可用 `git status --short --untracked-files=all -- Tools/TreasureRoutePlanner` 查看完整新增范围。

实测边长、转弯额外耗时、取宝耗时、终点端帽含义和实际黑线连通情况待在调试地图上确认。当前证明限于离线图模型；没有进行 STM32 编译或实车测试。
