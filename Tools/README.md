# 密室夺宝电脑端成果

本目录收录本次聊天形成的地图规划和双车追逃仿真最终成果。当前策略的自然语言说明见 [小偷与警察策略](../Docs/08_夺宝对抗策略说明.md)。

- [TreasureRoutePlanner](TreasureRoutePlanner/README.md)：原图、道路数据、路径规划程序、测试、计算结果及路线图。其策略说明保留早期静态分析，当前动态策略以仿真目录和上面的策略文档为准。
- [TreasurePursuitSimulator](TreasurePursuitSimulator/README.md)：当前追逃策略、参数、七场景回放、稳定节点与路段数据、测试及验证截图。

下载仓库后可直接用浏览器打开 `TreasurePursuitSimulator/replay.html`。GitHub 文件预览不会直接运行交互回放。

## 验证

在仓库根目录运行：

```powershell
python -m unittest discover -s Tools/TreasureRoutePlanner
python -m unittest discover -s Tools/TreasurePursuitSimulator
node Tools/TreasurePursuitSimulator/qa/verify_annotations.cjs
```

提交前实际通过：地图规划 8 项测试、仿真 100 项测试、七局回放输出核查。仅为电脑仿真验证，未执行 STM32 编译或硬件测试。

## 交付范围

各工具 README 中早期的“尚未跟踪、未提交”及阶段清单为当时工作记录。本次将下列最终文件纳入版本管理；临时备份、旧版 `baseline_v1.py`、历史 QA 对比、缓存留在本地，不作为当前运行依赖。

验证脚本使用 `qa/reference-map.json` 保存的早期原地图基准进行比较，不再依赖本地备份目录。保留桌面、手机标注截图和最新 `pocket-fix.jpg`；桌面/手机截图用于展示此前标注验证，最新策略回放以 `replay.html` 为准。

### 本次新增到 Git 的文件

- `Tools/README.md`
- `Tools/TreasurePursuitSimulator/.gitignore`
- `Tools/TreasurePursuitSimulator/config.json`
- `Tools/TreasurePursuitSimulator/escape_routes.json`
- `Tools/TreasurePursuitSimulator/map_annotations.js`
- `Tools/TreasurePursuitSimulator/map_annotations.json`
- `Tools/TreasurePursuitSimulator/map_annotations.py`
- `Tools/TreasurePursuitSimulator/qa/annotations-desktop.jpg`
- `Tools/TreasurePursuitSimulator/qa/annotations-mobile.jpg`
- `Tools/TreasurePursuitSimulator/qa/pocket-fix.jpg`
- `Tools/TreasurePursuitSimulator/qa/reference-map.json`
- `Tools/TreasurePursuitSimulator/qa/refuge-labels-audit.json`
- `Tools/TreasurePursuitSimulator/qa/verify_annotations.cjs`
- `Tools/TreasurePursuitSimulator/README.md`
- `Tools/TreasurePursuitSimulator/replay.html`
- `Tools/TreasurePursuitSimulator/replay.template.html`
- `Tools/TreasurePursuitSimulator/report.json`
- `Tools/TreasurePursuitSimulator/simulate.py`
- `Tools/TreasurePursuitSimulator/test_annotations.py`
- `Tools/TreasurePursuitSimulator/test_patrol.py`
- `Tools/TreasurePursuitSimulator/test_pockets.py`
- `Tools/TreasurePursuitSimulator/test_refuge.py`
- `Tools/TreasurePursuitSimulator/test_simulate.py`
- `Tools/TreasureRoutePlanner/.gitignore`
- `Tools/TreasureRoutePlanner/build_visuals.py`
- `Tools/TreasureRoutePlanner/escape_cycles.png`
- `Tools/TreasureRoutePlanner/map.json`
- `Tools/TreasureRoutePlanner/map_graph.svg`
- `Tools/TreasureRoutePlanner/planner.py`
- `Tools/TreasureRoutePlanner/README.md`
- `Tools/TreasureRoutePlanner/reference_map.png`
- `Tools/TreasureRoutePlanner/results.json`
- `Tools/TreasureRoutePlanner/route_paths.png`
- `Tools/TreasureRoutePlanner/test_planner.py`
- `Tools/TreasureRoutePlanner/策略说明.md`
