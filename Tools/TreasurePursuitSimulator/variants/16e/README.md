# 16E 真实 C 规划器的独立回放与策略比较

访问 `http://127.0.0.1:8765/variants/16e/replay.html`，或直接打开本目录 `replay.html`。
页面使用内嵌数据，直接打开时不需要服务器。

新回放保留 N001 等节点标注、颜色/形状图例、节点和路段信息点击、两个显示开关、地图 JSON 导出和双方不同颜色的轨迹。增加了“首次藏匿”“检测决策”的跳转，以及策略比较折叠说明。四个回放均不省略接触失败的时刻。

小偷规划来自 `Thief16E_VisibilityEscape_FourLaser_Test` 的真实 C 源码，经 ARMCC 编译后在 Cortex-M3 CPU 模拟器中运行。电脑只模拟道路运动和传感器，警方使用原 Python 的两格有限观测追击/搜索策略。没有把全局警方坐标传入 C 规划器。

默认取宝路线、速度/PID/转向控制和原 16D 保留。默认启用应急可走路线与近距快速确认；其他候选只作为比较入口，默认关闭。详细解释在 [策略分类比较](../../../../Docs/13_小偷策略算法分类对比_16E.md)。

## 复现

需要已有 ARMCC 5、Python、Unicorn 和 pyelftools。运行所用测试依赖放在新工程的 `Tests/build/deps`，构建目录已忽略。无需为小车安装操作系统、虚拟机或远程服务。

```powershell
python .\compare_strategies.py --variants 16D_baseline escape_fast --seeds 10 11 12 13 14 --output heldout_comparison.json
python .\prepare_template.py
python .\simulate_16e.py
python .\verify_replay.py
node .\test_ui.cjs
```

需要生成关键帧时，用已安装 Pillow 的 Python 运行 `render_preview.py`。`arm_adapter.py` 为所有 C 调用设置指令上限，并检查调用确实返回；模拟器耗时不等于板上耗时。

若需要 HTTP 服务，从 `Tools/TreasurePursuitSimulator` 目录运行：

```powershell
python -m http.server 8765 --bind 127.0.0.1
```

测试文件用途：

- `strategy_comparison.json`：开发种子 7–9，逐项比较应急撤离、近距确认和两档预警评分。
- `turn_reach_comparison.json`：额外警方转弯可达模型，开发阶段快照。
- `hide_candidate_comparison.json`：固定 N044/N059/N062/N063 藏点，开发阶段快照。
- `junction_candidate_comparison.json`：N011/N038/N076 扩展路口，仅种子 7 的筛选。
- `heldout_comparison.json`：终版源码的独立种子 10–14，40 次复核；包含源文件哈希和编译开关。
- `report.json`：终版四个展示回放，带实际 C 决策和源文件哈希。
- `map_annotations.json`：同一组 88 节点/106 路段导出；长度沿用原地图，尚未实地标定。
- `ui_smoke_result.json`：离线 DOM 事件检查结果，不是浏览器渲染证明。
- `preview_hide.png`、`preview_retreat.png`、`preview_escape.png`：离线关键帧，不是硬件照片或浏览器截图。

保留方案在复核完整开局中平均生存约 37.8/26.9 秒，16D 约 17.8/15.9 秒。新旧版在这组复核中小偷获胜数均为 0；专项从已持宝藏点开始，不合并成完整比赛胜率。

本回放不证明硬件性能：位置、循迹和运动等待采用理想/估计模型；移动期激光关闭，未模拟惯性、安装偏移、灰度误判、真实 MCU 计算时间和串口电气。两帧确认的实车延迟取决于激光帧率。算法分类、拒绝的候选和限制都保留在比较文档中。

文件清单见 `FILE_MANIFEST.md`；没有修改旧回放、提交或推送。
