# 16K 藏点结构对照

[打开16K](http://127.0.0.1:8765/variants/16k/replay.html) · [原16J](http://127.0.0.1:8765/variants/16j/replay.html)

N038 的四条快速出口和 N062 的两条快速出口，现在在页面上用结构等级、数量和公开地图巡查绕行先验显示。N062 棕色虚线圈、较好藏点绿色实线圈。保留节点编号、路段点击、开关、JSON 导出、不同颜色轨迹及连续车身首次接触裁判。

**当前保留版七项场景和九场完整对照均与16J相同，N062仍可能被选择；本轮未证明策略效果改善。** 多种强制替换和扩展藏点的试验更早被抓，已拒绝，不建议把16K当作已解决问题的实车升级。

实际执行独立16K工程的ARM C，使用理想定位及600mm有限检测。实车工程继承650mm、行驶接收、灰度与陀螺定位、地图边缘和文字过滤；这些硬件流程未在本回放验证。名义车身19×19.8cm未实测，连续旋转仍是运动学近似。

从仓库根目录，用安装了 Unicorn、pyelftools 的 Python 运行：

```powershell
python Tools/TreasurePursuitSimulator/variants/16k/compare_heldout.py
python Tools/TreasurePursuitSimulator/variants/16k/simulate_16k.py
python Tools/TreasurePursuitSimulator/variants/16k/verify_replay.py
python Tools/TreasurePursuitSimulator/variants/16k/verify_pruning.py
python Tools/TreasurePursuitSimulator/variants/16k/verify_legacy_mode.py
node Tools/TreasurePursuitSimulator/variants/16k/test_ui.cjs
```

ARMCC5 默认路径 `D:/keil5/ARM/ARMCC/bin`。先对照再生成页面，使页面嵌入最终比较。验证后可直接用已有8765本地服务查看。本次未启动真实硬件。

`report.json`、`heldout_comparison.json` 为当前结果；`development_*.json` 是失败或未改善试验摘要，不属于保留策略。离线DOM测试不代表真实浏览器、CSS布局或实车测试。

[技术分析](../../../../Docs/22_藏点结构与安全退路16K.md) · [修改文件清单](FILE_MANIFEST.md) · [差异摘要](DIFF_SUMMARY.txt) · [仿真验证](validation_result.json)

**仅编译通过，尚未完成硬件验证**。未提交、推送或烧录。
