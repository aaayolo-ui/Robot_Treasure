# 16I算法回放

[打开本机回放](http://127.0.0.1:8765/variants/16i/replay.html)。

本页执行独立16I工程的实际ARM C规划器，通过虚拟激光输入有限观测，再用16F连续车身模型产生回放。蓝色为警方，橙色为小偷；88个节点及N001等编号默认显示。检测决策面板显示候选数量、推演数量、入口推断和预测风险。

完整比赛三种子均值：同速16H 35.57秒→16I 100.36秒；警方快10%为34.20→95.79秒；快20%为33.19→67.32秒。全部最终被抓。N038横向等待、上方初始已见来车的同速专项变差，约91.30→13.55秒，页面保留这一场景。

从项目根目录复现：

```powershell
E:\Python\python.exe Tools\TreasurePursuitSimulator\variants\16i\simulate_16i.py
E:\Python\python.exe Tools\TreasurePursuitSimulator\variants\16i\compare_heldout.py
E:\Python\python.exe Tools\TreasurePursuitSimulator\variants\16i\verify_catalog.py
E:\Python\python.exe Tools\TreasurePursuitSimulator\variants\16i\verify_pruning.py
E:\Python\python.exe Tools\TreasurePursuitSimulator\variants\16i\verify_replay.py
```

运行完整对照后再次生成回放，页面会嵌入最新对照。`test_ui.cjs`用Node运行，验证离线DOM事件及地图标注，不是实际浏览器测试。`profile_firmware.py`统计完整Keil二进制的纯算法指令，需先编译新工程，不访问硬件。

`preview_map.png`、`preview_parking_axes.png`、`rollout_comparison.png`均为报告姿态数据绘制的静态预览，非浏览器截图。开发试验文件`development_*.json`保留推演过程，其中标为rejected的隐藏意图试验未采用；最终结果以`report.json`、`heldout_comparison.json`和`validation_result.json`为准。

本次未改历史版本；[16H回放](../16h/replay.html)可继续对照。完整文件清单见[FILE_MANIFEST.md](FILE_MANIFEST.md)，详细限制见[Docs/17](../../../../Docs/17_有界推演与最坏情况逃跑16I.md)。

**仅编译通过，尚未完成硬件验证**。未提交、推送、烧录或启动真实电机。
