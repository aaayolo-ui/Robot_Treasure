# 16J风险分档回放

[打开16J回放](http://127.0.0.1:8765/variants/16j/replay.html) · [原16I](http://127.0.0.1:8765/variants/16i/replay.html)

执行独立16J工程的实际ARM C规划器，有限激光输入，沿用16F连续车身和接触裁判。88个节点编号默认显示，两车轨迹颜色不同，节点和路段可点击，地图可导出JSON。

改进危险选路：先比较120ms风险档，再比较可完整转移的模型数量和停车轴两端的较短直行余地。能在所有模型中安全完成转移的路线仍优先。不是概率保证，也没有删除不利对照。

三个固定种子平均持续：同速100.36→150.94秒；警方快10%为95.79→101.03秒；快20%仍67.32秒。九场16J完整对照最终均被抓。默认种子7另有两场撑到180秒，详见页面。

从项目根目录复现：

```powershell
E:\Python\python.exe Tools\TreasurePursuitSimulator\variants\16j\simulate_16j.py
E:\Python\python.exe Tools\TreasurePursuitSimulator\variants\16j\compare_heldout.py
E:\Python\python.exe Tools\TreasurePursuitSimulator\variants\16j\verify_catalog.py
E:\Python\python.exe Tools\TreasurePursuitSimulator\variants\16j\verify_pruning.py
E:\Python\python.exe Tools\TreasurePursuitSimulator\variants\16j\verify_legacy_mode.py
E:\Python\python.exe Tools\TreasurePursuitSimulator\variants\16j\verify_replay.py
```

完成对照后再次生成回放，嵌入最新比较。`compare_heldout.py --rerun-all` 重跑原版和新版；默认只复用通过源码及配置校验的原16I测量。`test_ui.cjs` 验证离线DOM事件，不是真实浏览器测试。完整Keil二进制分析用 `profile_firmware.py`，需先编译工程。

[算法详细说明](../../../../Docs/18_风险分档与藏点撤离余地16J.md) · [文件清单](FILE_MANIFEST.md) · [差异摘要](DIFF_SUMMARY.txt)

`development_*` 是已筛掉的开发试验，正式结果以 `report.json`、`heldout_comparison.json`、`validation_result.json` 为准。PNG是回放数据静态预览，非浏览器截图。

**仅编译通过，尚未完成硬件验证**。未提交、推送、烧录或启动真实电机。
