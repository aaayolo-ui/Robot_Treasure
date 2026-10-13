# 16G 多出口藏点与等待朝向回放

查看：[16G 回放](http://127.0.0.1:8765/variants/16g/replay.html)。选择两个 N038 朝向对照场景：相同警方从右侧来，横向等待约1.00秒接触，纵向等待约12.90秒接触，两场最终都被抓。

绿色圈为11个真实多出口候选，绿色双向箭头为停车轴。点击节点可查看各出口的下一分岔与预计时间。小偷沿实际到达路线完成停车朝向，检出警方后沿可用前后方向离开，再换线到下一藏点。地图节点、路段、19×19.8cm车身及连续碰撞判据沿用16F。

完整复核中，同速平均生存时间16F37.78秒、16G28.53秒；警方快20%时25.43秒、25.76秒，两版均未获胜。故保留16F为基线，16G作为独立实验。地图先验来向惩罚未同时改善两个速度场景，默认关闭。

详细策略、参数、验证与限制见 [Docs/15](../../../../Docs/15_多出口藏点与等待朝向16G.md)。新实车工程 `Thief16G_MultiExitHide_FourLaser_Test`：**仅编译通过，尚未完成硬件验证**。

```powershell
& E:\Python\python.exe F:\Robot_Treasure\Tools\TreasurePursuitSimulator\variants\16g\prepare_template.py
& E:\Python\python.exe F:\Robot_Treasure\Tools\TreasurePursuitSimulator\variants\16g\simulate_16g.py
& E:\Python\python.exe F:\Robot_Treasure\Tools\TreasurePursuitSimulator\variants\16g\verify_replay.py
```

使用原16E/16F环境的ARMCC、Unicorn、pyelftools和纯Python运动学模型，无需虚拟机或外部服务器。本地8765服务只显示生成的回放。模型/离线页面脚本检查通过；PNG是报告数据绘制的静态预览，非浏览器截图，实际浏览器布局与实车尚未验证。

本次完整新增清单见 `FILE_MANIFEST.md`。早期比较JSON保存开发过程，最终源与指标以 `heldout_comparison.json`、`report.json`、`validation_result.json` 为准。未暂存、提交或推送。
