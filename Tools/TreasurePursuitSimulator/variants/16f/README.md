# 16F 车身转向与接触回放

查看：[http://127.0.0.1:8765/variants/16f/replay.html](http://127.0.0.1:8765/variants/16f/replay.html)。

两车长 19 cm、宽 19.8 cm，采用用户接受的资料名义值，未实测整车外包络。车头用白色箭头标记。转向时车身连续旋转；倒车保持原车头方向。有朝向的两个矩形外包络首次接触时立即判抓捕并冻结两车，红点表示接触位置。连续检查帧间位移和旋转扫角。

可勾选碰撞轮廓，用“跳到首次转向”与 0.25 倍查看转向，用“跳到车身接触”检查终局。原节点编号、路段点击、地图 JSON 导出、不同颜色的轨迹保留。最后一个场景是静态旁车与转向车的碰撞专项，不作为比赛结果。

转向中心固定在路口，角速度曲线和制动占比是近似值；沿用 16E 的 0.768 秒额外转弯预算，不能称为实测纯 90°旋转时间。找线前移、惯性滑行和轮胎打滑没有实测建模。检测仍沿用16E的地图轴向理想光束。详细定义、参数和验证边界见 [模型说明](../../../../Docs/14_小车转向与车身碰撞仿真模型.md)。

## 重新生成

```powershell
& E:\Python\python.exe F:\Robot_Treasure\Tools\TreasurePursuitSimulator\variants\16f\prepare_template.py
& E:\Python\python.exe F:\Robot_Treasure\Tools\TreasurePursuitSimulator\variants\16f\simulate_16f.py
& E:\Python\python.exe F:\Robot_Treasure\Tools\TreasurePursuitSimulator\variants\16f\test_body_model.py
& E:\Python\python.exe F:\Robot_Treasure\Tools\TreasurePursuitSimulator\variants\16f\verify_replay.py
```

尺寸和转向模型参数在 `body_model.json`。修改后必须重新生成，让显示和判定使用相同尺寸。模板修改位于 `prepare_template.py`；该脚本从保留的 16E 模板构造本版，只写入本目录。

沿用实际 16E ARM C 规划器、激光门控与原警方策略。本目录独立构建桥接文件，读入 16E 固件源码，不改写旧工程。测试没有启动真实电机或下载固件。

18 项几何/运动测试及五个回放终局检查通过；离线页面脚本检查通过。368 个旧回放/16E 工程基线文件保持原哈希。浏览器布局、触控和实车动作未验证。预览 PNG 由 `report.json` 绘制，并非浏览器截图。文件清单见 `FILE_MANIFEST.md`。
