# 16L 条件藏点与停车后脱身回放

[打开16L](http://127.0.0.1:8765/variants/16l/replay.html) · [保留16K](http://127.0.0.1:8765/variants/16k/replay.html)

真实ARM C算法在Cortex-M3模拟器中执行，沿用16F运动/相撞裁判。保留N001等编号、路段点击、开关、地图JSON、不同颜色轨迹、连续转向与首次车身接触结束。

N038主要藏点，N026南北停车为条件候选，N062应急备用。N026黄色虚线不表示值得停车，东侧封闭环不计有效退路。原七场和九场完整对照与16K结果相同；N062仍会使用，当前没有N026准入成功证据。新增后三场人为摆车在N026，约8.571/2.012/0.157秒被抓，用于显示风险，不是算法准入成功。

决策面板显示所选目标终端KNOWN/CLEAR、等待、余量与应急标记。最多检查最佳常规点及条件点两者，未知不能算安全；“所选目标未检查”也不表示未检查其他候选。

公平回放双方600mm；实车工程650mm。回放理想定位，仅停靠时输入激光，未覆盖实车定位误差、惯性、打滑、运动时激光和旋转光束。19×19.8cm车身未实测。离线DOM事件测试不等于真实浏览器布局验收。

**仅编译通过，尚未完成硬件验证**。没有自动提交、推送、烧录或操作电机。

## 复现

在 `F:/Robot_Treasure` 使用已有Python、ARMCC5和requirements依赖，或本仓库16E的Tests/build/deps：

```powershell
python STM32/Keil_Project/Thief16L_TerminalEscape_FourLaser_Test/Tests/verify_16l_tests.py
python Tools/TreasurePursuitSimulator/variants/16l/profile_stack.py
python Tools/TreasurePursuitSimulator/variants/16l/profile_stack.py --optimization=-O3 --output stack_profile_o3.json --label stack_profile_o3
python STM32/Keil_Project/Thief16L_TerminalEscape_FourLaser_Test/Tests/verify_16l_integration.py
python Tools/TreasurePursuitSimulator/variants/16l/compare_heldout.py
python Tools/TreasurePursuitSimulator/variants/16l/simulate_16l.py
node Tools/TreasurePursuitSimulator/variants/16l/test_ui.cjs
python Tools/TreasurePursuitSimulator/variants/16l/verify_replay.py
```

九场基准先核验16K配置、车况和源码哈希再复用；`--rerun-baseline`可重跑旧算法，不修改历史报告。电脑模拟耗时不能代替板上耗时，栈采样不包括实车IRQ嵌套。

若回放服务没有启动，从仓库根目录运行并保留该终端：

```powershell
python -m http.server 8765 --bind 127.0.0.1 --directory Tools/TreasurePursuitSimulator
```

不需要虚拟机或外部服务器。STM32仅使用离线Flash表与纯C算法。

[技术说明](../../../../Docs/23_条件藏点与停车后脱身16L.md) · [工程](../../../../STM32/Keil_Project/Thief16L_TerminalEscape_FourLaser_Test/README.md) · [全部新增文件与差异](FILES_16L.json) · [回放验证](validation_result.json) · [九场对照](heldout_comparison.json)
