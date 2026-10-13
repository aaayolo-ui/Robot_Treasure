# 16H 转弯安全窗口

[打开回放](http://127.0.0.1:8765/variants/16h/replay.html)。选择完整开局，或“不利停车对照”观察确实来不及脱身时的抓捕。“跳到预计转弯风险”显示C规划的保守风险时刻；“跳到抓捕时刻”显示实际矩形首次接触。

针对N059藏点之后在N060转向被追尾的问题，增加停车、旋转、转后离开路口的检查；只保留停车轴前后能先直达下一分岔的藏点。两格检测、速度、转弯代价、车身尺寸和抓捕判据沿用原版，旧版保留。

同速完整开局约35.57秒接触，警方快20%约33.19秒；相应16G为28.53秒、25.76秒。均最终被抓，同速16F仍较长。不能据此保证实车不会追尾或全面优于全部旧策略。

说明、参数、限制和完整验证：[Docs/16](../../../../Docs/16_转向前后追尾与安全窗口16H.md)。[完整新增清单](FILE_MANIFEST.md)，[验证结果](validation_result.json)，[静态对照图](turn_safety_comparison.png)。

```powershell
& E:\Python\python.exe F:\Robot_Treasure\Tools\TreasurePursuitSimulator\variants\16h\simulate_16h.py
& E:\Python\python.exe F:\Robot_Treasure\Tools\TreasurePursuitSimulator\variants\16h\prepare_template.py
& E:\Python\python.exe F:\Robot_Treasure\Tools\TreasurePursuitSimulator\variants\16h\verify_replay.py
```

`compare_heldout.py`复核最终源码；`compare_development.py`是开发消融。`development_before_transfer.json`、`development_comparison.json`记录早期版本，不作为当前源码通过验证的依据；被撤回的短转移优先试验见`rejected_short_transfer.json`。PNG为报告数据绘图，并非浏览器截图。

实车工程为 `Thief16H_TurnClearance_FourLaser_Test`：**仅编译通过，尚未完成硬件验证**。没有改GPIO、驱动、PID或执行器时序，未烧录或运行电机，未暂存、提交或推送。
