# 区域追踪与两格检测修订（2026-10-08）

当前回放统一双方两格（名义0.6米），保留基准、警方快20%、动作偏慢P90三个场景。缩短距离仅减少远处干扰机会，不能识别脚与车；当前仿真没有场外人员模型。

## 策略与问题修复

- 警方丢失目标后，先沿最后运动方向排查图上可达藏点和分岔，再查本区死角，最后恢复长直道巡查。旧的排查记录不直接清除新线索。
- 小偷统一藏匿、检出后转移、再次藏匿。藏点限真实路口/拐角，排除死路、单出口封闭环路与长直道藏点。象限偏好必须结合地图连接和风险评估。
- N052 旧记录在140.172秒重规划，迎着东侧警方返回N053。原评分在所有路线均有风险时优先争取极小的预测接触时间优势，压过远离方向。现在近期观测下若有侧向或远离出口，先排除朝已观测警方接近的第一段，再比较风险与藏点。N052记录状态回归选择南向N063。此规则是启发式，不能保证所有地形下最优或一定逃脱。
- 第二张截图按警方进入N029死路问题处理；图片没有时间，未宣称逐帧复现原截图。完整支路（含中点）连续三次可见且未检出时，取消指向该死路的旧巡查任务，也不立即重选该端点。未完整确认或检出目标时仍检查/追踪。普通长直道巡查保持承诺。
- 修复过程中发现取消死路任务后原搜索器会立即重选该点，已增加回归防止原地等待循环。

## 验证范围

Python单元回归覆盖转移方向、有限观测、区域排查、死路取消与重选、中央长直道承诺。回放生成采用 seed=7，每场景一次；不是胜率或实车性能结论。静态审计核对88节点、106路段、121原节点对应关系、原图连接、碰撞后终止及短周期折返。页面验证三场景、双方两格参数、节点显示开关与N052连接信息。

当前单次回放：基准173.789秒警方胜；警方快20%为17.165秒警方胜；P90为64.481秒警方胜。不能由三个样本声称策略最优。实测动作耗时继续沿用先前标定；未更改STM32工程、烧录或驱动车辆。

## 本次连续任务文件清单

| 文件（相对项目根目录） | 变化 |
|---|---|
| Tools/TreasurePursuitSimulator/simulate.py | 区域追踪、转移方向约束、空死路任务取消、三场景生成 |
| Tools/TreasurePursuitSimulator/config.json | 区域搜索配置、警方两格 |
| Tools/TreasurePursuitSimulator/replay.template.html | 策略解释、区域目标显示、两格参数 |
| Tools/TreasurePursuitSimulator/replay.html | 重新生成回放 |
| Tools/TreasurePursuitSimulator/report.json | 三场景轨迹及事件 |
| Tools/TreasurePursuitSimulator/escape_routes.json | 更新生成配置 |
| Tools/TreasurePursuitSimulator/test_regional_search.py | 新增区域追踪、N052、空死路与重选回归 |
| Tools/TreasurePursuitSimulator/test_patrol.py | 保留历史巡查参数夹具 |
| Tools/TreasurePursuitSimulator/test_pockets.py | 更新区域追踪预期 |
| Tools/TreasurePursuitSimulator/test_simulate.py | 更新区域追踪预期 |
| Tools/TreasurePursuitSimulator/test_refuge.py | 双方两格检测边界回归 |
| Tools/TreasurePursuitSimulator/qa/verify_annotations.cjs | 三场景、两格及藏点审计 |
| Tools/TreasurePursuitSimulator/qa/refuge-labels-audit.json | 重新生成审计结果 |
| Tools/TreasurePursuitSimulator/README.md | 当前策略与参数说明 |
| Docs/08_夺宝对抗策略说明.md | 双方自然语言策略及传感器边界 |
| Tools/TreasurePursuitSimulator/regional-search-changes.md | 本说明（新增） |
| Tools/TreasurePursuitSimulator/regional-search.jpg | 更新页面验证截图 |

Git差异包含此前尚未提交的实测运动模型修改，不能全部算成本轮新增。原有无关硬件工程改动保留，未暂存、提交或推送。地图标注导出已重新核对，连接和编号未改变。
