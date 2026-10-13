# 16F 本次新增文件清单

新增本目录 26 个文件和一份 `Docs/14_小车转向与车身碰撞仿真模型.md`。既有文件未修改；368项基线哈希全部一致。未暂存、提交或推送。

本次范围的 `git diff` 没有已跟踪文件差异，原因是全部以新文件保存。继承文件的派生差异见 [DIFF_SUMMARY.txt](DIFF_SUMMARY.txt)。

| 新增文件（相对本目录） | 用途 |
| --- | --- |
| [.gitignore](.gitignore) | 忽略本版构建产物和Python缓存 |
| [arm_adapter.py](arm_adapter.py) | 原样复制16E桌面ARM适配器；构建仅写入本版build目录 |
| [arm_bridge.c](arm_bridge.c) | 原样复制16E桌面C桥接，未新增MCU项目 |
| [BASELINE_SHA256.json](BASELINE_SHA256.json) | 修改前旧16E回放及固件368文件的SHA256基线 |
| [body_geometry.py](body_geometry.py) | 米制旋转矩形、分离轴与包含扫角的连续接触计算 |
| [body_model.json](body_model.json) | 用户接受的车身尺寸、转向近似与数值接触参数 |
| [body_motion.py](body_motion.py) | 连续车身角度、倒车方向与角点速度上界 |
| [DIFF_SUMMARY.txt](DIFF_SUMMARY.txt) | 本次增量与继承文件相对16E差异摘要 |
| [FILE_MANIFEST.md](FILE_MANIFEST.md) | 本次完整新增文件清单（本文件） |
| [finalize_delivery.py](finalize_delivery.py) | 生成完整文件清单和本次变更摘要 |
| [map_annotations.js](map_annotations.js) | 本版标注布局按旋转车身包围框避让标签 |
| [map_annotations.json](map_annotations.json) | 88节点/106路段稳定标注导出，与16E一致 |
| [prepare_template.py](prepare_template.py) | 可复现地从16E模板生成新页面 |
| [preview_map.png](preview_map.png) | 地图和终局车身接触的静态预览，已查看 |
| [preview_turn_contact.png](preview_turn_contact.png) | 转向前/中途/首次接触的三帧预览，已查看 |
| [README.md](README.md) | 查看、运行、参数与验证范围说明 |
| [render_preview.py](render_preview.py) | 从报告真实姿态绘制静态预览，不是浏览器截图 |
| [replay.html](replay.html) | 含回放数据与脚本的独立HTML |
| [replay.template.html](replay.template.html) | 新页面模板：连续车身、轮廓、转向/接触跳转和慢放 |
| [report.json](report.json) | 五个回放、38字段帧数据、实际C源哈希和终局接触信息 |
| [simulate_16f.py](simulate_16f.py) | 复用16E策略，独立生成四场策略回放及一场转向碰撞专项 |
| [test_body_model.py](test_body_model.py) | 18项运动/碰撞测试，含400对称检查及55组密集参照 |
| [test_ui.cjs](test_ui.cjs) | 执行实际页面脚本的离线DOM/事件检查 |
| [ui_smoke_result.json](ui_smoke_result.json) | 页面事件验证结果，明确不是浏览器测试 |
| [validation_result.json](validation_result.json) | 五个终局接触、C哈希与不可变基线验证结果 |
| [verify_replay.py](verify_replay.py) | 终局车身接触、单次抓捕、基线哈希和地图一致性检查 |

| 新增项目文档（相对项目根目录） | 用途 |
| --- | --- |
| [Docs/14_小车转向与车身碰撞仿真模型.md](../../../../Docs/14_小车转向与车身碰撞仿真模型.md) | 尺寸、动作定义、准确性边界、参数、验证与算法依据 |

构建输出 `build/` 与 `__pycache__/` 已忽略，不属于交付源文件。未修改任何 STM32 测试工程。PNG 是报告数据绘制的静态预览，非浏览器截图；实际浏览器和实车验证未执行。
