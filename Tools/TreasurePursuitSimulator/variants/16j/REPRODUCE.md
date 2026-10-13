# 下载后查看与复现16J

## 只看回放

下载或克隆整个仓库，直接打开本目录的`replay.html`。页面已嵌入场景数据；节点编号默认显示，原16I对照入口和地图JSON也一并保存。GitHub文件页面显示源码，不能直接播放HTML。

也可在项目根目录执行：

```powershell
python -m http.server 8765 --bind 127.0.0.1 --directory Tools/TreasurePursuitSimulator
```

随后打开`http://127.0.0.1:8765/variants/16j/replay.html`。

## 核对上传内容

```powershell
python Tools/TreasurePursuitSimulator/variants/16j/verify_published.py
```

该命令仅用Python标准库，核对当前C源码/16I对照SHA、嵌入数据、地图连接、逐帧车身接触和原始实车日志，不驱动硬件。历史`verify_replay.py`还检查开发时6852个本机文件及本机构建日志，应在原开发工作区使用；不要求下载者恢复无关本机文件。

## 重新运行实际C算法

Python3.9或更新版本；安装[requirements.txt](requirements.txt)中的`unicorn`和`pyelftools`。Node.js用于离线页面事件测试，Pillow仅用于生成静态预览。Keil仍使用ARM Compiler5；脚本中的默认工具路径为`D:/keil5/ARM/ARMCC/bin`，Python路径可使用自己的解释器。

```powershell
python -m pip install -r Tools/TreasurePursuitSimulator/variants/16j/requirements.txt
python STM32/Keil_Project/Thief16J_RefugeViability_FourLaser_Test/Tests/run_algorithms.py
python STM32/Keil_Project/Thief16J_RefugeViability_FourLaser_Test/Tests/run_algorithms.py --uart
python Tools/TreasurePursuitSimulator/variants/16j/verify_pruning.py
python Tools/TreasurePursuitSimulator/variants/16j/verify_legacy_mode.py
python Tools/TreasurePursuitSimulator/variants/16j/simulate_16j.py
python Tools/TreasurePursuitSimulator/variants/16j/compare_heldout.py --rerun-all
python Tools/TreasurePursuitSimulator/variants/16j/simulate_16j.py
node Tools/TreasurePursuitSimulator/variants/16j/test_ui.cjs
```

最后一次生成回放用于嵌入新的对照数据。完整Keil工程为`STM32/Keil_Project/Thief16J_RefugeViability_FourLaser_Test/MDK-ARM/Thief16J_RefugeViability_FourLaser_Test.uvprojx`；不上传构建产物，编译后自行生成。16F车身模型、16E共享执行流程、16I对照和基础地图/警方仿真是必要依赖，已包含在本次上传中。

独立算法及CPU仿真不需要虚拟机或远程服务器。100毫秒预算必须在实际芯片测量；指令数量不能替代该验证。仅编译通过，尚未完成硬件验证。
