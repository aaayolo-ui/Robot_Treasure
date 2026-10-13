# 已完成全图实车记录

`20261006-210343-671.log`是既有原始日志的逐字节存档，SHA256为`138576fbe7416f90fad63818e5d3d552f5463ef1cb8ddebf997868bcf0b09d12`。用户已反馈车辆完整走图；解析得到一趟210步、2906条有效记录、224.045秒。其余空接收、短时连接试验及合成日志不在这里。

在项目根目录执行：

```powershell
python Tools/MapTimingLogger/summarize_basics.py Tools/MapTimingLogger/datasets/20261006-210343-671.log --output Tools/MapTimingLogger/results/reproduced_basics.md
python Tools/MapTimingLogger/analyze_routes.py Tools/MapTimingLogger/datasets/20261006-210343-671.log --routes Tools/MapTimingLogger/full_map_route.json --output Tools/MapTimingLogger/results/reproduced_route.json
```

[基础统计报告](../reports/20261006-210343-671_basics.md)保留原结果。`20261006-210343-671_route_report.json`为既有路线分析结果存档，其`source_sha256`是解析器内部记录摘要；原始文件校验使用上面的原始字节SHA256，两者不可混用。

速度依赖名义地图长度和轮周，转弯额外代价由节点通行时间估计；换向时间截止于重新识线。它们不是物理静止、首次倒退或纯原地90°转完的精确测量。单趟分布不是重复试验置信区间。
