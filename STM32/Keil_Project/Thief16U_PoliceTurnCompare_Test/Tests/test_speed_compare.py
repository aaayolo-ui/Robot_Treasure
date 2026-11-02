"""Keep MONITOR12 speed assertions; only the explicitly chosen angle differs."""
from pathlib import Path
source=Path(__file__).with_name('test_speed16t.py').read_text(encoding='utf-8')
source=source.replace("check(a=='7500L','unknown-sign coarse turn reaches75degrees before gray handoff')", "check(a=='4500L','COMPARE1 uses police relative45degree handoff')")
source=source.replace("'results_speed16t.json'", "'results_speed_compare.json'")
exec(compile(source,str(Path(__file__).with_name('test_speed16t.py')),'exec'))
