"""Hidden launcher embedded into the desktop switch; all starts are explicit."""
import os
from pathlib import Path
import runpy
import sys
import traceback

script = Path(sys.argv[1]).resolve()
if script.name not in {"laser_live_server.py", "cloud_bridge.py"}:
    raise SystemExit("Unsupported monitor service")
log_root = Path(os.environ["LOCALAPPDATA"]) / "RobotTreasureMonitor" / "logs"
log_root.mkdir(parents=True, exist_ok=True)
log_file = (log_root / (script.stem + ".log")).open("a", encoding="utf-8", buffering=1)
sys.stdout = sys.stderr = log_file
try:
    sys.argv = [str(script), *sys.argv[2:]]
    sys.path.insert(0, str(script.parent))
    os.chdir(script.parent)
    runpy.run_path(str(script), run_name="__main__")
except BaseException:
    traceback.print_exc()
    raise
