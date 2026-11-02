import os
from pathlib import Path
import runpy
import sys
import traceback

script = Path(sys.argv[1]).resolve()
if script.name not in {"console_server.py", "cloud_bridge.py"}:
    raise SystemExit("Unsupported service")
logs = script.parent / "logs"
logs.mkdir(exist_ok=True)
sys.stdout = sys.stderr = (logs / (script.stem + ".log")).open("a",encoding="utf-8",buffering=1)
try:
    sys.argv = [str(script)]
    sys.path.insert(0,str(script.parent))
    os.chdir(script.parent)
    runpy.run_path(str(script),run_name="__main__")
except BaseException:
    traceback.print_exc()
    raise
