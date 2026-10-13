"""Run the actual ARM planner tests; no hardware access."""
from pathlib import Path
import subprocess,sys
ROOT=Path(__file__).resolve().parent
subprocess.run([sys.executable,str(ROOT/'run_algorithms.py')],check=True)
