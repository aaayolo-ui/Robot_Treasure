"""Run the five supported offline suites against the current linked firmware.

Requires Python, unicorn and pyelftools (existing Tests/build/deps is supported).
No serial port, flashing or physical motor operation is performed.
"""
from pathlib import Path
import subprocess
import sys

root = Path(__file__).resolve().parents[1]
for script in ['test_stable1.py', 'test_escape_completion.py',
               'test_turn_resume.py', 'test_left800.py', 'test_live_monitor.py']:
    print('RUN ' + script, flush=True)
    subprocess.run([sys.executable, '-X', 'utf8', str(root/'Tests'/script)],
                   cwd=str(root), check=True)
subprocess.run([sys.executable, '-X', 'utf8', str(root/'Tests/validate_stable1.py')],
               cwd=str(root), check=True)
