"""Run unchanged historical ARM scenarios against this monitoring AXF."""
from pathlib import Path
import subprocess,sys,json,hashlib
root=Path(__file__).resolve().parents[1]
names=['test_live_monitor','test_right900','test_gray_priority','test_documents123154','test_documents31','test_documents29','test_documents24','test_documents22','test_app_runtime','test_escape_completion']+['test_documents'+str(n) for n in range(12,22)]+['test_speed16t']
results=[]
for name in names:
    p=subprocess.run([sys.executable,'-X','utf8',str(root/'Tests'/(name+'.py'))],cwd=root,stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
    (root/'Tests'/('monitor_'+name+'.log')).write_bytes(p.stdout)
    print(name+': '+str(p.returncode),flush=True)
    results.append(dict(name=name,exit_code=p.returncode,log_sha256=hashlib.sha256(p.stdout).hexdigest()))
    if p.returncode:print(p.stdout.decode('utf-8',errors='replace')[-1600:],flush=True)
(root/'Tests/results_monitor_suite.json').write_text(json.dumps(results,indent=2),encoding='utf-8')
sys.exit(any(r['exit_code'] for r in results))
