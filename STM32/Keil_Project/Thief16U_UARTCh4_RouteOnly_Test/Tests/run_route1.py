"""Offline-only route and preserved RUN8 regressions."""
from pathlib import Path
import subprocess,sys,concurrent.futures,json,hashlib
r=Path(__file__).resolve().parents[1]
tests=['test_strategy_stable2.py','test_temp78_stable3.py','test_n011_first_plan_stable10.py','test_documents13.py','test_documents14.py','test_documents15.py','test_documents16.py','test_speed_run8_route1.py','test_route_baseline_contrast.py']
def run(n):
    with (r/'Tests'/('route1_'+n+'.log')).open('w',encoding='utf-8') as f:
        p=subprocess.run([sys.executable,'-X','utf8',str(r/'Tests'/n)],stdout=f,stderr=subprocess.STDOUT)
    return dict(test=n,exit_code=p.returncode)
with concurrent.futures.ThreadPoolExecutor(max_workers=5) as e:
    results=list(e.map(run,tests))
print(json.dumps(results,indent=2));assert all(x['exit_code']==0 for x in results)
subprocess.run([sys.executable,'-X','utf8',str(r/'Tests/validate_route1.py')],check=True)
