from pathlib import Path
import sys,subprocess,concurrent.futures,json
r=Path(__file__).resolve().parents[1]
tests=['test_strategy_stable2.py','test_temp78_stable3.py','test_n011_first_plan_stable10.py','test_hide_routes_stable4.py','test_escape_completion.py','test_turn_resume.py','test_left800.py','test_live_monitor.py']
def run(n):
    with (r/'Tests'/('route2_'+n+'.log')).open('w',encoding='utf-8') as f:
        p=subprocess.run([sys.executable,'-X','utf8',str(r/'Tests'/n)],stdout=f,stderr=subprocess.STDOUT)
    return dict(test=n,exit_code=p.returncode)
with concurrent.futures.ThreadPoolExecutor(max_workers=6) as e:results=list(e.map(run,tests))
(r/'Tests/route2_suites.json').write_text(json.dumps(results,indent=2),encoding='utf-8')
assert all(x['exit_code']==0 for x in results),results
subprocess.run([sys.executable,'-X','utf8',str(r/'Tests/validate_route2.py')],check=True)
