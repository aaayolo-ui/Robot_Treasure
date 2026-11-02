"""Actual linked firmware, simulated peripherals only. No serial/flash/motors."""
from pathlib import Path
import sys,subprocess,concurrent.futures,json,hashlib
r=Path(__file__).resolve().parents[1];sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
image=r/'MDK-ARM'/r.name/(r.name+'.axf')
inputs={p.relative_to(r).as_posix():sha(p) for folder in ['User','Control','Algorithm','Hardware','Core','System','Drivers','MDK-ARM'] for p in (r/folder).rglob('*') if p.is_file() and p.suffix.lower() in ['.c','.h','.s','.uvprojx']}
test_inputs={p.relative_to(r).as_posix():sha(p) for p in (r/'Tests').glob('*.py')}
(r/'Tests/route8_inputs.json').write_text(json.dumps(dict(production=inputs,test_sources=test_inputs,axf_sha256=sha(image)),indent=2),encoding='utf-8')
tests=['test_strategy_stable2.py','test_temp78_stable3.py','test_n011_first_plan_stable10.py','test_hide_routes_stable4.py','test_escape_completion.py','test_turn_resume.py','test_left800.py','test_live_monitor.py','test_west63.py','test_trace_front_probe.py','test_trace_return.py','test_trace_recovery.py','test_n068_recovery.py','test_rear_retrace.py','test_short_return.py','test_n063_near_route.py']
def run(n):
    with (r/'Tests'/('route8_'+n+'.log')).open('w',encoding='utf-8') as f:
        p=subprocess.run([sys.executable,'-X','utf8',str(r/'Tests'/n)],stdout=f,stderr=subprocess.STDOUT)
    return dict(test=n,exit_code=p.returncode)
results=[]
with concurrent.futures.ThreadPoolExecutor(max_workers=6) as ex:
    for f in concurrent.futures.as_completed([ex.submit(run,n) for n in tests]):
        result=f.result();results.append(result);print(result,flush=True)
        if result['exit_code']:print((r/'Tests'/('route8_'+result['test']+'.log')).read_text(encoding='utf-8')[-3000:],flush=True)
(r/'Tests/route8_suites.json').write_text(json.dumps(results,indent=2),encoding='utf-8')
assert all(x['exit_code']==0 for x in results),results
assert all(sha(r/p)==h for p,h in inputs.items()),'production changed during tests'
assert all(sha(r/p)==h for p,h in test_inputs.items()),'test source changed during tests'

subprocess.run([sys.executable,"-X","utf8",str(r/"Tests/validate_route8.py")],check=True)
