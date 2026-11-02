"""Targeted MONITOR13 suite. Historical probe/axis tests retain their old intent."""
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor
import subprocess,sys,json,hashlib
root=Path(__file__).resolve().parents[1]
image=root/'MDK-ARM'/root.name/(root.name+'.axf')
image_hash=hashlib.sha256(image.read_bytes()).hexdigest()
names=['test_topology_linefirst','test_police_handoff13','test_speed16t',
       'test_turn_resume','test_left800','test_live_monitor','test_treasure_fix',
       'test_route_identity','test_escape_completion']
previous=[]
if '--only' in sys.argv:
    only=sys.argv[sys.argv.index('--only')+1:]
    assert only and all(n in names for n in only)
    previous=json.loads((root/'Tests/results_monitor13_suite.json').read_text(encoding='utf-8'))
    assert all(r['axf_sha256']==image_hash for r in previous),'cannot reuse another firmware run'
    names=only
def run(name):
    script=root/'Tests'/(name+'.py')
    p=subprocess.run([sys.executable,'-X','utf8',str(script)],cwd=root,stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
    log=root/'Tests'/('monitor13_'+name+'.log');log.write_bytes(p.stdout)
    print(name+': '+str(p.returncode),flush=True)
    if p.returncode:print(p.stdout.decode('utf-8',errors='replace')[-1600:],flush=True)
    return dict(name=name,exit_code=p.returncode,axf_sha256=image_hash,
                test_sha256=hashlib.sha256(script.read_bytes()).hexdigest(),log_sha256=hashlib.sha256(p.stdout).hexdigest())
with ThreadPoolExecutor(max_workers=3) as pool:results=list(pool.map(run,names))
if previous:
    results=[r for r in previous if r['name'] not in names]+results
assert hashlib.sha256(image.read_bytes()).hexdigest()==image_hash,'image changed during testing'
(root/'Tests/results_monitor13_suite.json').write_text(json.dumps(results,indent=2),encoding='utf-8')
sys.exit(any(r['exit_code'] for r in results))
