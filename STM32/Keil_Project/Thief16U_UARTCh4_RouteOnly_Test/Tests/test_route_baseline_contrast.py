"""Actual RUN8 binary must differ from the newly requested escape policy."""
from pathlib import Path
import json,hashlib
setup=Path(__file__).with_name('test_escape_completion.py').read_text(encoding='utf-8').split('# Log regression:')[0]
setup=setup.replace('exec(compile(setup.split(', 'setup=setup.replace("axf=ROOT/\'MDK-ARM\'/ROOT.name/(ROOT.name+\'.axf\')", "axf=ROOT/\'Reference/RUN8/RUN8.axf\'")\nexec(compile(setup.split(',1)
exec(compile(setup,str(Path(__file__).with_name('test_escape_completion.py')),'exec'))
failures=[]
for travel in range(4):
    side=1<<((travel+1)%4)
    if call('ThiefIntercept_ShouldReverse',travel,side,0,0)!=0:failures.append(dict(travel=travel,issue='old side threat reverses'))
for node,axis in [(38,5),(78,5)]:
    if call('ThiefIntercept_HideAxis',node)!=axis:failures.append(dict(node=node,issue='old parking axis differs'))
assert len(failures)==6,failures
r=dict(expected_policy_differences=failures,baseline_axf_sha256=hashlib.sha256((ROOT/'Reference/RUN8/RUN8.axf').read_bytes()).hexdigest(),hardware_tested=False)
(ROOT/'Tests/route1_baseline_contrast.json').write_text(json.dumps(r,indent=2),encoding='utf-8');print(json.dumps(r,indent=2))
