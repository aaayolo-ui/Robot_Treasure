"""Generate current audit/runner without changing production firmware."""
from pathlib import Path
root=Path(__file__).resolve().parents[1]
source=(root/'Tests/validate_stable1.py').read_text(encoding='utf-8')
source=source.replace('build_stable1.log','build_stable4.log')
source=source.replace("'left800', 'live_monitor']", "'left800', 'live_monitor', 'strategy_stable2', 'temp78_stable3', 'hide_routes_stable4']")
source=source.replace('UARTCH4_16V_STABLE1','UARTCH4_16V_STABLE4')
source=source.replace('VALIDATION_STABLE1.json','VALIDATION_STABLE4.json')
source=source.replace("tofile='STABLE1/'", "tofile='STABLE4/'")
extra="""
saved=root/'Reference/STABLE3_before_hide_routes'
regression=json.loads((saved/'results_hide_routes.json').read_text(encoding='utf-8'))
assert regression['expected_before_failure'] and regression['failures']
assert regression['axf_sha256']==digest(saved/'MDK-ARM'/root.name/(root.name+'.axf'))
assert regression['test_sha256']==reports['hide_routes_stable4']['test_sha256']
assert len(reports['hide_routes_stable4']['lifecycles'])==7
round_changes=[];round_patch=[]
for rel in ['User/App_ThiefStrategy.c', 'Algorithm/ThiefStrategy/ThiefMapPlanner.c',
            'Algorithm/ThiefStrategy/ThiefRiskSearch.c', 'Algorithm/ThiefStrategy/ThiefInterceptPolicy.c']:
    assert digest(saved/rel)!=digest(root/rel), 'missing intended change: '+rel
    round_changes.append(rel)
    round_patch.extend(difflib.unified_diff((saved/rel).read_text(encoding='utf-8-sig').splitlines(True),
                       (root/rel).read_text(encoding='utf-8-sig').splitlines(True),
                       fromfile='STABLE3/'+rel,tofile='STABLE4/'+rel))
(root/'STABLE4_CHANGESET.patch').write_text(''.join(round_patch),encoding='utf-8')
result['changed_this_round']=round_changes
result['before_regression']=dict(checks=regression['checks'],failed_expectations=len(regression['failures']),
                                axf_sha256=regression['axf_sha256'])
"""
source=source.replace("save(root/'VALIDATION_STABLE4.json', result)",
                      extra+"\nsave(root/'VALIDATION_STABLE4.json', result)")
(root/'Tests/validate_stable4.py').write_text(source,encoding='utf-8')
runner=(root/'Tests/run_stable1.py').read_text(encoding='utf-8')
runner=runner.replace('five supported','eight supported')
runner=runner.replace("'test_live_monitor.py']", "'test_live_monitor.py', 'test_strategy_stable2.py', 'test_temp78_stable3.py', 'test_hide_routes_stable4.py']")
runner=runner.replace('validate_stable1.py','validate_stable4.py')
(root/'Tests/run_stable4.py').write_text(runner,encoding='utf-8')
