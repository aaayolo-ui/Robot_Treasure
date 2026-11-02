"""Generate current audit/runner without changing production firmware."""
from pathlib import Path
root=Path(__file__).resolve().parents[1]
source=(root/'Tests/validate_stable1.py').read_text(encoding='utf-8')
source=source.replace('build_stable1.log','build_stable5.log')
source=source.replace("'left800', 'live_monitor']", "'left800', 'live_monitor', 'strategy_stable2', 'temp78_stable3', 'hide_routes_stable4', 'turn_line_stable5']")
source=source.replace('UARTCH4_16V_STABLE1','UARTCH4_16V_STABLE5')
source=source.replace('VALIDATION_STABLE1.json','VALIDATION_STABLE5.json')
source=source.replace("tofile='STABLE1/'", "tofile='STABLE5/'")
extra="""
saved=root/'Reference/STABLE4_before_turn_exit'
regression=json.loads((saved/'results_turn_line.json').read_text(encoding='utf-8'))
assert regression['expected_before_failure'] and regression['failures']
assert regression['axf_sha256']==digest(saved/'MDK-ARM'/root.name/(root.name+'.axf'))
assert regression['test_sha256']==reports['turn_line_stable5']['test_sha256']
prior=json.loads((saved/'VALIDATION_STABLE4.json').read_text(encoding='utf-8'))
round_changes=[rel for rel,old in prior['source_sha256'].items() if digest(root/rel)!=old]
assert round_changes==['User/App_ThiefStrategy.c'], round_changes
assert len(reports['hide_routes_stable4']['lifecycles'])==7
assert len(reports['temp78_stable3']['lifecycles'])==2
app='User/App_ThiefStrategy.c'
round_patch=difflib.unified_diff((saved/app).read_text(encoding='utf-8-sig').splitlines(True),
                               (root/app).read_text(encoding='utf-8-sig').splitlines(True),
                               fromfile='STABLE4/'+app,tofile='STABLE5/'+app)
(root/'STABLE5_CHANGESET.patch').write_text(''.join(round_patch),encoding='utf-8')
result['changed_this_round']=round_changes
result['before_regression']=dict(checks=regression['checks'],failed_expectations=len(regression['failures']),
                                axf_sha256=regression['axf_sha256'])
result['log_inputs']={'小车串口_20261028_230118.txt':reports['turn_line_stable5']['log_sha256']}
"""
source=source.replace("save(root/'VALIDATION_STABLE5.json', result)",
                      extra+"\nsave(root/'VALIDATION_STABLE5.json', result)")
(root/'Tests/validate_stable5.py').write_text(source,encoding='utf-8')
runner=(root/'Tests/run_stable1.py').read_text(encoding='utf-8')
runner=runner.replace('five supported','nine supported')
runner=runner.replace("'test_live_monitor.py']", "'test_live_monitor.py', 'test_strategy_stable2.py', 'test_temp78_stable3.py', 'test_hide_routes_stable4.py', 'test_turn_line_stable5.py']")
runner=runner.replace('validate_stable1.py','validate_stable5.py')
(root/'Tests/run_stable5.py').write_text(runner,encoding='utf-8')
