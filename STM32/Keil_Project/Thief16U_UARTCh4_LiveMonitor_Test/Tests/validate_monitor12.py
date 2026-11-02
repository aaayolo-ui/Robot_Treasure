"""MONITOR12: measured brake/gray handoff, rear width and unchanged policies."""
from pathlib import Path
import difflib,hashlib,json
root=Path(__file__).resolve().parents[1];base=root/'Tests/before_line_handoff'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
read=lambda p:json.loads(p.read_text(encoding='utf-8'))
axf=root/'MDK-ARM'/root.name/(root.name+'.axf');firmware=sha(axf)
old=read(base/'VALIDATION_MONITOR11.json')
assert sha(base/'original.axf')==old['firmware_sha256']
before=read(base/'regression_before.json');after=read(root/'Tests/results_line_handoff.json')
assert before['axf_sha256']==old['firmware_sha256'] and len(before['failures'])==4
assert before['test_sha256']==after['test_sha256']==sha(root/'Tests/test_line_handoff.py')
assert after['axf_sha256']==firmware and not after['failures']
assert after['cases']==dict(five_degree_handoff=4,rear_width_follow=2,rear_width_recovery=4,first_unknown_sign_turn=1)
changed=['User/App_ThiefStrategy.c','Tests/test_line_handoff.py','Tests/test_monitor7_restore.py',
 'Tests/test_speed16t.py','Tests/analyze_turn_042252.py','Docs/turn_042252_analysis.json',
 'README.md','AGENTS.md','Docs/修改规范与回归清单.md']
for name,value in old['files'].items():
    if name.endswith(('.c','.h','.py')) and name not in changed:assert sha(root/name)==value,name
analysis=read(root/'Docs/turn_042252_analysis.json')
assert len(analysis['rows'])==4 and analysis['mean_alignment_ms']==1905
assert analysis['log_sha256']==sha(base/'小车串口_20261028_042252.txt')
assert abs(analysis['summary']['left']['total_deg_mean']-94.985)<0.001
assert abs(analysis['summary']['right']['total_deg_mean']-81.32)<0.001
general=read(root/'Tests/results_general_turns.json')
assert general['axf_sha256']==firmware and not general['failures']
assert general['test_sha256']==sha(root/'Tests/test_general_turns.py')
assert general['cases']['all_map_quarter_turns']['combinations']==1016
extra={}
for stem in ['yaw_reference','turn_resume','monitor7_restore']:
    r=read(root/'Tests'/('results_'+stem+'.json'))
    assert r['axf_sha256']==firmware and not r['failures']
    assert r['test_sha256']==sha(root/'Tests'/('test_'+stem+'.py'))
    extra[stem]=r['cases']
suite=read(root/'Tests/results_monitor_suite.json')
assert len(suite)==22 and len({r['name'] for r in suite})==22
for row in suite:
    assert row['exit_code']==0 and row['axf_sha256']==firmware,row['name']
    assert row['log_sha256']==sha(root/'Tests'/('monitor_'+row['name']+'.log'))
supplemental={}
for stem,minimum in [('route_identity',92),('exit_stall',988),('road_probe',1798),('left800',442)]:
    r=read(root/'Tests'/('results_'+stem+'.json'))
    assert r['axf_sha256']==firmware and r['checks']>=minimum
    assert not r.get('failures') and r.get('failure') is None
    assert r['test_sha256']==sha(root/'Tests'/('test_'+stem+'.py'))
    supplemental[stem]=r['checks']
pure=read(root/'Tests/results_16m.json')
assert pure['test_failures']==0 and pure['test_done']==1 and pure['test_case']==378
for name,value in pure['source_sha256'].items():assert sha(root/name)==value,name
assert '0 Error(s), 0 Warning(s)' in (root/'build_monitor12.log').read_text(encoding='utf-8',errors='replace')
assert 'UARTCH4_16U_MONITOR12' in (root/'User/App_ThiefStrategy.c').read_text(encoding='utf-8')
patch=[]
for name in changed:
    previous=(base/name).read_text(encoding='utf-8') if (base/name).exists() else ''
    patch.extend(difflib.unified_diff(previous.splitlines(keepends=True),
        (root/name).read_text(encoding='utf-8').splitlines(keepends=True),
        fromfile='MONITOR11/'+name,tofile='MONITOR12/'+name))
(root/'LINE_HANDOFF_CHANGESET.patch').write_text(''.join(patch),encoding='utf-8')
files=changed+['Tests/validate_monitor12.py','LINE_HANDOFF_CHANGESET.patch','build_monitor12.log',
 'Tests/results_line_handoff.json','Tests/results_general_turns.json','Tests/results_monitor_suite.json']
files+=['Tests/results_'+stem+'.json' for stem in list(extra)+list(supplemental)]
data=dict(firmware='UARTCH4_16U_MONITOR12',firmware_sha256=firmware,hex_sha256=sha(axf.with_suffix('.hex')),
 before_failures=before['failures'],new_checks=after['checks'],new_cases=after['cases'],
 general=general['cases'],extra=extra,suite=suite,supplemental=supplemental,
 unchanged_algorithm_checks=pure['test_checks'],unchanged_algorithm_evidence='source hashes revalidated; algorithm modules unchanged since MONITOR11',
 user_selected_policy=analysis['selected_policy'],files={name:sha(root/name) for name in files},
 hardware_tested=False,flashed=False,serial_opened=False,
 limitations=['Only4 logged angle samples; they mix entry skew, inertia and active trim. No fixed95/81degree cutoff was inferred.',
 'Normal turns no longer pulse-align to2degrees; actual missing-line/hide recovery remains separate.',
 'Wide rear gray is excluded from steering cues, but raw node evidence and actual black-presence timing are retained.',
 'Physical yaw drift, lateral displacement, node identity, traction and final line reacquisition need vehicle tests.'])
(root/'VALIDATION_MONITOR12.json').write_text(json.dumps(data,ensure_ascii=False,indent=2),encoding='utf-8')
print('MONITOR12 verified: brake/gray PID handoff,rear width,first unknown-sign turn,1016 map combinations and22 ARM scripts;hardware untested.')
