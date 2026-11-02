"""Audit the new copy and generate file-level evidence in this ZIP checkout."""
from pathlib import Path
import hashlib,json,difflib,xml.etree.ElementTree as ET
ROOT=Path(__file__).resolve().parents[1]
OLD=ROOT.parent/'Thief16T_UARTCh4_EntryFrameRestore_Test'
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
original={};changed=[];added=[];equal=[];diff=[]
directories=['Algorithm','Control','Core','Drivers','Hardware','System','User']
for directory in directories:
    for p in sorted((ROOT/directory).rglob('*')):
        if not p.is_file():continue
        rel=p.relative_to(ROOT).as_posix();old=OLD/rel
        if not old.exists():added.append(rel)
        elif sha(p)==sha(old):equal.append(rel)
        else:changed.append(rel)
        if old.exists():original[rel]=sha(old)
        if not old.exists() or sha(p)!=sha(old):
            diff.extend(difflib.unified_diff(old.read_text(encoding='utf-8').splitlines(True) if old.exists() else [],
                p.read_text(encoding='utf-8').splitlines(True),fromfile='baseline/'+rel,tofile='16U/'+rel))
expected_run3=['Algorithm/MapRoute/MapRouteNavigator.c', 'Algorithm/MapRoute/MapRouteNavigator.h', 'Algorithm/ThiefStrategy/FrontLaserGate.c', 'Algorithm/ThiefStrategy/FrontLaserGate.h', 'Control/FollowSmoothing.c', 'Control/FollowSmoothing.h', 'User/App_ThiefStrategy.c', 'User/MapRoutePosition.c', 'User/MapRoutePosition.h']
assert set(changed)==set(expected_run3+['Algorithm/ThiefStrategy/ThiefInterceptPolicy.c','Algorithm/ThiefStrategy/ThiefStrategyConfig.h']) and set(added)=={'Control/RoadHeadingControl.c','Control/RoadHeadingControl.h'}
for directory in ['Core','Drivers','Hardware','System']:
    assert not any(p.startswith(directory+'/') for p in changed+added),directory+' unexpectedly changed'
project=ROOT/'MDK-ARM'/(ROOT.name+'.uvprojx')
tree=ET.parse(project)
paths=[x.text for x in tree.findall('.//FilePath')]
normalized=[(project.parent/p.replace('\\','/')).resolve() for p in paths]
assert len(set(normalized))==len(normalized),'duplicate source paths'
assert all(p.is_file() for p in normalized),'missing project source'
assert all(p.is_relative_to(ROOT) for p in normalized),'external source dependency'
assert tree.findtext('.//TargetName')==ROOT.name and tree.findtext('.//uAC6')=='0'
assert not any(x.text=='1' for tag in ['RunUserProg1','RunUserProg2'] for x in tree.findall('.//'+tag))
assert any('ThiefInterceptPolicy.c' in p for p in paths)
assert not any(p.endswith('ThiefRollout.c') for p in paths)
app=(ROOT/'User/App_ThiefStrategy.c').read_text(encoding='utf-8')
assert 'ALGO=16U' in app and 'TRIGGER_CENTER_MM=1000' in app and 'REPLAN=INTERCEPT_RETREAT_FIRST' in app
assert 'TravelFrame_SetReverse(departure_entry_mode);' in app
assert 'DIAG_FRONT_OFF' not in app and 'PLANNING_BUDGET_EXCEEDED' not in app
assert app.count('MapRoutePosition_DistanceMm(map_leg_start_counts)')==1
log=(ROOT/'build_16U.log').read_text(encoding='utf-8',errors='replace')
assert '0 Error(s), 0 Warning(s)' in log
result=json.loads((ROOT/'Tests/results_16m.json').read_text())
assert result['test_done'] and not result['test_failures']
for path,value in result['source_sha256'].items():assert sha(ROOT/path)==value,path+' changed after algorithm tests'
app_result=json.loads((ROOT/'Tests/results_app_runtime.json').read_text())
assert app_result['failures']==0
hexfile=ROOT/'MDK-ARM'/ROOT.name/(ROOT.name+'.hex')
axf=hexfile.with_suffix('.axf')
assert hexfile.is_file() and hexfile.stat().st_size>0
assert app_result['firmware_sha256']==sha(axf),'application test firmware mismatch'
assert app_result['app_source_sha256']==sha(ROOT/'User/App_ThiefStrategy.c')
assert app_result['test_script_sha256']==sha(ROOT/'Tests/test_app_runtime.py')
for directory in directories:
    assert all(p.stat().st_mtime<=axf.stat().st_mtime for p in (ROOT/directory).rglob('*')
               if p.suffix in ['.c','.h']),'source newer than firmware'
escape_result=json.loads((ROOT/'Tests/results_escape_completion.json').read_text())
assert escape_result['failures']==0 and escape_result['firmware_sha256']==sha(axf)
assert escape_result['app_source_sha256']==sha(ROOT/'User/App_ThiefStrategy.c')
assert escape_result['test_script_sha256']==sha(ROOT/'Tests/test_escape_completion.py')
assert len(escape_result['escapes'])==36 and len(escape_result['ignored'])==4
assert len(escape_result['interceptions'])==4
assert 'UARTCH4_16U_RUN8' in app
before=ROOT/'Tests/before_documents16'
before_hashes=json.loads((before/'source_sha256.json').read_text())
fix_sources=[rel for rel,value in before_hashes.items() if sha(ROOT/rel)!=value]
assert fix_sources==['User/App_ThiefStrategy.c'],fix_sources
speed=json.loads((ROOT/'Tests/results_speed16t.json').read_text())
reference_speed=json.loads((ROOT/'Tests/results_speed16t_reference.json').read_text())
assert speed['failures']==0 and reference_speed['failures']==0
assert speed['firmware_sha256']==sha(axf)
assert speed['app_source_sha256']==sha(ROOT/'User/App_ThiefStrategy.c')
assert speed['test_script_sha256']==sha(ROOT/'Tests/test_speed16t.py')
assert reference_speed['test_script_sha256']==speed['test_script_sha256']
assert speed['curves']==reference_speed['curves'],'normal target curves differ from16T'
assert speed['reference_source_sha256']==sha(OLD/'User/App_ThiefStrategy.c')
assert reference_speed['app_source_sha256']==speed['reference_source_sha256']
assert reference_speed['firmware_sha256']==sha(OLD/'MDK-ARM'/OLD.name/(OLD.name+'.axf'))
assert 'FollowSmoothing_LimitBase' not in app
regression14=json.loads((ROOT/'Tests/results_documents14.json').read_text())
assert not regression14['failures'] and regression14['heading_sweep_cases']==44
assert regression14['firmware_sha256']==sha(axf)
assert regression14['app_source_sha256']==sha(ROOT/'User/App_ThiefStrategy.c')
assert regression14['test_script_sha256']==sha(ROOT/'Tests/test_documents14.py')
assert 'ROUTE_MAP_GRAY_YAW_TOL_X100       1000L' in app
assert 'recovery_limit' in regression14['passed']
for name,value in regression14['helper_sha256'].items():assert sha(ROOT/'Tests'/name)==value
before14dir=ROOT/'Tests/before_documents14'
before14=json.loads((before14dir/'regression_before.json').read_text())
assert {x['case'] for x in before14['failures']}=={'log_replay','heading_sweep'}
assert before14['test_script_sha256']==sha(before14dir/'test_documents14_before.py')
log14=(before14dir/'Documents14.txt').read_text(encoding='utf-8',errors='replace')
assert 'RAW=010,E00 YAW_ERR=-466' in log14 and 'YAW_ERR=-651' in log14
assert 'MAP_GRAY_END CAUSE=' in app
regression15=json.loads((ROOT/'Tests/results_documents15.json').read_text())
assert not regression15['failures']
assert regression15['cases']['parking_cases']==41 and regression15['cases']['turns']==34
assert regression15['firmware_sha256']==sha(axf)
assert regression15['app_source_sha256']==sha(ROOT/'User/App_ThiefStrategy.c')
assert regression15['test_script_sha256']==sha(ROOT/'Tests/test_documents15.py')
for name,value in regression15['helper_sha256'].items():assert sha(ROOT/'Tests'/name)==value
before15dir=ROOT/'Tests/before_documents15'
before15=json.loads((before15dir/'regression_before.json').read_text())
assert {x['case'] for x in before15['failures']}=={'parking_cases','turns'}
assert before15['test_script_sha256']==sha(before15dir/'test_documents15_before.py')
assert before15['firmware_sha256']==sha(before15dir/'original.axf')
assert sum('RoadHeadingControl.c' in x for x in paths)==1
assert any(g.findtext('GroupName')=='Control' and any('RoadHeadingControl.c' in (x.text or '') for x in g.findall('.//FilePath')) for g in tree.findall('.//Group'))
assert all(x in app for x in ['HIDE_READY','HIDE_ALIGN','TURN_EXIT_RECOVER','RoadHeading_AxisError'])
rules=ROOT/'Docs/修改规范与回归清单.md'
assert '包括同一轮中的后续修改' in rules.read_text(encoding='utf-8')
assert '包括同一轮中的后续修改' in (ROOT/'AGENTS.md').read_text(encoding='utf-8')
regression16=json.loads((ROOT/'Tests/results_documents16.json').read_text())
assert not regression16['failures']
assert len(regression16['matrix']['completed'])==72 and len(regression16['matrix']['ignored'])==8
assert regression16['firmware_sha256']==sha(axf)
assert regression16['app_source_sha256']==sha(ROOT/'User/App_ThiefStrategy.c')
assert regression16['test_script_sha256']==sha(ROOT/'Tests/test_documents16.py')
for name,value in regression16['helper_sha256'].items():assert sha(ROOT/'Tests'/name)==value
before16=json.loads((before/'regression_before.json').read_text())
assert {x['case'] for x in before16['failures']}=={'plateau','threat_matrix'}
assert before16['test_script_sha256']==sha(before/'test_documents16_before.py')
assert before16['firmware_sha256']==sha(before/'original.axf')
assert 'HIDE_HEADING_NOT_ALIGNED' not in app and 'HIDE_YAW_DIRECTION_UNKNOWN' not in app
assert all(x in app for x in ['HIDE_ALIGN_END','hide_escape_mask','Route_EndHideAlign(now,"THREAT")','Route_EndHideAlign(now,"DEFERRED")'])
assert sha(ROOT/'DOCUMENTS15_CHANGESET.patch')==sha(before/'DOCUMENTS15_CHANGESET.patch'),'historical RUN7 diff changed'



regression=json.loads((ROOT/'Tests/results_documents13.json').read_text())
assert not regression['failures'] and regression['firmware_sha256']==sha(axf)
assert regression['app_source_sha256']==sha(ROOT/'User/App_ThiefStrategy.c')
assert regression['test_script_sha256']==sha(ROOT/'Tests/test_documents13.py')
for name,value in regression['helper_sha256'].items():assert sha(ROOT/'Tests'/name)==value
prior_regression=json.loads((ROOT/'Tests/results_documents12.json').read_text())
assert not prior_regression['failures'] and prior_regression['brake_credit_matrix_cases']==240
assert prior_regression['firmware_sha256']==sha(axf)
assert prior_regression['app_source_sha256']==sha(ROOT/'User/App_ThiefStrategy.c')
assert prior_regression['test_script_sha256']==sha(ROOT/'Tests/test_documents12.py')
assert '#define THIEF_LASER_TRIGGER_MM 1000U' in (ROOT/'Algorithm/ThiefStrategy/ThiefStrategyConfig.h').read_text()
for name,value in prior_regression['helper_sha256'].items():assert sha(ROOT/'Tests'/name)==value
fix_diff=[]
fix_files=['User/App_ThiefStrategy.c','Tests/test_documents15.py','Tests/test_escape_completion.py','Tests/test_documents16.py','Tests/verify_snapshot.py','README.md','Docs/修改规范与回归清单.md']
for rel in fix_files:
    old=before/rel;current=ROOT/rel
    fix_diff.extend(difflib.unified_diff(old.read_text(encoding='utf-8').splitlines(True) if old.exists() else [],
        current.read_text(encoding='utf-8').splitlines(True),fromfile='16U_RUN7/'+rel,tofile='16U_RUN8/'+rel))
(ROOT/'DOCUMENTS16_CHANGESET.patch').write_text(''.join(fix_diff),encoding='utf-8')
old_project=OLD/'MDK-ARM'/(OLD.name+'.uvprojx')
assert (ROOT/(ROOT.name+'.ioc')).read_text(encoding='utf-8')==\
    (OLD/(OLD.name+'.ioc')).read_text(encoding='utf-8').replace(OLD.name,ROOT.name), 'CubeMX settings changed'
diff.extend(difflib.unified_diff(old_project.read_text(encoding='utf-8').splitlines(True),
    project.read_text(encoding='utf-8').splitlines(True),fromfile='baseline/project.uvprojx',tofile='16U/project.uvprojx'))
for rel in ['README.md','.gitignore','Tests/test_16m.c','Tests/test_app_runtime.py','Tests/verify_snapshot.py','Tests/test_escape_completion.py','Tests/test_documents12.py','Tests/test_documents13.py','Tests/test_speed16t.py','Tests/test_documents14.py','Tests/test_documents15.py','Tests/test_documents16.py','AGENTS.md','Docs/修改规范与回归清单.md']:
    p=ROOT/rel;old=OLD/rel
    diff.extend(difflib.unified_diff(old.read_text(encoding='utf-8').splitlines(True) if old.exists() else [],
        p.read_text(encoding='utf-8').splitlines(True),fromfile='baseline/'+rel,tofile='16U/'+rel))
    changed.append(rel)
(ROOT/'SOURCE_CHANGESET.patch').write_text(''.join(diff),encoding='utf-8')
added += [ROOT.name+'.ioc','MDK-ARM/'+ROOT.name+'.uvprojx','MDK-ARM/'+ROOT.name+'.uvoptx']
for inc in tree.findall('.//IncludePath'):
    for item in (inc.text or '').split(';'):
        if item:assert (project.parent/item.replace('\\','/')).resolve().is_dir(),item
report=dict(baseline=str(OLD),new_project=str(ROOT),git_available=False,
 baseline_sha256_at_verification=original,modified_source_files=changed,added_or_renamed_files=added,
 unchanged_source_count=len(equal),unchanged_hardware_core=True,project_file_count=len(paths),
 duplicate_sources=0,compile_errors=0,compile_warnings=0,hex_sha256=sha(hexfile),
 algorithm_checks=result['test_checks'],application_checks=app_result['checks'],
 hardware_tested=False,flashed=False,firmware='UARTCH4_16U_RUN8',
 documents16_source_files=fix_sources,documents16_fix_files=fix_files,
 documents16_checks=regression16['checks'],documents16_escape_cases=len(regression16['matrix']['completed']),
 documents16_ignored_cases=len(regression16['matrix']['ignored']),
 documents16_backup=str(before),documents16_log_sha256=sha(before/'Documents16.txt'),
 documents15_checks=regression15['checks'],documents15_cases=regression15['cases'],
 documents15_backup=str(before15dir),documents15_log_sha256=sha(before15dir/'Documents15.txt'),
 rules_sha256=sha(rules),agents_sha256=sha(ROOT/'AGENTS.md'),axf_sha256=sha(axf),
 documents14_checks=regression14['checks'],heading_sweep_cases=regression14['heading_sweep_cases'],gray_recovery_yaw_tolerance_x100=1000,
 documents14_log_sha256=sha(before14dir/'Documents14.txt'),
 speed_restore_checks=speed['checks'],reference_speed_checks=reference_speed['checks'],speed_curves_match_16t=True,
 documents13_checks=regression['checks'],documents12_checks=prior_regression['checks'],
 brake_credit_matrix_cases=prior_regression['brake_credit_matrix_cases'],trigger_center_mm=1000,
 documents14_backup=str(before14dir),speed_restore_backup=str(ROOT/'Tests/before_speed16t'),escape_completion_checks=escape_result['checks'],
 escape_completion_cases=len(escape_result['escapes']),ignored_no_road_cases=len(escape_result['ignored']),
 interception_completion_cases=len(escape_result['interceptions']),
 generated_evidence=['build_16U.log','Tests/results_16m.json','Tests/results_app_runtime.json',
 'Tests/Documents10_evidence.json','Tests/results_escape_completion.json',
 'Tests/before_documents13/regression_before.json','Tests/results_documents12.json','Tests/results_documents13.json','DOCUMENTS13_CHANGESET.patch','Tests/results_speed16t.json','Tests/results_speed16t_reference.json','SPEED16T_CHANGESET.patch','DOCUMENTS14_CHANGESET.patch','Tests/results_documents14.json','Tests/before_documents14/regression_before.json','DOCUMENTS15_CHANGESET.patch','Tests/results_documents15.json','Tests/before_documents15/regression_before.json','DOCUMENTS16_CHANGESET.patch','Tests/results_documents16.json','Tests/before_documents16/regression_before.json','Docs/修改规范与回归清单.md','AGENTS.md','SOURCE_CHANGESET.patch','VALIDATION_16U.json'],
 notes='16U RUN8 keeps lasers active during confirmed-node parking trim; a confirmed threat preempts trim after measured braking; trim timeout is nonfatal.16T speeds,10deg recovery and1000mm trigger retained in the user-specified Desktop project. Before-fix files/AXF and source hashes are saved in Tests/before_documents16. No hardware execution.')
(ROOT/'VALIDATION_16U.json').write_text(json.dumps(report,indent=2,ensure_ascii=False)+'\n',encoding='utf-8')
print(json.dumps({k:v for k,v in report.items() if k!='baseline_sha256_at_verification'},indent=2,ensure_ascii=False))
