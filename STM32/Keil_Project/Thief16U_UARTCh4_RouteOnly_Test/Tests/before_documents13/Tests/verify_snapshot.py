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
assert set(changed)==set(expected_run3+['Algorithm/ThiefStrategy/ThiefInterceptPolicy.c','Algorithm/ThiefStrategy/ThiefStrategyConfig.h']) and not added
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
assert 'ALGO=16U' in app and 'TRIGGER_CENTER_MM=700' in app and 'REPLAN=INTERCEPT_RETREAT_FIRST' in app
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
assert 'UARTCH4_16U_RUN3' in app
before=ROOT/'Tests/before_documents12'
before_hashes=json.loads((before/'source_sha256.json').read_text())
fix_sources=[rel for rel,value in before_hashes.items() if sha(ROOT/rel)!=value]
assert set(fix_sources)==set(expected_run3),fix_sources
before_regression=json.loads((before/'regression_before.json').read_text())
assert len(before_regression['failures'])==6
regression=json.loads((ROOT/'Tests/results_documents12.json').read_text())
assert not regression['failures'] and regression['firmware_sha256']==sha(axf)
assert regression['app_source_sha256']==sha(ROOT/'User/App_ThiefStrategy.c')
assert regression['test_script_sha256']==sha(ROOT/'Tests/test_documents12.py')
for name,value in regression['helper_sha256'].items():assert sha(ROOT/'Tests'/name)==value
assert regression['brake_credit_matrix_cases']==240
fix_diff=[]
fix_files=expected_run3+['Tests/test_app_runtime.py','Tests/test_escape_completion.py','Tests/test_documents12.py','Tests/test_16m.c','Tests/verify_snapshot.py','README.md']
for rel in fix_files:
    old=before/rel;current=ROOT/rel
    fix_diff.extend(difflib.unified_diff(old.read_text(encoding='utf-8').splitlines(True) if old.exists() else [],
        current.read_text(encoding='utf-8').splitlines(True),fromfile='16U_RUN2/'+rel,tofile='16U_RUN3/'+rel))
(ROOT/'DOCUMENTS12_CHANGESET.patch').write_text(''.join(fix_diff),encoding='utf-8')
old_project=OLD/'MDK-ARM'/(OLD.name+'.uvprojx')
assert (ROOT/(ROOT.name+'.ioc')).read_text(encoding='utf-8')==\
    (OLD/(OLD.name+'.ioc')).read_text(encoding='utf-8').replace(OLD.name,ROOT.name), 'CubeMX settings changed'
diff.extend(difflib.unified_diff(old_project.read_text(encoding='utf-8').splitlines(True),
    project.read_text(encoding='utf-8').splitlines(True),fromfile='baseline/project.uvprojx',tofile='16U/project.uvprojx'))
for rel in ['README.md','.gitignore','Tests/test_16m.c','Tests/test_app_runtime.py','Tests/verify_snapshot.py','Tests/test_escape_completion.py','Tests/test_documents12.py']:
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
 hardware_tested=False,flashed=False,firmware='UARTCH4_16U_RUN3',
 documents12_fix_source_files=fix_sources,documents12_fix_files=fix_files,
 documents12_checks=regression['checks'],brake_credit_matrix_cases=regression['brake_credit_matrix_cases'],
 documents12_backup=str(before),escape_completion_checks=escape_result['checks'],
 escape_completion_cases=len(escape_result['escapes']),ignored_no_road_cases=len(escape_result['ignored']),
 interception_completion_cases=len(escape_result['interceptions']),
 generated_evidence=['build_16U.log','Tests/results_16m.json','Tests/results_app_runtime.json',
 'Tests/Documents10_evidence.json','Tests/results_escape_completion.json',
 'Tests/before_documents12/regression_before.json','Tests/results_documents12.json','DOCUMENTS12_CHANGESET.patch','SOURCE_CHANGESET.patch','VALIDATION_16U.json'],
 notes='16U RUN3 is an in-place Documents12 fix in the user-specified Desktop project. Before-fix files/AXF and source hashes are saved in Tests/before_documents12. No hardware execution.')
(ROOT/'VALIDATION_16U.json').write_text(json.dumps(report,indent=2,ensure_ascii=False)+'\n',encoding='utf-8')
print(json.dumps({k:v for k,v in report.items() if k!='baseline_sha256_at_verification'},indent=2,ensure_ascii=False))
