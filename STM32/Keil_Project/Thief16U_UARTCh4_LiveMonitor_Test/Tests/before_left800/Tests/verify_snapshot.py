# -*- coding: utf-8 -*-
"""Audit the new copy and generate file-level evidence in this ZIP checkout."""
from pathlib import Path
import hashlib,json,difflib,re,xml.etree.ElementTree as ET
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
assert set(changed)==set(expected_run3+['Control/RestartLineSearch.c','Control/RestartLineSearch.h','Algorithm/ThiefStrategy/ThiefInterceptPolicy.c','Algorithm/ThiefStrategy/ThiefStrategyConfig.h','Algorithm/MapLocalization/ThiefLocalization.c','Algorithm/MapLocalization/ThiefLocalization.h','Algorithm/ThiefStrategy/ThiefMapPlanner.c','Algorithm/ThiefStrategy/ThiefRiskSearch.c','Algorithm/ThiefStrategy/ThiefRiskSearch.h','Hardware/LaserRx.c','Control/TravelFrame.c','Control/TravelFrame.h']) and set(added)=={'Control/RoadHeadingControl.c','Control/RoadHeadingControl.h','Algorithm/MapRoute/RouteHistory.c','Algorithm/MapRoute/RouteHistory.h'}
for directory in ['Core','Drivers','Hardware','System']:
    assert not any(p.startswith(directory+'/') and p!='Hardware/LaserRx.c' for p in changed+added),directory+' unexpectedly changed'
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
assert 'ALGO=16U' in app and 'TRIGGER_RAW_MM=1000' in app and 'REPLAN=INTERCEPT_RETREAT_FIRST' in app
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
assert len(escape_result['escapes'])==40 and len(escape_result['ignored'])==0
assert len(escape_result['interceptions'])==4
assert 'UARTCH4_16U_RUN21' in app
before=ROOT/'Tests/before_documents123154'
gray_backup=ROOT/'Tests/before_gray_priority'
before31dir=ROOT/'Tests/before_documents31'
before29dir=ROOT/'Tests/before_documents29'
before24dir=ROOT/'Tests/before_documents24'
before_hashes=json.loads((before/'source_sha256.json').read_text())
fix_sources=[rel for rel,value in before_hashes.items() if sha(ROOT/rel)!=value]
assert set(fix_sources)=={'User/App_ThiefStrategy.c','Algorithm/ThiefStrategy/FrontLaserGate.c','Algorithm/ThiefStrategy/ThiefStrategyConfig.h','Algorithm/ThiefStrategy/ThiefMapPlanner.c','Hardware/LaserRx.c','Control/TravelFrame.c','Control/TravelFrame.h','Algorithm/MapLocalization/ThiefLocalization.c','Algorithm/MapRoute/MapRouteNavigator.c'},fix_sources
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
assert len(regression16['matrix']['completed'])==80 and len(regression16['matrix']['ignored'])==0
assert regression16['firmware_sha256']==sha(axf)
assert regression16['app_source_sha256']==sha(ROOT/'User/App_ThiefStrategy.c')
assert regression16['test_script_sha256']==sha(ROOT/'Tests/test_documents16.py')
for name,value in regression16['helper_sha256'].items():assert sha(ROOT/'Tests'/name)==value
before16dir=ROOT/'Tests/before_documents16'
before16=json.loads((before16dir/'regression_before.json').read_text())
assert {x['case'] for x in before16['failures']}=={'plateau','threat_matrix'}
assert before16['test_script_sha256']==sha(before16dir/'test_documents16_before.py')
assert before16['firmware_sha256']==sha(before16dir/'original.axf')
assert 'HIDE_HEADING_NOT_ALIGNED' not in app and 'HIDE_YAW_DIRECTION_UNKNOWN' not in app
assert all(x in app for x in ['HIDE_ALIGN_END','hide_escape_mask','Route_EndHideAlign(now,"THREAT")','Route_EndHideAlign(now,"DEFERRED")'])
assert sha(ROOT/'DOCUMENTS15_CHANGESET.patch')==sha(before16dir/'DOCUMENTS15_CHANGESET.patch'),'historical RUN7 diff changed'
regression17=json.loads((ROOT/'Tests/results_documents17.json').read_text())
assert not regression17['failures']
assert regression17['cases']['parking_pulses']==40 and regression17['cases']['turn_visible_resume']==32
assert regression17['cases']['visible_road_speed']==4 and 'pulse_boundaries' in regression17['passed']
assert regression17['firmware_sha256']==sha(axf)
assert regression17['app_source_sha256']==sha(ROOT/'User/App_ThiefStrategy.c')
assert regression17['test_script_sha256']==sha(ROOT/'Tests/test_documents17.py')
for name,value in regression17['helper_sha256'].items():assert sha(ROOT/'Tests'/name)==value
before17dir=ROOT/'Tests/before_documents17'
before17=json.loads((before17dir/'regression_before.json').read_text())
assert {x['case'] for x in before17['failures']}=={'parking_pulses','turn_visible_resume','visible_road_speed'}
assert before17['test_script_sha256']==sha(before17dir/'test_documents17_before.py')
assert before17['firmware_sha256']==sha(before17dir/'original.axf')
assert all(x in app for x in ['RoadHeading_UpdatePulse','RoadHeading_UpdateTurn','PARK_MOTOR','turn_exit_trim_started'])
assert sha(ROOT/'DOCUMENTS16_CHANGESET.patch')==sha(before17dir/'DOCUMENTS16_CHANGESET.patch'),'historical RUN8 diff changed'
regression18=json.loads((ROOT/'Tests/results_documents18.json').read_text())
assert not regression18['failures'] and regression18['cases']['rear_first_plans']==10
assert set(regression18['passed'])=={'channel_off','boot_three_channels','rear_first_plans','side_replay_after_wait','latched_distance','oncoming_during_cooldown'}
assert regression18['firmware_sha256']==sha(axf)
assert regression18['app_source_sha256']==sha(ROOT/'User/App_ThiefStrategy.c')
assert regression18['test_script_sha256']==sha(ROOT/'Tests/test_documents18.py')
for name,value in regression18['helper_sha256'].items():assert sha(ROOT/'Tests'/name)==value
before18dir=ROOT/'Tests/before_documents18'
before18=json.loads((before18dir/'regression_before.json').read_text())
assert {x['case'] for x in before18['failures']}==set(regression18['passed'])
assert before18['test_script_sha256']==sha(before18dir/'test_documents18_before.py')
assert before18['firmware_sha256']==sha(before18dir/'original.axf')
assert 'THIEF_LASER_CHANNEL_MASK 0x0FU' in (ROOT/'Algorithm/ThiefStrategy/ThiefStrategyConfig.h').read_text()
assert 'THIEF_LASER_CHANNEL_MASK' in (ROOT/'Hardware/LaserRx.c').read_text()
assert 'status="DISABLED"' in app and 'RIGHT=CH3' in app
assert 'Thief_LatchClearance' in app
assert {x['laser'] for x in app_result['departure_laser_plans']}=={'FRONT','LEFT','RIGHT','REAR'}
assert sha(ROOT/'DOCUMENTS17_CHANGESET.patch')==sha(before18dir/'DOCUMENTS17_CHANGESET.patch'),'historical RUN9 diff changed'
regression19=json.loads((ROOT/'Tests/results_documents19.json').read_text())
assert not regression19['failures'] and regression19['cases']['ordinary_turn_matrix']==448
assert regression19['cases']['dual_gray_frame_symmetry']['samples']==14
assert regression19['firmware_sha256']==sha(axf)
assert regression19['app_source_sha256']==sha(ROOT/'User/App_ThiefStrategy.c')
assert regression19['test_script_sha256']==sha(ROOT/'Tests/test_documents19.py')
for name,value in regression19['helper_sha256'].items():assert sha(ROOT/'Tests'/name)==value
before19dir=ROOT/'Tests/before_documents19'
before19=json.loads((before19dir/'regression_before.json').read_text())
assert {x['case'] for x in before19['failures']}=={'ordinary_turn','reverse_first_follow','approach_corridor'}
assert before19['firmware_sha256']==sha(before19dir/'original.axf')
assert before19['test_script_sha256']==sha(before19dir/'test_documents19_before.py')
assert sha(ROOT/'DOCUMENTS18_CHANGESET.patch')==sha(before19dir/'DOCUMENTS18_CHANGESET.patch'),'historical RUN10 diff changed'
reference19=Path(r'C:/Users/Administrator/Desktop/Gray12_SideTurn_LineFollow_Test')
reference19_hashes=json.loads((before19dir/'reference_sha256.json').read_text())
for name,value in reference19_hashes.items():assert sha(reference19/name)==value,'reference modified: '+name
assert 'REVERSE_START_REQUIRES_LEADING_LINE' not in app
assert 'ApproachCorridor' in (ROOT/'Algorithm/ThiefStrategy/ThiefRiskSearch.c').read_text()
north_escape=next(x for x in escape_result['escapes'] if x['start']==38 and x['start_body']==1 and x['laser']=='LEFT')
assert north_escape['fixes'][0]==56 and north_escape['node']==north_escape['goal'],'latest single north threat prefers the clear opposite south exit'
regression20=json.loads((ROOT/'Tests/results_documents20.json').read_text())
assert not regression20['failures'] and regression20['cases']['straight_exit_gap']==12
assert regression20['cases']['startup_and_road_scopes']==12
assert regression20['cases']['complete_history_return']['returned']['fixes']==[55,37]
for key in ['gray_failure_backtrack','obstacle_history_return']:
    assert regression20['cases'][key]['fixes'][:2]==[55,37]
    assert regression20['cases'][key]['node'] in [11,26,35,38,63]
assert regression20['firmware_sha256']==sha(axf)
assert regression20['app_source_sha256']==sha(ROOT/'User/App_ThiefStrategy.c')
assert regression20['test_script_sha256']==sha(ROOT/'Tests/test_documents20.py')
for name,value in regression20['helper_sha256'].items():assert sha(ROOT/'Tests'/name)==value
before20dir=ROOT/'Tests/before_documents20'
before20=json.loads((before20dir/'regression_before.json').read_text())
assert {x['case'] for x in before20['failures']}=={'straight_exit_gap','middle_both_missing','history_available'}
assert before20['firmware_sha256']==sha(before20dir/'original.axf')
assert before20['test_script_sha256']==sha(before20dir/'test_documents20_before.py')
assert sha(ROOT/'DOCUMENTS19_CHANGESET.patch')==sha(before20dir/'DOCUMENTS19_CHANGESET.patch'),'historical RUN11 diff changed'
assert set(re.findall(r'FW=(UARTCH4_16U_RUN[0-9]+)',app))=={'UARTCH4_16U_RUN21'},'mixed firmware labels'
assert 'FRONT_NO_LINE"' not in app and 'GRAY_NO_LINE_NO_CONTEXT' in app
assert all(x in app for x in ['RouteHistory_Record','RouteHistory_PlanReturn','GRAY_BACKTRACK','HISTORY_RETURN'])
assert sum('RouteHistory.c' in x for x in paths)==1
assert any(any('RouteHistory.c' in (x.text or '') for x in g.findall('.//FilePath')) and
           any('MapRouteNavigator.c' in (x.text or '') for x in g.findall('.//FilePath')) for g in tree.findall('.//Group'))
assert 'ROUTE_HISTORY_CAPACITY 32U' in (ROOT/'Algorithm/MapRoute/RouteHistory.h').read_text()
regression21=json.loads((ROOT/'Tests/results_documents21.json').read_text())
assert not regression21['failures'] and regression21['cases']['signed_localization_matrix']==72
assert regression21['cases']['overrun_return']['fixes'][0]==4
assert regression21['cases']['overrun_return']['node'] in [11,26,35,38,63]
assert regression21['firmware_sha256']==sha(axf)
assert regression21['app_source_sha256']==sha(ROOT/'User/App_ThiefStrategy.c')
assert regression21['test_script_sha256']==sha(ROOT/'Tests/test_documents21.py')
for name,value in regression21['helper_sha256'].items():assert sha(ROOT/'Tests'/name)==value
before21dir=ROOT/'Tests/before_documents21'
before21=json.loads((before21dir/'regression_before.json').read_text())
assert {x['case'] for x in before21['failures']}=={'overrun_return','retreat_side_continuity'}
assert before21['firmware_sha256']==sha(before21dir/'original.axf')
assert before21['test_script_sha256']==sha(before21dir/'test_documents21_before.py')
assert sha(ROOT/'DOCUMENTS20_CHANGESET.patch')==sha(before21dir/'DOCUMENTS20_CHANGESET.patch'),'historical RUN12 diff changed'
assert 'reversal_entry_mm' in (ROOT/'Algorithm/MapLocalization/ThiefLocalization.h').read_text()
assert 'HasContinuation' in (ROOT/'Algorithm/MapLocalization/ThiefLocalization.c').read_text()
assert all(x in app for x in ['REVERSE_READY','REVERSE_REJECT','if(retreat_active && !(mask&(1U<<s->heading)))return;'])
old_test=(before21dir/'Tests/test_16m.c').read_text()
# RUN13's localization-only change is preserved in the RUN18 baseline. RUN19
# intentionally revises sensor geometry/range expectations under user direction.
prior_algorithm=(before/'Tests/test_16m.c').read_text()
assert old_test.replace('ReverseLeg(&l,&r,301,','ReverseLeg(&l,&r,581,',1)==prior_algorithm
current_algorithm=(ROOT/'Tests/test_16m.c').read_text()
def reverse_test_block(text):
    return text.split('static void TestReverse(void)',1)[1].split('/* Independent',1)[0]
assert reverse_test_block((gray_backup/'Tests/test_16m.c').read_text())==reverse_test_block(prior_algorithm)
assert 'l.reversal_entry_mm==-281L' in current_algorithm
regression22=json.loads((ROOT/'Tests/results_documents22.json').read_text())
assert not regression22['failures'] and regression22['cases']['scan_matrix']==12
assert regression22['cases']['east_obstacle_override']==4
assert len(regression22['cases']['n038_east_first'])==2
for case in regression22['cases']['n038_east_first']:assert case['fixes'][0]==56 and case['node'] in [11,26,35,38,63]
assert regression22['firmware_sha256']==sha(axf)
assert regression22['app_source_sha256']==sha(ROOT/'User/App_ThiefStrategy.c')
assert regression22['test_script_sha256']==sha(ROOT/'Tests/test_documents22.py')
for name,value in regression22['helper_sha256'].items():assert sha(ROOT/'Tests'/name)==value
before22dir=ROOT/'Tests/before_documents22'
before22=json.loads((before22dir/'regression_before.json').read_text())
assert {x['case'] for x in before22['failures']}=={'n038_east_first','exit_distance_scan'}
assert before22['firmware_sha256']==sha(before22dir/'original.axf')
assert before22['test_script_sha256']==sha(before22dir/'test_documents22_before.py')
assert sha(ROOT/'DOCUMENTS21_CHANGESET.patch')==sha(before22dir/'DOCUMENTS21_CHANGESET.patch'),'historical RUN13 diff changed'
assert 'MapRoutePosition_DistanceMm(turn_exit_line_counts)>=100L' in app
assert 'mm-map_gray_start_mm>=80L' in app
assert sha(ROOT/'DOCUMENTS22_CHANGESET.patch')==sha(before24dir/'DOCUMENTS22_CHANGESET.patch')
assert all(x in app for x in ['TURN_EXIT_SCAN','RestartLineSearch_BeginScan','turn_exit_line_phase!=2U'])
assert 'ThiefMapPlanner_PlanEscape' in (ROOT/'Algorithm/ThiefStrategy/ThiefMapPlanner.c').read_text()









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
regression24=json.loads((ROOT/'Tests/results_documents24.json').read_text())
assert not regression24['failures'] and regression24['cases']['slow_correction']==4
assert regression24['cases']['n010_log_replay']==1
assert regression24['firmware_sha256']==sha(axf)
assert regression24['app_source_sha256']==sha(ROOT/'User/App_ThiefStrategy.c')
assert regression24['test_script_sha256']==sha(ROOT/'Tests/test_documents24.py')
for name,value in regression24['helper_sha256'].items():assert sha(ROOT/'Tests'/name)==value
before24=json.loads((before24dir/'regression_before.json').read_text())
assert {x['case'] for x in before24['failures']}=={'slow_correction','deadline_line','stalled_small_error','n010_log_replay'}
assert before24['firmware_sha256']==sha(before24dir/'original.axf')
assert before24['test_script_sha256']==sha(before24dir/'test_documents24_before.py')
assert sha(ROOT/'EXIT100_CHANGESET.patch')==sha(before24dir/'EXIT100_CHANGESET.patch')
regression29=json.loads((ROOT/'Tests/results_documents29.json').read_text())
assert not regression29['failures'] and regression29['actual_speed_enable_bookkeeping']
assert regression29['cases']['reverse_matrix']==20
assert regression29['cases']['after_turn_reverse']['fixes'][0]==40
assert regression29['cases']['after_turn_reverse']['node'] in [11,26,35,38,63]
assert regression29['firmware_sha256']==sha(axf)
assert regression29['app_source_sha256']==sha(ROOT/'User/App_ThiefStrategy.c')
assert regression29['test_script_sha256']==sha(ROOT/'Tests/test_documents29.py')
for name,value in regression29['helper_sha256'].items():assert sha(ROOT/'Tests'/name)==value
before29=json.loads((before29dir/'regression_before.json').read_text())
assert {x['case'] for x in before29['failures']}=={'reverse_matrix','supported_over_one_second','continuous_both_missing','after_turn_reverse'}
assert before29['firmware_sha256']==sha(before29dir/'original.axf')
assert before29['test_script_sha256']==sha(before29dir/'test_documents29_before.py')
assert sha(ROOT/'DOCUMENTS24_CHANGESET.patch')==sha(before29dir/'DOCUMENTS24_CHANGESET.patch')
assert 'map_gray_braked=1U;map_gray_missing_active=0U;' in app
assert 'now-map_gray_missing_tick)>=2000U' in app
regression31=json.loads((ROOT/'Tests/results_documents31.json').read_text())
assert not regression31['failures']
assert len(regression31['cases']['mismatch_retry'])==2
assert len(regression31['cases']['full_black_next_junction'])==8
assert regression31['cases']['emergency_clear_departure']['first']==77
assert regression31['cases']['scan_junction_brake']['node'] in [11,26,35,38,63]
assert regression31['cases']['emergency_clear_departure']['completion']['node'] in [11,26,35,38,63]
assert regression31['firmware_sha256']==sha(axf)
assert regression31['app_source_sha256']==sha(ROOT/'User/App_ThiefStrategy.c')
assert regression31['test_script_sha256']==sha(ROOT/'Tests/test_documents31.py')
for name,value in regression31['helper_sha256'].items():assert sha(ROOT/'Tests'/name)==value
before31=json.loads((before31dir/'regression_before.json').read_text())
assert {x['case'] for x in before31['failures']}=={'mismatch_retry','full_black_next_junction','emergency_clear_departure','scan_junction_brake'}
assert before31['firmware_sha256']==sha(before31dir/'original.axf')
assert before31['test_script_sha256']==sha(before31dir/'test_documents31_before.py')
assert sha(ROOT/'DOCUMENTS29_CHANGESET.patch')==sha(before31dir/'DOCUMENTS29_CHANGESET.patch')
regression123=json.loads((ROOT/'Tests/results_documents123154.json').read_text())
assert not regression123['failures']
assert regression123['cases']['trailing_return_matrix']['combinations']==40
assert len(regression123['cases']['trailing_return_matrix']['completed'])==8
assert regression123['firmware_sha256']==sha(axf)
assert regression123['test_script_sha256']==sha(ROOT/'Tests/test_documents123154.py')
for name,value in regression123['source_sha256'].items():assert sha(ROOT/name)==value
for name,value in regression123['helper_sha256'].items():assert sha(ROOT/'Tests'/name)==value
prior123=json.loads((before/'regression_before.json').read_text())
assert {x['case'] for x in prior123['failures']}=={'trailing_return_matrix','return_requires_real_evidence','raw1000_four_channels','opposite_departure'}
assert prior123['firmware_sha256']==sha(before/'original.axf')
assert prior123['test_script_sha256']==sha(before/'test_documents123154_before.py')
assert sha(ROOT/'DOCUMENTS31_CHANGESET.patch')==sha(before/'DOCUMENTS31_CHANGESET.patch')
range_backup=ROOT/'Tests/before_right900'
assert sha(ROOT/'DOCUMENTS123154_CHANGESET.patch')==sha(range_backup/'DOCUMENTS123154_CHANGESET.patch')
regression900=json.loads((ROOT/'Tests/results_right900.json').read_text())
assert not regression900['failures'] and regression900['cases']['boundary_matrix']==32
assert regression900['cases']['right_cap_independent_of_heading']==12
assert regression900['firmware_sha256']==sha(axf)
assert regression900['test_sha256']==sha(ROOT/'Tests/test_right900.py')
for n,h in regression900['source_sha256'].items():assert sha(ROOT/n)==h
for n,h in regression900['helper_sha256'].items():assert sha(ROOT/'Tests'/n)==h
range_before=json.loads((range_backup/'regression_before.json').read_text())
assert {x['case'] for x in range_before['failures']}=={'boundary_matrix','right_cap_independent_of_heading'}
assert range_before['firmware_sha256']==sha(range_backup/'original.axf')
assert range_before['test_sha256']==sha(ROOT/'Tests/test_right900.py')
range_hashes=json.loads((range_backup/'source_sha256.json').read_text())
range_sources=[n for n,h in range_hashes.items() if sha(ROOT/n)!=h]
assert set(range_sources)=={'User/App_ThiefStrategy.c','Algorithm/ThiefStrategy/FrontLaserGate.c','Algorithm/ThiefStrategy/ThiefStrategyConfig.h','Algorithm/MapLocalization/ThiefLocalization.c','Algorithm/MapRoute/MapRouteNavigator.c'}
fix_files=sorted(set(fix_sources+['Tests/test_documents123154.py','Tests/test_16m.c','Tests/test_app_runtime.py','Tests/test_escape_completion.py','Tests/test_documents12.py','Tests/test_documents16.py','Tests/test_documents18.py','Tests/test_documents22.py','Tests/verify_snapshot.py','README.md','Docs/修改规范与回归清单.md','AGENTS.md']))
range_files=json.loads((gray_backup/'VALIDATION_16U.json').read_text(encoding='utf-8'))['right900_modified_files']
assert sha(ROOT/'RIGHT900_CHANGESET.patch')==sha(gray_backup/'RIGHT900_CHANGESET.patch')
gray_result=json.loads((ROOT/'Tests/results_gray_priority.json').read_text())
assert not gray_result['failures'] and gray_result['cases']['observed_node_matrix']==56
assert gray_result['cases']['reverse_signed_ledger']==12
assert gray_result['cases']['ambiguous_far_node_sequence']==1
assert gray_result['cases']['n004_to_n006_complete']['node'] in [11,26,35,38,63]
assert gray_result['firmware_sha256']==sha(axf)
assert gray_result['test_sha256']==sha(ROOT/'Tests/test_gray_priority.py')
for n,h in gray_result['source_sha256'].items():assert sha(ROOT/n)==h
for n,h in gray_result['helper_sha256'].items():assert sha(ROOT/'Tests'/n)==h
gray_before=json.loads((gray_backup/'regression_before.json').read_text())
assert len(gray_before['failures'])==6 and gray_before['firmware_sha256']==sha(gray_backup/'original.axf')
assert gray_before['test_sha256']==sha(ROOT/'Tests/test_gray_priority.py')
gray_hashes=json.loads((gray_backup/'source_sha256.json').read_text())
gray_sources=[n for n,h in gray_hashes.items() if sha(ROOT/n)!=h]
assert set(gray_sources)=={'User/App_ThiefStrategy.c','Algorithm/MapLocalization/ThiefLocalization.c','Algorithm/MapRoute/MapRouteNavigator.c'}
gray_files=gray_sources+['Tests/test_gray_priority.py','Tests/test_16m.c','Tests/test_documents12.py','Tests/test_documents21.py','Tests/verify_snapshot.py','README.md','Docs/修改规范与回归清单.md','AGENTS.md']
fix_diff=[]
for rel in gray_files:
    old=gray_backup/rel;current=ROOT/rel
    fix_diff.extend(difflib.unified_diff(old.read_text(encoding='utf-8-sig').splitlines(True) if old.exists() else [],
        current.read_text(encoding='utf-8-sig').splitlines(True),fromfile='16U_RUN20/'+rel,tofile='16U_RUN21/'+rel))
(ROOT/'GRAY_PRIORITY_CHANGESET.patch').write_text(''.join(fix_diff),encoding='utf-8')
historical=json.loads((before/'VALIDATION_16U.json').read_text(encoding='utf-8'))
old_project=OLD/'MDK-ARM'/(OLD.name+'.uvprojx')
assert (ROOT/(ROOT.name+'.ioc')).read_text(encoding='utf-8')==\
    (OLD/(OLD.name+'.ioc')).read_text(encoding='utf-8').replace(OLD.name,ROOT.name), 'CubeMX settings changed'
diff.extend(difflib.unified_diff(old_project.read_text(encoding='utf-8').splitlines(True),
    project.read_text(encoding='utf-8').splitlines(True),fromfile='baseline/project.uvprojx',tofile='16U/project.uvprojx'))
for rel in ['README.md','.gitignore','Tests/test_16m.c','Tests/run_16m.py','Tests/test_app_runtime.py','Tests/verify_snapshot.py','Tests/test_escape_completion.py','Tests/test_documents12.py','Tests/test_documents13.py','Tests/test_speed16t.py','Tests/test_documents14.py','Tests/test_documents15.py','Tests/test_documents16.py','Tests/test_documents17.py','Tests/test_documents18.py','Tests/test_documents19.py','Tests/test_documents20.py','Tests/test_documents21.py','Tests/test_documents22.py','Tests/test_documents24.py','Tests/test_documents29.py','Tests/test_documents31.py','Tests/test_documents123154.py','Tests/test_right900.py','Tests/test_gray_priority.py','AGENTS.md','Docs/修改规范与回归清单.md']:
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
 unchanged_source_count=len(equal),unchanged_hardware_core=False,unchanged_mcu_core_and_pin_drivers=True,hardware_wrapper_changes=['Hardware/LaserRx.c'],project_file_count=len(paths),
 duplicate_sources=0,compile_errors=0,compile_warnings=0,hex_sha256=sha(hexfile),
 algorithm_checks=result['test_checks'],application_checks=app_result['checks'],
 hardware_tested=False,flashed=False,firmware='UARTCH4_16U_RUN21',
 gray_priority_checks=gray_result['checks'],gray_priority_cases=gray_result['cases'],gray_priority_backup=str(gray_backup),gray_priority_modified_files=gray_files,gray_priority_file_sha256={n:sha(ROOT/n) for n in gray_files},gray_priority_patch_sha256=sha(ROOT/'GRAY_PRIORITY_CHANGESET.patch'),gray_priority_log_sha256=sha(gray_backup/'serial_160117.txt'),
 right900_checks=regression900['checks'],right900_cases=regression900['cases'],right900_backup=str(range_backup),right900_modified_files=range_files,right900_file_sha256={n:sha(ROOT/n) for n in range_files},right900_patch_sha256=sha(ROOT/'RIGHT900_CHANGESET.patch'),right900_raw_limits_mm=[1000,1000,900,1000],
 documents22_source_files=historical['documents22_source_files'],documents22_fix_files=historical['documents22_fix_files'],
 documents22_checks=regression22['checks'],documents22_cases=regression22['cases'],
 documents22_backup=str(before22dir),documents22_log_sha256=sha(before22dir/'Documents22.txt'),
 exit100_backup=str(ROOT/'Tests/before_exit100'),turn_exit_translation_limit_mm=100,
 documents24_backup=str(before24dir),documents24_checks=regression24['checks'],documents24_cases=regression24['cases'],documents24_log_sha256=sha(before24dir/'Documents24.txt'),
 documents24_modified_files=historical['documents24_modified_files'],documents24_file_sha256=historical['documents24_file_sha256'],
 exit_visible_tolerance_x100=1000,exit_heading_progress_x100=100,exit_inactivity_ms=2500,
 documents29_backup=str(before29dir),documents29_checks=regression29['checks'],documents29_cases=regression29['cases'],documents29_log_sha256=sha(before29dir/'Documents29.txt'),
 documents29_modified_files=historical['documents29_modified_files'],documents29_file_sha256=historical['documents29_file_sha256'],
 documents31_backup=str(before31dir),documents31_checks=regression31['checks'],documents31_cases=regression31['cases'],
 documents31_log_sha256=sha(before31dir/'Documents31.txt'),documents31_source_files=historical['documents31_source_files'],documents31_modified_files=historical['documents31_modified_files'],
 documents31_file_sha256=historical['documents31_file_sha256'],historical_change_metadata_from=str(before/'VALIDATION_16U.json'),
 documents123154_backup=str(before),documents123154_checks=regression123['checks'],documents123154_cases=regression123['cases'],
 documents123154_log_sha256=sha(before/'serial_123154.txt'),documents123154_source_files=fix_sources,documents123154_modified_files=fix_files,
 documents123154_file_sha256={name:sha(ROOT/name) for name in fix_files},
 road_both_missing_timeout_ms=2000,road_fixed_1000ms_fault_removed=True,road_80mm_history_return_is_auxiliary=True,
 north_threat_at_n038_first_node=56,missing_turn_exit_alignment_target_x100=200,stationary_exit_scan_limit_ms=2500,
 documents21_checks=regression21['checks'],documents21_cases=regression21['cases'],
 documents21_backup=str(before21dir),documents21_log_sha256=sha(before21dir/'Documents21.txt'),
 documents20_checks=regression20['checks'],documents20_cases=regression20['cases'],
 documents20_backup=str(before20dir),documents20_log_sha256=sha(before20dir/'Documents20.txt'),
 history_capacity=32,history_work_bytes=642,
 recovery_straight_exit_and_road_scan=True,recovery_can_return_on_confirmed_history=True,
 documents19_checks=regression19['checks'],documents19_cases=regression19['cases'],
 documents19_backup=str(before19dir),documents19_log_sha256=sha(before19dir/'Documents19.txt'),
 documents22_file_sha256=historical['documents22_file_sha256'],
 documents19_reference_source_sha256=reference19_hashes,documents19_reference_path=str(reference19),
 documents18_checks=regression18['checks'],documents18_cases=regression18['cases'],
 documents18_backup=str(before18dir),documents18_log_sha256=sha(before18dir/'Documents18.txt'),
 enabled_laser_channel_mask=15,disabled_laser_channel=None,front_sensor_channel=1,
 escape_departure_preference='single bearing: complete opposite departure first; then clear rear/side/corridor and bounded emergency',
 documents17_checks=regression17['checks'],documents17_cases=regression17['cases'],
 documents17_backup=str(before17dir),documents17_log_sha256=sha(before17dir/'Documents17.txt'),
 heading_control_sha256={x:sha(ROOT/x) for x in ['Control/RoadHeadingControl.c','Control/RoadHeadingControl.h']},
 documents16_checks=regression16['checks'],documents16_escape_cases=len(regression16['matrix']['completed']),
 documents16_ignored_cases=len(regression16['matrix']['ignored']),
 documents16_backup=str(before16dir),documents16_log_sha256=sha(before16dir/'Documents16.txt'),
 documents15_checks=regression15['checks'],documents15_cases=regression15['cases'],
 documents15_backup=str(before15dir),documents15_log_sha256=sha(before15dir/'Documents15.txt'),
 rules_sha256=sha(rules),agents_sha256=sha(ROOT/'AGENTS.md'),axf_sha256=sha(axf),
 documents14_checks=regression14['checks'],heading_sweep_cases=regression14['heading_sweep_cases'],gray_recovery_yaw_tolerance_x100=1000,
 documents14_log_sha256=sha(before14dir/'Documents14.txt'),
 speed_restore_checks=speed['checks'],reference_speed_checks=reference_speed['checks'],speed_curves_match_16t=True,
 documents13_checks=regression['checks'],documents12_checks=prior_regression['checks'],
 brake_credit_matrix_cases=prior_regression['brake_credit_matrix_cases'],trigger_raw_mm=1000,
 documents14_backup=str(before14dir),speed_restore_backup=str(ROOT/'Tests/before_speed16t'),escape_completion_checks=escape_result['checks'],
 escape_completion_cases=len(escape_result['escapes']),ignored_no_road_cases=len(escape_result['ignored']),
 interception_completion_cases=len(escape_result['interceptions']),
 generated_evidence=['GRAY_PRIORITY_CHANGESET.patch','Tests/results_gray_priority.json','Tests/before_gray_priority/regression_before.json','RIGHT900_CHANGESET.patch','Tests/results_right900.json','Tests/before_right900/regression_before.json','DOCUMENTS123154_CHANGESET.patch','Tests/results_documents123154.json','Tests/before_documents123154/regression_before.json','DOCUMENTS31_CHANGESET.patch','Tests/results_documents31.json','Tests/before_documents31/regression_before.json','DOCUMENTS29_CHANGESET.patch','Tests/results_documents29.json','Tests/before_documents29/regression_before.json','DOCUMENTS24_CHANGESET.patch','Tests/results_documents24.json','Tests/before_documents24/regression_before.json','EXIT100_CHANGESET.patch','build_16U.log','Tests/results_16m.json','Tests/results_app_runtime.json',
 'Tests/Documents10_evidence.json','Tests/results_escape_completion.json',
 'Tests/before_documents13/regression_before.json','Tests/results_documents12.json','Tests/results_documents13.json','DOCUMENTS13_CHANGESET.patch','Tests/results_speed16t.json','Tests/results_speed16t_reference.json','SPEED16T_CHANGESET.patch','DOCUMENTS14_CHANGESET.patch','Tests/results_documents14.json','Tests/before_documents14/regression_before.json','DOCUMENTS15_CHANGESET.patch','Tests/results_documents15.json','Tests/before_documents15/regression_before.json','DOCUMENTS16_CHANGESET.patch','Tests/results_documents16.json','Tests/before_documents16/regression_before.json','DOCUMENTS17_CHANGESET.patch','Tests/results_documents17.json','Tests/before_documents17/regression_before.json','DOCUMENTS18_CHANGESET.patch','Tests/results_documents18.json','Tests/before_documents18/regression_before.json','DOCUMENTS19_CHANGESET.patch','Tests/results_documents19.json','Tests/before_documents19/regression_before.json','Tests/before_documents19/reference_sha256.json','DOCUMENTS20_CHANGESET.patch','Tests/results_documents20.json','Tests/before_documents20/regression_before.json','DOCUMENTS21_CHANGESET.patch','Tests/results_documents21.json','Tests/before_documents21/regression_before.json','DOCUMENTS22_CHANGESET.patch','Tests/results_documents22.json','Tests/before_documents22/regression_before.json','Docs/修改规范与回归清单.md','AGENTS.md','SOURCE_CHANGESET.patch','VALIDATION_16U.json'],
 notes='RUN21 actual mapped gray nodes take precedence over auxiliary odometry, source rearm and ambiguous unconfirmed far-node exclusion retained, signed reversal ledger kept, both-no-line2s still stops. Original RUN20 immutable. RUN20 physical CH3 raw900 max, other3 raw1000 max; prior RUN19 source and evidence preserved. RUN19 short-return trailing real branch confirmation with mirrored physical bits, same confirmed node, old distance limits and fresh10degree yaw. CH1-CH4 restored; RUN20 raw50..900mm on physical CH3 and raw50..1000mm on other3, front/rear+150 and sides+100 for projection only; disconnected road rays accepted, off-field rejected. Single-bearing opposite complete route preferred. Normal16T speed unchanged. Legacy expectations explicitly updated to user requirements and originals retained. N001 correction plateau timeout unchanged; compile and emulation only, no serial, flash or motors.')
(ROOT/'VALIDATION_16U.json').write_text(json.dumps(report,indent=2,ensure_ascii=False)+'\n',encoding='utf-8')
print(json.dumps({k:v for k,v in report.items() if k!='baseline_sha256_at_verification'},indent=2,ensure_ascii=False))
