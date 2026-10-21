"""Read-only 16L source/configuration audit and isolated actual generator replay.

No Git mutations, compilation, flashing, hardware commands or shared Tests/build
writes. The only persistent output is VALIDATION_16L_INTEGRATION.json.
"""
from pathlib import Path
import collections
import hashlib
import importlib.util
import json
import re
import shutil
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
REPO = ROOT.parents[2]
NEW_NAME = ROOT.name
LOW = ('Core/', 'Hardware/', 'Control/', 'Drivers/', 'System/')
NEW_FILES = {'ThiefTerminalForecast.c', 'ThiefTerminalForecast.h'}
REPORT = ROOT/'VALIDATION_16L_INTEGRATION.json'
result = {'checks': {}, 'failures': [], 'hardware_run': False,
          'compile_run': False, 'scope': 'source/configuration and actual offline generator reproduction'}


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def check(name, passed, detail=None):
    result['checks'][name] = {'passed': bool(passed), 'detail': detail}
    if not passed:
        result['failures'].append(name)


def normalized(element):
    return (element.tag, tuple(sorted(element.attrib.items())),
            (element.text or '').strip(), tuple(normalized(c) for c in element))


def table(text, name, cols=None):
    pattern = rf'{re.escape(name)}\[88\]' + (rf'\[{cols}\]' if cols else '')
    block = re.search(pattern+r'\s*=\s*\{(.*?)\};', text, re.S).group(1)
    values = list(map(int, re.findall(r'(\d+)U', block)))
    return [values[n:n+cols] for n in range(0,len(values),cols)] if cols else values


def module(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    loaded = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(loaded)
    return loaded


def audit():
    reference = json.loads((ROOT/'copy_reference.json').read_text(encoding='utf-8'))
    source = Path(reference['source_project'])
    old_name = source.name
    copied = reference['copied_files']
    check('copy_reference_actual_schema', reference['target_project'].replace('\\','/')==str(ROOT).replace('\\','/')
          and len(copied)==reference['copy_file_count'] and all(set(('source','destination','sha256'))<=set(row) for row in copied),
          {'source_project':str(source),'copy_file_count':len(copied)})
    changed_source = [row['source'] for row in copied if sha(source/row['source'])!=row['sha256']]
    low_rows = [row for row in copied if row['source'].startswith(LOW)]
    changed_low = [row['destination'] for row in low_rows if sha(ROOT/row['destination'])!=row['sha256']]
    check('source_16k_copied_bytes_preserved', not changed_source, {'files':len(copied),'changed':changed_source})
    check('117_low_level_bytes_identical', len(low_rows)==117 and not changed_low,
          {'files':len(low_rows),'changed':changed_low,'covers':list(LOW)})

    uv = ROOT/'MDK-ARM'/f'{NEW_NAME}.uvprojx'
    old_uv = source/'MDK-ARM'/f'{old_name}.uvprojx'
    new_text = uv.read_text(encoding='utf-8')
    new_xml,old_xml = ET.fromstring(new_text),ET.parse(old_uv).getroot()
    registered = [(n.text or '').replace('\\','/') for n in new_xml.findall('.//FilePath')]
    duplicates = [p for p,n in collections.Counter(p.lower() for p in registered).items() if n>1]
    missing = [p for p in registered if not (uv.parent/p).is_file()]
    check('keil_101_unique_existing_registered_files',len(registered)==101 and not duplicates and not missing,
          {'registered':len(registered),'c_or_s_sources':sum(p.lower().endswith(('.c','.s')) for p in registered),
           'duplicates':duplicates,'missing':missing})
    counts = {name:sum(p.lower().endswith('/'+name.lower()) for p in registered) for name in NEW_FILES}
    check('terminal_c_h_registered_exactly_once',all(n==1 for n in counts.values()),counts)
    include = new_xml.findtext('.//IncludePath','').split(';')
    check('keil_include_paths_unique_existing', len(set(p.lower() for p in include))==len(include)
          and all((uv.parent/p).is_dir() for p in include),include)
    adjusted = ET.fromstring(new_text.replace(NEW_NAME,old_name))
    for files in adjusted.findall('.//Files'):
        for item in list(files):
            if item.findtext('FileName') in NEW_FILES:
                files.remove(item)
    check('keil_configuration_preserved_except_identity_registration', normalized(adjusted)==normalized(old_xml))
    check('keil_device_memory_armcc5_preserved',new_xml.findtext('.//Device')=='STM32F103RC'
          and new_xml.findtext('.//Cpu')==old_xml.findtext('.//Cpu') and new_xml.findtext('.//uAC6')=='0'
          and new_xml.findtext('.//pCCUsed')==old_xml.findtext('.//pCCUsed'),
          {'device':new_xml.findtext('.//Device'),'cpu_memory':new_xml.findtext('.//Cpu'),
           'compiler':new_xml.findtext('.//pCCUsed')})
    old_ioc=(source/f'{old_name}.ioc').read_text(encoding='utf-8')
    new_ioc=(ROOT/f'{NEW_NAME}.ioc').read_text(encoding='utf-8')
    check('cubemx_pins_clock_preserved_except_identity',new_ioc.replace(NEW_NAME,old_name)==old_ioc)
    debug=[]
    for row in copied:
        if row['source'].endswith(('.uvoptx','.dbgconf')):
            debug.append({'path':row['destination'],'identical_except_identity':
                          (ROOT/row['destination']).read_text(encoding='utf-8').replace(NEW_NAME,old_name)==
                          (source/row['source']).read_text(encoding='utf-8')})
    check('stlink_flash_debug_configuration_preserved',bool(debug) and all(r['identical_except_identity'] for r in debug),debug)

    algorithm=ROOT/'Algorithm/ThiefStrategy'
    top=(algorithm/'ThiefMapTopology.h').read_text()
    meta=(algorithm/'ThiefRefugeMetadata.h').read_text()
    catalog=json.loads((ROOT/'Tests/hide_catalog.json').read_text())
    classes=table(meta,'thief_refuge_class')
    external=table(top,'thief_external_branch',4)
    rows={r['node']:r for r in catalog}
    node26=rows[26]
    check('N026_NS_conditional_with_east_external_branch_zero',classes[25]==2 and node26['conditional']
          and not node26['normal_refuge'] and node26['requires_terminal_validation']
          and node26['parking_axis_mask']==5 and external[25][1]==0
          and node26['fast_main_exit_count']==2 and all(k in node26['exits'] for k in ('0','2')),
          {'node26':node26,'external_branch':external[25]})
    check('primary38_emergency62_roles',classes[37]==1 and classes[61]==0,
          {'N038':classes[37],'N062':classes[61],'conditional_nodes':[n+1 for n,c in enumerate(classes) if c==2]})
    topology=module(ROOT/'Tests/generate_topology.py','audit16l_topology')
    nodes,graph=topology.load_map()
    excluded=set(node26['excluded_single_entry_nodes'])
    bad_external=[(n+1,h,target) for n,row in enumerate(external) for h,target in enumerate(row)
                  if target and (target in excluded or nodes[target][3]<3)]
    check('external_branches_are_useful_real_main_graph_nodes',not bad_external,bad_external)
    rollout_catalog=json.loads((ROOT/'Tests/rollout_catalog.json').read_text())
    original=json.loads((REPO/'STM32/Keil_Project/Thief16B_FourLaser_MapStrategy_Test/Tests/rollout_catalog.json').read_text())
    filtered=[r for r in rollout_catalog['routes'] if not r.get('return_cycle') and r['path'][0]!=26 and r['path'][-1]!=26]
    check('legacy579_filtered_content_and_order',len(filtered)==579 and filtered==original['routes'],
          {'filtered_routes':len(filtered),'all_routes':len(rollout_catalog['routes'])})
    cycles=[r for r in rollout_catalog['routes'] if r.get('return_cycle')]
    cycle_errors=[]
    for index,r in enumerate(cycles):
        path=r['path'];origin=path[0];distance=0;previous=None
        if origin!=path[-1] or classes[origin-1]!=1 or origin==26 or not 5<=len(path)<=28 or len(set(path[:-1]))!=len(path)-1:
            cycle_errors.append((index,'shape/origin'))
        if len({n for n in path[1:-1] if nodes[n][3]>=3})<2 or r['time_ms']>18000:
            cycle_errors.append((index,'branch/time'))
        for a,b in zip(path,path[1:]):
            edges=[e for e in graph[a] if e[0]==b]
            if len(edges)!=1:
                cycle_errors.append((index,'road'));break
            _,mm,h=edges[0];distance+=mm
            if previous is not None and ((h-previous)%4==2 or (nodes[a][2]==0 and h!=previous)):
                cycle_errors.append((index,'turn'))
            previous=h
        if distance!=r['length_mm'] or distance<600 or (previous+2*r['mode'])%4!=r['body'] or not rows[origin]['parking_axis_mask']&(1<<r['body']):
            cycle_errors.append((index,'length/pose'))
    check('controlled107_cycles_real_roads_legal_pose_no26',len(cycles)==107 and not cycle_errors,
          {'cycles':len(cycles),'errors':cycle_errors})
    seen_cycles=set();order_errors=[]
    for r in rollout_catalog['routes']:
        key=(r['path'][0],r['first'],r['mode'])
        if r.get('return_cycle'):seen_cycles.add(key)
        elif key in seen_cycles:order_errors.append(key)
    check('old_transfers_before_cycles_per_departure',not order_errors,order_errors)
    check('enumeration_not_truncated',not any(r['enumeration_capped'] or r['return_enumeration_capped'] for r in rollout_catalog['enumeration']))

    policy=(algorithm/'ThiefRefugePolicy.c').read_text()
    rollout=(algorithm/'ThiefRollout.c').read_text()
    risk=(algorithm/'ThiefRiskSearch.c').read_text()
    terminal=(algorithm/'ThiefTerminalForecast.c').read_text()
    header=(algorithm/'ThiefTerminalForecast.h').read_text()
    check('class2_nonordinal_preference', 'return role==1U ? 2U : role==2U ? 1U : 0U;' in policy
          and 'a_class=ThiefRefugePolicy_Preference(a_class);' in policy)
    check('N026_excluded_legacy_and_graph_fallback','if(role==2U) continue;' in rollout
          and 'if(role==2U && !THIEF_TERMINAL_ESCAPE_ENABLED) continue;' in rollout
          and 'if(ThiefRefugePolicy_Role(node)==2U) continue;' in risk)
    check('conditional_admission_requires_all_model_clear_terminal','terminal_result.qualified=(uint8_t)(terminal_result.clear==THIEF_ROLLOUT_MODELS);' in rollout
          and 'if(conditional_terminal.qualified &&' in rollout
          and 'if(!terminal_result.qualified) terminal_result.reserve=0L;' in rollout
          and 'if(terminal_score.clear && terminal_score.reserve_ms>(int32_t)THIEF_TIMING_GUARD_MS)' in rollout)
    check('same_actor_box_and_callback_ABI', 'typedef ThiefTerminalActor_t Actor;' in rollout
          and 'typedef ThiefTerminalBox_t Box;' in rollout
          and 'static const ThiefTerminalOps_t terminal_ops={NULL,T_Pose,T_Visible,T_View,T_Contact,' in rollout
          and 'T_Command,T_Advance,T_Guide,T_Next,T_Length,T_Branch,T_Reach,T_Kind};' in rollout,
          {'actor_fields':'7 uint16 + 7 uint8, ARM ABI size 22; Box int32 x/y + int16 hx/hy',
           'callbacks':'shared pose/visibility/contact/committed command/advance/reach; no casts'})
    table_owners=[str(p.relative_to(ROOT)).replace('\\','/') for p in ROOT.rglob('*.c')
                  if not str(p.relative_to(ROOT)).startswith('Tests') and '#include "ThiefRolloutData.h"' in p.read_text(encoding='utf-8')]
    check('huge_RO_tables_only_owned_by_rollout_c',table_owners==['Algorithm/ThiefStrategy/ThiefRollout.c']
          and 'ThiefRolloutData.h' not in terminal+header,table_owners)
    check('terminal_static_guarded_sequential_workspace','static struct {' in terminal and 'static uint8_t active;' in terminal
          and 'if(active) return s;' in terminal and 'active=0U;return s;' in terminal
          and 'Non-reentrant' in header and 'never an ISR' in header
          and 'Actor thief,pursuer' in terminal and 'uint8_t seen[88],path[13];' in terminal
          and 'sizeof(workspace)+sizeof(active)' in terminal,
          {'state':'Bounded actor/box/path scratch lives in guarded module storage.',
           'proof':'source/ABI audit; actual byte count from compiled accessor in stack profile'})
    check('rollout_shared_scratch_guarded_before_select', 'static uint8_t select_active;' in rollout
          and re.search(r'if\s*\(select_active\s*\|\|',rollout) and 'select_active=0U;' in rollout,
          'Additional bounded static scratch replaces nested caller frames; sequential stopped main-loop only.')
    check('terminal_pure_algorithm_no_HAL_heap_IO',not re.search(r'\b(?:HAL_\w+|malloc|calloc|realloc|free|printf|UartDebug_\w+)\s*\(',terminal)
          and not re.search(r'#\s*include.*(?:hal|main|Drivers)',terminal,re.I))
    callers=[str(p.relative_to(ROOT)).replace('\\','/') for p in ROOT.rglob('*.c')
             if not str(p.relative_to(ROOT)).startswith('Tests') and 'ThiefTerminalForecast_Run(' in p.read_text(encoding='utf-8')]
    check('terminal_production_callers_rollout_only',set(callers)=={'Algorithm/ThiefStrategy/ThiefRollout.c','Algorithm/ThiefStrategy/ThiefTerminalForecast.c'},callers)
    app=(ROOT/'User/App_ThiefStrategy.c').read_text(encoding='utf-8')
    old_app=(source/'User/App_ThiefStrategy.c').read_text(encoding='utf-8')
    def business(text):
        text=re.sub(r'/\*[\s\S]*?\*/|//[^\n]*','',text)
        text=re.sub(r'static (RouteGraySample_t sample|RouteLeadingCache_t leading_cache);',r'\1;',text)
        return re.sub(r'\s+','',re.sub(r'UartDebug_(?:Printf|SendString)\([\s\S]*?\);','',text))
    check('application_logic_preserved_except_telemetry_and_gray_scratch_storage',business(app)==business(old_app))
    check('gray_scratch_main_loop_only_and_cache_invalidated_each_task',
          'static RouteGraySample_t sample;' in app and 'static RouteLeadingCache_t leading_cache;' in app
          and 'leading_cache.valid = 0U;' in app[app.index('void App_ThiefStrategy_Task'):],
          'Only storage duration changes; existing per-call cache invalidation and sampling logic retained.')
    check('enumeration_returns_before_terminal_callback_phase',
          'ok=Enumerate(node,body,mask,previous,now);' in rollout
          and 'if(ok) ok=Finalize(body,path,count,mode,d);' in rollout)
    nav_paths=('Algorithm/MapRoute/MapRouteNavigator.c','Algorithm/MapLocalization/ThiefLocalization.c')
    check('navigator_localization_stop_guards_unchanged',all((ROOT/p).read_bytes()==(source/p).read_bytes() for p in nav_paths),list(nav_paths))
    check('planner_called_only_stopped_app_no_ISR', 'ThiefMapPlanner_PlanEscape' in app
          and re.search(r'if\s*\(\(route_active\s*==\s*0U\)\s*&&\s*\(thief_strategy_state\s*==\s*THIEF_STRATEGY_HIDING\)',app)
          and all('ThiefMapPlanner_PlanEscape' not in p.read_text(encoding='utf-8')
                    and 'ThiefTerminalForecast_Run' not in p.read_text(encoding='utf-8') for p in (ROOT/'Core').rglob('*.c')),
          'Source call graph review; application business logic preserved; no IRQ-timing proof')

    metadata_generator=module(ROOT/'Tests/generate_refuge_metadata.py','audit16l_refuge')
    reproduced=metadata_generator.generate(ROOT)
    stored=json.loads((ROOT/'Tests/refuge_metadata.json').read_text(encoding='utf-8'))
    check('actual_refuge_metadata_generator_reproduces',reproduced==stored,
          {'inputs':reproduced['source_inputs']})
    with tempfile.TemporaryDirectory(prefix='robot16l-integration-') as directory:
        temp=Path(directory)
        inputs=('Tests/generate_topology.py','Tests/generate_refuge_header.py','Tests/generate_rollout_data.py',
                'Tests/refuge_metadata.json','Tests/hide_catalog.json','Algorithm/MapRoute/MapRoutePlan.c',
                'Algorithm/ThiefStrategy/ThiefStrategyConfig.h','Algorithm/ThiefStrategy/ThiefRefugeMetadata.h')
        for relative in inputs:
            dest=temp/relative;dest.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(ROOT/relative,dest)
        for generator,outputs in (
                ('generate_topology.py',('Algorithm/ThiefStrategy/ThiefMapTopology.h','Tests/hide_catalog.json')),
                ('generate_refuge_header.py',('Algorithm/ThiefStrategy/ThiefRefugeMetadata.h',)),
                ('generate_rollout_data.py',('Algorithm/ThiefStrategy/ThiefRolloutData.h','Tests/rollout_catalog.json'))):
            run=subprocess.run([sys.executable,str(temp/'Tests'/generator)],check=True,
                               stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True,encoding='utf-8')
            output_hashes={p:{'identical':(temp/p).read_bytes()==(ROOT/p).read_bytes(),'sha256':sha(ROOT/p)} for p in outputs}
            check('actual_'+generator.removesuffix('.py')+'_reproduces',all(x['identical'] for x in output_hashes.values()),
                  {'files':output_hashes,'generator_output':run.stdout.strip()})
    result['source_sha256']={str(p.relative_to(REPO)).replace('\\','/'):sha(p)
          for directory in ('Algorithm','Core','Hardware','Control','Drivers','System','User')
          for p in (ROOT/directory).rglob('*') if p.is_file()}
    result['source_sha256'].update({str(p.relative_to(REPO)).replace('\\','/'):sha(p)
          for p in (uv,ROOT/f'{NEW_NAME}.ioc',ROOT/'copy_reference.json')})
    profile_path=REPO/'Tools/TreasurePursuitSimulator/variants/16l/stack_profile.json'
    if profile_path.exists():
        profile=json.loads(profile_path.read_text())
        matching=all((REPO/p).is_file() and sha(REPO/p)==digest for p,digest in profile['source_sha256'].items())
        check('offline_stack_profile_matches_current_sources',matching,
              {'path':str(profile_path.relative_to(REPO)).replace('\\','/'),
               'peak_sampled_bytes':profile['peak_sampled_bytes'],
               'startup_stack_bytes':profile['startup_stack_bytes'],
               'requires_review':profile['requires_review'],
               'scope':'No application caller or IRQ stack; no board wall-clock proof'})
        check('sampled_stack_below_900B_review_threshold',not profile['requires_review']
              and profile['peak_sampled_bytes']<900 and profile['peak_sampled_bytes']<profile['startup_stack_bytes'],
              {'peak':profile['peak_sampled_bytes'],'startup':profile['startup_stack_bytes'],
               'interpretation':'Specified bridge paths only; application callers and IRQ allowance still require separate review'})
        terminal_cases=[c['name'] for c in profile['cases'] if c.get('observation',{}).get('plan')
                        and c['observation']['plan']['terminal_candidates']>0]
        check('stack_profile_exercises_actual_terminal_callback_paths',bool(terminal_cases)
              and 147<=profile['terminal_workspace_bytes_compiled_accessor']<=1024,
              {'cases':terminal_cases,'compiled_terminal_work_bytes':profile['terminal_workspace_bytes_compiled_accessor']})


if __name__=='__main__':
    def protected_hashes():
        result={}
        for name in ('Thief16B_FourLaser_MapStrategy_Test','Thief16J_RefugeViability_FourLaser_Test',
                     'Thief16K_RefugeAccess_FourLaser_Test'):
            directory=REPO/'STM32/Keil_Project'/name
            for path in directory.rglob('*'):
                if path.is_file() and 'build' not in path.parts and '__pycache__' not in path.parts:
                    if path.suffix.lower() in ('.c','.h','.s','.py','.json','.ioc','.uvprojx','.uvoptx','.dbgconf','.md') or path.name=='.gitignore':
                        result[str(path.relative_to(REPO)).replace('\\','/')]=sha(path)
        return result
    protected_before=protected_hashes()
    try:
        audit()
    except Exception as error:
        result['failures'].append('audit_exception')
        result['exception']=type(error).__name__+': '+str(error)
    protected_after=protected_hashes()
    check('audit_does_not_change_16b_16j_16k_files',protected_before==protected_after,
          {'files_hashed':len(protected_before),'changed':[p for p,d in protected_before.items() if protected_after.get(p)!=d]})
    result['passed']=not result['failures']
    REPORT.write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({'passed':result['passed'],'checks':len(result['checks']),
                      'failures':result['failures'],'exception':result.get('exception')},indent=2))
    sys.exit(0 if result['passed'] else 1)
