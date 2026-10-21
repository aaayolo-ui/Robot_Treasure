"""Read-only source/configuration audit; isolated offline table regeneration.

Does not compile, flash, drive motors, or write into shared Tests/build.
The only persistent output is VALIDATION_16K_INTEGRATION.json.
"""
from pathlib import Path
import collections
import difflib
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
NEW_NAME = 'Thief16K_RefugeAccess_FourLaser_Test'
OLD_NAME = 'Thief16B_FourLaser_MapStrategy_Test'
LOW_LEVEL = ('Core/', 'Hardware/', 'Control/', 'Drivers/', 'System/')
POLICY_FILES = {'ThiefRefugePolicy.c', 'ThiefRefugePolicy.h', 'ThiefRefugeMetadata.h'}
result = {'checks': {}, 'failures': [], 'hardware_run': False,
          'compile_run': False, 'scope': 'source/configuration and offline generated tables'}


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def checked(name, condition, detail=None):
    result['checks'][name] = {'passed': bool(condition), 'detail': detail}
    if not condition:
        result['failures'].append(name)


def command(args):
    return subprocess.run(args, cwd=str(REPO), check=True, stdout=subprocess.PIPE,
                          stderr=subprocess.PIPE, text=True, encoding='utf-8').stdout


def tracked_and_new(paths):
    names = command(['git', '-c', 'core.quotepath=false', 'ls-files', '--cached',
                     '--others', '--exclude-standard', '--'] + paths).splitlines()
    return {name: sha(REPO / name) for name in names if (REPO / name).is_file()}


def normalized_xml(element):
    return (element.tag, tuple(sorted(element.attrib.items())),
            (element.text or '').strip(), tuple(normalized_xml(c) for c in element))


def delivery_file_scope():
    """Prepare inspectable new-file hashes and a source-copy diff summary."""
    reference = json.loads((ROOT / 'copy_reference.json').read_text(encoding='utf-8'))
    source = Path(reference['source_project'])
    copied = {item['destination']: item for item in reference['copied_files']}
    paths = [str(ROOT.relative_to(REPO)).replace('\\', '/'),
             'Tools/TreasurePursuitSimulator/variants/16k']
    files = tracked_and_new(paths)
    self_report = str((ROOT / 'VALIDATION_16K_INTEGRATION.json').relative_to(REPO)).replace('\\', '/')
    files.pop(self_report, None)  # No recursively changing report hash.
    changes = []
    added = removed = 0
    firmware_prefix = str(ROOT.relative_to(REPO)).replace('\\', '/') + '/'
    for path, digest in files.items():
        if not path.startswith(firmware_prefix):
            continue
        relative = path[len(firmware_prefix):]
        item = copied.get(relative)
        if item is None:
            changes.append({'path': relative, 'kind': 'added'})
            continue
        if digest == item['sha256']:
            continue
        entry = {'path': relative, 'kind': 'changed_from_source_copy',
                 'source': item['source'], 'source_sha256': item['sha256'],
                 'current_sha256': digest}
        try:
            before = (source / item['source']).read_text(encoding='utf-8').splitlines()
            after = (ROOT / relative).read_text(encoding='utf-8').splitlines()
            plus = minus = 0
            for opcode, a0, a1, b0, b1 in difflib.SequenceMatcher(None, before, after,
                                                                autojunk=False).get_opcodes():
                if opcode in ('replace', 'delete'):
                    minus += a1 - a0
                if opcode in ('replace', 'insert'):
                    plus += b1 - b0
            entry.update({'added_lines': plus, 'removed_lines': minus,
                          'line_endings_ignored_for_diff_count': True})
            added += plus
            removed += minus
        except UnicodeDecodeError:
            entry['binary'] = True
        changes.append(entry)
    result['delivery_file_scope'] = {
        'files_without_ignored_build_products': [{'path': path, 'sha256': digest}
                                                for path, digest in sorted(files.items())],
        'file_count_excluding_this_validation_report': len(files),
        'firmware_changes_from_copy': changes,
        'modified_source_file_added_lines': added,
        'modified_source_file_removed_lines': removed,
        'note': 'Snapshot may be regenerated after configuration lock. New validation report itself is listed separately, without a recursive hash.'}


def audit():
    reference = json.loads((ROOT / 'copy_reference.json').read_text(encoding='utf-8'))
    source = Path(reference['source_project'])
    source_changed = []
    low_changed = []
    low_count = 0
    for item in reference['copied_files']:
        relative = item['source']
        if sha(source / relative) != item['sha256']:
            source_changed.append(relative)
        if relative.startswith(LOW_LEVEL):
            low_count += 1
            if sha(ROOT / item['destination']) != item['sha256']:
                low_changed.append(item['destination'])
    checked('source_16b_copied_bytes_preserved', not source_changed,
            {'files': len(reference['copied_files']), 'changed': source_changed})
    checked('low_level_bytes_identical', low_count == 117 and not low_changed,
            {'files': low_count, 'changed': low_changed,
             'covers': list(LOW_LEVEL), 'clock_gpio_pins_controls': 'byte identical'})

    old_project_path = source / 'MDK-ARM' / (OLD_NAME + '.uvprojx')
    new_project_path = ROOT / 'MDK-ARM' / (NEW_NAME + '.uvprojx')
    old_project = ET.fromstring(old_project_path.read_text(encoding='utf-8'))
    new_text = new_project_path.read_text(encoding='utf-8')
    new_project = ET.fromstring(new_text)
    registered = [(n.text or '').replace('\\', '/').lower()
                  for n in new_project.findall('.//FilePath')]
    duplicates = [name for name, count in collections.Counter(registered).items() if count > 1]
    missing = [name for name in registered if not (new_project_path.parent / name).is_file()]
    checked('keil_unique_and_existing_sources', not duplicates and not missing,
            {'file_paths': len(registered), 'duplicates': duplicates, 'missing': missing})
    policy_counts = {name: sum(p.endswith('/' + name.lower()) for p in registered)
                     for name in POLICY_FILES}
    checked('keil_policy_registration_once', all(n == 1 for n in policy_counts.values()), policy_counts)
    checked('keil_target_identity',
            new_project.findtext('.//TargetName') == NEW_NAME and
            new_project.findtext('.//OutputName') == NEW_NAME and
            NEW_NAME in new_project.findtext('.//OutputDirectory', ''), NEW_NAME)
    adjusted_project = ET.fromstring(new_text.replace(NEW_NAME, OLD_NAME))
    for files in adjusted_project.findall('.//Files'):
        for file in list(files):
            if file.findtext('FileName') in POLICY_FILES:
                files.remove(file)
    checked('keil_configuration_preserved_except_registration_identity',
            normalized_xml(adjusted_project) == normalized_xml(old_project))
    checked('keil_device_memory_armcc5',
            new_project.findtext('.//Device') == 'STM32F103RC' and
            new_project.findtext('.//Cpu') == old_project.findtext('.//Cpu') and
            new_project.findtext('.//uAC6') == '0' and
            new_project.findtext('.//pCCUsed') == old_project.findtext('.//pCCUsed'),
            {'device': new_project.findtext('.//Device'),
             'cpu_memory': new_project.findtext('.//Cpu'),
             'compiler': new_project.findtext('.//pCCUsed')})
    old_ioc = (source / (OLD_NAME + '.ioc')).read_text(encoding='utf-8')
    new_ioc = (ROOT / (NEW_NAME + '.ioc')).read_text(encoding='utf-8')
    checked('cubemx_only_project_identity_changed', new_ioc.replace(NEW_NAME, OLD_NAME) == old_ioc)
    preserved_debug = []
    for item in reference['copied_files']:
        if item['source'].endswith(('.uvoptx', '.dbgconf')):
            before = (source / item['source']).read_text(encoding='utf-8')
            after = (ROOT / item['destination']).read_text(encoding='utf-8')
            preserved_debug.append((item['destination'], after.replace(NEW_NAME, OLD_NAME) == before))
    checked('debug_stlink_configuration_preserved',
            bool(preserved_debug) and all(ok for _, ok in preserved_debug), preserved_debug)

    sys.path.insert(0, str(ROOT / 'Tests'))
    from generate_topology import load_map
    nodes, graph = load_map()
    baseline_catalog = json.loads((source / 'Tests/rollout_catalog.json').read_text())
    catalog = json.loads((ROOT / 'Tests/rollout_catalog.json').read_text())
    base = [r for r in catalog['routes'] if not r.get('return_cycle')]
    cycles = [r for r in catalog['routes'] if r.get('return_cycle')]
    checked('legacy_579_routes_content_and_order', len(base) == 579 and base == baseline_catalog['routes'])
    pose_catalog = json.loads((ROOT / 'Tests/hide_catalog.json').read_text())
    poses = {r['node']: [h for h in range(4) if r['parking_axis_mask'] & (1 << h)] for r in pose_catalog}
    metadata = (ROOT / 'Algorithm/ThiefStrategy/ThiefRefugeMetadata.h').read_text()
    tier_block = metadata.split('thief_refuge_class[88] = {', 1)[1].split('};', 1)[0]
    tiers = list(map(int, re.findall(r'(\d+)U', tier_block)))
    cycle_errors = []
    cycle_counts = collections.Counter()
    max_nodes = 0
    for index, r in enumerate(cycles):
        path = r['path']
        origin = path[0]
        max_nodes = max(max_nodes, len(path))
        cycle_counts[origin] += 1
        if (path[-1] != origin or tiers[origin - 1] != 1 or len(path) < 5 or
                len(path) > 28 or len(set(path[:-1])) != len(path) - 1 or
                r['body'] not in poses[origin] or r['time_ms'] > 18000):
            cycle_errors.append([index, 'shape/origin/parking/time'])
        branch_nodes = sorted({n for n in path[1:-1] if nodes[n][3] >= 3})
        if len(branch_nodes) < 2 or branch_nodes != r['other_branch_nodes']:
            cycle_errors.append([index, 'branch diversity'])
        mm = 0
        previous_heading = None
        for a, b in zip(path, path[1:]):
            matching = [e for e in graph[a] if e[0] == b]
            if len(matching) != 1:
                cycle_errors.append([index, 'missing real road'])
                break
            _, distance, heading = matching[0]
            if previous_heading is not None:
                if (heading - previous_heading) % 4 == 2:
                    cycle_errors.append([index, 'immediate reversal'])
                if nodes[a][2] == 0 and heading != previous_heading:
                    cycle_errors.append([index, 'turn at non-gray marker'])
            previous_heading = heading
            mm += distance
        if mm < 600 or mm != r['length_mm']:
            cycle_errors.append([index, 'road length'])
        if previous_heading is not None and (previous_heading + 2 * r['mode']) % 4 != r['body']:
            cycle_errors.append([index, 'closing body direction'])
    checked('controlled_cycles_real_roads_and_legal_closure',
            len(cycles) == 107 and not cycle_errors,
            {'cycles': len(cycles), 'by_refuge': dict(cycle_counts),
             'max_nodes': max_nodes, 'errors': cycle_errors})
    seen_cycle = {}
    prefix_errors = []
    for r in catalog['routes']:
        key = (r['path'][0], r['first'], r['mode'])
        if r.get('return_cycle'):
            seen_cycle[key] = True
        elif seen_cycle.get(key):
            prefix_errors.append(list(key))
    checked('legacy_prefix_before_return_candidates', not prefix_errors, prefix_errors)
    config = (ROOT / 'Algorithm/ThiefStrategy/ThiefStrategyConfig.h').read_text()
    rollout = (ROOT / 'Algorithm/ThiefStrategy/ThiefRollout.c').read_text()
    cycle_default_match = re.search(r'#define\s+THIEF_REFUGE_RETURN_CYCLES\s+([01])\b', config)
    checked('cycle_switch_and_legacy_filter',
            cycle_default_match is not None and
            'if(!THIEF_REFUGE_RETURN_CYCLES && rollout_nodes[r->offset]==goal) continue;' in rollout,
            {'default': int(cycle_default_match.group(1)) if cycle_default_match else None,
             'interpretation': '0 preserves legacy route candidates; 1 admits controlled return loops'})
    checked('enumeration_not_truncated',
            not any(a['enumeration_capped'] or a['return_enumeration_capped']
                    for a in catalog['enumeration']),
            {'routes': len(catalog['routes']), 'max_candidates': max(a['candidates'] for a in catalog['enumeration']),
             'table_bytes': catalog['table_bytes']})

    # Execute the actual offline metadata generator instead of mirroring its
    # map-search formulas in the verifier. Importing it does not write files.
    specification = importlib.util.spec_from_file_location(
        'refuge_metadata_reproduction', str(ROOT / 'Tests/generate_refuge_metadata.py'))
    refuge_generator = importlib.util.module_from_spec(specification)
    specification.loader.exec_module(refuge_generator)
    reproduced_refuge = refuge_generator.generate(ROOT)
    stored_refuge = json.loads((ROOT / 'Tests/refuge_metadata.json').read_text(encoding='utf-8'))
    checked('refuge_metadata_reproducible_from_current_inputs', reproduced_refuge == stored_refuge,
            {'map_metadata_generator': 'Tests/generate_refuge_metadata.py',
             'source_input_hashes': reproduced_refuge['source_inputs']})

    # Reproduce in an automatically cleaned, separate temporary project.
    # No writes to production tables or the shared Tests/build directory.
    with tempfile.TemporaryDirectory(prefix='robot16k-verify-') as directory:
        temp = Path(directory)
        for relative in ('Tests/generate_rollout_data.py', 'Tests/generate_topology.py',
                         'Tests/generate_refuge_header.py', 'Tests/refuge_metadata.json',
                         'Tests/hide_catalog.json', 'Algorithm/MapRoute/MapRoutePlan.c',
                         'Algorithm/ThiefStrategy/ThiefStrategyConfig.h',
                         'Algorithm/ThiefStrategy/ThiefRefugeMetadata.h'):
            destination = temp / relative
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(ROOT / relative, destination)
        subprocess.run([sys.executable, str(temp / 'Tests/generate_refuge_header.py')],
                       check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                       text=True, encoding='utf-8')
        checked('refuge_header_reproducible',
                (temp / 'Algorithm/ThiefStrategy/ThiefRefugeMetadata.h').read_bytes() ==
                (ROOT / 'Algorithm/ThiefStrategy/ThiefRefugeMetadata.h').read_bytes(),
                {'sha256': sha(ROOT / 'Algorithm/ThiefStrategy/ThiefRefugeMetadata.h'),
                 'generator': 'Tests/generate_refuge_header.py'})
        generated = subprocess.run([sys.executable, str(temp / 'Tests/generate_rollout_data.py')],
                                   check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                   text=True, encoding='utf-8')
        reproduction = {}
        for relative in ('Algorithm/ThiefStrategy/ThiefRolloutData.h', 'Tests/rollout_catalog.json'):
            reproduction[relative] = {'identical': (temp / relative).read_bytes() == (ROOT / relative).read_bytes(),
                                      'sha256': sha(ROOT / relative)}
        checked('generated_tables_reproducible', all(v['identical'] for v in reproduction.values()),
                {'files': reproduction, 'generator_output': generated.stdout.strip()})

    # Full return routes use step indices and stop at the final step; current
    # node equality is not a completion shortcut. These static checks support
    # the separately executed ARM algorithm/localization tests, not hardware.
    planner = (ROOT / 'Algorithm/ThiefStrategy/ThiefMapPlanner.c').read_text()
    navigator = (ROOT / 'Algorithm/MapRoute/MapRouteNavigator.c').read_text()
    application = (ROOT / 'User/App_ThiefStrategy.c').read_text()
    source_application = (source / 'User/App_ThiefStrategy.c').read_text()
    # Application changes are diagnostic UART calls only. Comparing business
    # tokens preserves KEY stop, wheel braking, localization guards and timing.
    def without_telemetry(text):
        text = re.sub(r'UartDebug_(?:Printf|SendString)\([\s\S]*?\);', '', text)
        return re.sub(r'\s+', '', text)
    checked('application_business_logic_preserved_except_uart_telemetry',
            without_telemetry(application) == without_telemetry(source_application))
    unchanged_navigation = ['Algorithm/MapRoute/MapRouteNavigator.c',
                            'Algorithm/MapLocalization/ThiefLocalization.c']
    checked('navigator_and_gyro_gray_guards_unchanged',
            all((ROOT / p).read_bytes() == (source / p).read_bytes()
                for p in unchanged_navigation), unchanged_navigation)
    max_steps = int(re.search(r'#define\s+THIEF_ROUTE_MAX_STEPS\s+(\d+)',
                    (ROOT / 'Algorithm/ThiefStrategy/ThiefMapPlanner.h').read_text()).group(1))
    checked('return_cycle_controller_uses_step_completion',
            max_nodes - 1 <= max_steps and
            'while (i + 1U < node_count)' in planner and
            'plan->steps[step_count].action = MAP_ACTION_STOP;' in planner and
            'nav->step++' in navigator and
            'if (action == MAP_ACTION_STOP)' in application and
            'thief_current_node = finished->to_node;' in application and
            'THIEF_STRATEGY_SETTLING' in application,
            {'max_catalog_legs': max_nodes - 1, 'runtime_step_capacity': max_steps,
             'static_review': 'No same-start/end early termination. Final STOP enters settling and re-arms hide sensing.',
             'hardware_verification': 'not performed'})


protected_paths = ['STM32/Keil_Project/' + OLD_NAME,
                   'STM32/Keil_Project/Thief16J_RefugeViability_FourLaser_Test',
                   'Tools/TreasurePursuitSimulator/variants/16j']
protected_before = tracked_and_new(protected_paths)
try:
    audit()
except Exception as error:
    result['failures'].append('audit_exception')
    result['exception'] = type(error).__name__ + ': ' + str(error)
protected_after = tracked_and_new(protected_paths)
checked('audit_does_not_change_16b_or_16j_bytes', protected_before == protected_after,
        {'files': len(protected_before)})
j_paths = protected_paths[1:]
j_diff = command(['git', 'diff', '--name-only', 'HEAD', '--'] + j_paths).splitlines()
checked('published_16j_tracked_content_unchanged', not j_diff,
        {'reference': 'Git HEAD content comparison; byte hashes before/after this audit are separate.',
         'changed': j_diff})
result['passed'] = not result['failures']
delivery_file_scope()
(ROOT / 'VALIDATION_16K_INTEGRATION.json').write_text(json.dumps(result, indent=2) + '\n', encoding='utf-8')
print(json.dumps({'passed': result['passed'], 'checks': len(result['checks']), 'failures': result['failures']}, indent=2))
sys.exit(0 if result['passed'] else 1)
