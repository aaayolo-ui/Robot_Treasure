"""Bind MONITOR6 evidence to the delivered sources and linked firmware."""
from pathlib import Path
import difflib
import hashlib
import json

root = Path(__file__).resolve().parents[1]
base = root / 'Tests/before_route_identity'
desktop = next(p / 'DesktopSerialMonitor' for p in root.parents if (p / 'DesktopSerialMonitor').is_dir())
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
read = lambda p: json.loads(p.read_text(encoding='utf-8'))
axf = root / 'MDK-ARM' / root.name / (root.name + '.axf')
firmware = sha(axf)
old = read(base / 'VALIDATION_MONITOR5.json')
assert sha(base / 'original.axf') == old['firmware_sha256']
before = read(base / 'regression_before.json')
after = read(root / 'Tests/results_route_identity.json')
assert before['axf_sha256'] == old['firmware_sha256']
assert len(before['failures']) == 8
assert before['test_sha256'] == after['test_sha256'] == sha(root / 'Tests/test_route_identity.py')
assert after['axf_sha256'] == firmware and not after['failures'] and after['checks'] == 92
assert len(after['cases']) == 8
production = ['User/App_ThiefStrategy.c', 'Algorithm/ThiefStrategy/ThiefMapPlanner.c',
              'Algorithm/ThiefStrategy/ThiefRiskSearch.c', 'Algorithm/MapLocalization/ThiefLocalization.c',
              'Algorithm/MapLocalization/ThiefLocalization.h']
adapted = ['Tests/test_treasure_fix.py', 'Tests/test_gray_priority.py',
           'Tests/test_documents123154.py', 'Tests/test_documents18.py']
for name, value in old['files'].items():
    if name.endswith(('.c', '.h', '.py')) and name not in production + adapted:
        assert sha(root / name) == value, name
supplemental = {}
for stem, count in [('exit_stall', 988), ('road_probe', 1798), ('left800', 442)]:
    report = read(root / 'Tests' / ('results_' + stem + '.json'))
    assert report['axf_sha256'] == firmware and report['checks'] == count
    assert not report.get('failures') and report.get('failure') is None
    assert report['test_sha256'] == sha(root / 'Tests' / ('test_' + stem + '.py'))
    supplemental[stem] = count
suite = read(root / 'Tests/results_monitor_suite.json')
assert len(suite) == 22 and len({r['name'] for r in suite}) == 22
for row in suite:
    assert row['exit_code'] == 0 and row['axf_sha256'] == firmware, row['name']
    assert row['log_sha256'] == sha(root / 'Tests' / ('monitor_' + row['name'] + '.log'))
pure = read(root / 'Tests/results_16m.json')
assert pure['test_failures'] == 0 and pure['test_done'] == 1 and pure['test_case'] == 378
for name, value in pure['source_sha256'].items():
    assert sha(root / name) == value, name
build = root / 'build_monitor6.log'
assert '0 Error(s), 0 Warning(s)' in build.read_text(encoding='utf-8', errors='replace')
assert 'UARTCH4_16U_MONITOR6' in (root / production[0]).read_text(encoding='utf-8')
changed = production + adapted + ['Tests/test_route_identity.py', 'README.md', 'AGENTS.md',
                                 'Docs/修改规范与回归清单.md']
patch = []
for name in changed:
    previous = (base / name).read_text(encoding='utf-8') if (base / name).exists() else ''
    patch.extend(difflib.unified_diff(previous.splitlines(keepends=True),
                 (root / name).read_text(encoding='utf-8').splitlines(keepends=True),
                 fromfile='MONITOR5/' + name, tofile='MONITOR6/' + name))
desktop_changed = ['assets/laser_live_server.py', 'assets/laser_live.html', 'README.md', 'test_identity_monitor.py']
for name in desktop_changed:
    previous_path = desktop / 'backups/before_route_identity' / name
    previous = previous_path.read_text(encoding='utf-8') if previous_path.exists() else ''
    patch.extend(difflib.unified_diff(previous.splitlines(keepends=True),
                 (desktop / name).read_text(encoding='utf-8').splitlines(keepends=True),
                 fromfile='MONITOR5/DesktopSerialMonitor/' + name, tofile='MONITOR6/DesktopSerialMonitor/' + name))
(root / 'ROUTE_IDENTITY_CHANGESET.patch').write_text(''.join(patch), encoding='utf-8')
files = changed + ['Tests/validate_monitor6.py', 'ROUTE_IDENTITY_CHANGESET.patch', 'build_monitor6.log',
                  'Tests/results_route_identity.json', 'Tests/results_exit_stall.json', 'Tests/results_road_probe.json',
                  'Tests/results_left800.json', 'Tests/results_monitor_suite.json', 'Tests/results_16m.json']
data = dict(firmware='UARTCH4_16U_MONITOR6', firmware_sha256=firmware,
            hex_sha256=sha(axf.with_suffix('.hex')), limits_raw_mm=dict(front=1000, left=800, right=800, rear=1000),
            new_checks=after['checks'], supplemental=supplemental, pure_checks=pure['test_checks'],
            pure_cases=pure['test_case'], suite=suite, before_failures=before['failures'],
            files={name: sha(root / name) for name in files},
            desktop_files={name: sha(desktop / name) for name in desktop_changed + ['小车串口监控一体助手.exe']},
            hardware_tested=False, flashed=False, serial_opened=False,
            limitations=['Bounded local candidates cannot guarantee globally unique localization.',
                         'The N011 replay selects N012-N013, with N063 as goal via N026; the goal is not fixed to N026.',
                         'Actual gray coverage, centering, traction and moving-obstacle performance require physical tests.',
                         'Continuous both-board loss still stops after 2000ms.'])
(root / 'VALIDATION_MONITOR6.json').write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding='utf-8')
print('MONITOR6 verified: 92 new checks, 22 ARM scripts, %s pure checks; hardware untested.' % pure['test_checks'])
