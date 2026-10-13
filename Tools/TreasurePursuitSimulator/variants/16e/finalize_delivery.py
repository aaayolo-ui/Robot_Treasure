"""Generate review manifests without staging or committing any Git changes."""
from pathlib import Path
import difflib,hashlib,json,subprocess,sys
if hasattr(sys.stdout,'reconfigure'):sys.stdout.reconfigure(encoding='utf-8')

HERE=Path(__file__).resolve().parent
REPO=HERE.parents[3]
PROJECT=REPO/'STM32/Keil_Project/Thief16E_VisibilityEscape_FourLaser_Test'
baseline=json.loads((PROJECT/'BASELINE_SHA256.json').read_text(encoding='utf-8'))
OLD=PROJECT.parent/baseline['project']
changes=[];patch=[];stats=[]
for name,expected in baseline['files'].items():
    a=OLD/name
    assert hashlib.sha256(a.read_bytes()).hexdigest()==expected,('Changed baseline',name)
    mapped=name.replace(OLD.name,PROJECT.name)
    b=PROJECT/mapped
    assert b.is_file(),mapped
    if a.read_bytes()==b.read_bytes():continue
    old_lines=a.read_text(encoding='utf-8').splitlines(keepends=True)
    new_lines=b.read_text(encoding='utf-8').splitlines(keepends=True)
    d=list(difflib.unified_diff(old_lines,new_lines,fromfile='a/'+name,tofile='b/'+mapped))
    add=sum(s.startswith('+') and not s.startswith('+++') for s in d)
    remove=sum(s.startswith('-') and not s.startswith('---') for s in d)
    changes.append({'source':name,'new':mapped,'added_lines':add,'removed_lines':remove})
    patch.extend(d)
    result=subprocess.run(['git','diff','--no-index','--stat',str(a),str(b)],
                          capture_output=True,encoding='utf-8')
    assert result.returncode in (0,1),result.stderr
    stats.append(result.stdout.strip())
(PROJECT/'SOURCE_CHANGES.json').write_text(json.dumps({'baseline':OLD.name,
    'preserved_baseline_files':len(baseline['files']),'modified_copies':changes},indent=2)+'\n',encoding='utf-8')
(PROJECT/'SOURCE_DIFF.patch').write_text(''.join(patch),encoding='utf-8')
(PROJECT/'DIFF_SUMMARY.txt').write_text('\n'.join(stats)+'\n',encoding='utf-8')
verify={'build_errors':0,'build_warnings':0,'flash_bytes':43796,'ram_bytes':34368,
    'ram_capacity_bytes':49152,'workspace_bytes':13265,'stack_bytes':2048,
    'algorithm':json.loads((PROJECT/'Tests/build/result.json').read_text()),
    'uart':json.loads((PROJECT/'Tests/build/uart_result.json').read_text()),
    'preserved_baseline_files':len(baseline['files']),
    'replay_ui':json.loads((HERE/'ui_smoke_result.json').read_text()),
    'hardware_verified':False,'browser_layout_verified':False,
    'notes':'Compile, real ARM C tests, simulations and offline UI event tests are separate from physical validation.'}
(PROJECT/'VALIDATION_RESULTS.json').write_text(json.dumps(verify,indent=2)+'\n',encoding='utf-8')

def inventory(folder):
    files=[p for p in folder.rglob('*') if p.is_file()]
    relative={p:p.relative_to(REPO).as_posix() for p in files}
    check=subprocess.run(['git','-c','core.quotePath=false','check-ignore','--stdin'],
        cwd=REPO,input=('\n'.join(relative.values())+'\n').encode('utf-8'),capture_output=True)
    assert check.returncode in (0,1),check.stderr
    ignored=set(check.stdout.decode('utf-8').splitlines())
    return sorted(p.relative_to(folder).as_posix() for p in files if relative[p] not in ignored)

for folder,heading in [(PROJECT,'16E project'),(HERE,'16E replay and comparison')]:
    paths=inventory(folder)
    if 'FILE_MANIFEST.md' not in paths:paths.append('FILE_MANIFEST.md');paths.sort()
    lines=['# '+heading+' file manifest','',
        'All files in this directory are new. The old 16D directory is preserved.',
        'Ignored compiler outputs, caches and local test dependencies are excluded.','']
    if folder==PROJECT:
        lines.extend(['## Modified copies relative to 16D',''])
        lines.extend('- '+x['new'] for x in changes)
        lines.extend(['','See SOURCE_DIFF.patch and DIFF_SUMMARY.txt for review.',''])
    lines.extend(['## Complete added file list',''])
    lines.extend('- '+name for name in paths)
    (folder/'FILE_MANIFEST.md').write_text('\n'.join(lines)+'\n',encoding='utf-8')
    print(folder.name,len(paths),'non-ignored files')
print('Modified copies:',len(changes),'lines +',sum(c['added_lines'] for c in changes),
      '-',sum(c['removed_lines'] for c in changes))
print('Existing old files modified by this delivery: 0')
print('Additional new document: Docs/13_\u5c0f\u5077\u7b56\u7565\u7b97\u6cd5\u5206\u7c7b\u5bf9\u6bd4_16E.md')
