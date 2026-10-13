"""Create a separate 16G experiment; never overwrite a previous experiment."""
from pathlib import Path
import hashlib,json,shutil,subprocess
HERE=Path(__file__).resolve().parent
REPO=HERE.parents[3]
OLD='Thief16E_VisibilityEscape_FourLaser_Test'
NEW='Thief16G_MultiExitHide_FourLaser_Test'
source=REPO/'STM32/Keil_Project'/OLD
target=source.with_name(NEW)
if target.exists():raise FileExistsError(target)
history={'BASELINE_SHA256.json','SOURCE_CHANGES.json','SOURCE_DIFF.patch','DIFF_SUMMARY.txt',
    'FILE_MANIFEST.md','README.md','VALIDATION_RESULTS.json','VERIFICATION.md'}
files=[p for p in source.rglob('*') if p.is_file()]
names=[p.relative_to(REPO).as_posix() for p in files]
ignored=subprocess.run(['git','check-ignore','--stdin'],cwd=str(REPO),
    input=('\n'.join(names)+'\n').encode('utf-8'),capture_output=True)
assert ignored.returncode in (0,1),ignored.stderr
skip=set(ignored.stdout.decode('utf-8').splitlines())
baseline={};copies={}
for p,name in zip(files,names):
    if name in skip:continue
    baseline[name]=hashlib.sha256(p.read_bytes()).hexdigest()
    if p.parent==source and p.name in history:continue
    relative=p.relative_to(source).as_posix().replace(OLD,NEW)
    out=target/relative;out.parent.mkdir(parents=True,exist_ok=True)
    data=p.read_bytes()
    if p.suffix.lower() in ('.ioc','.uvprojx','.dbgconf','.c','.md') or p.name=='.gitignore':
        data=data.replace(OLD.encode(),NEW.encode())
    out.write_bytes(data);copies[relative]=p.relative_to(source).as_posix()
for folder in (HERE.with_name('16e'),HERE.with_name('16f')):
    for p in folder.rglob('*'):
        if p.is_file() and not any(x in p.relative_to(folder).parts for x in ('build','__pycache__')):
            baseline[p.relative_to(REPO).as_posix()]=hashlib.sha256(p.read_bytes()).hexdigest()
(HERE/'BASELINE_SHA256.json').write_text(json.dumps(baseline,indent=2)+'\n',encoding='utf-8')
(HERE/'COPIED_PROJECT_FILES.json').write_text(json.dumps(copies,indent=2)+'\n',encoding='utf-8')
for name in ('arm_adapter.py','arm_bridge.c','map_annotations.js','test_ui.cjs','render_preview.py'):
    (HERE/name).write_bytes((HERE.with_name('16f')/name).read_bytes())
adapter=HERE/'arm_adapter.py'
s=adapter.read_text(encoding='utf-8').replace(OLD,NEW)
s=s.replace("sys.path.insert(0,str(FIRMWARE/'Tests/build/deps'))", 
    "sys.path.insert(0,str(REPO/'STM32/Keil_Project/Thief16E_VisibilityEscape_FourLaser_Test/Tests/build/deps'))")
adapter.write_text(s,encoding='utf-8')
print('Copied firmware source files',len(copies),'immutable baseline files',len(baseline))
