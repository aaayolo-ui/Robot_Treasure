"""Create separate 16J algorithm experiment, keep the full prior workspace."""
from pathlib import Path
import hashlib,json,subprocess
HERE=Path(__file__).resolve().parent;REPO=HERE.parents[3]
OLD='Thief16I_RolloutMinimax_FourLaser_Test';NEW='Thief16J_RefugeViability_FourLaser_Test'
source=REPO/'STM32/Keil_Project'/OLD;target=source.with_name(NEW)
if target.exists():raise FileExistsError(target)
history={'BASELINE_SHA256.json','SOURCE_CHANGES.json','SOURCE_DIFF.patch','DIFF_SUMMARY.txt',
         'FILE_MANIFEST.md','README.md','VALIDATION_RESULTS.json','VERIFICATION.md'}
files=[p for p in REPO.rglob('*') if p.is_file() and '.git' not in p.relative_to(REPO).parts
       and not any(s in p.relative_to(REPO).parts for s in ('build','__pycache__'))]
names=[p.relative_to(REPO).as_posix() for p in files]
r=subprocess.run(['git','check-ignore','--stdin'],cwd=str(REPO),input=('\n'.join(names)+'\n').encode(),capture_output=True)
assert r.returncode in (0,1),r.stderr
skip=set(r.stdout.decode().splitlines());baseline={};copies={}
for p,name in zip(files,names):
    if name in skip or HERE in p.parents:continue
    baseline[name]=hashlib.sha256(p.read_bytes()).hexdigest()
    if source not in p.parents or (p.parent==source and p.name in history):continue
    rel=p.relative_to(source).as_posix().replace(OLD,NEW);out=target/rel;out.parent.mkdir(parents=True,exist_ok=True)
    data=p.read_bytes()
    if p.suffix.lower() in ('.ioc','.uvprojx','.dbgconf','.c','.md') or p.name=='.gitignore':data=data.replace(OLD.encode(),NEW.encode())
    out.write_bytes(data);copies[rel]=p.relative_to(source).as_posix()
(HERE/'BASELINE_SHA256.json').write_text(json.dumps(baseline,indent=2)+'\n',encoding='utf-8')
(HERE/'COPIED_PROJECT_FILES.json').write_text(json.dumps(copies,indent=2)+'\n',encoding='utf-8')
for n in ('arm_adapter.py','arm_bridge.c','map_annotations.js','test_ui.cjs','render_preview.py','replay.template.html'):
    s=(HERE.with_name('16i')/n).read_text(encoding='utf-8').replace(OLD,NEW).replace('16I','16J')
    (HERE/n).write_text(s,encoding='utf-8')
(HERE/'.gitignore').write_text('build/\n__pycache__/\n',encoding='utf-8')
print('Copied',len(copies),'files; preserved',len(baseline),'files')
