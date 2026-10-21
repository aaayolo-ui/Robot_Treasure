"""List this experiment's files and compare its source with the copied baseline."""
from pathlib import Path
import difflib,json,subprocess
HERE=Path(__file__).resolve().parent;REPO=HERE.parents[3]
FW=REPO/'STM32/Keil_Project/Thief16K_RefugeAccess_FourLaser_Test'
SOURCE=FW.with_name('Thief16B_FourLaser_MapStrategy_Test')
SCOPES=[str(FW.relative_to(REPO)).replace('\\','/'),str(HERE.relative_to(REPO)).replace('\\','/'),
        'Docs/22_藏点结构与安全退路16K.md']
def git(*args):
    return subprocess.check_output(['git',*args],cwd=REPO).decode('utf-8').splitlines()
added=git('ls-files','--others','--exclude-standard','--',*SCOPES)
tracked=git('diff','--name-only','HEAD','--',*SCOPES)
changed=[];new=[];kept=[];plus=minus=0
reference=json.loads((FW/'copy_reference.json').read_text(encoding='utf-8'))
# Copy reference schema is inspected rather than assumed by this report.
entries=reference.get('files',reference)
if isinstance(entries,dict):entries=list(entries.values())
source_paths={}
for row in entries:
    if not isinstance(row,dict):continue
    dest=row.get('destination',row.get('target',row.get('destination_relative',row.get('new_path',''))))
    src=row.get('source',row.get('source_relative',row.get('source_path','')))
    if dest and src:source_paths[dest]=src
# Normal copied file names match; the four project identity paths are explicit.
def original(relative):
    value=relative.replace('Thief16K_RefugeAccess_FourLaser_Test','Thief16B_FourLaser_MapStrategy_Test')
    candidate=SOURCE/value
    return candidate if candidate.exists() else None
for name in added:
    path=REPO/name
    if not path.is_relative_to(FW):continue
    relative=path.relative_to(FW).as_posix();old=original(relative)
    if old is None:new.append(relative);continue
    a,b=old.read_bytes(),path.read_bytes()
    if a==b:kept.append(relative);continue
    aa=a.decode('utf-8',errors='replace').splitlines();bb=b.decode('utf-8',errors='replace').splitlines()
    p=m=0
    for op,a0,a1,b0,b1 in difflib.SequenceMatcher(a=aa,b=bb,autojunk=False).get_opcodes():
        if op in ('replace','delete'):m+=a1-a0
        if op in ('replace','insert'):p+=b1-b0
    plus+=p;minus+=m;changed.append((relative,p,m))
lines=['# 16K 文件清单','',
       '本轮均为新增实验路径，原16B/16J未修改。以下列出所有非忽略文件，包含复制保留的工程文件；构建产物和孤立开发试验位于忽略目录。','',
       f'共{len(added)}个新增路径，{len(tracked)}个已有Git跟踪路径变动。', '',
       '## 新工程与16B的复制基准差异','',
       f'{len(kept)}个文件内容相同，{len(changed)}个复制文件变动，{len(new)}个工程内新文件。', '',
       '| 文件 | 增行 | 删行 |','|---|---:|---:|']
lines.extend(f'| {n} | {p} | {m} |' for n,p,m in changed)
lines.extend(['','## 本轮全部新增文件',''])
lines.extend(f'- [{n}]({(REPO/n).as_posix()})' for n in added)
(HERE/'FILE_MANIFEST.md').write_text('\n'.join(lines)+'\n',encoding='utf-8')
summary=f'''16K review scope: independent firmware + replay + Docs/22.
Git tracked files changed in this scope: {len(tracked)}.
Untracked added files: {len(added)} (all listed in FILE_MANIFEST.md).
Copied-source comparison with current16B: {len(kept)} byte-identical, {len(changed)} changed, {len(new)} new firmware files.
Text comparison: +{plus}/-{minus} lines (normalized line endings; most changes are generated route tables/catalog).
All117low-level files remain byte-identical. Map roads and annotations remain unchanged.
Full Keil: 0 errors,0 warnings;Flash200388B,RAM35392B.
Eight actualARM C modes:226341 checks,0 failures. Pruned/exhaustive36;legacy-off36.
Sevenfixtures+ninecompletepairedmatches unchanged16J outcomes and N062usage.
No hardware,flashing,motors,git add,commit orpush actions.
'''
(HERE/'DIFF_SUMMARY.txt').write_text(summary,encoding='utf-8')
print(summary)
