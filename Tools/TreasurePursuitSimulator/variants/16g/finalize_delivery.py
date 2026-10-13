"""Publish local manifests/diffs; never stage, commit, push or flash."""
from pathlib import Path
import difflib,hashlib,json,subprocess
HERE=Path(__file__).resolve().parent;REPO=HERE.parents[3]
FW=REPO/'STM32/Keil_Project/Thief16G_MultiExitHide_FourLaser_Test'
OLD=FW.with_name('Thief16E_VisibilityEscape_FourLaser_Test')
DOC=REPO/'Docs/15_多出口藏点与等待朝向16G.md'
mapping=json.loads((HERE/'COPIED_PROJECT_FILES.json').read_text(encoding='utf-8'))
changes=[];patch=[]
for new,old in sorted(mapping.items()):
    a,b=(OLD/old).read_bytes(),(FW/new).read_bytes()
    if a==b:continue
    delta=list(difflib.unified_diff(a.decode('utf-8',errors='replace').splitlines(True),
        b.decode('utf-8',errors='replace').splitlines(True),fromfile='16E/'+old,tofile='16G/'+new))
    plus=sum(l.startswith('+') and not l.startswith('+++') for l in delta)
    minus=sum(l.startswith('-') and not l.startswith('---') for l in delta)
    changes.append({'new_file':new,'baseline_file':old,'added_lines':plus,'removed_lines':minus,
        'baseline_sha256':hashlib.sha256(a).hexdigest(),'new_sha256':hashlib.sha256(b).hexdigest()})
    patch.extend(delta)
(FW/'SOURCE_DIFF.patch').write_text(''.join(patch),encoding='utf-8')
(FW/'SOURCE_CHANGES.json').write_text(json.dumps(changes,indent=2)+'\n',encoding='utf-8')
audit=json.loads((HERE/'validation_result.json').read_text(encoding='utf-8'))
(FW/'VERIFICATION.md').write_text('# 验证范围\n\n'+
    'Keil ARMCC5：0 Error(s), 0 Warning(s)。实际C算法10,802检查、0失败。\n\n'+
    '工作区13,269字节；Flash44,976字节、RAM34,368字节。117个硬件/控制/Core/系统/厂商文件与16E字节一致。\n\n'+
    '五场回放及离线页面检查通过；完整比赛复核未整体优于16F。仅编译通过，尚未完成硬件验证。\n\n'+
    '详细依据见 [Docs/15](../../../Docs/15_多出口藏点与等待朝向16G.md) 和 [回放验证结果](../../../Tools/TreasurePursuitSimulator/variants/16g/validation_result.json)。\n',encoding='utf-8')
for folder in (FW,HERE):
    for filename in ('FILE_MANIFEST.md','DIFF_SUMMARY.txt'):
        if not (folder/filename).exists():(folder/filename).write_text('',encoding='utf-8')
all_files=sorted(p for folder in (FW,HERE) for p in folder.rglob('*') if p.is_file())+[DOC]
names=[p.relative_to(REPO).as_posix() for p in all_files]
ignored=subprocess.run(['git','check-ignore','--stdin'],cwd=str(REPO),
    input=('\n'.join(names)+'\n').encode('utf-8'),capture_output=True)
assert ignored.returncode in (0,1),ignored.stderr
skip=set(ignored.stdout.decode('utf-8').splitlines())
files=[p for p,name in zip(all_files,names) if name not in skip]
firmware_files=[p for p in files if FW in p.parents]
replay_files=[p for p in files if HERE in p.parents]
summary=[f'New firmware files: {len(firmware_files)}; new replay files: {len(replay_files)}; new Docs/15: 1.',
    'No existing user file edited. Old 16E/16F baseline hashes: 230/230 unchanged.',
    f'Derived differences from the copied 16E project: {len(changes)} files, '+
       f'+{sum(c["added_lines"] for c in changes)}/-{sum(c["removed_lines"] for c in changes)} lines.',
    'Source rename/config/map metadata/score/diagnostic/test differences are listed in SOURCE_CHANGES.json and SOURCE_DIFF.patch.',
    'Scoped git diff is empty because this task only adds new untracked files; no staging.',
    'Keil rebuild: 0 errors, 0 warnings; actual Cortex-M3 C checks: 10802 passed.',
    'Five contact/route scenarios and offline DOM/controls passed; real browser/hardware untested.',
    'Full-match same-speed regression: 37.784117 s baseline versus 28.532011 s experiment.',
    'Police +20%: 25.426029 s baseline versus 25.761864 s experiment; zero thief wins.',
    '16F baseline retained. No git add/commit/push, MCU flashing or real motor operation.']
for folder in (FW,HERE):(folder/'DIFF_SUMMARY.txt').write_text('\n'.join(summary)+'\n',encoding='utf-8')
fwrows=['# 16G 实车实验文件清单','',
    f'本目录新增 {len(firmware_files)} 个非忽略文件，复制来源16E保持不变。所有文件列在下表。','',
    '派生修改清单见 `SOURCE_CHANGES.json`，逐行差异见 `SOURCE_DIFF.patch`。','',
    '| 新增文件（相对本目录） | 相对16E |','| --- | --- |']
changed={c['new_file'] for c in changes}
for p in firmware_files:
    name=p.relative_to(FW).as_posix()
    state='复制后修改' if name in changed else ('原样复制' if name in mapping else '新增元数据/说明')
    fwrows.append(f'| [{name}]({name}) | {state} |')
fwrows+=['','构建产物、Keil日志、Tests/build和Python缓存均忽略。仅编译通过，尚未完成硬件验证。']
(FW/'FILE_MANIFEST.md').write_text('\n'.join(fwrows)+'\n',encoding='utf-8')
rows=['# 本次完整新增文件清单','',
    f'新增实车实验 {len(firmware_files)} 个文件、回放 {len(replay_files)} 个文件及一份Docs/15；旧文件未修改。','',
    'Git范围差异为空，因为交付全部是新目录/新文件，未暂存。复制工程的派生差异见 [SOURCE_DIFF.patch](../../../../STM32/Keil_Project/Thief16G_MultiExitHide_FourLaser_Test/SOURCE_DIFF.patch)。','',
    '| 回放新增文件（相对本目录） |','| --- |']
for p in replay_files:
    name=p.relative_to(HERE).as_posix();rows.append(f'| [{name}]({name}) |')
rows+=['','实车工程的每个新增文件见 [实车完整清单](../../../../STM32/Keil_Project/Thief16G_MultiExitHide_FourLaser_Test/FILE_MANIFEST.md)。','',
    '项目新增文档：[Docs/15_多出口藏点与等待朝向16G.md](../../../../Docs/15_多出口藏点与等待朝向16G.md)。','',
    '未暂存、提交或推送。PNG为报告数据静态绘图，非浏览器截图；真实浏览器与实车未验证。']
(HERE/'FILE_MANIFEST.md').write_text('\n'.join(rows)+'\n',encoding='utf-8')
print('\n'.join(summary))
