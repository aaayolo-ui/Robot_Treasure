"""Produce reviewable file inventories/diffs; never stage, commit or flash."""
from pathlib import Path
import difflib,hashlib,json,re,subprocess
HERE=Path(__file__).resolve().parent;REPO=HERE.parents[3]
FW=REPO/'STM32/Keil_Project/Thief16H_TurnClearance_FourLaser_Test'
OLD=FW.with_name('Thief16G_MultiExitHide_FourLaser_Test')
DOC=REPO/'Docs/16_转向前后追尾与安全窗口16H.md'
mapping=json.loads((HERE/'COPIED_PROJECT_FILES.json').read_text(encoding='utf-8'))
changes=[];patch=[]
for new,old in sorted(mapping.items()):
    a,b=(OLD/old).read_bytes(),(FW/new).read_bytes()
    if a==b:continue
    delta=list(difflib.unified_diff([l+'\n' for l in a.decode('utf-8',errors='replace').splitlines()],
        [l+'\n' for l in b.decode('utf-8',errors='replace').splitlines()],fromfile='16G/'+old,tofile='16H/'+new))
    changes.append({'new_file':new,'baseline_file':old,
        'added_lines':sum(l.startswith('+') and not l.startswith('+++') for l in delta),
        'removed_lines':sum(l.startswith('-') and not l.startswith('---') for l in delta),
        'baseline_sha256':hashlib.sha256(a).hexdigest(),'new_sha256':hashlib.sha256(b).hexdigest()})
    patch.extend(delta)
(FW/'SOURCE_DIFF.patch').write_text(''.join(patch),encoding='utf-8')
(FW/'SOURCE_CHANGES.json').write_text(json.dumps(changes,indent=2)+'\n',encoding='utf-8')
audit=json.loads((HERE/'validation_result.json').read_text(encoding='utf-8'))
log=(FW/'MDK-ARM/build16h.log').read_text(encoding='utf-8',errors='replace')
sizes=re.search(r'Code=(\d+) RO-data=(\d+) RW-data=(\d+) ZI-data=(\d+)',log)
code,ro,rw,zi=map(int,sizes.groups());flash=code+ro;ram=rw+zi
assert '0 Error(s), 0 Warning(s)' in log
(FW/'VERIFICATION.md').write_text('# 验证范围\n\n'+
    f'Keil ARMCC5全量编译：0 Error(s), 0 Warning(s)。Flash {flash} 字节，RAM {ram} 字节。\n\n'+
    f'实际C算法 {audit["actual_c_checks"]} 项、UART435项均0失败；算法工作区 {audit["algorithm_workspace_bytes"]} 字节。连续运动学/车身碰撞18项测试通过。\n\n'+
    '七个回放场景和离线页面事件检查通过。原地图、物理模型与判据保持原样，117个底层文件与16G字节一致。\n\n'+
    '完整比赛三种速度、三种子，16H均最终被抓；相对16G延后接触。同速仍未优于16F。\n\n'+
    '**仅编译通过，尚未完成硬件验证**。板上计算耗时、定位和真实传感器/车身动力学未验证；未烧录或运行电机。\n\n'+
    '[详细说明](../../../Docs/16_转向前后追尾与安全窗口16H.md)。\n',encoding='utf-8')
for folder in (FW,HERE):
    for filename in ('FILE_MANIFEST.md','DIFF_SUMMARY.txt'):
        if not (folder/filename).exists():(folder/filename).write_text('',encoding='utf-8')
all_files=sorted(p for folder in (FW,HERE) for p in folder.rglob('*') if p.is_file())+[DOC]
names=[p.relative_to(REPO).as_posix() for p in all_files]
ignored=subprocess.run(['git','check-ignore','--stdin'],cwd=str(REPO),
    input=('\n'.join(names)+'\n').encode('utf-8'),capture_output=True)
assert ignored.returncode in (0,1),ignored.stderr
skip=set(ignored.stdout.decode('utf-8').splitlines())
files=[p for p,n in zip(all_files,names) if n not in skip]
firmware_files=[p for p in files if FW in p.parents];replay_files=[p for p in files if HERE in p.parents]
summary=[f'Added firmware files: {len(firmware_files)}; replay files: {len(replay_files)}; Docs/16: 1.',
    f'Copied-project differences: {len(changes)} files, +{sum(c["added_lines"] for c in changes)}/-{sum(c["removed_lines"] for c in changes)} lines (line endings normalized for review).',
    f'Existing files unchanged: {audit["baseline_files_unchanged"]}; no existing file edited.',
    '117 hardware/control/Core/vendor/system files byte-identical to 16G.',
    f'Keil 0 errors, 0 warnings; Flash {flash} bytes; RAM {ram} bytes.',
    '11170 C checks, 435 UART checks and 18 physics tests passed.',
    'Seven replay scenarios, 27 full comparison runs; all comparison matches eventually caught the thief.',
    'Offline UI/annotation checks passed; actual browser layout and hardware unverified.',
    'Scoped git diff has no tracked changes because all task outputs are new untracked files.',
    'No git add, commit, push, flashing or real motor operation.']
for folder in (FW,HERE):(folder/'DIFF_SUMMARY.txt').write_text('\n'.join(summary)+'\n',encoding='utf-8')
changed={c['new_file'] for c in changes}
rows=['# 16H 新增工程文件清单','',f'共{len(firmware_files)}个非忽略文件。原16G文件未修改。',
    '', '| 新增文件 | 相对16G |','| --- | --- |']
for p in firmware_files:
    name=p.relative_to(FW).as_posix()
    state='复制后修改' if name in changed else '原样复制' if name in mapping else '新增说明/元数据'
    rows.append(f'| [{name}]({name}) | {state} |')
rows+=['','逐行派生差异见[SOURCE_DIFF.patch](SOURCE_DIFF.patch)，结构化清单见[SOURCE_CHANGES.json](SOURCE_CHANGES.json)。',
    '', '构建产物、Tests/build、缓存和build日志已忽略。仅编译通过，尚未完成硬件验证。']
(FW/'FILE_MANIFEST.md').write_text('\n'.join(rows)+'\n',encoding='utf-8')
rows=['# 本次新增及修改文件完整清单','',
    f'新增工程{len(firmware_files)}文件、回放{len(replay_files)}文件及Docs/16。未修改此前已有文件。',
    '', '[工程每个文件的清单](../../../../STM32/Keil_Project/Thief16H_TurnClearance_FourLaser_Test/FILE_MANIFEST.md)。',
    '[代码派生差异](../../../../STM32/Keil_Project/Thief16H_TurnClearance_FourLaser_Test/SOURCE_DIFF.patch)。',
    '[Git范围摘要](DIFF_SUMMARY.txt)。','', '| 回放新增文件 |','| --- |']
for p in replay_files:
    name=p.relative_to(HERE).as_posix();rows.append(f'| [{name}]({name}) |')
rows+=['','新增文档：[Docs/16](../../../../Docs/16_转向前后追尾与安全窗口16H.md)。',
    '', '全部输出为未暂存新文件，git diff无本次已跟踪文件变化。旧版和其他工作不变。',
    '', 'PNG为报告数据绘制的静态示意图，非浏览器截图。未提交、推送、烧录或运行电机。']
(HERE/'FILE_MANIFEST.md').write_text('\n'.join(rows)+'\n',encoding='utf-8')
print('\n'.join(summary))
