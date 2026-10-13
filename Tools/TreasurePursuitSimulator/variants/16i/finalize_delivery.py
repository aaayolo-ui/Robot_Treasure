"""Write scoped source diffs and file inventories; never stage, commit or flash."""
from pathlib import Path
import difflib,hashlib,json,re,subprocess
HERE=Path(__file__).resolve().parent;REPO=HERE.parents[3]
FW=REPO/"STM32/Keil_Project/Thief16I_RolloutMinimax_FourLaser_Test"
OLD=FW.with_name("Thief16H_TurnClearance_FourLaser_Test")
DOC=REPO/"Docs/17_有界推演与最坏情况逃跑16I.md"
mapping=json.loads((HERE/"COPIED_PROJECT_FILES.json").read_text(encoding="utf8"))
audit=json.loads((HERE/"validation_result.json").read_text(encoding="utf8"))
changes=[];patch=[]
for new,old in sorted(mapping.items()):
    a,b=(OLD/old).read_bytes(),(FW/new).read_bytes()
    if a==b:continue
    delta=list(difflib.unified_diff([l+"\n" for l in a.decode("utf8",errors="replace").splitlines()],
        [l+"\n" for l in b.decode("utf8",errors="replace").splitlines()],fromfile="16H/"+old,tofile="16I/"+new))
    changes.append(dict(new_file=new,baseline_file=old,status="copied_then_modified",
        added_lines=sum(l.startswith("+") and not l.startswith("+++") for l in delta),
        removed_lines=sum(l.startswith("-") and not l.startswith("---") for l in delta),
        baseline_sha256=hashlib.sha256(a).hexdigest(),new_sha256=hashlib.sha256(b).hexdigest()))
    patch.extend(delta)
for folder in (FW/"Algorithm",FW/"Tests"):
    for f in sorted(folder.rglob("*")):
        name=f.relative_to(FW).as_posix()
        if not f.is_file() or f.suffix not in (".c",".h",".py") or "build" in f.parts or name in mapping:continue
        b=f.read_bytes();lines=[l+"\n" for l in b.decode("utf8").splitlines()]
        patch.extend(difflib.unified_diff([],lines,fromfile="/dev/null",tofile="16I/"+name))
        changes.append(dict(new_file=name,baseline_file=None,status="new_source",added_lines=len(lines),removed_lines=0,new_sha256=hashlib.sha256(b).hexdigest()))
(FW/"SOURCE_DIFF.patch").write_text("".join(patch),encoding="utf8")
(FW/"SOURCE_CHANGES.json").write_text(json.dumps(changes,indent=2)+"\n",encoding="utf8")
log=(FW/"MDK-ARM/build16i.log").read_text(encoding="utf8",errors="replace")
assert "0 Error(s), 0 Warning(s)" in log
code,ro,rw,zi=map(int,re.search(r"Code=(\d+) RO-data=(\d+) RW-data=(\d+) ZI-data=(\d+)",log).groups())
(FW/"VERIFICATION.md").write_text(f"""# 16I验证范围

Keil ARMCC5全量编译：0 Error(s), 0 Warning(s)。Code+RO-data {code+ro}字节，RW+ZI {rw+zi}字节。

实际C算法{audit['actual_c_checks']}项、UART435项均0失败；连续车体模型18项通过。579条路线、7631条连接与重生成稳定性通过。36组剪枝前后决策相同。

七个可播放场景及18场完整对照通过数据检查；全部比赛最终被抓，一个专项有退步。离线页面脚本、节点点击和开关通过；不是实际浏览器验证。

此前{audit['baseline_files_unchanged']}个文件SHA一致；117个底层文件与16H一致。完整Keil二进制两输入约118万/313万条ARM指令，不是板上100ms预算验证。

**仅编译通过，尚未完成硬件验证**。真实定位、测距、计算耗时和运动均需上车验证。未暂存、提交、推送、烧录或运行电机。

[详细算法与结果](../../../Docs/17_有界推演与最坏情况逃跑16I.md)。
""",encoding="utf8")
for folder in (FW,HERE):
    for name in ("FILE_MANIFEST.md","DIFF_SUMMARY.txt"):
        if not (folder/name).exists():(folder/name).write_text("",encoding="utf8")
allfiles=sorted(f for folder in (FW,HERE) for f in folder.rglob("*") if f.is_file())+[DOC]
names=[f.relative_to(REPO).as_posix() for f in allfiles]
r=subprocess.run(["git","check-ignore","--stdin"],cwd=str(REPO),input=("\n".join(names)+"\n").encode(),capture_output=True)
assert r.returncode in (0,1),r.stderr
ignored=set(r.stdout.decode().splitlines());files=[f for f,n in zip(allfiles,names) if n not in ignored]
ff=[f for f in files if FW in f.parents];rf=[f for f in files if HERE in f.parents]
summary=[f"Added firmware files: {len(ff)}; replay files: {len(rf)}; Docs/17: 1.",
 f"Derived source differences: {len(changes)} files, +{sum(c['added_lines'] for c in changes)}/-{sum(c['removed_lines'] for c in changes)} lines. Includes generated read-only table.",
 f"Existing {audit['baseline_files_unchanged']} files unchanged; no prior file modified by this task.",
 "117 Hardware/Control/Core/Drivers/System files byte-identical to 16H.",
 f"Keil: 0 errors, 0 warnings. Code+RO-data={code+ro}; RW+ZI={rw+zi} bytes.",
 "11397 C checks, 435 UART checks, 18 body tests, 36 pruning-parity cases passed.",
 "579 catalog routes and 7631 edges validated; seven replays and 18 complete comparison runs checked.",
 "All compared matches eventually captured the thief; the initial-visible north-approach fixture regressed.",
 "Offline UI tests passed. Hardware and actual browser layout/clicking unverified.",
 "Scoped git diff is empty because these are new untracked files; existing unrelated edits preserved.",
 "No git add, commit, push, flashing or real motor operation."]
for folder in (FW,HERE):(folder/"DIFF_SUMMARY.txt").write_text("\n".join(summary)+"\n",encoding="utf8")
changed={c['new_file'] for c in changes}
rows=["# 16I工程完整文件清单","",f"新增{len(ff)}个非忽略文件。旧16H未改。","","| 新增文件 | 相对16H |","| --- | --- |"]
for f in ff:
    name=f.relative_to(FW).as_posix();state="复制后修改" if name in changed and name in mapping else "原样复制" if name in mapping else "新增"
    rows.append(f"| [{name}]({name}) | {state} |")
rows += ["","[逐行派生差异](SOURCE_DIFF.patch)、[结构化差异](SOURCE_CHANGES.json)、[差异摘要](DIFF_SUMMARY.txt)。","","构建产物与缓存已忽略。仅编译通过，尚未完成硬件验证。"]
(FW/"FILE_MANIFEST.md").write_text("\n".join(rows)+"\n",encoding="utf8")
rows=["# 本次新增及修改文件完整清单","",f"新增工程{len(ff)}文件、回放{len(rf)}文件及Docs/17。此前文件未修改。","",
 "[工程逐文件清单](../../../../STM32/Keil_Project/Thief16I_RolloutMinimax_FourLaser_Test/FILE_MANIFEST.md)。",
 "[代码派生差异](../../../../STM32/Keil_Project/Thief16I_RolloutMinimax_FourLaser_Test/SOURCE_DIFF.patch)。",
 "[Git范围与差异摘要](DIFF_SUMMARY.txt)。","","| 回放新增文件 |","| --- |"]
for f in rf:
    name=f.relative_to(HERE).as_posix();rows.append(f"| [{name}]({name}) |")
rows += ["","新增文档：[Docs/17](../../../../Docs/17_有界推演与最坏情况逃跑16I.md)。","","PNG为报告数据绘制的静态示意图，非浏览器截图。未暂存、提交、推送或运行硬件。"]
(HERE/"FILE_MANIFEST.md").write_text("\n".join(rows)+"\n",encoding="utf8")
print("\n".join(summary))
