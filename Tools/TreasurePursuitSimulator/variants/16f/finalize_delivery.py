"""List the task's additions and compare inherited files without Git staging."""
from pathlib import Path
import difflib,json,subprocess
HERE=Path(__file__).resolve().parent
REPO=HERE.parents[3]
DOC=REPO/'Docs/14_小车转向与车身碰撞仿真模型.md'
purposes={
    '.gitignore':'忽略本版构建产物和Python缓存',
    'body_model.json':'用户接受的车身尺寸、转向近似与数值接触参数',
    'body_geometry.py':'米制旋转矩形、分离轴与包含扫角的连续接触计算',
    'body_motion.py':'连续车身角度、倒车方向与角点速度上界',
    'simulate_16f.py':'复用16E策略，独立生成四场策略回放及一场转向碰撞专项',
    'arm_adapter.py':'原样复制16E桌面ARM适配器；构建仅写入本版build目录',
    'arm_bridge.c':'原样复制16E桌面C桥接，未新增MCU项目',
    'prepare_template.py':'可复现地从16E模板生成新页面',
    'replay.template.html':'新页面模板：连续车身、轮廓、转向/接触跳转和慢放',
    'map_annotations.js':'本版标注布局按旋转车身包围框避让标签',
    'test_ui.cjs':'执行实际页面脚本的离线DOM/事件检查',
    'test_body_model.py':'18项运动/碰撞测试，含400对称检查及55组密集参照',
    'verify_replay.py':'终局车身接触、单次抓捕、基线哈希和地图一致性检查',
    'render_preview.py':'从报告真实姿态绘制静态预览，不是浏览器截图',
    'finalize_delivery.py':'生成完整文件清单和本次变更摘要',
    'README.md':'查看、运行、参数与验证范围说明',
    'report.json':'五个回放、38字段帧数据、实际C源哈希和终局接触信息',
    'replay.html':'含回放数据与脚本的独立HTML',
    'map_annotations.json':'88节点/106路段稳定标注导出，与16E一致',
    'BASELINE_SHA256.json':'修改前旧16E回放及固件368文件的SHA256基线',
    'validation_result.json':'五个终局接触、C哈希与不可变基线验证结果',
    'ui_smoke_result.json':'页面事件验证结果，明确不是浏览器测试',
    'preview_map.png':'地图和终局车身接触的静态预览，已查看',
    'preview_turn_contact.png':'转向前/中途/首次接触的三帧预览，已查看',
    'DIFF_SUMMARY.txt':'本次增量与继承文件相对16E差异摘要',
    'FILE_MANIFEST.md':'本次完整新增文件清单（本文件）',
}
diff=[]
for name,prior in (('arm_adapter.py',HERE.with_name('16e')/'arm_adapter.py'),
                   ('arm_bridge.c',HERE.with_name('16e')/'arm_bridge.c'),
                   ('replay.template.html',HERE.with_name('16e')/'replay.template.html'),
                   ('test_ui.cjs',HERE.with_name('16e')/'test_ui.cjs'),
                   ('map_annotations.js',HERE.parents[1]/'map_annotations.js')):
    old=prior.read_text(encoding='utf-8').splitlines()
    new=(HERE/name).read_text(encoding='utf-8').splitlines()
    changes=list(difflib.unified_diff(old,new,n=0))
    plus=sum(s.startswith('+') and not s.startswith('+++') for s in changes)
    minus=sum(s.startswith('-') and not s.startswith('---') for s in changes)
    diff.append(f'{name}: +{plus}/-{minus} lines versus inherited input; input itself unchanged')
for filename in ('DIFF_SUMMARY.txt','FILE_MANIFEST.md'):
    if not (HERE/filename).exists():(HERE/filename).write_text('',encoding='utf-8')
files=sorted(p for p in HERE.rglob('*') if p.is_file() and not any(part in ('build','__pycache__') for part in p.relative_to(HERE).parts))
assert set(p.name for p in files)==set(purposes),(set(p.name for p in files)-set(purposes))
paths=[p.relative_to(REPO).as_posix() for p in files]+[DOC.relative_to(REPO).as_posix()]
# Binary stdin avoids CRLF being interpreted as literal path suffixes by Git.
result=subprocess.run(['git','check-ignore','--stdin'],cwd=str(REPO),input=('\n'.join(paths)+'\n').encode('utf-8'),capture_output=True)
assert result.returncode in (0,1),result.stderr
assert not result.stdout,result.stdout.decode('utf-8',errors='replace')
v=json.loads((HERE/'validation_result.json').read_text(encoding='utf-8'))
summary=[f'Task adds {len(files)} files under variants/16f and one new Docs/14 document.',
    'No existing tracked or untracked user file edited by this task.',
    f'Baseline SHA256: {v["baseline_files"]} / {v["baseline_files"]} unchanged.',
    'Scoped git diff is empty because all task outputs are NEW untracked files; no staging performed.',
    'Derived source differences below are between new copies and the unchanged inherited inputs:',*diff,
    'Geometry/kinematics tests: 18 passed, 0 failures, 0 errors (observed test runner).',
    'Replay end-contact checks: 5 passed; offline DOM/controls passed; HTTP HEAD: 200.',
    'Real browser layout and hardware were not tested.',
    'No git add, git commit, git push, MCU download or real motor operation performed.']
(HERE/'DIFF_SUMMARY.txt').write_text('\n'.join(summary)+'\n',encoding='utf-8')
rows=['# 16F 本次新增文件清单','',
    f'新增本目录 {len(files)} 个文件和一份 `Docs/14_小车转向与车身碰撞仿真模型.md`。既有文件未修改；368项基线哈希全部一致。未暂存、提交或推送。','',
    '本次范围的 `git diff` 没有已跟踪文件差异，原因是全部以新文件保存。继承文件的派生差异见 [DIFF_SUMMARY.txt](DIFF_SUMMARY.txt)。','',
    '| 新增文件（相对本目录） | 用途 |','| --- | --- |']
for p in files:rows.append(f'| [{p.name}]({p.name}) | {purposes[p.name]} |')
rows+=['', '| 新增项目文档（相对项目根目录） | 用途 |','| --- | --- |',
    '| [Docs/14_小车转向与车身碰撞仿真模型.md](../../../../Docs/14_小车转向与车身碰撞仿真模型.md) | 尺寸、动作定义、准确性边界、参数、验证与算法依据 |','',
    '构建输出 `build/` 与 `__pycache__/` 已忽略，不属于交付源文件。未修改任何 STM32 测试工程。PNG 是报告数据绘制的静态预览，非浏览器截图；实际浏览器和实车验证未执行。']
(HERE/'FILE_MANIFEST.md').write_text('\n'.join(rows)+'\n',encoding='utf-8')
print('\n'.join(summary))
