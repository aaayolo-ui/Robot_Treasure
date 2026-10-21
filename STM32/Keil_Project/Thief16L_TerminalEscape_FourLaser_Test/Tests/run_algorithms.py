"""Run actual ARMCC-compiled map/gate code in a CPU emulator; no hardware access.
Requires unicorn + pyelftools. Install into your Python environment or set PYTHONPATH.
"""
from pathlib import Path
import subprocess, json, sys, hashlib
sys.path.insert(0,str(Path(__file__).resolve().parent/"build/deps"))
if not (Path(__file__).resolve().parent/'build/deps').exists():
 sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'Thief16E_VisibilityEscape_FourLaser_Test/Tests/build/deps'))
from elftools.elf.elffile import ELFFile
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_HOOK_MEM_WRITE
from unicorn.arm_const import UC_ARM_REG_SP, UC_ARM_REG_LR
import sys
root=Path(__file__).resolve().parents[1]
uart_test="--uart" in sys.argv
localization_test="--localization" in sys.argv
treasure_test="--treasure-laser" in sys.argv
edge_test="--edge-mask" in sys.argv
gray_test="--treasure-gray" in sys.argv
policy_test="--refuge-policy" in sys.argv
terminal_test="--terminal" in sys.argv
conditional_test="--conditional" in sys.argv
defines=[x.split('=',1)[1] for x in sys.argv if x.startswith('--define=')]
build=root/'Tests'/'build'; build.mkdir(exist_ok=True)
compiler=Path('D:/keil5/ARM/ARMCC/bin/armcc.exe')
linker=compiler.with_name('armlink.exe')
sources=[root/'Tests/test_algorithms.c',root/'Algorithm/ThiefStrategy/ThiefMapPlanner.c',root/'Algorithm/ThiefStrategy/FrontLaserGate.c',root/'Algorithm/MapRoute/MapRoutePlan.c']
if not uart_test: sources += [root/'Algorithm/ThiefStrategy/ThiefRiskSearch.c',root/'Algorithm/ThiefStrategy/ThiefThreatModel.c',root/'Algorithm/MapRoute/MapRouteNavigator.c']
if not uart_test: sources += [root/'Algorithm/ThiefStrategy/ThiefRollout.c',root/'Algorithm/ThiefStrategy/ThiefRefugePolicy.c',root/'Algorithm/ThiefStrategy/ThiefTerminalForecast.c']
if uart_test: sources=[root/'Tests/test_uart.c',root/'Hardware/LaserRx.c']
if localization_test:
 sources[0]=root/'Tests/test_localization.c'
 sources.append(root/'Algorithm/MapLocalization/ThiefLocalization.c')
if treasure_test:
 sources[0]=root/'Tests/test_treasure_laser.c'
 sources.append(root/'Algorithm/MapLocalization/TreasureLaserStart.c')
if edge_test:
 sources[0]=root/'Tests/test_edge_mask.c'
 sources.append(root/'Algorithm/MapLocalization/MapLaserMask.c')
if gray_test:
 sources[0]=root/'Tests/test_treasure_gray.c'
 sources += [root/'Algorithm/MapRoute/TreasureGrayMask.c',
             root/'Algorithm/MapLocalization/ThiefLocalization.c',
             root/'Algorithm/GrayPosition/gray_position.c']
if policy_test:
 sources[0]=root/'Tests/test_refuge_policy.c'
if terminal_test: sources=[root/'Tests/test_terminal_forecast.c',root/'Algorithm/ThiefStrategy/ThiefTerminalForecast.c']
if conditional_test: sources[0]=root/'Tests/test_conditional_stop.c'
objects=[]
source_sha256={str(source.relative_to(root)):hashlib.sha256(source.read_bytes()).hexdigest() for source in sources}
for folder in ('Tests/stubs','Hardware','Algorithm/ThiefStrategy','Algorithm/MapRoute','Algorithm/MapLocalization','Algorithm/GrayPosition'):
 for header in sorted((root/folder).glob('*.h')):
  source_sha256[str(header.relative_to(root))]=hashlib.sha256(header.read_bytes()).hexdigest()
for source in sources:
 obj=build/(source.stem+'.o');objects.append(obj)
 subprocess.run([str(compiler),'--cpu=Cortex-M3','--c99','-O1','-g','-DTHIEF_ROLLOUT_TEST=1',*['-D'+x for x in defines],'-I'+str(root/'Tests/stubs'),'-I'+str(root/'Hardware'),'-I'+str(root/'Algorithm/ThiefStrategy'),'-I'+str(root/'Algorithm/MapRoute'),'-I'+str(root/'Algorithm/MapLocalization'),'-I'+str(root/'Algorithm/GrayPosition'),'-c',str(source),'-o',str(obj)],check=True)
test_name='conditional' if conditional_test else 'terminal' if terminal_test else 'uart' if uart_test else 'localization' if localization_test else 'treasure_laser' if treasure_test else 'edge_mask' if edge_test else 'treasure_gray' if gray_test else 'refuge_policy' if policy_test else 'algorithms'
elf=build/(test_name+'.axf')
subprocess.run([str(linker),'--cpu=Cortex-M3','--entry=test_main','--datacompressor=off','--ro_base=0x08000000','--rw_base=0x20000000',*[str(x) for x in objects],'-o',str(elf)],check=True)
u=Uc(UC_ARCH_ARM,UC_MODE_THUMB)
u.mem_map(0x08000000,0x100000);u.mem_map(0x20000000,0xc000)
with elf.open('rb') as f:
 e=ELFFile(f)
 for seg in e.iter_segments():
  if seg['p_type']=='PT_LOAD':u.mem_write(seg['p_vaddr'],seg.data())
 for section in e.iter_sections():
  if section['sh_flags'] & 2 and section['sh_type']!='SHT_NOBITS' and section['sh_size']:
   u.mem_write(section['sh_addr'],section.data())
 sym={x.name:x['st_value'] for x in e.get_section_by_name('.symtab').iter_symbols()}
u.reg_write(UC_ARM_REG_SP,0x2000bff0);u.reg_write(UC_ARM_REG_LR,0x08000001)
completed=[False]
failure_callers=[]
def hook(uc,access,address,size,value,data):
 if address==sym['test_done'] and value==1:
  completed[0]=True;uc.emu_stop()
u.hook_add(UC_HOOK_MEM_WRITE,hook,begin=sym['test_done'],end=sym['test_done']+3)
def failure_hook(uc,access,address,size,value,data):
 if value: failure_callers.append(uc.reg_read(UC_ARM_REG_LR)&~1)
u.hook_add(UC_HOOK_MEM_WRITE,failure_hook,begin=sym['test_failures'],end=sym['test_failures']+3)
u.emu_start(sym['test_main']|1,0,count=2000000000)
assert completed[0], 'Instruction budget exhausted before completion'
names=['test_checks','test_failures','test_done']
if conditional_test: names += ['test_first_line','test_observations','test_plans','test_conditional_plans','test_conditional_checked_candidates','test_fallback_plans','test_max_model_steps','test_max_terminal_steps','test_finalist_fixtures','test_unknown_terminal_plans','test_partial_terminal_plans']
elif terminal_test: names += ['test_first_line','test_known_cases','test_unknown_cases','test_view_clear','test_view_escape_ms','test_work_bytes']
elif localization_test: names += ['test_routes','test_confirmed_nodes','test_first_line','test_first_error','test_first_from','test_first_to','test_first_phase']
elif treasure_test: names += ['test_first_line']
elif edge_test: names += ['test_first_line','test_node_poses','test_midpoint_poses']
elif gray_test: names += ['test_first_line','test_patterns']
elif policy_test: names += ['test_first_line']
elif not uart_test: names += ['test_initial_goal','test_initial_steps','test_workspace_bytes','test_routes_accepted','test_max_states']
result={x:int.from_bytes(u.mem_read(sym[x],4),'little') for x in names}
result['source_sha256']=source_sha256
result['defines']=defines
result['runner_sha256']=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
if failure_callers:
 result['failure_callers']=[hex(x) for x in failure_callers]
if not uart_test and not localization_test and not treasure_test and not edge_test and not gray_test and not policy_test and not terminal_test and not conditional_test: result['initial_nodes']=[int.from_bytes(u.mem_read(sym['test_initial_nodes']+i*4,4),'little') for i in range(result['test_initial_steps']+1)]
print(json.dumps(result));(build/('result.json' if test_name=='algorithms' else test_name+'_result.json')).write_text(json.dumps(result,indent=2))
assert result['test_done']==1 and result['test_failures']==0
