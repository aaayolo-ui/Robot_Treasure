"""Run actual ARMCC5 UART-CH4 transport and decoder against CPU-only HAL fixtures."""
from pathlib import Path
import hashlib
import json
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT.parent / 'Thief16E_VisibilityEscape_FourLaser_Test/Tests/build/deps'))
from elftools.elf.elffile import ELFFile
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS, UC_HOOK_MEM_WRITE
from unicorn.arm_const import UC_ARM_REG_SP, UC_ARM_REG_PC, UC_CPU_ARM_CORTEX_M3

build = ROOT / 'Tests/build'
build.mkdir(exist_ok=True)
compiler = Path('D:/keil5/ARM/ARMCC/bin/armcc.exe')
sources = [ROOT/'Tests/test_uartch4.c', ROOT/'Hardware/UartCh4.c', ROOT/'Hardware/LaserRx.c', ROOT/'Algorithm/LaserMux/AI1811Mux.c']
objects = []
diagnostics = []
for source in sources:
    obj = build / (source.stem + '.o')
    cmd = [str(compiler), '--cpu=Cortex-M3', '--c99', '-O1', '-g',
           '-DSTM32F103xE', '-DUSE_HAL_DRIVER', '--library_type=microlib']
    cmd += ['-I'+str(ROOT/d) for d in ['Hardware','User','System','Algorithm/LaserMux','Core/Inc',
        'Drivers/STM32F1xx_HAL_Driver/Inc','Drivers/CMSIS/Include',
        'Drivers/CMSIS/Device/ST/STM32F1xx/Include']]
    p = subprocess.run(cmd+['-c',str(source),'-o',str(obj)], capture_output=True, text=True)
    diagnostics.append(p.stdout+p.stderr)
    if p.returncode: print(diagnostics[-1]); p.check_returncode()
    objects.append(obj)
axf=build/'probe.axf'
p=subprocess.run([str(compiler.with_name('armlink.exe')), '--cpu=Cortex-M3',
    '--entry=test_main','--datacompressor=off','--library_type=microlib',
    '--ro_base=0x08000000','--rw_base=0x20000000',*map(str,objects),'-o',str(axf)],
    capture_output=True,text=True)
diagnostics.append(p.stdout+p.stderr)
if p.returncode: print(diagnostics[-1]); p.check_returncode()
u=Uc(UC_ARCH_ARM,UC_MODE_THUMB|UC_MODE_MCLASS)
u.ctl_set_cpu_model(UC_CPU_ARM_CORTEX_M3)
for address,size in [(0x08000000,0x100000),(0x20000000,0xc000),(0x40000000,0x100000)]: u.mem_map(address,size)
with axf.open('rb') as f:
    elf=ELFFile(f)
    for section in elf.iter_sections():
        if section['sh_flags']&2 and section['sh_type']!='SHT_NOBITS' and section['sh_size']:
            u.mem_write(section['sh_addr'],section.data())
    symbols={s.name:s['st_value'] for s in elf.get_section_by_name('.symtab').iter_symbols()}
u.reg_write(UC_ARM_REG_SP,0x2000bff0)
done=[False]
def written(uc,access,address,size,value,data):
    if value==1: done[0]=True; uc.emu_stop()
u.hook_add(UC_HOOK_MEM_WRITE,written,begin=symbols['test_done'],end=symbols['test_done']+3)
try:
    u.emu_start(symbols['test_main']|1,0,count=50_000_000)
except Exception:
    print('pc=',hex(u.reg_read(UC_ARM_REG_PC)), 'nearest=',sorted(
        [(hex(v),n) for n,v in symbols.items() if v<=u.reg_read(UC_ARM_REG_PC)],
        key=lambda x:int(x[0],16))[-8:])
    print({n:int.from_bytes(u.mem_read(symbols[n],4),'little') for n in
           ['test_checks','test_failures','test_first_line']})
    raise
result={n:int.from_bytes(u.mem_read(symbols[n],4),'little') for n in
        ['test_checks','test_failures','test_first_line','test_done']}
result.update(completed=done[0], diagnostics=[x for x in diagnostics if x.strip()],
    source_sha256={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources},
    scope='ARMCC5 UART-CH4 transport and decoder on CPU fixtures; not real module or motor proof')
(build/'uartch4_result.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
print(json.dumps(result,indent=2))
assert done[0] and result['test_failures']==0
assert not result['diagnostics']
