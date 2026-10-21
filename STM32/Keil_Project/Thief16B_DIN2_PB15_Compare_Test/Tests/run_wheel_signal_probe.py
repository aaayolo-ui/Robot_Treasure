"""ARMCC5/Unicorn register-fixture checks; no connection to real hardware."""
from pathlib import Path
import hashlib
import json
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT.parent / 'Thief16E_VisibilityEscape_FourLaser_Test/Tests/build/deps'))
from elftools.elf.elffile import ELFFile
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_HOOK_MEM_WRITE, UC_HOOK_MEM_INVALID
from unicorn.arm_const import UC_ARM_REG_SP, UC_ARM_REG_LR, UC_ARM_REG_PC

BUILD = ROOT / 'Tests/build/wheel_signal'
BUILD.mkdir(parents=True, exist_ok=True)
compiler = Path('D:/keil5/ARM/ARMCC/bin/armcc.exe')
sources = [ROOT / 'Tests/test_wheel_signal_probe.c', ROOT / 'Hardware/TreasureMotorSnapshot.c']
inputs = sources + [ROOT / 'Hardware/TreasureMotorSnapshot.h', Path(__file__).resolve()]
hashes = {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in inputs}
objects = []
diagnostics = []
for source in sources:
    obj = BUILD / (source.stem + '.o')
    args = [str(compiler), '--cpu=Cortex-M3', '--c99', '-O1', '-g',
            '-DSTM32F103xE', '-DUSE_HAL_DRIVER',
            *['-I' + str(ROOT / d) for d in ['Hardware', 'Core/Inc',
              'Drivers/STM32F1xx_HAL_Driver/Inc', 'Drivers/CMSIS/Include',
              'Drivers/CMSIS/Device/ST/STM32F1xx/Include']],
            '-c', str(source), '-o', str(obj)]
    proc = subprocess.run(args, text=True, capture_output=True)
    diagnostics.append(proc.stdout + proc.stderr)
    if proc.returncode:
        print(diagnostics[-1]); proc.check_returncode()
    objects.append(obj)
elf = BUILD / 'wheel_signal.axf'
proc = subprocess.run([str(compiler.with_name('armlink.exe')), '--cpu=Cortex-M3',
                       '--entry=test_main', '--datacompressor=off',
                       '--ro_base=0x08000000', '--rw_base=0x20000000',
                       *map(str, objects), '-o', str(elf)], text=True, capture_output=True)
diagnostics.append(proc.stdout + proc.stderr)
if proc.returncode:
    print(diagnostics[-1]); proc.check_returncode()
u = Uc(UC_ARCH_ARM, UC_MODE_THUMB)
u.mem_map(0x08000000, 0x100000)
u.mem_map(0x20000000, 0xc000)
u.mem_map(0x40000000, 0x100000)
with elf.open('rb') as stream:
    e = ELFFile(stream)
    for section in e.iter_sections():
        if section['sh_flags'] & 2 and section['sh_type'] != 'SHT_NOBITS' and section['sh_size']:
            u.mem_write(section['sh_addr'], section.data())
    symbols = {s.name: s['st_value'] for s in e.get_section_by_name('.symtab').iter_symbols()}
u.reg_write(UC_ARM_REG_SP, 0x2000bff0)
u.reg_write(UC_ARM_REG_LR, 0x08000001)
completed = [False]
production_mmio_writes = [0]
production_active = [False]
def value(name):
    return int.from_bytes(u.mem_read(symbols[name], 4), 'little')
def writes(uc, access, address, size, written, user_data):
    # Shadow the fixture marker from the write event itself. Avoid reentrant
    # register/memory reads inside Unicorn's MMIO write hook.
    if address == symbols['test_production_call']:
        production_active[0] = bool(written)
    if address == symbols['test_done'] and written == 1:
        completed[0] = True
        uc.emu_stop()
    if 0x40000000 <= address < 0x40100000 and production_active[0]:
        production_mmio_writes[0] += 1
u.hook_add(UC_HOOK_MEM_WRITE, writes, begin=symbols['test_production_call'], end=symbols['test_production_call']+3)
u.hook_add(UC_HOOK_MEM_WRITE, writes, begin=symbols['test_done'], end=symbols['test_done']+3)
u.hook_add(UC_HOOK_MEM_WRITE, writes, begin=0x40000000, end=0x400fffff)
def invalid(uc, access, address, size, written, user_data):
    print(json.dumps({'invalid_address': hex(address), 'pc': hex(uc.reg_read(UC_ARM_REG_PC)),
                      'sp': hex(uc.reg_read(UC_ARM_REG_SP)), 'checks': value('test_checks')}))
    return False
u.hook_add(UC_HOOK_MEM_INVALID, invalid)
u.emu_start(symbols['test_main'] | 1, 0, count=50_000_000)
assert completed[0], 'Instruction budget exhausted'
assert hashes == {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in inputs}
result = {n: value(n) for n in ('test_checks', 'test_failures', 'test_first_line', 'test_done')}
result.update(production_mmio_writes=production_mmio_writes[0], source_sha256=hashes,
              diagnostics=[s for s in diagnostics if s.strip()],
              scope='Actual Hardware probe on emulated MMIO; not a real pin, sensor or motor test')
(BUILD / 'result.json').write_text(json.dumps(result, indent=2), encoding='utf-8')
print(json.dumps(result, indent=2))
assert result['test_failures'] == 0 and result['production_mmio_writes'] == 0
assert not result['diagnostics']
