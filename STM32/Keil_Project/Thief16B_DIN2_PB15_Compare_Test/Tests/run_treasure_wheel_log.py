"""Compile and emulate the actual passive treasure log, never hardware drivers.

Test-only getters provide reproducible register and feedback snapshots. The
production logger and wheel map are ARMCC 5 sources, not reimplemented models.
"""
from pathlib import Path
import hashlib
import json
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
DEPS = ROOT.parent / 'Thief16E_VisibilityEscape_FourLaser_Test/Tests/build/deps'
sys.path.insert(0, str(DEPS))
from elftools.elf.elffile import ELFFile
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_HOOK_CODE, UC_HOOK_MEM_WRITE, UC_HOOK_MEM_INVALID
from unicorn.arm_const import UC_ARM_REG_SP, UC_ARM_REG_LR, UC_ARM_REG_PC

BUILD = ROOT / 'Tests/build/treasure_diag'
BUILD.mkdir(parents=True, exist_ok=True)
COMPILER = Path('D:/keil5/ARM/ARMCC/bin/armcc.exe')
LINKER = COMPILER.with_name('armlink.exe')
SOURCES = [ROOT / 'Tests/test_treasure_wheel_log.c',
           ROOT / 'User/TreasureWheelLog.c',
           ROOT / 'Control/NewFrontDriveMap.c']
HEADERS = [ROOT / 'User/TreasureWheelLog.h',
           ROOT / 'Hardware/TreasureMotorSnapshot.h',
           ROOT / 'Control/NewFrontDriveMap.h',
           ROOT / 'Control/ChassisSpeedControl.h',
           ROOT / 'Control/Chassis.h',
           ROOT / 'Algorithm/SpeedPI/WheelSpeedPi.h',
           ROOT / 'Hardware/MotorDriver.h',
           ROOT / 'Hardware/Encoder.h',
           ROOT / 'System/UartDebug.h']
FILES = SOURCES + HEADERS + [Path(__file__).resolve()]
def hashes():
    return {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in FILES}
before = hashes()
objects, diagnostics = [], []
for source in SOURCES:
    obj = BUILD / (source.stem + '.o')
    objects.append(obj)
    args = [str(COMPILER), '--cpu=Cortex-M3', '--c99', '-O1', '-g',
            *['-I' + str(ROOT / d) for d in
              ['User', 'Control', 'Hardware', 'Algorithm/SpeedPI', 'System']],
            '-c', str(source), '-o', str(obj)]
    proc = subprocess.run(args, text=True, capture_output=True)
    diagnostics.append(proc.stdout + proc.stderr)
    if proc.returncode:
        print(diagnostics[-1]); proc.check_returncode()
elf = BUILD / 'treasure_wheel_log.axf'
proc = subprocess.run([str(LINKER), '--cpu=Cortex-M3', '--entry=test_main',
                       '--datacompressor=off', '--ro_base=0x08000000',
                       '--rw_base=0x20000000', *map(str, objects), '-o', str(elf)],
                      text=True, capture_output=True)
diagnostics.append(proc.stdout + proc.stderr)
if proc.returncode:
    print(diagnostics[-1]); proc.check_returncode()
u = Uc(UC_ARCH_ARM, UC_MODE_THUMB)
u.mem_map(0x08000000, 0x100000)
u.mem_map(0x20000000, 0xc000)
with elf.open('rb') as stream:
    e = ELFFile(stream)
    for section in e.iter_sections():
        if section['sh_flags'] & 2 and section['sh_type'] != 'SHT_NOBITS' and section['sh_size']:
            u.mem_write(section['sh_addr'], section.data())
    symbols = {s.name: s['st_value'] for s in e.get_section_by_name('.symtab').iter_symbols()}
u.reg_write(UC_ARM_REG_SP, 0x2000bff0)
u.reg_write(UC_ARM_REG_LR, 0x08000001)
completed, format_calls, moving_format_calls = [False], [0], [0]
def value(name):
    return int.from_bytes(u.mem_read(symbols[name], 4), 'little')
def done(uc, access, address, size, written, user_data):
    if address == symbols['test_done'] and written == 1:
        completed[0] = True; uc.emu_stop()
def formatting(uc, address, size, user_data):
    format_calls[0] += 1
    if value('test_running'):
        moving_format_calls[0] += 1
def invalid(uc, access, address, size, written, user_data):
    print(json.dumps({'invalid_address': hex(address), 'pc': hex(uc.reg_read(UC_ARM_REG_PC)),
                      'sp': hex(uc.reg_read(UC_ARM_REG_SP)), 'test_first_line': value('test_first_line')}))
    return False
u.hook_add(UC_HOOK_MEM_INVALID, invalid)
u.hook_add(UC_HOOK_MEM_WRITE, done, begin=symbols['test_done'], end=symbols['test_done'] + 3)
format_symbols = {a & ~1 for name, a in symbols.items() if name in ('snprintf', '__2snprintf')}
for addr in format_symbols:
    u.hook_add(UC_HOOK_CODE, formatting, begin=addr, end=addr)
assert format_symbols, 'No actual formatter symbol found; moving-format proof would be empty'
u.emu_start(symbols['test_main'] | 1, 0, count=100_000_000)
assert completed[0], 'CPU instruction budget exhausted'
after = hashes()
assert before == after, 'Source changed during compile/emulation: result is stale'
result = {n: value(n) for n in ['test_checks', 'test_failures', 'test_first_line',
                              'test_done', 'test_workspace_bytes', 'test_runs',
                              'test_uart_attempts', 'test_uart_accepted']}
result.update(format_calls=format_calls[0], moving_format_calls=moving_format_calls[0],
              compiler_diagnostics=[s for s in diagnostics if s.strip()],
              source_sha256=after,
              proof_scope='ARMCC 5 production User logger and wheel map with test-only passive getters; no real motor or UART operation')
(BUILD / 'result.json').write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8')
print(json.dumps(result, ensure_ascii=False, indent=2))
assert result['test_done'] == 1 and result['test_failures'] == 0
assert result['test_workspace_bytes'] < 13 * 1024
assert result['moving_format_calls'] == 0 and result['format_calls'] > 0
assert not result['compiler_diagnostics'], 'Compiler warning/error diagnostics remain'
