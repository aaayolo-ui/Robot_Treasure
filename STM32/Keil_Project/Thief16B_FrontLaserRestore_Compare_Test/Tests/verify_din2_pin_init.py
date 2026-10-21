"""Execute GPIO/MSP functions from the built ARMCC5 AXF in CPU-only MMIO.

No serial, debugger, target, motor, or USB access. MMIO is ordinary emulated RAM;
BSRR writes are inspected explicitly, not mistaken for physical output levels.
"""
from pathlib import Path
import hashlib
import json
import struct
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT.parent / 'Thief16E_VisibilityEscape_FourLaser_Test/Tests/build/deps'))
from elftools.elf.elffile import ELFFile
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_HOOK_MEM_WRITE
from unicorn.arm_const import UC_ARM_REG_SP, UC_ARM_REG_LR, UC_ARM_REG_PC, UC_ARM_REG_R0

AXF = ROOT / 'MDK-ARM/Thief16B_FrontLaserRestore_Compare_Test/Thief16B_FrontLaserRestore_Compare_Test.axf'
u = Uc(UC_ARCH_ARM, UC_MODE_THUMB)
for address, size in [(0x08000000, 0x100000), (0x20000000, 0xc000),
                      (0x40000000, 0x100000), (0xe000e000, 0x1000)]:
    u.mem_map(address, size)
with AXF.open('rb') as stream:
    elf = ELFFile(stream)
    for section in elf.iter_sections():
        if section['sh_flags'] & 2 and section['sh_type'] != 'SHT_NOBITS' and section['sh_size']:
            u.mem_write(section['sh_addr'], section.data())
    symbols = {s.name: s['st_value'] for s in elf.get_section_by_name('.symtab').iter_symbols()}

GPIOA, GPIOB, GPIOC = 0x40010800, 0x40010c00, 0x40011000
STOP, ARG = 0x080ff000, 0x2000b000
writes, calls = [], []
checks = 0


def put(address, value):
    u.mem_write(address, struct.pack('<I', value))


def get(address):
    return int.from_bytes(u.mem_read(address, 4), 'little')


def mode(port, pin):
    return (get(port + (0 if pin < 8 else 4)) >> ((pin % 8) * 4)) & 15


def check(condition, message):
    global checks
    checks += 1
    assert condition, message


def call(name, argument=0):
    u.reg_write(UC_ARM_REG_SP, 0x2000bff0)
    u.reg_write(UC_ARM_REG_LR, STOP | 1)
    u.reg_write(UC_ARM_REG_R0, argument)
    u.emu_start(symbols[name] | 1, STOP, count=1_000_000)
    check(u.reg_read(UC_ARM_REG_PC) == STOP, name + ' did not return')
    calls.append(name)


def isolation(stage):
    check(mode(GPIOA, 10) == 4, stage + ': PA10 must remain floating input')
    check(mode(GPIOB, 15) == 2, stage + ': PB15 must remain 2MHz push-pull output')
    check(mode(GPIOB, 14) == 4, stage + ': encoder B phase B must remain PB14 input')


def on_write(uc, access, address, size, value, data):
    writes.append((address, value))


u.hook_add(UC_HOOK_MEM_WRITE, on_write, begin=0x40000000, end=0x400fffff)
for port in (GPIOA, GPIOB, GPIOC):
    put(port, 0x44444444)
    put(port + 4, 0x44444444)
call('MX_GPIO_Init')
isolation('GPIO init')
check(mode(GPIOC, 10) == 4, 'encoder B phase A must remain PC10 input')
check(mode(GPIOC, 2) == 2, 'STBY must remain push-pull output')
check(any(a == GPIOB + 0x10 and v & (1 << 31) for a, v in writes), 'PB15 reset BSRR missing')
check(any(a == GPIOC + 0x10 and v & (1 << 18) for a, v in writes), 'STBY reset BSRR missing')
check((get(0x40010010) >> 8) & 15 == 2, 'EXTI10 must select GPIOC')
check((get(0x40010014) >> 8) & 15 == 1, 'EXTI14 must select GPIOB')
for register in (0x40010400, 0x40010408, 0x4001040c):
    check(get(register) & ((1 << 10) | (1 << 14)) == ((1 << 10) | (1 << 14)),
          'encoder EXTI mask/edge configuration missing')

for instance in (0x40000c00, 0x40013400):
    put(ARG, instance)
    call('HAL_TIM_MspPostInit', ARG)
    isolation('PWM MSP')
for instance in (0x40012c00, 0x40000000, 0x40000400):
    put(ARG, instance)
    call('HAL_TIM_Encoder_MspInit', ARG)
    isolation('encoder MSP')
for instance in (0x40004800, 0x40005000, 0x40013800, 0x40004c00, 0x40004400):
    put(ARG, instance)
    call('HAL_UART_MspInit', ARG)
    isolation('UART MSP')
for instance in (0x40013800, 0x40004c00, 0x40004400):
    put(ARG, instance)
    call('HAL_UART_MspDeInit', ARG)
    isolation('laser UART MSP deinit')

put(GPIOB + 8, 1 << 15)
put(GPIOC + 8, 0)
before = len(writes)
call('TreasureMotorSnapshot_ReadDIn2PinMap', ARG)
check(bytes(u.mem_read(ARG, 4)) == bytes([4, 2, 1, 0]), 'boot pin diagnostic readback differs')
check(len(writes) == before, 'passive pin diagnostic must not write MMIO')
put(GPIOB + 8, 0)
put(GPIOC + 8, 1 << 2)
call('TreasureMotorSnapshot_ReadDIn2PinMap', ARG)
check(bytes(u.mem_read(ARG, 4)) == bytes([4, 2, 0, 1]), 'boot level diagnostic does not follow IDR')
check(len(writes) == before, 'passive pin diagnostic wrote MMIO')

result = {'checks': checks, 'failures': 0, 'calls': calls,
          'axf_sha256': hashlib.sha256(AXF.read_bytes()).hexdigest(),
          'scope': 'Actual linked GPIO/MSP and pin readback executed on emulated MMIO; not hardware proof',
          'physical_interfaces_accessed': False}
out = ROOT / 'Tests/build'
out.mkdir(exist_ok=True)
(out / 'din2_pin_result.json').write_text(json.dumps(result, indent=2) + '\n', encoding='utf-8')
print(json.dumps(result, indent=2))
