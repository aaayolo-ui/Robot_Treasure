"""Run the linked ARMCC5 test application with hardware stubbed; no serial IO."""
from pathlib import Path
import sys, json
ROOT = Path(__file__).absolute().parents[1]
sys.path.insert(0, str(ROOT.parent/'Thief16M_UARTCh4_InterceptEscape_Test/Tests/build/deps'))
from elftools.elf.elffile import ELFFile
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS, UC_HOOK_CODE
from unicorn.arm_const import *

with (ROOT/'MDK-ARM'/ROOT.name/(ROOT.name+'.axf')).open('rb') as source:
    elf = ELFFile(source)
    symbols = {s.name:s['st_value'] for s in elf.get_section_by_name('.symtab').iter_symbols()}
    u = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
    u.ctl_set_cpu_model(UC_CPU_ARM_CORTEX_M3)
    u.mem_map(0x08000000, 0x100000)
    u.mem_map(0x20000000, 0x10000)
    for section in elf.iter_sections():
        if section['sh_flags'] & 2 and section['sh_type'] != 'SHT_NOBITS' and section['sh_size']:
            u.mem_write(section['sh_addr'], section.data())
    sample_type = None
    for cu in elf.get_dwarf_info().iter_CUs():
        for die in cu.iter_DIEs():
            if die.tag == 'DW_TAG_typedef' and die.attributes.get('DW_AT_name') and die.attributes['DW_AT_name'].value == b'RangeSample':
                sample_type = die.get_DIE_from_attribute('DW_AT_type')
    stride = sample_type.attributes['DW_AT_byte_size'].value
    fields = {}
    for member in sample_type.iter_children():
        if member.tag == 'DW_TAG_member':
            offset = member.attributes['DW_AT_data_member_location'].value
            if not isinstance(offset, int): offset = offset[-1]  # ARMCC DW_OP_plus_uconst, small offsets
            typ = member.get_DIE_from_attribute('DW_AT_type')
            while typ.tag in ('DW_TAG_typedef', 'DW_TAG_const_type'):
                typ = typ.get_DIE_from_attribute('DW_AT_type')
            fields[member.attributes['DW_AT_name'].value.decode()] = (offset, typ.attributes['DW_AT_byte_size'].value)

def get(addr, size=4): return int.from_bytes(u.mem_read(addr, size), 'little')
def value(channel, field):
    offset, size = fields[field]
    return get(symbols['samples']+channel*stride+offset, size)
def string(addr):
    result = bytearray()
    while True:
        b = get(addr, 1); addr += 1
        if not b: return result.decode()
        result.append(b)
clock = 1000
ready = 1
calls = []
lines = []
statistics = []
handles = []
def hook(name, fn):
    address = symbols[name] & ~1
    def callback(uc, address, size, data):
        result = fn()
        if result is not None: uc.reg_write(UC_ARM_REG_R0, result)
        uc.reg_write(UC_ARM_REG_PC, uc.reg_read(UC_ARM_REG_LR))
    handles.append(u.hook_add(UC_HOOK_CODE, callback, begin=address, end=address))
def log_call(name, result=0):
    def fn(): calls.append(name); return result
    hook(name, fn)
for name in ('MotorDriver_Init', 'MotorDriver_Disable', 'LaserRx_Init', 'LaserRx_Poll',
             'LaserRx_BootLogTask', 'UartDebug_Task', 'LaserRx_Counters', 'UartDebug_SendString'):
    log_call(name)
hook('HAL_GetTick', lambda:clock)
hook('LaserRx_LinkReady', lambda:ready)
hook('LaserRx_SetEnabled', lambda:ready)
hook('LaserRx_InitFault', lambda:0 if ready else 7)
u.mem_write(0x2000e000,b'FRAMES4\0')
hook('LaserRx_LinkSource', lambda:0x2000e000)
def printf():
    fmt = string(u.reg_read(UC_ARM_REG_R0))
    sp = u.reg_read(UC_ARM_REG_SP)
    lines.append((fmt, string(get(sp)) if fmt.startswith('LASER DIR=') else None))
    if fmt.startswith('RANGE DIR='):
        statistics.append([u.reg_read(UC_ARM_REG_R2),u.reg_read(UC_ARM_REG_R3)] +
                          [get(sp+4*i) for i in range(8)])
hook('UartDebug_Printf', printf)
def call(name, *args):
    sp = 0x2000d000
    u.reg_write(UC_ARM_REG_SP, sp)
    for i, arg in enumerate(args):
        if i<4: u.reg_write((UC_ARM_REG_R0,UC_ARM_REG_R1,UC_ARM_REG_R2,UC_ARM_REG_R3)[i],arg)
        else: u.mem_write(sp+4*(i-4), int(arg).to_bytes(4,'little'))
    u.reg_write(UC_ARM_REG_LR, 0x080ff001)
    u.emu_start(symbols[name]|1, 0x080ff000, count=100000)
    assert u.reg_read(UC_ARM_REG_PC)==0x080ff000, name

call('App_LaserRangeTest_Init')
assert calls[:2] == ['MotorDriver_Init','MotorDriver_Disable']
for channel in range(4):
    for distance,confidence in ((900,100),(1000,100),(1100,100),(1500,100),(0,100),(600,0)):
        call('RangeFrame',channel,distance,confidence,104,1000)
    assert value(channel,'count')==4
    assert value(channel,'sum')==4500  # includes >1000; no 0.9 scaling
    assert value(channel,'minimum')==900 and value(channel,'maximum')==1500
    assert value(channel,'rejected')==2
    assert value(channel,'raw')==600 and value(channel,'confidence')==0
call('RangeFrame',0,0,0,0,1000)
assert value(0,'frames')==6  # synthetic FIFO loss did not add a frame
call('RangeFrame',0,1234,100,104,1000)
clock=1024; call('App_LaserRangeTest_Task')
assert not any(fmt.startswith('LASER DIR=') for fmt,status in lines)
clock=1025; call('App_LaserRangeTest_Task')
assert lines[-1][1]=='RAW' and value(0,'count')==5  # fast output must not clear statistics
clock=1050; call('App_LaserRangeTest_Task')
assert lines[-1][1]=='BAD_DATA'
clock=1750; call('App_LaserRangeTest_Task')
assert lines[-1][1]=='STALE'
clock=2000; call('App_LaserRangeTest_Task')
assert lines[-2][1]=='STALE'
assert statistics[-1]==[4,1125,900,1500,2,104,0,6,1000,60]
# Feed real 50 Hz traffic, including invalid ranges, while reporting at 10 Hz.
call('App_LaserRangeTest_Init'); statistics.clear(); lines.clear()
for elapsed in range(5,1201,5):
    clock=2000+elapsed
    if elapsed%20==0:
        for channel in range(4): call('RangeFrame',channel,900,100 if channel<3 else 0,104,clock)
    call('App_LaserRangeTest_Task')
assert len([fmt for fmt,status in lines if fmt.startswith('LASER DIR=')])==48
assert len(statistics)==4
for stat in statistics:
    assert 490<=stat[-1]<=510  # includes actual elapsed time for staggered windows
assert statistics[0][0]==0 and statistics[0][-1]==500  # zero-confidence frames still count for Hz
assert statistics[0][4]==50
ready=0; call('App_LaserRangeTest_Init')
clock+=50; call('App_LaserRangeTest_Task')
assert lines[-1][1]=='LINK_ERROR'
count=calls.count('LaserRx_Init')
clock+=3000; ready=1; call('App_LaserRangeTest_Task')
assert calls.count('LaserRx_Init')==count+1
assert lines[-2][1]=='WAIT_DATA'
assert 'MotorDriver_Enable' not in symbols  # linker removes all movement entry points
result={'result':'PASS','channels':4,'raw_range_and_statistics':True,'timeout_and_retry':True,
        'display_hz':10,'actual_50_hz_traffic':True,'zero_confidence_counts_for_hz':True,
        'motor_enable_symbol_absent':True,'hardware_tested':False}
(ROOT/'Tests/results.json').write_text(json.dumps(result,indent=2))
print(json.dumps(result))
