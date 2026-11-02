"""Compile real pure-C modules with ARMCC5 and run on Cortex-M3 emulation."""
from pathlib import Path
import json, subprocess, sys, hashlib, time
ROOT=Path(__file__).resolve().parents[1]
build=ROOT/'Tests/build'
sys.path.insert(0,str(build/'deps'))
from elftools.elf.elffile import ELFFile
from unicorn import Uc,UC_ARCH_ARM,UC_MODE_THUMB,UC_MODE_MCLASS,UC_HOOK_MEM_WRITE
from unicorn.arm_const import UC_ARM_REG_SP,UC_CPU_ARM_CORTEX_M3
compiler=Path('D:/keil5/ARM/ARMCC/bin/armcc.exe')
sources=['Tests/test_16m.c','Algorithm/ThiefStrategy/ThiefInterceptPolicy.c',
 'Algorithm/ThiefStrategy/ThiefMapPlanner.c','Algorithm/ThiefStrategy/ThiefRiskSearch.c',
 'Algorithm/ThiefStrategy/ThiefThreatModel.c','Algorithm/ThiefStrategy/FrontLaserGate.c',
 'Algorithm/MapLocalization/ThiefLocalization.c','Algorithm/MapLocalization/MapLaserMask.c',
 'Algorithm/MapRoute/MapRoutePlan.c','Algorithm/MapRoute/MapRouteNavigator.c']
build.mkdir(exist_ok=True)
objects=[]
for source in sources:
    obj=build/(Path(source).stem+'.o')
    cmd=[str(compiler),'--cpu=Cortex-M3','--c99','-O1','-g','--library_type=microlib']
    cmd+=['-I'+str(ROOT/d) for d in ['Algorithm/ThiefStrategy','Algorithm/MapLocalization','Algorithm/MapRoute']]
    p=subprocess.run(cmd+['-c',str(ROOT/source),'-o',str(obj)],capture_output=True,text=True)
    assert p.returncode==0 and not (p.stdout+p.stderr).strip(),p.stdout+p.stderr
    objects.append(obj)
axf=build/'test_16m.axf'
p=subprocess.run([str(compiler.with_name('armlink.exe')),'--cpu=Cortex-M3',
 '--entry=test_main','--datacompressor=off','--library_type=microlib',
 '--ro_base=0x08000000','--rw_base=0x20000000',*map(str,objects),'-o',str(axf)],capture_output=True,text=True)
assert p.returncode==0 and not (p.stdout+p.stderr).strip(),p.stdout+p.stderr
u=Uc(UC_ARCH_ARM,UC_MODE_THUMB|UC_MODE_MCLASS)
u.ctl_set_cpu_model(UC_CPU_ARM_CORTEX_M3)
u.mem_map(0x08000000,0x100000);u.mem_map(0x20000000,0xc000)
with axf.open('rb') as f:
    elf=ELFFile(f)
    for s in elf.iter_sections():
        if s['sh_flags']&2 and s['sh_type']!='SHT_NOBITS' and s['sh_size']:
            u.mem_write(s['sh_addr'],s.data())
    symbols={s.name:s['st_value'] for s in elf.get_section_by_name('.symtab').iter_symbols()}
u.reg_write(UC_ARM_REG_SP,0x2000bff0)
def done(uc,access,address,size,value,data):
    if value: uc.emu_stop()
u.hook_add(UC_HOOK_MEM_WRITE,done,begin=symbols['test_done'],end=symbols['test_done']+3)
start=time.monotonic()
u.emu_start(symbols['test_main']|1,0,count=2_000_000_000)
result={n:int.from_bytes(u.mem_read(symbols[n],4),'little') for n in
 ['test_checks','test_failures','test_first_line','test_case','test_done']}
result['elapsed_seconds']=round(time.monotonic()-start,3)
result['scope']='ARMCC5 actual algorithm and localization on CPU emulation; no motor/hardware execution'
result['source_sha256']={s:hashlib.sha256((ROOT/s).read_bytes()).hexdigest() for s in sources}
(ROOT/'Tests/results_16m.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
print(json.dumps(result,indent=2),flush=True)
assert result['test_done'] and not result['test_failures']
