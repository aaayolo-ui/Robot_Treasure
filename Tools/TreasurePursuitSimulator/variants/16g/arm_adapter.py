"""Execute the bounded 16G C planner/gate in a Cortex-M3 CPU emulator."""
from pathlib import Path
import hashlib,json,struct,subprocess,sys

HERE=Path(__file__).resolve().parent
REPO=HERE.parents[3]
FIRMWARE=REPO/'STM32/Keil_Project/Thief16G_MultiExitHide_FourLaser_Test'
sys.path.insert(0,str(REPO/'STM32/Keil_Project/Thief16E_VisibilityEscape_FourLaser_Test/Tests/build/deps'))
from elftools.elf.elffile import ELFFile
from unicorn import Uc,UC_ARCH_ARM,UC_MODE_THUMB
from unicorn.arm_const import UC_ARM_REG_SP,UC_ARM_REG_LR,UC_ARM_REG_PC

def build(defines=(),label='selected',firmware=None):
    root=Path(firmware) if firmware else FIRMWARE
    out=HERE/'build'/label;out.mkdir(parents=True,exist_ok=True)
    cc=Path('D:/keil5/ARM/ARMCC/bin/armcc.exe');ld=cc.with_name('armlink.exe')
    sources=[HERE/'arm_bridge.c']+[root/'Algorithm/ThiefStrategy'/n for n in
        ('ThiefMapPlanner.c','ThiefRiskSearch.c','ThiefThreatModel.c','FrontLaserGate.c')]
    sources.append(root/'Algorithm/MapRoute/MapRoutePlan.c')
    hashes={str(s.relative_to(REPO)).replace('\\','/'):hashlib.sha256(s.read_bytes()).hexdigest() for s in sources}
    for n in ('ThiefStrategyConfig.h','ThiefMapTopology.h','ThiefMapPlanner.h',
              'ThiefRiskSearch.h','ThiefThreatModel.h','FrontLaserGate.h'):
        p=root/'Algorithm/ThiefStrategy'/n
        hashes[str(p.relative_to(REPO)).replace('\\','/')]=hashlib.sha256(p.read_bytes()).hexdigest()
    p=root/'Algorithm/MapRoute/MapRoutePlan.h'
    hashes[str(p.relative_to(REPO)).replace('\\','/')]=hashlib.sha256(p.read_bytes()).hexdigest()
    objects=[]
    for s in sources:
        obj=out/(s.stem+'.o');objects.append(obj)
        subprocess.run([str(cc),'--cpu=Cortex-M3','--c99','-O1','-g',
            *['-D'+x for x in defines],
            '-I'+str(root/'Algorithm/ThiefStrategy'),'-I'+str(root/'Algorithm/MapRoute'),
            '-c',str(s),'-o',str(obj)],check=True)
    elf=out/'bridge.axf'
    subprocess.run([str(ld),'--cpu=Cortex-M3','--entry=bridge_execute','--datacompressor=off',
        '--ro_base=0x08000000','--rw_base=0x20000000',*[str(o) for o in objects],'-o',str(elf)],check=True)
    return elf,hashes

class Planner:
    def __init__(self,elf):
        self.u=Uc(UC_ARCH_ARM,UC_MODE_THUMB)
        self.u.mem_map(0x08000000,0x100000);self.u.mem_map(0x20000000,0xc000)
        with elf.open('rb') as f:
            e=ELFFile(f)
            for seg in e.iter_segments():
                if seg['p_type']=='PT_LOAD':self.u.mem_write(seg['p_vaddr'],seg.data())
            for s in e.iter_sections():
                if s['sh_flags']&2 and s['sh_type']!='SHT_NOBITS' and s['sh_size']:
                    self.u.mem_write(s['sh_addr'],s.data())
            self.sym={x.name:x['st_value'] for x in e.get_section_by_name('.symtab').iter_symbols()}
    def put(self,key,value):
        self.u.mem_write(self.sym['bridge_'+key],struct.pack('<I',int(value)&0xffffffff))
    def get(self,key,count=1,signed=False):
        fmt='<'+('i' if signed else 'I')*count
        values=struct.unpack(fmt,self.u.mem_read(self.sym['bridge_'+key],4*count))
        return values[0] if count==1 else list(values)
    def execute(self,command):
        self.put('command',command)
        self.u.reg_write(UC_ARM_REG_SP,0x2000bff0)
        self.u.reg_write(UC_ARM_REG_LR,0x080ff001)
        self.u.emu_start(self.sym['bridge_execute']|1,0x080ff000,count=20000000)
        assert self.u.reg_read(UC_ARM_REG_PC)==0x080ff000, 'ARM instruction budget exhausted before returning'
    def plan(self):
        n=self.get('count')
        assert 0<n<=48
        fields={k:self.get(k,n,signed=(k=='action')) for k in ('from','to','length','step_heading','step_mode','action')}
        # get(count=1) intentionally returns a scalar; normalize step arrays.
        fields={k:v if isinstance(v,list) else [v] for k,v in fields.items()}
        return dict(fields,goal=self.get('goal'),mode=self.get('mode'),heading=self.get('heading'),
            travel_ms=self.get('time_ms'),margin_ms=self.get('margin',signed=True),
            states=self.get('states'),threats=self.get('threats'),degraded=bool(self.get('degraded')),
            reused_hide=bool(self.get('reused')),escape_exits=self.get('exits'),
            exit_branch_ms=self.get('branch_ms'),parking_heading=self.get('parking'))
    def initial(self):
        self.execute(0);assert self.get('ok');return self.plan()
    def clear_gate(self):self.execute(1)
    def observe(self,node,body,tick,distances,confidences,fresh,previous):
        for k,v in [('node',node),('body',body),('tick',tick),('previous',previous),('fresh',fresh)]:self.put(k,v)
        self.u.mem_write(self.sym['bridge_distance'],struct.pack('<4I',*distances))
        self.u.mem_write(self.sym['bridge_confidence'],struct.pack('<4I',*confidences))
        self.execute(2)
        return {'requested':bool(self.get('requested')),'ok':bool(self.get('ok')),
                'mask':self.get('mask'),'status':self.get('status',4),
                'plan':self.plan() if self.get('ok') else None}
