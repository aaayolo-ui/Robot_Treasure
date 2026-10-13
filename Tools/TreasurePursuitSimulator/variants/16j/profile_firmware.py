"""Profile actual fully linked Keil binary on an offline CPU, without HAL/motors."""
import json,struct,hashlib
from arm_adapter import Planner,FIRMWARE,HERE
from unicorn import UC_HOOK_CODE
from unicorn.arm_const import UC_ARM_REG_R0,UC_ARM_REG_R1,UC_ARM_REG_R2,UC_ARM_REG_R3,UC_ARM_REG_SP,UC_ARM_REG_LR,UC_ARM_REG_PC
elf=FIRMWARE/"MDK-ARM"/FIRMWARE.name/(FIRMWARE.name+".axf")
rows=[]
for evidence in (False,True):
    p=Planner(elf)
    def call(name,args=(),profile=False):
        sp=0x2000bfc0
        for reg,value in zip((UC_ARM_REG_R0,UC_ARM_REG_R1,UC_ARM_REG_R2,UC_ARM_REG_R3),args):p.u.reg_write(reg,value&0xffffffff)
        if len(args)>4:p.u.mem_write(sp,struct.pack("<"+"I"*(len(args)-4),*args[4:]))
        p.u.reg_write(UC_ARM_REG_SP,sp);p.u.reg_write(UC_ARM_REG_LR,0x080ff001)
        count=[0]
        def hook(uc,address,size,data):count[0]+=1
        handle=p.u.hook_add(UC_HOOK_CODE,hook) if profile else None
        p.u.emu_start(p.sym[name]|1,0x080ff000,count=50000000)
        if handle:p.u.hook_del(handle)
        assert p.u.reg_read(UC_ARM_REG_PC)==0x080ff000
        return p.u.reg_read(UC_ARM_REG_R0),count[0]
    call("ThiefMapPlanner_Init")
    if evidence:
        for k in range(4):call("ThiefRollout_Evidence",(38,0,2000,25,10000+k*20))
        call("ThiefRollout_Evidence",(38,0,600,25,10080))
    call("ThiefMapPlanner_ObserveThreat",(38,0,300,600,10160))
    ok,instructions=call("ThiefMapPlanner_PlanEscape",(38,1,1,0,10160,0x2000a000),True)
    assert ok
    address,_=call("ThiefMapPlanner_Decision")
    data=bytes(p.u.mem_read(address,48))
    row=dict(evidence=evidence,arm_instructions=instructions,goal=struct.unpack_from("<H",data,10)[0],
             candidates=struct.unpack_from("<H",data,24)[0],models=struct.unpack_from("<H",data,26)[0],
             model_steps=struct.unpack_from("<I",data,40)[0],entry_inferred=data[45])
    rows.append(row);print(row,flush=True)
(HERE/"profile_firmware_result.json").write_text(json.dumps({"runs":rows,
 "binary":elf.relative_to(HERE.parents[3]).as_posix(),
 "binary_sha256":hashlib.sha256(elf.read_bytes()).hexdigest(),
 "scope":"Actual fully linked Keil binary; dynamic ARM instructions on CPU emulator, not hardware cycles/time. All calls use pure algorithm APIs. The unchanged 100ms guard still needs board verification."},indent=2)+"\n")
