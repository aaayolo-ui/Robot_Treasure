"""Count emulated ARM instructions for representative plans; never wall time on MCU."""
import json
from arm_adapter import build,Planner,HERE
from unicorn import UC_HOOK_CODE
elf,_=build(label="profile");rows=[]
for evidence in (False,True):
    p=Planner(elf);p.initial();p.clear_gate()
    if evidence:
        for k in range(4):p.observe(38,1,10000+k*20,[2000]*4,[25]*4,15,0)
    per_frame=[];plan=None
    for k in range(5):
        count=[0]
        def hook(uc,address,size,data):count[0]+=1
        handle=p.u.hook_add(UC_HOOK_CODE,hook)
        obs=p.observe(38,1,10080+k*20,[2000,600,2000,2000],[25]*4,15,0)
        p.u.hook_del(handle);per_frame.append(count[0])
        if obs["plan"]:plan=obs["plan"]
    assert plan
    rows.append(dict(evidence=evidence,arm_instructions=max(per_frame),frame_instructions=per_frame,plan=plan))
    print(evidence,max(per_frame),plan["model_steps"],flush=True)
(HERE/"profile_result.json").write_text(json.dumps({"runs":rows,"scope":"Dynamic instruction counts for two inputs. No MCU cycle or 100ms-budget proof. Firmware reports COMPUTE_MS and enforces the unchanged 100ms guard."},indent=2)+"\n")
