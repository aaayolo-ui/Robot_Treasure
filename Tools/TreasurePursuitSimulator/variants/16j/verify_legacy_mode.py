"""Legacy flag versus frozen 16I C decision parity; no hardware or browser."""
import json
from arm_adapter import build,Planner,HERE,FIRMWARE
reference=Planner(build((),"legacy_reference",FIRMWARE.with_name("Thief16I_RolloutMinimax_FourLaser_Test"))[0])
selected=Planner(build(("THIEF_ROLLOUT_RISK_TIERS=0",),"legacy_mode")[0])
for p in (reference,selected):p.initial()
report=json.loads((HERE/"report.json").read_text(encoding="utf8"))
nodes={int(n["id"][1:]):n for n in report["annotations"]["nodes"]}
offsets=(0,3,1,2);directions={"N":0,"E":1,"S":2,"W":3}
rows=[];tick=30000
for own,body in ((11,1),(12,3),(38,0),(38,1),(62,1)):
    for neighbor in nodes[own]["connections"]:
        # Read the actual annotated outgoing direction and physical length.
        h=directions[neighbor["direction"]]
        for evidence in (False,True):
            channel=offsets.index((h-body)%4);distance=round(neighbor["length_m"]*1000)
            if distance>600:distance=600
            plans=[]
            for p in (reference,selected):
                p.clear_gate();plan=None
                if evidence:
                    for k in range(4):p.observe(own,body,tick+k*20,[2000]*4,[25]*4,15,0)
                for k in range(5):
                    d=[2000]*4;d[channel]=distance
                    r=p.observe(own,body,tick+80+k*20,d,[25]*4,15,0)
                    if r["plan"]:plan=r["plan"]
                assert plan is not None,(own,body,h,evidence)
                plans.append(plan)
            ignored={"rollouts","model_steps","firmware_version","surviving_models","risk_band_ms","recovery_runway_ms"}
            assert {k:v for k,v in plans[0].items() if k not in ignored}=={k:v for k,v in plans[1].items() if k not in ignored},(own,body,h,evidence,plans)
            rows.append(dict(node=own,body=body,threat=h,evidence=evidence,
                full_models=plans[0]["rollouts"],pruned_models=plans[1]["rollouts"],goal=plans[1]["goal"]))
            tick+=1000
(HERE/"legacy_mode_result.json").write_text(json.dumps({"cases":len(rows),"all_decisions_identical":True,"rows":rows},indent=2)+"\n")
print(len(rows),"16I/16J-legacy decisions identical",flush=True)
