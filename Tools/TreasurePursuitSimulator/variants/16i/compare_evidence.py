"""Ablation: same rollout catalog/models, remove only clear-ray entry inference."""
import json,copy
from simulate_16i import physics,MultiExitGame,build,FIRMWARE,HERE,base
cfg,model=physics.load_config();track=base.TrackMap();rows=[]
elf,hashes=build(("THIEF_ENTRY_EVIDENCE=0",),"no_entry")
for factor in (1.,1.1,1.2):
    cc=copy.deepcopy(cfg);cc["police_speed_m_s"]*=factor
    game=MultiExitGame(track,cc,elf,model,seed=7);r=game.run()
    rows.append(dict(factor=factor,duration_s=r["duration_s"],winner=r["winner"],goals=[e.get("goal") for e in game.planner_events]))
    print("no entry",factor,r["duration_s"],flush=True)
(HERE/"entry_ablation.json").write_text(json.dumps(dict(defines=["THIEF_ENTRY_EVIDENCE=0"],runs=rows,source_hashes=hashes,scope="Development seeds, not validation of absence-of-detection under real miss/noise."),indent=2)+"\n")
