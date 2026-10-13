"""Check the frozen selected source on seeds unused for development."""
from pathlib import Path
import copy,json,statistics,sys
from simulate_16g import physics,MultiExitGame,build,FIRMWARE,HERE,base
cfg,model=physics.load_config();track=base.TrackMap();runs=[];hashes={}
for name,root in [('16F_baseline',FIRMWARE.with_name('Thief16E_VisibilityEscape_FourLaser_Test')),('selected',FIRMWARE)]:
    elf,hashes[name]=build((),name+'_heldout',root)
    for factor in (1.,1.2):
        for seed in (10,11,12):
            cc=copy.deepcopy(cfg);cc['police_speed_m_s']*=factor
            game=MultiExitGame(track,cc,elf,model,seed=seed);r=game.run()
            row={'variant':name,'factor':factor,'seed':seed,'duration_s':r['duration_s'],
                'winner':r['winner'],'end_reason':r['end_reason'],
                'initial_hide':game.planner_events[0]['goal'],
                'initial_parking_heading':game.planner_events[0]['plan']['step_heading'][-1],
                'plan_failures':sum(e['kind']=='fault' for e in game.planner_events)}
            runs.append(row);print(name,factor,seed,round(r['duration_s'],6),flush=True)
summary=[]
for name in ('16F_baseline','selected'):
    for factor in (1.,1.2):
        rows=[r for r in runs if r['variant']==name and r['factor']==factor]
        summary.append({'variant':name,'factor':factor,'runs':len(rows),
            'mean_s':statistics.mean(r['duration_s'] for r in rows),
            'min_s':min(r['duration_s'] for r in rows),'max_s':max(r['duration_s'] for r in rows),
            'thief_wins':sum(r['winner']=='thief' for r in rows),
            'plan_failures':sum(r['plan_failures'] for r in rows)})
data={'seeds':[10,11,12],'summary':summary,'runs':runs,'source_hashes':hashes,'body_model':model,
      'scope':'Same police, finite sensors and 16F body contact; frozen sources, no hardware proof.'}
(HERE/'heldout_comparison.json').write_text(json.dumps(data,indent=2)+'\n',encoding='utf-8')
print(json.dumps(summary,indent=2),flush=True)
