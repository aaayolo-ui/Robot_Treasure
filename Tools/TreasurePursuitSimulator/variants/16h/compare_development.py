"""Small semantic ablation; no changes to sensor/vehicle physics or older code."""
import copy,json,statistics
from pathlib import Path
from simulate_16h import physics,base,MultiExitGame,FIRMWARE,build
HERE=Path(__file__).resolve().parent
cfg,model=physics.load_config();track=base.TrackMap()
variants=[('16G',FIRMWARE.with_name('Thief16G_MultiExitHide_FourLaser_Test'),()),
          ('clearance',FIRMWARE,()),
          ('without_away',FIRMWARE,('THIEF_AWAY_TURN_PENALTY=0',)),
          ('turn_bound_500',FIRMWARE,('THIEF_TURN_REACH_MODEL=1','THIEF_POLICE_MIN_TURN_MS=500')),
          ('short_horizon',FIRMWARE,('THIEF_RISK_HORIZON_MS=3000',)),
          ('without_pause_window',FIRMWARE,('THIEF_TURN_SAFETY=0',))]
rows=[];hashes={}
for label,fw,defines in variants:
    elf,hashes[label]=build(defines,'dev_'+label,fw)
    for factor in (1.,1.2):
        cc=copy.deepcopy(cfg);cc['police_speed_m_s']*=factor
        game=MultiExitGame(track,cc,elf,model,seed=7);result=game.run()
        f=result['frames'][-1]
        row={'variant':label,'factor':factor,'seed':7,'duration_s':result['duration_s'],
             'winner':result['winner'],'end_stages':[f[35],f[36]],
             'goals':[e.get('goal') for e in game.planner_events],
             'faults':sum(e['kind']=='fault' for e in game.planner_events)}
        rows.append(row);print(json.dumps(row),flush=True)
data={'runs':rows,'source_hashes':hashes,'body_model':model,'scope':'Development seed 7; identical body/sensors/motion/police. Not hardware evidence.'}
(HERE/'development_comparison.json').write_text(json.dumps(data,indent=2)+'\n',encoding='utf-8')
