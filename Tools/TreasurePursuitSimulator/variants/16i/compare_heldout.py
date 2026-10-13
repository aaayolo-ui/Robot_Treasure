"""Frozen C implementations, identical body referee; extra seeds/speed ratios."""
import copy,json,statistics,sys,hashlib
from simulate_16i import physics,MultiExitGame,build,FIRMWARE,HERE,base
cfg,model=physics.load_config();track=base.TrackMap();runs=[];hashes={}
variants=[('16H',FIRMWARE.with_name('Thief16H_TurnClearance_FourLaser_Test')),
          ('16I',FIRMWARE)]
if '--refresh-selected' in sys.argv:
    prior=json.loads((HERE/'heldout_comparison.json').read_text(encoding='utf-8'))
    runs=[r for r in prior['runs'] if r['variant']!='16I']
    hashes={k:v for k,v in prior['source_hashes'].items() if k!='16I'}
    for mapping in hashes.values():
        for name,digest in mapping.items():
            assert hashlib.sha256((HERE.parents[3]/name).read_bytes()).hexdigest()==digest,name
for name,root in variants:
    if '--refresh-selected' in sys.argv and name!='16I':continue
    elf,hashes[name]=build((),name+'_heldout',root)
    for factor in (1.,1.1,1.2):
        for seed in (10,11,12):
            cc=copy.deepcopy(cfg);cc['police_speed_m_s']*=factor
            game=MultiExitGame(track,cc,elf,model,seed=seed);r=game.run()
            f=r['frames'][-1]
            row={'variant':name,'factor':factor,'seed':seed,'duration_s':r['duration_s'],
                 'winner':r['winner'],'end_stages':[f[35],f[36]],
                 'plan_failures':sum(e['kind']=='fault' for e in game.planner_events),
                 'goals':[e.get('goal') for e in game.planner_events],
                 'max_model_steps':max((e.get('plan',{}).get('model_steps',0) for e in game.planner_events),default=0),
                 'max_candidates':max((e.get('plan',{}).get('candidates',0) for e in game.planner_events),default=0),
                 'arrived_hides':sum(e['event']=='firmware_hide' for e in game.events)}
            runs.append(row);print(name,factor,seed,round(r['duration_s'],6),flush=True)
summary=[]
for name,_ in variants:
    for factor in (1.,1.1,1.2):
        rr=[r for r in runs if r['variant']==name and r['factor']==factor]
        summary.append({'variant':name,'factor':factor,'runs':len(rr),
                        'mean_s':statistics.mean(r['duration_s'] for r in rr),
                        'min_s':min(r['duration_s'] for r in rr),'max_s':max(r['duration_s'] for r in rr),
                        'thief_wins':sum(r['winner']=='thief' for r in rr),
                        'plan_failures':sum(r['plan_failures'] for r in rr)})
(HERE/'heldout_comparison.json').write_text(json.dumps({'seeds':[10,11,12],'summary':summary,
    'runs':runs,'source_hashes':hashes,'body_model':model,
    'scope':'Same finite sensors, police and body contact. Miss randomness differs by seed; deterministic trajectories can coincide. No hardware evidence.'},indent=2)+'\n',encoding='utf-8')
print(json.dumps(summary),flush=True)
