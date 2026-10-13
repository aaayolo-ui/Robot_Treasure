"""Joint multi-exit hide and arrival-axis selection; same 16F body physics."""
from pathlib import Path
import argparse,copy,importlib.util,json,statistics,sys
HERE=Path(__file__).resolve().parent
REPO=HERE.parents[3]
sys.path.insert(0,str(HERE))
from arm_adapter import build,FIRMWARE
spec=importlib.util.spec_from_file_location('physics_baseline_16f',HERE.with_name('16f')/'simulate_16f.py')
physics=importlib.util.module_from_spec(spec);spec.loader.exec_module(physics)
base=physics.base
CATALOG=json.loads((FIRMWARE/'Tests/hide_catalog.json').read_text(encoding='utf-8'))
CAT={n['node']:n for n in CATALOG}


class MultiExitGame(physics.BodyGame):
    def install(self,plan,kind):
        super().install(plan,kind)
        if not plan.get('escape_exits'):return
        assert base.DIRECTIONS[plan['parking_heading']]==self.hide_facing
        assert plan['escape_exits']>=3
        catalog=CAT[plan['goal']]
        assert catalog['parking_axis_mask']&(1<<plan['parking_heading'])
        self.planner_events[-1]['source']='16G ARM C'
        self.hide_decisions[-1].update(source='16G ARM C',escape_exits=plan['escape_exits'],
            parking_heading=plan['parking_heading'],exit_branch_ms=plan['exit_branch_ms'])


def benchmark(probe=False):
    cfg,model=physics.load_config();track=base.TrackMap();runs=[]
    variants=[('16F_baseline',FIRMWARE.with_name('Thief16E_VisibilityEscape_FourLaser_Test'),()),
        ('multi_exit',FIRMWARE,()),
        ('multi_exit_strong',FIRMWARE,('THIEF_HIDE_EXIT_WEIGHT=3000','THIEF_HIDE_AXIS_WEIGHT=3'))]
    if probe:
        variants=[(f'initial_N{n:03d}',FIRMWARE,(f'THIEF_INITIAL_HIDE_NODE={n}',)) for n in (12,23,38,62)]
    hashes={}
    for name,root,defines in variants:
        elf,hashes[name]=build(defines,name,root)
        for factor in (1.,1.2):
            for seed in ((7,) if probe else (7,8)):
                cc=copy.deepcopy(cfg);cc['police_speed_m_s']*=factor
                game=MultiExitGame(track,cc,elf,model,seed=seed);result=game.run()
                row={'variant':name,'factor':factor,'seed':seed,'duration_s':result['duration_s'],
                    'winner':result['winner'],'initial_hide':game.planner_events[0]['goal'],
                    'initial_parking':game.hide_facing if not game.planner_events else
                        base.DIRECTIONS[(game.planner_events[0]['plan']['step_heading'][-1]+2*game.planner_events[0]['plan']['step_mode'][-1])%4],
                    'faults':sum(e['kind']=='fault' for e in game.planner_events),
                    'goals':[e.get('goal') for e in game.planner_events],
                    'arrived_hides':sum(e['event']=='firmware_hide' for e in game.events)}
                runs.append(row);print(name,factor,seed,round(row['duration_s'],3),row['initial_hide'],row['initial_parking'],flush=True)
    summary=[]
    for name,_,_ in variants:
        for factor in (1.,1.2):
            rows=[r for r in runs if r['variant']==name and r['factor']==factor]
            summary.append({'variant':name,'factor':factor,'mean_s':statistics.mean(r['duration_s'] for r in rows),
                'thief_wins':sum(r['winner']=='thief' for r in rows),'faults':sum(r['faults'] for r in rows)})
    data={'seeds':[7,8],'runs':runs,'summary':summary,'source_hashes':hashes,'body_model':model,
        'scope':'Same continuous rectangle judge, limited sensors, police policy and speed parameters; development seeds only.'}
    (HERE/('initial_hide_probe.json' if probe else 'development_comparison.json')).write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(summary,indent=2),flush=True)


def main():
    cfg,model=physics.load_config();elf,hashes=build();track=base.TrackMap()
    labels=['完整开局 · 多出口藏匿 · 双方同速','完整开局 · 多出口藏匿 · 警方快20%',
        '朝向对照 · N038横向停车 · 右侧来车','朝向对照 · N038纵向停车 · 同一来车',
        '藏点专项 · N062横向停车 · 上方两格来车']
    cases=[(1,None),(1.2,None),(1,(38,'W',40,'W')),(1,(38,'N',40,'W')),(1,(62,'E',34,'S'))]
    replays=[];summary=[]
    for (factor,stage),label in zip(cases,labels):
        cc=copy.deepcopy(cfg);cc['police_speed_m_s']*=factor
        game=MultiExitGame(track,cc,elf,model,stage)
        if stage and stage[0]==62:
            # Same real road segment; start the referee car two cells away.
            # Its true coordinates still enter only virtual laser observations.
            p=game.robots['P'];p.node='(7,4)';p.target='(7,5)';p.progress=0.
            p.phase='move';p.facing='S';p.body_facing='S';p.angle=physics.math.pi/2
            p.move_speed=p.speed;p.remaining=.3/p.speed
        result=game.run()
        result.update(label=label,fixture=stage,geometry_demo=False,metrics=base.motion_metrics(result),
            motion_parameters=cc['measured_motion']|{'turn90_s':cc['turn90_s']})
        replays.append(result)
        summary.append({'scenario':label,'police_speed_m_s':cc['police_speed_m_s'],
            'thief_speed_m_s':cc['thief_speed_m_s'],'runs':1,'police_wins':int(result['winner']=='police'),
            'thief_wins':int(result['winner']=='thief'),'mean_duration_s':result['duration_s'],
            'mean_max_escape_straight_m':result['max_escape_straight_m']})
        print(label,result['winner'],round(result['duration_s'],6),
            [(e['kind'],e.get('goal'),e.get('plan',{}).get('parking_heading')) for e in game.planner_events],flush=True)
    data={'map':track.data,'config':cfg,'annotations':physics.ANNOTATIONS,'replays':replays,'summary':summary,
        'source_hashes':hashes,'firmware':FIRMWARE.name,'body_model':model,'hide_catalog':CATALOG,
        'experiment_status':'Independent multi-exit/heading experiment; 16F retained as baseline; not claimed better in full matches.',
        'scope':'Actual 16G ARM C planner; 16F continuous car bodies and judge; ideal localization/limited sensors; no hardware proof.'}
    if (HERE/'heldout_comparison.json').exists():data['comparison']=json.loads((HERE/'heldout_comparison.json').read_text(encoding='utf-8'))
    (HERE/'report.json').write_text(json.dumps(data,ensure_ascii=False,separators=(',',':')),encoding='utf-8')
    (HERE/'map_annotations.json').write_text(json.dumps(physics.ANNOTATIONS,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    template=(HERE/'replay.template.html').read_text(encoding='utf-8')
    payload=json.dumps(data,ensure_ascii=False,separators=(',',':')).replace('</','<\\/')
    (HERE/'replay.html').write_text(template.replace('__REPORT_DATA__',payload).replace('__ANNOTATION_SCRIPT__',
        (HERE/'map_annotations.js').read_text(encoding='utf-8')),encoding='utf-8')


if __name__=='__main__':
    if hasattr(sys.stdout,'reconfigure'):sys.stdout.reconfigure(encoding='utf-8')
    p=argparse.ArgumentParser();p.add_argument('--benchmark',action='store_true');p.add_argument('--probe',action='store_true');a=p.parse_args()
    benchmark(a.probe) if a.benchmark or a.probe else main()
