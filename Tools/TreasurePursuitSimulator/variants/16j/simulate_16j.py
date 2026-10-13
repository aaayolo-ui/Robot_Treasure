"""Bounded maximin/leximin C rollouts; unchanged continuous 16F car physics."""
from pathlib import Path
import copy,importlib.util,json,sys
HERE=Path(__file__).resolve().parent
REPO=HERE.parents[3]
sys.path.insert(0,str(HERE))
from arm_adapter import build,FIRMWARE
spec=importlib.util.spec_from_file_location('physics_baseline_16f',HERE.with_name('16f')/'simulate_16f.py')
physics=importlib.util.module_from_spec(spec);spec.loader.exec_module(physics)
base=physics.base
parameter=physics.parameter
CATALOG=json.loads((FIRMWARE/'Tests/hide_catalog.json').read_text(encoding='utf-8'))
CAT={n['node']:n for n in CATALOG}


class MultiExitGame(physics.BodyGame):
    def install(self,plan,kind):
        super().install(plan,kind)
        if plan.get('firmware_version')!=1610:return
        if not plan.get('escape_exits'):return
        assert base.DIRECTIONS[plan['parking_heading']]==self.hide_facing
        assert plan['escape_exits']>=3
        catalog=CAT[plan['goal']]
        assert catalog['parking_axis_mask']&(1<<plan['parking_heading'])
        self.planner_events[-1]['source']='16J ARM C'
        self.hide_decisions[-1].update(source='16J ARM C',escape_exits=plan['escape_exits'],
            parking_heading=plan['parking_heading'],exit_branch_ms=plan['exit_branch_ms'])


def main():
    cfg,model=physics.load_config();elf,hashes=build();track=base.TrackMap()
    labels=['完整开局 · 多出口藏匿 · 双方同速','完整开局 · 多出口藏匿 · 警方快20%',
        '不利停车对照 · N038同轴近距追击','朝向对照 · N038纵向停车 · 右侧来车',
        '不利停车对照 · N062后方来车后的强制拐弯',
        '换线专项 · N038横向停车 · 上方两格来车',
        '换线专项 · N038横向停车 · 上方来车快20%']
    cases=[(1,None),(1.2,None),(1,(38,'W',40,'W')),(1,(38,'N',40,'W')),
           (1,(62,'E',34,'S')),(1,(38,'E',21,'S')),(1.2,(38,'E',21,'S'))]
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
        'turn_safety':{'contact_envelope_mm':parameter('THIEF_CONTACT_MM'),
                       'clear_mm':parameter('THIEF_TURN_CLEAR_MM'),
                       'horizon_ms':parameter('THIEF_ROLLOUT_MAX_MS')},
        'experiment_status':'Independent bounded rollout experiment; 16I and earlier versions retained. Original adverse fixtures are unchanged.',
        'scope':'Actual 16J ARM C planner; unchanged 16F continuous bodies/referee; ideal localization/limited sensors; no hardware proof.'}
    if (HERE/'heldout_comparison.json').exists():data['comparison']=json.loads((HERE/'heldout_comparison.json').read_text(encoding='utf-8'))
    old=json.loads((HERE.with_name('16i')/'report.json').read_text(encoding='utf-8'))
    assert len(old['replays'])==len(replays)
    data['fixture_comparison']=[{'label':new['label'],'old_duration_s':before['duration_s'],
        'new_duration_s':new['duration_s'],'old_winner':before['winner'],'new_winner':new['winner']}
        for before,new in zip(old['replays'],replays)]
    data['algorithm']={'name':'bounded rollout + epsilon risk bands + hypothesis coverage + next-corner runway + exact pruning',
        'risk_band_ms':parameter('THIEF_ROLLOUT_TIME_BUCKET_MS'),'models':6,'terminal_reaction_ms':parameter('THIEF_ROLLOUT_HOLD_MS'),
        'catalog_routes':579,'max_candidates_per_root':67,'global_opponent_state':False,
        'motion_integration':'committed phase endpoint events; interval visibility and swept body hull contact'}
    (HERE/'report.json').write_text(json.dumps(data,ensure_ascii=False,separators=(',',':')),encoding='utf-8')
    (HERE/'map_annotations.json').write_text(json.dumps(physics.ANNOTATIONS,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    template=(HERE/'replay.template.html').read_text(encoding='utf-8')
    payload=json.dumps(data,ensure_ascii=False,separators=(',',':')).replace('</','<\\/')
    (HERE/'replay.html').write_text(template.replace('__REPORT_DATA__',payload).replace('__ANNOTATION_SCRIPT__',
        (HERE/'map_annotations.js').read_text(encoding='utf-8')),encoding='utf-8')


if __name__=='__main__':
    if hasattr(sys.stdout,'reconfigure'):sys.stdout.reconfigure(encoding='utf-8')
    main()
