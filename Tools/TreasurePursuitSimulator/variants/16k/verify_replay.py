"""Check actual C provenance, unchanged map/physics and first body contact."""
from pathlib import Path
import hashlib,json,sys
HERE=Path(__file__).resolve().parent;REPO=HERE.parents[3]
sys.path.insert(0,str(HERE.with_name('16f')))
from body_geometry import Body,Pose,separation
R=json.loads((HERE/'report.json').read_text(encoding='utf-8'))
old=json.loads((HERE.with_name('16j')/'report.json').read_text(encoding='utf-8'))
assert R['map']==old['map'] and R['annotations']==old['annotations']
assert R['body_model']==old['body_model'] and R['config']==old['config']
assert len(R['replays'])==7 and len(R['comparison']['runs'])==18
assert R['comparison']['source_hashes']['16K']==R['source_hashes']
for mapping in [R['source_hashes'],*R['comparison']['source_hashes'].values()]:
    for path,digest in mapping.items():assert hashlib.sha256((REPO/path).read_bytes()).hexdigest()==digest,path
assert R['config']['sensor_range_m']==R['config']['police_sensor_range_m']==.6
assert R['algorithm']['global_opponent_state'] is False
assert R['algorithm']['catalog_routes']==686
assert len(R['annotations']['nodes'])==88 and len(R['annotations']['segments'])==106
edges={frozenset((int(s['start'][1:]),int(s['end'][1:]))) for s in R['annotations']['segments']}
roads={}; headings={'N':0,'E':1,'S':2,'W':3}
for e in R['annotations']['segments']:
    a,b=int(e['start'][1:]),int(e['end'][1:]);mm=round(e['length_m']*1000)
    roads[a,headings[e['direction_from_start']]]=b,mm
    roads[b,headings[e['direction_from_end']]]=a,mm
catalog={r['node']:r for r in R['hide_catalog']}
assert set(catalog)=={11,12,38,62}
refuges={r['node']:r for r in R['refuge_access']['candidates']}
assert refuges[38]['tier']==1 and refuges[38]['fast_exit_count']==4
assert refuges[62]['tier']==0 and refuges[62]['fast_exit_count']==2
bodies={k:Body(**v) for k,v in R['body_model']['bodies'].items()}
space=R['map']['grid_spacing_m'];tol=R['body_model']['collision']['contact_tolerance_m']
cases=[];frames_checked=0;plans_checked=0
for m,before in zip(R['replays'],old['replays']):
    assert m['fixture']==before['fixture'] and m['geometry_demo']==before['geometry_demo']
    for i,f in enumerate(m['frames']):
        assert len(f)==38
        if i:assert f[0]>m['frames'][i-1][0]
        gap=separation(bodies['P'],Pose(f[2]*space,f[3]*space,f[33]),
                       bodies['B'],Pose(f[9]*space,f[10]*space,f[34]))
        assert abs(gap-f[37])<1e-8
        if i<len(m['frames'])-1 or not m['collision']:assert gap>tol-1e-8
        frames_checked+=1
    if m['collision']:
        assert m['winner']=='police' and m['frames'][-1][37]<=tol+1e-8
        assert abs(m['duration_s']-m['collision']['time_s'])<1e-8
        assert sum(e['event']=='capture' for e in m['events'])==1
    for event in m['planner_events']:
        assert event['kind']!='fault'
        p=event['plan'];goal=p['goal'];h=(p['step_heading'][-1]+2*p['step_mode'][-1])%4
        assert p['firmware_version']==1611 and goal in catalog
        assert p['parking_heading']==h and catalog[goal]['parking_axis_mask']&(1<<h)
        assert p['refuge_class']==refuges[goal]['tier']
        assert p['fast_escape_exits']==refuges[goal]['fast_exit_count']
        path=[p['from'][0],*p['to']]
        # Controller steps may merge a task-marker node on a straight line
        # (e.g. N006→N005→N004). Validate every original segment and length.
        for a,b,h,mm in zip(p['from'],p['to'],p['step_heading'],p['length']):
            at=a;length=0
            for _ in range(106):
                if at==b:break
                nxt,part=roads[at,h];assert frozenset((at,nxt)) in edges
                at=nxt;length+=part
            assert at==b and length==mm,(a,b,h,mm,length)
        if event['kind']=='initial':assert goal==38
        else:
            assert p['algorithm']==1 and 1<p['candidates']<=67
            assert p['candidates']<=p['rollouts']<=p['candidates']*6
            assert p['forecast_contact_ms']<=p['travel_ms']+340
            assert p['forecast_clear']==(p['surviving_models']==6)
            assert len(path)<=28
            if path[0]==path[-1]:
                assert len(path)>=5 and len(set(path[:-1]))==len(path)-1
                assert goal==38 and p['forecast_clear'] and p['terminal_escape_ms']>460
            else:assert len(path)==len(set(path))
            if not p['forecast_clear']:
                assert p['degraded'] and p['terminal_escape_ms']==0
                assert p['risk_band_ms']<=p['forecast_contact_ms']<p['risk_band_ms']+120
        plans_checked+=1
    assert m['winner']==before['winner'] and abs(m['duration_s']-before['duration_s'])<1e-8
    cases.append({'label':m['label'],'duration_s':m['duration_s'],'winner':m['winner'],
                  'no_earlier_overlap':True,'no_planner_fault':True,'unchanged_16j_outcome':True})
assert all(abs(r['duration_delta_s'])<1e-8 for r in R['comparison']['paired_runs'])
fw=REPO/'STM32/Keil_Project/Thief16K_RefugeAccess_FourLaser_Test'
assert '0 Error(s), 0 Warning(s)' in (fw/'MDK-ARM/build16k.log').read_text(errors='replace')
tests=json.loads((fw/'VALIDATION_16K_TESTS.json').read_text(encoding='utf-8'))
integration=json.loads((fw/'VALIDATION_16K_INTEGRATION.json').read_text(encoding='utf-8'))
assert integration['passed']
result={'passed':True,'frames':frames_checked,'plans':plans_checked,'nodes':88,'segments':106,
        'source_files':len(R['source_hashes']),'same_body_collision_referee':True,
        'earliest_contact_terminates_match':True,'comparison_pairs':9,
        'unchanged_comparison_outcomes':True,'no_claimed_n062_reduction':True,
        'algorithm_tests':tests,'cases':cases,'hardware':False,'browser':False,
        'status':'仅编译通过，尚未完成硬件验证'}
(HERE/'validation_result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
if hasattr(sys.stdout,'reconfigure'):sys.stdout.reconfigure(encoding='utf-8')
print(json.dumps({k:v for k,v in result.items() if k not in ('algorithm_tests','cases')},ensure_ascii=False))
