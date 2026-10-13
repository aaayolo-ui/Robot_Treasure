"""Validate topology, executable poses, actual C hashes and car contact."""
from pathlib import Path
import hashlib,json,math,sys,xml.etree.ElementTree as ET
HERE=Path(__file__).resolve().parent;REPO=HERE.parents[3]
sys.path.insert(0,str(HERE.with_name('16f')))
from body_geometry import Body,Pose,separation
FW=REPO/'STM32/Keil_Project/Thief16I_RolloutMinimax_FourLaser_Test'
R=json.loads((HERE/'report.json').read_text(encoding='utf-8'))
old=json.loads((HERE.with_name('16f')/'report.json').read_text(encoding='utf-8'))
assert R['map']==old['map'] and R['annotations']==old['annotations']
assert R['body_model']==old['body_model']
baseline=json.loads((HERE/'BASELINE_SHA256.json').read_text(encoding='utf-8'))
for name,digest in baseline.items():assert hashlib.sha256((REPO/name).read_bytes()).hexdigest()==digest,name
for name,digest in R['source_hashes'].items():assert hashlib.sha256((REPO/name).read_bytes()).hexdigest()==digest,name
for mapping in R['comparison']['source_hashes'].values():
    for name,digest in mapping.items():assert hashlib.sha256((REPO/name).read_bytes()).hexdigest()==digest,name
assert len(R['comparison']['runs'])==18
assert all(r['plan_failures']==0 and r['winner']=='police' for r in R['comparison']['runs'])
assert R['config']['sensor_range_m']==.6 and R['config']['police_sensor_range_m']==.6
nodes={int(n['id'][1:]):n for n in R['annotations']['nodes']}
edges={frozenset((int(s['start'][1:]),int(s['end'][1:]))) for s in R['annotations']['segments']}
catalog={n['node']:n for n in R['hide_catalog']}
assert set(catalog)=={11,12,38,62}
for n,v in catalog.items():
    assert nodes[n]['type'] in ('cross','t_junction')
    assert v['escape_exits']>=3 and len(v['independent_branch_targets'])==v['escape_exits']
    assert n not in (8,9,18,19,26,27,28,44,76)
    for h,e in v['exits'].items():
        assert e['path'][0]==n and e['path'][-1]==e['node']
        for a,b in zip(e['path'],e['path'][1:]):assert frozenset((a,b)) in edges
    for h in range(4):
        if v['parking_axis_mask']&(1<<h):
            assert str(h) in v['exits'] and str((h+2)%4) in v['exits']
            assert v['exits'][str(h)]['turns']==0 and v['exits'][str((h+2)%4)]['turns']==0
bodies={k:Body(**v) for k,v in R['body_model']['bodies'].items()};space=R['map']['grid_spacing_m']
tol=R['body_model']['collision']['contact_tolerance_m'];cases=[]
for m in R['replays']:
    frames=m['frames']
    for i,f in enumerate(frames):
        assert len(f)==38
        if i:assert f[0]>frames[i-1][0]
        pp,bp=Pose(f[2]*space,f[3]*space,f[33]),Pose(f[9]*space,f[10]*space,f[34])
        gap=separation(bodies['P'],pp,bodies['B'],bp)
        assert abs(gap-f[37])<1e-8
        if i<len(frames)-1 or m['collision'] is None:assert gap>tol-1e-8
    if m['collision']:
        assert m['winner']=='police' and frames[-1][37]<=tol+1e-8
        assert abs(m['duration_s']-m['collision']['time_s'])<1e-8
        assert len([e for e in m['events'] if e['event']=='capture'])==1
    for e in m['planner_events']:
        assert e['kind']!='fault'
        p=e['plan'];n=int(e['goal'][1:]);h=(p['step_heading'][-1]+2*p['step_mode'][-1])%4
        assert n in catalog and p['escape_exits']>=3
        assert p['firmware_version']==1609
        assert p['parking_heading']==h and catalog[n]['parking_axis_mask']&(1<<h)
        if e['kind']=='initial':assert n==38
        else:
            assert p['algorithm']==1 and p['candidates']>1
            assert p['candidates']<=67 and p['candidates']<=p['rollouts']<=p['candidates']*6
            assert p['forecast_contact_ms']<=p['travel_ms']+340
            if not p['forecast_clear']:assert p['degraded'] and p['terminal_escape_ms']==0
    cases.append({'label':m['label'],'duration_s':m['duration_s'],'winner':m['winner'],
        'terminal_body_contact':bool(m['collision']),'no_earlier_overlap':True,'no_planner_fault':True})
project=FW/'MDK-ARM'/ (FW.name+'.uvprojx')
xml=ET.parse(project);paths=[x.text.replace('\\','/').lower() for x in xml.findall('.//FilePath')]
assert len(paths)==len(set(paths))
assert xml.find('.//Device').text=='STM32F103RC'
assert sum(name.endswith('/thiefrollout.c') for name in paths)==1
assert R['algorithm']['global_opponent_state'] is False
copied=json.loads((HERE/'COPIED_PROJECT_FILES.json').read_text(encoding='utf-8'))
oldfw=FW.with_name('Thief16H_TurnClearance_FourLaser_Test')
unchanged=[]
for new,prior in copied.items():
    if new.startswith(('Hardware/','Control/','Core/','Drivers/','System/')):
        assert (FW/new).read_bytes()==(oldfw/prior).read_bytes(),new
        unchanged.append(new)
log=(FW/'MDK-ARM/build16i.log').read_text(encoding='utf-8',errors='replace')
assert '0 Error(s), 0 Warning(s)' in log
alg=json.loads((FW/'Tests/build/result.json').read_text())
assert alg['test_failures']==0 and alg['test_initial_goal']==38
audit={'baseline_files_unchanged':len(baseline),'c_source_hashes_verified':len(R['source_hashes']),
    'nodes':88,'segments':106,'candidate_hides':len(catalog),'map_and_car_physics_unchanged':True,
    'hardware_control_core_vendor_system_files_unchanged':len(unchanged),
    'duplicate_project_sources':False,'keil_errors':0,'keil_warnings':0,'actual_c_checks':alg['test_checks'],
    'algorithm_workspace_bytes':alg['test_workspace_bytes'],'comparison_runs':18,'cases':cases,
    'browser':False,'hardware':False,'status':'仅编译通过，尚未完成硬件验证',
    'scope':'Offline Cortex-M3 algorithm, geometry and page script checks; no physical validation.'}
(HERE/'validation_result.json').write_text(json.dumps(audit,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
if hasattr(sys.stdout,'reconfigure'):sys.stdout.reconfigure(encoding='utf-8')
print(json.dumps({k:v for k,v in audit.items() if k!='cases'},ensure_ascii=False))
for c in cases:print(c['label'],round(c['duration_s'],6),c['winner'])
