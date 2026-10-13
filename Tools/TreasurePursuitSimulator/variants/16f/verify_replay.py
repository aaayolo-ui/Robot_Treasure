"""Replay geometry, immutable baselines, C source hashes and topology audit."""
from pathlib import Path
import hashlib,json,math,sys
HERE=Path(__file__).resolve().parent
REPO=HERE.parents[3]
from body_geometry import Body,Pose,rectangle,separation
report=json.loads((HERE/'report.json').read_text(encoding='utf-8'))
bodies={k:Body(**v) for k,v in report['body_model']['bodies'].items()}
tol=report['body_model']['collision']['contact_tolerance_m']
audit={'baselines_unchanged':True,'baseline_files':0,'c_hashes_verified':0,'cases':[],
       'browser':False,'hardware':False}
for name,digest in json.loads((HERE/'BASELINE_SHA256.json').read_text(encoding='utf-8')).items():
    assert hashlib.sha256((REPO/name).read_bytes()).hexdigest()==digest,name
    audit['baseline_files']+=1
for name,digest in report['source_hashes'].items():
    assert hashlib.sha256((REPO/name).read_bytes()).hexdigest()==digest,name
    audit['c_hashes_verified']+=1
old=json.loads((HERE.with_name('16e')/'report.json').read_text(encoding='utf-8'))
assert report['annotations']==old['annotations']
assert report['map']==old['map']
assert len(report['annotations']['nodes'])==88
assert len(report['annotations']['segments'])==106
audit.update(nodes=88,segments=106,stable_numbering=True)
space=report['map']['grid_spacing_m']
for match in report['replays']:
    frames=match['frames'];earlier=[]
    for i,f in enumerate(frames):
        assert len(f)==38
        assert all(math.isfinite(f[n]) for n in (0,2,3,9,10,33,34,37))
        if i:assert f[0]>frames[i-1][0]
        p,b=Pose(f[2]*space,f[3]*space,f[33]),Pose(f[9]*space,f[10]*space,f[34])
        gap=separation(bodies['P'],p,bodies['B'],b)
        assert abs(gap-f[37])<1e-8
        if i<len(frames)-1:
            assert gap>tol-1e-8,(match['label'],f[0],gap)
            earlier.append(gap)
        # Replay times/polygons use the same precision, not visual markers.
    assert match['collision'] is not None
    assert match['winner']=='police'
    assert match['end_reason'].startswith('capture_')
    assert frames[-1][37]<=tol+1e-8
    assert abs(frames[-1][0]-match['collision']['time_s'])<1e-8
    assert abs(match['duration_s']-match['collision']['time_s'])<1e-10
    for role,offset,angle,key in [('P',2,33,'police_polygon_m'),('B',9,34,'thief_polygon_m')]:
        f=frames[-1];poly=rectangle(bodies[role],Pose(f[offset]*space,f[offset+1]*space,f[angle]))
        assert max(math.dist(a,b) for a,b in zip(poly,match['collision'][key]))<1e-8
    captures=[e for e in match['events'] if e['event']=='capture']
    assert len(captures)==1 and captures[0]['geometry']=='oriented_rectangles'
    if match['geometry_demo']:
        assert not match['planner_events']
        f=frames[0]
        p=Pose(f[2]*space,f[3]*space,f[33])
        b_end=Pose(f[9]*space,f[10]*space,0)
        assert separation(bodies['P'],p,bodies['B'],b_end)>0
        assert frames[-1][36]=='pivot'
    else:
        assert all(e['kind']!='fault' for e in match['planner_events'])
        if not match['fixture']:
            first=match['planner_events'][0]['plan']
            assert first['goal']==23
    audit['cases'].append({'label':match['label'],'duration_s':match['duration_s'],
        'frames':len(frames),'end_geometry_gap_m':frames[-1][37],
        'no_earlier_sample_overlap':True,'terminal_body_contact':True,
        'c_planner_fault':False,'geometry_demo':match['geometry_demo']})
audit['scope']='Offline geometry/strategy replay validation, not browser or physical vehicle verification.'
(HERE/'validation_result.json').write_text(json.dumps(audit,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
if hasattr(sys.stdout,'reconfigure'):sys.stdout.reconfigure(encoding='utf-8')
print(json.dumps({k:v for k,v in audit.items() if k!='cases'},ensure_ascii=False))
for case in audit['cases']:print(case['label'],round(case['duration_s'],6),'body contact verified')
