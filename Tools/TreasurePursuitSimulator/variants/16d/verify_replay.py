"""Validate replay/model boundaries offline; not a browser interaction test."""
from pathlib import Path
import hashlib,json,math
ROOT=Path(__file__).resolve().parent
REPO=ROOT.parents[3]
d=json.loads((ROOT/'report.json').read_text(encoding='utf-8'))
for name,h in d['source_hashes'].items():
    assert hashlib.sha256((REPO/name).read_bytes()).hexdigest()==h,name
assert len(d['annotations']['nodes'])==88 and len(d['annotations']['segments'])==106
assert len(d['replays'])==4
for m in d['replays']:
    assert m['frames'] and all(len(f)==33 for f in m['frames'])
    for f in m['frames'][:-1]:
        assert math.dist(f[2:4],f[9:11])*.3>=.18-1e-5
    for f in m['frames']:
        assert not(f[29] and f[12] in (1,2,6,7)),(m['label'],f[0])
    for event in m['planner_events']:
        if event.get('plan'):
            assert event['plan']['states']<=704
            assert all(n in d['annotations']['original_node_mapping'] for n in event['original_route'])
    if not m['fixture']:
        p=m['planner_events'][0]['plan']
        assert p['to'][:10]==[53,44,36,35,34,33,25,15,6,4]
        assert p['action'][:9]==[1,0,-1,0,0,1,0,0,-1]
        assert '(5.5,0)' in m['planner_events'][0]['original_route']
    end=m['frames'][-1]
    assert abs(math.dist(end[2:4],end[9:11])*.3-.18)<1e-4
assert [e['goal'] for e in d['replays'][3]['planner_events'] if e['kind']=='escape']==['N063','N044']
html=(ROOT/'replay.html').read_text(encoding='utf-8')
for name in ('show-nodes','show-segments','timeline','hide-jump','decision-jump','firmware'):
    assert f'id="{name}"' in html,name
print('PASS: unchanged C source, fixed treasure prefix, annotations, sensor isolation and continuous capture ending')
