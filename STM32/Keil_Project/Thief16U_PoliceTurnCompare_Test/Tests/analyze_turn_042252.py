"""Derive observed total angle and alignment time, not unmeasured inertia."""
from pathlib import Path
import re,json,statistics,hashlib
root=Path(__file__).resolve().parents[1]
path=root/'Tests/before_line_handoff/小车串口_20261028_042252.txt'
lines=path.read_text(encoding='utf-8',errors='replace').splitlines()
signs={int(v) for v in re.findall(r'CW_SIGN=(-?\d+)', '\n'.join(lines))}
assert signs=={-1},signs
cw=-1;wrap=lambda x:(x+18000)%36000-18000
pending=None;done=None;rows=[]
for i,line in enumerate(lines):
    m=re.search(r'TURN_DONE=.*START=(-?\d+) CURRENT=(-?\d+) DELTA=(-?\d+)',line)
    if m:done=tuple(map(int,m.groups()))
    m=re.search(r'TURN_AXIS_BEGIN N=(\d+)',line)
    if m:pending=dict(node=int(m[1]),start=done[0],coarse=done[1],delta=done[2],line=i+1)
    if pending and line.startswith('MT3,'):
        f=line.split(',')
        if int(f[4])==7:pending['begin_ms']=int(f[3])
        if int(f[4])==8 and 'error_cw' in pending:
            pending['ready_yaw']=int(f[11]);pending['wait_ms']=int(f[3])-pending['begin_ms']
            pending['derived_target']=wrap(pending['ready_yaw']-pending['error_cw']*cw)
            direction=1 if pending['delta']>0 else -1
            pending['direction']='left' if direction*cw<0 else 'right'
            pending['additional_deg']=direction*wrap(pending['derived_target']-pending['coarse'])/100
            pending['total_deg']=direction*wrap(pending['derived_target']-pending['start'])/100
            rows.append(pending);pending=None
    m=re.search(r'TURN_AXIS_READY YAW_ERR=(-?\d+)',line)
    if m and pending:pending['error_cw']=int(m[1])
summary={}
for direction in ['left','right']:
    a=[r for r in rows if r['direction']==direction]
    summary[direction]=dict(samples=len(a),total_deg_mean=statistics.mean(r['total_deg'] for r in a),
        additional_deg_mean=statistics.mean(r['additional_deg'] for r in a),alignment_ms_mean=statistics.mean(r['wait_ms'] for r in a))
report=dict(log_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),rows=rows,summary=summary,
 mean_alignment_ms=statistics.mean(r['wait_ms'] for r in rows),
 limitation='Only4complete turns. Totals include entry skew; no first-stopped-before-pulse YAW was logged. This does not estimate pure inertia or a reliable fixed left/right motor cutoff.',
 selected_policy='User chose actual map exit direction, no mandatory +/-2degree fine trim, real gray PID handles small residuals.')
(root/'Docs/turn_042252_analysis.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps(summary,indent=2));print('mean alignment ms:',report['mean_alignment_ms'])
