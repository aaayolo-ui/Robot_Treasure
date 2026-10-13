"""Independent map, route length/cost and reproducibility audit."""
import json,hashlib,subprocess,sys
from arm_adapter import FIRMWARE,HERE
sys.path.insert(0,str(FIRMWARE/"Tests"))
from generate_topology import load_map
nodes,g=load_map();c=json.loads((FIRMWARE/"Tests/rollout_catalog.json").read_text())
poses={e["node"]:e for e in json.loads((FIRMWARE/"Tests/hide_catalog.json").read_text())}
p=c["parameters"];times=0
for r in c["routes"]:
    path=r["path"];assert 1<len(path)<=28 and len(set(path))==len(path)
    assert path[0] in poses and path[-1] in poses and path[0]!=path[-1]
    assert poses[path[-1]]["parking_axis_mask"]&(1<<r["body"])
    h=r["first"];total=p["THIEF_STARTUP_MS"];speed=p["THIEF_REVERSE_MM_S"] if r["mode"] else p["THIEF_FORWARD_MM_S"]
    for i,(a,b) in enumerate(zip(path,path[1:])):
        mm,out=next((mm,out) for n,mm,out in g[a] if n==b)
        assert (out-h)%4!=2
        if nodes[a][2]==0:assert out==h
        total+=(mm*1000+speed-1)//speed
        if i and (out-h)%2:total+=p["THIEF_TURN_EXTRA_MS"]
        h=out;times+=1
    assert total==r["time_ms"]<=18000
    assert r["body"]==(h+2*r["mode"])%4
header=FIRMWARE/"Algorithm/ThiefStrategy/ThiefRolloutData.h"
catalog=FIRMWARE/"Tests/rollout_catalog.json"
before={str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in (header,catalog)}
subprocess.run([sys.executable,str(FIRMWARE/"Tests/generate_rollout_data.py")],check=True)
assert before=={str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in (header,catalog)}
audit=dict(routes=len(c["routes"]),legs_checked=times,deterministic=True,table_bytes=c["table_bytes"],
           max_candidates_per_root=max(e["candidates"] for e in c["enumeration"]),enumeration_capped=any(e["enumeration_capped"] for e in c["enumeration"]))
(HERE/"catalog_validation.json").write_text(json.dumps(audit,indent=2)+"\n")
print(audit)
