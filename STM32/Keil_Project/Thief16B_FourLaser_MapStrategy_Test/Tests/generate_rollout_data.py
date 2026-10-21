"""Offline bounded loop-free alternative routes and heading-aware travel tables.
All arrays go into Flash; no workstation or network is needed on the car.
"""
from pathlib import Path
import heapq,json,re,hashlib
from generate_topology import load_map
ROOT=Path(__file__).resolve().parents[1]
nodes,g=load_map();catalog=json.loads((ROOT/'Tests/hide_catalog.json').read_text())
poses={r['node']:[h for h in range(4) if r['parking_axis_mask']&(1<<h)] for r in catalog}
N=88;S=N*8;INF=65535
config=(ROOT/'Algorithm/ThiefStrategy/ThiefStrategyConfig.h').read_text()
names=('THIEF_FORWARD_MM_S','THIEF_REVERSE_MM_S','THIEF_POLICE_MAX_MM_S',
       'THIEF_TURN_EXTRA_MS','THIEF_ROLLOUT_POLICE_TURN_MS','THIEF_STARTUP_MS',
       'THIEF_SWITCH_TO_REV_MS','THIEF_SWITCH_TO_FWD_MS')
parameters={n:int(re.search(r'#define\s+'+n+r'\s+(\d+)',config).group(1)) for n in names}
F=parameters['THIEF_FORWARD_MM_S'];R=parameters['THIEF_REVERSE_MM_S']
P=parameters['THIEF_POLICE_MAX_MM_S'];TURN=parameters['THIEF_TURN_EXTRA_MS']
PTURN=parameters['THIEF_ROLLOUT_POLICE_TURN_MS'];START=parameters['THIEF_STARTUP_MS']
TO_R=parameters['THIEF_SWITCH_TO_REV_MS'];TO_F=parameters['THIEF_SWITCH_TO_FWD_MS']
def state(n,h,m):return (n-1)*8+h*2+m
def leg(mm,m,delta,speed=P):
    speed=speed*R//F if m else speed
    return (mm*1000+speed-1)//speed+(PTURN if delta%2 else (TO_R if m else TO_F) if delta==2 else 0)

# Reverse Dijkstra: travel time to every target, from every heading/mode.
reverse=[[] for _ in range(S)]
for n in g:
    for h in range(4):
        for m in range(2):
            u=state(n,h,m)
            for a,mm,out in g[n]:
                d=(out-h)%4
                if nodes[n][2]==0 and d:continue
                mode=m^(d==2);v=state(a,out,mode)
                reverse[v].append((u,leg(mm,mode,d)))
time=[]
for target in range(1,N+1):
    dist=[INF]*S;queue=[]
    for h in range(4):
        for m in range(2):
            u=state(target,h,m);dist[u]=0;heapq.heappush(queue,(0,u))
    while queue:
        cost,v=heapq.heappop(queue)
        if cost!=dist[v]:continue
        for u,dt in reverse[v]:
            x=cost+dt
            if x<dist[u]:dist[u]=x;heapq.heappush(queue,(x,u))
    assert all(dist[state(n,h,m)]<INF for n in g for h in range(4) for m in range(2)
               if nodes[n][2]!=0 or any(out==h for _,_,out in g[n]))
    time.append(dist)

routes=[];offsets=[];audit=[]
def corridor_signature(path,first):
    """Retain alternatives whose first two pivots use different junctions."""
    h=first;signature=[]
    for n,a in zip(path[1:],path[2:]):
        out=next(out for b,_,out in g[n] if b==a)
        if (out-h)%2:
            signature.append((n,out))
            if len(signature)==2:break
        h=out
    return tuple(signature)
for origin in range(1,N+1):
    for body in range(4):
        offsets.append(len(routes))
        if origin not in poses:continue
        queue=[];groups={};expanded=0
        for a,mm,h in g[origin]:
            delta=(h-body)%4
            if delta%2:continue
            m=int(delta==2)
            speed=R if m else F
            heapq.heappush(queue,(START+(mm*1000+speed-1)//speed,(origin,a),h,m,h))
        while queue and expanded<20000:
            cost,path,h,m,first=heapq.heappop(queue);expanded+=1;n=path[-1]
            if cost>18000 or len(path)>28:continue
            final=(h+2*m)%4;key=(n,final,first)
            signatures=groups.setdefault(key,[]) if n in poses else []
            signature=corridor_signature(path,first) if n in poses else ()
            if (n!=origin and n in poses and final in poses[n] and cost>800 and
                    len(signatures)<6 and (len(signatures)<2 or signature not in signatures)):
                signatures.append(signature)
                routes.append({'path':path,'time_ms':cost,'body':final,'mode':m,'first':first,
                               'corridor_signature':signature})
            for a,mm,out in g[n]:
                if a in path:continue
                d=(out-h)%4
                if nodes[n][2]==0 and d:continue
                assert d!=2
                speed=R if m else F
                dt=(mm*1000+speed-1)//speed+(TURN if d%2 else 0)
                heapq.heappush(queue,(cost+dt,path+(a,),out,m,first))
        audit.append({'node':origin,'body':body,'expanded':expanded,'candidates':len(routes)-offsets[-1],
                      'enumeration_capped':bool(queue)})
offsets.append(len(routes))
flat=[];records=[]
for r in routes:
    records.append((len(flat),r['time_ms'],len(r['path']),r['mode'],r['body'],r['first']))
    flat.extend(r['path'])
out=['/* Generated from fixed map and motion parameters; do not edit. */',
     '#ifndef THIEF_ROLLOUT_DATA_H','#define THIEF_ROLLOUT_DATA_H','#include <stdint.h>',
     '#include "ThiefStrategyConfig.h"']
for name,value in parameters.items():
    out += [f'#if {name} != {value}UL',
            '#error Regenerate ThiefRolloutData.h with Tests/generate_rollout_data.py', '#endif']
out += [
     'typedef struct {uint16_t offset,time_ms;uint8_t count,mode,body,first;} RolloutRoute_t;',
     f'#define ROLLOUT_ROUTE_COUNT {len(records)}U',
     'static const uint16_t rollout_start[353] = {']
for i in range(0,len(offsets),16):out.append(','.join(str(x)+'U' for x in offsets[i:i+16])+',')
out+=['};',f'static const uint8_t rollout_nodes[{len(flat)}] = {{']
for i in range(0,len(flat),32):out.append(','.join(str(x)+'U' for x in flat[i:i+32])+',')
out+=['};',f'static const RolloutRoute_t rollout_routes[{len(records)}] = {{']
for r in records:out.append('{'+','.join(str(x)+'U' for x in r)+'},')
out+=['};','static const uint16_t rollout_police_ms[88][704] = {']
for row in time:
    out.append('{')
    for i in range(0,S,32):out.append(','.join(str(x)+'U' for x in row[i:i+32])+',')
    out.append('},')
out+=['};','static const uint8_t rollout_branch[88][4] = {']
items={r['node']:r for r in catalog}
for n in range(1,N+1):out.append('{'+','.join(str(items.get(n,{}).get('exits',{}).get(str(h),{}).get('node',0))+'U' for h in range(4))+'},')
out+=['};','static const uint8_t rollout_next[88][4] = {']
for n in range(1,N+1):out.append('{'+','.join(str(next((a for a,mm,h in g[n] if h==d),0))+'U' for d in range(4))+'},')
out+=['};','static const uint16_t rollout_length[88][4] = {']
for n in range(1,N+1):out.append('{'+','.join(str(next((mm for a,mm,h in g[n] if h==d),0))+'U' for d in range(4))+'},')
out+=['};','#endif']
(ROOT/'Algorithm/ThiefStrategy/ThiefRolloutData.h').write_text('\n'.join(out)+'\n',encoding='utf-8')
(ROOT/'Tests/rollout_catalog.json').write_text(json.dumps({'routes':routes,'enumeration':audit,
    'police_table_bytes':N*S*2,'route_nodes_bytes':len(flat),'route_record_bytes':len(records)*8,
    'table_bytes':N*S*2+len(flat)+len(records)*8+353*2+88*4*(1+1+2),
    'parameters':parameters,
    'map_sha256':hashlib.sha256((ROOT/'Algorithm/MapRoute/MapRoutePlan.c').read_bytes()).hexdigest(),
    'assumptions':{'police_reference_speed_mm_s':P,'police_reference_turn_ms':PTURN,'max_route_ms':18000}},indent=2)+'\n')
print('Routes',len(routes),'Flash tables',N*S*2+len(flat)+len(records)*8+353*2+88*4*(1+1+2),'max per root',max(a['candidates'] for a in audit))
