"""Generate fixed map topology metadata offline; no third-party packages."""
from pathlib import Path
import heapq, re

ROOT=Path(__file__).resolve().parents[1]
def load_map():
    text=(ROOT/'Algorithm/MapRoute/MapRoutePlan.c').read_text(encoding='utf-8')
    nb=text.split('const MapRouteNode_t map_route_nodes[] = {')[1].split('};')[0]
    eb=text.split('const MapRouteSegment_t map_route_segments[] = {')[1].split('};')[0]
    nodes={r[0]:r[1:] for r in (tuple(map(int,m)) for m in re.findall(r'\{(\d+),\s*(-?\d+),\s*(-?\d+),\s*(\d+),\s*(\d+)\}',nb))}
    graph={n:[] for n in nodes}
    for _,a,b,mm,h in (tuple(map(int,m)) for m in re.findall(r'\{(\d+),\s*(\d+),\s*(\d+),\s*(\d+),\s*(\d+)\}',eb)):
        graph[a].append((b,mm,h));graph[b].append((a,mm,(h+2)%4))
    return nodes,graph

def components(graph,removed):
    left=set(graph)-{removed}; result=[]
    while left:
        group=set(); pending=[min(left)]
        while pending:
            n=pending.pop()
            if n not in left:continue
            left.remove(n);group.add(n)
            pending.extend(a for a,_,_ in graph[n] if a in left)
        result.append(group)
    return result

def distances(graph,start):
    dist={start:0}; queue=[(0,start)]
    while queue:
        cost,n=heapq.heappop(queue)
        if cost!=dist[n]:continue
        for a,mm,_ in graph[n]:
            v=cost+mm
            if v<dist.get(a,10**9):dist[a]=v;heapq.heappush(queue,(v,a))
    return dist

def metadata():
    nodes,g=load_map(); core=set(g)
    while True:
        leaves={n for n in core if sum(a in core for a,_,_ in g[n])<2}
        if not leaves:break
        core-=leaves
    # Removing each gateway finds single-entry regions even if they contain a loop.
    # Keep the largest component as the main map; blacklist the detached regions
    # and their gateway. Conservative offline hide screening, not all graph games.
    pockets=set()
    for gate in g:
        groups=components(g,gate)
        if len(groups)<2:continue
        main=max(groups,key=lambda c:(len(c),-min(c)))
        detached=set().union(*(c for c in groups if c is not main))
        if detached & core:pockets|=detached|{gate}
    def span(n):
        def ray(h):
            cur=n;seen={n};total=0
            while True:
                edge=next((e for e in g[cur] if e[2]==h),None)
                if not edge or edge[0] in seen:return total
                cur,mm,_=edge;total+=mm;seen.add(cur)
        return max(ray(0)+ray(2),ray(1)+ray(3))
    all_dist={n:distances(g,n) for n in g}
    branches={n for n in core-pockets if nodes[n][3]>=3}
    candidates={n for n in core-pockets if nodes[n][2] in (1,2,3)
                and span(n)<=900 and sum(a in core-pockets for a,_,_ in g[n])>=2}
    parent={}; queue=[(0,81)]; best={81:0}
    while queue:
        cost,n=heapq.heappop(queue)
        if cost!=best[n]:continue
        for a,mm,_ in g[n]:
            if cost+mm<best.get(a,10**9):best[a]=cost+mm;parent[a]=n;heapq.heappush(queue,(cost+mm,a))
    approach=[5]
    while approach[-1]!=81:approach.append(parent[approach[-1]])
    approach=set(approach)
    cfg=(ROOT/'Algorithm/ThiefStrategy/ThiefStrategyConfig.h').read_text()
    def parameter(name):
        return int(re.search(r'#define\s+'+name+r'\s+(\d+)',cfg).group(1))
    police_speed=parameter('THIEF_POLICE_MAX_MM_S')
    turn_ms=parameter('THIEF_TURN_EXTRA_MS')
    # Offline time prior includes heading and turns. This is a search-quality
    # prior, not knowledge of the opponent's chosen path. Risk uses the separate
    # conservative reach model without opponent turn delays.
    pq=[(0,81,0,0)];pt={(81,0,0):0}
    while pq:
        cost,n,h,mode=heapq.heappop(pq)
        if cost!=pt[n,h,mode]:continue
        for a,mm,nh in g[n]:
            if nodes[n][2]==0 and h!=nh:continue
            delta=(nh-h)%4
            next_mode=mode^(delta==2)
            extra=0 if delta==0 else (parameter('THIEF_SWITCH_TO_REV_MS' if next_mode else 'THIEF_SWITCH_TO_FWD_MS') if delta==2 else turn_ms)
            value=cost+(mm*1000+police_speed-1)//police_speed+extra
            if value<pt.get((a,nh,next_mode),10**9):pt[a,nh,next_mode]=value;heapq.heappush(pq,(value,a,nh,next_mode))
    police_time={n:min(v for (a,h,mode),v in pt.items() if a==n) for n in g}
    def detectable(a,b):
        if a==b:return True
        ax,ay=nodes[a][:2];bx,by=nodes[b][:2]
        if ax!=bx and ay!=by:return False
        h=1 if bx>ax else 3 if bx<ax else 2 if by>ay else 0
        cur=a;seen={a};distance=0
        while distance<=600:
            edge=next((e for e in g[cur] if e[2]==h),None)
            if not edge:return False
            cur,mm,_=edge;distance+=mm
            if distance>600:return False
            if cur==b:return True
            if cur in seen:return False
            seen.add(cur)
        return False
    quality=[];safe=[];prior=[];parking=[]
    for n in sorted(g):
        nearby=sum(all_dist[n][b]<=600 for b in branches)
        # Greater police detour and nearby junction count improves hide quality.
        separation=min(all_dist[n][p] for p in approach)
        exits={h for a,_,h in g[n] if a in core-pockets}
        axes=sum(1<<h for h in exits if (h+2)%4 in exits)
        # The current chassis starts escape forward/reverse, without a new
        # stationary corner turn. Both directions on the parking axis must be
        # genuine cycle exits; a dead branch must not count as a second exit.
        parking.append(axes)
        safe.append(int(n in candidates and axes!=0))
        prior.append(min(60000,min(police_time[a] for a in g if detectable(a,n))))
        quality.append(min(6000,separation*2+nearby*150+nodes[n][3]*80))
    return safe,quality,prior,parking,pockets

def main():
    safe,quality,prior,parking,pockets=metadata()
    out=['/* Generated offline from MapRoutePlan.c. Do not edit by hand. */',
         '#ifndef THIEF_MAP_TOPOLOGY_H','#define THIEF_MAP_TOPOLOGY_H','#include <stdint.h>']
    for typ,name,values in [('uint8_t','thief_hide_safe',safe),('uint8_t','thief_parking_axis_mask',parking),('uint16_t','thief_hide_quality',quality),('uint16_t','thief_police_prior_ms',prior)]:
        out.append('static const '+typ+' '+name+'[88] = {')
        for i in range(0,88,16):out.append('  '+', '.join(str(v)+'U' for v in values[i:i+16])+',')
        out.append('};')
    out.append('#endif')
    (ROOT/'Algorithm/ThiefStrategy/ThiefMapTopology.h').write_text('\n'.join(out)+'\n',encoding='utf-8')
    print('Safe hides:',[i+1 for i,v in enumerate(safe) if v])
    print('Single-entry exclusions:',sorted(pockets))
if __name__=='__main__':main()
