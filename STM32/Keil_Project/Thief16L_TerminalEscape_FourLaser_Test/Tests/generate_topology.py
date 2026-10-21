"""Generate fixed map topology metadata offline; no third-party packages."""
from pathlib import Path
import heapq, re, math
import json

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
    def ray(n,h):
        cur=n;seen={n};total=0
        while True:
            edge=next((e for e in g[cur] if e[2]==h),None)
            if not edge or edge[0] in seen:return total
            cur,mm,_=edge;total+=mm;seen.add(cur)
    def span(n):
        return max(ray(n,0)+ray(n,2),ray(n,1)+ray(n,3))
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
    quality=[];safe=[];prior=[];parking=[];reaction=[];junction_safe=[]
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
        # Trial alternative: a four-way junction has multiple real cycle exits
        # even if one entire axis is long. Still exclude single-entry pockets.
        # This never admits a degree-two straight road as a hiding location.
        junction_safe.append(int(n in core-pockets and nodes[n][2]==3 and
                                 len(exits)>=3 and axes!=0))
        # Local warning budget for an already stopped vehicle. Perpendicular
        # arrivals are harder: closest approach uses both speeds and a contact
        # radius. This is an offline quality heuristic, not an evasion proof.
        # Include confirmation, startup, planning budget and timing uncertainty.
        budgets=[]
        for incoming in range(4):
            warning=min(600,ray(n,incoming))
            if not warning:continue
            delay=parameter('THIEF_STARTUP_MS')+parameter('THIEF_PLAN_BUDGET_MS')+parameter('THIEF_TIMING_GUARD_MS')
            delay+=(1 if warning<=400 else 4)*parameter('THIEF_CONFIRM_FRAME_MS')
            options=[]
            for leave in range(4):
                if not axes & (1<<leave) or leave==incoming:continue
                perpendicular=(leave-incoming)%2
                speed=parameter('THIEF_FORWARD_MM_S')
                radius=parameter('THIEF_CONTACT_MM')*(math.hypot(speed,police_speed)/speed if perpendicular else 1)
                options.append(round((warning-radius)*1000/police_speed-delay))
            budgets.append(max(options) if options else -3000)
        reaction.append(max(-3000,min(budgets)) if budgets else -3000)
        prior.append(min(60000,min(police_time[a] for a in g if detectable(a,n))))
        quality.append(min(6000,separation*2+nearby*150+nodes[n][3]*80))
    return safe,quality,prior,parking,pockets,reaction,junction_safe

def external_graph_metadata(pockets):
    """Main-map road domain and first useful branch, retaining validated gate26.
    Leaf corridors and detached single-entry interiors are not escape routes.
    This is not a deletion of raw roads: MapRoutePlan remains unchanged.
    """
    nodes,g=load_map();core=set(g)
    while True:
        leaves={n for n in core if sum(a in core for a,_,_ in g[n])<2}
        if not leaves:break
        core-=leaves
    allowed=core-pockets
    # This experiment requests N026 specifically. Its classification is still
    # checked from map components instead of assuming a four-way means safety.
    for gate in (26,):
        groups=components(g,gate)
        if gate not in core or nodes[gate][2] not in (2,3) or len(groups)<2:continue
        main=max(groups,key=lambda c:(len(c),-min(c)))
        if sum(a in main for a,_,_ in g[gate])>=3:allowed.add(gate)
    branches={n for n in allowed if sum(a in allowed for a,_,_ in g[n])>=3}
    cfg=(ROOT/'Algorithm/ThiefStrategy/ThiefStrategyConfig.h').read_text(encoding='utf-8')
    value=lambda name:int(re.search(r'#define\s+'+name+r'\s+(\d+)',cfg).group(1))
    speed=min(value('THIEF_FORWARD_MM_S'),value('THIEF_REVERSE_MM_S'))
    def trace(n,h):
        if n not in allowed:return None
        edge=next((e for e in g[n] if e[2]==h and e[0] in allowed),None)
        if edge is None:return None
        prev=n;cur,mm,nh=edge;path=[n,cur];cost=(mm*1000+speed-1)//speed
        seen={n,cur};heading=nh;distance=mm;turns=0
        while cur not in branches:
            choices=[e for e in g[cur] if e[0] in allowed and e[0]!=prev]
            if len(choices)!=1:return None
            nxt,mm,nh=choices[0]
            if nxt in seen:return None
            delta=(nh-heading)%4
            if nodes[cur][2]==0 and delta!=0:return None
            cost+=(mm*1000+speed-1)//speed+(value('THIEF_TURN_EXTRA_MS') if delta in (1,3) else 0)
            distance+=mm;turns+=int(delta in (1,3))
            seen.add(nxt);path.append(nxt);prev,cur,heading=cur,nxt,nh
        return {'node':cur,'time_ms':cost+value('THIEF_STARTUP_MS'),
                'length_mm':distance,'turns':turns,'path':path}
    details={n:{h:v for h in range(4) if (v:=trace(n,h)) is not None} for n in sorted(g)}
    return allowed,details


def multi_exit_metadata(pockets):
    nodes,g=load_map()
    core=set(g)
    while True:
        leaves={n for n in core if sum(a in core for a,_,_ in g[n])<2}
        if not leaves:break
        core-=leaves
    allowed=core-pockets
    degree={n:sum(a in allowed for a,_,_ in g[n]) for n in g}
    branches={n for n in allowed if degree[n]>=3}
    cfg=(ROOT/'Algorithm/ThiefStrategy/ThiefStrategyConfig.h').read_text(encoding='utf-8')
    value=lambda name:int(re.search(r'#define\s+'+name+r'\s+(\d+)',cfg).group(1))
    speed=min(value('THIEF_FORWARD_MM_S'),value('THIEF_REVERSE_MM_S'))
    turn_ms=value('THIEF_TURN_EXTRA_MS')
    # Public map prior: a pursuer has reached the treasure facing west.
    # Predict its cheapest approach to each hide; this is not a sensor sighting.
    pq=[(0,5,3,0)];pt={(5,3,0):0}
    while pq:
        cost,n,h,mode=heapq.heappop(pq)
        if cost!=pt[n,h,mode]:continue
        for a,mm,nh in g[n]:
            if nodes[n][2]==0 and nh!=h:continue
            delta=(nh-h)%4;next_mode=mode^(delta==2)
            extra=turn_ms if delta in (1,3) else (value('THIEF_SWITCH_TO_REV_MS' if next_mode else 'THIEF_SWITCH_TO_FWD_MS') if delta==2 else 0)
            v=cost+(mm*1000+value('THIEF_POLICE_MAX_MM_S')-1)//value('THIEF_POLICE_MAX_MM_S')+extra
            if v<pt.get((a,nh,next_mode),10**9):pt[a,nh,next_mode]=v;heapq.heappush(pq,(v,a,nh,next_mode))
    expected_entry=[min((v,h) for (a,h,m),v in pt.items() if a==n)[1] for n in sorted(g)]
    def ray(n,h):
        cur=n;seen={n};total=0
        while True:
            edge=next((e for e in g[cur] if e[2]==h),None)
            if not edge or edge[0] in seen:return total
            cur,mm,_=edge;total+=mm;seen.add(cur)
    def first_branch(n,h):
        edge=next((e for e in g[n] if e[2]==h and e[0] in allowed),None)
        if edge is None:return None
        prev=n;cur,mm,nh=edge;path=[n,cur];cost=(mm*1000+speed-1)//speed
        seen={n,cur};heading=nh;distance=mm;turns=0
        while cur not in branches:
            choices=[e for e in g[cur] if e[0] in allowed and e[0]!=prev]
            if len(choices)!=1:return None
            nxt,mm,nh=choices[0]
            if nxt in seen:return None
            delta=(nh-heading)%4
            if nodes[cur][2]==0 and delta!=0:return None
            cost+=(mm*1000+speed-1)//speed+(turn_ms if delta in (1,3) else 0)
            distance+=mm;turns+=int(delta in (1,3))
            seen.add(nxt);path.append(nxt);prev,cur,heading=cur,nxt,nh
        return {'node':cur,'time_ms':cost+value('THIEF_STARTUP_MS'),'length_mm':distance,'turns':turns,'path':path}
    external_nodes,external_exits=external_graph_metadata(pockets)
    safe=[];parking=[];counts=[];times=[];catalog=[]
    for n in sorted(g):
        exits={h:first_branch(n,h) for h in range(4)}
        exits={h:v for h,v in exits.items() if v is not None}
        distinct={v['node'] for v in exits.values()}
        axis=0
        for h in exits:
            # Nearby intersections may lie on a long overall straight road.
            # Judge the two actual escape corridors, not its total ray span.
            if ((h+2)%4 in exits and exits[h]['turns']==0 and
                exits[(h+2)%4]['turns']==0 and ray(n,h)+ray(n,(h+2)%4)<=1200):
                axis|=1<<h
        legal=n in branches and nodes[n][2] in (2,3) and len(distinct)>=3 and axis!=0
        conditional=False
        if n==26 and not legal:
            main_exits=external_exits[n]
            fast={v['node'] for v in main_exits.values() if v['time_ms']<=2000 and v['turns']<=1}
            main_targets={v['node'] for v in main_exits.values()}
            main_axis=sum(1<<h for h in main_exits if (h+2)%4 in main_exits)
            # Gate26 can only become an explicitly conditional stop. Two fast
            # main exits plus a third slower main route are not a normal hide.
            conditional=(n in external_nodes and nodes[n][2] in (2,3) and
                         len(main_targets)>=3 and len(fast)>=2 and
                         any(v['time_ms']>2000 for v in main_exits.values()) and main_axis!=0)
            if conditional:
                exits=main_exits;distinct=main_targets;axis=main_axis;legal=True
        safe.append(int(legal));parking.append(axis if legal else 0)
        counts.append(len(distinct) if legal else 0)
        times.append([min(65535,exits[h]['time_ms']) if legal and h in exits else 65535 for h in range(4)])
        if legal:catalog.append({'node':n,'xy_mm':[nodes[n][0]*150,nodes[n][1]*150],
            'exits':{str(h):v for h,v in exits.items()},'independent_branch_targets':sorted(distinct),
            'escape_exits':len(distinct),'parking_axis_mask':axis,'expected_police_entry_heading':expected_entry[n-1]})
        if conditional:
            groups=components(g,n);main=max(groups,key=lambda c:(len(c),-min(c)))
            detached=sorted(set().union(*(c for c in groups if c is not main)))
            catalog[-1].update(conditional=True,normal_refuge=False,refuge_role='conditional_escape_stop',
                              requires_terminal_validation=True,excluded_single_entry_nodes=detached,
                              fast_main_exit_count=len(fast),external_gateway=n)
    # A nearby branch is useful, but a forced corner immediately after it can
    # still trap a fleeing car. Cache straight distance until a required turn.
    runway=[]
    for n in sorted(g):
        row=[]
        for h in range(4):
            cur=n;seen={n};distance=0
            while True:
                edge=next((e for e in g[cur] if e[2]==h and e[0] in allowed),None)
                if not edge or edge[0] in seen:break
                cur,mm,_=edge;seen.add(cur);distance+=mm
            row.append(distance)
        runway.append(row)
    for item in catalog:item['straight_runway_mm']=runway[item['node']-1]
    return safe,parking,counts,times,catalog,expected_entry,runway

def main():
    safe,quality,prior,parking,pockets,reaction,junction_safe=metadata()
    safe,parking,counts,times,catalog,expected_entry,runway=multi_exit_metadata(pockets)
    junction_safe=[0]*88
    out=['/* Generated offline from MapRoutePlan.c. Do not edit by hand. */',
         '#ifndef THIEF_MAP_TOPOLOGY_H','#define THIEF_MAP_TOPOLOGY_H','#include <stdint.h>']
    for typ,name,values in [('uint8_t','thief_hide_safe',safe),('uint8_t','thief_parking_axis_mask',parking),('uint16_t','thief_hide_quality',quality),('uint16_t','thief_police_prior_ms',prior),('int16_t','thief_hide_reaction_ms',reaction),('uint8_t','thief_junction_hide_safe',junction_safe)]:
        out.append('static const '+typ+' '+name+'[88] = {')
        for i in range(0,88,16):out.append('  '+', '.join(str(v)+('' if typ=='int16_t' else 'U') for v in values[i:i+16])+',')
        out.append('};')
    out.append('static const uint8_t thief_exit_count[88] = {')
    for i in range(0,88,16):out.append('  '+', '.join(str(v)+'U' for v in counts[i:i+16])+',')
    out.append('};')
    out.append('static const uint16_t thief_exit_branch_ms[88][4] = {')
    for row in times:out.append('  {'+', '.join(str(v)+'U' for v in row)+'},')
    out.append('};')
    out.append('static const uint8_t thief_initial_police_entry[88] = {')
    for i in range(0,88,16):out.append('  '+', '.join(str(v)+'U' for v in expected_entry[i:i+16])+',')
    out.append('};')
    out.append('static const uint16_t thief_straight_runway_mm[88][4] = {')
    for row in runway:out.append('  {'+', '.join(str(v)+'U' for v in row)+'},')
    out.append('};')
    external_nodes,external_exits=external_graph_metadata(pockets)
    out.append('static const uint8_t thief_external_node[88] = {')
    for i in range(0,88,16):
        out.append('  '+', '.join(str(int(n in external_nodes))+'U' for n in range(i+1,min(89,i+17)))+',')
    out.append('};')
    out.append('static const uint8_t thief_external_branch[88][4] = {')
    for n in range(1,89):
        out.append('  {'+', '.join(str(external_exits[n].get(h,{}).get('node',0))+'U' for h in range(4))+'},')
    out.append('};')
    out.append('#endif')
    (ROOT/'Algorithm/ThiefStrategy/ThiefMapTopology.h').write_text('\n'.join(out)+'\n',encoding='utf-8')
    print('Standard hide candidates:',[r['node'] for r in catalog if not r.get('conditional')])
    print('Conditional escape-stop candidates:',[r['node'] for r in catalog if r.get('conditional')])
    print('Single-entry region/gateway screening:',sorted(pockets),'N026 has an explicit conditional exception')
    (ROOT/'Tests/hide_catalog.json').write_text(json.dumps(catalog,indent=2)+'\n',encoding='utf-8')
if __name__=='__main__':main()
