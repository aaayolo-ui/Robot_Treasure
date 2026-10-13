"""Generate deterministic forward-only map routes and STM32 ROM tables."""

import argparse
import hashlib
import json
from collections import deque
from pathlib import Path

ROOT = Path(__file__).resolve().parent
MAP = ROOT.parent / "TreasurePursuitSimulator" / "map_annotations.json"
PROJECT = ROOT.parents[1] / "STM32/Keil_Project/Gray12_MapRoute_Calibration_Test"
HEADINGS = ("N", "E", "S", "W")


def direction(a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    if bool(dx) == bool(dy):
        raise ValueError("Map connection must be a nonzero straight line")
    return "E" if dx > 0 else "W" if dx < 0 else "S" if dy > 0 else "N"


def action(incoming, outgoing):
    return ("STRAIGHT", "RIGHT", "UTURN", "LEFT")[(HEADINGS.index(outgoing) - HEADINGS.index(incoming)) % 4]


def build_plan(annotations, max_steps=18):
    if not 1 <= max_steps <= 64:
        raise ValueError("max_steps must be between 1 and 64")
    nodes = {n["id"]: n for n in annotations["nodes"]}
    graph = {n: [] for n in nodes}
    for segment in annotations["segments"]:
        for start, end in ((segment["start"], segment["end"]), (segment["end"], segment["start"])):
            graph[start].append((end, segment["id"], segment["length_m"]))
    landmarks = {n for n, data in nodes.items() if data["is_track_landmark"]}
    legs = {}
    for start in sorted(landmarks):
        for neighbor, sid, length in sorted(graph[start]):
            chain = [{"id": sid, "from": start, "to": neighbor, "length_m": length}]
            previous, current = start, neighbor
            while current not in landmarks:
                options = [entry for entry in graph[current] if entry[0] != previous]
                if len(options) != 1:
                    raise ValueError("Unobservable point is not a straight subdivision")
                target, sid, length = options[0]
                if direction(nodes[previous]["xy_grid"], nodes[current]["xy_grid"]) != direction(nodes[current]["xy_grid"], nodes[target]["xy_grid"]):
                    raise ValueError("Unobservable point cannot hide a corner")
                chain.append({"id": sid, "from": current, "to": target, "length_m": length})
                previous, current = current, target
            legs[start, current] = dict(from_node=start, to_node=current,
                                       heading=direction(nodes[start]["xy_grid"], nodes[current]["xy_grid"]),
                                       length_m=round(sum(s["length_m"] for s in chain), 6), segments=chain)
    outgoing = {n: sorted(v for u, v in legs if u == n) for n in landmarks}
    remaining = set(legs)

    def next_path(current, previous):
        queue = deque([(current, previous, [])])
        seen = {(current, previous)}
        while queue:
            u, back, path = queue.popleft()
            candidates = [v for v in outgoing[u] if v != back]
            candidates.sort(key=lambda v: ((u, v) not in remaining, v))
            for v in candidates:
                extended = path + [(u, v)]
                if (u, v) in remaining:
                    return extended
                state = (v, u)
                if state not in seen:
                    seen.add(state)
                    queue.append((v, u, extended))
        return []

    routes = []
    first_start = next(n for n in landmarks if "police_spawn" in nodes[n]["types"])
    while remaining:
        if not routes:
            first = (first_start, outgoing[first_start][0])
        else:
            first = min(remaining, key=lambda pair: (nodes[pair[0]]["degree"] != 1, pair))
        path = [first]
        remaining.discard(first)
        while len(path) < max_steps:
            back, current = path[-1]
            if nodes[current]["degree"] == 1:
                break
            extension = next_path(current, back)
            if not extension:
                break
            for edge in extension[:max_steps - len(path)]:
                path.append(edge)
                remaining.discard(edge)
        steps = []
        for index, edge in enumerate(path):
            leg = dict(legs[edge])
            target = nodes[leg["to_node"]]
            leg["index"] = index
            leg["arrival_action"] = action(leg["heading"], legs[path[index + 1]]["heading"]) if index + 1 < len(path) else "STOP"
            leg["target_kind"] = target["geometry_type"]
            ports = 0
            for v in outgoing[edge[1]]:
                relative = action(leg["heading"], legs[edge[1], v]["heading"])
                ports |= {"LEFT": 1, "RIGHT": 2, "STRAIGHT": 4, "UTURN": 0}[relative]
            leg["expected_ports"] = ports
            if leg["arrival_action"] == "UTURN":
                raise ValueError("Forward-only route contains a U-turn")
            steps.append(leg)
        routes.append(dict(id=f"R{len(routes) + 1:03}", start_node=path[0][0],
                           start_heading=steps[0]["heading"], end_node=path[-1][1], steps=steps))
    ordered_nodes = sorted(annotations["nodes"], key=lambda n: n['id'])
    ordered_segments = sorted(annotations["segments"], key=lambda s: s['id'])
    canonical = json.dumps(dict(nodes=ordered_nodes, segments=ordered_segments, routes=routes), sort_keys=True, separators=(",", ":"))
    digest = hashlib.sha256(canonical.encode()).hexdigest()
    return dict(schema_version=2, map_route_signature=int(digest[:8], 16) & 0x7FFFFFFF,
                source_sha256=digest, mode="forward_only_batches", calibrated=annotations["calibrated"],
                node_count=len(nodes), segment_count=len(annotations["segments"]),
                directed_leg_count=len(legs), planned_directed_segment_count=sum(len(v["segments"]) for v in legs.values()),
                nodes=ordered_nodes, segments=ordered_segments, routes=routes,
                note="Known manually placed start, gray landmark sequence and odometry sanity checks; not absolute localization. N005 is traversed inside a combined straight leg.")


HEADER = '''#ifndef MAP_ROUTE_PLAN_H
#define MAP_ROUTE_PLAN_H
#include <stdint.h>
typedef enum { MAP_ACTION_LEFT=-1, MAP_ACTION_STRAIGHT=0, MAP_ACTION_RIGHT=1,
               MAP_ACTION_STOP=2, MAP_ACTION_ERROR=3 } MapRouteAction_t;
typedef struct { uint16_t id; int16_t x_half, y_half; uint8_t kind, degree; } MapRouteNode_t;
typedef struct { uint16_t id, start, end, length_mm; uint8_t heading; } MapRouteSegment_t;
typedef struct { uint16_t from_node, to_node, length_mm, segment_offset, segment_count;
                 uint8_t heading, expected_ports, target_kind; int8_t action; } MapRouteStep_t;
typedef struct { uint16_t id, offset, count, start_node; uint8_t heading; } MapRoute_t;
extern const MapRouteNode_t map_route_nodes[];
extern const MapRouteSegment_t map_route_segments[];
extern const uint16_t map_route_step_segments[];
extern const MapRouteStep_t map_route_steps[];
extern const MapRoute_t map_routes[];
extern const uint16_t map_route_count;
extern const uint32_t map_route_signature;
#define MAP_ROUTE_NODE_COUNT __NODE_COUNT__U
#define MAP_ROUTE_SEGMENT_COUNT __SEGMENT_COUNT__U
#endif
'''


def write_tables(plan, project):
    kinds = {"corner": 1, "t_junction": 2, "cross": 3, "endpoint": 4, "straight_special": 0}
    numeric = lambda sid: int(sid[1:])
    content = ['/* Generated by build_routes.py. Do not renumber N/S entries. */', '#include "MapRoutePlan.h"']
    content.append('const MapRouteNode_t map_route_nodes[] = {')
    for n in plan["nodes"]:
        x, y = n["xy_grid"]
        content.append(f'  {{{numeric(n["id"])}, {round(x*2)}, {round(y*2)}, {kinds[n["geometry_type"]]}, {n["degree"]}}},')
    content.append('};\nconst MapRouteSegment_t map_route_segments[] = {')
    for s in plan["segments"]:
        content.append(f'  {{{numeric(s["id"])}, {numeric(s["start"])}, {numeric(s["end"])}, {round(s["length_m"]*1000)}, {HEADINGS.index(s["direction_from_start"])}}},')
    content.append('};\nconst uint16_t map_route_step_segments[] = {')
    all_steps = [s for r in plan["routes"] for s in r["steps"]]
    segment_ids = [numeric(s["id"]) for step in all_steps for s in step["segments"]]
    for offset in range(0, len(segment_ids), 16):
        content.append('  ' + ', '.join(map(str, segment_ids[offset:offset+16])) + ',')
    content.append('};\nconst MapRouteStep_t map_route_steps[] = {')
    offset = 0
    for s in all_steps:
        act = {"LEFT": -1, "STRAIGHT": 0, "RIGHT": 1, "STOP": 2, "REVERSE": 4}[s["arrival_action"]]
        count = len(s["segments"])
        mode = f', {s["drive_mode"]}' if plan['schema_version'] >= 3 else ''
        content.append(f'  {{{numeric(s["from_node"])}, {numeric(s["to_node"])}, {round(s["length_m"]*1000)}, {offset}, {count}, {HEADINGS.index(s["heading"])}, {s["expected_ports"]}, {kinds[s["target_kind"]]}, {act}{mode}}},')
        offset += count
    content.append('};\nconst MapRoute_t map_routes[] = {')
    offset = 0
    for r in plan["routes"]:
        count = len(r["steps"])
        content.append(f'  {{{numeric(r["id"])}, {offset}, {count}, {numeric(r["start_node"])}, {HEADINGS.index(r["start_heading"])}}},')
        offset += count
    content.append(f'}};\nconst uint16_t map_route_count = {len(plan["routes"])}U;\nconst uint32_t map_route_signature = {plan["map_route_signature"]}UL;\n')
    destination = project / "Algorithm/MapRoute"
    destination.mkdir(parents=True, exist_ok=True)
    (destination / "MapRoutePlan.c").write_text('\n'.join(content), encoding="utf-8")
    header = HEADER
    if plan['schema_version'] >= 3:
        header = header.replace('MAP_ACTION_ERROR=3', 'MAP_ACTION_ERROR=3, MAP_ACTION_REVERSE=4')
        header = header.replace('int8_t action;', 'int8_t action; uint8_t drive_mode;')
    (destination / "MapRoutePlan.h").write_text(header.replace('__NODE_COUNT__',str(plan['node_count'])).replace('__SEGMENT_COUNT__',str(plan['segment_count'])), encoding="utf-8")


def route_instructions(plan):
    nodes = {n['id']:n for n in plan['nodes']}
    words = {'STRAIGHT':'直行','LEFT':'左转','RIGHT':'右转','STOP':'停车'}
    lines = ['# 地图自动测试路线', '', '地图 N/S 编号与现有回放一致。N=向上、E=向右、S=向下、W=向左；先按已知起点和朝向摆车。短按 KEY2 开始；停止时长按 KEY2 或短按 WK_UP 选择下一条，KEY1 选择上一条。运行中 KEY1/WK_UP/KEY2 停车。', '',
             f'共 {len(plan["routes"])} 条前进路线，规划覆盖 106 条路段的两个方向。不执行掉头或倒车；每条路线结束后由用户重新摆车。尚未实车验证。', '',
             '距离为规则地图名义尺寸。N005 位于连续直道内，其两侧路段共用一次直道测量，不生成假的灰度事件。', '']
    for route in plan['routes']:
        xy = nodes[route['start_node']]['xy_grid']
        lines += [f'## {route["id"]}', '', f'起点 **{route["start_node"]}**，格坐标 {xy}，车头朝 **{route["start_heading"]}**。终点 {route["end_node"]}。', '',
                  '| 步骤 | 从节点 | 到节点 | 路段 | 名义长度/m | 到达动作 |', '| --- | --- | --- | --- | --- | --- |']
        for step in route['steps']:
            segments=' + '.join(s['id'] for s in step['segments'])
            lines.append(f'| {step["index"]} | {step["from_node"]} | {step["to_node"]} | {segments} | {step["length_m"]:.2f} | {words[step["arrival_action"]]} |')
        lines.append('')
    return '\n'.join(lines)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map', type=Path, default=MAP)
    parser.add_argument('--project', type=Path, default=PROJECT)
    parser.add_argument('--output', type=Path, default=ROOT / 'map_routes.json')
    parser.add_argument('--max-steps', type=int, default=18)
    args = parser.parse_args()
    plan = build_plan(json.loads(args.map.read_text(encoding='utf-8-sig')), args.max_steps)
    write_tables(plan, args.project)
    args.output.write_text(json.dumps(plan, ensure_ascii=False, indent=2), encoding='utf-8')
    args.output.with_suffix('.md').write_text(route_instructions(plan),encoding='utf-8')
    print(f'{len(plan["routes"])} routes; {plan["planned_directed_segment_count"]} directed segments; signature={plan["map_route_signature"]}')


if __name__ == '__main__':
    main()
