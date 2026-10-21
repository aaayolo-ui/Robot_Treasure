"""Offline invariants for the STM32 thief map planner.

This mirrors route-selection constraints only; it does not claim motor or sensor
hardware behaviour.
"""
from __future__ import annotations

import heapq
import re
from pathlib import Path

ROOT = Path(__file__).parent.parent
PLAN = ROOT / "Algorithm" / "MapRoute" / "MapRoutePlan.c"
PLANNER = ROOT / "Algorithm" / "ThiefStrategy" / "ThiefMapPlanner.c"


def load_map():
    text = PLAN.read_text(encoding="utf-8")
    node_block = text.split("const MapRouteNode_t map_route_nodes[] = {", 1)[1].split("};", 1)[0]
    seg_block = text.split("const MapRouteSegment_t map_route_segments[] = {", 1)[1].split("};", 1)[0]
    nodes = {
        row[0]: tuple(row[1:])
        for row in (
            tuple(map(int, match))
            for match in re.findall(
                r"\{(\d+),\s*(-?\d+),\s*(-?\d+),\s*(\d+),\s*(\d+)\}",
                node_block,
            )
        )
    }
    segments = [
        tuple(map(int, match))
        for match in re.findall(
            r"\{(\d+),\s*(\d+),\s*(\d+),\s*(\d+),\s*(\d+)\}",
            seg_block,
        )
    ]
    adjacency = {node: [] for node in nodes}
    for _, start, end, length, heading in segments:
        adjacency[start].append((end, length, heading))
        adjacency[end].append((start, length, (heading + 2) % 4))
    return nodes, adjacency


def shortest(adjacency, start, goal, blocked=0):
    queue = [(0, start)]
    distance = {start: 0}
    previous = {}
    while queue:
        cost, node = heapq.heappop(queue)
        if cost != distance[node]:
            continue
        if node == goal:
            break
        for nxt, length, _ in adjacency[node]:
            if nxt == blocked:
                continue
            candidate = cost + length
            if candidate < distance.get(nxt, 10**9):
                distance[nxt] = candidate
                previous[nxt] = node
                heapq.heappush(queue, (candidate, nxt))
    if goal not in distance:
        return None, None
    path = [goal]
    while path[-1] != start:
        path.append(previous[path[-1]])
    return list(reversed(path)), distance[goal]


def main():
    nodes, adjacency = load_map()
    planner = PLANNER.read_text(encoding="utf-8")
    block = planner.split("initial_treasure_path[] = {", 1)[1].split("};", 1)[0]
    prefix = [int(value) for value in re.findall(r"(\d+)U", block)]
    assert prefix == [54, 53, 44, 36, 35, 34, 33, 25, 15, 6, 5, 4]
    departure, _ = shortest(adjacency, 4, 59, 5)
    initial = prefix + departure[1:]
    edges = [next(edge for edge in adjacency[a] if edge[0] == b)
             for a, b in zip(initial, initial[1:])]
    total_mm = sum(edge[1] for edge in edges)
    turns = {initial[i]: (edges[i][2] - edges[i-1][2]) % 4
             for i in range(1, len(prefix)-1)}
    assert {n: turns[n] for n in (53, 36, 33, 6)} == {53: 1, 36: 3, 33: 1, 6: 3}
    assert all(turns[n] == 0 for n in (44, 35, 34, 25, 15, 5))
    assert initial[0] == 54 and initial[-1] == 59
    assert initial.count(5) == 1
    before = initial[initial.index(5) - 1]
    after = initial[initial.index(5) + 1]
    assert {before, after} == {4, 6}

    planner = PLANNER.read_text(encoding="utf-8")
    safe_block = planner.split("safe_hide_nodes[] = {", 1)[1].split("};", 1)[0]
    safe = [int(value) for value in re.findall(r"(\d+)U", safe_block)]
    assert all(nodes[node][2] not in (0, 4) for node in safe)

    current, previous, body_heading = 59, 54, 3
    traces = []
    for _ in range(12):
        away = (body_heading + 2) % 4
        forced = [edge for edge in adjacency[current] if edge[2] == away]
        assert forced, f"N{current:03d} has no road away from laser direction"
        forced_node, forced_mm, _ = forced[0]
        options = []
        for goal in safe:
            if goal in (current, previous):
                continue
            path, path_mm = shortest(adjacency, forced_node, goal, current)
            if not path:
                continue
            path = [current] + path
            total = forced_mm + path_mm
            if len(path) < 3 or total < 600:
                continue
            arrival = next(h for n, _, h in adjacency[path[-2]] if n == goal)
            if not any(h == arrival for _, _, h in adjacency[goal]):
                continue
            options.append((abs(total - 1500), total, goal, path, arrival))
        assert options, f"N{current:03d} has no legal escape"
        _, distance, goal, path, arrival = min(options)
        assert nodes[goal][2] != 4
        assert path[1] == forced_node
        traces.append((current, goal, distance, path))
        previous, current = current, goal
        body_heading = (arrival + 2) % 4

    print(f"initial: {total_mm} mm, {initial}")
    for index, (start, goal, distance, path) in enumerate(traces, 1):
        print(f"escape {index:02d}: N{start:03d}->N{goal:03d}, {distance} mm, {path}")
    print("PASS: treasure crossing, non-endpoint hides, and 12 escape replans")


if __name__ == "__main__":
    main()

