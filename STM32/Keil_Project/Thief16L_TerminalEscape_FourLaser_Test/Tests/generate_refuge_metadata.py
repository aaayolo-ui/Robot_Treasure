"""Public-map refuge prior; writes analysis JSON only, never firmware tables.

The sensing-point set is nodes plus road midpoints, not a continuous optimum.
No opponent pose, intent, trace or live observations are inputs to this policy.
"""
from pathlib import Path
import argparse
import hashlib
import heapq
import json
import re


REPO = Path(__file__).resolve().parents[4]
DEFAULT_FIRMWARE = Path(__file__).resolve().parents[1]


def load_inputs(firmware):
    map_path = firmware / "Algorithm/MapRoute/MapRoutePlan.c"
    config_path = firmware / "Algorithm/ThiefStrategy/ThiefStrategyConfig.h"
    catalog_path = firmware / "Tests/hide_catalog.json"
    text = map_path.read_text(encoding="utf-8")
    node_text = text.split("const MapRouteNode_t map_route_nodes[] = {")[1].split("};")[0]
    edge_text = text.split("const MapRouteSegment_t map_route_segments[] = {")[1].split("};")[0]
    nodes = {}
    for match in re.findall(r"\{(\d+),\s*(-?\d+),\s*(-?\d+),\s*(\d+),\s*(\d+)\}", node_text):
        node, x, y, kind, degree = map(int, match)
        nodes[node] = {"x_mm": x * 150, "y_mm": y * 150,
                       "kind": kind, "degree": degree, "label": f"N{node:03d}"}
    edges = [tuple(map(int, match)) for match in
             re.findall(r"\{(\d+),\s*(\d+),\s*(\d+),\s*(\d+),\s*(\d+)\}", edge_text)]
    graph = {node: [] for node in nodes}
    for segment, a, b, mm, heading in edges:
        graph[a].append((b, mm, heading, segment))
        graph[b].append((a, mm, (heading + 2) % 4, segment))
        dx, dy = nodes[b]["x_mm"] - nodes[a]["x_mm"], nodes[b]["y_mm"] - nodes[a]["y_mm"]
        assert abs(dx) + abs(dy) == mm and bool(dx) != bool(dy), (segment, "length/axis")
        assert heading == (1 if dx > 0 else 3 if dx < 0 else 2 if dy > 0 else 0)
    for node in graph:
        assert len({edge[2] for edge in graph[node]}) == len(graph[node]), (node, "duplicate port")
        assert len(graph[node]) == nodes[node]["degree"], (node, "degree")
        graph[node].sort()
    config_text = config_path.read_text(encoding="utf-8")
    names = ["THIEF_POLICE_MAX_MM_S", "THIEF_TURN_EXTRA_MS", "THIEF_SWITCH_TO_REV_MS",
             "THIEF_SWITCH_TO_FWD_MS", "THIEF_HIDE_REACTION_MS"]
    params = {name: int(re.search(r"#define\s+" + name + r"\s+(\d+)", config_text).group(1))
              for name in names}
    catalog = json.loads(catalog_path.read_text(encoding="utf-8"))
    inputs = [{"path": str(path.relative_to(REPO)).replace("\\", "/"),
               "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}
              for path in (map_path, config_path, catalog_path)]
    return nodes, graph, edges, params, catalog, inputs


def patrol_corridors(nodes, graph, edges):
    corridors = []
    for heading in (1, 2):  # East and south enumerate each undirected chain once.
        for start in sorted(graph):
            if any(edge[2] == (heading + 2) % 4 for edge in graph[start]):
                continue
            path, segments, length = [start], [], 0
            cursor = start
            while True:
                edge = next((edge for edge in graph[cursor] if edge[2] == heading), None)
                if edge is None:
                    break
                target, mm, _, segment = edge
                assert target not in path, "A constant cardinal chain cannot cycle"
                path.append(target); segments.append(segment); length += mm; cursor = target
            if not segments:
                continue
            branches = [node for node in path if nodes[node]["degree"] >= 3]
            corridors.append({"heading": heading, "axis": "horizontal" if heading == 1 else "vertical",
                              "nodes": path, "segments": segments, "length_mm": length,
                              "branch_nodes": branches, "branch_count": len(branches),
                              "coverage_score": length * len(branches)})
    covered = [segment for corridor in corridors for segment in corridor["segments"]]
    assert sorted(covered) == sorted(edge[0] for edge in edges), "Gap/duplicate in corridor partition"
    corridors.sort(key=lambda item: (-item["coverage_score"], -item["length_mm"],
                                     -item["branch_count"], item["heading"], item["nodes"]))
    assert corridors and corridors[0]["branch_count"] > 0
    return corridors


def add_midpoints(nodes, edges):
    points = {node: dict(value) for node, value in nodes.items()}
    graph = {node: [] for node in points}
    for index, (segment, a, b, mm, heading) in enumerate(edges):
        midpoint = max(nodes) + index + 1
        assert mm % 2 == 0, "Midpoint lengths must be integral millimeters"
        points[midpoint] = {"x_mm": (nodes[a]["x_mm"] + nodes[b]["x_mm"]) // 2,
                            "y_mm": (nodes[a]["y_mm"] + nodes[b]["y_mm"]) // 2,
                            "kind": 0, "degree": 2, "label": f"S{segment:03d}@mid",
                            "virtual_midpoint": True, "segment": segment}
        graph[midpoint] = []
        for left, right in ((a, midpoint), (midpoint, b)):
            graph[left].append((right, mm // 2, heading))
            graph[right].append((left, mm // 2, (heading + 2) % 4))
    for node in graph:
        graph[node].sort()
    return points, graph


def sees(points, graph, source, target, maximum_mm):
    if source == target:
        return True
    dx = points[target]["x_mm"] - points[source]["x_mm"]
    dy = points[target]["y_mm"] - points[source]["y_mm"]
    if dx and dy:
        return False
    heading = 1 if dx > 0 else 3 if dx < 0 else 2 if dy > 0 else 0
    seen, cursor, total = {source}, source, 0
    while total <= maximum_mm:
        edge = next((edge for edge in graph[cursor] if edge[2] == heading), None)
        if edge is None:
            return False
        cursor, mm, _ = edge; total += mm
        if total > maximum_mm or cursor in seen:
            return False
        if cursor == target:
            return True
        seen.add(cursor)
    return False


def public_search(points, graph, corridor, parameters):
    speed = parameters["THIEF_POLICE_MAX_MM_S"]
    starts = {}
    parent = {}
    best = {}
    queue = []
    for node in corridor["nodes"]:
        for heading in (corridor["heading"], (corridor["heading"] + 2) % 4):
            state = (node, heading, 0)
            best[state] = (0, 0, 0, 0)  # Rational time numerator, turns, distance, switches.
            starts[state] = state
            heapq.heappush(queue, (0, 0, 0, 0, *state))
    while queue:
        units, turns, distance, switches, node, heading, mode = heapq.heappop(queue)
        state = (node, heading, mode)
        if best[state] != (units, turns, distance, switches):
            continue
        for target, mm, next_heading in graph[node]:
            # Task markers and midpoint probes do not create new turning places.
            if points[node]["kind"] == 0 and next_heading != heading:
                continue
            delta = (next_heading - heading) % 4
            next_mode = mode ^ int(delta == 2)
            quarter = int(delta in (1, 3))
            extra = parameters["THIEF_TURN_EXTRA_MS"] if quarter else (
                parameters["THIEF_SWITCH_TO_REV_MS" if next_mode else "THIEF_SWITCH_TO_FWD_MS"]
                if delta == 2 else 0)
            value = (units + mm * 1000 + extra * speed, turns + quarter,
                     distance + mm, switches + int(delta == 2))
            next_state = (target, next_heading, next_mode)
            if value < best.get(next_state, (10**30,) * 4):
                best[next_state] = value; parent[next_state] = state; starts[next_state] = starts[state]
                heapq.heappush(queue, (*value, *next_state))
    routes = {}
    for node in points:
        choices = [(value, state) for state, value in best.items() if state[0] == node]
        if not choices:
            continue
        value, state = min(choices)
        path = [state[0]]
        cursor = state
        while cursor in parent:
            cursor = parent[cursor]; path.append(cursor[0])
        units, turns, mm, switches = value
        start = starts[state]
        routes[node] = {"point": points[node]["label"], "xy_mm": [points[node]["x_mm"], points[node]["y_mm"]],
                        "time_ms": (units + speed - 1) // speed,
                        "time_rational": {"numerator": units, "denominator": speed},
                        "quarter_turns": turns, "switches": switches, "length_mm": mm,
                        "start_node": start[0], "start_heading": start[1],
                        "path": [points[item]["label"] for item in reversed(path)],
                        "arrival_travel_heading": state[1], "arrival_drive_mode": state[2]}
    return routes


def generate(firmware):
    nodes, graph, edges, params, catalog, inputs = load_inputs(firmware)
    corridors = patrol_corridors(nodes, graph, edges)
    dominant = corridors[0]
    points, expanded_graph = add_midpoints(nodes, edges)
    routes = public_search(points, expanded_graph, dominant, params)
    sensor_range_mm, fast_exit_limit_ms, fast_exit_turns = 600, 2000, 1
    candidates = []
    for hide in catalog:
        node = hide["node"]
        prior_points = [routes[point] for point in points if point in routes and
                        sees(points, expanded_graph, point, node, sensor_range_mm)]
        assert prior_points
        prior_points.sort(key=lambda item: (item["time_rational"]["numerator"], item["quarter_turns"],
                                           item["length_mm"], item["point"]))
        fast_exits = [{"heading": int(heading), **details} for heading, details in hide["exits"].items()
                      if details["time_ms"] <= fast_exit_limit_ms and details["turns"] <= fast_exit_turns]
        fast_targets = sorted({item["node"] for item in fast_exits})
        search_ms = prior_points[0]["time_ms"]
        tier = int(len(fast_targets) >= 3 and search_ms >= params["THIEF_HIDE_REACTION_MS"])
        conditional = bool(hide.get("conditional",False))
        if conditional:
            # Class2 is a role, not a better ordinal quality score than Class1.
            tier = 2
        candidates.append({"node": node, "xy_mm": hide["xy_mm"], "tier": tier,
                           "patrol_search_ms": search_ms, "fast_exit_count": len(fast_targets),
                           "fast_exit_targets": fast_targets, "fast_exits": fast_exits,
                           "all_exits": hide["exits"], "parking_axis_mask": hide["parking_axis_mask"],
                           "best_prior_point": prior_points[0], "prior_points": prior_points,
                           "reasons": {"three_fast_exits": len(fast_targets) >= 3,
                                       "patrol_search_covers_reaction_budget": search_ms >= params["THIEF_HIDE_REACTION_MS"]}})
        if conditional:
            candidates[-1].update(conditional=True,normal_refuge=False,
                refuge_role='conditional_escape_stop',requires_terminal_validation=True,
                excluded_single_entry_nodes=hide['excluded_single_entry_nodes'])
    by_node = {item["node"]: item for item in candidates}
    gap_ok = not sees(points, expanded_graph, 7, 8, 600)
    outer = [item for item in corridors if item["axis"] == "vertical" and
             all(nodes[node]["x_mm"] == 0 for node in item["nodes"])]
    assert gap_ok
    assert by_node[38]["tier"] == 1 and by_node[62]["tier"] == 0
    assert by_node[26]['tier']==2 and by_node[26]['fast_exit_count']==2
    assert by_node[26]['patrol_search_ms']==0 and by_node[26]['parking_axis_mask']==5
    assert all(item["coverage_score"] < dominant["coverage_score"] for item in outer)
    return {"schema_version": 1, "method": {
                "inputs": "Public fixed map, existing refuge exit catalog, nominal motion parameters only",
                "corridor": "Enumerate maximal connected constant-heading chains; score length_mm * count(degree >= 3)",
                "tie_break": "Descending score, length, branch count; ascending heading and node path",
                "search_prior": "Minimum heading-aware nominal travel time from any dominant-corridor node, initially facing along either corridor direction, to a visible probe",
                "probe_set": "All original nodes and segment midpoints; a discrete proxy, not the exact continuous first-detection optimum",
                "probe_gap_guarantee": "Connected cardinal rays only; coordinate alignment cannot bridge a missing road",
                "time_arithmetic": "Exact rational millimeter/speed travel plus configured action milliseconds; rounded up once at path output",
                "virtual_probe_motion": "Midpoints and task-marker nodes cannot initiate a turn or reversal",
                "tier_policy": "Class1 normal iff >=3 fast distinct exits and patrol-search prior >= reaction budget; Class0 weak; Class2 conditional escape-stop role needing live terminal validation, never an ordinal preference above Class1",
                "scope": "Structural stationary-refuge prior; does not forbid road traversal or override route collision safety",
                "limitations": "Single dominant-corridor coverage heuristic; optimistic unspecified starting patrol point; no hardware guarantee or opponent identity inference",
                "uses_global_opponent_state": False},
            "parameters": params | {"sensor_range_mm": sensor_range_mm,
                                    "fast_exit_limit_ms": fast_exit_limit_ms,
                                    "fast_exit_max_forced_turns": fast_exit_turns},
            "source_inputs": inputs, "dominant_corridor": dominant, "all_corridors": corridors,
            "candidates": candidates, "validation": {
                "original_node_count": len(nodes), "original_segment_count": len(edges),
                "corridor_segment_partition_exact": True, "map_lengths_match_coordinates": True,
                "probe_count": len(points), "gap_N007_N008_rejected": gap_ok,
                "outer_vertical_max_score": max(item["coverage_score"] for item in outer),
                "outer_long_corridor_does_not_dominate": True,
                "N038_tier": by_node[38]["tier"], "N062_tier": by_node[62]["tier"]}}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--firmware", type=Path, default=DEFAULT_FIRMWARE)
    parser.add_argument("--output", type=Path, default=Path(__file__).with_name("refuge_metadata.json"))
    arguments = parser.parse_args()
    result = generate(arguments.firmware)
    arguments.output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"output": str(arguments.output), "dominant": result["dominant_corridor"],
                      "candidates": [{key: row[key] for key in ("node", "tier", "patrol_search_ms", "fast_exit_count")}
                                     for row in result["candidates"]]}, ensure_ascii=False))


if __name__ == "__main__":
    main()
