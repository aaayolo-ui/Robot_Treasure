"""Offline line-track planner. Standard library only; never controls hardware."""
from __future__ import annotations

import argparse
import heapq
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parent
HEADINGS = ("N", "E", "S", "W")


def node_id(xy):
    x, y = xy
    return f"({x:g},{y:g})"


def heading(a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    if dx and dy or not (dx or dy):
        raise ValueError(f"Non-cardinal edge: {a} -> {b}")
    return "E" if dx > 0 else "W" if dx < 0 else "S" if dy > 0 else "N"


def turn(old, new):
    if old is None:
        return "START"
    delta = (HEADINGS.index(new) - HEADINGS.index(old)) % 4
    return ("STRAIGHT", "RIGHT", "UTURN", "LEFT")[delta]


class TrackMap:
    def __init__(self, filename=ROOT / "map.json"):
        self.data = json.loads(Path(filename).read_text(encoding="utf-8"))
        self.spacing = self.data["grid_spacing_m"]
        self.xy = {}
        self.graph = {}
        self.special = {}

        def add_node(xy):
            xy = tuple(xy)
            name = node_id(xy)
            self.xy[name] = xy
            self.graph.setdefault(name, {})
            return name

        def add_edge(a, b):
            u, v = add_node(a), add_node(b)
            heading(a, b)
            # Integer half-cell units prevent shortest-distance rounding errors.
            ticks = round(2 * (abs(a[0] - b[0]) + abs(a[1] - b[1])))
            if ticks <= 0:
                raise ValueError("Zero-length edge")
            self.graph[u][v] = self.graph[v][u] = ticks

        for y, x0, x1 in self.data["horizontal_segments"]:
            for x in range(x0, x1):
                add_edge((x, y), (x + 1, y))
        for x, y0, y1 in self.data["vertical_segments"]:
            for y in range(y0, y1):
                add_edge((x, y), (x, y + 1))
        for alias, detail in self.data["special_nodes"].items():
            xy = tuple(detail["xy"])
            name = node_id(xy)
            if name not in self.graph:
                # The treasure is in the middle of a horizontal half-cell.
                a, b = (math.floor(xy[0]), xy[1]), (math.ceil(xy[0]), xy[1])
                u, v = node_id(a), node_id(b)
                if v not in self.graph.get(u, {}):
                    raise ValueError(f"Special node is not on an edge: {alias}")
                del self.graph[u][v]
                del self.graph[v][u]
                add_edge(a, xy)
                add_edge(xy, b)
            self.special[alias] = name

    def resolve(self, name):
        aliases = {"police": "P", "thief": "B", "treasure": "T"}
        name = aliases.get(name, name)
        name = self.special.get(name, name)
        if name not in self.graph:
            raise ValueError(f"Unknown node: {name}")
        return name

    def initial_heading(self, name):
        node = self.resolve(name)
        for key, value in self.special.items():
            if value == node:
                return self.data["special_nodes"][key]["heading"]
        return None

    def dijkstra(self, start, target, mode="distance", speed=0.35,
                 turn90=0.6, uturn=1.2, initial_heading=None,
                 blocked_nodes=(), blocked_edges=()):
        start, target = self.resolve(start), self.resolve(target)
        if mode not in ("distance", "time"):
            raise ValueError("mode must be distance or time")
        if not all(math.isfinite(v) for v in (speed, turn90, uturn)):
            raise ValueError("Non-finite motion parameter")
        if speed <= 0 or turn90 < 0 or uturn < 0:
            raise ValueError("Invalid motion parameters")
        blocked = {self.resolve(x) for x in blocked_nodes}
        unavailable = {frozenset((self.resolve(a), self.resolve(b))) for a, b in blocked_edges}
        if start in blocked or target in blocked:
            raise ValueError("Start or target is blocked")
        initial_heading = initial_heading or self.initial_heading(start)
        if initial_heading is not None and initial_heading not in HEADINGS:
            raise ValueError("Invalid initial heading")
        initial = (start, initial_heading)
        best = {initial: (0, 0, 0)}
        previous = {}
        serial = 0
        queue = [(best[initial], serial, initial)]
        while queue:
            score, _, state = heapq.heappop(queue)
            if best[state] != score:
                continue
            u, facing = state
            if u == target:
                states = [state]
                while states[-1] in previous:
                    states.append(previous[states[-1]])
                path = [s[0] for s in states[::-1]]
                return self.describe(path, speed, turn90, uturn, initial_heading, mode)
            for v, ticks in sorted(self.graph[u].items()):
                if v in blocked or frozenset((u, v)) in unavailable:
                    continue
                direction = heading(self.xy[u], self.xy[v])
                action = turn(facing, direction)
                n90, n180 = int(action in ("LEFT", "RIGHT")), int(action == "UTURN")
                seconds = ticks * self.spacing / 2 / speed + n90 * turn90 + n180 * uturn
                increment = (ticks, n90, n180) if mode == "distance" else (seconds, ticks, n90 + n180)
                new_score = tuple(a + b for a, b in zip(score, increment))
                new_state = (v, direction)
                if new_state not in best or new_score < best[new_state]:
                    best[new_state] = new_score
                    previous[new_state] = state
                    serial += 1
                    heapq.heappush(queue, (new_score, serial, new_state))
        raise ValueError("No reachable route")

    def describe(self, path, speed=0.35, turn90=0.6, uturn=1.2,
                 initial_heading=None, mode="distance"):
        total = n90 = n180 = 0
        facing = initial_heading
        legs = []
        events = []
        for u, v in zip(path, path[1:]):
            ticks = self.graph[u][v]
            direction = heading(self.xy[u], self.xy[v])
            action = turn(facing, direction)
            n90 += action in ("LEFT", "RIGHT")
            n180 += action == "UTURN"
            length = ticks * self.spacing / 2
            total += ticks
            if legs and legs[-1]["heading"] == direction:
                legs[-1]["to"] = v
                legs[-1]["length_m"] = round(legs[-1]["length_m"] + length, 6)
            else:
                legs.append({"from": u, "to": v, "heading": direction,
                             "action": action, "length_m": round(length, 6)})
            if len(self.graph[u]) != 2 or action not in ("START", "STRAIGHT"):
                events.append({"node": u, "incoming": facing, "outgoing": direction,
                               "action": action, "degree": len(self.graph[u]),
                               "exits": sorted(heading(self.xy[u], self.xy[w]) for w in self.graph[u])})
            facing = direction
        distance = total * self.spacing / 2
        return {"mode": mode, "nodes": path, "xy": [self.xy[n] for n in path],
                "distance_m": round(distance, 6), "quarter_turns": n90,
                "uturns": n180, "initial_heading": initial_heading,
                "arrival_heading": facing,
                "estimated_seconds": round(distance / speed + n90 * turn90 + n180 * uturn, 6),
                "legs": legs, "navigation_events": events}

    def expand_waypoints(self, points):
        path = []
        for a, b in zip(points, points[1:]):
            direction = heading(a, b)
            candidates = [n for n, xy in self.xy.items()
                          if (xy[1] == a[1] and min(a[0], b[0]) <= xy[0] <= max(a[0], b[0]))
                          if direction in ("E", "W")]
            if direction in ("N", "S"):
                candidates = [n for n, xy in self.xy.items()
                              if xy[0] == a[0] and min(a[1], b[1]) <= xy[1] <= max(a[1], b[1])]
            candidates.sort(key=lambda n: abs(self.xy[n][0] - a[0]) + abs(self.xy[n][1] - a[1]))
            if not candidates or self.xy[candidates[0]] != tuple(a) or self.xy[candidates[-1]] != tuple(b):
                raise ValueError("Waypoint is not on the track")
            for u, v in zip(candidates, candidates[1:]):
                if v not in self.graph[u]:
                    raise ValueError(f"Cycle crosses a track gap: {u} -> {v}")
            path.extend(candidates if not path else candidates[1:])
        return path

    def structure(self):
        visited = set()
        components = []
        for start in self.graph:
            if start in visited:
                continue
            pending = [start]
            comp = []
            visited.add(start)
            while pending:
                u = pending.pop()
                comp.append(u)
                for v in self.graph[u]:
                    if v not in visited:
                        visited.add(v)
                        pending.append(v)
            components.append(comp)
        clock, discovered, low, cuts, bridges = 0, {}, {}, set(), []

        def dfs(u, parent=None):
            nonlocal clock
            clock += 1
            discovered[u] = low[u] = clock
            children = 0
            for v in self.graph[u]:
                if v == parent:
                    continue
                if v not in discovered:
                    children += 1
                    dfs(v, u)
                    low[u] = min(low[u], low[v])
                    if low[v] > discovered[u]:
                        bridges.append([u, v])
                    if parent is not None and low[v] >= discovered[u]:
                        cuts.add(u)
                else:
                    low[u] = min(low[u], discovered[v])
            if parent is None and children > 1:
                cuts.add(u)

        for start in self.graph:
            if start not in discovered:
                dfs(start)
        edges = sum(map(len, self.graph.values())) // 2
        return {"node_count": len(self.graph), "edge_count": edges,
                "connected_components": len(components),
                "cycle_rank": edges - len(self.graph) + len(components),
                "dead_ends": sorted(n for n, adjacent in self.graph.items() if len(adjacent) == 1),
                "articulation_points": sorted(cuts), "bridges": bridges}

    def shortest_path_count(self, start, target):
        start, target = self.resolve(start), self.resolve(target)
        dist, ways = {start: 0}, {start: 1}
        queue = [(0, start)]
        while queue:
            d, u = heapq.heappop(queue)
            if d != dist[u]:
                continue
            for v, weight in self.graph[u].items():
                new = d + weight
                if new < dist.get(v, math.inf):
                    dist[v], ways[v] = new, ways[u]
                    heapq.heappush(queue, (new, v))
                elif new == dist[v]:
                    ways[v] += ways[u]
        return ways.get(target, 0)

    def shortest_alternatives(self, start, target, speed=0.35, turn90=0.6, uturn=1.2, limit=20):
        """Enumerate the shortest-distance DAG; limit bounds the returned list."""
        start, target = self.resolve(start), self.resolve(target)
        dist = {target: 0}
        queue = [(0, target)]
        while queue:
            cost, u = heapq.heappop(queue)
            if cost != dist[u]:
                continue
            for v, ticks in self.graph[u].items():
                candidate = cost + ticks
                if candidate < dist.get(v, math.inf):
                    dist[v] = candidate
                    heapq.heappush(queue, (candidate, v))
        found = []

        def visit(path):
            if len(found) >= limit:
                return
            u = path[-1]
            if u == target:
                found.append(self.describe(path, speed, turn90, uturn, self.initial_heading(start)))
                return
            for v, ticks in sorted(self.graph[u].items()):
                if dist.get(v, math.inf) + ticks == dist[u]:
                    visit(path + [v])

        if start in dist:
            visit([start])
        return sorted(found, key=lambda r: (r["quarter_turns"], r["uturns"], r["nodes"]))

    def export(self, speed=0.35, turn90=0.6, uturn=1.2):
        routes = {}
        for key, start in (("police", "P"), ("thief", "B")):
            routes[key] = {
                mode: self.dijkstra(start, "T", mode, speed, turn90, uturn)
                for mode in ("distance", "time")}
            routes[key]["number_of_shortest_paths"] = self.shortest_path_count(start, "T")
            routes[key]["shortest_alternatives"] = self.shortest_alternatives(start, "T", speed, turn90, uturn)
        cycles = {}
        for key, points in self.data["candidate_cycles"].items():
            path = self.expand_waypoints(points)
            # A whole lap includes the closing turn at the first vertex.
            arrival = heading(self.xy[path[-2]], self.xy[path[-1]])
            cycles[key] = self.describe(path, speed, turn90, uturn, arrival, "cycle")
            cycles[key]["waypoints"] = points
        return {"assumptions": self.data["verification"],
                "parameters": {"speed_m_s": speed, "turn90_s": turn90, "uturn_s": uturn,
                               "pickup_s": "not measured; not included"},
                "structure": self.structure(), "routes": routes, "cycles": cycles,
                "scenarios": {
                    "police_slow_turn": self.dijkstra("P", "T", "time", speed, 1.0, 2.0),
                    "police_lower_route": self.describe(self.expand_waypoints(
                        [[0, 9], [0, 7], [3, 7], [3, 9], [6, 9], [6, 0], [5.5, 0]]),
                        speed, turn90, uturn, "N", "time"),
                    "thief_upper_route": self.describe(self.expand_waypoints(
                        [[11, 5], [10, 5], [10, 3], [8, 3], [8, 0], [5.5, 0]]),
                        speed, turn90, uturn, "W", "distance")},
                "nodes": [{"id": n, "xy": self.xy[n], "degree": len(self.graph[n])} for n in sorted(self.graph)],
                "edges": [{"a": u, "b": v, "length_m": ticks * self.spacing / 2}
                          for u in sorted(self.graph) for v, ticks in sorted(self.graph[u].items()) if u < v]}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map", type=Path, default=ROOT / "map.json")
    parser.add_argument("--start", default="P")
    parser.add_argument("--target", default="T")
    parser.add_argument("--mode", choices=("distance", "time"), default="distance")
    parser.add_argument("--speed", type=float, default=0.35)
    parser.add_argument("--turn90", type=float, default=0.6)
    parser.add_argument("--uturn", type=float, default=1.2)
    parser.add_argument("--heading", choices=HEADINGS)
    parser.add_argument("--blocked", nargs="*", default=[])
    parser.add_argument("--report", type=Path)
    args = parser.parse_args()
    track = TrackMap(args.map)
    if args.report:
        result = track.export(args.speed, args.turn90, args.uturn)
        args.report.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        print(f"Wrote {args.report}")
    else:
        result = track.dijkstra(args.start, args.target, args.mode, args.speed,
                                args.turn90, args.uturn, args.heading, args.blocked)
        print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
