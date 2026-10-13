"""Offline pursuit/escape prototype. Standard library only; no hardware I/O."""
from __future__ import annotations

import argparse
import heapq
import json
import math
import random
import sys
from dataclasses import dataclass
from pathlib import Path

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT.parent / "TreasureRoutePlanner"))
from planner import TrackMap, heading, turn  # noqa: E402

EPS = 1e-9
PHASES = {"ready": 0, "turn": 1, "move": 2, "pickup": 3, "wait": 4}
REASONS = {"treasure": 0, "loop": 1, "escape": 2, "chase": 3, "intercept": 4, "hold": 5}


def validate_config(config):
    positive = ["police_speed_m_s", "thief_speed_m_s", "sensor_range_m", "horizon_s",
                "time_limit_s", "sensor_period_s", "frame_period_s"]
    nonnegative = ["turn90_s", "uturn_s", "pickup_s", "capture_center_distance_m"]
    for key in positive + nonnegative:
        value = config[key]
        if not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError(f"Invalid {key}")
        if value < 0 or key in positive and value == 0:
            raise ValueError(f"Invalid {key}")
    if not 0 <= config["miss_probability"] <= 1 or not 0 <= config["sensor_half_angle_deg"] <= 45:
        raise ValueError("Invalid sensor model")


class Routing:
    """All-target heading-aware travel-time tables cached at decision nodes."""
    def __init__(self, track, config):
        self.track, self.config = track, config
        self.cache, self.distance = {}, {}
        for source in track.graph:
            best, queue = {source: 0}, [(0, source)]
            while queue:
                value, u = heapq.heappop(queue)
                if value != best[u]:
                    continue
                for v, ticks in track.graph[u].items():
                    candidate = value + ticks
                    if candidate < best.get(v, math.inf):
                        best[v] = candidate
                        heapq.heappush(queue, (candidate, v))
            self.distance[source] = {v: ticks * track.spacing / 2 for v, ticks in best.items()}

    def edge_time(self, u, v, facing, speed):
        direction = heading(self.track.xy[u], self.track.xy[v])
        action = turn(facing, direction)
        delay = self.config["turn90_s"] if action in ("LEFT", "RIGHT") else self.config["uturn_s"] if action == "UTURN" else 0
        return self.track.graph[u][v] * self.track.spacing / 2 / speed + delay, direction

    def table(self, source, facing, speed):
        key = source, facing, speed
        if key in self.cache:
            return self.cache[key]
        initial = source, facing
        best, first = {initial: 0}, {initial: None}
        queue, serial = [(0, 0, initial)], 0
        times, next_nodes = {}, {}
        while queue:
            value, _, state = heapq.heappop(queue)
            if value != best[state]:
                continue
            u, direction = state
            if u not in times:
                times[u], next_nodes[u] = value, first[state]
            for v in sorted(self.track.graph[u]):
                duration, new_direction = self.edge_time(u, v, direction, speed)
                new_state, candidate = (v, new_direction), value + duration
                if candidate + EPS < best.get(new_state, math.inf):
                    best[new_state], first[new_state] = candidate, first[state] or v
                    serial += 1
                    heapq.heappush(queue, (candidate, serial, new_state))
        self.cache[key] = times, next_nodes
        return times, next_nodes

    def paths(self, source, facing, speed, horizon=None, limit=48):
        horizon = horizon or self.config["horizon_s"]
        leaves = []

        def visit(path, times, direction, cap):
            if len(leaves) >= cap:
                return
            u = path[-1]
            if times[-1] >= horizon or len(path) >= 18:
                leaves.append((path, times))
                return
            neighbors = sorted(self.track.graph[u])
            if len(path) > 1 and len(neighbors) > 1:
                neighbors = [v for v in neighbors if v != path[-2]]
            if not neighbors:
                leaves.append((path, times))
            for v in neighbors:
                if len(leaves) >= cap:
                    break
                duration, new_direction = self.edge_time(u, v, direction, speed)
                if times[-1] + duration > horizon and len(path) > 1:
                    leaves.append((path, times))
                    continue
                visit(path + [v], times + [times[-1] + duration], new_direction, cap)

        # Reserve candidates for every legal first action, avoiding DFS bias.
        neighbors = sorted(self.track.graph[source])
        budget = max(1, limit // max(1, len(neighbors)))
        for neighbor in neighbors:
            duration, new_direction = self.edge_time(source, neighbor, facing, speed)
            visit([source, neighbor], [0.0, duration], new_direction, len(leaves) + budget)
        return [(p, t) for p, t in leaves if len(p) > 1]


@dataclass
class Robot:
    role: str
    node: str
    facing: str
    speed: float
    phase: str = "ready"
    target: str | None = None
    remaining: float = 0.0
    progress: float = 0.0
    reason: str = "treasure"
    last_node: str | None = None
    traveled_m: float = 0.0

    def xy(self, track):
        a = track.xy[self.node]
        if self.phase == "move":
            b = track.xy[self.target]
            return (a[0] + (b[0] - a[0]) * self.progress,
                    a[1] + (b[1] - a[1]) * self.progress)
        return a

    def velocity(self, track):
        if self.phase != "move":
            return 0.0, 0.0
        a, b = track.xy[self.node], track.xy[self.target]
        distance = abs(b[0] - a[0]) + abs(b[1] - a[1])
        return (b[0] - a[0]) / distance * self.speed / track.spacing, (b[1] - a[1]) / distance * self.speed / track.spacing

    def command(self, track, routing, neighbor, reason):
        self.reason = reason
        if neighbor is None:
            self.phase, self.remaining, self.target = "wait", .8, None
            return
        if neighbor not in track.graph[self.node]:
            raise ValueError("Command must select an adjacent track node")
        duration, direction = routing.edge_time(self.node, neighbor, self.facing, self.speed)
        travel = track.graph[self.node][neighbor] * track.spacing / 2 / self.speed
        self.target, self.progress, self.facing = neighbor, 0.0, direction
        delay = max(0.0, duration - travel)
        self.phase, self.remaining = ("turn", delay) if delay > EPS else ("move", travel)

    def advance(self, track, dt):
        if self.phase == "move":
            distance = track.graph[self.node][self.target] * track.spacing / 2
            self.progress = min(1.0, self.progress + dt * self.speed / distance)
            self.traveled_m += dt * self.speed
        self.remaining = max(0.0, self.remaining - dt)

    def finish(self, track):
        if self.remaining > EPS or self.phase in ("ready", "pickup"):
            return
        if self.phase == "turn":
            self.phase = "move"
            self.remaining = track.graph[self.node][self.target] * track.spacing / 2 / self.speed
        elif self.phase == "move":
            self.last_node, self.node = self.node, self.target
            self.target, self.progress, self.phase = None, 0.0, "ready"
        else:
            self.phase = "ready"


def detectable(a, b, config, spacing):
    dx, dy = (b[0] - a[0]) * spacing, (b[1] - a[1]) * spacing
    if math.hypot(dx, dy) > config["sensor_range_m"] + EPS:
        return False
    # Ideal four cardinal sensors, without physical walls or occlusion.
    angle = math.degrees(math.atan2(abs(dy), abs(dx)))
    return min(angle, 90 - angle) <= config["sensor_half_angle_deg"] + EPS


class Belief:
    """A speed-bounded set of possible nodes, not a Bayesian POMDP filter."""
    def __init__(self, initial):
        self.last_seen, self.last_time = initial, 0.0
        self.last_heading, self.seen = None, False
        self.nodes = [initial]

    def observe(self, ego, other_xy, now, vision, routing, enemy_speed, rng):
        track, config = routing.track, routing.config
        self.seen = vision == "global" or (detectable(ego.xy(track), other_xy, config, track.spacing)
                                            and rng.random() >= config["miss_probability"])
        if self.seen:
            measured = min(track.xy, key=lambda n: math.dist(track.xy[n], other_xy))
            if measured != self.last_seen:
                a, b = track.xy[self.last_seen], track.xy[measured]
                if a[0] == b[0] or a[1] == b[1]:
                    self.last_heading = heading(a, b)
                else:
                    self.last_heading = None
            self.last_seen, self.last_time = measured, now
            self.nodes = [measured]
        else:
            radius = enemy_speed * max(0, now - self.last_time) + track.spacing
            self.nodes = [n for n in track.graph if routing.distance[self.last_seen][n] <= radius + EPS]


def capture_delay(police, thief, track, radius_m, maximum_dt):
    """Earliest center-threshold contact in one collinear, continuous interval."""
    p, e = police.xy(track), thief.xy(track)
    axis_p = 0 if police.facing in ("E", "W") else 1
    axis_e = 0 if thief.facing in ("E", "W") else 1
    if axis_p != axis_e or abs(p[1 - axis_p] - e[1 - axis_p]) > EPS:
        return None
    # Ensure both positions belong to the same connected straight track run.
    axis = axis_p
    low, high = sorted((p[axis], e[axis]))
    intervals = sorted((min(track.xy[u][axis], track.xy[v][axis]),
                        max(track.xy[u][axis], track.xy[v][axis]))
                       for u in track.graph for v in track.graph[u] if u < v
                       and abs(track.xy[u][1 - axis] - p[1 - axis]) < EPS
                       and abs(track.xy[v][1 - axis] - p[1 - axis]) < EPS)
    cursor, covered = low, False
    for start, end in intervals:
        if end < cursor - EPS:
            continue
        if start > cursor + EPS:
            break
        cursor = max(cursor, end)
        if cursor >= high - EPS:
            covered = True
            break
    if not covered:
        return None
    delta = (p[axis] - e[axis]) * track.spacing
    relative = (police.velocity(track)[axis] - thief.velocity(track)[axis]) * track.spacing
    if abs(delta) <= radius_m + EPS:
        return 0.0
    if abs(relative) < EPS:
        return None
    times = sorted(((-radius_m - delta) / relative, (radius_m - delta) / relative))
    enter, leave = max(0.0, times[0]), min(maximum_dt, times[1])
    return enter if enter <= leave + EPS else None


class Game:
    def __init__(self, track, config, police_policy="intercept", thief_policy="escape", vision="global", seed=None):
        validate_config(config)
        if police_policy not in ("race", "chase", "intercept") or thief_policy not in ("loop", "escape"):
            raise ValueError("Unknown policy")
        if vision not in ("global", "limited"):
            raise ValueError("Unknown vision")
        self.track, self.config, self.routing = track, config, Routing(track, config)
        self.police_policy, self.thief_policy, self.vision = police_policy, thief_policy, vision
        self.robots = {
            "P": Robot("P", track.resolve("P"), "N", config["police_speed_m_s"]),
            "B": Robot("B", track.resolve("B"), "W", config["thief_speed_m_s"])}
        self.beliefs = {"P": Belief(track.resolve("B")), "B": Belief(track.resolve("P"))}
        seed = config["seed"] if seed is None else seed
        self.rngs = {"P": random.Random(seed), "B": random.Random(seed + 100003)}
        self.owner, self.now, self.winner, self.end_reason = None, 0.0, None, None
        self.frames, self.events, self.pickup_times = [], [], {}
        loop = track.expand_waypoints(track.data["candidate_cycles"]["left_large"])
        self.loop_next = dict(zip(loop, loop[1:]))

    def nearest_enemy(self, role):
        actor, belief = self.robots[role], self.beliefs[role]
        return min(belief.nodes, key=lambda n: (self.routing.distance[belief.last_seen][n], n != actor.node, n))

    def chase(self, actor, belief):
        times, next_nodes = self.routing.table(actor.node, actor.facing, actor.speed)
        targets = [n for n in belief.nodes if n != actor.node]
        if not targets:
            # Target can move inside the quantized cell: keep searching adjacent roads.
            targets = list(self.track.graph[actor.node])
        target = min(targets, key=lambda n: (self.routing.distance[belief.last_seen][n], times[n]))
        return next_nodes[target], "chase", target

    def escape_score(self, path, times, threats, pursuer_speed):
        margin = min(min(self.routing.distance[q][node] for q in threats) / pursuer_speed
                     - elapsed - self.config["capture_center_distance_m"] / pursuer_speed
                     for node, elapsed in zip(path[1:], times[1:]))
        trap = any(len(self.track.graph[n]) == 1 for n in path[1:])
        options = len(self.track.graph[path[-1]])
        return (not trap, margin, options, -times[-1])

    def escape(self, actor, belief):
        candidates = self.routing.paths(actor.node, actor.facing, actor.speed)
        if not candidates:
            return next(iter(self.track.graph[actor.node])), "escape", actor.node
        opponent_speed = self.config["police_speed_m_s"]
        path, _ = max(candidates, key=lambda item: self.escape_score(*item, belief.nodes, opponent_speed))
        return path[1], "escape", path[-1]

    def intercept(self, actor, belief):
        # Predict a bounded selection of plausible escape continuations.
        roots = sorted(belief.nodes, key=lambda n: self.routing.distance[belief.last_seen][n])[:6]
        predicted = []
        enemy_speed = self.config["thief_speed_m_s"]
        for root in roots:
            paths = self.routing.paths(root, belief.last_heading, enemy_speed, limit=24)
            paths.sort(key=lambda item: self.escape_score(*item, [actor.node], actor.speed), reverse=True)
            predicted.extend(paths[:3])
        if not predicted:
            return self.chase(actor, belief)
        times, next_nodes = self.routing.table(actor.node, actor.facing, actor.speed)
        targets = {n for path, _ in predicted for n in path[1:]}
        best = None
        for target in sorted(targets):
            arrival = times[target]
            if target == actor.node and self.now < 5.0:
                continue  # No delaying the initial departure.
            windows = [min((eta for n, eta in zip(path, etas) if n == target), default=math.inf)
                       for path, etas in predicted]
            covered = [eta for eta in windows if arrival <= eta + .2 and math.isfinite(eta)]
            if not covered:
                continue
            # Prefer shared interception points, then short forecast horizons.
            score = (len(covered) / len(predicted), -sum(covered) / len(covered), -arrival,
                     len(self.track.graph[target]))
            if best is None or score > best[0]:
                best = score, target
        if best is None:
            return self.chase(actor, belief)
        target = best[1]
        return next_nodes[target], "hold" if target == actor.node else "intercept", target

    def decide(self, role):
        actor, belief = self.robots[role], self.beliefs[role]
        if self.owner is None:
            # First prototype races for treasure before applying pursuit heuristics.
            _, next_nodes = self.routing.table(actor.node, actor.facing, actor.speed)
            return next_nodes[self.track.resolve("T")], "treasure", self.track.resolve("T")
        if role == "B":
            if self.thief_policy == "loop":
                if actor.node in self.loop_next:
                    return self.loop_next[actor.node], "loop", self.track.resolve("T")
                _, next_nodes = self.routing.table(actor.node, actor.facing, actor.speed)
                return next_nodes[self.track.resolve("T")], "loop", self.track.resolve("T")
            return self.escape(actor, belief)
        if self.police_policy == "intercept":
            return self.intercept(actor, belief)
        return self.chase(actor, belief)

    def observe(self):
        # Both policies observe the same instant before either command is applied.
        for role, opposite in (("P", "B"), ("B", "P")):
            self.beliefs[role].observe(self.robots[role], self.robots[opposite].xy(self.track),
                                      self.now, self.vision, self.routing, self.robots[opposite].speed, self.rngs[role])

    def settle(self):
        if self.winner is not None:
            return
        for actor in self.robots.values():
            actor.finish(self.track)
        complete = [r.role for r in self.robots.values() if r.phase == "pickup" and r.remaining <= EPS]
        if complete and self.owner is None:
            if len(complete) > 1:
                self.winner, self.end_reason = "judge", "simultaneous_pickup"
                return
            self.owner = complete[0]
            self.pickup_times[self.owner] = round(self.now, 6)
            self.events.append({"t": round(self.now, 3), "event": "pickup", "role": self.owner})
            self.robots[self.owner].phase = "ready"
            if self.owner == "P":
                self.winner, self.end_reason = "police", "police_pickup"
                return
        if self.owner:
            for actor in self.robots.values():
                if actor.phase == "pickup":
                    actor.phase = "ready"

    def command_ready(self):
        decisions = {}
        for role, actor in self.robots.items():
            if actor.phase != "ready":
                continue
            if self.owner is None and actor.node == self.track.resolve("T"):
                decisions[role] = "pickup"
            else:
                decisions[role] = self.decide(role)
        for role, command in decisions.items():
            actor = self.robots[role]
            if command == "pickup":
                actor.phase, actor.remaining, actor.reason = "pickup", self.config["pickup_s"], "treasure"
            else:
                next_node, reason, goal = command
                actor.command(self.track, self.routing, next_node, reason)
                if reason in ("intercept", "escape"):
                    self.events.append({"t": round(self.now, 3), "event": reason, "role": role, "goal": goal})

    def record(self):
        values = [round(self.now, 4), 0 if self.owner is None else 1 if self.owner == "P" else 2]
        for role in ("P", "B"):
            actor, belief = self.robots[role], self.beliefs[role]
            x, y = actor.xy(self.track)
            values.extend([round(x, 4), round(y, 4), actor.facing, PHASES[actor.phase],
                           REASONS[actor.reason], int(belief.seen), len(belief.nodes)])
        if self.frames and abs(self.frames[-1][0] - values[0]) < EPS:
            self.frames[-1] = values
        else:
            self.frames.append(values)

    def run(self):
        self.observe()
        next_sensor, next_frame = self.config["sensor_period_s"], 0.0
        while self.winner is None and self.now < self.config["time_limit_s"] - EPS:
            self.settle()
            if self.winner:
                break
            self.command_ready()
            # Zero-duration pickup/turn transitions must settle before advancing.
            self.settle()
            if self.winner:
                break
            if self.now >= next_frame - EPS:
                self.record()
                next_frame += self.config["frame_period_s"]
            pending = [r.remaining for r in self.robots.values() if r.phase != "ready" and r.remaining > EPS]
            dt = min([self.config["time_limit_s"] - self.now, max(EPS, next_sensor - self.now)] + pending)
            delay = capture_delay(self.robots["P"], self.robots["B"], self.track,
                                  self.config["capture_center_distance_m"], dt)
            if delay is not None:
                for actor in self.robots.values():
                    actor.advance(self.track, delay)
                self.now += delay
                self.winner = "police"
                self.end_reason = "capture_before_pickup" if self.owner is None else "capture_after_pickup"
                if self.owner == "B":
                    self.owner = "P"  # Model assumes a successful seizure on contact.
                self.events.append({"t": round(self.now, 3), "event": "capture"})
                break
            for actor in self.robots.values():
                actor.advance(self.track, dt)
            self.now += dt
            if self.now >= next_sensor - EPS:
                self.observe()
                next_sensor += self.config["sensor_period_s"]
        self.settle()
        if self.winner is None:
            self.winner, self.end_reason = ("thief", "survived_timeout") if self.owner == "B" else ("draw", "no_treasure_timeout")
        self.record()
        return {"police_policy": self.police_policy, "thief_policy": self.thief_policy,
                "vision": self.vision, "winner": self.winner, "end_reason": self.end_reason,
                "duration_s": round(self.now, 6), "pickup_times": self.pickup_times,
                "traveled_m": {r: round(a.traveled_m, 5) for r, a in self.robots.items()},
                "frames": self.frames, "events": self.events}


def suite(track, config, seeds):
    scenarios = [
        ("固定回路对直接追击", "chase", "loop", "global"),
        ("动态逃跑对直接追击", "chase", "escape", "global"),
        ("动态逃跑对预测拦截", "intercept", "escape", "global"),
        ("有限视野下的预测拦截", "intercept", "escape", "limited")]
    replays, summary = [], []
    for label, police, thief, vision in scenarios:
        outcomes = []
        for offset in range(seeds):
            result = Game(track, config, police, thief, vision, config["seed"] + offset).run()
            outcomes.append(result)
        replay = outcomes[0]
        replay["label"] = label
        replays.append(replay)
        summary.append({"scenario": label, "runs": seeds,
                        "police_wins": sum(r["winner"] == "police" for r in outcomes),
                        "thief_wins": sum(r["winner"] == "thief" for r in outcomes),
                        "draws": sum(r["winner"] == "draw" for r in outcomes),
                        "judge_required": sum(r["winner"] == "judge" for r in outcomes),
                        "mean_duration_s": round(sum(r["duration_s"] for r in outcomes) / seeds, 4)})
        print(f"{label}: {summary[-1]}", flush=True)
    return {"config": config, "map": track.data,
            "frame_fields": ["t", "owner", "px", "py", "pheading", "pphase", "preason", "pseen", "pbelief",
                             "bx", "by", "bheading", "bphase", "breason", "bseen", "bbelief"],
            "summary": summary, "replays": replays}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=ROOT / "config.json")
    parser.add_argument("--seeds", type=int, default=1)
    args = parser.parse_args()
    if args.seeds < 1:
        parser.error("--seeds must be positive")
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")
    config = json.loads(args.config.read_text(encoding="utf-8"))
    report = suite(TrackMap(), config, args.seeds)
    (ROOT / "report.json").write_text(json.dumps(report, ensure_ascii=False, separators=(",", ":")), encoding="utf-8")
    payload = json.dumps(report, ensure_ascii=False, separators=(",", ":")).replace("</", "<\\/")
    template = (ROOT / "replay.template.html").read_text(encoding="utf-8")
    (ROOT / "replay.html").write_text(template.replace("__REPORT_DATA__", payload), encoding="utf-8")
    print("Saved report.json and replay.html")


if __name__ == "__main__":
    main()
