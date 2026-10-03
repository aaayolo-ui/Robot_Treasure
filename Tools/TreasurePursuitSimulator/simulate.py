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
from map_annotations import build_annotations, landmark_type

EPS = 1e-9
DIRECTIONS = ("N", "E", "S", "W")
VECTORS = {"N": (0, -1), "E": (1, 0), "S": (0, 1), "W": (-1, 0)}
PHASES = {"ready": 0, "turn": 1, "move": 2, "pickup": 3, "wait": 4, "hide": 5, "orient": 6, "reverse": 7}
REASONS = {"treasure": 0, "loop": 1, "escape": 2, "chase": 3, "intercept": 4, "hold": 5,
           "search": 6, "cutoff": 7, "cruise": 8, "seek_hide": 9, "hide": 10,
           "prepare_hide": 11, "evade": 12, "patrol": 13, "preset_evade": 14}
REASONS.update(trail=15, check_deadend=16, relocate=17)


def pose_heading(a, b):
    """Cardinal bearing with tolerance for interpolated floating-point poses."""
    dx, dy = b[0] - a[0], b[1] - a[1]
    return ("E" if dx > 0 else "W") if abs(dx) >= abs(dy) else ("S" if dy > 0 else "N")


def sensor_channel(ego, measured_xy, track):
    """Relative IR channel from an in-range measurement and the own travel axis.

    The old motion model stores a commanded heading during a turn, not a
    continuous physical yaw. Do not invent a left/right channel in that phase.
    """
    if ego.phase in ("turn", "orient") or math.dist(ego.xy(track), measured_xy) < EPS:
        return None
    facing = ego.body_facing or (heading(track.xy[ego.node], track.xy[ego.target]) if ego.phase == "move" else ego.facing)
    bearing = pose_heading(ego.xy(track), measured_xy)
    return ("front", "right", "rear", "left")[(DIRECTIONS.index(bearing) - DIRECTIONS.index(facing)) % 4]


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
    if "police_sensor_range_m" in config and (not isinstance(config["police_sensor_range_m"], (int, float)) or
            not math.isfinite(config["police_sensor_range_m"]) or config["police_sensor_range_m"] <= 0):
        raise ValueError("Invalid police_sensor_range_m")
    for key in ("hide_reaction_margin_s", "hide_escape_hold_s", "hide_revisit_cooldown_s",
                "pursuit_memory_s", "escape_risk_step_s", "escape_straight_limit_cells",
                "escape_safety_slack_s", "side_escape_cells"):
        if key in config and (not isinstance(config[key], (int, float)) or not math.isfinite(config[key]) or config[key] <= 0):
            raise ValueError(f"Invalid {key}")
    for key in ("police_search_memory_s", "police_deadend_check_cells", "police_deadend_recheck_s", "hide_max_straight_cells"):
        if key in config and (not isinstance(config[key], (int, float)) or not math.isfinite(config[key]) or config[key] <= 0):
            raise ValueError(f"Invalid {key}")
    if not 0 <= config["miss_probability"] <= 1 or not 0 <= config["sensor_half_angle_deg"] <= 45:
        raise ValueError("Invalid sensor model")


class Routing:
    """All-target heading-aware travel-time tables cached at decision nodes."""
    def __init__(self, track, config):
        self.track, self.config = track, config
        self.cache, self.distance, self.contact_cache = {}, {}, {}
        self.runs = {0: {}, 1: {}}
        for axis in (0, 1):
            lines = self.runs[axis]
            for u in track.graph:
                for v in track.graph[u]:
                    a, b = track.xy[u], track.xy[v]
                    if u < v and abs(a[1 - axis] - b[1 - axis]) < EPS:
                        lines.setdefault(round(a[1 - axis], 8), []).append(sorted((a[axis], b[axis])))
            for line, intervals in lines.items():
                merged = []
                for low, high in sorted(intervals):
                    if merged and low <= merged[-1][1] + EPS:
                        merged[-1][1] = max(merged[-1][1], high)
                    else:
                        merged.append([low, high])
                lines[line] = merged
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

    def straight_connected(self, a, b):
        for axis in (0, 1):
            if abs(a[1 - axis] - b[1 - axis]) < EPS:
                low, high = sorted((a[axis], b[axis]))
                if any(start - EPS <= low and high <= end + EPS
                       for start, end in self.runs[axis].get(round(a[1 - axis], 8), [])):
                    return True
        return False

    def contact_segments(self, u, v):
        key = tuple(sorted((u, v)))
        if key not in self.contact_cache:
            radius = self.config["capture_center_distance_m"] / self.track.spacing
            a, b = self.track.xy[u], self.track.xy[v]
            selected = []
            for s in self.track.graph:
                for t in self.track.graph[s]:
                    if s >= t:
                        continue
                    c, d = self.track.xy[s], self.track.xy[t]
                    gap2 = sum(max(0, min(a[i], b[i]) - max(c[i], d[i]),
                                   min(c[i], d[i]) - max(a[i], b[i])) ** 2 for i in (0, 1))
                    if gap2 <= radius * radius + EPS:
                        selected.append((s, t))
            self.contact_cache[key] = selected
        return self.contact_cache[key]

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

    def route(self, source, facing, speed, target):
        nodes, times = [source], [0.0]
        while nodes[-1] != target:
            _, first = self.table(nodes[-1], facing, speed)
            neighbor = first[target]
            duration, facing = self.edge_time(nodes[-1], neighbor, facing, speed)
            nodes.append(neighbor)
            times.append(times[-1] + duration)
        return nodes, times, facing

    def goal_paths(self, actor, targets, allowed, exposure=None):
        """A simple route to each destination for every legal first exit.

        No repeated-node loops and no excursion into a dead-end arm. This
        replaces unconstrained long DFS forecasts in the active escape policy.
        """
        routes = []
        for first in sorted(self.track.graph[actor.node]):
            if first not in allowed:
                continue
            delay, facing = self.edge_time(actor.node, first, actor.facing, actor.speed)
            initial = first, facing
            best, previous = {initial: delay}, {initial: None}
            queue, serial, found = [(delay, 0, initial)], 0, set()
            while queue:
                cost, _, state = heapq.heappop(queue)
                if cost != best[state]:
                    continue
                u, direction = state
                if u in targets and u not in found:
                    chain, cursor = [], state
                    while cursor is not None:
                        chain.append(cursor[0])
                        cursor = previous[cursor]
                    path = [actor.node] + list(reversed(chain))
                    if len(path) == len(set(path)):
                        routes.append(path)
                        found.add(u)
                for v in sorted(self.track.graph[u]):
                    if v not in allowed or v == actor.node:
                        continue
                    seconds, new_direction = self.edge_time(u, v, direction, actor.speed)
                    candidate = cost + seconds + (exposure(v) if exposure else 0)
                    nxt = v, new_direction
                    if candidate + EPS < best.get(nxt, math.inf):
                        best[nxt], previous[nxt] = candidate, state
                        serial += 1
                        heapq.heappush(queue, (candidate, serial, nxt))
        return routes

    def paths(self, source, facing, speed, horizon=None, limit=48):
        horizon = horizon or self.config["horizon_s"]
        leaves = []

        def visit(path, times, direction, cap):
            if len(leaves) >= cap:
                return
            u = path[-1]
            if times[-1] >= horizon or len(path) >= 40:
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
    escape_straight_m: float = 0.0
    max_escape_straight_m: float = 0.0
    escape_heading: str | None = None
    body_facing: str | None = None

    @property
    def reversing(self):
        return self.body_facing is not None and turn(self.body_facing, self.facing) == "UTURN"

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
            self.phase = "hide" if reason == "hide" else "wait"
            self.remaining = routing.config["sensor_period_s"] if reason == "hide" else .8
            self.target = None
            return
        if neighbor not in track.graph[self.node]:
            raise ValueError("Command must select an adjacent track node")
        duration, direction = routing.edge_time(self.node, neighbor, self.facing, self.speed)
        action = turn(self.facing, direction)
        # Reversing switches wheel direction without rotating the chassis.
        body = self.body_facing or self.facing
        self.body_facing = direction if action in ("LEFT", "RIGHT") else body
        travel = track.graph[self.node][neighbor] * track.spacing / 2 / self.speed
        self.target, self.progress, self.facing = neighbor, 0.0, direction
        delay = max(0.0, duration - travel)
        if reason == "prepare_hide":
            self.phase, self.remaining, self.target = "orient", delay, None
            return
        self.phase, self.remaining = ("reverse" if action == "UTURN" else "turn", delay) if delay > EPS else ("move", travel)

    def advance(self, track, dt):
        if self.phase in ("hide", "wait", "pickup") or self.reason not in ("escape", "cruise", "evade", "preset_evade"):
            self.escape_heading, self.escape_straight_m = None, 0.0
        if self.phase == "move":
            distance = track.graph[self.node][self.target] * track.spacing / 2
            self.progress = min(1.0, self.progress + dt * self.speed / distance)
            self.traveled_m += dt * self.speed
            if self.reason in ("escape", "cruise", "evade", "preset_evade"):
                if self.escape_heading != self.facing:
                    self.escape_straight_m = 0.0
                self.escape_heading = self.facing
                self.escape_straight_m += dt * self.speed
                self.max_escape_straight_m = max(self.max_escape_straight_m, self.escape_straight_m)
        self.remaining = max(0.0, self.remaining - dt)

    def finish(self, track):
        if self.remaining > EPS or self.phase in ("ready", "pickup"):
            return
        if self.phase in ("turn", "reverse"):
            self.phase = "move"
            self.remaining = track.graph[self.node][self.target] * track.spacing / 2 / self.speed
        elif self.phase == "move":
            self.last_node, self.node = self.node, self.target
            self.target, self.progress, self.phase = None, 0.0, "ready"
        else:
            self.phase = "ready"


def detectable(a, b, config, spacing, role=None):
    dx, dy = (b[0] - a[0]) * spacing, (b[1] - a[1]) * spacing
    radius = config.get("police_sensor_range_m", config["sensor_range_m"]) if role == "P" else config["sensor_range_m"]
    if math.hypot(dx, dy) > radius + EPS:
        return False
    # Ideal four cardinal sensors, without physical walls or occlusion.
    angle = math.degrees(math.atan2(abs(dy), abs(dx)))
    return min(angle, 90 - angle) <= config["sensor_half_angle_deg"] + EPS


class Belief:
    """Approximate node/heading filter with motion and negative-observation updates."""
    def __init__(self, initial):
        self.last_seen, self.last_time = initial, 0.0
        self.last_heading, self.seen = None, False
        self.nodes = [initial]
        self.weights = {initial: 1.0}
        self.states = {(initial, None): 1.0}
        self.updated = 0.0
        self.last_xy = None
        self.sensor = None

    def observe(self, ego, other_xy, now, vision, routing, enemy_speed, rng):
        track, config = routing.track, routing.config
        if vision == "fixed":
            self.seen, self.updated, self.sensor = False, now, None
            return
        dt = max(0.0, now - self.updated)
        self.updated = now
        self.seen = vision == "limited" and (detectable(ego.xy(track), other_xy, config, track.spacing, ego.role)
                                            and rng.random() >= config["miss_probability"])
        self.sensor = sensor_channel(ego, other_xy, track) if self.seen else None
        if self.seen:
            measured = min(track.xy, key=lambda n: math.dist(track.xy[n], other_xy))
            # Only successive detections can reveal heading; spawn assumptions
            # must not pretend to measure the opponent's motion.
            if self.last_xy is not None and now - self.last_time <= 2:
                dx, dy = other_xy[0] - self.last_xy[0], other_xy[1] - self.last_xy[1]
                if abs(dx) > EPS or abs(dy) > EPS:
                    self.last_heading = ("E" if dx > 0 else "W") if abs(dx) >= abs(dy) else ("S" if dy > 0 else "N")
                else:
                    # A stationary detection cannot distinguish parking from
                    # turning in place, so the previous travel heading is stale.
                    self.last_heading = None
            else:
                self.last_heading = None
            self.last_xy = tuple(other_xy)
            self.last_seen, self.last_time = measured, now
            self.nodes = [measured]
            self.weights = {measured: 1.0}
            self.states = {(measured, self.last_heading): 1.0}
        else:
            radius = enemy_speed * max(0, now - self.last_time) + track.spacing
            steps = max(1, math.ceil(dt / config["sensor_period_s"] - EPS))
            step, propagated = dt / steps, self.states
            for _ in range(steps):
                previous, propagated = propagated, {}
                for (node, facing), probability in previous.items():
                    options = []
                    for neighbor in track.graph[node]:
                        duration, direction = routing.edge_time(node, neighbor, facing, enemy_speed)
                        preference = .03 if turn(facing, direction) == "UTURN" and len(track.graph[node]) > 1 else 1.0
                        options.append((neighbor, direction, duration, preference))
                    total = sum(option[3] for option in options)
                    moved = 0.0
                    for neighbor, direction, duration, preference in options:
                        amount = probability * preference / total * min(1.0, step / duration)
                        key = neighbor, direction
                        propagated[key] = propagated.get(key, 0) + amount
                        moved += amount
                    key = node, facing
                    propagated[key] = propagated.get(key, 0) + probability - moved
            # A missed detection reduces visible-node likelihood instead of turning
            # the whole reachable map into equally plausible opponent positions.
            kept = {}
            for (node, facing), probability in propagated.items():
                if routing.distance[self.last_seen][node] > radius + EPS:
                    continue
                if detectable(ego.xy(track), track.xy[node], config, track.spacing, ego.role):
                    probability *= max(.02, config["miss_probability"])
                if probability > 1e-14:
                    kept[node, facing] = probability
            total = sum(kept.values())
            if total <= 1e-14:
                kept = {(n, None): 1.0 for n in track.graph
                        if routing.distance[self.last_seen][n] <= radius + EPS}
                total = len(kept)
            self.states = {key: value / total for key, value in kept.items()}
            self.weights = {}
            for (node, _), probability in self.states.items():
                self.weights[node] = self.weights.get(node, 0.0) + probability
            self.nodes = sorted(self.weights, key=lambda n: (-self.weights[n], n))

    def likely(self, limit=8):
        return sorted(self.weights, key=lambda n: (-self.weights[n], n))[:limit]

    def facing_at(self, node):
        candidates = [(weight, direction or "") for (n, direction), weight in self.states.items() if n == node]
        return max(candidates)[1] or None if candidates else None


def capture_delay(police, thief, track, radius_m, maximum_dt):
    """Earliest 2-D contact, including perpendicular traffic and turn waits.

    This is the referee's physical state, never an input to either policy.
    Solve the relative-motion quadratic rather than comparing replay frames.
    """
    p, e = police.xy(track), thief.xy(track)
    pv, ev = police.velocity(track), thief.velocity(track)
    delta = [(p[i] - e[i]) * track.spacing for i in (0, 1)]
    relative = [(pv[i] - ev[i]) * track.spacing for i in (0, 1)]
    c = sum(x * x for x in delta) - radius_m ** 2
    if c <= EPS * EPS:
        return 0.0
    a = sum(v * v for v in relative)
    if a <= EPS * EPS:
        return None
    b = 2 * sum(x * v for x, v in zip(delta, relative))
    discriminant = b * b - 4 * a * c
    if discriminant < -EPS:
        return None
    root = math.sqrt(max(0.0, discriminant))
    enter, leave = (-b - root) / (2 * a), (-b + root) / (2 * a)
    return max(0.0, min(maximum_dt, enter)) if leave >= -EPS and enter <= maximum_dt + EPS else None


class Game:
    def __init__(self, track, config, police_policy="reactive", thief_policy="hide", vision="limited", seed=None,
                 escape_mode="smart"):
        validate_config(config)
        if police_policy not in ("fixed", "reactive") or thief_policy not in ("hide", "escape", "refuge"):
            raise ValueError("Unknown policy")
        if vision not in ("fixed", "limited"):
            raise ValueError("Unknown vision")
        if escape_mode not in ("smart", "preset"):
            raise ValueError("Unknown escape mode")
        self.track, self.config, self.routing = track, config, Routing(track, config)
        self.police_policy, self.thief_policy, self.vision = police_policy, thief_policy, vision
        self.escape_mode = escape_mode
        self.robots = {
            "P": Robot("P", track.resolve("P"), "N", config["police_speed_m_s"]),
            "B": Robot("B", track.resolve("B"), "W", config["thief_speed_m_s"])}
        self.beliefs = {"P": Belief(track.resolve("B")), "B": Belief(track.resolve("P"))}
        seed = config["seed"] if seed is None else seed
        self.rngs = {"P": random.Random(seed), "B": random.Random(seed + 100003)}
        self.owner, self.now, self.winner, self.end_reason = None, 0.0, None, None
        self.frames, self.events, self.pickup_times = [], [], {}
        self.plans, self.holds = {}, {}
        self.visits = {"P": {}, "B": {}}
        core = set(track.graph)
        while True:
            leaves = {n for n in core if sum(v in core for v in track.graph[n]) < 2}
            if not leaves:
                break
            core -= leaves
        self.cycle_core = core
        self.police_checked_treasure = False
        self.police_trail, self.trail_goal = None, None
        self.trail_exits = set()
        self.deadend_goal = None
        self.deadend_checked = {track.resolve("P"): 0.0}
        self.nominal_police = self.routing.route(track.resolve("P"), "N", config["police_speed_m_s"], track.resolve("T"))
        # A likely search continuation is straight ahead after reaching treasure.
        # This is a map-based prior, never the other robot's runtime route.
        approach, arrivals, facing = self.nominal_police
        approach, arrivals = list(approach), list(arrivals)
        while True:
            following = [n for n in track.graph[approach[-1]]
                         if heading(track.xy[approach[-1]], track.xy[n]) == facing]
            if not following:
                break
            neighbor = following[0]
            duration, _ = self.routing.edge_time(approach[-1], neighbor, facing, config["police_speed_m_s"])
            approach.append(neighbor)
            arrivals.append(arrivals[-1] + duration)
        self.nominal_approach = approach, arrivals, facing
        treasure_xy = track.xy[track.resolve("T")]
        incoming_xy = track.xy[self.nominal_police[0][-2]]
        self.safe_block = {"anchor": treasure_xy, "direction": pose_heading(incoming_xy, treasure_xy),
                           "source": "approach_prior"}
        self.fixed_patrol, self.patrol_index = [], 0
        self.hide_goal, self.hide_facing = None, None
        self.hide_banned, self.evade_until = {}, 0.0
        self.hide_decisions = []
        self.escape_plans, self.escape_plan_index = [], -1
        self.active_escape_route, self.escape_route_index = [], 0
        self.route_goal, self.route_purpose = None, None
        self.route_threat = None
        self.navigation_events = []
        self.side_escape = None
        self.response_events = []
        self.threat_active = {"P": False, "B": False}
        self.pending_response = {}
        self.branches = {n for n in core if len(track.graph[n]) >= 3}
        self.pockets = self.build_pockets()
        self.pocket_nodes = {n for members in self.pockets.values() for n in members}
        self.pocket_block = None
        self.pocket_block_cooldown = {}
        self.hide_candidates = [n for n in sorted(core) if self.hide_site_safe(n)]
        self.deadend_arms = self.build_deadend_arms()
        self.search_sites = sorted(n for n in track.graph if landmark_type(track, n))
        self.search_covered, self.negative_counts = set(), {}
        self.arm_clear_counts = {}
        self.search_epoch, self.search_sweeps = 0, []
        self.search_edges = {}
        self.search_route, self.search_route_target = [], None
        self.search_corridors = self.build_search_corridors()
        self.preset_routes = self.build_preset_routes()

    def build_pockets(self):
        """Single-mouth components: two loop directions can share one external exit."""
        pockets = {}
        anchor = self.track.resolve("T")
        for gate in sorted(self.branches):
            remaining = set(self.track.graph) - {gate}
            while remaining:
                stack, component = [min(remaining)], set()
                while stack:
                    node = stack.pop()
                    if node in component:
                        continue
                    component.add(node)
                    stack.extend(n for n in self.track.graph[node] if n != gate and n not in component)
                remaining -= component
                if (anchor not in component and component <= self.cycle_core and len(component) >= 3
                        and all(len(self.track.graph[n]) == 2 for n in component)):
                    pockets.setdefault(gate, set()).update(component)
        return pockets

    def advance_search_route(self, actor):
        if actor.node in self.search_route:
            # Rejoin ahead after pursuit. Skipped nodes remain unobserved, not magically covered.
            index = self.search_route.index(actor.node)
            if index:
                self.search_sweeps.append({"t": round(self.now, 4), "event": "patrol_rejoin",
                                           "node": actor.node, "deferred": self.search_route[:index]})
            del self.search_route[:index + 1]

    def build_search_corridors(self):
        """Maximal straight runs continue through junctions, not just degree-2 nodes."""
        corridors = []
        for source in sorted(self.track.graph):
            for neighbor in sorted(self.track.graph[source]):
                direction = heading(self.track.xy[source], self.track.xy[neighbor])
                if any(heading(self.track.xy[n], self.track.xy[source]) == direction
                       for n in self.track.graph[source]):
                    continue
                path = [source, neighbor]
                while True:
                    forward = [n for n in self.track.graph[path[-1]]
                               if heading(self.track.xy[path[-1]], self.track.xy[n]) == direction]
                    if not forward:
                        break
                    path.append(forward[0])
                length = sum(self.track.graph[u][v] for u, v in zip(path, path[1:])) * self.track.spacing / 2
                if length >= 4 * self.track.spacing - EPS:
                    corridors.append(path)
        return corridors

    def continue_search_route(self, actor):
        self.advance_search_route(actor)
        if self.search_route:
            _, first = self.routing.table(actor.node, actor.facing, actor.speed)
            return first[self.search_route[0]], "search", self.search_route_target
        if self.search_route_target is not None:
            self.search_sweeps.append({"t": round(self.now, 4), "event": "patrol_complete",
                                       "node": self.search_route_target})
            self.search_route_target = None
            self.plans.pop("P", None)
        return None

    def build_deadend_arms(self):
        arms = {}
        for leaf in sorted(self.track.graph):
            if len(self.track.graph[leaf]) != 1:
                continue
            path = [leaf]
            while True:
                following = [n for n in self.track.graph[path[-1]] if n not in path]
                if not following:
                    break
                path.append(following[0])
                if len(self.track.graph[path[-1]]) != 2:
                    break
            arms[leaf] = list(reversed(path))
        return arms

    def hide_site_safe(self, node):
        if node not in self.cycle_core or node in self.pocket_nodes or landmark_type(self.track, node) not in ("corner", "t_junction", "cross") or node == self.track.resolve("T"):
            return False
        span = self.road_span(node)
        limit = self.config.get("hide_max_straight_cells", 3) * self.track.spacing
        return (span <= limit + EPS and self.branches and
                min(self.routing.distance[node][j] for j in self.branches) <= 4 * self.track.spacing + EPS and
                sum(n in self.cycle_core and n not in self.pocket_nodes for n in self.track.graph[node]) >= 2)

    def road_span(self, node):
        return max(self.straight_run(node, "E") + self.straight_run(node, "W"),
                   self.straight_run(node, "N") + self.straight_run(node, "S"))

    def hide_exits(self, node):
        exits = []
        for neighbor in sorted(self.track.graph[node]):
            if neighbor not in self.cycle_core:
                continue
            path = [node, neighbor]
            while len(self.track.graph[path[-1]]) == 2:
                following = [n for n in self.track.graph[path[-1]] if n != path[-2]][0]
                if following in path:
                    break
                path.append(following)
            exits.append(path)
        return exits

    def build_preset_routes(self):
        """Finite map-only cards end at a landmark and never loop back."""
        cards = {}
        for source in sorted(self.track.graph):
            actor = Robot("B", source, None, self.config["thief_speed_m_s"])
            targets = (self.branches | set(self.hide_candidates)) - {source}
            cards[source] = self.routing.goal_paths(actor, targets, self.cycle_core | {source})
            if not cards[source]:
                target = min(self.cycle_core, key=lambda n: (self.routing.distance[source][n], n))
                path, _, _ = self.routing.route(source, None, actor.speed, target)
                cards[source] = [path] if len(path) > 1 else []
        return cards

    def escape_topology(self, path):
        """Separate a straight run from a corridor with no alternative exits.

        A degree-two corner is still a forced corridor; repeated visits to the
        same junction do not create new exits. Merely turning often is no gain.
        """
        run = corridor = longest = longest_corridor = distance = 0.0
        previous, first_branch = None, None
        branches = set()
        turns = 0
        for u, v in zip(path, path[1:]):
            if len(self.track.graph[u]) >= 3 and u in self.cycle_core:
                branches.add(u)
                if first_branch is None:
                    first_branch = distance
                longest_corridor = max(longest_corridor, corridor)
                corridor = 0.0
            direction = heading(self.track.xy[u], self.track.xy[v])
            length = self.track.graph[u][v] * self.track.spacing / 2
            if previous is not None and direction != previous:
                longest = max(longest, run)
                run = 0.0
                turns += 1
            run += length
            corridor += length
            distance += length
            previous = direction
        if path and len(self.track.graph[path[-1]]) >= 3 and path[-1] in self.cycle_core:
            branches.add(path[-1])
            if first_branch is None:
                first_branch = distance
        return {"max_straight_m": round(max(longest, run), 6),
                "max_corridor_m": round(max(longest_corridor, corridor), 6),
                "first_branch_m": round(distance if first_branch is None else first_branch, 6),
                "branch_count": len(branches), "turns": turns, "distance_m": round(distance, 6)}

    def topology_score(self, shape):
        spacing = self.track.spacing
        # A junction-heavy block offers choices; a bend in a single corridor does
        # not. The three-cell threshold is a preference, never a hard constraint.
        excess = max(0, shape["max_straight_m"] / spacing - self.config.get("escape_straight_limit_cells", 3))
        return (min(4, shape["branch_count"]) - 2 * excess
                - shape["max_corridor_m"] / spacing - .5 * shape["first_branch_m"] / spacing
                - .15 * shape["turns"])

    def update_side_escape(self, actor, belief):
        """Latch the opposite half-plane from left/right IR, using sensed data only."""
        if not belief.seen or belief.sensor not in ("left", "right"):
            return
        own = actor.xy(self.track)
        bearing = pose_heading(own, belief.last_xy)
        away = DIRECTIONS[(DIRECTIONS.index(bearing) + 2) % 4]
        cue = self.side_escape
        if cue and cue["direction"] == away and self.now - cue["last_time"] <= self.config.get("pursuit_memory_s", 2):
            cue["last_time"] = self.now
            return
        self.side_escape = {"anchor": tuple(own), "direction": away, "sensor": belief.sensor,
                            "detected_at": self.now, "last_time": self.now}
        self.events.append({"t": round(self.now, 4), "event": "side_detected", "role": "B",
                            "sensor": belief.sensor, "opposite_direction": away,
                            "anchor": tuple(own)})

    def side_preference(self, path, times):
        cue = self.side_escape
        if not cue or self.now - cue["last_time"] > self.config.get("pursuit_memory_s", 2):
            return 0, 0.0
        dx, dy = VECTORS[cue["direction"]]
        anchor = cue["anchor"]
        # Evaluate the next two seconds, not a remote endpoint reached after
        # driving into the opponent's block. The risk filter still comes first.
        values = [(self.track.xy[n][0] - anchor[0]) * dx + (self.track.xy[n][1] - anchor[1]) * dy
                  for n, t in zip(path, times) if t <= 2 + EPS]
        target = self.config.get("side_escape_cells", 2)
        return int(max(values, default=0) >= target - EPS), round(
            min(target, max(values, default=0)) + 2 * min(0, min(values, default=0)), 3)

    def region_projection(self, node):
        dx, dy = VECTORS[self.safe_block["direction"]]
        xy, anchor = self.track.xy[node], self.safe_block["anchor"]
        return (xy[0] - anchor[0]) * dx + (xy[1] - anchor[1]) * dy

    def region_preference(self, path, times):
        values = [self.region_projection(n) for n, t in zip(path[1:], times[1:]) if t <= 3 + EPS]
        if not values:
            values = [self.region_projection(path[1])]
        # A birth/approach prior persists when nothing has been detected. After
        # detection, retain the last observed threat's half-plane as a prior.
        return int(min(values) >= -EPS), round(min(2, max(values)) + 3 * min(0, min(values)), 3)

    def update_safe_block(self, actor, belief):
        if belief.seen and belief.last_xy is not None and math.dist(actor.xy(self.track), belief.last_xy) > EPS:
            self.safe_block = {"anchor": tuple(belief.last_xy),
                               "direction": pose_heading(belief.last_xy, actor.xy(self.track)),
                               "source": "last_detection", "detected_at": belief.last_time}

    def patrol(self, actor):
        """Build a deterministic map sweep, independent of the opponent state."""
        if not self.fixed_patrol:
            unseen = set(self.track.graph) - set(self.nominal_police[0])
            nodes, facing = [actor.node], actor.facing
            while unseen:
                times, _ = self.routing.table(nodes[-1], facing, actor.speed)
                target = min(unseen, key=lambda n: (times[n], n))
                route, _, facing = self.routing.route(nodes[-1], facing, actor.speed, target)
                nodes.extend(route[1:])
                unseen.difference_update(route)
            route, _, _ = self.routing.route(nodes[-1], facing, actor.speed, actor.node)
            nodes.extend(route[1:])
            self.fixed_patrol = nodes
        if self.patrol_index >= len(self.fixed_patrol) - 1:
            self.patrol_index = 0
        if actor.node != self.fixed_patrol[self.patrol_index]:
            raise ValueError("Fixed patrol left its planned route")
        self.patrol_index += 1
        target = self.fixed_patrol[self.patrol_index]
        return target, "patrol", target

    def straight_run(self, node, direction):
        length = 0.0
        while True:
            following = [n for n in self.track.graph[node]
                         if heading(self.track.xy[node], self.track.xy[n]) == direction]
            if not following:
                return length
            target = following[0]
            length += self.routing.distance[node][target]
            node = target

    def choose_hide(self, actor, belief):
        targets = {n for n in self.hide_candidates if self.hide_banned.get(n, -1) <= self.now
                   and n != actor.node and n not in self.nominal_police[0]}
        chosen = self.destination_route(actor, belief, targets, parking=True)
        if chosen is None:
            return False
        path, times, risk = chosen
        self.hide_goal = path[-1]
        arrived = heading(self.track.xy[path[-2]], self.track.xy[path[-1]])
        threat = belief.last_seen if belief.last_xy is not None else self.track.resolve("T")
        exits = self.hide_exits(self.hide_goal)
        exit_path = max(exits, key=lambda p: (self.routing.distance[threat][p[1]],
            -self.timed_path(p, Robot("B", self.hide_goal, heading(self.track.xy[p[0]], self.track.xy[p[1]]), actor.speed))[-1],
            int(heading(self.track.xy[p[0]], self.track.xy[p[1]]) == arrived)))
        self.hide_facing = heading(self.track.xy[exit_path[0]], self.track.xy[exit_path[1]])
        main_route = self.nominal_approach[0]
        exposure = sum(detectable(self.track.xy[self.hide_goal], self.track.xy[n], self.config, self.track.spacing, "P")
                       for n in main_route) / len(main_route)
        self.commit_route(path, risk, "relocate" if self.thief_policy == "refuge" else self.escape_mode, "hide")
        self.hide_decisions.append({"t": round(self.now, 3), "node": self.hide_goal,
                                    "facing": self.hide_facing, "travel_s": round(times[-1], 3),
                                    "nominal_route_exposure": round(exposure, 3),
                                    "straight_escape_m": round(self.straight_run(self.hide_goal, self.hide_facing), 3),
                                    "road_span_m": round(self.road_span(self.hide_goal), 3),
                                    "nearby_junctions": sum(self.routing.distance[self.hide_goal][j] <= 2 * self.track.spacing + EPS for j in self.branches),
                                    "escape_exits": self.hide_exits(self.hide_goal),
                                    "geometry_type": landmark_type(self.track, self.hide_goal),
                                    "safe_block": dict(self.safe_block)})
        return True

    def destination_route(self, actor, belief, targets, parking=False):
        recent = belief.last_xy is not None and self.now - belief.last_time <= self.config.get("pursuit_memory_s", 2)
        main = self.nominal_approach[0]
        allowed = self.cycle_core | {actor.node}
        paths = ([p for p in self.preset_routes[actor.node] if p[-1] in targets]
                 if self.escape_mode == "preset" else self.routing.goal_paths(actor, targets, allowed))
        if not paths:
            return None
        arrivals = self.measured_police_arrivals(belief) if recent else None
        evaluated = []
        for path in paths:
            times = self.timed_path(path, actor)
            risk = self.route_risk(path, times, actor, belief, arrivals) if recent else None
            target = path[-1]
            # Whole-route scores used to reverse at every grid subdivision.
            # Compare departures and destinations, then commit the full route.
            early = [n for n, t in zip(path, times) if t <= 1.5 + EPS]
            if len(early) == 1:
                early.append(path[1])
            if recent:
                threat = belief.last_seen
                separation = min(self.routing.distance[threat][n] for n in early[1:])
                approach, _, _ = self.routing.route(threat, belief.last_heading, self.config["police_speed_m_s"], actor.node)
                joins_approach = sum(n in approach[:-1] for n in early[1:])
                visible_goal = detectable(belief.last_xy, self.track.xy[target], self.config, self.track.spacing, "P")
            else:
                separation = min(self.routing.distance[n][q] for n in early[1:] for q in main)
                joins_approach = sum(n in main for n in early[1:])
                visible_goal = False
            sensing_positions = [q for q in self.track.graph
                                 if detectable(self.track.xy[q], self.track.xy[target], self.config, self.track.spacing, "P")]
            # Favor points that take multiple bends to inspect, even with 3-cell vision.
            search_delay = min(self.routing.table(q, None, self.config["police_speed_m_s"])[0][s]
                               for q in (main[-1], self.track.resolve("T")) for s in sensing_positions)
            nearby = sum(self.routing.distance[target][j] <= 2 * self.track.spacing + EPS for j in self.branches)
            exposures = sum(detectable(self.track.xy[target], self.track.xy[q], self.config, self.track.spacing, "P") for q in main)
            revisits = sum(max(0, 1 - (self.now - self.visits["B"].get(n, -100)) / 12) for n in path[1:])
            utility = (2.5 if self.thief_policy == "refuge" else 1.3) * search_delay + .5 * min(4, nearby)
            utility -= times[-1] + .7 * revisits + exposures + self.road_span(target) / self.track.spacing
            region = self.region_preference(path, times)
            quality = (int(not visible_goal) if parking else 1, -joins_approach, round(separation, 3), region,
                       round(utility, 3), -int(path[1] == actor.last_node))
            if not recent:
                quality = (region, -joins_approach, round(utility, 3), -int(path[1] == actor.last_node))
            evaluated.append((path, times, risk, quality))
        if recent:
            # Do not reward ending a forecast early at a nearby visible stop.
            # Safety of the departure is the gate; a distant worst-case reach
            # envelope must not send the car back toward the pursuer's corridor.
            best_departure = max(min(1.2, item[2][0]) for item in evaluated)
            evaluated = [item for item in evaluated if min(1.2, item[2][0]) >= best_departure - self.config.get("escape_safety_slack_s", .15) - EPS]
            if best_departure <= .35:
                chosen = max(evaluated, key=lambda item: (item[2][0], item[2][1], item[3]))
            else:
                chosen = max(evaluated, key=lambda item: (item[3], item[2][1], item[2][0]))
        else:
            chosen = max(evaluated, key=lambda item: item[3])
        return chosen[:3]

    def commit_route(self, path, risk, mode, purpose="escape"):
        self.active_escape_route, self.escape_route_index = list(path), 0
        self.route_goal, self.route_purpose = path[-1], purpose
        self.save_escape_plan(path, risk, mode)
        self.navigation_events.append({"t": round(self.now, 4), "event": "route_committed", "goal": path[-1],
                                       "nodes": list(path), "purpose": purpose})

    def committed_step(self, actor, belief):
        route, index = self.active_escape_route, self.escape_route_index
        if route and index + 1 < len(route) and route[index + 1] == actor.node:
            index += 1
        if not route or index >= len(route) or route[index] != actor.node:
            return None
        self.escape_route_index = index
        if index + 1 >= len(route):
            return None
        remainder = route[index:]
        # A fresh observation can invalidate the next leg, never an old
        # pessimistic 8-second envelope or a change of relative sensor label.
        if belief.seen and belief.last_xy is not None:
            checkpoint = 1
            while checkpoint + 1 < len(remainder) and landmark_type(self.track, remainder[checkpoint]) is None:
                checkpoint += 1
            leg = remainder[:checkpoint + 1]
            risk = self.route_risk(leg, self.timed_path(leg, actor), actor, belief)
            if risk[0] <= self.config.get("escape_replan_contact_s", .3) and risk[1] < 0:
                self.navigation_events.append({"t": round(self.now, 4), "event": "fresh_conflict", "node": actor.node,
                                               "previous_goal": self.route_goal})
                self.active_escape_route = []
                return None
        return remainder[1]

    def danger_nearby(self, actor, belief):
        if self.vision != "limited" or not (belief.seen or belief.last_time > 0):
            return False
        age = self.now - belief.last_time
        if age > max(self.config["turn90_s"], self.config["uturn_s"]) + 2:
            return False
        police_speed = self.config["police_speed_m_s"]
        times, _ = self.routing.table(belief.last_seen, belief.last_heading, police_speed)
        # Allow for a half-cell quantization error, contact threshold and reaction.
        approach = times[actor.node] - age - (self.config["capture_center_distance_m"] + self.track.spacing / 2) / police_speed
        leave = min(self.routing.edge_time(actor.node, n, actor.facing, actor.speed)[0]
                    for n in self.track.graph[actor.node])
        return approach <= leave + self.config.get("hide_reaction_margin_s", 1.0)

    def begin_evade(self):
        actor = self.robots["B"]
        if self.hide_goal is not None and actor.node == self.hide_goal:
            self.hide_banned[self.hide_goal] = self.now + self.config.get("hide_revisit_cooldown_s", 30.0)
            self.hide_goal = None
            self.active_escape_route = []
        self.evade_until = self.now + self.config.get("hide_escape_hold_s", 6.0)

    def hide(self, actor, belief):
        if belief.seen and self.vision == "limited":
            self.begin_evade()
        if self.hide_goal is not None and not self.hide_site_safe(self.hide_goal):
            self.hide_goal = None
            self.active_escape_route = []
        escaping = self.now < self.evade_until
        if self.hide_goal is not None and actor.node != self.hide_goal:
            neighbor = self.committed_step(actor, belief)
            if neighbor is not None:
                return neighbor, "preset_evade" if escaping and self.escape_mode == "preset" else "evade" if escaping else "seek_hide", self.hide_goal
            self.hide_goal = None
        if self.hide_goal is None and not self.choose_hide(actor, belief):
            return self.escape(actor, belief)
        if actor.node != self.hide_goal:
            return self.active_escape_route[1], "preset_evade" if escaping and self.escape_mode == "preset" else "evade" if escaping else "seek_hide", self.hide_goal
        if actor.facing != self.hide_facing:
            neighbor = next(n for n in self.track.graph[actor.node]
                            if heading(self.track.xy[actor.node], self.track.xy[n]) == self.hide_facing)
            return neighbor, "prepare_hide", self.hide_goal
        self.evade_until = self.now
        self.active_escape_route = []
        return None, "hide", self.hide_goal

    def search(self, actor, belief):
        """Commit a complete observation route; absence alone never cancels it."""
        continued = self.continue_search_route(actor)
        if continued:
            return continued
        times, next_nodes = self.routing.table(actor.node, actor.facing, actor.speed)
        remaining = set(self.search_sites) - self.search_covered
        if not remaining:
            self.search_sweeps.append({"t": round(self.now, 4), "event": "sweep_complete", "round": self.search_epoch,
                                       "covered": len(self.search_covered)})
            self.search_epoch += 1
            self.search_covered.clear()
            self.negative_counts.clear()
            remaining = set(self.search_sites)
        plan = self.plans.get(actor.role)
        if plan and plan[0] == "search" and plan[1] != actor.node and self.now < plan[2]:
            self.search_route = self.routing.route(actor.node, actor.facing, actor.speed, plan[1])[0][1:]
            self.search_route_target = plan[1]
            return self.continue_search_route(actor)
        best = None
        for target in self.track.graph:
            visible = [n for n in remaining if detectable(self.track.xy[target], self.track.xy[n], self.config, self.track.spacing, "P")]
            if not visible:
                continue
            path, _, _ = self.routing.route(actor.node, actor.facing, actor.speed, target)
            repeated = sum(min(3, self.search_edges.get(tuple(sorted((u, v))), 0)) for u, v in zip(path, path[1:]))
            gain = sum(3 if n in self.hide_candidates else 1 for n in visible)
            mass = sum(belief.weights.get(n, 0) for n in visible)
            score = (gain + mass) / (.35 + times[target] + .12 * repeated)
            item = (score, -times[target], target, visible)
            if best is None or item > best:
                best = item
        chosen_path = self.routing.route(actor.node, actor.facing, actor.speed, best[2])[0]
        route_kind = "observation"
        for corridor in self.search_corridors:
            # If already on a run, continue towards its end instead of detouring to its start.
            run = corridor[corridor.index(actor.node):] if actor.node in corridor else corridor
            if len(run) < 2:
                continue
            approach, etas, facing = self.routing.route(actor.node, actor.facing, actor.speed, run[0])
            path = approach + run[1:]
            if len(set(path)) != len(path):
                continue
            visible = sorted(n for n in remaining if any(
                detectable(self.track.xy[u], self.track.xy[n], self.config, self.track.spacing, "P") for u in run))
            if not visible:
                continue
            duration = etas[-1]
            for u, v in zip(run, run[1:]):
                seconds, facing = self.routing.edge_time(u, v, facing, actor.speed)
                duration += seconds
            repeated = sum(min(3, self.search_edges.get(tuple(sorted((u, v))), 0)) for u, v in zip(path, path[1:]))
            gain = sum(3 if n in self.hide_candidates else 1 for n in visible)
            mass = sum(belief.weights.get(n, 0) for n in visible)
            # A modest preference for continuous sweeps, still requiring new coverage.
            score = 1.25 * (gain + mass) / (.35 + duration + .12 * repeated)
            item = (score, -duration, run[-1], visible)
            if item > best:
                best, chosen_path, route_kind = item, path, "straight_sweep"
        target = best[2]
        self.search_sweeps.append({"t": round(self.now, 4), "event": "observation_goal", "round": self.search_epoch,
                                   "node": target, "new_sites": best[3], "already_covered": len(self.search_covered),
                                   "route": chosen_path, "kind": route_kind})
        self.plans[actor.role] = ("search", target, self.now + max(6.0, times[target] + 1))
        self.search_route, self.search_route_target = chosen_path[1:], target
        return self.continue_search_route(actor) or (None, "search", target)

    def update_search_coverage(self):
        if self.vision != "limited":
            return
        actor, belief = self.robots["P"], self.beliefs["P"]
        own = actor.xy(self.track)
        if belief.seen:
            self.negative_counts.clear()
            self.arm_clear_counts.clear()
            self.search_covered.discard(belief.last_seen)
            return
        for node in self.search_sites:
            if detectable(own, self.track.xy[node], self.config, self.track.spacing, "P"):
                self.negative_counts[node] = self.negative_counts.get(node, 0) + 1
                if self.negative_counts[node] >= 3:
                    self.search_covered.add(node)
            else:
                self.negative_counts[node] = 0
        for leaf, path in self.deadend_arms.items():
            # Include midpoints: a narrow IR cone can miss a bend in an arm.
            points = [self.track.xy[n] for n in path]
            points += [tuple((self.track.xy[u][i] + self.track.xy[v][i]) / 2 for i in (0, 1))
                       for u, v in zip(path, path[1:])]
            visible = all(detectable(own, xy, self.config, self.track.spacing, "P") for xy in points)
            self.arm_clear_counts[leaf] = self.arm_clear_counts.get(leaf, 0) + 1 if visible else 0
            if self.arm_clear_counts[leaf] < 3:
                continue
            if self.deadend_goal == leaf and self.police_trail and self.now - self.police_trail["last_time"] < self.config.get("pursuit_memory_s", 2):
                continue
            if self.now - self.deadend_checked.get(leaf, -math.inf) >= self.config.get("police_deadend_recheck_s", 20):
                self.events.append({"t": round(self.now, 4), "event": "deadend_cleared_by_sensor", "role": "P", "node": leaf})
            self.deadend_checked[leaf] = self.now
            if self.deadend_goal == leaf:
                self.deadend_goal = None

    def remember_police_trail(self, actor, belief):
        if not belief.seen:
            return
        previous = self.police_trail or {}
        changed = previous.get("node") != belief.last_seen
        if belief.last_time > previous.get("last_time", -math.inf):
            self.trail_goal = None
        if changed:
            self.trail_goal = None
            self.trail_exits.clear()
        self.police_trail = {"node": belief.last_seen, "xy": belief.last_xy,
                             "last_time": belief.last_time,
                             "direction": belief.last_heading or previous.get("direction"),
                             "reached": actor.node == belief.last_seen or (not changed and previous.get("reached", False)),
                             "extended": False}
        if belief.sensor in ("left", "right"):
            arms = [leaf for leaf, path in self.deadend_arms.items() if belief.last_seen in path[1:]]
            if arms:
                target = min(arms, key=lambda n: self.routing.distance[belief.last_seen][n])
                if self.deadend_goal != target:
                    self.events.append({"t": round(self.now, 4), "event": "side_deadend_target", "role": "P", "node": target})
                self.deadend_goal = target
        if self.deadend_goal and belief.last_seen not in self.deadend_arms[self.deadend_goal][1:]:
            self.deadend_goal = None

    def inspect_deadends(self, actor):
        """Local one-bend checks; positive side detection can force a return."""
        if self.deadend_goal == actor.node:
            self.deadend_checked[actor.node] = self.now
            self.events.append({"t": round(self.now, 4), "event": "deadend_checked", "role": "P", "node": actor.node})
            self.deadend_goal = None
        times, next_nodes = self.routing.table(actor.node, actor.facing, actor.speed)
        if self.deadend_goal is None:
            options = []
            for leaf in self.deadend_arms:
                if leaf in self.search_covered:
                    continue
                if self.now - self.deadend_checked.get(leaf, -math.inf) < self.config.get("police_deadend_recheck_s", 20):
                    continue
                path, etas, _ = self.routing.route(actor.node, actor.facing, actor.speed, leaf)
                if self.routing.distance[actor.node][leaf] > self.config.get("police_deadend_check_cells", 4) * self.track.spacing + EPS:
                    continue
                directions = [actor.facing] + [heading(self.track.xy[u], self.track.xy[v]) for u, v in zip(path, path[1:])]
                bends = sum(turn(a, b) in ("LEFT", "RIGHT") for a, b in zip(directions, directions[1:]))
                if bends <= 1 and leaf != actor.node:
                    options.append((times[leaf], leaf))
            if not options:
                return None
            self.deadend_goal = min(options)[1]
        return next_nodes[self.deadend_goal], "check_deadend", self.deadend_goal

    def follow_trail(self, actor, belief):
        trail = self.police_trail
        if self.deadend_goal:
            check = self.inspect_deadends(actor)
            if check:
                return check
        if not trail or self.now - trail["last_time"] > self.config.get("police_search_memory_s", 12):
            return self.search(actor, belief)
        times, first = self.routing.table(actor.node, actor.facing, actor.speed)
        hint = trail["direction"]
        if hint and trail.get("xy") is not None:
            a, b = self.track.xy[actor.node], trail["xy"]
            dx, dy = VECTORS[hint]
            # Already beyond the measured position on its travel axis: don't chase stale coordinates backwards.
            if abs((a[0]-b[0])*dy-(a[1]-b[1])*dx) < EPS and (a[0]-b[0])*dx+(a[1]-b[1])*dy >= -EPS:
                trail["reached"] = True
        if not trail["reached"]:
            if actor.node != trail["node"]:
                return first[trail["node"]], "trail", trail["node"]
            trail["reached"] = True
        if self.trail_goal and actor.node != self.trail_goal:
            return first[self.trail_goal], "trail", self.trail_goal
        self.trail_goal = None
        if trail.get("extended"):
            return self.search(actor, belief)
        neighbors = list(self.track.graph[actor.node])
        unchecked = [n for n in neighbors if (actor.node, n) not in self.trail_exits]
        if unchecked:
            neighbors = unchecked
        if len(neighbors) > 1:
            onward = [n for n in neighbors if n != actor.last_node]
            if onward:
                neighbors = onward
        hint = trail["direction"] or actor.facing
        def priority(n):
            direction = heading(self.track.xy[actor.node], self.track.xy[n])
            probability = sum(belief.weights.get(v, 0) for v in self.track.graph[n]) + belief.weights.get(n, 0)
            return (int(direction == hint), probability, -self.visits["P"].get(n, -100), n)
        neighbor = max(neighbors, key=priority)
        self.trail_exits.add((actor.node, neighbor))
        path = [actor.node, neighbor]
        straight = heading(self.track.xy[actor.node], self.track.xy[neighbor])
        while True:
            forward = [n for n in self.track.graph[path[-1]] if n not in path and
                       heading(self.track.xy[path[-1]], self.track.xy[n]) == straight]
            if forward:
                path.append(forward[0])
                continue
            # A turn is an actual boundary of this straight-line follow-up.
            break
        self.trail_goal = path[-1]
        trail["extended"] = True
        return neighbor, "trail", self.trail_goal

    def chase(self, actor, belief):
        if self.now - belief.last_time > self.config.get("pursuit_memory_s", 2.0):
            return self.follow_trail(actor, belief)
        if self.deadend_goal:
            check = self.inspect_deadends(actor)
            if check:
                return check
        if not belief.seen and self.police_trail:
            return self.follow_trail(actor, belief)
        times, next_nodes = self.routing.table(actor.node, actor.facing, actor.speed)
        targets = [belief.last_seen] if belief.last_seen != actor.node else []
        if not targets:
            # Use measured bearing within the same node, not an unseen true pose.
            if belief.last_xy is not None:
                a = self.track.xy[actor.node]
                dx, dy = belief.last_xy[0] - a[0], belief.last_xy[1] - a[1]
                direction = ("E" if dx > 0 else "W") if abs(dx) >= abs(dy) else ("S" if dy > 0 else "N")
                forward = [n for n in self.track.graph[actor.node]
                           if heading(a, self.track.xy[n]) == direction]
                if forward and abs(dx) + abs(dy) > EPS:
                    return forward[0], "chase", belief.last_seen
            targets = list(self.track.graph[actor.node])
        target = min(targets, key=lambda n: (self.routing.distance[belief.last_seen][n], times[n]))
        return next_nodes[target], "chase", target

    def escape_score(self, path, times, threats, pursuer_speed, facing=None, threat_facing=None,
                     weights=None, horizon=None):
        """Compare safety during turn waits as well as at future arrival nodes."""
        horizon = horizon or self.config["horizon_s"]
        samples, turn_seconds, distance, direction = [], 0.0, 0.0, facing
        for u, v, start, end in zip(path, path[1:], times, times[1:]):
            length = self.track.graph[u][v] * self.track.spacing / 2
            new_direction = heading(self.track.xy[u], self.track.xy[v])
            action = turn(direction, new_direction)
            delay = self.config["turn90_s"] if action in ("LEFT", "RIGHT") else self.config["uturn_s"] if action == "UTURN" else 0
            if delay:
                samples.append((u, start + delay))
            samples.append((v, end))
            turn_seconds += delay
            distance += length
            direction = new_direction
        samples.append((path[-1], max(times[-1], horizon)))
        table = {q: self.routing.table(q, threat_facing, pursuer_speed)[0] for q in threats}
        probabilities = weights or {q: 1 / len(threats) for q in threats}
        norm = sum(probabilities.get(q, 0) for q in threats)
        safety, margin = 0.0, 0.0
        for q in threats:
            arrival = table[q]
            clearance = self.config["capture_center_distance_m"] / pursuer_speed
            earliest = min((max(0, arrival[n] - clearance) for n, t in samples
                            if arrival[n] - clearance <= t), default=horizon)
            worst = min(arrival[n] - elapsed - clearance for n, elapsed in samples)
            probability = probabilities.get(q, 0) / norm
            safety += probability * min(horizon, earliest)
            margin += probability * max(-horizon, min(horizon, worst))
        trapped = path[-1] not in self.cycle_core
        # Safety first; among comparable routes prefer fewer costly turns and
        # a useful continuous run. A dead end is penalized, never an absolute veto.
        return (round(safety, 1), round(margin, 1) - (2 if trapped else 0),
                -turn_seconds, distance, len(self.track.graph[path[-1]]))

    def timed_path(self, path, actor):
        times, facing = [0.0], actor.facing
        for u, v in zip(path, path[1:]):
            duration, facing = self.routing.edge_time(u, v, facing, actor.speed)
            times.append(times[-1] + duration)
        return times

    def measured_police_arrivals(self, belief):
        """Earliest map arrivals from a detected pose; no true opponent state."""
        speed = self.config["police_speed_m_s"]
        if belief.last_xy is None:
            return self.routing.table(belief.last_seen, belief.last_heading, speed)[0]
        pose, roots = belief.last_xy, []
        for node, xy in self.track.xy.items():
            if math.dist(pose, xy) < EPS:
                roots.append((node, belief.last_heading, 0.0))
        if not roots:
            for u in self.track.graph:
                for v in self.track.graph[u]:
                    if u >= v:
                        continue
                    a, b = self.track.xy[u], self.track.xy[v]
                    if math.dist(a, pose) + math.dist(pose, b) <= math.dist(a, b) + EPS:
                        for node in (u, v):
                            xy = self.track.xy[node]
                            direction = pose_heading(pose, xy)
                            action = turn(belief.last_heading, direction)
                            delay = self.config["turn90_s"] if action in ("LEFT", "RIGHT") else self.config["uturn_s"] if action == "UTURN" else 0
                            roots.append((node, direction, delay + math.dist(pose, xy) * self.track.spacing / speed))
        if not roots:
            roots = [(belief.last_seen, belief.last_heading, 0.0)]
        tables = [(self.routing.table(node, facing, speed)[0], delay) for node, facing, delay in roots]
        return {n: min(table[n] + delay for table, delay in tables) for n in self.track.graph}

    def route_risk(self, path, times, actor, belief, arrivals=None):
        """Sample motion and turn waits against earliest possible police arrivals.

        Mid-edge samples catch opposing traffic even when endpoint arrival
        comparisons look safe. This is a conservative reachable-set estimate,
        not access to the police controller or a guarantee against capture.
        """
        arrivals = arrivals or self.measured_police_arrivals(belief)
        speed, horizon = self.config["police_speed_m_s"], self.config["horizon_s"]
        age = max(0.0, self.now - belief.last_time)
        radius = self.config["capture_center_distance_m"] / self.track.spacing
        first_risk, early_margin, worst_margin = horizon, math.inf, math.inf
        turning = 0.0
        for u, v, start, end in zip(path, path[1:], times, times[1:]):
            if start > horizon:
                break
            a, b = self.track.xy[u], self.track.xy[v]
            length = math.dist(a, b) * self.track.spacing
            delay = end - start - length / actor.speed
            moving_at = start + delay
            turning += delay
            segments = self.routing.contact_segments(u, v)
            count = max(1, math.ceil((min(end, horizon) - start) / self.config.get("escape_risk_step_s", .05)))
            for i in range(count + 1):
                elapsed = start + (min(end, horizon) - start) * i / count
                fraction = max(0.0, min(1.0, (elapsed - moving_at) * actor.speed / length))
                xy = (a[0] + (b[0] - a[0]) * fraction, a[1] + (b[1] - a[1]) * fraction)
                eta = math.inf
                for s, t in segments:
                    c, d = self.track.xy[s], self.track.xy[t]
                    axis = 0 if abs(c[1] - d[1]) < EPS else 1
                    offset = abs(c[1 - axis] - xy[1 - axis])
                    if offset > radius + EPS:
                        continue
                    reach = math.sqrt(max(0, radius ** 2 - offset ** 2))
                    low = max(min(c[axis], d[axis]), xy[axis] - reach)
                    high = min(max(c[axis], d[axis]), xy[axis] + reach)
                    if low > high + EPS:
                        continue
                    for n, point in ((s, c), (t, d)):
                        distance = abs(point[axis] - max(low, min(high, point[axis]))) * self.track.spacing
                        eta = min(eta, arrivals[n] + distance / speed)
                    if belief.last_xy is not None and abs(belief.last_xy[1 - axis] - c[1 - axis]) < EPS:
                        contact = list(c)
                        contact[axis] = max(low, min(high, belief.last_xy[axis]))
                        if self.routing.straight_connected(belief.last_xy, contact):
                            direct = math.dist(belief.last_xy, contact) * self.track.spacing / speed
                            if direct > EPS:
                                action = turn(belief.last_heading, pose_heading(belief.last_xy, contact))
                                direct += self.config["turn90_s"] if action in ("LEFT", "RIGHT") else self.config["uturn_s"] if action == "UTURN" else 0
                            eta = min(eta, direct)
                margin = eta - age - elapsed
                worst_margin = min(worst_margin, margin)
                if elapsed <= 1:
                    early_margin = min(early_margin, margin)
                if margin <= EPS:
                    first_risk = min(first_risk, elapsed)
        return round(first_risk, 3), round(early_margin, 3), round(worst_margin, 3), -round(turning, 3)

    def save_escape_plan(self, path, score, mode):
        cue = self.side_escape
        side = dict(cue) if cue and self.now - cue["last_time"] <= self.config.get("pursuit_memory_s", 2) else None
        self.escape_plans.append({"t": round(self.now, 3), "mode": mode, "nodes": path,
                                  "first_possible_contact_s": score[0] if score else None,
                                  "early_margin_s": score[1] if score else None,
                                  "topology": self.escape_topology(path), "side_escape": side,
                                  "safe_block": dict(self.safe_block)})
        self.escape_plan_index = len(self.escape_plans) - 1

    def select_escape_path(self, candidates, actor, belief, recent, arrivals=None):
        evaluated = []
        threats = belief.likely(8) if not recent else []
        for path, times in candidates:
            risk = self.route_risk(path, times, actor, belief, arrivals) if recent else None
            shape = self.escape_topology(path)
            # Recency discourages orbiting one tiny loop after losing detection.
            visited = sum(max(0, 1 - (self.now - self.visits["B"].get(n, -math.inf)) / 10)
                          for n in set(path[1:]))
            quality = self.topology_score(shape) - .6 * visited
            quality -= .4 * int(path[1] == actor.last_node)
            if not recent:
                expected = self.escape_score(path, times, threats, self.config["police_speed_m_s"],
                                             actor.facing, None, belief.weights)
                quality += .4 * (expected[0] + .2 * expected[1])
            side = self.side_preference(path, times) + self.region_preference(path, times)
            evaluated.append((path, times, risk, quality, side))
        if recent:
            best_contact = max(item[2][0] for item in evaluated)
            slack = self.config.get("escape_safety_slack_s", .15)
            evaluated = [item for item in evaluated if item[2][0] >= best_contact - slack - EPS]
            core_routes = [item for item in evaluated if all(n in self.cycle_core for n in item[0][1:])]
            if actor.node in self.cycle_core and core_routes:
                evaluated = core_routes
            if best_contact <= 1:
                # In imminent danger, an expensive turn can lose immediately.
                # Survive first; do not force an opposite-side maneuver.
                key = lambda item: (item[2][0], item[2][1], item[4], item[3], item[2][2], item[2][3])
            else:
                if any(item[2][1] >= 0 for item in evaluated):
                    evaluated = [item for item in evaluated if item[2][1] >= 0]
                # Among similarly safe routes, prefer the opposite block and
                # available branches rather than maximizing straight-line speed.
                key = lambda item: (item[4], item[3], item[2][0], item[2][1], item[2][2], item[2][3])
        else:
            key = lambda item: (int(all(n in self.cycle_core for n in item[0][1:])),
                                item[4], item[3], -int(item[0][1] == actor.last_node), -item[1][-1])
        chosen = max(evaluated, key=key)
        return chosen[:3]

    def escape(self, actor, belief):
        if self.escape_mode == "preset":
            return self.preset_escape(actor, belief)
        neighbor = self.committed_step(actor, belief)
        recent = (belief.seen or belief.last_time > 0) and self.now - belief.last_time <= self.config.get("pursuit_memory_s", 2.0)
        if neighbor is not None:
            return neighbor, "escape" if recent else "cruise", self.route_goal
        targets = self.branches | set(self.hide_candidates)
        targets = {n for n in targets if n != actor.node and self.routing.distance[actor.node][n] >= 2 * self.track.spacing - EPS}
        chosen = self.destination_route(actor, belief, targets)
        if chosen is None:
            # Recover from the initial spawn arm; active core routes never enter it.
            neighbors = list(self.track.graph[actor.node])
            neighbor = min(neighbors, key=lambda n: min(self.routing.distance[n][q] for q in self.cycle_core))
            return neighbor, "escape", neighbor
        path, times, risk = chosen
        self.commit_route(path, risk, "smart")
        return path[1], "escape" if recent else "cruise", path[-1]

    def preset_escape(self, actor, belief):
        """Follow a stored route; a detected conflict may select another card."""
        neighbor = self.committed_step(actor, belief)
        if neighbor is not None:
            return neighbor, "preset_evade", self.route_goal
        # The stored map is sufficient to create finite destination cards.
        # A card ends at a real landmark, not a cyclic DFS forecast.
        candidates = [(p, self.timed_path(p, actor)) for p in self.preset_routes[actor.node]
                      if len(p) == len(set(p)) and all(n in self.cycle_core for n in p[1:])]
        recent = belief.last_xy is not None and self.now - belief.last_time <= self.config.get("pursuit_memory_s", 2)
        if not candidates:
            chosen = self.destination_route(actor, belief, (self.branches | set(self.hide_candidates)) - {actor.node})
            if chosen is None:
                neighbor = next(iter(self.track.graph[actor.node]))
                return neighbor, "preset_evade", neighbor
            path, times, score = chosen
        else:
            path, times, score = self.select_escape_path(candidates, actor, belief, recent)
        self.commit_route(path, score, "preset")
        return path[1], "preset_evade", path[-1]

    def predicted_routes(self, actor, belief):
        """Plausible routes using observations and the locally inspected treasure."""
        predicted = []
        enemy_speed = self.config["thief_speed_m_s"]
        for root in belief.likely(6):
            direction = belief.facing_at(root)
            prefix, elapsed = [root], [0.0]
            if not self.police_checked_treasure:
                node = root
                while node != self.track.resolve("T"):
                    _, next_nodes = self.routing.table(node, direction, enemy_speed)
                    target = next_nodes[self.track.resolve("T")]
                    duration, direction = self.routing.edge_time(node, target, direction, enemy_speed)
                    prefix.append(target)
                    elapsed.append(elapsed[-1] + duration)
                    node = target
            paths = self.routing.paths(prefix[-1], direction, enemy_speed, limit=24)
            paths.sort(key=lambda item: self.escape_score(*item, [actor.node], actor.speed,
                                                         direction, actor.facing), reverse=True)
            # Keep different first exits instead of three near-identical paths.
            selected, first_steps = [], set()
            for path, times in paths:
                if path[1] not in first_steps:
                    selected.append((path, times))
                    first_steps.add(path[1])
            for path, times in paths:
                if len(selected) >= 5:
                    break
                if (path, times) not in selected:
                    selected.append((path, times))
            for path, times in selected:
                delay = elapsed[-1] + (self.config["pickup_s"] if not self.police_checked_treasure else 0)
                predicted.append((prefix + path[1:], elapsed + [delay + t for t in times[1:]],
                                  belief.weights[root] / max(1, len(selected))))
        return predicted

    def intercept(self, actor, belief):
        if self.now - belief.last_time > 2.0:
            return self.search(actor, belief)
        predicted = self.predicted_routes(actor, belief)
        if not predicted:
            return self.chase(actor, belief)
        times, next_nodes = self.routing.table(actor.node, actor.facing, actor.speed)
        targets = {n for path, _, _ in predicted for n in path[1:]}
        best = None
        for target in sorted(targets):
            arrival = times[target]
            if target == actor.node:
                start = self.holds.get((actor.role, target), self.now)
                if self.now < 5.0 or self.now - start >= 1.6 - EPS:
                    continue
            windows = [(min((eta for n, eta in zip(path, etas) if n == target), default=math.inf), weight)
                       for path, etas, weight in predicted]
            covered = [(eta, weight) for eta, weight in windows if arrival <= eta + .2 and math.isfinite(eta)
                       and (target != actor.node or eta <= 1.6)]
            if not covered:
                continue
            coverage = sum(weight for _, weight in covered)
            mean_eta = sum(eta * weight for eta, weight in covered) / coverage
            next_node = next_nodes[target]
            reverse_cost = self.config["uturn_s"] if next_node == actor.last_node else 0.0
            # Discount distant forecasts and expensive reversals. A nearby,
            # achievable cut-off should beat chasing the target's old position.
            score = (coverage / (2 + mean_eta + .25 * arrival + .5 * reverse_cost), -arrival)
            if best is None or score > best[0]:
                best = score, target
        if best is None:
            return self.chase(actor, belief)
        target = best[1]
        if target == actor.node:
            self.holds.setdefault((actor.role, target), self.now)
        return next_nodes[target], "hold" if target == actor.node else "intercept", target

    def block_pocket_exit(self, actor, belief):
        """Use only a finite detection and map bottleneck, never hidden thief state."""
        if belief.seen:
            gates = [g for g, members in self.pockets.items() if belief.last_seen in members and actor.node not in members]
            if not gates:
                self.pocket_block = None
                return None
            times, _ = self.routing.table(actor.node, actor.facing, actor.speed)
            feasible = False
            for gate in sorted(gates, key=lambda g: times[g]):
                if self.pocket_block_cooldown.get(gate, -1) > self.now:
                    continue
                speed = self.config["thief_speed_m_s"]
                thief_time = self.routing.table(belief.last_seen, None, speed)[0][gate]
                thief_time = max(0, thief_time - math.dist(belief.last_xy, self.track.xy[belief.last_seen])*self.track.spacing/speed)
                if times[gate] > thief_time + EPS:
                    continue
                feasible = True
                if not self.pocket_block or self.pocket_block["gate"] != gate:
                    self.pocket_block = {"gate":gate, "observed_at":self.now, "arrived_at":None}
                    self.events.append({"t":round(self.now, 4), "role":"P", "event":"pocket_exit_target",
                                        "gate":gate, "observed_node":belief.last_seen,
                                        "police_eta_s":round(times[gate],4), "thief_earliest_eta_s":round(thief_time,4)})
                self.pocket_block["observed_at"] = self.now
                break
            if not feasible:
                self.pocket_block = None
        block = self.pocket_block
        if not block:
            return None
        gate = block["gate"]
        if self.now-block["observed_at"] > self.config.get("police_search_memory_s",12):
            self.pocket_block = None
            return None
        if actor.node == gate:
            if block["arrived_at"] is None:
                block["arrived_at"] = self.now
            if self.now-block["arrived_at"] >= 2:
                self.pocket_block_cooldown[gate] = self.now+4
                self.pocket_block = None
                return None
            return None, "cutoff", gate
        return self.routing.table(actor.node, actor.facing, actor.speed)[1][gate], "cutoff", gate

    def decide(self, role):
        actor, belief = self.robots[role], self.beliefs[role]
        # Reaching pending waypoints while pursuing still advances the saved patrol.
        if role == "P":
            self.advance_search_route(actor)
        recent = (belief.seen or self.threat_active[role]) and self.now - belief.last_time <= self.config.get("pursuit_memory_s", 2.0)
        if self.vision == "limited":
            if role == "P" and self.police_policy == "reactive":
                self.remember_police_trail(actor, belief)
                block = self.block_pocket_exit(actor, belief)
                if block:
                    return block
                if recent:
                    return self.chase(actor, belief)
                if self.police_trail:
                    return self.follow_trail(actor, belief)
            if role == "B" and self.owner == "B" and self.thief_policy in ("hide", "refuge"):
                return self.hide(actor, belief)
            if role == "B" and (belief.seen or self.now < self.evade_until):
                if belief.seen:
                    self.begin_evade()
                neighbor, _, target = self.escape(actor, belief)
                return neighbor, "preset_evade" if self.escape_mode == "preset" else "evade", target
        # The police cannot remotely read the simulator's treasure ownership.
        needs_treasure = not self.police_checked_treasure if role == "P" else self.owner != "B"
        if needs_treasure:
            _, next_nodes = self.routing.table(actor.node, actor.facing, actor.speed)
            return next_nodes[self.track.resolve("T")], "treasure", self.track.resolve("T")
        if role == "B":
            if self.thief_policy in ("hide", "refuge"):
                return self.hide(actor, belief)
            return self.escape(actor, belief)
        if self.police_policy == "fixed" or self.vision == "fixed":
            return self.patrol(actor)
        return self.search(actor, belief)

    def observe(self):
        # Both policies observe the same instant before either command is applied.
        for role, opposite in (("P", "B"), ("B", "P")):
            self.beliefs[role].observe(self.robots[role], self.robots[opposite].xy(self.track),
                                      self.now, self.vision, self.routing, self.robots[opposite].speed, self.rngs[role])
        self.update_search_coverage()
        # Sensing changes the behavior immediately. A physical turn already in
        # progress must finish; movement reaches the next node before branching.
        for role, actor in self.robots.items():
            belief = self.beliefs[role]
            if not belief.seen or role == "P" and self.police_policy != "reactive":
                if self.now - belief.last_time > self.config.get("pursuit_memory_s", 2.0):
                    self.threat_active[role] = False
                continue
            if not self.threat_active[role]:
                self.events.append({"t": round(self.now, 4), "event": "detected", "role": role,
                                    "measured_xy": belief.last_xy, "phase": actor.phase})
                self.pending_response[role] = self.now
            self.threat_active[role] = True
            self.plans.pop(role, None)
            if role == "B":
                if not self.active_escape_route or self.route_purpose != "hide":
                    self.update_safe_block(actor, belief)
                self.update_side_escape(actor, belief)
                self.begin_evade()
            else:
                self.remember_police_trail(actor, belief)
            actor.reason = "chase" if role == "P" else "preset_evade" if self.escape_mode == "preset" else "evade"
            if actor.phase in ("hide", "wait", "pickup"):
                actor.phase, actor.remaining, actor.target = "ready", 0.0, None

    def update_belief_from_empty_treasure(self):
        """Use a local empty-site observation and a map-derived earliest pickup."""
        if self.vision == "fixed":
            return
        belief = self.beliefs["P"]
        speed = self.config["thief_speed_m_s"]
        travel, _ = self.routing.table(self.track.resolve("B"), "W", speed)
        earliest = travel[self.track.resolve("T")] + self.config["pickup_s"]
        if belief.seen or belief.last_time >= earliest:
            return
        times, _ = self.routing.table(self.track.resolve("T"), None, speed)
        budget = max(0.0, self.now - earliest)
        weights = {}
        for node, duration in times.items():
            if duration > budget + EPS:
                continue
            weight = (1.5 if node in self.hide_candidates else 1.0) / (1 + duration)
            if detectable(self.robots["P"].xy(self.track), self.track.xy[node], self.config, self.track.spacing, "P"):
                weight *= max(.02, self.config["miss_probability"])
            weights[node] = weight
        total = sum(weights.values())
        belief.last_seen, belief.last_time, belief.last_heading = self.track.resolve("T"), min(earliest, self.now), None
        belief.weights = {node: weight / total for node, weight in weights.items()}
        belief.nodes = sorted(belief.weights, key=lambda n: (-belief.weights[n], n))
        belief.states = {(node, None): weight for node, weight in belief.weights.items()}
        belief.updated = self.now

    def settle(self):
        if self.winner is not None:
            return
        for actor in self.robots.values():
            actor.finish(self.track)
        complete = [r.role for r in self.robots.values() if r.phase == "pickup" and r.remaining <= EPS]
        if self.config["pickup_s"] <= EPS:
            # Passing through treasure changes ownership at the arrival instant,
            # even while evading. It adds no stationary phase or sensing delay.
            complete += [r.role for r in self.robots.values()
                         if r.phase == "ready" and r.node == self.track.resolve("T")]
        if complete and self.owner is None:
            if len(complete) > 1:
                self.winner, self.end_reason = "judge", "simultaneous_pickup"
                return
            self.owner = complete[0]
            self.pickup_times[self.owner] = round(self.now, 6)
            self.events.append({"t": round(self.now, 3), "event": "pickup", "role": self.owner,
                                "moving_pickup": self.config["pickup_s"] <= EPS})
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
            self.visits[role][actor.node] = self.now
            if role == "P" and len(self.track.graph[actor.node]) == 1 and self.deadend_checked.get(actor.node) != self.now:
                self.deadend_checked[actor.node] = self.now
                self.events.append({"t": round(self.now, 4), "event": "deadend_checked", "role": "P", "node": actor.node})
            if role == "P" and actor.node == self.track.resolve("T") and self.owner == "B" and not self.police_checked_treasure:
                self.police_checked_treasure = True
                self.update_belief_from_empty_treasure()
                self.events.append({"t": round(self.now, 3), "event": "treasure_empty", "role": role})
            responding = (role == "B" and self.now < self.evade_until or
                          role == "P" and self.vision == "limited" and self.police_policy == "reactive" and
                          self.threat_active[role] and self.now - self.beliefs[role].last_time <= self.config.get("pursuit_memory_s", 2.0))
            if self.owner is None and actor.node == self.track.resolve("T") and not responding:
                decisions[role] = "pickup"
            else:
                decisions[role] = self.decide(role)
        for role, command in decisions.items():
            actor = self.robots[role]
            if command == "pickup":
                actor.phase, actor.remaining, actor.reason = "pickup", self.config["pickup_s"], "treasure"
            else:
                next_node, reason, goal = command
                previous_reason = actor.reason
                actor.command(self.track, self.routing, next_node, reason)
                if role == "P" and next_node is not None:
                    edge = tuple(sorted((actor.node, next_node)))
                    self.search_edges[edge] = self.search_edges.get(edge, 0) + 1
                if role in self.pending_response and reason in ("chase", "cutoff", "check_deadend", "evade", "preset_evade"):
                    detected_at = self.pending_response.pop(role)
                    self.response_events.append({"role": role, "detected_at": round(detected_at, 4),
                                                 "command_at": round(self.now, 4),
                                                 "delay_s": round(self.now - detected_at, 6), "action": reason})
                if reason != "treasure" and (reason != "hide" or previous_reason != "hide"):
                    self.events.append({"t": round(self.now, 3), "event": reason, "role": role, "goal": goal})

    def record(self):
        values = [round(self.now, 4), 0 if self.owner is None else 1 if self.owner == "P" else 2]
        for role in ("P", "B"):
            actor, belief = self.robots[role], self.beliefs[role]
            x, y = actor.xy(self.track)
            values.extend([round(x, 4), round(y, 4), actor.facing, PHASES[actor.phase],
                           REASONS[actor.reason], int(belief.seen), len(belief.nodes)])
        values.extend([*(self.track.xy[self.hide_goal] if self.hide_goal else (None, None)),
                       self.hide_facing if self.hide_goal else None, int(self.police_checked_treasure),
                       self.escape_plan_index if self.active_escape_route else -1,
                       self.beliefs["B"].sensor,
                       self.side_escape["direction"] if self.side_escape and
                       self.now - self.side_escape["last_time"] <= self.config.get("pursuit_memory_s", 2) else None,
                       self.safe_block["direction"],
                       self.robots["P"].body_facing or self.robots["P"].facing,
                       self.robots["B"].body_facing or self.robots["B"].facing,
                       int(self.robots["P"].reversing), int(self.robots["B"].reversing)])
        if self.frames and abs(self.frames[-1][0] - values[0]) < EPS:
            self.frames[-1] = values
        else:
            self.frames.append(values)

    def run(self):
        self.observe()
        next_sensor, next_frame = self.config["sensor_period_s"], 0.0
        while self.winner is None and self.now < self.config["time_limit_s"] - EPS:
            if capture_delay(self.robots["P"], self.robots["B"], self.track, self.config["capture_center_distance_m"], 0) is not None:
                self.capture(0)
                break
            self.settle()
            if self.winner:
                break
            self.command_ready()
            # Zero-duration pickup/turn transitions must settle before advancing.
            self.settle()
            if self.winner:
                break
            if any(r.phase == "ready" for r in self.robots.values()):
                # Complete chained zero-time transitions before time advances.
                continue
            if self.now >= next_frame - EPS:
                self.record()
                next_frame += self.config["frame_period_s"]
            pending = [r.remaining for r in self.robots.values() if r.phase != "ready" and r.remaining > EPS]
            dt = min([self.config["time_limit_s"] - self.now, max(EPS, next_sensor - self.now)] + pending)
            delay = capture_delay(self.robots["P"], self.robots["B"], self.track,
                                  self.config["capture_center_distance_m"], dt)
            if delay is not None:
                self.capture(delay)
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
                "escape_mode": self.escape_mode, "sensor_range_m": self.config["sensor_range_m"],
                "police_sensor_range_m": self.config.get("police_sensor_range_m", self.config["sensor_range_m"]),
                "police_speed_m_s": self.config["police_speed_m_s"], "thief_speed_m_s": self.config["thief_speed_m_s"],
                "vision": self.vision, "winner": self.winner, "end_reason": self.end_reason,
                "duration_s": round(self.now, 6), "pickup_times": self.pickup_times,
                "traveled_m": {r: round(a.traveled_m, 5) for r, a in self.robots.items()},
                "max_escape_straight_m": round(self.robots["B"].max_escape_straight_m, 5),
                "hide_decisions": self.hide_decisions,
                "deadend_checks": self.deadend_checked,
                "escape_plans": self.escape_plans, "response_events": self.response_events,
                "navigation_events": self.navigation_events, "search_sweeps": self.search_sweeps,
                "frames": self.frames, "events": self.events}

    def capture(self, delay):
        for actor in self.robots.values():
            actor.advance(self.track, delay)
        self.now += delay
        self.winner = "police"
        self.end_reason = "capture_before_pickup" if self.owner is None else "capture_after_pickup"
        if self.owner == "B":
            self.owner = "P"
        self.events.append({"t": round(self.now, 6), "event": "capture", "geometry": "2d_center_distance",
                            "police_xy": self.robots["P"].xy(self.track), "thief_xy": self.robots["B"].xy(self.track)})


def motion_metrics(result):
    """Approximate phase times from replay samples; heading changes count reversals."""
    metrics = {}
    for role, offset in (("P", 2), ("B", 9)):
        turn_s = wait_s = hide_s = 0.0
        reversals = 0
        for a, b in zip(result["frames"], result["frames"][1:]):
            dt = b[0] - a[0]
            turn_s += dt if a[offset + 3] in (PHASES["turn"], PHASES["orient"], PHASES["reverse"]) else 0
            wait_s += dt if a[offset + 3] == PHASES["wait"] else 0
            hide_s += dt if a[offset + 3] == PHASES["hide"] else 0
            reversals += turn(a[offset + 2], b[offset + 2]) == "UTURN"
        metrics[role] = {"turn_s": round(turn_s, 2), "wait_s": round(wait_s, 2),
                         "hide_s": round(hide_s, 2), "uturns": reversals, "distance_m": result["traveled_m"][role]}
    return metrics


def suite(track, config, seeds, sweep=False):
    scenarios = [
        ("两格检测 · 藏身后智能逃跑", "hide", "smart", 2, 1),
        ("三格检测 · 藏身后智能逃跑", "hide", "smart", 3, 1),
        ("两格检测 · 藏身后预设路线", "hide", "preset", 2, 1),
        ("三格检测 · 藏身后预设路线", "hide", "preset", 3, 1),
        ("两格检测 · 多路口连续逃跑", "escape", "smart", 2, 1),
        ("两格检测 · 警方快20% · 连续逃跑", "escape", "smart", 2, 1.2),
        ("两格检测 · 定点藏匿与下一藏点转移", "refuge", "smart", 2, 1)]
    replays, summary = [], []
    speed_cases = [(config["police_speed_m_s"], config["thief_speed_m_s"], True)]
    if sweep:
        speed_cases += [(1.8, 1.8, False), (2.1, 2.1, False)]
    for p_speed, b_speed, keep_replay in speed_cases:
      for label, thief, mode, cells, police_factor in scenarios:
        actual_police_speed = round(p_speed * police_factor, 6)
        case_config = dict(config, police_speed_m_s=actual_police_speed, thief_speed_m_s=b_speed,
                           sensor_range_m=round(cells * track.spacing, 3), police_sensor_range_m=round(3 * track.spacing, 3))
        outcomes = []
        for offset in range(seeds):
            result = Game(track, case_config, thief_policy=thief, seed=config["seed"] + offset, escape_mode=mode).run()
            result["metrics"] = motion_metrics(result)
            outcomes.append(result)
        replay = outcomes[0]
        replay["label"] = label
        if keep_replay:
            replays.append(replay)
        summary.append({"scenario": label, "runs": seeds, "police_speed_m_s": actual_police_speed,
                        "thief_speed_m_s": b_speed, "sensor_cells": cells, "police_sensor_cells": 3, "escape_mode": mode,
                        "police_wins": sum(r["winner"] == "police" for r in outcomes),
                        "thief_wins": sum(r["winner"] == "thief" for r in outcomes),
                        "draws": sum(r["winner"] == "draw" for r in outcomes),
                        "judge_required": sum(r["winner"] == "judge" for r in outcomes),
                        "mean_duration_s": round(sum(r["duration_s"] for r in outcomes) / seeds, 4),
                        "mean_max_escape_straight_m": round(sum(r["max_escape_straight_m"] for r in outcomes) / seeds, 3),
                        "mean_thief_hide_s": round(sum(r["metrics"]["B"]["hide_s"] for r in outcomes) / seeds, 2)})
        print(f"{label}: {summary[-1]}", flush=True)
    return {"config": config, "map": track.data,
            "frame_fields": ["t", "owner", "px", "py", "pheading", "pphase", "preason", "pseen", "pbelief",
                             "bx", "by", "bheading", "bphase", "breason", "bseen", "bbelief",
                             "hide_x", "hide_y", "hide_facing", "police_checked_treasure", "escape_plan_index",
                             "thief_sensor", "opposite_block_direction", "safe_block_direction",
                             "police_body_heading", "thief_body_heading", "police_reversing", "thief_reversing"],
            "summary": summary, "replays": replays}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=ROOT / "config.json")
    parser.add_argument("--seeds", type=int, default=1)
    parser.add_argument("--sweep", action="store_true", help="Also compare 6x and 7x straight-line speed")
    args = parser.parse_args()
    if args.seeds < 1:
        parser.error("--seeds must be positive")
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")
    config = json.loads(args.config.read_text(encoding="utf-8"))
    track = TrackMap()
    report = suite(track, config, args.seeds, args.sweep)
    annotation_file = ROOT / "map_annotations.json"
    previous = json.loads(annotation_file.read_text(encoding="utf-8")) if annotation_file.exists() else None
    annotations = build_annotations(track, previous)
    annotation_file.write_text(json.dumps(annotations, ensure_ascii=False, indent=2), encoding="utf-8")
    report["annotations"] = annotations
    for replay in report["replays"]:
        for choice in replay["hide_decisions"]:
            reference = annotations["original_node_mapping"][choice["node"]]
            if reference["node_id"] is None:
                raise ValueError("A hiding destination must be a physical localization landmark")
            choice["node_id"] = reference["node_id"]
    route_game = Game(track, config, escape_mode="preset")
    cards = {"status": "离线预设节点路线，未移植或烧录；按路口计数与自身朝向执行，遇到已检测冲突时切换备用路线",
             "config": config, "routes": route_game.preset_routes}
    (ROOT / "escape_routes.json").write_text(json.dumps(cards, ensure_ascii=False, indent=2), encoding="utf-8")
    (ROOT / "report.json").write_text(json.dumps(report, ensure_ascii=False, separators=(",", ":")), encoding="utf-8")
    payload = json.dumps(report, ensure_ascii=False, separators=(",", ":")).replace("</", "<\\/")
    template = (ROOT / "replay.template.html").read_text(encoding="utf-8")
    annotation_script = (ROOT / "map_annotations.js").read_text(encoding="utf-8")
    (ROOT / "replay.html").write_text(template.replace("__REPORT_DATA__", payload).replace("__ANNOTATION_SCRIPT__", annotation_script), encoding="utf-8")
    print("Saved report.json and replay.html")


if __name__ == "__main__":
    main()
