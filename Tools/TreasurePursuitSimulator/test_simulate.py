"""Offline checks of motion, sensing and simultaneous contact adjudication."""
import json
import math
import random
import unittest
from types import SimpleNamespace

from simulate import ROOT, Belief, Game, Robot, Routing, TrackMap, capture_delay, detectable, heading, sensor_channel


class SimulatorTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.track = TrackMap()
        cls.config = dict(json.loads((ROOT / "config.json").read_text(encoding="utf-8")),
                          police_speed_m_s=.35, thief_speed_m_s=.35, turn90_s=.6, uturn_s=1.2, pickup_s=.8, horizon_s=8)
        cls.routing = Routing(cls.track, cls.config)

    def test_time_routing_agrees_with_single_car_planner(self):
        for role, facing, speed in (("P", "N", .35), ("P", "N", 1.0), ("B", "W", .35)):
            times, next_nodes = self.routing.table(self.track.resolve(role), facing, speed)
            expected = self.track.dijkstra(role, "T", "time", speed, .6, 1.2)
            self.assertAlmostEqual(times[self.track.resolve("T")], expected["estimated_seconds"])
            self.assertIn(next_nodes[self.track.resolve("T")], self.track.graph[self.track.resolve(role)])

    def test_half_edge_continuous_motion(self):
        actor = Robot("P", "(5,0)", "E", .3)
        actor.command(self.track, self.routing, self.track.resolve("T"), "treasure")
        actor.advance(self.track, .25)
        self.assertEqual(actor.xy(self.track), (5.25, 0))
        self.assertAlmostEqual(actor.traveled_m, .075)
        actor.advance(self.track, .25)
        actor.finish(self.track)
        self.assertEqual((actor.node, actor.phase), (self.track.resolve("T"), "ready"))

    def test_turn_wait_does_not_translate(self):
        actor = Robot("P", "(5,0)", "N", .3)
        actor.command(self.track, self.routing, self.track.resolve("T"), "treasure")
        actor.advance(self.track, .6)
        self.assertEqual(actor.xy(self.track), (5, 0))
        self.assertEqual(actor.traveled_m, 0)
        actor.finish(self.track)
        self.assertEqual(actor.phase, "move")

    def test_swept_contact_catches_opposite_motion(self):
        track = SimpleNamespace(spacing=1, xy={"a": (0, 0), "b": (1, 0)},
                                graph={"a": {"b": 2}, "b": {"a": 2}})
        police = Robot("P", "a", "E", .4, phase="move", target="b", remaining=2.5)
        thief = Robot("B", "b", "W", .4, phase="move", target="a", remaining=2.5)
        # Both start and end outside the threshold, but cross during the interval.
        self.assertAlmostEqual(capture_delay(police, thief, track, .2, 2), 1.0)

    def test_perpendicular_overlap_counts_immediately(self):
        track = SimpleNamespace(spacing=1, xy={"a": (0, 0)}, graph={"a": {}})
        self.assertEqual(capture_delay(Robot("P", "a", "E", 1),
                                       Robot("B", "a", "N", 1), track, .2, 1), 0)

    def test_separate_roads_far_apart_do_not_make_contact(self):
        track = SimpleNamespace(spacing=1, xy={"a": (0, 0), "b": (1.44, 0),
                                               "c": (1.46, 0), "d": (3, 0)},
                                graph={"a": {"b": 2.88}, "b": {"a": 2.88},
                                       "c": {"d": 3.08}, "d": {"c": 3.08}})
        self.assertIsNone(capture_delay(Robot("P", "a", "E", 1),
                                        Robot("B", "d", "W", 1), track, .18, 1))

    def test_sensor_range_and_cardinal_cones(self):
        self.assertTrue(detectable((0, 0), (2, 0), self.config, .3))
        self.assertFalse(detectable((0, 0), (4, 0), self.config, .3))
        self.assertFalse(detectable((0, 0), (1, 1), self.config, .3))

    def test_unseen_opponent_truth_does_not_change_belief(self):
        ego = Robot("P", self.track.resolve("P"), "N", .35)
        left, right = Belief(self.track.resolve("B")), Belief(self.track.resolve("B"))
        left.observe(ego, (10, 5), 3, "limited", self.routing, .35, random.Random(7))
        right.observe(ego, (6, 0), 3, "limited", self.routing, .35, random.Random(7))
        self.assertFalse(left.seen or right.seen)
        self.assertEqual(left.nodes, right.nodes)
        self.assertGreater(len(left.nodes), 1)
        self.assertTrue(all(self.routing.distance[left.last_seen][n] <= .35 * 3 + .3 + 1e-9
                            for n in left.nodes))

    def test_candidate_search_covers_all_first_actions(self):
        source = max(self.track.graph, key=lambda n: len(self.track.graph[n]))
        candidates = self.routing.paths(source, "N", .35)
        self.assertEqual({path[1] for path, _ in candidates}, set(self.track.graph[source]))

    def test_faster_police_can_pick_up_first(self):
        config = dict(self.config, police_speed_m_s=1.0, thief_speed_m_s=.1)
        result = Game(self.track, config).run()
        self.assertEqual((result["winner"], result["end_reason"]), ("police", "police_pickup"))
        self.assertAlmostEqual(result["duration_s"], 5.85 / 1.0 + 5 * .6 + .8)

    def test_simultaneous_pickup_requires_adjudication(self):
        game = Game(self.track, self.config)
        for actor in game.robots.values():
            actor.node, actor.phase, actor.remaining = self.track.resolve("T"), "pickup", 0
        game.settle()
        self.assertEqual((game.winner, game.end_reason), ("judge", "simultaneous_pickup"))

    def test_moving_pickup_has_no_time_cost_on_fastest_route(self):
        config = dict(self.config, pickup_s=0, police_speed_m_s=1.0, thief_speed_m_s=.1)
        result = Game(self.track, config).run()
        self.assertEqual(result["end_reason"], "police_pickup")
        self.assertAlmostEqual(result["duration_s"], 5.85 / 1.0 + 5 * .6)
        self.assertTrue(all(f[5] != 3 and f[12] != 3 for f in result["frames"]))

    def test_thief_keeps_straight_line_velocity_at_pickup(self):
        game = Game(self.track, dict(self.config, pickup_s=0))
        actor = game.robots["B"]
        actor.node, actor.facing = "(6,0)", "W"
        game.hide_goal, game.hide_facing = "(5,4)", "S"
        actor.command(self.track, game.routing, self.track.resolve("T"), "treasure")
        velocity = actor.velocity(self.track)
        game.now = actor.remaining
        actor.advance(self.track, actor.remaining)
        game.settle()
        self.assertEqual(game.owner, "B")
        pickup_time = game.now
        game.command_ready()
        game.settle()
        self.assertEqual((actor.phase, actor.target), ("move", "(5,0)"))
        self.assertEqual(actor.velocity(self.track), velocity)
        self.assertEqual(game.now, pickup_time)
        self.assertTrue(next(e for e in game.events if e["event"] == "pickup")["moving_pickup"])

    def test_passing_treasure_collects_even_during_evasion(self):
        game = Game(self.track, dict(self.config, pickup_s=0))
        actor = game.robots["B"]
        actor.node, actor.phase, actor.target, actor.remaining = "(6,0)", "move", self.track.resolve("T"), 0
        actor.progress, actor.reason = 1, "evade"
        game.evade_until, game.beliefs["B"].seen = 10, True
        game.settle()
        self.assertEqual(game.owner, "B")

    def test_simultaneous_pass_through_pickup_requires_adjudication(self):
        game = Game(self.track, dict(self.config, pickup_s=0))
        for actor in game.robots.values():
            actor.node, actor.phase = self.track.resolve("T"), "ready"
        game.settle()
        self.assertEqual((game.winner, game.end_reason), ("judge", "simultaneous_pickup"))

    def test_zero_time_exit_orientation_adds_no_sensor_tick_pause(self):
        game = Game(self.track, dict(self.config, pickup_s=0, turn90_s=0, uturn_s=0, time_limit_s=.01), vision="fixed")
        actor = game.robots["B"]
        game.owner = "B"
        actor.node, actor.facing = "(4,2)", "E"
        actor.phase, actor.remaining = "orient", 0
        game.hide_goal, game.hide_facing = "(4,2)", "W"
        game.run()
        self.assertEqual(game.frames[0][12], 5)
        self.assertEqual(game.frames[0][0], 0)

    def test_capture_result_not_overwritten_by_pickup(self):
        game = Game(self.track, self.config)
        game.robots["B"].phase, game.robots["B"].remaining = "pickup", 0
        game.winner, game.end_reason = "police", "capture_before_pickup"
        game.settle()
        self.assertIsNone(game.owner)
        self.assertEqual(game.end_reason, "capture_before_pickup")

    def test_reproducibility_and_no_off_track_teleport(self):
        config = dict(self.config, time_limit_s=25)
        one = Game(self.track, config, vision="limited", seed=11).run()
        two = Game(self.track, config, vision="limited", seed=11).run()
        self.assertEqual(one, two)
        for previous, current in zip(one["frames"], one["frames"][1:]):
            dt = current[0] - previous[0]
            self.assertGreater(dt, 0)
            for offset in (2, 9):
                a, b = previous[offset:offset + 2], current[offset:offset + 2]
                self.assertLessEqual(math.dist(a, b) * self.track.spacing, .35 * dt + .0001)
                self.assertTrue(any(abs(b[0] - u[0]) < .0001 and abs(b[0] - v[0]) < .0001
                                    and min(u[1], v[1]) - .0001 <= b[1] <= max(u[1], v[1]) + .0001
                                    or abs(b[1] - u[1]) < .0001 and abs(b[1] - v[1]) < .0001
                                    and min(u[0], v[0]) - .0001 <= b[0] <= max(u[0], v[0]) + .0001
                                    for n in self.track.graph for m in self.track.graph[n]
                                    for u, v in [(self.track.xy[n], self.track.xy[m])]))

    def test_short_timeout_and_invalid_parameters(self):
        result = Game(self.track, dict(self.config, time_limit_s=.1)).run()
        self.assertEqual((result["winner"], result["end_reason"]), ("draw", "no_treasure_timeout"))
        for value in (0, -1, math.nan, math.inf):
            with self.assertRaises(ValueError):
                Game(self.track, dict(self.config, police_speed_m_s=value))

    def test_expensive_turns_favor_fewer_turns_over_shorter_distance(self):
        route = self.track.dijkstra("P", "T", "time", .3, 10, 20)
        self.assertEqual((route["distance_m"], route["quarter_turns"]), (5.85, 5))
        self.assertAlmostEqual(route["estimated_seconds"], 69.5)
        self.assertLess(route["estimated_seconds"], 5.25 / .3 + 7 * 10)

    def test_negative_observation_reduces_visible_position_probability(self):
        belief = Belief("(6,6)")
        belief.states = {("(0,8)", None): .5, ("(6,6)", None): .5}
        belief.updated = 100
        ego = Robot("P", self.track.resolve("P"), "N", .35)
        belief.observe(ego, (8, 2), 100, "limited", self.routing, .35, random.Random(7))
        self.assertFalse(belief.seen)
        self.assertAlmostEqual(sum(belief.weights.values()), 1)
        self.assertLess(belief.weights["(0,8)"], belief.weights["(6,6)"] / 10)

    def test_search_keeps_a_goal_while_traversing_a_corridor(self):
        game = Game(self.track, self.config)
        game.now = 50
        game.plans["P"] = ("search", "(3,7)", 70)
        first = game.search(game.robots["P"], game.beliefs["P"])
        game.robots["P"].node = first[0]
        game.now = 51
        second = game.search(game.robots["P"], game.beliefs["P"])
        self.assertEqual(first[2], "(3,7)")
        self.assertEqual(second[2], first[2])

    def test_thief_does_not_start_a_slow_turn_with_police_close_behind(self):
        config = dict(self.config, turn90_s=10, uturn_s=20, horizon_s=30,
                      police_speed_m_s=.3, thief_speed_m_s=.3)
        game = Game(self.track, config)
        game.owner, game.now = "B", 80
        actor = Robot("B", "(6,6)", "N", .3, last_node="(6,7)")
        belief = Belief("(6,7)")
        belief.last_time, belief.last_heading, belief.seen = 80, "N", True
        self.assertEqual(game.escape(actor, belief)[0], "(6,5)")

    def test_interception_cannot_wait_at_one_node_indefinitely(self):
        game = Game(self.track, self.config)
        game.owner, game.now = "B", 10
        actor, belief = game.robots["P"], game.beliefs["P"]
        belief.last_time = 10
        game.predicted_routes = lambda a, b: [([belief.last_seen, actor.node], [0, .5], 1)]
        self.assertEqual(game.intercept(actor, belief)[1], "hold")
        game.now = 11.6
        self.assertNotEqual(game.intercept(actor, belief)[1], "hold")

    def test_global_observation_is_not_an_active_mode(self):
        with self.assertRaises(ValueError):
            Game(self.track, self.config, vision="global")

    def test_police_take_fastest_treasure_route_without_remote_ownership(self):
        game = Game(self.track, self.config)
        route, _, _ = game.nominal_police
        actor = game.robots["P"]
        for node, following in zip(route, route[1:]):
            actor.node = node
            game.owner = None
            before = game.decide("P")
            game.owner = "B"
            after = game.decide("P")
            self.assertEqual(before, after)
            self.assertEqual(before[:2], (following, "treasure"))
            actor.facing = self.routing.edge_time(node, following, actor.facing, actor.speed)[1]

    def test_police_only_confirm_empty_treasure_at_the_treasure_node(self):
        game = Game(self.track, self.config)
        game.owner = "B"
        game.command_ready()
        self.assertFalse(game.police_checked_treasure)
        game.robots["P"].node = self.track.resolve("T")
        game.robots["P"].phase = "ready"
        game.command_ready()
        self.assertTrue(game.police_checked_treasure)
        self.assertTrue(any(e["event"] == "treasure_empty" for e in game.events))

    def test_fixed_observation_ignores_even_nearby_opponent(self):
        belief = Belief(self.track.resolve("B"))
        ego = Robot("P", self.track.resolve("P"), "N", .35)
        belief.observe(ego, ego.xy(self.track), 5, "fixed", self.routing, .35, random.Random(7))
        self.assertFalse(belief.seen)
        self.assertEqual(belief.last_seen, self.track.resolve("B"))

    def test_hiding_point_is_not_selected_using_unseen_true_police_position(self):
        first, second = Game(self.track, self.config), Game(self.track, self.config)
        for game, police_node in ((first, "(0,9)"), (second, "(6,0)")):
            game.owner, game.now = "B", 12.2
            game.robots["B"].node, game.robots["B"].facing = self.track.resolve("T"), "W"
            game.robots["P"].node = police_node
            self.assertTrue(game.choose_hide(game.robots["B"], game.beliefs["B"]))
        self.assertEqual((first.hide_goal, first.hide_facing), (second.hide_goal, second.hide_facing))
        self.assertIn(first.hide_goal, first.cycle_core)
        self.assertGreaterEqual(len(self.track.graph[first.hide_goal]), 2)
        self.assertNotIn(first.hide_goal, first.nominal_police[0])

    def test_hiding_holds_position_instead_of_tracing_a_loop(self):
        game = Game(self.track, self.config)
        actor = game.robots["B"]
        actor.node, actor.facing = "(4,2)", "W"
        game.owner, game.now, game.hide_goal, game.hide_facing = "B", 50, actor.node, "W"
        neighbor, reason, _ = game.hide(actor, game.beliefs["B"])
        self.assertEqual((neighbor, reason), (None, "hide"))
        actor.command(self.track, game.routing, neighbor, reason)
        self.assertEqual(actor.phase, "hide")
        actor.advance(self.track, actor.remaining)
        actor.finish(self.track)
        self.assertEqual(actor.xy(self.track), (4, 2))
        self.assertEqual(actor.traveled_m, 0)
        self.assertEqual(game.hide(actor, game.beliefs["B"])[1], "hide")

    def test_prepare_exit_heading_does_not_move_out_of_hiding_point(self):
        game = Game(self.track, self.config)
        actor = game.robots["B"]
        actor.node, actor.facing = "(4,2)", "E"
        game.owner, game.now, game.hide_goal, game.hide_facing = "B", 50, actor.node, "W"
        neighbor, reason, _ = game.hide(actor, game.beliefs["B"])
        self.assertEqual(reason, "prepare_hide")
        actor.command(self.track, game.routing, neighbor, reason)
        self.assertEqual(actor.phase, "orient")
        self.assertAlmostEqual(actor.remaining, self.config["uturn_s"])
        actor.advance(self.track, actor.remaining)
        actor.finish(self.track)
        self.assertEqual((actor.node, actor.facing, actor.traveled_m), ("(4,2)", "W", 0))

    def test_detection_of_nearby_police_interrupts_hiding(self):
        game = Game(self.track, self.config)
        actor = game.robots["B"]
        actor.node, actor.facing, actor.last_node = "(6,4)", "N", "(6,5)"
        game.owner, game.now, game.hide_goal, game.hide_facing = "B", 50, actor.node, "N"
        belief = Belief("(6,5)")
        belief.last_time, belief.last_heading, belief.seen = 50, "N", True
        neighbor, reason, _ = game.hide(actor, belief)
        self.assertEqual(reason, "evade")
        self.assertIn(neighbor, self.track.graph[actor.node])
        self.assertNotEqual(game.hide_goal, actor.node)
        self.assertGreater(game.hide_banned[actor.node], game.now)

    def test_fixed_patrol_does_not_depend_on_opponent_position(self):
        choices = []
        for police_target in ("(0,0)", "(11,5)"):
            game = Game(self.track, self.config, police_policy="fixed", vision="fixed")
            game.owner, game.police_checked_treasure = "B", True
            game.robots["P"].node, game.robots["P"].facing = self.track.resolve("T"), "W"
            game.robots["B"].node = police_target
            choices.append(game.decide("P"))
            self.assertEqual(set(game.fixed_patrol) | set(game.nominal_police[0]), set(self.track.graph))
        self.assertEqual(choices[0], choices[1])

    def test_empty_treasure_updates_reachable_region_without_remote_position(self):
        config = dict(self.config, police_speed_m_s=.3, thief_speed_m_s=.3, turn90_s=3, uturn_s=2)
        results = []
        for hidden_node in ("(1,0)", "(11,5)"):
            game = Game(self.track, config)
            game.now, game.owner = 34.5, "B"
            game.robots["P"].node = self.track.resolve("T")
            game.robots["B"].node = hidden_node
            game.update_belief_from_empty_treasure()
            belief = game.beliefs["P"]
            self.assertFalse(belief.seen)
            self.assertNotIn(self.track.resolve("B"), belief.nodes)
            self.assertAlmostEqual(sum(belief.weights.values()), 1)
            results.append(belief.weights)
        self.assertEqual(results[0], results[1])

    def test_police_detection_overrides_treasure_route_before_pickup(self):
        game = Game(self.track, self.config)
        actor, belief = game.robots["P"], game.beliefs["P"]
        actor.node, actor.facing = "(6,6)", "N"
        belief.last_seen, belief.nodes = "(6,5)", ["(6,5)"]
        belief.seen, belief.last_time, game.now = True, 10, 10
        self.assertFalse(game.police_checked_treasure)
        self.assertEqual(game.decide("P")[:2], ("(6,5)", "chase"))

    def test_thief_detection_overrides_treasure_route_before_pickup(self):
        game = Game(self.track, self.config)
        actor, belief = game.robots["B"], game.beliefs["B"]
        actor.node, actor.facing = "(6,6)", "N"
        belief.last_seen, belief.nodes = "(6,7)", ["(6,7)"]
        belief.weights, belief.last_xy = {"(6,7)": 1}, (6, 7)
        belief.seen, belief.last_time, belief.last_heading, game.now = True, 10, "N", 10
        self.assertEqual(game.decide("B")[1], "evade")
        self.assertIsNone(game.owner)

    def test_detecting_police_releases_parking_on_the_same_sensor_tick(self):
        config = dict(self.config, miss_probability=0)
        game = Game(self.track, config)
        game.owner, game.now = "B", 10
        actor = game.robots["B"]
        actor.node, actor.facing, actor.phase, actor.remaining = "(6,4)", "N", "hide", .02
        game.robots["P"].node, game.robots["P"].facing = "(6,5)", "N"
        game.hide_goal = actor.node
        game.observe()
        self.assertEqual((actor.phase, actor.reason, actor.remaining), ("ready", "evade", 0))
        self.assertIsNone(game.hide_goal)
        game.command_ready()
        event = next(e for e in game.response_events if e["role"] == "B")
        self.assertEqual(event["delay_s"], 0)

    def test_existing_turn_is_not_completed_for_free_by_detection(self):
        game = Game(self.track, dict(self.config, miss_probability=0))
        actor = game.robots["B"]
        actor.node, actor.phase, actor.target, actor.remaining = "(6,4)", "turn", "(6,3)", .4
        game.robots["P"].node = "(6,5)"
        game.observe()
        self.assertEqual((actor.phase, actor.remaining, actor.reason), ("turn", .4, "evade"))

    def test_every_detection_interrupts_hiding_without_extra_danger_threshold(self):
        game = Game(self.track, self.config)
        actor, belief = game.robots["B"], game.beliefs["B"]
        game.owner, game.now = "B", 50
        actor.node, actor.facing = "(6,4)", "N"
        game.hide_goal, game.hide_facing = actor.node, actor.facing
        belief.last_seen, belief.nodes, belief.weights = "(6,7)", ["(6,7)"], {"(6,7)": 1}
        belief.last_xy, belief.seen, belief.last_time = (6, 7), True, 50
        self.assertEqual(game.hide(actor, belief)[1], "evade")

    def test_mid_edge_head_on_conflict_and_turn_wait_are_checked(self):
        config = dict(self.config, police_speed_m_s=1.95, thief_speed_m_s=1.95, turn90_s=3, uturn_s=2)
        game = Game(self.track, config)
        actor = Robot("B", "(6,6)", "N", 1.95)
        behind = Belief("(6,7)")
        behind.last_xy, behind.last_heading = (6, 7), "N"
        forward = ["(6,6)", "(6,5)"]
        reverse = ["(6,6)", "(6,7)"]
        safe = game.route_risk(forward, game.timed_path(forward, actor), actor, behind)
        unsafe = game.route_risk(reverse, game.timed_path(reverse, actor), actor, behind)
        self.assertGreater(safe[0], unsafe[0])
        self.assertLess(unsafe[0], .2)
        incoming = Belief("(6,5)")
        incoming.last_xy, incoming.last_heading = (6, 5), "S"
        head_on = game.route_risk(forward, game.timed_path(forward, actor), actor, incoming)
        self.assertLess(head_on[0], .15)

    def test_preset_routes_are_legal_and_independent_of_true_police_position(self):
        first, second = Game(self.track, self.config, escape_mode="preset"), Game(self.track, self.config, escape_mode="preset")
        second.robots["P"].node = "(6,6)"
        self.assertEqual(first.preset_routes, second.preset_routes)
        for source, cards in first.preset_routes.items():
            if source in first.cycle_core:
                exits = set(self.track.graph[source]) & first.cycle_core
                if source not in first.pocket_nodes:
                    exits -= first.pocket_nodes
                self.assertEqual({p[1] for p in cards}, exits)
            for path in cards:
                self.assertEqual(len(path), len(set(path)))
                self.assertTrue(all(v in self.track.graph[u] for u, v in zip(path, path[1:])))

    def test_first_detection_does_not_infer_heading_from_initial_spawn(self):
        belief = Belief("(11,5)")
        ego = Robot("P", "(6,4)", "N", .35)
        config = dict(self.config, miss_probability=0)
        routing = Routing(self.track, config)
        belief.observe(ego, (6, 5), 10, "limited", routing, .35, random.Random(7))
        self.assertIsNone(belief.last_heading)
        belief.observe(ego, (6, 4.9), 10.02, "limited", routing, .35, random.Random(7))
        self.assertEqual(belief.last_heading, "N")

    def test_two_and_three_cell_detection_boundaries(self):
        for cells in (2, 3):
            config = dict(self.config, sensor_range_m=cells * self.track.spacing)
            self.assertTrue(detectable((6, 4), (6, 4 + cells), config, self.track.spacing))
            self.assertFalse(detectable((6, 4), (6, 4 + cells + .01), config, self.track.spacing))

    def test_stationary_detection_does_not_reuse_stale_travel_heading(self):
        routing = Routing(self.track, dict(self.config, miss_probability=0))
        ego, belief = Robot("P", "(6,4)", "N", .35), Belief("(11,5)")
        belief.observe(ego, (6, 5), 10, "limited", routing, .35, random.Random(7))
        belief.observe(ego, (6, 4.9), 10.02, "limited", routing, .35, random.Random(7))
        self.assertEqual(belief.last_heading, "N")
        belief.observe(ego, (6, 4.9), 10.04, "limited", routing, .35, random.Random(7))
        self.assertIsNone(belief.last_heading)

    def test_fresh_escape_decision_does_not_read_true_police_position(self):
        decisions = []
        for node in ("(0,0)", "(11,5)"):
            game = Game(self.track, self.config)
            game.now = 50
            game.robots["P"].node = node
            actor, belief = game.robots["B"], game.beliefs["B"]
            actor.node, actor.facing = "(6,6)", "N"
            belief.last_seen, belief.last_xy, belief.last_heading = "(6,7)", (6, 7), "N"
            belief.nodes, belief.weights = ["(6,7)"], {"(6,7)": 1}
            belief.seen, belief.last_time = True, 50
            decisions.append(game.escape(actor, belief))
        self.assertEqual(decisions[0], decisions[1])

    def test_preset_route_has_alternate_branch_instead_of_only_police_corridor(self):
        game = Game(self.track, self.config, escape_mode="preset")
        cards = game.preset_routes["(5,1)"]
        self.assertTrue(any(p[1:4] == ["(5,2)", "(5,3)", "(4,3)"] for p in cards))

    def test_preset_sequence_is_kept_after_losing_detection(self):
        game = Game(self.track, self.config, escape_mode="preset")
        actor = game.robots["B"]
        actor.node, actor.facing = "(6,6)", "N"
        game.preset_escape(actor, game.beliefs["B"])
        route = game.active_escape_route
        actor.facing = heading(self.track.xy[route[0]], self.track.xy[route[1]])
        actor.node, game.now = route[1], 3
        if len(route) > 2:
            self.assertEqual(game.preset_escape(actor, game.beliefs["B"])[0], route[2])
            self.assertEqual(game.active_escape_route, route)

    def test_side_ir_channels_rotate_with_own_travel_direction(self):
        actor = Robot("B", "(6,6)", "N", .35)
        for facing, point, expected in (("N", (5, 6), "left"), ("N", (7, 6), "right"),
                                         ("E", (6, 5), "left"), ("E", (6, 7), "right"),
                                         ("S", (6, 7), "front"), ("W", (7, 6), "rear")):
            actor.facing = facing
            self.assertEqual(sensor_channel(actor, point, self.track), expected)
        actor.phase = "turn"
        self.assertIsNone(sensor_channel(actor, (5, 6), self.track))

    def test_missing_or_fixed_observation_cannot_report_side_ir(self):
        routing = Routing(self.track, dict(self.config, miss_probability=0))
        actor, belief = Robot("B", "(6,6)", "N", .35), Belief("(0,9)")
        belief.observe(actor, (5, 6), 10, "limited", routing, .35, random.Random(7))
        self.assertEqual(belief.sensor, "left")
        belief.observe(actor, (0, 0), 10.02, "limited", routing, .35, random.Random(7))
        self.assertFalse(belief.seen)
        self.assertIsNone(belief.sensor)
        belief.observe(actor, (5, 6), 10.04, "fixed", routing, .35, random.Random(7))
        self.assertIsNone(belief.sensor)

    def test_side_detection_latches_opposite_area_without_following_hidden_truth(self):
        game = Game(self.track, dict(self.config, miss_probability=0))
        game.now = 10
        actor = game.robots["B"]
        actor.node, actor.facing = "(6,6)", "N"
        game.robots["P"].node = "(5,6)"
        game.observe()
        self.assertEqual(game.side_escape["direction"], "E")
        anchor = game.side_escape["anchor"]
        game.now += .02
        game.robots["P"].node = "(0,0)"
        game.observe()
        self.assertEqual(game.side_escape["anchor"], anchor)
        east = ["(6,6)", "(7,6)", "(8,6)"]
        north = ["(6,6)", "(6,5)", "(6,4)"]
        self.assertGreater(game.side_preference(east, [0, .3, .6]), game.side_preference(north, [0, .3, .6]))
        game.now += 3
        self.assertEqual(game.side_preference(east, [0, .3, .6]), (0, 0))

    def test_comparable_safe_routes_prefer_opposite_block(self):
        game = Game(self.track, self.config)
        actor = Robot("B", "(6,6)", "N", .35)
        game.side_escape = {"anchor": (6, 6), "direction": "E", "last_time": 0}
        east = ["(6,6)", "(7,6)", "(8,6)"]
        north = ["(6,6)", "(6,5)", "(6,4)"]
        # Controlled arrival envelopes isolate the selection rule. Real mid-edge
        # and turn exposures are checked separately by route_risk tests.
        game.route_risk = lambda path, *args: (4.0 if path == north else 3.9, .2, -.5, -.6)
        candidates = [(east, [0, .8, 1.6]), (north, [0, .8, 1.6])]
        choice = game.select_escape_path(candidates, actor, game.beliefs["B"], True)
        self.assertEqual(choice[0], east)

    def test_opposite_side_preference_never_forces_earlier_collision(self):
        game = Game(self.track, self.config)
        actor = Robot("B", "(6,6)", "N", .35)
        game.side_escape = {"anchor": (6, 6), "direction": "E", "last_time": 0}
        east = ["(6,6)", "(7,6)", "(8,6)"]
        north = ["(6,6)", "(6,5)", "(6,4)"]
        game.route_risk = lambda path, *args: (.1, -.2, -1, -.6) if path == east else (4, .2, -.5, 0)
        choice = game.select_escape_path([(east, [0, .8, 1.6]), (north, [0, .8, 1.6])],
                                         actor, game.beliefs["B"], True)
        self.assertEqual(choice[0], north)

    def test_tiny_forecast_gain_does_not_trigger_escape_uturn(self):
        game = Game(self.track, self.config)
        actor = Robot("B", "(3,2)", "N", .35, last_node="(3,3)")
        forward, reverse = ["(3,2)", "(3,1)"], ["(3,2)", "(3,3)"]
        game.route_risk = lambda path, *args: (1.01 if path == reverse else 1.0, .02, -1, 0)
        choice = game.select_escape_path([(p, game.timed_path(p, actor)) for p in (forward, reverse)],
                                         actor, game.beliefs["B"], True)
        self.assertEqual(choice[0], forward)

    def test_branching_block_beats_long_straight_with_weak_observation(self):
        game = Game(self.track, self.config)
        actor = Robot("B", "(6,8)", "N", .35)
        straight = ["(6,8)", "(7,8)", "(8,8)", "(9,8)", "(10,8)", "(11,8)"]
        branching = ["(6,8)", "(6,7)", "(6,6)", "(7,6)", "(7,7)", "(8,7)", "(8,8)"]
        self.assertGreater(game.topology_score(game.escape_topology(branching)),
                           game.topology_score(game.escape_topology(straight)))
        choice = game.select_escape_path([(p, game.timed_path(p, actor)) for p in (straight, branching)],
                                         actor, game.beliefs["B"], False)
        self.assertEqual(choice[0], branching)

    def test_forced_corner_is_not_an_escape_junction(self):
        game = Game(self.track, self.config)
        path = ["(7,7)", "(8,7)", "(8,8)"]
        self.assertEqual(game.escape_topology(path)["branch_count"], 1)
        self.assertAlmostEqual(game.escape_topology(path)["max_corridor_m"], .6)

    def test_faster_police_reduce_straight_escape_contact_time(self):
        path = [f"(6,{y})" for y in range(6, 1, -1)]
        actor = Robot("B", "(6,6)", "N", 1.95)
        belief = Belief("(6,7)")
        belief.last_xy, belief.last_heading = (6, 7), "N"
        equal = Game(self.track, dict(self.config, police_speed_m_s=1.95, thief_speed_m_s=1.95, turn90_s=.5))
        faster = Game(self.track, dict(self.config, police_speed_m_s=2.34, thief_speed_m_s=1.95, turn90_s=.5))
        equal_risk = equal.route_risk(path, equal.timed_path(path, actor), actor, belief)
        faster_risk = faster.route_risk(path, faster.timed_path(path, actor), actor, belief)
        self.assertGreater(equal_risk[0], faster_risk[0])
        self.assertAlmostEqual(faster_risk[0], (.3 - .18) / (2.34 - 1.95), delta=.05)

    def test_escape_straight_distance_accumulates_across_nodes_and_resets_on_turn(self):
        actor = Robot("B", "(6,8)", "E", .3)
        for target in ("(7,8)", "(8,8)"):
            actor.command(self.track, self.routing, target, "cruise")
            actor.advance(self.track, actor.remaining)
            actor.finish(self.track)
        self.assertAlmostEqual(actor.max_escape_straight_m, .6)
        actor.command(self.track, self.routing, "(8,7)", "evade")
        actor.advance(self.track, actor.remaining)
        actor.finish(self.track)
        actor.advance(self.track, actor.remaining)
        actor.finish(self.track)
        self.assertAlmostEqual(actor.escape_straight_m, .3)
        self.assertAlmostEqual(actor.max_escape_straight_m, .6)
        actor.phase, actor.reason = "hide", "hide"
        actor.advance(self.track, .02)
        self.assertEqual(actor.escape_straight_m, 0)

    def test_perpendicular_swept_contact_is_caught_between_frames(self):
        track = SimpleNamespace(spacing=1, xy={"a": (-1, 0), "b": (1, 0), "c": (0, -1), "d": (0, 1)},
                                graph={"a": {"b": 4}, "b": {"a": 4}, "c": {"d": 4}, "d": {"c": 4}})
        police = Robot("P", "a", "E", 1, phase="move", target="b", remaining=2)
        thief = Robot("B", "c", "S", 1, phase="move", target="d", remaining=2)
        self.assertAlmostEqual(capture_delay(police, thief, track, .2, 2), 1 - .2 / math.sqrt(2))
        self.assertIsNone(capture_delay(police, thief, track, .2, .8))

    def test_crossing_at_different_times_is_not_a_collision(self):
        track = SimpleNamespace(spacing=1, xy={"a": (-1, 0), "b": (1, 0), "c": (0, -2), "d": (0, 2)},
                                graph={"a": {"b": 4}, "b": {"a": 4}, "c": {"d": 8}, "d": {"c": 8}})
        police = Robot("P", "a", "E", 1, phase="move", target="b", remaining=2)
        thief = Robot("B", "c", "S", 1, phase="move", target="d", remaining=4)
        self.assertIsNone(capture_delay(police, thief, track, .2, 2))

    def test_contact_while_thief_turns_does_not_depend_on_facing(self):
        track = SimpleNamespace(spacing=1, xy={"a": (-1, 0), "b": (1, 0), "c": (0, 0)},
                                graph={"a": {"b": 4}, "b": {"a": 4}, "c": {}})
        police = Robot("P", "a", "E", 1, phase="move", target="b", remaining=2)
        thief = Robot("B", "c", "N", 1, phase="turn", remaining=1)
        self.assertAlmostEqual(capture_delay(police, thief, track, .2, 1), .8)

    def test_blind_junction_contact_ends_match_even_without_detection(self):
        config = dict(self.config, police_speed_m_s=1.95, thief_speed_m_s=1.95,
                      turn90_s=.5, uturn_s=.5, miss_probability=0, time_limit_s=1)
        game = Game(self.track, config)
        game.owner, game.police_checked_treasure = "B", True
        game.robots["P"] = Robot("P", "(1,2)", "E", 1.95, phase="move", target="(2,2)", remaining=.3 / 1.95)
        game.robots["B"] = Robot("B", "(2,3)", "N", 1.95, phase="move", target="(2,2)", remaining=.3 / 1.95)
        result = game.run()
        self.assertEqual((result["winner"], result["end_reason"]), ("police", "capture_after_pickup"))
        self.assertFalse(game.beliefs["P"].seen or game.beliefs["B"].seen)
        self.assertAlmostEqual(result["duration_s"], (.3 - .18 / math.sqrt(2)) / 1.95, places=6)
        self.assertEqual(result["events"][-1]["geometry"], "2d_center_distance")

    def test_existing_overlap_is_captured_before_either_can_depart(self):
        game = Game(self.track, self.config)
        for actor, facing in ((game.robots["P"], "E"), (game.robots["B"], "N")):
            actor.node, actor.facing = "(2,2)", facing
        result = game.run()
        self.assertEqual(result["end_reason"], "capture_before_pickup")
        self.assertEqual(result["duration_s"], 0)

    def test_collision_risk_checks_orthogonal_branch_before_center(self):
        game = Game(self.track, dict(self.config, police_speed_m_s=1.95, thief_speed_m_s=1.95, turn90_s=.5))
        actor = Robot("B", "(2,3)", "N", 1.95)
        belief = Belief("(1,2)")
        belief.last_xy, belief.last_heading = (1, 2), "E"
        path = ["(2,3)", "(2,2)"]
        risk = game.route_risk(path, game.timed_path(path, actor), actor, belief)
        self.assertLess(risk[0], .3 / 1.95)

    def test_all_hiding_sites_have_short_roads_and_two_junction_exits(self):
        game = Game(self.track, self.config)
        self.assertTrue(game.hide_candidates)
        for node in game.hide_candidates:
            self.assertIn(node, game.cycle_core)
            self.assertLessEqual(game.road_span(node), 3 * self.track.spacing + 1e-9)
            exits = game.hide_exits(node)
            self.assertGreaterEqual(len(exits), 2)
            for path in exits:
                self.assertIn(path[-1], game.branches)
                self.assertTrue(all(n in game.cycle_core for n in path))
                self.assertTrue(all(v in self.track.graph[u] for u, v in zip(path, path[1:])))

    def test_stored_deadend_or_long_road_cannot_become_a_hiding_stop(self):
        for node in ("(2,3)", "(1,0)"):
            game = Game(self.track, self.config)
            actor = game.robots["B"]
            actor.node, game.owner = node, "B"
            game.hide_goal, game.hide_facing = node, actor.facing
            command = game.hide(actor, game.beliefs["B"])
            self.assertNotEqual(command[1], "hide")
            self.assertNotEqual(game.hide_goal, node)

    def test_unknown_police_speed_does_not_reverse_treasure_exit_into_approach(self):
        for police_speed in (1.17, 1.95, 2.34):
            game = Game(self.track, dict(self.config, police_speed_m_s=police_speed, thief_speed_m_s=1.95,
                                         turn90_s=.5, uturn_s=.5), thief_policy="escape")
            actor = game.robots["B"]
            actor.node, actor.facing, game.owner = self.track.resolve("T"), "W", "B"
            self.assertEqual(game.escape(actor, game.beliefs["B"])[0], "(5,0)")
            self.assertTrue(game.choose_hide(actor, game.beliefs["B"]))
            self.assertLess(self.track.xy[game.hide_goal][0], 5.5)

    def test_last_observed_safe_region_survives_sensor_gap(self):
        game = Game(self.track, dict(self.config, miss_probability=0))
        actor = game.robots["B"]
        actor.node, actor.facing = "(5,2)", "S"
        game.robots["P"].node, game.now = "(6,2)", 10
        game.observe()
        prior = dict(game.safe_block)
        self.assertEqual(prior["direction"], "W")
        game.now, game.robots["P"].node = 15, "(0,0)"
        game.observe()
        self.assertEqual(game.safe_block, prior)

    def test_retreat_can_beat_turn_at_junction(self):
        game = Game(self.track, dict(self.config, turn90_s=.5, uturn_s=.5))
        actor = Robot("B", "(3,1)", "N", .35, last_node="(3,2)")
        retreat = ["(3,1)", "(3,2)", "(3,3)"]
        side_turn = ["(3,1)", "(2,1)", "(1,1)"]
        game.safe_block = {"anchor": (3, .5), "direction": "S", "source": "last_detection"}
        game.route_risk = lambda path, *args: (2.4, .2, -.5, -.5)
        choice = game.select_escape_path([(p, game.timed_path(p, actor)) for p in (retreat, side_turn)],
                                         actor, game.beliefs["B"], True)
        self.assertEqual(choice[0], retreat)
        self.assertAlmostEqual(game.routing.edge_time(actor.node, retreat[1], actor.facing, actor.speed)[0],
                               game.routing.edge_time(actor.node, side_turn[1], actor.facing, actor.speed)[0])

    def test_police_keep_trail_after_detection_memory_expires_before_treasure(self):
        game = Game(self.track, self.config)
        actor, belief = game.robots["P"], game.beliefs["P"]
        actor.node, actor.facing, game.now = "(6,6)", "N", 1
        belief.last_seen, belief.last_xy, belief.last_heading = "(6,5)", (6, 5), "N"
        belief.last_time, belief.seen = 1, True
        game.remember_police_trail(actor, belief)
        actor.node, actor.last_node = "(6,5)", "(6,6)"
        game.now, belief.seen = 4, False
        command = game.decide("P")
        self.assertEqual(command[:2], ("(6,4)", "trail"))
        self.assertNotEqual(command[1], "treasure")

    def test_police_follow_last_heading_after_reaching_last_seen_junction(self):
        game = Game(self.track, self.config)
        actor, belief = game.robots["P"], game.beliefs["P"]
        actor.node, actor.facing, game.now = "(2,1)", "E", 1
        belief.last_seen, belief.last_time, belief.seen, belief.last_heading = "(2,1)", 1, True, "W"
        game.remember_police_trail(actor, belief)
        belief.seen, game.now = False, 1.1
        self.assertEqual(game.decide("P")[:2], ("(1,1)", "trail"))

    def test_police_nearby_one_bend_deadend_check_is_not_immediately_repeated(self):
        game = Game(self.track, self.config)
        actor = game.robots["P"]
        actor.node, actor.facing, game.now = "(2,2)", "E", 10
        self.assertEqual(game.inspect_deadends(actor), ("(2,3)", "check_deadend", "(2,3)"))
        actor.node, actor.facing, game.now = "(2,3)", "S", 11
        game.inspect_deadends(actor)
        self.assertEqual(game.deadend_checked["(2,3)"], 11)
        actor.node, actor.facing, game.now = "(2,2)", "E", 12
        command = game.inspect_deadends(actor)
        self.assertTrue(command is None or command[2] != "(2,3)")

    def test_side_detection_makes_police_return_to_deadend_arm(self):
        game = Game(self.track, dict(self.config, miss_probability=0))
        actor = game.robots["P"]
        actor.node, actor.facing, game.now = "(1,3)", "S", 10
        game.robots["B"].node = "(2,3)"
        game.observe()
        self.assertEqual(game.deadend_goal, "(2,3)")
        self.assertEqual(game.decide("P"), ("(1,2)", "check_deadend", "(2,3)"))

    def test_police_arrival_marks_deadend_checked_even_during_another_plan(self):
        game = Game(self.track, self.config)
        game.robots["P"].node, game.now = "(2,3)", 10
        game.owner, game.police_checked_treasure = "B", True
        game.command_ready()
        self.assertEqual(game.deadend_checked["(2,3)"], 10)

    def test_police_lost_trail_decision_does_not_read_hidden_thief_position(self):
        decisions = []
        for hidden in ("(0,0)", "(11,5)"):
            game = Game(self.track, self.config)
            actor, belief = game.robots["P"], game.beliefs["P"]
            actor.node, actor.facing, game.now = "(2,1)", "E", 1
            belief.last_seen, belief.last_time, belief.last_heading, belief.seen = "(2,1)", 1, "W", True
            game.remember_police_trail(actor, belief)
            game.robots["B"].node, belief.seen, game.now = hidden, False, 4
            decisions.append(game.decide("P"))
        self.assertEqual(decisions[0], decisions[1])

    def test_reverse_keeps_chassis_heading_and_side_ir_assignment(self):
        game = Game(self.track, dict(self.config, turn90_s=.5, uturn_s=.5))
        actor = Robot("B", "(6,6)", "N", .3)
        actor.command(self.track, game.routing, "(6,7)", "evade")
        self.assertEqual((actor.phase, actor.facing, actor.body_facing, actor.reversing), ("reverse", "S", "N", True))
        self.assertEqual(sensor_channel(actor, (5, 6), self.track), "left")
        self.assertEqual(actor.velocity(self.track), (0, 0))
        actor.advance(self.track, .5)
        actor.finish(self.track)
        self.assertEqual(actor.phase, "move")
        self.assertGreater(actor.velocity(self.track)[1], 0)
        self.assertEqual(sensor_channel(actor, (5, 6), self.track), "left")

    def test_switching_back_to_forward_does_not_rotate_chassis(self):
        game = Game(self.track, dict(self.config, turn90_s=.5, uturn_s=.5))
        actor = Robot("B", "(6,6)", "S", .3, body_facing="N")
        actor.command(self.track, game.routing, "(6,5)", "evade")
        self.assertEqual(actor.body_facing, "N")
        self.assertFalse(actor.reversing)
        self.assertEqual(actor.phase, "reverse")


if __name__ == "__main__":
    unittest.main()
