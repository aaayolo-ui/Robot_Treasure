"""Offline checks for graph connectivity, shortest paths and motion costs."""
import math
import unittest
from collections import deque

from planner import TrackMap


class PlannerTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.track = TrackMap()

    def bfs_half_cells(self, start, target):
        # Independent unweighted BFS after splitting every full-cell edge.
        graph = {}
        for u, adjacent in self.track.graph.items():
            graph.setdefault(u, [])
            for v, ticks in adjacent.items():
                if u >= v:
                    continue
                chain = [u] + [f"mid:{u}:{v}:{i}" for i in range(1, ticks)] + [v]
                for a, b in zip(chain, chain[1:]):
                    graph.setdefault(a, []).append(b)
                    graph.setdefault(b, []).append(a)
        start, target = self.track.resolve(start), self.track.resolve(target)
        queue, seen = deque([(start, 0)]), {start}
        while queue:
            u, d = queue.popleft()
            if u == target:
                return d * self.track.spacing / 2
            for v in graph[u]:
                if v not in seen:
                    seen.add(v)
                    queue.append((v, d + 1))
        self.fail("Disconnected map")

    def test_distance_optimum_against_independent_bfs(self):
        for start, meters, turns in (("P", 5.25, 7), ("B", 3.15, 4)):
            route = self.track.dijkstra(start, "T")
            self.assertAlmostEqual(route["distance_m"], self.bfs_half_cells(start, "T"))
            self.assertAlmostEqual(route["distance_m"], meters)
            self.assertEqual(route["quarter_turns"], turns)
            self.assertEqual(route["uturns"], 0)

    def test_physical_line_gaps_and_stop_caps(self):
        # The drawing's stop bars do not connect to nearby roads.
        absent = [((0, 6), (0, 7)), ((8, 0), (9, 0)),
                  ((11, 4), (11, 5)), ((11, 5), (11, 6)),
                  ((11, 6), (11, 7)), ((8, 9), (9, 9))]
        for a, b in absent:
            u, v = self.track.resolve(f"({a[0]},{a[1]})"), self.track.resolve(f"({b[0]},{b[1]})")
            self.assertNotIn(v, self.track.graph[u])
        self.assertEqual(self.track.structure()["connected_components"], 1)
        self.assertEqual(len(self.track.graph[self.track.resolve("B")]), 1)

    def test_slower_turn_selects_longer_route(self):
        # Analytical crossover: 5.25/v+7*t == 5.85/v+5*t.
        threshold = 0.6 / (2 * 0.35)
        short = self.track.dijkstra("P", "T", "time", .35, threshold - .001, 2)
        long = self.track.dijkstra("P", "T", "time", .35, threshold + .001, 2)
        self.assertEqual((short["distance_m"], short["quarter_turns"]), (5.25, 7))
        self.assertEqual((long["distance_m"], long["quarter_turns"]), (5.85, 5))

    def test_all_equal_distance_alternatives(self):
        self.assertEqual(self.track.shortest_path_count("P", "T"), 1)
        self.assertEqual(self.track.shortest_path_count("B", "T"), 3)
        paths = self.track.shortest_alternatives("B", "T")
        self.assertEqual(len(paths), 3)
        self.assertEqual(sorted(r["quarter_turns"] for r in paths), [4, 4, 6])
        self.assertTrue(all(r["distance_m"] == 3.15 for r in paths))

    def test_replanning_around_blocked_corridor(self):
        route = self.track.dijkstra("P", "T", blocked_nodes=["(6,3)"])
        self.assertNotIn("(6,3)", route["nodes"])
        self.assertEqual(route["distance_m"], 5.55)
        with self.assertRaisesRegex(ValueError, "No reachable"):
            self.track.dijkstra("P", "T", blocked_nodes=["(0,7)"])

    def test_validated_cycles_and_half_treasure_edge(self):
        report = self.track.export()
        expected = {"upper_small": 1.2, "central": 3.6, "left_large": 9.0, "right_large": 5.4}
        for key, meters in expected.items():
            route = report["cycles"][key]
            self.assertEqual(route["nodes"][0], route["nodes"][-1])
            self.assertEqual(len(route["nodes"]) - 1, len(set(route["nodes"][:-1])))
            self.assertEqual(route["distance_m"], meters)
            self.assertTrue(all(b in self.track.graph[a] for a, b in zip(route["nodes"], route["nodes"][1:])))
        self.assertEqual(set(self.track.graph[self.track.resolve("T")].values()), {1})

    def test_model_seconds_include_turns(self):
        route = self.track.dijkstra("P", "T", "time", .35, .6, 1.2)
        self.assertAlmostEqual(route["estimated_seconds"], 5.25 / .35 + 7 * .6)
        wrong_facing = self.track.dijkstra("P", "T", initial_heading="S")
        self.assertEqual(wrong_facing["uturns"], 1)

    def test_invalid_or_blocked_input(self):
        for speed in [0, -1, math.nan, math.inf]:
            with self.assertRaises(ValueError):
                self.track.dijkstra("P", "T", speed=speed)
        with self.assertRaises(ValueError):
            self.track.dijkstra("P", "T", blocked_nodes=["T"])


if __name__ == "__main__":
    unittest.main(verbosity=2)
