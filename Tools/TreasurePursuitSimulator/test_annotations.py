"""Localization export must preserve original geometry and stable identities."""
import copy
import heapq
import unittest

from map_annotations import build_annotations, landmark_type
from simulate import TrackMap


class AnnotationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.track = TrackMap()
        cls.data = build_annotations(cls.track)

    def test_export_is_deterministic_and_does_not_mutate_track(self):
        graph, xy, source = copy.deepcopy(self.track.graph), copy.deepcopy(self.track.xy), copy.deepcopy(self.track.data)
        shuffled = copy.deepcopy(self.track)
        shuffled.graph = {n: dict(reversed(list(edges.items()))) for n, edges in reversed(list(graph.items()))}
        shuffled.xy = dict(reversed(list(xy.items())))
        self.assertEqual(build_annotations(shuffled), self.data)
        self.assertEqual((self.track.graph, self.track.xy, self.track.data), (graph, xy, source))

    def test_registry_preserves_ids_and_reserves_retired_numbers(self):
        prior = copy.deepcopy(self.data)
        prior['identity_registry']['nodes']['retired'] = 'N999'
        prior['identity_registry']['segments']['retired'] = 'S999'
        regenerated = build_annotations(self.track, prior)
        self.assertEqual(regenerated['nodes'], self.data['nodes'])
        self.assertEqual(regenerated['segments'], self.data['segments'])
        self.assertEqual(regenerated['identity_registry']['nodes']['retired'], 'N999')

    def test_every_landmark_is_labelled_without_inventing_straight_junctions(self):
        labelled = {n['original_node'] for n in self.data['nodes']}
        for name in self.track.graph:
            expected = landmark_type(self.track, name) is not None or name in self.track.special.values()
            self.assertEqual(name in labelled, expected)
        treasure = next(n for n in self.data['nodes'] if n['type'] == 'treasure')
        self.assertFalse(treasure['is_track_landmark'])
        self.assertEqual(treasure['geometry_type'], 'straight_special')
        self.assertIsNone(self.data['original_node_mapping']['(4,3)']['node_id'])

    def test_segments_cover_each_original_edge_exactly_once(self):
        expected = {frozenset((a, b)) for a in self.track.graph for b in self.track.graph[a]}
        actual = []
        for s in self.data['segments']:
            path = s['original_nodes']
            actual.extend(frozenset((a, b)) for a, b in zip(path, path[1:]))
            length = sum(self.track.graph[a][b] * self.track.spacing / 2 for a, b in zip(path, path[1:]))
            self.assertAlmostEqual(s['length_m'], length)
            self.assertFalse(s['calibrated'])
            self.assertEqual(s['original_offsets'][0]['distance_from_start_m'], 0)
            self.assertAlmostEqual(s['original_offsets'][-1]['distance_from_start_m'], length)
            points = [self.track.xy[n] for n in path]
            self.assertTrue(len({p[0] for p in points}) == 1 or len({p[1] for p in points}) == 1)
        self.assertEqual(set(actual), expected)
        self.assertEqual(len(actual), len(expected))

    def test_reciprocal_connections_and_all_original_node_offsets(self):
        nodes = {n['id']: n for n in self.data['nodes']}
        for s in self.data['segments']:
            for start, end, direction in ((s['start'], s['end'], s['direction_from_start']),
                                           (s['end'], s['start'], s['direction_from_end'])):
                c = next(c for c in nodes[start]['connections'] if c['segment'] == s['id'])
                self.assertEqual((c['neighbor'], c['direction'], c['length_m']), (end, direction, s['length_m']))
        self.assertEqual(set(self.data['original_node_mapping']), set(self.track.graph))
        for original, ref in self.data['original_node_mapping'].items():
            self.assertTrue(ref['segments'])
            for mapping in ref['segments']:
                segment = next(s for s in self.data['segments'] if s['id'] == mapping['segment_id'])
                self.assertIn(original, segment['original_nodes'])
                self.assertAlmostEqual(mapping['distance_from_start_m'] + mapping['distance_from_end_m'], segment['length_m'])

    def test_compressed_graph_preserves_landmark_shortest_distances(self):
        graph = {n['id']: {c['neighbor']: c['length_m'] for c in n['connections']} for n in self.data['nodes']}
        def distances(g, source):
            best, queue = {source: 0}, [(0, source)]
            while queue:
                cost, node = heapq.heappop(queue)
                if cost != best[node]: continue
                for neighbor, length in g[node].items():
                    candidate = cost + length
                    if candidate < best.get(neighbor, float('inf')):
                        best[neighbor] = candidate
                        heapq.heappush(queue, (candidate, neighbor))
            return best
        for root in self.data['nodes']:
            compressed = distances(graph, root['id'])
            original = distances(self.track.graph, root['original_node'])
            for target in self.data['nodes']:
                self.assertAlmostEqual(compressed[target['id']], original[target['original_node']] * self.track.spacing / 2)


if __name__ == '__main__':
    unittest.main()
