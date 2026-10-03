"""Police patrol commitments and sensing coverage, without hidden opponent truth."""
import json
import unittest
from simulate import Game, ROOT, TrackMap


class PatrolTests(unittest.TestCase):
    def setUp(self):
        self.track = TrackMap()
        config = json.loads((ROOT / 'config.json').read_text(encoding='utf-8'))
        self.game = Game(self.track, dict(config, miss_probability=0))
        self.game.now = 30
        self.game.owner, self.game.police_checked_treasure = 'B', True
        self.actor, self.belief = self.game.robots['P'], self.game.beliefs['P']
        self.actor.node, self.actor.facing = '(6,0)', 'S'

    def test_central_corridor_selected_and_completed_despite_negative_coverage(self):
        g = self.game
        self.assertEqual(g.decide('P'), ('(6,1)', 'search', '(6,9)'))
        self.assertEqual(g.search_sweeps[-1]['kind'], 'straight_sweep')
        # Even newly covered sites and an expired old point-goal timer do not cancel it.
        g.search_covered = set(g.search_sites)
        g.now = 100
        for y in range(1, 9):
            self.actor.node = f'(6,{y})'
            self.assertEqual(g.decide('P'), (f'(6,{y+1})', 'search', '(6,9)'))
        self.actor.node = '(6,9)'
        g.decide('P')
        self.assertTrue(any(e['event'] == 'patrol_complete' and e['node'] == '(6,9)' for e in g.search_sweeps))

    def test_positive_detection_preempts_then_expired_trail_resumes_saved_route(self):
        g = self.game
        g.decide('P')
        pending = list(g.search_route)
        self.belief.last_seen, self.belief.last_xy = '(8,0)', (8, 0)
        self.belief.last_time, self.belief.last_heading, self.belief.seen = 30, 'E', True
        self.assertEqual(g.decide('P')[1], 'chase')
        self.assertEqual(g.search_route, pending)
        self.actor.node, self.belief.seen, g.now = '(8,1)', False, 50
        self.assertEqual(g.decide('P')[1:], ('search', '(6,9)'))
        self.assertEqual(g.search_route, pending)

    def test_lost_target_checks_straight_end_then_resumes_not_endless_trail(self):
        g = self.game
        g.decide('P')
        self.actor.node = '(6,1)'
        g.police_trail = {'node':'(6,1)', 'xy':(6,1), 'last_time':27,
                          'direction':'S', 'reached':True, 'extended':False}
        self.assertEqual(g.decide('P'), ('(6,2)', 'trail', '(6,9)'))
        self.actor.node = '(6,2)'
        self.assertEqual(g.decide('P'), ('(6,3)', 'trail', '(6,9)'))
        self.actor.node = '(6,9)'
        self.assertEqual(g.decide('P')[1], 'search')

    def test_passage_without_consecutive_samples_does_not_mark_coverage(self):
        g = self.game
        self.actor.node = '(6,3)'
        g.update_search_coverage()
        self.assertEqual(g.negative_counts['(6,3)'], 1)
        self.actor.node = '(0,9)'
        g.update_search_coverage()
        self.assertEqual(g.negative_counts['(6,3)'], 0)
        self.actor.node = '(6,3)'
        for _ in range(2):
            g.update_search_coverage()
        self.assertNotIn('(6,3)', g.search_covered)
        g.update_search_coverage()
        self.assertIn('(6,3)', g.search_covered)
        self.belief.seen = True
        g.update_search_coverage()
        self.assertFalse(g.negative_counts)
        self.assertFalse(g.arm_clear_counts)

    def test_central_route_has_no_hidden_opponent_input(self):
        g = self.game
        first = g.decide('P')
        g.search_route, g.search_route_target = [], None
        g.plans.clear()
        g.robots['B'].node = '(0,9)'
        self.assertEqual(g.decide('P'), first)


if __name__ == '__main__':
    unittest.main()
